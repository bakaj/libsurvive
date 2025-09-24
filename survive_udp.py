#!/usr/bin/env python3
"""
Async UDP Telemetry Receiver for libsurvive

An async UDP receiver for real-time telemetry data from libsurvive tracking systems.
Provides structured access to pose, IMU, lighthouse, and button data.

Features:
- Async/await based UDP message processing
- Real-time data processing with callbacks
- Structured data storage with automatic parsing
- Statistics and monitoring
- Cross-platform compatibility (Linux, macOS, Windows)

Data Types Supported:
- POSE: 6DOF position and orientation (x, y, z, qx, qy, qz, qw)
- IMU: Accelerometer and gyroscope data (ax, ay, az, gx, gy, gz)
- VELOCITY: Linear velocity in 3D space (vx, vy, vz)
- LIGHTHOUSE: Base station positions and orientations
- BUTTON: Button press/release events
- ANGLE: Lighthouse angle measurements
- LIGHT: Photodiode light intensity readings
- CONFIG: Configuration parameter updates

Usage:
    receiver = UDPTelemetryReceiver(port=2333)
    await receiver.start()
    
    # Add real-time callback
    async def on_message(device, data_type, values, timestamp):
        if data_type == 'POSE':
            print(f"Device {device} at position {values[:3]}")
    
    receiver.add_callback(on_message)
    
    # Access structured data
    for device_name in receiver.get_active_devices():
        device = receiver.get_device(device_name)
        pose = device.get_latest_pose()
        if pose:
            print(f"Position: {pose[:3]}, Orientation: {pose[3:7]}")
"""

import asyncio
import socket
import time
import threading
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any, Callable
from collections import defaultdict, deque
import argparse


@dataclass
class TelemetryData:
    """
    Structured telemetry data container for a single tracking device.
    Based on the RecordedData class from pysurvive/recorder.py
    
    This class stores all telemetry data received from a single libsurvive device
    (HMD, controller, tracker, etc.) in organized, timestamped collections.
    """
    device_name: str
    datalog_whitelist: Optional[List[str]] = None
    
    # IMU data (matching RecordedData structure)
    imu_times: List[float] = field(default_factory=list)
    gyros: List[List[float]] = field(default_factory=list)  # gyroscope data [gx, gy, gz]
    accels: List[List[float]] = field(default_factory=list)  # accelerometer data [ax, ay, az]
    raw_imu_times: List[float] = field(default_factory=list)
    raw_gyros: List[List[float]] = field(default_factory=list)
    raw_accels: List[List[float]] = field(default_factory=list)
    
    # Pose and velocity data
    poses: List[Tuple[float, List[float]]] = field(default_factory=list)  # [(timestamp, [x, y, z, qx, qy, qz, qw])]
    velocities: List[Tuple[float, List[float]]] = field(default_factory=list)  # [(timestamp, [vx, vy, vz])]
    
    # Lighthouse tracking data (matching RecordedData structure)
    angles: Dict[Tuple[int, int, int], List[Tuple[float, float]]] = field(default_factory=lambda: defaultdict(list))  # {(sensor_id, lighthouse_id, axis): [(timestamp, angle)]}
    lengths: Dict[Tuple[int, int, int], List[Tuple[float, float]]] = field(default_factory=lambda: defaultdict(list))  # {(sensor_id, lighthouse_id, axis): [(timestamp, length)]}
    angle_per_sweep: Dict[Tuple[int, int], List[Tuple[float, List[float]]]] = field(default_factory=lambda: defaultdict(list))  # {(lighthouse_id, axis): [(timestamp, [angles])]}
    raw_angles: Dict[Tuple[int, int], List[Tuple[float, float]]] = field(default_factory=lambda: defaultdict(list))  # {(sensor_id, lighthouse_id): [(timestamp, angle)]}
    
    # Light data
    light_data: Dict[Tuple[int, int, int], List[Tuple[float, float]]] = field(default_factory=lambda: defaultdict(list))  # {(sensor_id, lighthouse_id, axis): [(timestamp, intensity)]}
    
    # Button and config data
    button_events: List[Tuple[float, int, int]] = field(default_factory=list)  # [(timestamp, button_id, state)]
    config_data: List[Tuple[float, str, str]] = field(default_factory=list)  # [(timestamp, key, value)]
    
    # Lighthouse data
    lighthouse_data: Dict[int, List[Tuple[float, List[float]]]] = field(default_factory=lambda: defaultdict(list))  # {lighthouse_id: [(timestamp, [x, y, z, qx, qy, qz, qw])]}
    
    # Movement tracking
    time_since_move: List[float] = field(default_factory=list)
    
    # Data logs (matching RecordedData structure)
    datalogs: Dict[str, List[Tuple[float, List[float]]]] = field(default_factory=lambda: defaultdict(list))
    
    # Metadata
    last_update: float = 0.0
    
    def get_latest_pose(self) -> Optional[List[float]]:
        """
        Get the most recent 6DOF pose data for this device.
        
        Returns:
            Optional[List[float]]: Latest pose as [x, y, z, qx, qy, qz, qw] or None if no pose data
        """
        return self.poses[-1][1] if self.poses else None
    
    def get_latest_velocity(self) -> Optional[List[float]]:
        """
        Get the most recent velocity data for this device.
        
        Returns:
            Optional[List[float]]: Latest velocity as [vx, vy, vz] in m/s or None if no velocity data
        """
        return self.velocities[-1][1] if self.velocities else None
    
    def get_latest_imu(self) -> Optional[List[float]]:
        """
        Get the most recent IMU data for this device.
        
        Returns:
            Optional[List[float]]: Latest IMU as [ax, ay, az, gx, gy, gz] or None if no IMU data
        """
        if self.imu_times and self.accels and self.gyros:
            return self.accels[-1] + self.gyros[-1]
        return None
    
    def is_moving(self, threshold: float = 0.1) -> bool:
        """
        Check if the device is currently moving based on recent velocity data.
        
        Args:
            threshold (float): Velocity threshold in m/s. Device is considered moving
                             if average velocity magnitude exceeds this value.
        
        Returns:
            bool: True if device is moving, False otherwise
        """
        if not self.velocities:
            return False
        
        recent_velocities = self.velocities[-10:]  # Last 10 velocity readings
        if len(recent_velocities) < 2:
            return False
        
        # Calculate average velocity magnitude
        total_velocity = 0.0
        for _, velocity in recent_velocities:
            if len(velocity) >= 3:
                magnitude = sum(v*v for v in velocity[:3]) ** 0.5
                total_velocity += magnitude
        
        avg_velocity = total_velocity / len(recent_velocities)
        return avg_velocity > threshold


class BaseUDPReceiver:
    """
    Base class for UDP telemetry receivers with shared functionality.
    """
    
    def __init__(self, port: int = 2333, buffer_size: int = 8192):
        """
        Initialize the base UDP receiver.
        
        Args:
            port (int): UDP port to listen on (default: 2333)
            buffer_size (int): UDP socket buffer size in bytes (default: 8192)
        """
        self.port = port
        self.buffer_size = buffer_size
        self.socket = None
        self.running = False
        self.devices: Dict[str, TelemetryData] = {}
        self.callbacks: List[Callable] = []
        
        # Statistics
        self.stats = {
            'messages_received': 0,
            'parse_errors': 0,
            'start_time': 0.0,
            'last_message_time': 0.0,
            'processing_times': deque(maxlen=1000)
        }
    
    def add_callback(self, callback: Callable) -> None:
        """
        Add a callback function to be called for each received message.
        
        Args:
            callback (Callable): Function to call with signature:
                callback(device_name: str, data_type: str, data_values: List[str], timestamp: float)
        """
        self.callbacks.append(callback)
    
    def _setup_socket(self) -> None:
        """
        Setup the UDP socket with common configuration.
        """
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # Try to set socket buffer size (may fail on some systems)
        try:
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.buffer_size)
        except OSError:
            pass  # Ignore if not supported
        
        # Try to set UDP-specific options (may fail on some systems)
        try:
            self.socket.setsockopt(socket.IPPROTO_UDP, socket.SO_RCVBUF, self.buffer_size)
        except OSError:
            pass  # Ignore if not supported (e.g., macOS)
        
        self.socket.bind(('0.0.0.0', self.port))
    
    def _process_message(self, message: str, addr: Tuple[str, int]) -> None:
        """
        Process a single UDP message and update device data.
        
        Args:
            message (str): Raw message string from UDP
            addr (Tuple[str, int]): Source address and port
        """
        start_time = time.time()
        
        try:
            # Parse the message
            parts = message.split()
            if len(parts) < 3:
                self.stats['parse_errors'] += 1
                return
            
            timestamp = float(parts[0])
            device_name = parts[1]
            data_type = parts[2]
            data_values = parts[3:] if len(parts) > 3 else []
            
            # Get or create device
            if device_name not in self.devices:
                self.devices[device_name] = TelemetryData(device_name=device_name)
            
            device = self.devices[device_name]
            device.last_update = timestamp
            
            # Process the data type
            self._process_data_type(device, data_type, data_values, timestamp)
            
            # Update statistics
            self.stats['messages_received'] += 1
            self.stats['last_message_time'] = timestamp
            
            # Call callbacks
            self._call_callbacks(device_name, data_type, data_values, timestamp)
            
        except Exception as e:
            self.stats['parse_errors'] += 1
            print(f"Error processing message '{message}': {e}")
        finally:
            # Update processing time statistics
            processing_time = (time.time() - start_time) * 1000  # Convert to ms
            self._update_performance_stats(processing_time)
    
    def _call_callbacks(self, device_name: str, data_type: str, data_values: List[str], timestamp: float) -> None:
        """
        Call all registered callbacks. Override in subclasses for async handling.
        
        Args:
            device_name (str): Name of the device
            data_type (str): Type of data received
            data_values (List[str]): Data values
            timestamp (float): Message timestamp
        """
        for callback in self.callbacks:
            try:
                callback(device_name, data_type, data_values, timestamp)
            except Exception as e:
                print(f"Error in callback: {e}")
    
    def _process_data_type(self, device: TelemetryData, data_type: str, 
                          data_values: List[str], timestamp: float) -> None:
        """
        Process a specific data type and update device data structures.
        """
        try:
            if data_type == 'POSE':
                # POSE: x y z qx qy qz qw
                if len(data_values) >= 7:
                    pose_data = [float(x) for x in data_values[:7]]
                    device.poses.append((timestamp, pose_data))
            
            elif data_type == 'VELOCITY':
                # VELOCITY: vx vy vz wx wy wz
                if len(data_values) >= 6:
                    velocity_data = [float(x) for x in data_values[:6]]
                    device.velocities.append((timestamp, velocity_data))
            
            elif data_type == 'I':  # Calibrated IMU
                # I: mask timecode ax ay az gx gy gz mx my mz id
                if len(data_values) >= 11:
                    mask = int(data_values[0])
                    timecode = int(data_values[1])
                    accelgyro = [float(x) for x in data_values[2:11]]
                    device_id = int(float(data_values[10])) if len(data_values) > 10 else 0
                    
                    # Store IMU data in RecordedData format
                    device.imu_times.append(timestamp)
                    device.accels.append(accelgyro[:3])  # ax, ay, az
                    device.gyros.append(accelgyro[3:6])  # gx, gy, gz
            
            elif data_type == 'i':  # Raw IMU
                # i: mask timecode ax ay az gx gy gz mx my mz id
                if len(data_values) >= 11:
                    mask = int(data_values[0])
                    timecode = int(data_values[1])
                    accelgyro = [float(x) for x in data_values[2:11]]
                    device_id = int(float(data_values[10])) if len(data_values) > 10 else 0
                    
                    # Store raw IMU data
                    device.raw_imu_times.append(timestamp)
                    device.raw_accels.append(accelgyro[:3])  # ax, ay, az
                    device.raw_gyros.append(accelgyro[3:6])  # gx, gy, gz
            
            elif data_type == 'LH_POSE':
                # LH_POSE: lighthouse_id x y z qx qy qz qw basestation_id
                if len(data_values) >= 8:
                    lighthouse_id = int(float(data_values[0]))  # Handle float lighthouse_id
                    pose_data = [float(x) for x in data_values[1:8]]
                    device.lighthouse_data[lighthouse_id].append((timestamp, pose_data))
            
            elif data_type == 'A':  # Angle data
                # A: sensor_id acode timecode length angle lighthouse_id
                if len(data_values) >= 6:
                    sensor_id = int(float(data_values[0]))  # Handle float sensor_id
                    acode = int(float(data_values[1]))  # Handle float acode
                    timecode = int(float(data_values[2]))  # Handle float timecode
                    length = float(data_values[3])
                    angle = float(data_values[4])
                    lighthouse_id = int(float(data_values[5]))  # Handle float lighthouse_id
                    
                    # Store angle data
                    key = (sensor_id, lighthouse_id, acode & 1)  # axis is acode & 1
                    device.angles[key].append((timestamp, angle))
                    device.lengths[key].append((timestamp, length))
            
            elif data_type == 'B':  # Sweep angle
                # B: channel sensor_id timecode plane angle
                if len(data_values) >= 5:
                    channel = int(float(data_values[0]))  # Handle float channel
                    sensor_id = int(float(data_values[1]))  # Handle float sensor_id
                    timecode = int(float(data_values[2]))  # Handle float timecode
                    plane = int(float(data_values[3]))  # Handle float plane
                    angle = float(data_values[4])
                    
                    # Store sweep angle data
                    key = (sensor_id, channel, plane)
                    device.angles[key].append((timestamp, angle))
            
            elif data_type == 'W':  # Sweep data
                # W: channel sensor_id timecode flag
                if len(data_values) >= 4:
                    channel = int(float(data_values[0]))  # Handle float channel
                    sensor_id = int(float(data_values[1]))  # Handle float sensor_id
                    timecode = int(float(data_values[2]))  # Handle float timecode
                    flag = int(float(data_values[3]))  # Handle float flag
                    
                    # Initialize sweep data
                    key = (channel, 0)  # horizontal axis
                    device.angle_per_sweep[key].append((timestamp, []))
                    key = (channel, 1)  # vertical axis
                    device.angle_per_sweep[key].append((timestamp, []))
            
            elif data_type == 'Y':  # Sync data
                # Y: channel timecode ootx gen
                if len(data_values) >= 4:
                    channel = int(float(data_values[0]))  # Handle float channel
                    timecode = int(float(data_values[1]))  # Handle float timecode
                    ootx = int(float(data_values[2]))  # Handle float ootx
                    gen = int(float(data_values[3]))  # Handle float gen
                    
                    # Initialize sync data
                    key = (channel, 0)  # horizontal axis
                    device.angle_per_sweep[key].append((timestamp, []))
                    key = (channel, 1)  # vertical axis
                    device.angle_per_sweep[key].append((timestamp, []))
            
            elif data_type == 'C':  # Light intensity
                # C: sensor_id timecode length
                if len(data_values) >= 3:
                    sensor_id = int(float(data_values[0]))  # Handle float sensor_id
                    timecode = int(float(data_values[1]))  # Handle float timecode
                    length = float(data_values[2])
                    
                    # Store light data (assuming lighthouse 0, axis 0)
                    key = (sensor_id, 0, 0)
                    device.light_data[key].append((timestamp, length))
            
            elif data_type == 'BUTTON':
                # BUTTON: button_id state
                if len(data_values) >= 2:
                    button_id = int(float(data_values[0]))  # Handle float button_id
                    state = int(float(data_values[1]))  # Handle float state
                    device.button_events.append((timestamp, button_id, state))
            
            elif data_type == 'CONFIG':
                # CONFIG: key value
                if len(data_values) >= 2:
                    key = data_values[0]
                    value = ' '.join(data_values[1:])
                    device.config_data.append((timestamp, key, value))
            
            elif data_type == 'DISCONNECT':
                # DISCONNECT: device_name
                if len(data_values) >= 1:
                    device_name = data_values[0]
                    # Mark device as disconnected (could add disconnect tracking)
                    pass
            
            elif data_type == 'EXTERNAL_POSE':
                # EXTERNAL_POSE: name x y z qx qy qz qw
                if len(data_values) >= 8:
                    name = data_values[0]
                    pose_data = [float(x) for x in data_values[1:8]]
                    # Store as external pose (could add external pose tracking)
                    pass
            
            elif data_type == 'EXTERNAL_VELOCITY':
                # EXTERNAL_VELOCITY: name vx vy vz wx wy wz
                if len(data_values) >= 7:
                    name = data_values[0]
                    velocity_data = [float(x) for x in data_values[1:7]]
                    # Store as external velocity (could add external velocity tracking)
                    pass
            
            elif data_type == 'INFO':
                # INFO LOG: message
                if len(data_values) >= 2 and data_values[0] == 'LOG':
                    message = ' '.join(data_values[1:])
                    # Store as info log (could add info log tracking)
                    pass
            
            elif data_type == 'S':  # Light data (from light_process)
                # S: sensor_id acode timeinsweep timecode length lighthouse_id
                if len(data_values) >= 6:
                    sensor_id = int(float(data_values[0]))
                    acode = int(float(data_values[1]))
                    timeinsweep = int(float(data_values[2]))
                    timecode = int(float(data_values[3]))
                    length = int(float(data_values[4]))
                    lighthouse_id = int(float(data_values[5]))
                    
                    # Store light data
                    key = (sensor_id, lighthouse_id, acode & 1)
                    device.light_data[key].append((timestamp, length))
            
            elif data_type == 'L':  # Light data (from light_process)
                # L: sensor_id acode timeinsweep timecode length lighthouse_id
                if len(data_values) >= 6:
                    sensor_id = int(float(data_values[0]))
                    acode = int(float(data_values[1]))
                    timeinsweep = int(float(data_values[2]))
                    timecode = int(float(data_values[3]))
                    length = int(float(data_values[4]))
                    lighthouse_id = int(float(data_values[5]))
                    
                    # Store light data
                    key = (sensor_id, lighthouse_id, acode & 1)
                    device.light_data[key].append((timestamp, length))
            
            elif data_type == 'R':  # Light data (from light_process)
                # R: sensor_id acode timeinsweep timecode length lighthouse_id
                if len(data_values) >= 6:
                    sensor_id = int(float(data_values[0]))
                    acode = int(float(data_values[1]))
                    timeinsweep = int(float(data_values[2]))
                    timecode = int(float(data_values[3]))
                    length = int(float(data_values[4]))
                    lighthouse_id = int(float(data_values[5]))
                    
                    # Store light data
                    key = (sensor_id, lighthouse_id, acode & 1)
                    device.light_data[key].append((timestamp, length))
            
            # Handle other data types as needed
            else:
                # Store unknown data types in datalogs
                # Try to convert numeric values, keep strings as-is
                converted_values = []
                for x in data_values:
                    try:
                        # Try to convert to float first, then to int if it's a whole number
                        float_val = float(x)
                        if float_val.is_integer():
                            converted_values.append(int(float_val))
                        else:
                            converted_values.append(float_val)
                    except ValueError:
                        # Keep as string if conversion fails
                        converted_values.append(x)
                
                device.datalogs[data_type].append((timestamp, converted_values))
        
        except (ValueError, IndexError) as e:
            print(f"Error parsing {data_type} data: {e}")
            self.stats['parse_errors'] += 1
    
    def _update_performance_stats(self, processing_time: float) -> None:
        """
        Update performance statistics with processing time.
        
        Args:
            processing_time (float): Processing time in milliseconds
        """
        self.stats['processing_times'].append(processing_time)
    
    def get_active_devices(self) -> List[str]:
        """
        Get list of device names that have received data recently.
        
        Returns:
            List[str]: List of active device names
        """
        current_time = time.time()
        active_devices = []
        
        for device_name, device in self.devices.items():
            # Consider device active if it has received data in the last 5 seconds
            if current_time - device.last_update < 5.0:
                active_devices.append(device_name)
        
        return active_devices
    
    def get_device(self, device_name: str) -> Optional[TelemetryData]:
        """
        Get telemetry data for a specific device.
        
        Args:
            device_name (str): Name of the device
            
        Returns:
            Optional[TelemetryData]: Device data or None if not found
        """
        return self.devices.get(device_name)
    
    def get_all_devices(self) -> Dict[str, TelemetryData]:
        """
        Get all device data.
        
        Returns:
            Dict[str, TelemetryData]: All device data
        """
        return self.devices.copy()
    
    def get_stats(self) -> Dict[str, Any]:
        """
        Get receiver statistics.
        
        Returns:
            Dict[str, Any]: Statistics including message counts, errors, and timing
        """
        current_time = time.time()
        runtime = current_time - self.stats['start_time'] if self.stats['start_time'] > 0 else 0
        
        avg_processing_time = 0.0
        if self.stats['processing_times']:
            avg_processing_time = sum(self.stats['processing_times']) / len(self.stats['processing_times'])
        
        return {
            'messages_received': self.stats['messages_received'],
            'parse_errors': self.stats['parse_errors'],
            'active_devices': len(self.get_active_devices()),
            'runtime_seconds': runtime,
            'messages_per_second': self.stats['messages_received'] / runtime if runtime > 0 else 0.0,
            'average_processing_time_ms': avg_processing_time,
            'last_message_time': self.stats['last_message_time']
        }
    
    def export_data(self) -> Dict[str, Any]:
        """
        Export all collected data for analysis or storage.
        
        Returns:
            Dict[str, Any]: Complete data export including all devices and statistics
        """
        return {
            'devices': {
                name: {
                    'device_name': device.device_name,
                    'poses': device.poses,
                    'velocities': device.velocities,
                    'imu_times': device.imu_times,
                    'gyros': device.gyros,
                    'accels': device.accels,
                    'raw_imu_times': device.raw_imu_times,
                    'raw_gyros': device.raw_gyros,
                    'raw_accels': device.raw_accels,
                    'angles': dict(device.angles),
                    'lengths': dict(device.lengths),
                    'light_data': dict(device.light_data),
                    'button_events': device.button_events,
                    'lighthouse_data': {k: v for k, v in device.lighthouse_data.items()},
                    'config_data': device.config_data,
                    'datalogs': {k: v for k, v in device.datalogs.items()},
                    'last_update': device.last_update
                }
                for name, device in self.devices.items()
            },
            'statistics': self.get_stats(),
            'export_timestamp': time.time()
        }
    
    def get_latest_data(self) -> Dict[str, Any]:
        """
        Get the latest data from all active devices in a convenient format.
        
        Returns:
            Dict[str, Any]: Latest data from all devices
        """
        latest_data = {}
        
        for device_name in self.get_active_devices():
            device = self.get_device(device_name)
            if device:
                latest_data[device_name] = {
                    'pose': device.get_latest_pose(),
                    'velocity': device.get_latest_velocity(),
                    'imu': device.get_latest_imu(),
                    'is_moving': device.is_moving(),
                    'last_update': device.last_update,
                    'pose_count': len(device.poses),
                    'imu_count': len(device.imu_times),
                    'raw_imu_count': len(device.raw_imu_times),
                    'angle_count': sum(len(angles) for angles in device.angles.values()),
                    'button_count': len(device.button_events)
                }
        
        return latest_data


class UDPTelemetryReceiver(BaseUDPReceiver):
    """
    Async UDP telemetry receiver for libsurvive tracking data.
    
    This class provides real-time reception and parsing of UDP telemetry messages
    from libsurvive tracking systems. It supports all standard libsurvive data
    types including poses, IMU data, lighthouse tracking, and button events.
    """
    
    async def start(self) -> None:
        """
        Start the UDP receiver and begin processing messages.
        
        Raises:
            OSError: If socket creation or binding fails
        """
        if self.running:
            return
        
        self._setup_socket()
        self.socket.setblocking(False)
        
        self.running = True
        self.stats['start_time'] = time.time()
        
        print(f"UDP Telemetry Receiver started on port {self.port}")
        
        # Start the receive loop
        await self._receive_loop()
    
    async def stop(self) -> None:
        """
        Stop the UDP receiver and close the socket.
        """
        self.running = False
        if self.socket:
            self.socket.close()
            self.socket = None
        print("UDP Telemetry Receiver stopped")
    
    async def _receive_loop(self) -> None:
        """
        Main receive loop for processing UDP messages.
        """
        loop = asyncio.get_event_loop()
        
        while self.running:
            try:
                # Wait for data with timeout
                data, addr = await asyncio.wait_for(
                    loop.sock_recvfrom(self.socket, self.buffer_size),
                    timeout=0.1
                )
                
                # Process the message
                message = data.decode('utf-8', errors='ignore').strip()
                if message:
                    self._process_message(message, addr)
                    
            except asyncio.TimeoutError:
                # Normal timeout, continue
                continue
            except Exception as e:
                print(f"Error receiving UDP data: {e}")
                await asyncio.sleep(0.1)
    
    async def _call_callbacks(self, device_name: str, data_type: str, data_values: List[str], timestamp: float) -> None:
        """
        Call all registered callbacks with async support.
        
        Args:
            device_name (str): Name of the device
            data_type (str): Type of data received
            data_values (List[str]): Data values
            timestamp (float): Message timestamp
        """
        for callback in self.callbacks:
            try:
                if asyncio.iscoroutinefunction(callback):
                    await callback(device_name, data_type, data_values, timestamp)
                else:
                    callback(device_name, data_type, data_values, timestamp)
            except Exception as e:
                print(f"Error in callback: {e}")


class JupyterUDPReceiver(BaseUDPReceiver):
    """
    Jupyter-compatible UDP telemetry receiver for libsurvive tracking data.
    
    This class provides a synchronous interface that works well in Jupyter notebooks
    by using threading instead of asyncio. It provides the same functionality as
    UDPTelemetryReceiver but is designed for interactive use in notebooks.
    """
    
    def __init__(self, port: int = 2333, buffer_size: int = 8192):
        """
        Initialize the Jupyter-compatible UDP receiver.
        
        Args:
            port (int): UDP port to listen on (default: 2333)
            buffer_size (int): UDP socket buffer size in bytes (default: 8192)
        """
        super().__init__(port, buffer_size)
        self.receive_thread = None
    
    def start(self) -> None:
        """
        Start the UDP receiver in a separate thread.
        
        Raises:
            OSError: If socket creation or binding fails
        """
        if self.running:
            return
        
        self._setup_socket()
        self.socket.settimeout(0.1)  # Non-blocking with timeout
        
        self.running = True
        self.stats['start_time'] = time.time()
        
        # Start receive thread
        self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.receive_thread.start()
        
        print(f"Jupyter UDP Telemetry Receiver started on port {self.port}")
    
    def stop(self) -> None:
        """
        Stop the UDP receiver and close the socket.
        """
        self.running = False
        if self.socket:
            self.socket.close()
            self.socket = None
        if self.receive_thread:
            self.receive_thread.join(timeout=1.0)
        print("Jupyter UDP Telemetry Receiver stopped")
    
    def _receive_loop(self) -> None:
        """
        Main receive loop for processing UDP messages (runs in separate thread).
        """
        while self.running:
            try:
                # Wait for data with timeout
                data, addr = self.socket.recvfrom(self.buffer_size)
                
                # Process the message
                message = data.decode('utf-8', errors='ignore').strip()
                if message:
                    self._process_message(message, addr)
                    
            except socket.timeout:
                # Normal timeout, continue
                continue
            except Exception as e:
                if self.running:  # Only print errors if we're supposed to be running
                    print(f"Error receiving UDP data: {e}")
                time.sleep(0.1)


async def main():
    """
    Main function for testing the UDP receiver.
    """
    parser = argparse.ArgumentParser(description='UDP Telemetry Receiver for libsurvive')
    parser.add_argument('--port', type=int, default=2333, help='UDP port to listen on')
    parser.add_argument('--buffer-size', type=int, default=8192, help='UDP buffer size')
    args = parser.parse_args()
    
    receiver = UDPTelemetryReceiver(port=args.port, buffer_size=args.buffer_size)
    
    # Add a simple callback to print received messages
    async def message_callback(device_name: str, data_type: str, data_values: List[str], timestamp: float):
        print(f"[{timestamp:.3f}] {device_name} {data_type}: {data_values}")
    
    receiver.add_callback(message_callback)
    
    try:
        print(f"Starting UDP receiver on port {args.port}...")
        await receiver.start()
    except KeyboardInterrupt:
        print("\nStopping receiver...")
        await receiver.stop()
        
        # Print final statistics
        stats = receiver.get_stats()
        print(f"\nFinal Statistics:")
        print(f"Messages received: {stats['messages_received']}")
        print(f"Parse errors: {stats['parse_errors']}")
        print(f"Active devices: {stats['active_devices']}")
        print(f"Messages per second: {stats['messages_per_second']:.1f}")
        print(f"Average processing time: {stats['average_processing_time_ms']:.2f}ms")


if __name__ == "__main__":
    asyncio.run(main())
