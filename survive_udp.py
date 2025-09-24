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
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any, Callable
from collections import defaultdict, deque
import argparse


@dataclass
class TelemetryData:
    """
    Structured telemetry data container for a single tracking device.
    
    This class stores all telemetry data received from a single libsurvive device
    (HMD, controller, tracker, etc.) in organized, timestamped collections.
    
    Attributes:
        device_name (str): Name/identifier of the device (e.g., "HMD", "Controller_L")
        
        poses (List[Tuple[float, List[float]]]): 6DOF pose data
            Format: [(timestamp, [x, y, z, qx, qy, qz, qw]), ...]
            - x, y, z: Position in meters (world coordinates)
            - qx, qy, qz, qw: Quaternion orientation (unit quaternion)
            - Timestamp: Unix timestamp when pose was calculated
            
        velocities (List[Tuple[float, List[float]]]): Linear velocity data
            Format: [(timestamp, [vx, vy, vz]), ...]
            - vx, vy, vz: Velocity in m/s (world coordinates)
            - Timestamp: Unix timestamp when velocity was calculated
            
        imu_data (List[Tuple[float, List[float]]]): Processed IMU data
            Format: [(timestamp, [ax, ay, az, gx, gy, gz]), ...]
            - ax, ay, az: Accelerometer data in m/s²
            - gx, gy, gz: Gyroscope data in rad/s
            - Timestamp: Unix timestamp when IMU data was captured
            
        raw_imu_data (List[Tuple[float, List[float]]]): Raw IMU sensor data
            Format: [(timestamp, [ax, ay, az, gx, gy, gz]), ...]
            - Raw sensor values before calibration/processing
            - Same format as imu_data but unprocessed
            
        angles (Dict[Tuple[int, int, int], List[Tuple[float, float]]]): Lighthouse angle data
            Format: {(sensor_id, lighthouse_id, axis): [(timestamp, angle), ...]}
            - sensor_id: Photodiode sensor ID on the device
            - lighthouse_id: Base station ID (0, 1, etc.)
            - axis: Sweep axis (0=horizontal, 1=vertical)
            - angle: Sweep angle in radians
            - Timestamp: When angle was measured
            
        light_data (Dict[Tuple[int, int, int], List[Tuple[float, float]]]): Light intensity data
            Format: {(sensor_id, lighthouse_id, axis): [(timestamp, intensity), ...]}
            - sensor_id: Photodiode sensor ID on the device
            - lighthouse_id: Base station ID (0, 1, etc.)
            - axis: Sweep axis (0=horizontal, 1=vertical)
            - intensity: Light intensity value (0.0 to 1.0)
            - Timestamp: When light was detected
            
        button_events (List[Tuple[float, int, int]]): Button press/release events
            Format: [(timestamp, button_id, state), ...]
            - button_id: Button identifier (0, 1, 2, etc.)
            - state: 0=pressed, 1=released
            - Timestamp: When button state changed
            
        lighthouse_data (Dict[int, List[Tuple[float, List[float]]]]): Base station data
            Format: {lighthouse_id: [(timestamp, [x, y, z, qx, qy, qz, qw]), ...]}
            - lighthouse_id: Base station ID (0, 1, etc.)
            - x, y, z: Base station position in meters
            - qx, qy, qz, qw: Base station orientation quaternion
            - Timestamp: When base station pose was calculated
            
        config_data (List[Tuple[float, str, str]]): Configuration updates
            Format: [(timestamp, key, value), ...]
            - key: Configuration parameter name
            - value: Configuration parameter value
            - Timestamp: When configuration was updated
            
        last_update (float): Unix timestamp of last received data
            Used to determine if device is still active/connected
    
    Coordinate System:
        - X-axis: Left/Right (positive = right)
        - Y-axis: Forward/Back (positive = forward)  
        - Z-axis: Up/Down (positive = up)
        - Origin: Usually at floor level in tracking space
        
    Quaternion Format:
        - qx, qy, qz: Vector part (rotation axis * sin(angle/2))
        - qw: Scalar part (cos(angle/2))
        - Unit quaternion: qx² + qy² + qz² + qw² = 1
        - Represents rotation from reference frame to device frame
    """
    device_name: str
    poses: List[Tuple[float, List[float]]] = field(default_factory=list)
    velocities: List[Tuple[float, List[float]]] = field(default_factory=list)
    imu_data: List[Tuple[float, List[float]]] = field(default_factory=list)
    raw_imu_data: List[Tuple[float, List[float]]] = field(default_factory=list)
    angles: Dict[Tuple[int, int, int], List[Tuple[float, float]]] = field(default_factory=lambda: defaultdict(list))
    light_data: Dict[Tuple[int, int, int], List[Tuple[float, float]]] = field(default_factory=lambda: defaultdict(list))
    button_events: List[Tuple[float, int, int]] = field(default_factory=list)
    lighthouse_data: Dict[int, List[Tuple[float, List[float]]]] = field(default_factory=lambda: defaultdict(list))
    config_data: List[Tuple[float, str, str]] = field(default_factory=list)
    last_update: float = 0.0
    
    def get_latest_pose(self) -> Optional[List[float]]:
        """
        Get the most recent 6DOF pose data for this device.
        
        Returns:
            Optional[List[float]]: Latest pose as [x, y, z, qx, qy, qz, qw] or None if no pose data
            
        Example:
            pose = device.get_latest_pose()
            if pose:
                position = pose[:3]  # [x, y, z] in meters
                orientation = pose[3:7]  # [qx, qy, qz, qw] quaternion
        """
        return self.poses[-1][1] if self.poses else None
    
    def get_latest_velocity(self) -> Optional[List[float]]:
        """
        Get the most recent velocity data for this device.
        
        Returns:
            Optional[List[float]]: Latest velocity as [vx, vy, vz] in m/s or None if no velocity data
            
        Example:
            velocity = device.get_latest_velocity()
            if velocity:
                speed = (velocity[0]**2 + velocity[1]**2 + velocity[2]**2)**0.5
        """
        return self.velocities[-1][1] if self.velocities else None
    
    def get_latest_imu(self) -> Optional[List[float]]:
        """
        Get the most recent IMU data for this device.
        
        Returns:
            Optional[List[float]]: Latest IMU as [ax, ay, az, gx, gy, gz] or None if no IMU data
            - ax, ay, az: Accelerometer data in m/s²
            - gx, gy, gz: Gyroscope data in rad/s
            
        Example:
            imu = device.get_latest_imu()
            if imu:
                acceleration = imu[:3]  # [ax, ay, az] in m/s²
                angular_velocity = imu[3:6]  # [gx, gy, gz] in rad/s
        """
        return self.imu_data[-1][1] if self.imu_data else None
    
    def is_moving(self, threshold: float = 0.1) -> bool:
        """
        Check if the device is currently moving based on recent velocity data.
        
        Args:
            threshold (float): Velocity threshold in m/s. Device is considered moving
                             if average velocity magnitude exceeds this value.
                             Default: 0.1 m/s (10 cm/s)
        
        Returns:
            bool: True if device is moving, False otherwise
            
        Note:
            Uses the last 10 velocity readings to calculate average velocity magnitude.
            Returns False if insufficient velocity data is available.
            
        Example:
            if device.is_moving(threshold=0.05):  # 5 cm/s threshold
                print("Device is moving slowly")
            elif device.is_moving(threshold=1.0):  # 1 m/s threshold
                print("Device is moving fast")
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


class UDPTelemetryReceiver:
    """
    Async UDP telemetry receiver for libsurvive tracking systems.
    
    This class provides a non-blocking, async UDP receiver that processes
    real-time telemetry data from libsurvive devices. It automatically parses incoming
    messages and organizes them into structured data containers for easy access.
    
    Features:
    - Async/await based UDP message processing
    - Automatic message parsing and data organization
    - Real-time callback system for immediate data processing
    - Statistics and monitoring
    - Cross-platform socket support
    - Data storage with automatic cleanup
    
    Message Format:
    All messages follow the format: "timestamp device_name data_type data_values..."
    
    Supported Data Types:
    - POSE: 6DOF position and orientation
    - IMU: Accelerometer and gyroscope data
    - VELOCITY: Linear velocity in 3D space
    - LIGHTHOUSE: Base station positions and orientations
    - BUTTON: Button press/release events
    - ANGLE: Lighthouse angle measurements
    - LIGHT: Photodiode light intensity readings
    - CONFIG: Configuration parameter updates
    
    Usage Example:
        # Create receiver
        receiver = UDPTelemetryReceiver(port=2333)
        
        # Add real-time callback
        async def on_message(device_name, data_type, values, timestamp):
            if data_type == 'POSE':
                print(f"Device {device_name} at {values[:3]}")
        
        receiver.add_callback(on_message)
        
        # Start receiving
        await receiver.start()
        
        # Access structured data
        for device_name in receiver.get_active_devices():
            device = receiver.get_device(device_name)
            pose = device.get_latest_pose()
            if pose:
                print(f"Position: {pose[:3]}")
    """
    
    def __init__(self, port: int = 2333, buffer_size: int = 8192):
        """
        Initialize the UDP telemetry receiver.
        
        Args:
            port (int): UDP port to listen on. Default: 2333 (libsurvive default)
            buffer_size (int): UDP socket buffer size in bytes. Default: 8192
            
        Note:
            The receiver will bind to all available network interfaces (0.0.0.0)
            on the specified port. Make sure the port is not already in use.
        """
        self.port = port
        self.buffer_size = buffer_size
        
        # Async socket
        self.socket = None
        self.running = False
        
        # Data storage
        self.devices: Dict[str, TelemetryData] = {}
        self.raw_messages: deque = deque(maxlen=10000)  # Keep last 10k messages
        
        # Statistics
        self.stats = {
            'messages_received': 0,
            'parse_errors': 0,
            'last_message_time': 0.0,
            'start_time': 0.0,
            'bytes_received': 0,
            'packets_dropped': 0
        }
        
        # Callbacks for real-time processing
        self.callbacks: List[Callable] = []
        
        # Statistics monitoring
        self.performance_stats = {
            'avg_processing_time': 0.0,
            'max_processing_time': 0.0,
            'messages_per_second': 0.0,
            'last_stats_update': time.time()
        }
    
    def add_callback(self, callback: Callable) -> None:
        """
        Add a callback function to be called for each received message.
        
        Args:
            callback (Callable): Function to call for each message. Can be async or sync.
                                Signature: callback(device_name, data_type, data_values, timestamp)
                                - device_name (str): Name of the device
                                - data_type (str): Type of data (POSE, IMU, etc.)
                                - data_values (List[str]): Parsed data values
                                - timestamp (float): Unix timestamp of the message
        
        Example:
            # Sync callback
            def on_pose(device_name, data_type, values, timestamp):
                if data_type == 'POSE':
                    print(f"Device {device_name} moved to {values[:3]}")
            
            receiver.add_callback(on_pose)
            
            # Async callback
            async def on_button(device_name, data_type, values, timestamp):
                if data_type == 'BUTTON':
                    await handle_button_press(device_name, values)
            
            receiver.add_callback(on_button)
        """
        self.callbacks.append(callback)
    
    async def start(self) -> None:
        """
        Start the async UDP receiver and begin processing messages.
        
        This method:
        1. Creates and configures the UDP socket
        2. Binds to the specified port on all interfaces
        3. Sets socket options
        4. Starts the async receive loop
        
        Note:
            This method will run indefinitely until stop() is called.
            Use asyncio.create_task() to run it concurrently with other operations.
            
        Example:
            receiver = UDPTelemetryReceiver(port=2333)
            await receiver.start()  # This will run forever
            
            # Or run concurrently:
            task = asyncio.create_task(receiver.start())
            # Do other work...
            task.cancel()  # Stop the receiver
        """
        # Create async socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(('', self.port))
        self.socket.setblocking(False)
        
        # Set socket options (OS-specific)
        try:
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)  # 1MB buffer
        except OSError:
            pass  # Some systems don't support this option
        
        try:
            self.socket.setsockopt(socket.IPPROTO_UDP, socket.SO_RCVBUF, 1024 * 1024)
        except OSError:
            pass  # macOS doesn't support this option
        
        self.running = True
        self.stats['start_time'] = time.time()
        
        print(f"Async UDP Telemetry Receiver started on port {self.port}")
        print(f"Socket buffer size: {self.buffer_size} bytes")
        
        # Start receiving loop
        await self._receive_loop()
    
    async def stop(self) -> None:
        """Stop the async UDP receiver"""
        self.running = False
        if self.socket:
            self.socket.close()
        print("Async UDP Telemetry Receiver stopped")
    
    async def _receive_loop(self) -> None:
        """Main async receive loop"""
        loop = asyncio.get_event_loop()
        
        while self.running:
            try:
                # Use asyncio to wait for data (non-blocking)
                data, addr = await loop.sock_recvfrom(self.socket, self.buffer_size)
                
                # Process message asynchronously
                await self._process_message_async(data.decode('utf-8', errors='ignore'), addr)
                
                # Update stats
                self.stats['messages_received'] += 1
                self.stats['bytes_received'] += len(data)
                self.stats['last_message_time'] = time.time()
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                if self.running:  # Only log errors if we're still supposed to be running
                    print(f"Error receiving data: {e}")
                    self.stats['packets_dropped'] += 1
                    await asyncio.sleep(0.001)  # Small delay to prevent busy loop
    
    async def _process_message_async(self, message: str, addr: Tuple[str, int]) -> None:
        """Process a single telemetry message asynchronously"""
        start_time = time.time()
        
        try:
            # Store raw message
            self.raw_messages.append({
                'timestamp': time.time(),
                'message': message,
                'source': addr
            })
            
            # Parse the message (format: "timestamp device_name data_type data...")
            parts = message.strip().split()
            if len(parts) < 3:
                return
            
            timestamp = float(parts[0])
            device_name = parts[1]
            data_type = parts[2]
            data_values = parts[3:]
            
            # Get or create device data
            if device_name not in self.devices:
                self.devices[device_name] = TelemetryData(device_name)
            
            device = self.devices[device_name]
            device.last_update = time.time()
            
            # Process different data types
            await self._process_data_type(device, data_type, data_values, timestamp)
            
            # Call registered callbacks
            for callback in self.callbacks:
                try:
                    if asyncio.iscoroutinefunction(callback):
                        await callback(device_name, data_type, data_values, timestamp)
                    else:
                        callback(device_name, data_type, data_values, timestamp)
                except Exception as e:
                    print(f"Callback error: {e}")
            
            # Update performance stats
            processing_time = time.time() - start_time
            self._update_performance_stats(processing_time)
            
        except Exception as e:
            self.stats['parse_errors'] += 1
            print(f"Error processing message: {e}")
    
    async def _process_data_type(self, device: TelemetryData, data_type: str, 
                                data_values: List[str], timestamp: float) -> None:
        """Process different types of telemetry data"""
        try:
            if data_type == 'POSE':
                if len(data_values) >= 7:
                    pose_data = [float(x) for x in data_values[:7]]
                    device.poses.append((timestamp, pose_data))
                    # Keep only recent poses
                    if len(device.poses) > 1000:
                        device.poses = device.poses[-500:]
            
            elif data_type == 'VELOCITY':
                if len(data_values) >= 3:
                    velocity_data = [float(x) for x in data_values[:3]]
                    device.velocities.append((timestamp, velocity_data))
                    if len(device.velocities) > 1000:
                        device.velocities = device.velocities[-500:]
            
            elif data_type == 'IMU':
                if len(data_values) >= 6:
                    imu_data = [float(x) for x in data_values[:6]]
                    device.imu_data.append((timestamp, imu_data))
                    if len(device.imu_data) > 1000:
                        device.imu_data = device.imu_data[-500:]
            
            elif data_type == 'RAW_IMU':
                if len(data_values) >= 6:
                    raw_imu_data = [float(x) for x in data_values[:6]]
                    device.raw_imu_data.append((timestamp, raw_imu_data))
                    if len(device.raw_imu_data) > 1000:
                        device.raw_imu_data = device.raw_imu_data[-500:]
            
            elif data_type == 'ANGLE':
                if len(data_values) >= 5:
                    sensor_id = int(data_values[0])
                    lighthouse_id = int(data_values[1])
                    axis = int(data_values[2])
                    angle = float(data_values[3])
                    confidence = float(data_values[4])
                    
                    key = (sensor_id, lighthouse_id, axis)
                    device.angles[key].append((timestamp, angle))
                    if len(device.angles[key]) > 1000:
                        device.angles[key] = device.angles[key][-500:]
            
            elif data_type == 'LIGHT':
                if len(data_values) >= 5:
                    sensor_id = int(data_values[0])
                    lighthouse_id = int(data_values[1])
                    axis = int(data_values[2])
                    light_value = float(data_values[3])
                    confidence = float(data_values[4])
                    
                    key = (sensor_id, lighthouse_id, axis)
                    device.light_data[key].append((timestamp, light_value))
                    if len(device.light_data[key]) > 1000:
                        device.light_data[key] = device.light_data[key][-500:]
            
            elif data_type == 'BUTTON':
                if len(data_values) >= 2:
                    button_id = int(data_values[0])
                    button_state = int(data_values[1])
                    device.button_events.append((timestamp, button_id, button_state))
                    if len(device.button_events) > 1000:
                        device.button_events = device.button_events[-500:]
            
            elif data_type == 'LIGHTHOUSE':
                if len(data_values) >= 7:
                    lighthouse_id = int(data_values[0])
                    lighthouse_pose = [float(x) for x in data_values[1:8]]
                    device.lighthouse_data[lighthouse_id].append((timestamp, lighthouse_pose))
                    if len(device.lighthouse_data[lighthouse_id]) > 1000:
                        device.lighthouse_data[lighthouse_id] = device.lighthouse_data[lighthouse_id][-500:]
            
            elif data_type == 'CONFIG':
                if len(data_values) >= 2:
                    config_key = data_values[0]
                    config_value = data_values[1]
                    device.config_data.append((timestamp, config_key, config_value))
                    if len(device.config_data) > 1000:
                        device.config_data = device.config_data[-500:]
        
        except (ValueError, IndexError) as e:
            self.stats['parse_errors'] += 1
            print(f"Data parsing error: {e}")
    
    def _update_performance_stats(self, processing_time: float) -> None:
        """Update processing statistics"""
        current_time = time.time()
        
        # Update processing time stats
        if self.performance_stats['avg_processing_time'] == 0:
            self.performance_stats['avg_processing_time'] = processing_time
        else:
            # Exponential moving average
            alpha = 0.1
            self.performance_stats['avg_processing_time'] = (
                alpha * processing_time + 
                (1 - alpha) * self.performance_stats['avg_processing_time']
            )
        
        self.performance_stats['max_processing_time'] = max(
            self.performance_stats['max_processing_time'], 
            processing_time
        )
        
        # Update messages per second
        if current_time - self.performance_stats['last_stats_update'] >= 1.0:
            time_diff = current_time - self.performance_stats['last_stats_update']
            messages_diff = self.stats['messages_received'] - self.performance_stats.get('last_message_count', 0)
            self.performance_stats['messages_per_second'] = messages_diff / time_diff
            self.performance_stats['last_message_count'] = self.stats['messages_received']
            self.performance_stats['last_stats_update'] = current_time
    
    def get_active_devices(self) -> List[str]:
        """
        Get list of device names that have received data recently.
        
        Returns:
            List[str]: List of device names that have been active within the last 5 seconds
            
        Note:
            A device is considered active if it has received any telemetry data
            within the last 5 seconds. This helps filter out disconnected devices.
            
        Example:
            active_devices = receiver.get_active_devices()
            print(f"Active devices: {active_devices}")
            # Output: ['HMD', 'Controller_L', 'Controller_R']
        """
        current_time = time.time()
        active_devices = []
        
        for device_name, device in self.devices.items():
            if current_time - device.last_update < 5.0:  # Active within last 5 seconds
                active_devices.append(device_name)
        
        return active_devices
    
    def get_device(self, device_name: str) -> Optional[TelemetryData]:
        """
        Get telemetry data container for a specific device.
        
        Args:
            device_name (str): Name of the device to retrieve data for
            
        Returns:
            Optional[TelemetryData]: Device data container or None if device not found
            
        Example:
            device = receiver.get_device("HMD")
            if device:
                pose = device.get_latest_pose()
                if pose:
                    print(f"HMD position: {pose[:3]}")
        """
        return self.devices.get(device_name)
    
    def get_all_devices(self) -> Dict[str, TelemetryData]:
        """
        Get all device data containers.
        
        Returns:
            Dict[str, TelemetryData]: Dictionary mapping device names to their data containers
            
        Note:
            This returns a copy of the internal device dictionary. Modifications to
            the returned dictionary will not affect the receiver's internal state.
            
        Example:
            all_devices = receiver.get_all_devices()
            for device_name, device in all_devices.items():
                print(f"Device {device_name}: {len(device.poses)} poses")
        """
        return self.devices.copy()
    
    def get_stats(self) -> Dict[str, Any]:
        """
        Get comprehensive receiver statistics and metrics.
        
        Returns:
            Dict[str, Any]: Dictionary containing various statistics:
                - messages_received: Total messages received
                - parse_errors: Number of messages that failed to parse
                - last_message_time: Timestamp of last received message
                - start_time: When the receiver was started
                - bytes_received: Total bytes received
                - packets_dropped: Number of dropped packets
                - avg_processing_time: Average message processing time (seconds)
                - max_processing_time: Maximum message processing time (seconds)
                - messages_per_second: Current message processing rate
                - runtime: Total runtime in seconds
                - active_devices: Number of currently active devices
                - total_devices: Total number of devices seen
                
        Example:
            stats = receiver.get_stats()
            print(f"Messages/sec: {stats['messages_per_second']:.1f}")
            print(f"Active devices: {stats['active_devices']}")
            print(f"Parse errors: {stats['parse_errors']}")
        """
        stats = self.stats.copy()
        stats.update(self.performance_stats)
        
        # Add runtime
        if stats['start_time'] > 0:
            stats['runtime'] = time.time() - stats['start_time']
        
        # Add device count
        stats['active_devices'] = len(self.get_active_devices())
        stats['total_devices'] = len(self.devices)
        
        return stats
    
    def export_data(self) -> Dict[str, Any]:
        """
        Export all telemetry data for analysis or storage.
        
        Returns:
            Dict[str, Any]: Complete data export containing:
                - devices: Dictionary of all device data
                - stats: Current receiver statistics
                - raw_messages: Last 10,000 raw UDP messages
                
        Note:
            This method creates a complete snapshot of all received data.
            Use this for data analysis, logging, or backup purposes.
            
        Example:
            # Export data for analysis
            data = receiver.export_data()
            
            # Save to file
            import json
            with open('tracking_session.json', 'w') as f:
                json.dump(data, f, indent=2)
            
            # Analyze device data
            for device_name, device_data in data['devices'].items():
                poses = device_data['poses']
                print(f"Device {device_name}: {len(poses)} poses recorded")
        """
        return {
            'devices': {
                name: {
                    'device_name': device.device_name,
                    'poses': device.poses,
                    'velocities': device.velocities,
                    'imu_data': device.imu_data,
                    'raw_imu_data': device.raw_imu_data,
                    'angles': dict(device.angles),
                    'light_data': dict(device.light_data),
                    'button_events': device.button_events,
                    'lighthouse_data': dict(device.lighthouse_data),
                    'config_data': device.config_data,
                    'last_update': device.last_update
                }
                for name, device in self.devices.items()
            },
            'stats': self.get_stats(),
            'raw_messages': list(self.raw_messages)
        }


async def main():
    """Main function for testing the async receiver"""
    parser = argparse.ArgumentParser(description='Async UDP Telemetry Receiver for libsurvive')
    parser.add_argument('--port', type=int, default=2333, help='UDP port to listen on')
    parser.add_argument('--buffer-size', type=int, default=8192, help='UDP buffer size')
    parser.add_argument('--export-interval', type=int, default=30, help='Export data every N seconds')
    
    args = parser.parse_args()
    
    # Create receiver
    receiver = UDPTelemetryReceiver(port=args.port, buffer_size=args.buffer_size)
    
    # Add callback for real-time processing
    async def message_callback(device_name: str, data_type: str, data_values: List[str], timestamp: float):
        """Example callback for real-time message processing"""
        if data_type == 'POSE':
            print(f"Device {device_name} pose: {data_values[:3]}")
        elif data_type == 'BUTTON':
            print(f"Device {device_name} button {data_values[0]} state: {data_values[1]}")
    
    receiver.add_callback(message_callback)
    
    try:
        print("Starting async UDP receiver...")
        print("Press Ctrl+C to stop")
        
        # Start receiver
        await receiver.start()
        
    except KeyboardInterrupt:
        print("\nStopping receiver...")
    finally:
        await receiver.stop()
        
        # Print final stats
        stats = receiver.get_stats()
        print(f"\nFinal Statistics:")
        print(f"Messages received: {stats['messages_received']}")
        print(f"Parse errors: {stats['parse_errors']}")
        print(f"Active devices: {stats['active_devices']}")
        print(f"Messages per second: {stats['messages_per_second']:.1f}")
        print(f"Average processing time: {stats['avg_processing_time']*1000:.2f}ms")


if __name__ == "__main__":
    asyncio.run(main())
