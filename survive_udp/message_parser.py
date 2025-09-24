#!/usr/bin/env python3
"""
Message Parser for libsurvive UDP Messages

This module handles parsing of UDP messages from libsurvive tracking systems.
It supports all standard libsurvive data types and formats.
"""

from typing import List, Dict, Any
from .telemetry_data import TelemetryData


class MessageParser:
    """
    Parser for libsurvive UDP telemetry messages.
    
    This class handles the parsing of all libsurvive message types including
    poses, IMU data, lighthouse tracking, and button events.
    """
    
    def __init__(self):
        """Initialize the message parser."""
        self.parse_errors = 0
    
    def parse_message(self, message: str, timestamp: float, device: TelemetryData) -> None:
        """
        Parse a single UDP message and update device data.
        
        Args:
            message (str): Raw message string from UDP
            timestamp (float): Message timestamp
            device (TelemetryData): Device to update with parsed data
        """
        try:
            # Parse the message
            parts = message.split()
            if len(parts) < 3:
                self.parse_errors += 1
                return
            
            data_type = parts[2]
            data_values = parts[3:] if len(parts) > 3 else []
            
            # Update device timestamp
            device.last_update = timestamp
            
            # Process the data type
            self._process_data_type(device, data_type, data_values, timestamp)
            
        except Exception as e:
            self.parse_errors += 1
            print(f"Error parsing message '{message}': {e}")
    
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
            self.parse_errors += 1
    
    def get_parse_errors(self) -> int:
        """Get the number of parse errors encountered."""
        return self.parse_errors
