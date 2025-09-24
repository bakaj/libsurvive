#!/usr/bin/env python3
"""
Telemetry Data Structures

This module contains the TelemetryData class and related data structures
for storing and organizing libsurvive tracking data.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any
from collections import defaultdict


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
    
    def export_data(self) -> Dict[str, Any]:
        """
        Export all telemetry data for this device.
        
        Returns:
            Dict[str, Any]: Complete device data export
        """
        return {
            'device_name': self.device_name,
            'poses': self.poses,
            'velocities': self.velocities,
            'imu_times': self.imu_times,
            'gyros': self.gyros,
            'accels': self.accels,
            'raw_imu_times': self.raw_imu_times,
            'raw_gyros': self.raw_gyros,
            'raw_accels': self.raw_accels,
            'angles': dict(self.angles),
            'lengths': dict(self.lengths),
            'light_data': dict(self.light_data),
            'button_events': self.button_events,
            'lighthouse_data': dict(self.lighthouse_data),
            'config_data': self.config_data,
            'datalogs': dict(self.datalogs),
            'last_update': self.last_update
        }
