#!/usr/bin/env python3
"""
System Data Structures

This module contains classes for storing system-level information
from libsurvive, separate from device telemetry data.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any, TYPE_CHECKING
from collections import defaultdict

if TYPE_CHECKING:
    from .telemetry_data import TelemetryData


@dataclass
class ConfigOption:
    """
    Configuration option with type and value information.
    """
    name: str
    option_type: str  # 'b' (bool), 'i' (int), 'f' (float), 's' (string)
    value: Any
    timestamp: float
    
    def __post_init__(self):
        """Convert value to appropriate type based on option_type."""
        if self.option_type == 'b':  # boolean
            self.value = bool(self.value)
        elif self.option_type == 'i':  # integer
            self.value = int(self.value)
        elif self.option_type == 'f':  # float
            self.value = float(self.value)
        elif self.option_type == 's':  # string
            self.value = str(self.value)


@dataclass
class SystemInfo:
    """
    System information and log messages from libsurvive.
    
    This class stores system-level data like INFO messages, OPTION configurations,
    and other non-device telemetry data.
    """
    # System log messages
    info_logs: List[Tuple[float, List[str]]] = field(default_factory=list)  # [(timestamp, [message_parts])]
    
    # Configuration options (structured)
    config_options: Dict[str, ConfigOption] = field(default_factory=dict)  # {option_name: ConfigOption}
    
    # Lighthouse system data
    lighthouse_updates: Dict[int, List[Tuple[float, List[float]]]] = field(default_factory=lambda: defaultdict(list))  # {lighthouse_id: [(timestamp, [mode, accel_x, accel_y, accel_z])]}
    
    # SPHERE estimated positions (MPFIT algorithm results)
    sphere_estimates: Dict[str, List[Tuple[float, List[float]]]] = field(default_factory=lambda: defaultdict(list))  # {device_pair: [(timestamp, [radius, confidence, x, y, z])]}
    
    # System statistics
    system_stats: Dict[str, Any] = field(default_factory=dict)
    
    # Metadata
    last_update: float = 0.0
    
    def add_info_log(self, timestamp: float, message_parts: List[str]) -> None:
        """
        Add a system info log message.
        
        Args:
            timestamp (float): Message timestamp
            message_parts (List[str]): Parsed message components
        """
        self.info_logs.append((timestamp, message_parts))
        self.last_update = timestamp
    
    def add_config_option(self, option_name: str, option_type: str, value: Any, timestamp: float = None) -> None:
        """
        Add a configuration option with type information.
        
        Args:
            option_name (str): Name of the configuration option
            option_type (str): Type of the option ('b', 'i', 'f', 's')
            value (Any): Option value
            timestamp (float): Option timestamp (defaults to current time)
        """
        if timestamp is None:
            import time
            timestamp = time.time()
        
        config_option = ConfigOption(
            name=option_name,
            option_type=option_type,
            value=value,
            timestamp=timestamp
        )
        
        self.config_options[option_name] = config_option
        self.last_update = timestamp
    
    def get_latest_logs(self, count: int = 10) -> List[Tuple[float, List[str]]]:
        """
        Get the most recent log messages.
        
        Args:
            count (int): Number of recent logs to return
            
        Returns:
            List[Tuple[float, List[str]]]: Recent log messages
        """
        return self.info_logs[-count:] if self.info_logs else []
    
    def get_logs_by_keyword(self, keyword: str) -> List[Tuple[float, List[str]]]:
        """
        Get log messages containing a specific keyword.
        
        Args:
            keyword (str): Keyword to search for
            
        Returns:
            List[Tuple[float, List[str]]]: Matching log messages
        """
        return [
            (timestamp, message) for timestamp, message in self.info_logs
            if any(keyword.lower() in part.lower() for part in message)
        ]
    
    def get_config_option(self, option_name: str) -> Optional[ConfigOption]:
        """
        Get a specific configuration option.
        
        Args:
            option_name (str): Name of the option
            
        Returns:
            Optional[ConfigOption]: The configuration option or None if not found
        """
        return self.config_options.get(option_name)
    
    def get_config_options_by_type(self, option_type: str) -> Dict[str, ConfigOption]:
        """
        Get all configuration options of a specific type.
        
        Args:
            option_type (str): Type to filter by ('b', 'i', 'f', 's')
            
        Returns:
            Dict[str, ConfigOption]: Options of the specified type
        """
        return {
            name: option for name, option in self.config_options.items()
            if option.option_type == option_type
        }
    
    def get_config_value(self, option_name: str, default: Any = None) -> Any:
        """
        Get the value of a configuration option.
        
        Args:
            option_name (str): Name of the option
            default (Any): Default value if option not found
            
        Returns:
            Any: The option value or default
        """
        option = self.config_options.get(option_name)
        return option.value if option else default
    
    def add_lighthouse_update(self, lighthouse_id: int, timestamp: float, mode: int, accel_data: List[float]) -> None:
        """
        Add a lighthouse update (LH_UP message).
        
        Args:
            lighthouse_id (int): ID of the lighthouse
            timestamp (float): Update timestamp
            mode (int): Lighthouse mode
            accel_data (List[float]): Accelerometer data [x, y, z]
        """
        self.lighthouse_updates[lighthouse_id].append((timestamp, [mode] + accel_data))
        self.last_update = timestamp
    
    def get_lighthouse_updates(self, lighthouse_id: int) -> List[Tuple[float, List[float]]]:
        """
        Get lighthouse updates for a specific lighthouse.
        
        Args:
            lighthouse_id (int): ID of the lighthouse
            
        Returns:
            List[Tuple[float, List[float]]]: Updates for the lighthouse
        """
        return self.lighthouse_updates.get(lighthouse_id, [])
    
    def get_lighthouse_ids(self) -> List[int]:
        """
        Get list of lighthouse IDs that have updates.
        
        Returns:
            List[int]: List of lighthouse IDs
        """
        return list(self.lighthouse_updates.keys())
    
    def add_sphere_estimate(self, device_pair: str, timestamp: float, radius: float, confidence: float, position: List[float]) -> None:
        """
        Add a SPHERE estimated position (MPFIT algorithm result).
        
        Args:
            device_pair (str): Device pair identifier (e.g., 'WM1_2', 'WM0_2')
            timestamp (float): Estimate timestamp
            radius (float): Estimated radius
            confidence (float): Confidence value
            position (List[float]): Estimated position [x, y, z]
        """
        self.sphere_estimates[device_pair].append((timestamp, [radius, confidence] + position))
        self.last_update = timestamp
    
    def get_sphere_estimates(self, device_pair: str) -> List[Tuple[float, List[float]]]:
        """
        Get SPHERE estimates for a specific device pair.
        
        Args:
            device_pair (str): Device pair identifier
            
        Returns:
            List[Tuple[float, List[float]]]: Estimates for the device pair
        """
        return self.sphere_estimates.get(device_pair, [])
    
    def get_sphere_device_pairs(self) -> List[str]:
        """
        Get list of device pairs that have SPHERE estimates.
        
        Returns:
            List[str]: List of device pair identifiers
        """
        return list(self.sphere_estimates.keys())
    
    def export_data(self) -> Dict[str, Any]:
        """
        Export system information data.
        
        Returns:
            Dict[str, Any]: Complete system data export
        """
        return {
            'info_logs': self.info_logs,
            'config_options': {name: option.__dict__ for name, option in self.config_options.items()},
            'lighthouse_updates': self.lighthouse_updates,
            'sphere_estimates': self.sphere_estimates,
            'system_stats': self.system_stats,
            'last_update': self.last_update
        }


@dataclass
class TelemetrySystem:
    """
    Complete telemetry system container.
    
    This class manages both device telemetry data and system information,
    providing a clean separation between device data and system messages.
    """
    # Device telemetry data
    devices: Dict[str, 'TelemetryData'] = field(default_factory=dict)
    
    # System information
    system_info: SystemInfo = field(default_factory=SystemInfo)
    
    # Statistics
    total_messages: int = 0
    parse_errors: int = 0
    start_time: float = 0.0
    
    def add_device(self, device_name: str) -> 'TelemetryData':
        """
        Add a new device to the system.
        
        Args:
            device_name (str): Name of the device
            
        Returns:
            TelemetryData: The created device data container
        """
        if device_name not in self.devices:
            from .telemetry_data import TelemetryData
            self.devices[device_name] = TelemetryData(device_name)
        return self.devices[device_name]
    
    def get_device(self, device_name: str) -> Optional['TelemetryData']:
        """
        Get device telemetry data.
        
        Args:
            device_name (str): Name of the device
            
        Returns:
            Optional[TelemetryData]: Device data or None if not found
        """
        return self.devices.get(device_name)
    
    def get_active_devices(self) -> List[str]:
        """
        Get list of active device names.
        
        Returns:
            List[str]: List of device names that have received data
        """
        return list(self.devices.keys())
    
    def is_device_name(self, name: str) -> bool:
        """
        Check if a name represents a tracking device.
        
        Args:
            name (str): Name to check
            
        Returns:
            bool: True if it's a device name, False if it's system data
        """
        system_names = {'INFO', 'OPTION', 'LOG', 'SYSTEM', 'LH_UP', 'SPHERE'}
        return name not in system_names
    
    def export_data(self) -> Dict[str, Any]:
        """
        Export all telemetry system data.
        
        Returns:
            Dict[str, Any]: Complete system export
        """
        return {
            'devices': {name: device.export_data() for name, device in self.devices.items()},
            'system_info': self.system_info.export_data(),
            'statistics': {
                'total_messages': self.total_messages,
                'parse_errors': self.parse_errors,
                'active_devices': len(self.devices),
                'start_time': self.start_time
            }
        }
