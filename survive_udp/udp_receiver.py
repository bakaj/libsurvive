#!/usr/bin/env python3
"""
UDP Receiver Classes for libsurvive Telemetry

This module contains both async and threading-based UDP receivers for
libsurvive telemetry data, optimized for different use cases.
"""

import asyncio
import socket
import time
import threading
from typing import Dict, List, Optional, Tuple, Any, Callable
from collections import defaultdict

from .telemetry_data import TelemetryData
from .message_parser import MessageParser


class UDPTelemetryReceiver:
    """
    Async UDP telemetry receiver for libsurvive tracking systems.
    
    This class provides an async/await interface for receiving and processing
    UDP telemetry messages from libsurvive. It's optimized for production use
    with high-performance async I/O.
    
    Features:
    - Async/await based message processing
    - Real-time data processing with callbacks
    - Structured data storage with automatic parsing
    - Statistics and monitoring
    - Cross-platform compatibility
    
    Usage:
        receiver = UDPTelemetryReceiver(port=2333)
        await receiver.start()
        
        # Add real-time callback
        async def on_message(device, data_type, values, timestamp):
            if data_type == 'POSE':
                print(f"Device {device} at position {values[:3]}")
        
        receiver.add_callback(on_message)
    """
    
    def __init__(self, host: str = "0.0.0.0", port: int = 2333, buffer_size: int = 8192):
        """
        Initialize the UDP telemetry receiver.
        
        Args:
            host (str): Host address to bind to (default: "0.0.0.0")
            port (int): Port to listen on (default: 2333)
            buffer_size (int): UDP buffer size in bytes (default: 8192)
        """
        self.host = host
        self.port = port
        self.buffer_size = buffer_size
        
        # Network
        self.socket = None
        self.running = False
        
        # Data storage
        self.devices: Dict[str, TelemetryData] = {}
        self.callbacks: List[Callable] = []
        
        # Parser
        self.parser = MessageParser()
        
        # Statistics
        self.stats = {
            'messages_received': 0,
            'parse_errors': 0,
            'start_time': 0.0,
            'last_message_time': 0.0
        }
    
    async def start(self) -> None:
        """
        Start the UDP receiver.
        
        Creates and binds the UDP socket, then begins listening for messages.
        """
        if self.running:
            return
        
        # Create UDP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Configure socket options
        try:
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        except OSError:
            pass  # Not supported on all systems
        
        try:
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.buffer_size)
        except OSError:
            pass  # Not supported on all systems
        
        try:
            self.socket.setsockopt(socket.IPPROTO_UDP, socket.SO_RCVBUF, self.buffer_size)
        except OSError:
            pass  # Not supported on all systems
        
        # Bind socket
        self.socket.bind((self.host, self.port))
        self.socket.setblocking(False)
        
        # Start receiving
        self.running = True
        self.stats['start_time'] = time.time()
        
        print(f"UDP telemetry receiver started on {self.host}:{self.port}")
        
        # Start receive loop
        await self._receive_loop()
    
    async def stop(self) -> None:
        """Stop the UDP receiver and close the socket."""
        if not self.running:
            return
        
        self.running = False
        
        if self.socket:
            self.socket.close()
            self.socket = None
        
        print("UDP telemetry receiver stopped")
    
    async def _receive_loop(self) -> None:
        """Main receive loop for processing UDP messages."""
        loop = asyncio.get_event_loop()
        
        while self.running:
            try:
                # Wait for data with timeout
                data, addr = await loop.run_in_executor(
                    None, 
                    lambda: self.socket.recvfrom(self.buffer_size)
                )
                
                if data:
                    await self._process_message(data.decode('utf-8', errors='ignore'))
                    
            except asyncio.CancelledError:
                break
            except Exception as e:
                if self.running:
                    print(f"Error receiving UDP data: {e}")
                break
    
    async def _process_message(self, message: str) -> None:
        """
        Process a single UDP message.
        
        Args:
            message (str): Raw UDP message string
        """
        try:
            # Parse message header
            parts = message.split()
            if len(parts) < 3:
                return
            
            timestamp = float(parts[0])
            device_name = parts[1]
            data_type = parts[2]
            data_values = parts[3:] if len(parts) > 3 else []
            
            # Get or create device
            if device_name not in self.devices:
                self.devices[device_name] = TelemetryData(device_name)
            
            device = self.devices[device_name]
            
            # Parse message
            self.parser.parse_message(message, timestamp, device)
            
            # Update statistics
            self.stats['messages_received'] += 1
            self.stats['last_message_time'] = time.time()
            self.stats['parse_errors'] = self.parser.get_parse_errors()
            
            # Call callbacks
            for callback in self.callbacks:
                try:
                    if asyncio.iscoroutinefunction(callback):
                        await callback(device_name, data_type, data_values, timestamp)
                    else:
                        callback(device_name, data_type, data_values, timestamp)
                except Exception as e:
                    print(f"Error in callback: {e}")
        
        except Exception as e:
            print(f"Error processing message: {e}")
            self.stats['parse_errors'] += 1
    
    def add_callback(self, callback: Callable) -> None:
        """
        Add a callback function to be called for each received message.
        
        Args:
            callback (Callable): Function to call with (device_name, data_type, values, timestamp)
        """
        self.callbacks.append(callback)
    
    def get_active_devices(self) -> List[str]:
        """
        Get list of active device names.
        
        Returns:
            List[str]: List of device names that have received data
        """
        return list(self.devices.keys())
    
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
        Get all device telemetry data.
        
        Returns:
            Dict[str, TelemetryData]: Dictionary mapping device names to their data
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
        
        return {
            'messages_received': self.stats['messages_received'],
            'parse_errors': self.stats['parse_errors'],
            'active_devices': len(self.devices),
            'runtime_seconds': runtime,
            'messages_per_second': self.stats['messages_received'] / runtime if runtime > 0 else 0,
            'last_message_time': self.stats['last_message_time']
        }
    
    def export_data(self) -> Dict[str, Any]:
        """
        Export all telemetry data for analysis.
        
        Returns:
            Dict[str, Any]: Complete telemetry data export
        """
        return {
            'devices': {name: device for name, device in self.devices.items()},
            'statistics': self.get_stats(),
            'export_time': time.time()
        }


class JupyterUDPReceiver:
    """
    Jupyter-compatible UDP telemetry receiver using threading.
    
    This class provides a synchronous interface for Jupyter notebooks by using
    threading instead of asyncio. It's designed to work seamlessly in Jupyter
    environments where asyncio can cause conflicts.
    
    Features:
    - Threading-based message processing
    - Synchronous interface for Jupyter compatibility
    - Real-time data processing with callbacks
    - Structured data storage with automatic parsing
    - Statistics and monitoring
    
    Usage:
        receiver = JupyterUDPReceiver(port=2333)
        receiver.start()
        
        # Access data synchronously
        devices = receiver.get_active_devices()
        device = receiver.get_device(devices[0])
        pose = device.get_latest_pose()
    """
    
    def __init__(self, host: str = "0.0.0.0", port: int = 2333, buffer_size: int = 8192):
        """
        Initialize the Jupyter-compatible UDP receiver.
        
        Args:
            host (str): Host address to bind to (default: "0.0.0.0")
            port (int): Port to listen on (default: 2333)
            buffer_size (int): UDP buffer size in bytes (default: 8192)
        """
        self.host = host
        self.port = port
        self.buffer_size = buffer_size
        
        # Network
        self.socket = None
        self.running = False
        self.receive_thread = None
        
        # Data storage
        self.devices: Dict[str, TelemetryData] = {}
        self.callbacks: List[Callable] = []
        
        # Parser
        self.parser = MessageParser()
        
        # Statistics
        self.stats = {
            'messages_received': 0,
            'parse_errors': 0,
            'start_time': 0.0,
            'last_message_time': 0.0
        }
    
    def start(self) -> None:
        """
        Start the UDP receiver in a background thread.
        
        Creates and binds the UDP socket, then starts a background thread
        for receiving messages.
        """
        if self.running:
            return
        
        # Create UDP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Configure socket options
        try:
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        except OSError:
            pass  # Not supported on all systems
        
        try:
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.buffer_size)
        except OSError:
            pass  # Not supported on all systems
        
        try:
            self.socket.setsockopt(socket.IPPROTO_UDP, socket.SO_RCVBUF, self.buffer_size)
        except OSError:
            pass  # Not supported on all systems
        
        # Bind socket
        self.socket.bind((self.host, self.port))
        self.socket.settimeout(0.1)  # Non-blocking with timeout
        
        # Start receiving
        self.running = True
        self.stats['start_time'] = time.time()
        
        # Start receive thread
        self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.receive_thread.start()
        
        print(f"Jupyter UDP telemetry receiver started on {self.host}:{self.port}")
    
    def stop(self) -> None:
        """Stop the UDP receiver and close the socket."""
        if not self.running:
            return
        
        self.running = False
        
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=1.0)
        
        if self.socket:
            self.socket.close()
            self.socket = None
        
        print("Jupyter UDP telemetry receiver stopped")
    
    def _receive_loop(self) -> None:
        """Background thread receive loop for processing UDP messages."""
        while self.running:
            try:
                # Receive data with timeout
                data, addr = self.socket.recvfrom(self.buffer_size)
                
                if data:
                    self._process_message(data.decode('utf-8', errors='ignore'))
                    
            except socket.timeout:
                continue  # Normal timeout, keep running
            except Exception as e:
                if self.running:
                    print(f"Error receiving UDP data: {e}")
                break
    
    def _process_message(self, message: str) -> None:
        """
        Process a single UDP message.
        
        Args:
            message (str): Raw UDP message string
        """
        try:
            # Parse message header
            parts = message.split()
            if len(parts) < 3:
                return
            
            timestamp = float(parts[0])
            device_name = parts[1]
            data_type = parts[2]
            data_values = parts[3:] if len(parts) > 3 else []
            
            # Get or create device
            if device_name not in self.devices:
                self.devices[device_name] = TelemetryData(device_name)
            
            device = self.devices[device_name]
            
            # Parse message
            self.parser.parse_message(message, timestamp, device)
            
            # Update statistics
            self.stats['messages_received'] += 1
            self.stats['last_message_time'] = time.time()
            self.stats['parse_errors'] = self.parser.get_parse_errors()
            
            # Call callbacks
            for callback in self.callbacks:
                try:
                    callback(device_name, data_type, data_values, timestamp)
                except Exception as e:
                    print(f"Error in callback: {e}")
        
        except Exception as e:
            print(f"Error processing message: {e}")
            self.stats['parse_errors'] += 1
    
    def add_callback(self, callback: Callable) -> None:
        """
        Add a callback function to be called for each received message.
        
        Args:
            callback (Callable): Function to call with (device_name, data_type, values, timestamp)
        """
        self.callbacks.append(callback)
    
    def get_active_devices(self) -> List[str]:
        """
        Get list of active device names.
        
        Returns:
            List[str]: List of device names that have received data
        """
        return list(self.devices.keys())
    
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
        Get all device telemetry data.
        
        Returns:
            Dict[str, TelemetryData]: Dictionary mapping device names to their data
        """
        return self.devices.copy()
    
    def get_latest_data(self) -> Dict[str, Dict[str, Any]]:
        """
        Get latest data summary for all devices.
        
        Returns:
            Dict[str, Dict[str, Any]]: Latest data for each device
        """
        latest_data = {}
        for device_name, device in self.devices.items():
            latest_data[device_name] = {
                'pose_count': len(device.poses),
                'imu_count': len(device.imu_times),
                'is_moving': device.is_moving(),
                'last_update': device.last_update
            }
        return latest_data
    
    def get_stats(self) -> Dict[str, Any]:
        """
        Get receiver statistics.
        
        Returns:
            Dict[str, Any]: Statistics including message counts, errors, and timing
        """
        current_time = time.time()
        runtime = current_time - self.stats['start_time'] if self.stats['start_time'] > 0 else 0
        
        return {
            'messages_received': self.stats['messages_received'],
            'parse_errors': self.stats['parse_errors'],
            'active_devices': len(self.devices),
            'runtime_seconds': runtime,
            'messages_per_second': self.stats['messages_received'] / runtime if runtime > 0 else 0,
            'last_message_time': self.stats['last_message_time']
        }
    
    def export_data(self) -> Dict[str, Any]:
        """
        Export all telemetry data for analysis.
        
        Returns:
            Dict[str, Any]: Complete telemetry data export
        """
        return {
            'devices': {name: device for name, device in self.devices.items()},
            'statistics': self.get_stats(),
            'export_time': time.time()
        }
