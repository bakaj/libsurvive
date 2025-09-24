"""
Survive UDP Telemetry Receiver Package

A comprehensive UDP telemetry receiver for libsurvive tracking systems.
Provides both async and Jupyter-compatible interfaces for real-time data processing.

Main Classes:
- UDPTelemetryReceiver: Async UDP receiver for production use
- JupyterUDPReceiver: Threading-based receiver for Jupyter notebooks
- TelemetryData: Structured data container for device telemetry

Usage:
    from survive_udp import UDPTelemetryReceiver, JupyterUDPReceiver
    
    # Async version
    receiver = UDPTelemetryReceiver(port=2333)
    await receiver.start()
    
    # Jupyter version
    receiver = JupyterUDPReceiver(port=2333)
    receiver.start()
"""

from .telemetry_data import TelemetryData
from .udp_receiver import UDPTelemetryReceiver, JupyterUDPReceiver
from .message_parser import MessageParser
from .system_data import SystemInfo, TelemetrySystem, ConfigOption

__version__ = "1.0.0"
__all__ = [
    "TelemetryData",
    "UDPTelemetryReceiver", 
    "JupyterUDPReceiver",
    "MessageParser",
    "SystemInfo",
    "TelemetrySystem",
    "ConfigOption"
]
