# Survive UDP Telemetry Package

A modular Python package for receiving and processing UDP telemetry data from libsurvive tracking systems.

## Package Structure

```
survive_udp/
├── __init__.py              # Package initialization and exports
├── telemetry_data.py        # Data structures and containers
├── message_parser.py        # UDP message parsing logic
├── udp_receiver.py          # Async and Jupyter receivers
├── example_usage.py         # Usage examples
└── README.md               # This file
```

## Features

- **Modular Design**: Clean separation of concerns across multiple files
- **Dual Interfaces**: Both async and Jupyter-compatible receivers
- **Complete Coverage**: Supports all libsurvive message types
- **Real-time Processing**: Callback-based message handling
- **Structured Data**: Organized telemetry data storage
- **Cross-platform**: Works on Linux, macOS, and Windows

## Quick Start

### Async Usage (Production)

```python
import asyncio
from survive_udp import UDPTelemetryReceiver

async def main():
    # Create receiver
    receiver = UDPTelemetryReceiver(port=2333)
    
    # Add callback
    async def on_message(device, data_type, values, timestamp):
        if data_type == 'POSE':
            print(f"Device {device} at {values[:3]}")
    
    receiver.add_callback(on_message)
    
    # Start receiving
    await receiver.start()
    
    # Let it run
    await asyncio.sleep(10)
    
    # Check data
    devices = receiver.get_active_devices()
    for device_name in devices:
        device = receiver.get_device(device_name)
        pose = device.get_latest_pose()
        if pose:
            print(f"Latest pose: {pose}")
    
    # Stop
    await receiver.stop()

asyncio.run(main())
```

### Jupyter Usage (Notebooks)

```python
from survive_udp import JupyterUDPReceiver

# Create receiver
receiver = JupyterUDPReceiver(port=2333)

# Start receiving
receiver.start()

# Access data synchronously
devices = receiver.get_active_devices()
device = receiver.get_device(devices[0])
pose = device.get_latest_pose()

# Stop when done
receiver.stop()
```

## Module Overview

### `telemetry_data.py`
Contains the `TelemetryData` class for structured data storage:

```python
from survive_udp import TelemetryData

device = TelemetryData("HMD")
pose = device.get_latest_pose()
velocity = device.get_latest_velocity()
imu = device.get_latest_imu()
is_moving = device.is_moving()
```

### `message_parser.py`
Handles parsing of UDP messages from libsurvive:

```python
from survive_udp import MessageParser

parser = MessageParser()
parser.parse_message(message, timestamp, device)
```

### `udp_receiver.py`
Contains both receiver classes:

- `UDPTelemetryReceiver`: Async-based for production use
- `JupyterUDPReceiver`: Threading-based for Jupyter compatibility

## Supported Message Types

- **POSE**: 6DOF position and orientation
- **VELOCITY**: Linear velocity in 3D space
- **I/i**: Calibrated and raw IMU data
- **LH_POSE**: Lighthouse base station poses
- **A**: Angle measurements
- **B**: Sweep angle data
- **W**: Sweep data
- **Y**: Sync data
- **C**: Light intensity
- **BUTTON**: Button events
- **CONFIG**: Configuration updates
- **DISCONNECT**: Device disconnection
- **EXTERNAL_POSE**: External pose data
- **EXTERNAL_VELOCITY**: External velocity data
- **INFO**: System information
- **S/L/R**: Light data from different lighthouse axes

## Data Structure

Each device's telemetry data is organized into:

- **Poses**: 6DOF position and orientation
- **Velocities**: Linear and angular velocities
- **IMU Data**: Accelerometer and gyroscope readings
- **Lighthouse Data**: Base station poses and tracking
- **Button Events**: Input device interactions
- **Config Data**: Parameter updates
- **Light Data**: Photodiode intensity readings

## Statistics and Monitoring

```python
# Get receiver statistics
stats = receiver.get_stats()
print(f"Messages: {stats['messages_received']}")
print(f"Errors: {stats['parse_errors']}")
print(f"Devices: {stats['active_devices']}")
print(f"Rate: {stats['messages_per_second']:.1f} msg/s")

# Export all data
export_data = receiver.export_data()
```

## Error Handling

The package includes robust error handling for:

- Network connectivity issues
- Malformed UDP messages
- Data type conversion errors
- Socket configuration problems

## Requirements

- Python 3.7+
- No external dependencies (uses only standard library)

## Installation

Simply copy the `survive_udp` package to your project directory and import:

```python
from survive_udp import UDPTelemetryReceiver, JupyterUDPReceiver
```

## Examples

See `example_usage.py` for complete usage examples demonstrating both async and Jupyter interfaces.
