#!/usr/bin/env python3
"""
Example Usage of the Modular Survive UDP Package

This script demonstrates how to use the split survive_udp package
for both async and Jupyter-compatible scenarios.
"""

import asyncio
import time
import sys
import os

# Add the parent directory to the Python path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from survive_udp import UDPTelemetryReceiver, JupyterUDPReceiver, TelemetryData


async def async_example():
    """Example of using the async UDP receiver."""
    print("=== Async UDP Receiver Example ===")
    
    # Create receiver
    receiver = UDPTelemetryReceiver(port=2333)
    
    # Add callback for real-time processing
    async def on_message(device, data_type, values, timestamp):
        if data_type == 'POSE':
            print(f"Device {device} pose: {values[:3]}")
        elif data_type == 'I':
            print(f"Device {device} IMU: {values[2:8]}")
    
    receiver.add_callback(on_message)
    
    # Start receiver
    print("Starting async receiver...")
    await receiver.start()
    
    # Let it run for a bit
    await asyncio.sleep(5)
    
    # Check statistics
    stats = receiver.get_stats()
    print(f"Received {stats['messages_received']} messages")
    print(f"Active devices: {receiver.get_active_devices()}")
    
    # Stop receiver
    await receiver.stop()


def jupyter_example():
    """Example of using the Jupyter-compatible receiver."""
    print("=== Jupyter UDP Receiver Example ===")
    
    # Create receiver
    receiver = JupyterUDPReceiver(port=2334)
    
    # Add callback for real-time processing
    def on_message(device, data_type, values, timestamp):
        if data_type == 'POSE':
            print(f"Device {device} pose: {values[:3]}")
        elif data_type == 'I':
            print(f"Device {device} IMU: {values[2:8]}")
    
    receiver.add_callback(on_message)
    
    # Start receiver
    print("Starting Jupyter receiver...")
    receiver.start()
    
    # Let it run for a bit
    time.sleep(5)
    
    # Check statistics
    stats = receiver.get_stats()
    print(f"Received {stats['messages_received']} messages")
    print(f"Active devices: {receiver.get_active_devices()}")
    
    # Access device data
    for device_name in receiver.get_active_devices():
        device = receiver.get_device(device_name)
        pose = device.get_latest_pose()
        if pose:
            print(f"Device {device_name} latest pose: {pose[:3]}")
    
    # Stop receiver
    receiver.stop()


def main():
    """Run both examples."""
    print("Survive UDP Package Examples")
    print("=" * 40)
    
    # Run Jupyter example (synchronous)
    jupyter_example()
    
    print("\n" + "=" * 40)
    
    # Run async example
    asyncio.run(async_example())


if __name__ == "__main__":
    main()
