#!/usr/bin/env python3
"""
Test script for the modular survive_udp package.

This script verifies that all modules can be imported and basic
functionality works correctly.
"""

import sys
import time
import os

# Add the parent directory to the Python path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from survive_udp import UDPTelemetryReceiver, JupyterUDPReceiver, TelemetryData, MessageParser


def test_imports():
    """Test that all modules can be imported."""
    print("Testing imports...")
    
    try:
        from survive_udp import UDPTelemetryReceiver
        print("✓ UDPTelemetryReceiver imported")
    except ImportError as e:
        print(f"✗ Failed to import UDPTelemetryReceiver: {e}")
        return False
    
    try:
        from survive_udp import JupyterUDPReceiver
        print("✓ JupyterUDPReceiver imported")
    except ImportError as e:
        print(f"✗ Failed to import JupyterUDPReceiver: {e}")
        return False
    
    try:
        from survive_udp import TelemetryData
        print("✓ TelemetryData imported")
    except ImportError as e:
        print(f"✗ Failed to import TelemetryData: {e}")
        return False
    
    try:
        from survive_udp import MessageParser
        print("✓ MessageParser imported")
    except ImportError as e:
        print(f"✗ Failed to import MessageParser: {e}")
        return False
    
    return True


def test_telemetry_data():
    """Test TelemetryData class functionality."""
    print("\nTesting TelemetryData...")
    
    try:
        device = TelemetryData("test_device")
        print(f"✓ Created device: {device.device_name}")
        
        # Test pose data
        device.poses.append((time.time(), [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]))
        pose = device.get_latest_pose()
        print(f"✓ Latest pose: {pose}")
        
        # Test velocity data
        device.velocities.append((time.time(), [0.1, 0.2, 0.3]))
        velocity = device.get_latest_velocity()
        print(f"✓ Latest velocity: {velocity}")
        
        # Test IMU data
        device.accels.append([0.1, 0.2, 0.3])
        device.gyros.append([0.01, 0.02, 0.03])
        device.imu_times.append(time.time())
        imu = device.get_latest_imu()
        print(f"✓ Latest IMU: {imu}")
        
        # Test movement detection
        is_moving = device.is_moving()
        print(f"✓ Is moving: {is_moving}")
        
        return True
        
    except Exception as e:
        print(f"✗ TelemetryData test failed: {e}")
        return False


def test_message_parser():
    """Test MessageParser class functionality."""
    print("\nTesting MessageParser...")
    
    try:
        parser = MessageParser()
        device = TelemetryData("test_device")
        
        # Test POSE message
        pose_message = "123.456 test_device POSE 1.0 2.0 3.0 0.0 0.0 0.0 1.0"
        parser.parse_message(pose_message, 123.456, device)
        print(f"✓ Parsed POSE message, poses: {len(device.poses)}")
        
        # Test IMU message
        imu_message = "123.456 test_device I 1 1000 0.1 0.2 0.3 0.01 0.02 0.03 0.0 0.0 0.0 0"
        parser.parse_message(imu_message, 123.456, device)
        print(f"✓ Parsed IMU message, IMU times: {len(device.imu_times)}")
        
        # Test error handling
        error_message = "invalid message"
        parser.parse_message(error_message, 123.456, device)
        print(f"✓ Handled invalid message, errors: {parser.get_parse_errors()}")
        
        return True
        
    except Exception as e:
        print(f"✗ MessageParser test failed: {e}")
        return False


def test_receivers():
    """Test receiver classes (without actually starting them)."""
    print("\nTesting Receivers...")
    
    try:
        # Test UDPTelemetryReceiver creation
        async_receiver = UDPTelemetryReceiver(port=2333)
        print(f"✓ Created UDPTelemetryReceiver on port {async_receiver.port}")
        
        # Test JupyterUDPReceiver creation
        jupyter_receiver = JupyterUDPReceiver(port=2334)
        print(f"✓ Created JupyterUDPReceiver on port {jupyter_receiver.port}")
        
        # Test callback addition
        def test_callback(device, data_type, values, timestamp):
            pass
        
        async_receiver.add_callback(test_callback)
        jupyter_receiver.add_callback(test_callback)
        print("✓ Added callbacks to both receivers")
        
        # Test statistics
        stats = async_receiver.get_stats()
        print(f"✓ Async receiver stats: {stats}")
        
        stats = jupyter_receiver.get_stats()
        print(f"✓ Jupyter receiver stats: {stats}")
        
        return True
        
    except Exception as e:
        print(f"✗ Receiver test failed: {e}")
        return False


def main():
    """Run all tests."""
    print("Testing Modular Survive UDP Package")
    print("=" * 40)
    
    tests = [
        test_imports,
        test_telemetry_data,
        test_message_parser,
        test_receivers
    ]
    
    passed = 0
    total = len(tests)
    
    for test in tests:
        if test():
            passed += 1
        print()
    
    print("=" * 40)
    print(f"Tests passed: {passed}/{total}")
    
    if passed == total:
        print("✓ All tests passed! The modular package is working correctly.")
        return 0
    else:
        print("✗ Some tests failed. Check the output above for details.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
