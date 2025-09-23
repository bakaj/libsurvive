#!/usr/bin/env python3
"""
Simple UDP receiver to test libsurvive UDP streaming
Usage: python3 udp_receiver_test.py [port]
"""

import socket
import sys
import time

def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 2333
    
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', port))
    sock.settimeout(1.0)  # 1 second timeout
    
    print(f"UDP receiver listening on port {port}")
    print("Waiting for libsurvive telemetry data...")
    print("Press Ctrl+C to stop")
    print("-" * 50)
    
    try:
        while True:
            try:
                data, addr = sock.recvfrom(8192)
                timestamp = time.strftime("%H:%M:%S")
                print(f"[{timestamp}] {addr[0]}:{addr[1]} -> {data.decode('utf-8', errors='ignore').strip()}")
            except socket.timeout:
                print(".", end="", flush=True)
            except KeyboardInterrupt:
                break
    except KeyboardInterrupt:
        pass
    
    print("\nShutting down...")
    sock.close()

if __name__ == "__main__":
    main()
