# UDP Streaming Integration Documentation

## Overview

This document describes the UDP streaming integration in libsurvive. The UDP streaming feature allows all telemetry data (poses, IMU data, angles, lighthouse data, etc.) to be streamed in real-time over UDP in addition to (or instead of) being recorded to files. This enables real-time monitoring, visualization, and integration with external systems.

## Features

- **Real-time streaming**: All telemetry data streamed via UDP as it's generated
- **Zero code duplication**: Reuses existing recording infrastructure
- **Easy to enable**: Simple command-line flags
- **Configurable**: Customizable target host and port
- **Same data format**: Uses the same text format as recording files
- **Independent operation**: Can be used with or without file recording
- **Cross-platform**: Works on Windows, Linux, and macOS

## Architecture

### Integration Points

The UDP streaming is integrated into the existing recording system:

1. **Recording Functions**: Both `survive_recording_write_to_output()` and `survive_recording_write_to_output_nopreamble()` send data via UDP
2. **Initialization**: UDP socket is created during recording initialization
3. **Cleanup**: UDP socket is closed when recording is destroyed
4. **Configuration**: Uses the same configuration system as other recording options

### Data Flow

```
Telemetry Event (pose, IMU, angle, etc.)
    ↓
Recording Function Called
    ↓
Format Data String
    ↓
Write to File (if enabled)
    ↓
Write to stdout (if enabled)
    ↓
Send via UDP (if enabled) ← UDP Streaming Integration
    ↓
External Receiver Application
```

## Data Format

The UDP stream uses the **exact same text format** as recording files. Each message is a single line of text terminated with `\r\n`.

### Format Structure

```
<timestamp> <device_data>\r\n
```

Where:
- `<timestamp>`: Floating-point timestamp in seconds (time since libsurvive started)
- `<device_data>`: Device-specific data line (same format as file recording)

### Example Messages

**Pose Data:**
```
123.456789 WM0 POSE 1.234 -0.567 2.345 0.707 0.0 0.0 0.707
```

**IMU Data:**
```
123.456790 WM0 IMU 0.1 0.2 0.3 0.4 0.5 0.6
```

**Trackref Pose:**
```
123.456791 WM0 LH_POSE_TRACKREF 0 1.234 -0.567 2.345 0.707 0.0 0.0 0.707 12345678
```

**Serial Number:**
```
0.123456 WM0 SERIAL_NUMBER HTCVive_Controller_12345
```

**Lighthouse Pose:**
```
123.456792 0 LH_POSE 1.234 -0.567 2.345 0.707 0.0 0.0 0.707 12345678
```

### Message Characteristics

- **Line-based**: Each UDP packet contains one complete line of data
- **Text format**: Human-readable text format (not binary)
- **UTF-8 encoding**: Standard UTF-8 character encoding
- **Buffer size**: Maximum 8192 bytes per message (truncated if longer)
- **No fragmentation**: Each message fits in a single UDP packet

## Configuration

### Command Line Options

**Enable UDP Streaming:**
```bash
survive-cli --udp-stream
```

**Custom Host and Port:**
```bash
survive-cli --udp-stream --udp-stream-host 192.168.1.100 --udp-stream-port 5000
```

**Combine with File Recording:**
```bash
survive-cli --record data.txt --udp-stream --udp-stream-host 192.168.1.100
```

**Combine with stdout Recording:**
```bash
survive-cli --record-stdout --udp-stream
```

### Configuration Options

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `--udp-stream` | boolean | `false` | Enable UDP streaming |
| `--udp-stream-host` | string | `127.0.0.1` | Target host IP address |
| `--udp-stream-port` | integer | `2333` | Target port number |

### Configuration File

Add to your configuration file:
```
udp-stream = 1
udp-stream-host = 192.168.1.100
udp-stream-port = 5000
```

## Implementation Details

### Data Structure

**SurviveRecordingData** (`survive_recording.c:43-59`):
```c
typedef struct SurviveRecordingData {
    // ... existing fields ...
    
    // UDP streaming fields
    bool udpStreamEnabled;
    int udp_socket;
    struct sockaddr_in udp_target_addr;
    char udp_buffer[8192];  // Buffer for UDP messages
} SurviveRecordingData;
```

### Key Functions

1. **`udp_stream_init()`** (`survive_recording.c:178-206`)
   - Creates UDP socket
   - Resolves target host IP address
   - Configures target address and port
   - Validates configuration

2. **`udp_stream_send()`** (`survive_recording.c:208-223`)
   - Sends data via UDP socket
   - Handles buffer truncation if message is too long
   - Uses `sendto()` for connectionless UDP transmission

3. **`udp_stream_cleanup()`** (`survive_recording.c:225-230`)
   - Closes UDP socket
   - Resets socket descriptor

### Integration Points

**Initialization** (`survive_recording.c:544-549`):
- UDP streaming is initialized when recording is installed
- Socket is created if `udp-stream` config flag is enabled
- Initialization happens before any data is recorded

**Data Transmission** (`survive_recording.c:136-144`, `167-174`):
- Both `survive_recording_write_to_output()` and `survive_recording_write_to_output_nopreamble()` send data via UDP
- Data is formatted into the buffer and sent immediately
- No queuing or buffering - data is sent as it's generated

**Cleanup** (`survive_recording.c:521-529`):
- UDP socket is closed when recording is destroyed
- Cleanup happens during `survive_destroy_recording()`

### Platform Support

**Windows:**
- Uses `winsock2.h` for socket functions
- `MSG_NOSIGNAL` is not available (defined as 0)

**Linux/macOS:**
- Uses standard POSIX socket functions
- `MSG_NOSIGNAL` flag prevents SIGPIPE signals

## Usage Examples

### Basic UDP Receiver (Python)

```python
import socket
import sys

def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 2333
    
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', port))
    
    print(f"UDP receiver listening on port {port}")
    print("Waiting for libsurvive telemetry data...")
    print("Press Ctrl+C to stop")
    print("-" * 50)
    
    try:
        while True:
            data, addr = sock.recvfrom(8192)
            line = data.decode('utf-8', errors='ignore').strip()
            print(f"[{addr[0]}:{addr[1]}] {line}")
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()

if __name__ == "__main__":
    main()
```

### Parsing Telemetry Data

```python
import socket
import re

def parse_telemetry_line(line):
    """Parse a telemetry line into components"""
    parts = line.strip().split()
    if len(parts) < 2:
        return None
    
    timestamp = float(parts[0])
    device = parts[1]
    data_type = parts[2] if len(parts) > 2 else None
    data_values = parts[3:] if len(parts) > 3 else []
    
    return {
        'timestamp': timestamp,
        'device': device,
        'type': data_type,
        'values': data_values,
        'raw': line
    }

# Receive and parse UDP stream
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 2333))

while True:
    data, addr = sock.recvfrom(8192)
    line = data.decode('utf-8', errors='ignore')
    parsed = parse_telemetry_line(line)
    
    if parsed:
        if parsed['type'] == 'POSE':
            print(f"Pose: {parsed['device']} at {parsed['timestamp']}")
        elif parsed['type'] == 'IMU':
            print(f"IMU: {parsed['device']} at {parsed['timestamp']}")
        elif 'LH_POSE_TRACKREF' in parsed['type']:
            print(f"Trackref: {parsed['device']} LH {parsed['values'][0]}")
```

### Filtering Specific Data Types

```python
import socket

def filter_telemetry(port=2333, filter_type=None):
    """Receive UDP stream and filter by data type"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', port))
    
    print(f"Listening on port {port}")
    if filter_type:
        print(f"Filtering for: {filter_type}")
    
    while True:
        data, addr = sock.recvfrom(8192)
        line = data.decode('utf-8', errors='ignore')
        
        if filter_type is None or filter_type in line:
            print(line.strip())

# Example: Only show pose data
filter_telemetry(port=2333, filter_type='POSE')
```

### Real-time Visualization

```python
import socket
import json
from collections import defaultdict

class TelemetryCollector:
    def __init__(self, port=2333):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', port))
        self.data = defaultdict(dict)
    
    def run(self):
        while True:
            data, addr = self.sock.recvfrom(8192)
            line = data.decode('utf-8', errors='ignore')
            self.process_line(line)
    
    def process_line(self, line):
        parts = line.strip().split()
        if len(parts) < 3:
            return
        
        timestamp = float(parts[0])
        device = parts[1]
        data_type = parts[2]
        
        if data_type == 'POSE' and len(parts) >= 10:
            # Store pose data
            self.data[device]['pose'] = {
                'timestamp': timestamp,
                'position': [float(parts[3]), float(parts[4]), float(parts[5])],
                'rotation': [float(parts[6]), float(parts[7]), float(parts[8]), float(parts[9])]
            }
            print(f"{device}: {self.data[device]['pose']['position']}")

collector = TelemetryCollector(2333)
collector.run()
```

## Testing

### Using the Test Receiver

A test receiver is included in the repository:

```bash
# Terminal 1: Start UDP receiver
python3 udp_receiver_test.py 2333

# Terminal 2: Start libsurvive with UDP streaming
./survive-cli --udp-stream
```

The test receiver will display all incoming UDP messages with timestamps.

### Verifying UDP Stream

**Check if UDP is enabled:**
```bash
./survive-cli --udp-stream --verbose 100
# Look for: "UDP streaming enabled to 127.0.0.1:2333"
```

**Test with netcat:**
```bash
# Terminal 1: Listen with netcat
nc -u -l 2333

# Terminal 2: Start libsurvive
./survive-cli --udp-stream
```

**Test with tcpdump/wireshark:**
```bash
# Capture UDP packets
sudo tcpdump -i any -n udp port 2333

# Or use wireshark
wireshark -f "udp port 2333"
```

## Troubleshooting

### No Data Received

**Possible Causes:**

1. **UDP streaming not enabled**
   - **Solution**: Ensure `--udp-stream` flag is specified
   - **Check**: Look for "UDP streaming enabled" message in logs

2. **Wrong host/port**
   - **Solution**: Verify target host and port match receiver
   - **Check**: Use `--udp-stream-host` and `--udp-stream-port` to configure

3. **Firewall blocking**
   - **Solution**: Check firewall rules allow UDP traffic
   - **Check**: Test with localhost first (`127.0.0.1`)

4. **Receiver not listening**
   - **Solution**: Ensure receiver application is running and bound to correct port
   - **Check**: Use `netstat -ulnp | grep <port>` to verify listener

### Socket Creation Failed

**Error**: "Failed to create UDP socket for streaming"

**Possible Causes:**
- System resource limits (too many open sockets)
- Permission issues
- Network subsystem not available

**Solution**: Check system logs, verify network is available, restart application

### Invalid Host Error

**Error**: "Invalid UDP stream host: <host>"

**Possible Causes:**
- Invalid IP address format
- Hostname resolution failed (only IP addresses are supported)

**Solution**: Use IP addresses (e.g., `192.168.1.100`) instead of hostnames

### Data Truncation

**Symptom**: Messages are cut off or incomplete

**Cause**: Message exceeds 8192 byte buffer limit

**Solution**: This is expected behavior for very long messages. Most telemetry messages are much shorter than 8192 bytes.

## Performance Considerations

### Network Impact

- **Bandwidth**: UDP streaming adds network traffic proportional to recording data rate
- **Packet rate**: One UDP packet per telemetry event (can be high frequency for IMU data)
- **No reliability**: UDP is connectionless and unreliable - packets may be lost
- **No ordering guarantee**: Packets may arrive out of order

### Recommendations

1. **Local network**: Use UDP streaming on local networks for best performance
2. **Network capacity**: Ensure network can handle the data rate (typically < 1 MB/s)
3. **Receiver performance**: Ensure receiver can process messages fast enough
4. **Firewall**: Configure firewall to allow UDP traffic on the specified port

### Limitations

- **No retransmission**: Lost packets are not retransmitted
- **No flow control**: Sender doesn't wait for receiver acknowledgment
- **Buffer size**: Maximum 8192 bytes per message
- **IPv4 only**: Currently supports IPv4 addresses only (not IPv6)

## Use Cases

### Real-time Visualization

Stream pose and IMU data to a visualization application for real-time monitoring:

```bash
./survive-cli --udp-stream --udp-stream-host 192.168.1.100 --udp-stream-port 5000
```

### Multi-system Integration

Stream data to multiple systems simultaneously by using UDP multicast or multiple unicast streams:

```bash
# System 1: Visualization
./survive-cli --udp-stream --udp-stream-host 192.168.1.100

# System 2: Logging (with file recording)
./survive-cli --record data.txt --udp-stream --udp-stream-host 192.168.1.101
```

### Remote Monitoring

Monitor tracking data from a remote system:

```bash
# On tracking system
./survive-cli --udp-stream --udp-stream-host <remote_ip>

# On monitoring system
python3 udp_receiver_test.py 2333
```

### Development and Debugging

Stream data during development for real-time debugging:

```bash
./survive-cli --udp-stream --record-stdout --verbose 200
```

## Code Locations

- **Recording Functions**: `src/survive_recording.c` (lines 112-175)
- **UDP Initialization**: `src/survive_recording.c` (lines 178-206)
- **UDP Send Function**: `src/survive_recording.c` (lines 208-223)
- **UDP Cleanup**: `src/survive_recording.c` (lines 225-230)
- **Recording Installation**: `src/survive_recording.c` (lines 535-583)
- **Data Structure**: `src/survive_recording.c` (lines 43-59)
- **Configuration**: `src/survive_recording.c` (lines 69, 75-76)
- **Test Receiver**: `udp_receiver_test.py`

## Related Documentation

- `TRACKREF_POSE_RECORDING.md` - Documentation for trackref pose recording
- `SERIAL_NUMBER_RECORDING.md` - Documentation for serial number recording
- `UDP_STREAMING_README.md` - Brief overview of UDP streaming
- `architecture.md` - Overall libsurvive architecture

## Future Enhancements

Potential improvements for future versions:

- **IPv6 support**: Add support for IPv6 addresses
- **Multicast support**: Enable UDP multicast for one-to-many streaming
- **Compression**: Optional message compression for bandwidth efficiency
- **Binary format**: Optional binary format for reduced bandwidth
- **Reliability**: Optional TCP mode for reliable delivery
- **Buffering**: Configurable buffering for high-frequency data

