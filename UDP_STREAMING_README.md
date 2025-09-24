# UDP Streaming for libsurvive

This implementation adds UDP streaming capability to libsurvive by modifying the existing recording system. All telemetry data that would normally be recorded to a file is now also streamed over UDP in real-time.

## Features

- **Real-time streaming**: All telemetry data (poses, IMU, angles, lighthouse data, etc.) streamed via UDP
- **Zero code duplication**: Reuses existing recording infrastructure
- **Easy to enable**: Simple command-line flags
- **Configurable**: Customizable target host and port
- **Same data format**: Uses the same text format as recording files

## Usage

### Enable UDP Streaming

```bash
# Basic usage - streams to localhost:2333
./survive-cli --udp-stream

# Custom host and port
./survive-cli --udp-stream --udp-stream-host 192.168.1.100 --udp-stream-port 5000

# Combine with file recording
./survive-cli --record data.txt --udp-stream --udp-stream-host 192.168.1.100
```

### Configuration Options

- `--udp-stream`: Enable UDP streaming (boolean)
- `--udp-stream-host HOST`: Target host IP address (default: 127.0.0.1)
- `--udp-stream-port PORT`: Target port number (default: 2333)

### Testing with UDP Receiver

Use the included test receiver to see the streamed data:

```bash
# Terminal 1: Start UDP receiver
python3 udp_receiver_test.py 2333

# Terminal 2: Start libsurvive with UDP streaming
./survive-cli --udp-stream
```

## Data Format

The UDP stream uses the same text format as recording files:

```
123.456789 HMD0 POSE 1.0 2.0 3.0 0.0 0.0 0.0 1.0
123.456790 HMD0 IMU 0.1 0.2 0.3 0.4 0.5 0.6
123.456791 LH0 0 0.123456 0.234567
...
```

Where:
- First number is timestamp
- Device name (HMD0, LH0, etc.)
- Data type (POSE, IMU, angle data, etc.)
- Data values

## Implementation Details

The implementation modifies `survive_recording.c` to:

1. Add UDP socket fields to `SurviveRecordingData` structure
2. Initialize UDP socket in `survive_install_recording()`
3. Send data via UDP in `survive_recording_write_to_output()` functions
4. Clean up UDP resources in `survive_destroy_recording()`

## Building

The UDP streaming feature is automatically included when building libsurvive. No additional dependencies are required beyond the standard socket libraries.

## Troubleshooting

- **No data received**: Check that the target host/port is correct and accessible
- **Permission denied**: Ensure the target port is not restricted
- **Connection refused**: Verify the receiving application is running and listening on the correct port
- **Invalid host**: Use IP addresses instead of hostnames for better reliability

## Example Applications

You can create custom applications to receive and process the UDP stream:

```python
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 2333))

while True:
    data, addr = sock.recvfrom(8192)
    # Process telemetry data
    print(data.decode())
```

This allows for real-time visualization, logging, analysis, or integration with other systems.
