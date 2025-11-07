# Serial Number Recording Documentation

## Overview

This document describes the implementation of device serial number recording in libsurvive. Serial numbers are unique identifiers for tracked devices (controllers, trackers, HMDs) that are extracted from the device configuration data and recorded to the output stream.

## What is a Serial Number?

A **serial number** is a unique identifier for a tracked device:
- **Format**: String of up to 15 characters (stored in a 16-byte buffer including null terminator)
- **Source**: Extracted from the device's configuration data (`device_serial_number` field in JSON config)
- **Purpose**: Provides a stable, unique identifier for each physical device
- **Uniqueness**: Each device has a unique serial number that persists across sessions

Serial numbers are distinct from:
- **Codename**: Runtime-assigned identifier (e.g., `WM0`, `WM1`) that may change between sessions
- **Base Station ID**: Identifier for lighthouses/base stations (hexadecimal format)

## Data Recorded

For each tracked device, the following data is recorded:

- **Tracker codename** (e.g., `WM0`, `WM1`, `HMD`)
- **Serial number** (device's unique serial number string)

The serial number is only recorded if it is available (non-empty string).

## Output Format

**Format:**
```
<timestamp> <tracker_codename> SERIAL_NUMBER <serial_number>\r\n
```

**Example:**
```
0.123456 WM0 SERIAL_NUMBER HTCVive_Controller_12345
0.234567 WM1 SERIAL_NUMBER HTCVive_Controller_67890
0.345678 HMD SERIAL_NUMBER HTCVive_HMD_ABCDE
```

**Fields:**
- `<timestamp>`: Floating-point timestamp in seconds (time since libsurvive started)
- `<tracker_codename>`: Device identifier (e.g., `WM0`, `WM1`, `HMD`)
- `SERIAL_NUMBER`: Literal tag indicating serial number data
- `<serial_number>`: The device's serial number string (up to 15 characters)

**Note:** The serial number is recorded exactly as it appears in the device configuration, with no modifications except null-termination.

## When Serial Numbers are Recorded

Serial numbers are recorded **once per device** when:

1. **Device Configuration is Processed** (`survive_process.c:303`)
   - **When**: After the device configuration JSON is parsed and loaded
   - **Trigger**: `survive_default_config_process` is called with the device's configuration data
   - **Condition**: Serial number must be non-empty (extracted from config)
   - **Frequency**: Once per device, immediately after config parsing

The recording happens in this sequence:
1. Device connects and is detected
2. Device configuration is read from the device
3. Configuration JSON is parsed
4. `device_serial_number` field is extracted and stored in `so->serial_number`
5. `survive_recording_serial_number_process` is called
6. Serial number is written to the recording output

## How Serial Numbers are Obtained

### Source: Device Configuration

Serial numbers are extracted from the device's configuration data, which is stored in JSON format on the device itself. The configuration contains a `device_serial_number` field that is parsed during device initialization.

### Parsing Process

1. **Configuration Reading** (`survive_default_devices.c:332-340`)
   - Device configuration JSON is read from the hardware
   - JSON parser extracts the `device_serial_number` field
   - Serial number is copied to `so->serial_number[16]` buffer
   - String is null-terminated to ensure safety

2. **Storage**
   - Stored in `SurviveObject->serial_number` (16-byte character array)
   - Maximum length: 15 characters + null terminator
   - If serial number is longer than 15 characters, it is truncated

3. **Validation**
   - Serial number is only recorded if `so->serial_number[0] != '\0'`
   - Empty or missing serial numbers are silently skipped

## How to Enable Recording

Serial number recording is **always enabled** when recording is active. There is no separate configuration flag to enable/disable serial number recording specifically.

### Prerequisites

1. **Recording must be enabled**: Use `--dataout` or `--record-to-stdout` flags
2. **Device must have a serial number**: The device's configuration must contain a `device_serial_number` field
3. **Configuration must be parsed**: The device must successfully read and parse its configuration

### Command Line

```bash
# Record to file (serial numbers will be included)
survive-cli --dataout recording.txt

# Record to stdout (serial numbers will be included)
survive-cli --record-to-stdout
```

### Automatic Recording

Serial numbers are automatically recorded as part of the device initialization process. No additional configuration is needed beyond enabling general recording.

## Implementation Details

### Key Functions

1. **`survive_recording_serial_number_process`** (`survive_recording.c:255-263`)
   - Records the device's serial number
   - Checks if serial number is non-empty before recording
   - Outputs `SERIAL_NUMBER` tag with codename and serial number
   - No return value (void function)

2. **Configuration Parsing** (`survive_default_devices.c:332-340`)
   - Extracts `device_serial_number` from JSON configuration
   - Stores in `SurviveObject->serial_number[16]` buffer
   - Ensures null-termination

3. **Recording Trigger** (`survive_process.c:303`)
   - Called after configuration is successfully parsed
   - Part of `survive_default_config_process` function
   - Executes once per device during initialization

### Data Structure

**SurviveObject Structure** (`survive.h:138`):
```c
char serial_number[16]; // 13 letters device serial number
```

**Note:** The comment says "13 letters" but the buffer is 16 bytes, allowing for up to 15 characters plus null terminator. The actual length may vary by device.

### Recording Flow

```
Device Connection
    ↓
Read Device Configuration (JSON)
    ↓
Parse JSON Configuration
    ↓
Extract device_serial_number field
    ↓
Store in so->serial_number[16]
    ↓
survive_default_config_process()
    ↓
survive_recording_serial_number_process()
    ↓
Check: serial_number[0] != '\0'?
    ↓ (if true)
Write: "<codename> SERIAL_NUMBER <serial_number>\r\n"
```

## Usage Examples

### Parsing Serial Number Records

**Python Example:**
```python
import re

def parse_serial_number(line):
    # Pattern: 0.123456 WM0 SERIAL_NUMBER HTCVive_Controller_12345
    pattern = r'([\d.]+)\s+(\w+)\s+SERIAL_NUMBER\s+(.+?)\r?\n?$'
    match = re.match(pattern, line)
    if match:
        timestamp, codename, serial = match.groups()
        return {
            'timestamp': float(timestamp),
            'codename': codename,
            'serial_number': serial.strip()
        }
    return None

# Read serial numbers from recording
serial_numbers = {}
with open('recording.txt', 'r') as f:
    for line in f:
        if 'SERIAL_NUMBER' in line:
            data = parse_serial_number(line)
            if data:
                serial_numbers[data['codename']] = data['serial_number']

print(serial_numbers)
# Output: {'WM0': 'HTCVive_Controller_12345', 'WM1': 'HTCVive_Controller_67890'}
```

### Mapping Codename to Serial Number

```python
# Create mapping from codename to serial number
codename_to_serial = {}
with open('recording.txt', 'r') as f:
    for line in f:
        if 'SERIAL_NUMBER' in line:
            parts = line.strip().split()
            if len(parts) >= 3:
                codename = parts[1]  # Second field is codename
                serial = ' '.join(parts[3:])  # Everything after SERIAL_NUMBER
                codename_to_serial[codename] = serial

# Use mapping to identify devices
for codename, serial in codename_to_serial.items():
    print(f"Device {codename} has serial number {serial}")
```

### Filtering by Serial Number

```python
# Find all data for a specific device by serial number
target_serial = "HTCVive_Controller_12345"
target_codename = None

# First, find the codename for this serial number
with open('recording.txt', 'r') as f:
    for line in f:
        if 'SERIAL_NUMBER' in line and target_serial in line:
            parts = line.strip().split()
            target_codename = parts[1]
            break

# Then filter all data for this codename
if target_codename:
    with open('recording.txt', 'r') as f:
        for line in f:
            if line.startswith(target_codename) or target_codename in line.split():
                print(line.strip())
```

## Troubleshooting

### Serial Number Not Appearing

**Possible Causes:**

1. **Recording not enabled**
   - **Solution**: Ensure `--dataout` or `--record-to-stdout` is specified

2. **Device has no serial number**
   - **Cause**: Some devices may not have a `device_serial_number` in their configuration
   - **Solution**: Check device configuration or use codename instead

3. **Configuration parsing failed**
   - **Cause**: Device configuration could not be read or parsed
   - **Solution**: Check device connection and configuration format

4. **Serial number is empty**
   - **Cause**: `device_serial_number` field exists but is empty in config
   - **Solution**: This is expected behavior - empty serial numbers are not recorded

### Verifying Serial Number Extraction

To verify that serial numbers are being extracted correctly, check the log output:

```
Device WM0 serial_number set to: HTCVive_Controller_12345
Device WM0 serial_number at end of config parsing: HTCVive_Controller_12345
Device WM0 serial_number when added to context: HTCVive_Controller_12345
```

If these log messages appear, the serial number was successfully extracted. If the recording still doesn't show the serial number, check that recording is enabled.

### Serial Number Truncation

If a device's serial number is longer than 15 characters, it will be truncated when stored. The recording will show the truncated version. This is a limitation of the 16-byte buffer size.

## Relationship to Other Identifiers

### Codename vs Serial Number

| Aspect | Codename | Serial Number |
|--------|----------|---------------|
| **Format** | `WM0`, `WM1`, `HMD` | Device-specific string |
| **Stability** | May change between sessions | Persistent across sessions |
| **Uniqueness** | Unique per session | Unique per device (hardware) |
| **Source** | Runtime assignment | Device configuration |
| **Length** | Fixed format | Up to 15 characters |
| **Use Case** | Runtime identification | Hardware identification |

### When to Use Each

- **Use Codename**: For runtime tracking, logging, and session-specific operations
- **Use Serial Number**: For device identification across sessions, device management, and hardware tracking

## Related Features

### Configuration Serialization

When `--serialize-device-config` is enabled, device configurations are saved to files named using the serial number:

```
<serial_number>_config.json
```

If no serial number is available, the codename is used instead:

```
<codename>_config.json
```

This allows configurations to be associated with specific hardware devices.

### External Pose Matching

Serial numbers can be used to match external pose data to specific devices. The external pose system can use serial numbers to identify which device a pose belongs to.

## Code Locations

- **Recording Function**: `src/survive_recording.c` (lines 255-263)
- **Header Declaration**: `src/survive_recording.h` (line 41)
- **Recording Trigger**: `src/survive_process.c` (line 303)
- **Configuration Parsing**: `src/survive_default_devices.c` (lines 332-340)
- **Data Structure**: `include/libsurvive/survive.h` (line 138)

## Related Documentation

- `TRACKREF_POSE_RECORDING.md` - Documentation for trackref pose recording
- `architecture.md` - Overall libsurvive architecture
- Device configuration format documentation

