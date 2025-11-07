# Trackref Pose Recording Documentation

## Overview

This document describes the implementation of lighthouse pose recording in **trackref space** (tracker reference coordinate system). Trackref space represents the original tracker geometry frame, where sensor positions are defined in the device's configuration. This provides a consistent, tracker-specific coordinate system that is independent of IMU calibration or world coordinate transformations.

Two types of trackref pose recordings are available:
1. **Normal Solve Recording** (`LH_POSE_TRACKREF`) - Records poses during regular tracking and single-tracker calibration
2. **Global Scene Solve Recording** (`LH_POSE_TRACKREF_GSS`) - Records poses during multi-tracker global scene optimization

## What is Trackref Space?

**Trackref space** is the coordinate system defined by the tracker's sensor geometry:
- Origin: Defined by the tracker's sensor layout (typically at the tracker's physical center)
- Axes: Fixed orientation based on the tracker's sensor configuration
- Consistency: Same coordinate system for the same tracker type across all runs
- Independence: Not affected by IMU calibration or world coordinate system setup

The transformation from IMU space to trackref space (`imu2trackref`) is a fixed, per-tracker transformation loaded from the device configuration. This transformation is applied to convert poses from the optimizer's IMU space to the tracker's native geometry frame.

## Data Recorded

For each tracker-lighthouse combination, the following data is recorded:

- **Tracker codename** (e.g., `WM0`, `WM1`)
- **Lighthouse index** (0, 1, 2, etc.)
- **Lighthouse pose in trackref space**:
  - Position: `[x, y, z]` (3D coordinates)
  - Rotation: `[w, x, y, z]` (quaternion in wxyz format)
- **Base station ID** (unique identifier for the lighthouse)

The recorded pose represents **lighthouse-to-tracker** (`lh2trackref`), meaning the pose of the lighthouse relative to the tracker in the tracker's trackref coordinate system.

## Output Format

### Normal Solve Recording (`LH_POSE_TRACKREF`)

**Format:**
```
<tracker_codename> LH_POSE_TRACKREF <lighthouse_index> <pos_x> <pos_y> <pos_z> <rot_w> <rot_x> <rot_y> <rot_z> <basestation_id>\r\n
```

**Example:**
```
WM0 LH_POSE_TRACKREF 0 1.234 -0.567 2.345 0.707 0.0 0.0 0.707 12345678
```

**Fields:**
- `tracker_codename`: Device identifier (e.g., `WM0`, `WM1`)
- `lighthouse_index`: Lighthouse index (0, 1, 2, ...)
- `pos_x`, `pos_y`, `pos_z`: Position coordinates (meters)
- `rot_w`, `rot_x`, `rot_y`, `rot_z`: Quaternion rotation (wxyz format)
- `basestation_id`: Base station unique identifier (hexadecimal)

### Global Scene Solve Recording (`LH_POSE_TRACKREF_GSS`)

**Format:**
```
<tracker_codename> LH_POSE_TRACKREF_GSS <lighthouse_index> <pos_x> <pos_y> <pos_z> <rot_w> <rot_x> <rot_y> <rot_z> <basestation_id>\r\n
```

**Example:**
```
WM0 LH_POSE_TRACKREF_GSS 0 1.234 -0.567 2.345 0.707 0.0 0.0 0.707 12345678
WM1 LH_POSE_TRACKREF_GSS 0 1.234 -0.567 2.345 0.707 0.0 0.0 0.707 12345678
```

**Fields:** Same as normal solve recording, but with `LH_POSE_TRACKREF_GSS` tag to distinguish global scene solver results.

**Note:** The `_GSS` suffix indicates this pose was calculated during a global scene optimization that solves for multiple trackers simultaneously.

## How to Enable Recording

Trackref pose recording is controlled by a single configuration flag that enables both normal and global scene recordings:

### Command Line

```bash
survive-cli --record-lh-trackref
```

### Configuration File

Add to your configuration:
```
record-lh-trackref = 1
```

### Programmatic

The recording is automatically enabled when:
1. The `record-lh-trackref` config flag is set to `true` (or `1`)
2. Recording is enabled in the SurviveContext
3. The tracker has valid `imu2trackref` transformation loaded

## When Recordings Occur

### Normal Solve Recording (`LH_POSE_TRACKREF`)

Recorded in three scenarios:

1. **During MPFIT Optimization** (`poser_mpfit.c:550-561`)
   - When: After successful MPFIT optimization during calibration
   - Frequency: Once per optimization run when lighthouse poses are solved
   - Condition: Only when `canPossiblySolveLHS` is true and optimization succeeds

2. **During Single Lighthouse Calibration** (`poser.c:164-171`)
   - When: During initial lighthouse pose calculation
   - Frequency: Once per lighthouse when pose is first established
   - Condition: When lighthouse pose is calculated in arbitrary/IMU space

3. **During Normal Operation** (`survive_process.c:65-85`)
   - When: Every time a pose update occurs (same frequency as `POSE` recordings)
   - Frequency: Same as raw pose updates (typically every frame)
   - Condition: Only for lighthouses where `PositionSet` is true (calibrated lighthouses)

### Global Scene Solve Recording (`LH_POSE_TRACKREF_GSS`)

Recorded in one scenario:

1. **During Global Scene Optimization** (`poser_mpfit.c:1014-1047`)
   - When: After successful global scene optimization that solves for multiple trackers
   - Frequency: Once per global scene optimization run
   - Condition: Only when optimization succeeds and results are valid
   - Scope: Records poses for **all tracker-lighthouse combinations** in a single optimization run

## Transformation Chain

The transformation from optimizer results to trackref space follows this chain:

### Normal Solve

1. **Optimizer Output**: `opt_cameras[i]` = `object2lh` (in IMU space)
2. **Invert**: `lh2imu` = `InvertPoseRtn(opt_cameras[i])`
3. **Transform to Trackref**: `lh2trackref` = `imu2trackref * lh2imu`
4. **Normalize**: Quaternion normalization to prevent drift
5. **Record**: Output `LH_POSE_TRACKREF`

### Normal Operation (After Calibration)

1. **World Space**: `lh2world` (from `ctx->bsd[i].Pose`)
2. **Get Object Pose**: `imu2world` (from `so->OutPoseIMU`)
3. **Invert**: `world2imu` = `InvertPoseRtn(imu2world)`
4. **Transform**: `lh2imu` = `world2imu * lh2world`
5. **Transform to Trackref**: `lh2trackref` = `imu2trackref * lh2imu`
6. **Normalize**: Quaternion normalization
7. **Record**: Output `LH_POSE_TRACKREF`

### Global Scene Solve

1. **Optimizer Output**: 
   - `opt_poses[s]` = `imu2world` for tracker `s`
   - `opt_cameras[i]` = `world2lh` for lighthouse `i`
2. **Invert Camera**: `cameras[i]` = `lh2world` = `InvertPoseRtn(opt_cameras[i])`
3. **For Each Tracker-Lighthouse Pair**:
   - `world2imu` = `InvertPoseRtn(opt_poses[s])`
   - `lh2imu` = `world2imu * lh2world`
   - `lh2trackref` = `imu2trackref * lh2imu`
   - Normalize quaternion
   - Record: Output `LH_POSE_TRACKREF_GSS`

## Implementation Details

### Key Functions

1. **`survive_recording_lighthouse_trackref_process`** (`survive_recording.c:278-289`)
   - Records normal solve trackref poses
   - Outputs `LH_POSE_TRACKREF` tag
   - Checks `writeLHTrackref` config flag

2. **`survive_recording_lighthouse_trackref_global_process`** (`survive_recording.c:290-301`)
   - Records global scene solve trackref poses
   - Outputs `LH_POSE_TRACKREF_GSS` tag
   - Checks same `writeLHTrackref` config flag

### Coordinate System Notes

- **Quaternion Format**: All quaternions use **wxyz** format (scalar `w` first, then vector `x, y, z`)
- **Angle Units**: All angles are in **radians** (quaternions don't directly use angles, but Euler conversions use radians)
- **Pose Convention**: `A2B` means "pose of B in A's coordinate system" (transformation from A to B)
- **IMU Space**: The optimizer works in IMU space because `sensor_locations` are stored relative to the IMU

### Per-Tracker Transformation

The `imu2trackref` transformation is:
- **Per-tracker**: Each tracker has its own `imu2trackref` transformation
- **Fixed**: Loaded from device configuration, doesn't change during runtime
- **Identity by default**: If not specified in config, defaults to identity quaternion `[1, 0, 0, 0]`
- **Location**: Stored in `SurviveObject->imu2trackref`

## Usage Examples

### Parsing Recorded Data

**Python Example:**
```python
import re

def parse_trackref_line(line):
    # Pattern: WM0 LH_POSE_TRACKREF 0 1.234 -0.567 2.345 0.707 0.0 0.0 0.707 12345678
    pattern = r'(\w+)\s+LH_POSE_TRACKREF(_GSS)?\s+(\d+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+(\d+)'
    match = re.match(pattern, line)
    if match:
        tracker, is_gss, lh_idx, px, py, pz, qw, qx, qy, qz, bs_id = match.groups()
        return {
            'tracker': tracker,
            'is_global': is_gss == '_GSS',
            'lighthouse': int(lh_idx),
            'position': [float(px), float(py), float(pz)],
            'rotation': [float(qw), float(qx), float(qy), float(qz)],  # wxyz
            'basestation_id': int(bs_id)
        }
    return None
```

### Filtering by Type

```python
# Read only normal solve recordings
normal_poses = []
with open('recording.txt', 'r') as f:
    for line in f:
        if 'LH_POSE_TRACKREF ' in line and '_GSS' not in line:
            pose = parse_trackref_line(line)
            if pose:
                normal_poses.append(pose)

# Read only global scene solve recordings
global_poses = []
with open('recording.txt', 'r') as f:
    for line in f:
        if 'LH_POSE_TRACKREF_GSS' in line:
            pose = parse_trackref_line(line)
            if pose:
                global_poses.append(pose)
```

## Differences Between Normal and Global Recordings

| Aspect | Normal Solve (`LH_POSE_TRACKREF`) | Global Scene Solve (`LH_POSE_TRACKREF_GSS`) |
|--------|-----------------------------------|----------------------------------------------|
| **When** | During single-tracker optimization or normal operation | During multi-tracker global optimization |
| **Frequency** | Every frame (normal op) or per optimization (calibration) | Once per global scene optimization |
| **Scope** | One tracker at a time | All trackers simultaneously |
| **Use Case** | Regular tracking, single-tracker calibration | Multi-tracker calibration, global scene refinement |
| **Tag** | `LH_POSE_TRACKREF` | `LH_POSE_TRACKREF_GSS` |
| **Coordinate System** | Same (trackref space) | Same (trackref space) |

## Troubleshooting

### No Recordings Appearing

1. **Check config flag**: Ensure `--record-lh-trackref` is enabled
2. **Check recording enabled**: Verify recording is enabled in SurviveContext
3. **Check lighthouse calibration**: Normal operation recordings only appear after lighthouse calibration (`PositionSet` must be true)
4. **Check verbose output**: Use `--verbose 200` to see `imu2trackref` transformation logs

### Inconsistent Poses

- **Per-tracker transformation**: Each tracker has its own `imu2trackref`, so poses are tracker-specific
- **Coordinate system**: Trackref space is fixed per tracker type, but may differ between tracker types
- **Normalization**: Quaternions are normalized after transformation to prevent drift

### Distinguishing Record Types

- Look for `LH_POSE_TRACKREF_GSS` tag for global scene recordings
- Look for `LH_POSE_TRACKREF` (without `_GSS`) for normal recordings
- Global scene recordings typically appear in batches (all trackers at once)
- Normal recordings appear continuously during operation

## Related Documentation

- `QUATERNION_FORMAT_REPORT.md` - Details on quaternion format (wxyz) used throughout libsurvive
- `architecture.md` - Overall libsurvive architecture
- `writing_a_poser.md` - How to write custom posers

## Code Locations

- **Recording Functions**: `src/survive_recording.c` (lines 278-301)
- **Normal Solve Recording**: `src/poser_mpfit.c` (lines 545-562), `src/poser.c` (lines 164-171)
- **Normal Operation Recording**: `src/survive_process.c` (lines 65-85)
- **Global Scene Recording**: `src/poser_mpfit.c` (lines 1014-1047)
- **Header Declarations**: `src/survive_recording.h` (lines 44-45)

