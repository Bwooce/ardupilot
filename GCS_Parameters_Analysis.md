# Ground Control Station Mandatory Parameters Analysis

## Overview
Analysis of mandatory and expected parameters that QGroundControl and other Ground Control Stations expect from ArduPilot Rover systems, focusing on parameter compatibility issues and missing component warnings.

## Common QGroundControl Error Scenarios

### 1. "Parameters are missing from firmware"
**Root Causes:**
- QGroundControl parameter database expects components not present in minimal builds
- Version mismatch between QGroundControl and ArduPilot firmware
- Custom builds with reduced feature sets

### 2. "Can't received full set of parameters"
**Root Causes:**
- Poor telemetry link quality affecting parameter download
- Parameter stream rate too low (SR0_PARAMS)
- MAVLink protocol conflicts or timing issues

### 3. Component-Specific Parameter Expectations
QGroundControl expects certain parameters to exist even when components are disabled, leading to warnings and reduced functionality.

## Mandatory Parameter Categories for GCS Compatibility

### 1. Vehicle Identification Parameters
```
FRAME_CLASS = 2           # Vehicle type (2=Rover) - REQUIRED
FRAME_TYPE = 0            # Frame subtype (0=Generic rover)
```
**GCS Impact:** Determines available setup screens and configuration options.

### 2. Battery Monitoring Parameters
```
BATT_MONITOR = 0|3|4|8    # Battery monitor type (0=Disabled, 3=Voltage, 4=Volt+Current, 8=DroneCAN)
BATT_LOW_VOLT = 10.5      # Low voltage threshold
BATT_CRT_VOLT = 10.0      # Critical voltage threshold
BATT_FS_LOW_ACT = 0       # Low battery failsafe action
BATT_FS_CRT_ACT = 0       # Critical battery failsafe action
```
**GCS Impact:** Battery setup screen, failsafe configuration, power monitoring display.

### 3. Safety/Failsafe Parameters
```
FS_GCS_ENABL = 0          # GCS failsafe enable
FS_THR_ENABLE = 0         # Throttle failsafe enable  
FS_THR_VALUE = 910        # Throttle failsafe PWM value
ARMING_CHECK = 1          # Pre-arm safety checks bitmask
```
**GCS Impact:** Safety setup screen functionality, arming checks display.

### 4. Serial Protocol Configuration
```
SERIAL0_PROTOCOL = 2      # MAVLink2 on USB (required for GCS communication)
SERIAL1_PROTOCOL = 2      # MAVLink2 on telemetry port
SERIAL2_PROTOCOL = 5      # GPS protocol
```
**GCS Impact:** Communication establishment, telemetry link configuration.

### 5. Sensor Availability Parameters
Even when sensors are disabled, GCS expects these parameters to exist:

```
# Compass parameters (even if disabled)
COMPASS_USE = 0           # Compass enable/disable
COMPASS_ENABLE = 0        # Compass availability

# Barometer parameters  
BARO_TYPE = 0             # Barometer type (0=Disabled)

# Airspeed parameters
ARSPD_TYPE = 0            # Airspeed sensor type (0=Disabled)

# Rangefinder parameters
RNGFND1_TYPE = 0          # Rangefinder type (0=Disabled)
```
**GCS Impact:** Prevents "missing parameters" warnings, enables proper setup screen behavior.

## Component-Specific GCS Requirements

### QGroundControl Setup Screens Expected Parameters

#### Power Setup Screen
- `BATT_MONITOR` (any valid type)
- `BATT_VOLT_*` parameters for voltage monitoring
- `BATT_CURR_*` parameters for current monitoring
- Battery failsafe thresholds

#### Safety Setup Screen  
- `FS_GCS_ENABL` for GCS failsafe
- `FS_THR_ENABLE` for throttle failsafe
- `ARMING_CHECK` bitmask
- Fence parameters (if AP_FENCE_ENABLED)

#### Airframe Setup Screen
- `FRAME_CLASS` and `FRAME_TYPE` 
- Note: Full airframe setup only available for Copter/Sub, not Rover

#### Sensor Setup Screens
- Compass calibration expects `COMPASS_*` parameters
- GPS setup expects `GPS_TYPE*` parameters
- Accelerometer calibration expects INS parameters

## Parameter Stream Configuration

### Critical for GCS Parameter Download
```
SR0_PARAMS = 10           # Parameter download rate (Hz) - keep fast for setup
```
**Impact:** Slow parameter downloads, setup timeouts, "can't receive full set" errors.

## Missing Component Parameter Solutions

### Explicit Disable Strategy
Instead of omitting parameters, explicitly set disable values:

```
# Prevent missing compass warnings
COMPASS_USE = 0
COMPASS_ENABLE = 0  
COMPASS_LEARN = 0

# Prevent missing sensor warnings
BARO_TYPE = 0
ARSPD_TYPE = 0  
RNGFND1_TYPE = 0
RNGFND2_TYPE = 0

# Prevent missing advanced feature warnings
TERRAIN_ENABLE = 0
FENCE_ENABLE = 0
```

### Parameter Database Compatibility
Ensure parameter names match QGroundControl's expected parameter database for the ArduPilot version in use.

## GCS-Specific Compatibility Notes

### QGroundControl
- Expects complete parameter set for vehicle type
- Shows warnings for missing expected parameters
- Disables setup screens when required parameters absent
- Parameter download can fail with incomplete sets

### Mission Planner  
- More tolerant of missing parameters
- Better handles custom/minimal builds
- Provides more detailed parameter information

### MAVProxy
- Command-line interface less affected by missing parameters
- Direct parameter access regardless of completeness

## Troubleshooting Parameter Issues

### Diagnostic Steps
1. Check `SR0_PARAMS` rate (should be ≥10 Hz for setup)
2. Verify telemetry link quality during parameter download
3. Compare parameter list against vehicle type expectations
4. Check ArduPilot version compatibility with GCS version

### Common Fixes
1. Add explicit disable parameters for unused components
2. Increase parameter stream rate for downloads  
3. Improve telemetry link reliability
4. Update GCS to match ArduPilot version

## Conclusion

GCS parameter compatibility issues primarily stem from:
1. **Missing parameters for disabled components** - solved with explicit disable parameters
2. **Incomplete parameter downloads** - solved with adequate SR0_PARAMS rate and link quality
3. **Version mismatches** - solved with compatible GCS and firmware versions

The key is providing explicit parameter values for all expected components, even when disabled, rather than omitting them entirely.