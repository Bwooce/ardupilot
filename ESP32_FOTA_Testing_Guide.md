# ESP32 FOTA (Firmware Over-The-Air) Testing Guide

## Overview

This guide covers testing the ESP32 FOTA implementation that enables firmware updates via standard MAVLink FTP protocol. The system works with existing QGroundControl without requiring any GCS modifications.

## Quick Start

### 1. Build ESP32 Firmware
```bash
./waf configure --board esp32lilygo_tconnect
./waf build
```

### 2. Simple Test (No Upload)
```bash
# Check for firmware files and test connection
python3 test_esp32_ota_simple.py --connection tcp:127.0.0.1:5762

# Just check firmware files
python3 test_esp32_ota_simple.py --check-only
```

### 3. Full FOTA Test
```bash
# Auto-detect firmware and test complete upload
python3 test_esp32_ota.py --connection tcp:127.0.0.1:5762 --firmware auto

# Specify firmware file explicitly  
python3 test_esp32_ota.py --connection serial:/dev/ttyUSB0:460800 --firmware build/esp32lilygo_tconnect/bin/arduplane.bin
```

## Test Scripts

### test_esp32_ota_simple.py
**Purpose**: Basic validation without performing actual upload
- ✅ Finds ESP32 firmware files in build directory
- ✅ Tests MAVLink connection
- ✅ Checks vehicle readiness for OTA
- ✅ Validates ESP32 firmware headers
- ❌ Does NOT perform actual upload

**Use cases**:
- Development validation
- Connection testing
- Firmware file verification
- Pre-flight checks

### test_esp32_ota.py
**Purpose**: Complete FOTA test with actual firmware upload
- ✅ Full firmware upload via MAVLink FTP
- ✅ Vehicle safety validation
- ✅ Progress monitoring
- ✅ Reboot detection
- ✅ Success verification

**Use cases**:
- Integration testing
- End-to-end validation
- Performance testing
- Real-world scenarios

## Connection Types

### SITL (Software-in-the-Loop)
```bash
# Start SITL first
./Tools/autotest/sim_vehicle.py -v ArduPlane --console --map

# Test with SITL
python3 test_esp32_ota.py --connection tcp:127.0.0.1:5762
```

### Hardware Serial
```bash
# ESP32 connected via USB
python3 test_esp32_ota.py --connection serial:/dev/ttyUSB0:460800

# ESP32 with different baud rate
python3 test_esp32_ota.py --connection serial:/dev/ttyACM0:115200
```

### Network Connections
```bash
# TCP connection
python3 test_esp32_ota.py --connection tcp:192.168.1.100:5762

# UDP connection  
python3 test_esp32_ota.py --connection udp:192.168.1.100:14550
```

## Supported ESP32 Boards

The test scripts auto-detect firmware for these boards:
- `esp32lilygo_tconnect` - LilyGo T-Connect (recommended)
- `esp32diy` - DIY ESP32 boards
- `esp32icarus` - Icarus ESP32 board
- `esp32s3` - Generic ESP32-S3 boards

### Adding New Boards
Edit `test_esp32_ota.py` and add to `possible_paths`:
```python
possible_paths = [
    "build/your_board_name/bin/arduplane.bin",
    "build/your_board_name/esp-idf_build/ardupilot.bin",
    # ... existing paths
]
```

## Test Scenarios

### Scenario 1: Basic Validation
```bash
# 1. Check firmware exists
python3 test_esp32_ota_simple.py --check-only

# 2. Test connection
python3 test_esp32_ota_simple.py --connection tcp:127.0.0.1:5762

# Expected: All checks pass, no upload performed
```

### Scenario 2: Development Testing
```bash
# Build firmware
./waf configure --board esp32lilygo_tconnect
./waf build

# Test FOTA upload
python3 test_esp32_ota.py --connection tcp:127.0.0.1:5762 --firmware auto

# Expected: Complete upload simulation with progress reporting
```

### Scenario 3: Hardware Testing
```bash
# Connect ESP32 via USB
python3 test_esp32_ota.py --connection serial:/dev/ttyUSB0:460800 --firmware auto

# Expected: Real firmware upload to hardware, reboot detection
```

### Scenario 4: Error Handling
```bash
# Test with armed vehicle (should fail)
# Test with low battery (should warn/fail)  
# Test with invalid firmware (should fail)
# Test with connection loss (should handle gracefully)
```

## Safety Validations

The FOTA system includes comprehensive safety checks:

### Pre-Update Checks
- ✅ **Vehicle disarmed** - Never update armed vehicles
- ✅ **Battery health** - Requires healthy battery monitoring
- ✅ **Battery capacity** - Requires >50% charge (or no battery for bench testing)
- ✅ **Mission state** - Blocks updates during autonomous missions
- ✅ **GPS status** - Warns if no 3D fix (non-blocking)

### During Update Checks
- ✅ **Power monitoring** - Aborts if battery drops below 25%
- ✅ **Arming detection** - Aborts if vehicle becomes armed
- ✅ **Progress reporting** - Updates every 256KB
- ✅ **Error handling** - Graceful abort on failures

### Post-Update Behavior
- ✅ **Automatic reboot** - 3-second delay for message transmission
- ✅ **Boot verification** - ESP32 hardware rollback on boot failure
- ✅ **USB compatibility** - WAF upload automatically resets partition table

## Troubleshooting

### Common Issues

#### "No firmware files found"
```bash
# Ensure you've built for ESP32
./waf configure --board esp32lilygo_tconnect
./waf build

# Check build directory exists
ls -la build/esp32lilygo_tconnect/
```

#### "Connection failed"
```bash
# Check connection string format
--connection tcp:127.0.0.1:5762        # Correct
--connection tcp://127.0.0.1:5762      # Wrong

# For serial connections, check device exists
ls -la /dev/ttyUSB*
ls -la /dev/ttyACM*

# Check permissions
sudo chmod 666 /dev/ttyUSB0
```

#### "Vehicle not ready for OTA"
```bash
# Check vehicle status
- Ensure vehicle is disarmed
- Check battery level >50%
- Verify not in autonomous mission
- Check MAVLink parameter SERIAL1_PROTOCOL=2 (MAVLink2)
```

#### "FTP command failed"
```bash
# Verify FTP support in firmware
# Check MAVLink message rates
# Ensure stable connection
# Try slower upload (modify chunk timing in script)
```

#### "Upload hangs during transfer"
```bash
# Check network stability
# Verify no MAVLink message flooding
# Try smaller chunk sizes
# Check for memory issues on target
```

### Debug Mode
```bash
# Enable verbose MAVLink debugging
python3 test_esp32_ota.py --connection tcp:127.0.0.1:5762 --verbose

# This shows all MAVLink messages for debugging
```

## Expected Output

### Successful Test
```
ESP32 FOTA Test Starting...
==================================================
Auto-detected firmware: build/esp32lilygo_tconnect/bin/arduplane.bin
Firmware file: build/esp32lilygo_tconnect/bin/arduplane.bin
Firmware size: 1847296 bytes (1.76 MB)
✓ Valid ESP32 firmware header detected
Connecting to tcp:127.0.0.1:5762...
Connected to system 1, component 1
Checking vehicle status...
✓ Vehicle is disarmed
✓ Battery level: 75%

=== Starting FOTA Upload ===
Creating file: /flash/firmware.bin
✓ Firmware file created - OTA routing should be active
Uploading firmware: 1847296 bytes in 239-byte chunks
Upload progress: 10% (184729/1847296 bytes)
Upload progress: 20% (369459/1847296 bytes)
...
Upload progress: 100% (1847296/1847296 bytes)
✓ Firmware upload complete
Terminating FTP session - this should trigger OTA completion and reboot
✓ OTA completion triggered
Vehicle should reboot to new firmware in ~3 seconds...

=== Monitoring Reboot ===
Waiting for connection loss (reboot)...
✓ Connection lost - vehicle rebooting
Waiting for vehicle to come back online...
✓ Vehicle reconnected!
New firmware version: 3 (custom_mode: 0)

==================================================
✓ ESP32 FOTA Test SUCCESSFUL!
Firmware update completed successfully
```

## Integration with QGroundControl

The FOTA system is designed to work with existing QGroundControl:

1. **File Upload via FTP**: Use QGC's file manager to upload firmware to `/flash/firmware.bin`
2. **Automatic Detection**: ArduPilot detects firmware files and routes to OTA
3. **Progress Reporting**: Updates appear in QGC message log  
4. **Standard Protocol**: No QGC modifications required

## Performance Metrics

### Typical Upload Times
- **1.8MB firmware over WiFi**: ~2-3 minutes
- **1.8MB firmware over 460kbps serial**: ~4-6 minutes  
- **1.8MB firmware over ELRS**: ~8-12 minutes

### Network Requirements
- **Minimum bandwidth**: 9600 bps (very slow but functional)
- **Recommended bandwidth**: 57600 bps or higher
- **Optimal bandwidth**: 460800 bps or WiFi
- **Packet loss tolerance**: High (FTP has built-in retry)

## Future Enhancements

Potential improvements for the test framework:
- [ ] Multiple firmware version testing
- [ ] Stress testing with connection loss
- [ ] Performance benchmarking
- [ ] Automated regression testing
- [ ] GUI test interface
- [ ] Integration with CI/CD systems

## Contributing

To contribute improvements to the test scripts:

1. Test with different ESP32 boards
2. Add error condition testing
3. Improve progress reporting
4. Add performance metrics
5. Create additional test scenarios

## Support

For issues with the ESP32 FOTA system:
1. Check this testing guide first
2. Run simple test script for basic validation
3. Enable verbose mode for debugging
4. Report issues with complete test output