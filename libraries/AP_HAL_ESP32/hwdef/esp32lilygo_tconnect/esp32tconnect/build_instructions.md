# LilyGO T-Connect ArduPilot Rover Build Instructions

## Overview

This configuration enables ArduPilot Rover firmware for the LilyGO T-Connect ESP32-S3 board with support for:
- 2 CAN bus interfaces for DroneCAN/UAVCAN motor control (T-Connect hardware limitation)
- Ackermann steering with single front servo
- DroneCAN motor controllers for 4-wheel drive
- Comprehensive sensor suite (IMU, compass, barometer, GPS)
- Dual battery monitoring
- Extensive I/O capabilities
- WiFi networking support

## Hardware Configuration

### LilyGO T-Connect ESP32-S3 Board Features:
- **MCU**: ESP32-S3-WROOM-1 (Dual-core Xtensa LX7, 240MHz)
- **Flash**: 16MB
- **RAM**: 512KB SRAM + 8MB PSRAM
- **CAN**: 4x CAN transceivers
- **Connectivity**: WiFi 802.11 b/g/n, Bluetooth 5.0
- **I/O**: Multiple UART, I2C, SPI interfaces
- **Power**: Wide input voltage range with power monitoring

### Pin Assignment Summary:

#### CAN Buses (2 interfaces):
- **CAN1**: RX=PA11, TX=PA12 (Primary - Front motors)
- **CAN2**: RX=PB5, TX=PB6 (Secondary - Rear motors)

#### Motor/Servo Outputs:
- **SERVO1**: PA0 (Front Ackermann Steering Servo)
- **SERVO2**: PA1 (Auxiliary - lights, etc.)
- **SERVO3**: PA15 (Auxiliary - gripper, etc.)
- **SERVO4**: PB3 (Auxiliary - camera gimbal, etc.)
- **SERVO5**: PC6 (Auxiliary output)
- **SERVO6**: PC7 (Auxiliary output)
- **SERVO7**: PC8 (Auxiliary output)
- **SERVO8**: PC9 (Auxiliary output)

#### DroneCAN Motor Controllers (2 CAN buses):
- **Front Left Motor**: DroneCAN Node ID 1 (CAN1)
- **Front Right Motor**: DroneCAN Node ID 2 (CAN1)
- **Rear Left Motor**: DroneCAN Node ID 3 (CAN2)
- **Rear Right Motor**: DroneCAN Node ID 4 (CAN2)

#### Communication Interfaces:
- **UART1**: GPS Primary (TX=PA9, RX=PA10)
- **UART2**: Telemetry (TX=PA2, RX=PA3)
- **UART3**: GPS Secondary (TX=PB10, RX=PB11)
- **UART4**: User configurable (TX=PC10, RX=PC11)
- **UART5**: Debug/User (TX=PC12, RX=PD2)

#### Sensors:
- **I2C1**: Primary sensor bus (SCL=PB6, SDA=PB7)
- **I2C2**: External devices (SCL=PB10, SDA=PB11)
- **SPI1**: IMU (SCK=PA5, MISO=PA6, MOSI=PA7, CS=PA4)
- **SPI2**: External devices (SCK=PB13, MISO=PB14, MOSI=PB15)
- **SPI3**: CAN controllers (SCK=PC10, MISO=PC11, MOSI=PC12)

#### Power Monitoring:
- **BATT_VOLTAGE**: PC0 (ADC1)
- **BATT_CURRENT**: PC1 (ADC1)
- **BATT2_VOLTAGE**: PC2 (ADC1)
- **BATT2_CURRENT**: PC3 (ADC1)

## Directory Structure

Create the following directory structure in your ArduPilot source:

```
libraries/AP_HAL_ESP32/hwdef/lilygo-tconnect/
├── hwdef.dat
├── board_config.h
├── wscript
├── defaults.parm
├── partitions.csv
└── BUILD_INSTRUCTIONS.md
```

## Build Process

### Prerequisites:
1. ArduPilot source code
2. ESP32 toolchain (ESP-IDF v4.4+)
3. Python 3.8+
4. Git

### Step 1: Setup Environment

```bash
# Clone ArduPilot repository
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot

# Initialize submodules
git submodule update --init --recursive

# Install ESP32 toolchain
./Tools/environment_install/install-prereqs-ubuntu.sh

# Setup ESP-IDF
export IDF_PATH=$HOME/esp/esp-idf
export PATH=$PATH:$IDF_PATH/tools
```

### Step 2: Configure Board Files

Place all the configuration files in the appropriate directories:

```bash
# Create board directory
mkdir -p libraries/AP_HAL_ESP32/hwdef/lilygo-tconnect/

# Copy configuration files
cp hwdef.dat libraries/AP_HAL_ESP32/hwdef/lilygo-tconnect/
cp board_config.h libraries/AP_HAL_ESP32/hwdef/lilygo-tconnect/
cp wscript libraries/AP_HAL_ESP32/hwdef/lilygo-tconnect/
cp defaults.parm libraries/AP_HAL_ESP32/hwdef/lilygo-tconnect/
cp partitions.csv libraries/AP_HAL_ESP32/hwdef/lilygo-tconnect/
```

### Step 3: Configure Build

```bash
# Configure for ESP32
./waf configure --board=lilygo-tconnect

# Alternative configuration with specific options
./waf configure --board=lilygo-tconnect \
    --enable-can \
    --enable-dronecan \
    --enable-networking \
    --enable-scripting
```

### Step 4: Build Firmware

```bash
# Build rover firmware
./waf rover

# Build with verbose output
./waf rover -v

# Clean build
./waf clean
./waf rover
```

### Step 5: Flash Firmware

```bash
# Flash firmware to board
./waf rover --upload

# Flash with specific port
./waf rover --upload --serial-port=/dev/ttyUSB0

# Manual flash using esptool
python3 -m esptool --chip esp32s3 \
    --port /dev/ttyUSB0 \
    --baud 921600 \
    --before default_reset \
    --after hard_reset \
    write_flash -z \
    --flash_mode dio \
    --flash_freq 80m \
    --flash_size 16MB \
    0x1000 bootloader.bin \
    0x8000 partitions.bin \
    0x10000 ardurover.bin
```

## Configuration and Testing

### Initial Setup:

1. **Connect to Mission Planner/QGroundControl**
2. **Load default parameters** from `defaults.parm`
3. **Calibrate sensors**:
   - Accelerometer calibration
   - Compass calibration
   - Radio calibration
4. **Configure CAN buses**:
   - Set CAN protocols
   - Configure node IDs
   - Test DroneCAN devices

### Ackermann Steering with DroneCAN Motors:

The default configuration supports:
- **Ackermann steering** with single front servo
- **DroneCAN motor control** for all 4 wheels
- **Differential steering assist** from front motors

#### Configuration:
```
SERVO1_FUNCTION = 26  # Front Ackermann Steering
SERVO2_FUNCTION = 0   # Disabled (motors on CAN)
SERVO3_FUNCTION = 0   # Disabled (motors on CAN)
SERVO4_FUNCTION = 0   # Disabled (motors on CAN)
FRAME_TYPE = 2        # Ackermann steering
SKID_STEER_OUT = 0    # Disabled for Ackermann
```

#### DroneCAN Motor Setup:
```
UAVCAN_ENABLE = 3     # Enable DroneCAN
UAVCAN_NODE = 120     # Autopilot node ID
CAN_D1_PROTOCOL = 1   # DroneCAN on CAN1
CAN_D2_PROTOCOL = 1   # DroneCAN on CAN2
```

### DroneCAN Motor Controller Setup:

Each motor controller should be configured with specific node IDs across 2 CAN buses:

1. **Configure ESC Node IDs**:
   - **CAN1** (Front motors):
     - Front Left Motor ESC: Node ID 1
     - Front Right Motor ESC: Node ID 2
   - **CAN2** (Rear motors):
     - Rear Left Motor ESC: Node ID 3
     - Rear Right Motor ESC: Node ID 4

2. **ESC Configuration Parameters**:
   ```
   ESC_INDEX = [1,2,3,4]  # Respective node IDs
   ESC_DIRECTION = [1,0,1,0]  # Forward/reverse for each motor
   ESC_MAX_RPM = 3000     # Maximum RPM
   ESC_MIN_VOLTAGE = 10.0 # Minimum operating voltage
   ```

3. **CAN Bus Distribution**:
   - **CAN1**: Front motor controllers + front sensors
   - **CAN2**: Rear motor controllers + additional devices
   - Load balancing prevents bus saturation

4. **Motor Mixing**:
   - Front motors provide differential steering assist
   - Rear motors provide primary propulsion
   - Ackermann geometry handled by front steering servo

### Sensor Configuration:

The board supports multiple sensor types:
- **IMU**: BMI270 via SPI
- **Compass**: AK09915 via I2C
- **Barometer**: DPS310 via I2C
- **GPS**: Dual GPS support via UART1 and UART3

## Troubleshooting

### Common Issues:

1. **Build Errors**:
   - Ensure ESP-IDF is properly installed
   - Check toolchain version compatibility
   - Verify all submodules are updated

2. **Flash Errors**:
   - Check USB connection
   - Verify correct port selection
   - Try different baud rates
   - Reset board into download mode

3. **DroneCAN Motor Issues**:
   - Verify ESC node ID configuration across both CAN buses
   - Check CAN bus termination resistors (120Ω at each end of each bus)
   - Confirm ESC firmware compatibility with DroneCAN protocol
   - Test individual ESCs with CAN tools
   - Verify power supply to ESCs
   - Monitor CAN bus load (avoid overloading single bus)

4. **Ackermann Steering Issues**:
   - Check servo center position (1500µs)
   - Verify steering geometry parameters
   - Calibrate steering servo limits
   - Test steering response in manual mode

5. **CAN Bus Issues**:
   - Verify CAN transceiver connections on both buses
   - Check termination resistors (120Ω at each end of each bus)
   - Confirm baud rate settings (500kbps default)
   - Test with CAN analyzer on each bus separately
   - Monitor bus utilization to prevent overload

6. **Sensor Initialization**:
   - Check I2C connections
   - Verify sensor addresses
   - Test individual sensors
   - Review boot logs

### Debug Options:

Enable debugging in `hwdef.dat`:
```
define HAL_DEBUG_BUILD 1
define HAL_ENABLE_THREAD_STATISTICS 1
```

Monitor debug output:
```bash
# Monitor serial output
screen /dev/ttyUSB0 115200

# Or using minicom
minicom -D /dev/ttyUSB0 -b 115200
```

## Advanced Configuration

### Custom Modifications:

1. **Additional CAN buses**: Modify SPI CAN controller configuration
2. **Custom sensor integration**: Add new sensor definitions
3. **GPIO expansion**: Configure additional I/O pins
4. **Power management**: Implement custom power control

### Performance Optimization:

1. **Loop rate**: Adjust `SCHED_LOOP_RATE` parameter
2. **Memory usage**: Monitor heap usage
3. **Flash optimization**: Enable size optimization flags
4. **Network performance**: Configure WiFi parameters

### Networking Features:

The board supports:
- **WiFi Access Point mode**
- **WiFi Station mode**
- **Web server interface**
- **Telemetry over WiFi**
- **Parameter configuration via web**

## Support and Resources

### Documentation:
- [ArduPilot Documentation](https://ardupilot.org/rover/)
- [ESP32-S3 Technical Reference](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/)
- [LilyGO T-Connect Documentation](https://github.com/Xinyuan-LilyGO/T-Connect)

### Community:
- [ArduPilot Forums](https://discuss.ardupilot.org/)
- [Discord Server](https://discord.gg/ardupilot)
- [GitHub Issues](https://github.com/ArduPilot/ardupilot/issues)

### Additional Tools:
- [Mission Planner](https://ardupilot.org/planner/)
- [QGroundControl](http://qgroundcontrol.com/)
- [MAVProxy](https://ardupilot.org/mavproxy/)

## License

This configuration is released under the GPL v3 license, same as ArduPilot.