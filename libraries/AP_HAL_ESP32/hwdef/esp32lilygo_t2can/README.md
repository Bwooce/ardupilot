# LilyGO T-2CAN Board Support

## Overview

The LilyGO T-2CAN is an ESP32-S3 development board designed for dual CAN bus communication. It features:

- **MCU**: ESP32-S3-WROOM-1U (MCN16R8)
- **Flash**: 16MB
- **PSRAM**: 8MB OPI PSRAM
- **CAN Interfaces**: 
  - Native ESP32-S3 TWAI (Two-Wire Automotive Interface)
  - MCP2515 CAN controller via SPI

## Current ArduPilot Support

### ✅ Supported Features
- **Single CAN Interface**: Native ESP32-S3 TWAI (CAN1)
- **PSRAM**: 8MB OPI PSRAM fully configured
- **WiFi Disabled**: For optimal dual-core performance
- **DroneCAN**: Full DroneCAN protocol support on CAN1
- **Pin Definitions**: All board pins properly defined

### ⚠️ Current Limitations
- **MCP2515 Not Supported**: ArduPilot doesn't currently have MCP2515 driver support
- **Single CAN Only**: Only CAN1 (native TWAI) is functional
- **CAN2 Pins Available**: MCP2515 pins are defined but not used

## Pin Configuration

### CAN Interface 1 (Native ESP32-S3 TWAI)
- **CAN_TX**: GPIO 7
- **CAN_RX**: GPIO 6

### MCP2515 Pins (Available for Future Use)
- **CS**: GPIO 10
- **SCLK**: GPIO 12
- **MOSI**: GPIO 11
- **MISO**: GPIO 13
- **RST**: GPIO 9

### General
- **Boot Button**: GPIO 0

## Usage

### Building Firmware
```bash
./waf configure --board esp32lilygo_t2can
./waf rover  # or copter, plane, etc.
```

### Default Configuration
- **CAN1**: DroneCAN protocol enabled (500 kbps)
- **Frame Type**: Rover (can be changed via parameters)
- **Serial1**: MAVLink1 for telemetry

## Future Development

To fully utilize the T-2CAN's dual CAN capability, the following would need to be implemented:

### Option 1: MCP2515 Driver
- Implement SPI-based MCP2515 driver in ArduPilot
- Add MCP2515 support to ESP32 HAL
- Enable true dual CAN operation

### Option 2: Alternative Pin Usage
- Repurpose MCP2515 pins for other functions:
  - SPI devices (sensors, displays)
  - GPIO expansion
  - Serial interfaces

## Hardware Resources

- **GitHub**: https://github.com/Xinyuan-LilyGO/T-2CAN
- **Example Code**: CAN communication examples available in repository
- **Pinout**: Complete pin definitions in `pin_config.h`

### Serial Interfaces
- **SERIAL0**: USB-Serial/JTAG (GPIO 43 TX, GPIO 44 RX) - Console/MAVLink
- **SERIAL1**: GPIO 1 TX, GPIO 2 RX - Telemetry
- **SERIAL2**: GPIO 17 TX, GPIO 18 RX - GPS or other devices

### I2C
- **SDA**: GPIO 41
- **SCL**: GPIO 42 (400 kHz, internal pullups)

### RC Input (Optional)
RC receiver input is not enabled by default. The ESP32 RMT (Remote Control Transceiver)
peripheral can decode PPM sum signals from a standard RC receiver on a single GPIO pin.
To enable, add `RCIN_PIN <gpio>` to hwdef.dat. Suggested pin: GPIO 14 (free on this board).

### Available GPIOs (not assigned)
GPIO 3, 4, 5, 8, 14, 15, 16, 21, 35-40, 47, 48

## Development Notes

The board definition is structured to support future MCP2515 implementation:
- All MCP2515 pins are defined with proper naming
- Configuration comments explain the dual CAN architecture
- Easy to extend when MCP2515 driver becomes available