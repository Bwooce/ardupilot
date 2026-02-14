# LilyGO T-Connect Board Support

## Overview

The LilyGO T-Connect is an ESP32-S3 development board designed as a DroneCAN hub for rover applications. It features:

- **MCU**: ESP32-S3
- **Flash**: 16MB with OTA support
- **PSRAM**: 8MB OPI PSRAM
- **CAN Interface**: Native ESP32-S3 TWAI (single bus)

## Pin Configuration

### CAN Interface (Native ESP32-S3 TWAI)
- **CAN_TX**: GPIO 9
- **CAN_RX**: GPIO 10

### Motor Outputs
- **RCOUT1**: GPIO 13 (right motor)
- **RCOUT2**: GPIO 14 (left motor)

### APA102 LED
- **DATA**: GPIO 8
- **CLOCK**: GPIO 3

### Serial Interfaces
- **SERIAL0**: USB-Serial/JTAG (GPIO 1 TX, GPIO 2 RX) - Console/MAVLink
- **SERIAL1**: GPIO 12 TX, GPIO 46 RX - CRSF/ELRS (57600 baud)
- **SERIAL2**: GPIO 43 TX, GPIO 44 RX - GPS (230 baud for u-blox config)

### I2C
- **SDA**: GPIO 41
- **SCL**: GPIO 42 (400 kHz, internal pullups)

### RC Input (Optional)
RC receiver input is not enabled by default. The ESP32 RMT (Remote Control Transceiver)
peripheral can decode PPM sum signals from a standard RC receiver on a single GPIO pin.
To enable, add `RCIN_PIN <gpio>` to hwdef.dat. Suggested pin: GPIO 7 (free on this board).
Note: GPIO 14 (previously suggested) conflicts with RCOUT1.

### Available GPIOs (not assigned)
GPIO 4, 5, 6, 7, 11, 15, 16, 17, 18, 21, 35-40, 47, 48

## Default Configuration
- **CAN1**: DroneCAN protocol enabled (500 kbps)
- **Frame Type**: Rover
- **Logging**: PSRAM-based (4MB, zero flash wear)
- **Filesystem**: SPIFFS (8MB, for OTA updates)
- **WiFi**: Disabled by default (can be enabled in hwdef.dat)

## Building Firmware
```bash
./waf configure --board esp32lilygo_tconnect
./waf rover
```

## Hardware Resources

- **GitHub**: https://github.com/Xinyuan-LilyGO/T-Connect
