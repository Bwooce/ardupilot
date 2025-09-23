# ESP32 Logging Architecture

## CRITICAL: NEVER Use SPIFFS for Logging

**SPIFFS filesystem logging is permanently disabled on ESP32 due to architectural incompatibility.**

### The Problem

Every SPIFFS write operation causes watchdog timeout crashes due to:

1. **SPI Flash Cache Synchronization**: SPIFFS writes require disabling the SPI flash cache on BOTH CPU cores
2. **Cross-Core IPC Deadlock**: The synchronization mechanism sends an Inter-Processor Call (IPC) that frequently fails
3. **Watchdog Timeout**: When CPU synchronization takes >5 seconds, the watchdog fires and reboots the system

### Technical Details

The deadlock occurs in `spi_flash_disable_interrupts_caches_and_other_cpu()` at cache_utils.c:176. This function:
- Sends IPC to the other CPU to disable its cache
- Waits for acknowledgment
- Frequently times out when the other CPU is busy
- Cannot be interrupted or recovered

### Solution: PSRAM-Only Logging

ESP32 ArduPilot uses **PSRAM (SPI RAM) exclusively** for logging:

- **PSRAM Logger**: Primary and only logging backend
- **Zero Flash Wear**: No flash writes during normal operation
- **High Performance**: No cache synchronization needed
- **4MB Buffer**: Circular buffer in external RAM

### OTA Updates Still Work

SPIFFS remains mounted at `/APM` for:
- OTA firmware updates
- One-time configuration storage
- **NOT for continuous logging**

### Configuration

In hwdef.dat:
```
define HAL_ESP32_USE_PSRAM_LOGGING 1        # Enable PSRAM logger
define HAL_LOGGING_FILESYSTEM_ENABLED 0     # NEVER enable this
define HAL_LOGGING_MAVLINK_ENABLED 1        # Fallback if no PSRAM
```

### Board Requirements

- **MUST have PSRAM**: ESP32-S3 with 8MB PSRAM recommended
- **SD Card logging**: Safe on boards with SD card (no SPIFFS deadlock)
- **MAVLink fallback**: If PSRAM unavailable, logs via telemetry

### Partition Table

The partition named `ota_storage` (formerly `spiffs` or `logs`) is **ONLY** for OTA updates.

### Never Attempt To:

- Enable `HAL_LOGGING_FILESYSTEM_ENABLED` on ESP32
- Use AP_Logger_File backend on ESP32
- Create SPIFFS-based logging solutions
- Mount SPIFFS at high-frequency write paths

This is a **hardware limitation** of the ESP32's dual-core architecture and cannot be fixed in software.