# ESP32 Board Migration Guide: Legacy to hwdef.dat

This document outlines the steps to migrate an ESP32 board configuration from the legacy static header system to the modern `hwdef.dat` configuration system.

## Scope
The migration covers:
*   **Pin Mappings:** All GPIO assignments for Serial, SPI, I2C, RCOUT, ADC, and RCInput.
*   **Sensor Probing:** Macro-based discovery logic for IMUs, Barometers, and Compasses.
*   **System Config:** Transitioning from manual `sdkconfig` edits to automated generation.
*   **Storage/Partitions:** Flash partition table definitions and filesystem setup.

## Prerequisites
*   ArduPilot development environment (Waf, ESP-IDF v5.5+).
*   The legacy C++ header for the board (e.g., `libraries/AP_HAL_ESP32/boards/myboard.h`).
*   Familiarity with standard ArduPilot `hwdef.dat` syntax.

## Migration Steps

### 1. Create Board Directory
Each board now requires its own directory under `hwdef/`:
```bash
mkdir -p libraries/AP_HAL_ESP32/hwdef/<board_name>
```

### 2. Initialize hwdef.dat
Create `libraries/AP_HAL_ESP32/hwdef/<board_name>/hwdef.dat`. 
Define the target MCU at the top:
```bash
MCU ESP32S3  # Options: ESP32, ESP32S2, ESP32S3, ESP32C3, ESP32P4
```

### 3. Translate Pin Assignments
Translate `#define` statements from your old header to `hwdef.dat` syntax.

| Function | Old Define (C++) | New Syntax (hwdef.dat) |
|----------|-------------------|------------------------|
| UART TX  | `HAL_ESP32_UART1_TX` | `SERIAL1_TX_PIN 18` |
| SPI SCK  | `HAL_ESP32_SPI1_SCK` | `SPI1_SCK_PIN 35` |
| RCOUT    | `HAL_ESP32_RCOUT1`   | `RCOUT1_PIN 11` |
| ADC      | `HAL_ESP32_ADC1`      | `ADC1_PIN 1` |

### 4. Translate Sensor Probing
The refactored build system uses standard probe helpers to minimize boilerplate.

**Example (SPI IMU):**
*   **Old:** `#define HAL_INS_PROBE_LIST ADD_BACKEND(AP_InertialSensor_Invensense::probe(*this,hal.spi->get_device("mpu9250"),ROTATION_NONE))`
*   **New:** `define HAL_INS_PROBE_LIST PROBE_IMU_SPI(Invensense, "mpu9250", ROTATION_NONE)`

### 5. Manage ESP-IDF Settings
The build system now generates `sdkconfig` automatically.
*   **Defaults:** Base settings are pulled from `libraries/AP_HAL_ESP32/targets/<mcu>/esp-idf/sdkconfig.defaults`.
*   **Board Overrides:** If you need specific IDF settings (e.g., PSRAM tweaks), create `sdkconfig.board` in your board directory.
*   **Flash Size:** Use `FLASH_SIZE_MB 8` in `hwdef.dat` to set the flash size.

### 6. Handle Flash Partitions
If your board requires a non-standard partition layout:
1.  Place your `partitions.csv` in the board directory.
2.  Add to `hwdef.dat`: `PARTITION_TABLE_CUSTOM_FILENAME partitions.csv`

### 7. Custom Pin Restrictions
Use the new `RESERVED_PINS` directive to block pins physically used by your board's internal flash/PSRAM that aren't covered by standard module defaults:
```bash
RESERVED_PINS 33 34 35 36 37  # For Octal PSRAM boards
```

### 8. Verification
1.  Configure the board: `./waf configure --board=<board_name>`
2.  Inspect the generated `build/hwdef.h` to ensure all defines are present.
3.  Compile: `./waf copter`.

## Support for Hybrid Transition
The build system supports both legacy and refactored boards during the migration period. A board is detected as "New Way" if it has an `hwdef.dat` file. Otherwise, it defaults to looking for a legacy header.
