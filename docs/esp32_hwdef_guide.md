# ESP32 Hardware Definition (hwdef) Guide

This guide explains how to add and configure hardware-specific options in the ESP32 build system using hwdef.dat files.

## Overview

The ESP32 hardware definition system allows board-specific configurations to be specified in `hwdef.dat` files. These configurations are processed during the build configure phase and automatically generate appropriate ESP-IDF settings.

## File Location

Each ESP32 board has its own hwdef directory under:
```
libraries/AP_HAL_ESP32/hwdef/<board_name>/hwdef.dat
```

## Supported hwdef Options

### WiFi Configuration

WiFi can be disabled at the board level to optimize dual-core usage:

```
# Disable WiFi for better dual-core performance
define HAL_WITH_WIFI 0
```

When WiFi is disabled, the hwdef system automatically:
- Disables ESP-IDF WiFi components (`CONFIG_ESP_WIFI_ENABLED`)
- Pins the main task to CPU0 (`CONFIG_ESP_MAIN_TASK_AFFINITY_CPU0`)
- Disables WiFi task configurations
- Allows ArduPilot to utilize both cores more effectively

### PSRAM Configuration

For boards with external PSRAM/SPIRAM, you can configure the memory settings:

```
# PSRAM Configuration - Enable 8MB OPI PSRAM
PSRAM_SIZE 8MB
PSRAM_MODE OPI
PSRAM_MALLOC_THRESHOLD 16384
PSRAM_RESERVE_INTERNAL 16384
```

**Available Options:**

- **PSRAM_SIZE**: Amount of PSRAM on the board
  - Values: `0MB`, `2MB`, `4MB`, `8MB`, `16MB`, `32MB`
  - Default: `0MB` (PSRAM disabled)

- **PSRAM_MODE**: PSRAM interface mode
  - Values: `QUAD`, `OPI`
  - `QUAD`: Traditional quad-SPI mode (most common)
  - `OPI`: Octal-SPI mode (higher performance, ESP32-S3 only)

- **PSRAM_MALLOC_THRESHOLD**: Minimum allocation size for PSRAM
  - Values: Size in bytes (e.g., `16384`)
  - Allocations smaller than this use internal RAM
  - Default: `16384` (16KB)

- **PSRAM_RESERVE_INTERNAL**: Amount of internal RAM to reserve
  - Values: Size in bytes (e.g., `16384`)
  - Ensures critical operations have internal RAM available
  - Default: `16384` (16KB)

### Example Configurations

**Board with 8MB OPI PSRAM (like LilyGO T-Connect, T-2CAN):**
```
PSRAM_SIZE 8MB
PSRAM_MODE OPI
PSRAM_MALLOC_THRESHOLD 16384
PSRAM_RESERVE_INTERNAL 16384
```

**Board with 2MB QUAD PSRAM:**
```
PSRAM_SIZE 2MB
PSRAM_MODE QUAD
PSRAM_MALLOC_THRESHOLD 8192
PSRAM_RESERVE_INTERNAL 8192
```

**Board without PSRAM:**
```
# No PSRAM configuration needed - PSRAM disabled by default
```

## How It Works

1. **Configure Phase**: During `./waf configure --board <board_name>`, the hwdef processor (`esp32_hwdef.py`) reads the hwdef.dat file
2. **ESP-IDF Config Generation**: PSRAM directives are converted to ESP-IDF configuration symbols in `sdkconfig.board`
3. **Build Integration**: The build system merges target-level and board-specific configurations into `sdkconfig.combined`
4. **ESP-IDF Build**: ESP-IDF uses the combined configuration to enable PSRAM support

## Generated ESP-IDF Symbols

The hwdef PSRAM configuration automatically generates appropriate ESP-IDF symbols:

```
CONFIG_ESP32S3_SPIRAM_SUPPORT=y
CONFIG_SPIRAM_USE_MALLOC=y
CONFIG_SPIRAM_MODE_OCT=y              # for OPI mode
CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=16384
CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL=16384
CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY=y
```

## Board-Specific Considerations

### Dual CAN Boards (e.g., T-2CAN)
Some ESP32 boards feature dual CAN interfaces using both native TWAI and external CAN controllers (like MCP2515). Currently, ArduPilot only supports the native ESP32 TWAI interface. External CAN controllers would require additional driver implementation.

## Adding New hwdef Options

To add new hwdef options to the ESP32 build system:

1. **Extend the Parser**: Add handling for your new directive in `esp32_hwdef.py`:
   ```python
   elif line.startswith("YOUR_OPTION"):
       parts = line.split()
       if len(parts) >= 2:
           self.your_config[parts[0]] = parts[1]
   ```

2. **Add Configuration Generation**: Extend `generate_esp_idf_config()` to convert your option to ESP-IDF symbols:
   ```python
   if 'YOUR_OPTION' in self.your_config:
       config_lines.append(f"CONFIG_YOUR_SYMBOL={self.your_config['YOUR_OPTION']}")
   ```

3. **Update Documentation**: Add your new option to this guide and any relevant board documentation.

## Debugging Configuration

To verify your hwdef configuration is applied:

1. **Check Generated Files**: After configure, check these files in your build directory:
   - `sdkconfig.board` - Board-specific ESP-IDF configuration
   - `sdkconfig.combined` - Merged target and board configuration
   - `hwdef.h` - Generated ArduPilot header file

2. **Build Output**: Look for hwdef processing messages during configure:
   ```
   Found PSRAM_SIZE: 8MB
   Found PSRAM_MODE: OPI
   Generating ESP-IDF PSRAM configuration
   Writing ESP-IDF config to sdkconfig.board
   ```

3. **ESP-IDF Menuconfig**: You can also inspect the final configuration:
   ```bash
   cd build/<board_name>/esp-idf_build
   idf.py menuconfig
   ```

## Best Practices

1. **Conditional Configuration**: Only specify PSRAM options if your board actually has PSRAM
2. **Conservative Thresholds**: Use reasonable malloc thresholds to ensure system stability
3. **Comment Examples**: Provide commented examples in hwdef.dat for optional features
4. **Validation**: Test your configuration with actual hardware to ensure PSRAM is working correctly

## Troubleshooting

**PSRAM not detected:**
- Verify PSRAM_SIZE matches your board's actual PSRAM
- Check PSRAM_MODE matches your board's interface (QUAD vs OPI)
- Ensure your ESP32-S3 variant supports PSRAM

**Build failures:**
- Check ESP-IDF version compatibility with generated symbols
- Verify sdkconfig.combined contains expected SPIRAM configurations
- Look for symbol name conflicts in ESP-IDF output