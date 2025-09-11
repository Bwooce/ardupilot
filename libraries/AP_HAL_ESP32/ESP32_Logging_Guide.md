# ESP32 Logging and Console Systems Guide

The ESP32 HAL has multiple logging and console systems that serve different purposes and use different underlying mechanisms. Understanding these is crucial for debugging and development.

## Console and Logging Architecture

### 1. ESP-IDF Native Logging System
**Purpose**: System-level logging from ESP-IDF and early boot messages  
**Output**: USB CDC console (native ESP32 USB interface)  
**Functions**: `ESP_LOGE()`, `ESP_LOGW()`, `ESP_LOGI()`, `ESP_LOGD()`, `ESP_LOGV()`  
**Availability**: Always available, even during early boot  
**Configuration**: Via ESP-IDF menuconfig, cannot be disabled by ArduPilot  

**Example**:
```cpp
ESP_LOGI("TAG", "System initialized with %d interfaces", count);
ESP_LOGW("TAG", "Warning: Low memory %d bytes", free_mem);
ESP_LOGE("TAG", "Critical error: %s", error_msg);
```

**Output format**:
```
.[0;32mI (260) TAG: System initialized with 2 interfaces.[0m
.[0;33mW (565) TAG: Warning: Low memory 1024 bytes.[0m
.[0;31mE (1024) TAG: Critical error: Init failed.[0m
```

### 2. ArduPilot Console System (hal.console)
**Purpose**: ArduPilot application logging and interactive console  
**Output**: UART0 via USB-Serial/JTAG driver  
**Functions**: `hal.console->printf()`, `hal.console->write()`  
**Availability**: Only after ArduPilot UART initialization  
**Configuration**: Can be disabled via `HAL_CONSOLE_ENABLED 0`  

**Example**:
```cpp
hal.console->printf("ArduPilot: System ready, mode %d\n", mode);
```

**Interactive Features**:
- Parameter access: `param show PARAM_NAME`
- CLI commands when enabled
- MAVLink message display
- Real-time telemetry output

### 3. Standard C printf()
**Purpose**: Simple debug output  
**Output**: Goes to ArduPilot console (UART0)  
**Functions**: `printf()`, `snprintf()`  
**Availability**: After console initialization  
**Configuration**: Affected by `HAL_CONSOLE_ENABLED` setting  

**Example**:
```cpp
printf("Debug: Value = %d\n", debug_value);
```

### 4. ESP32 Debug Macros
**Purpose**: ArduPilot-specific debug output with level control  
**Output**: ArduPilot console (UART0)  
**Functions**: `ESP32_DEBUG_VERBOSE()`, `ESP32_DEBUG_INFO()`, etc.  
**Availability**: After console initialization  
**Configuration**: Controlled by debug level defines  

**Example**:
```cpp
ESP32_DEBUG_VERBOSE("UART%d: Sent %d bytes", uart_num, count);
ESP32_DEBUG_INFO("System: Initialization complete");
```

## Physical Interfaces

### UART0 - ArduPilot Console
- **Hardware**: USB-Serial/JTAG interface on ESP32-S3
- **Driver**: `usb_serial_jtag_driver` 
- **Purpose**: ArduPilot console, debug output, interactive CLI
- **Pins**: GPIO 43 (TX), GPIO 44 (RX) - but USB CDC, not physical pins
- **Protocol**: USB CDC ACM (appears as /dev/ttyACM0 on Linux)

### UART1 - MAVLink Telemetry  
- **Hardware**: Physical UART pins
- **Driver**: Standard ESP32 `uart_driver`
- **Purpose**: MAVLink telemetry to ground station
- **Pins**: Configurable via hwdef.dat (e.g., TX=12, RX=46)
- **Protocol**: Standard UART with configurable baud rate

### USB CDC Native Console
- **Hardware**: ESP32-S3 native USB OTG interface
- **Driver**: ESP-IDF built-in USB CDC
- **Purpose**: ESP-IDF system messages only
- **Pins**: USB D+/D- pins (hardware level)
- **Protocol**: Raw USB CDC

## Boot Sequence and Logging Availability

### Phase 1: ESP-IDF Boot (0-500ms)
- **Available**: ESP-IDF logging only (`ESP_LOG*`)
- **Output**: USB CDC native console
- **Messages**: Core system initialization, hardware detection

### Phase 2: ArduPilot HAL Init (500ms-2s) 
- **Available**: ESP-IDF logging + printf() 
- **Output**: Both USB CDC and UART0 (if initialized)
- **Messages**: HAL component initialization

### Phase 3: ArduPilot Application (2s+)
- **Available**: All logging methods
- **Output**: All interfaces operational  
- **Messages**: Vehicle-specific initialization, MAVLink startup

## Debugging Guidelines

### Early Boot Issues (Before "SCHEDULER: All tasks created")
**Use**: `ESP_LOG*` functions only
```cpp
ESP_LOGI("BOOT", "Checking hardware configuration");
ESP_LOGE("BOOT", "Failed to initialize component: %s", component);
```

### ArduPilot Initialization Issues  
**Use**: `ESP_LOG*` or `printf()` 
```cpp
ESP_LOGI("ARDUPILOT", "Starting vehicle initialization");
printf("Debug: Parameter count = %d\n", param_count);
```

### Runtime Application Issues
**Use**: ArduPilot console or ESP32 debug macros
```cpp  
hal.console->printf("Status: Armed=%d, Mode=%d\n", armed, mode);
ESP32_DEBUG_INFO("Telemetry: Sent heartbeat to GCS");
```

### MAVLink Communication Issues
**Critical**: Never use MAVLink-related logging from MAVLink context
```cpp
// WRONG - causes recursion:
// GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Debug message"); 

// CORRECT - use ESP-IDF logging:
ESP_LOGI("MAVLINK", "Sent packet: ID=%d, len=%d", msg_id, len);
```

## Configuration Options

### hwdef.dat Settings
```bash
# Disable ArduPilot console to prevent MAVLink interference
define HAL_CONSOLE_ENABLED 0

# Enable console for debugging (may interfere with MAVLink)  
define HAL_CONSOLE_ENABLED 1

# Control debug output levels
define DEBUG_LEVEL 2
```

### ESP-IDF menuconfig Settings
- Component config → Log output → Default log verbosity
- Component config → ESP System Settings → Console output

## Best Practices

### 1. Choose the Right Logging Method
- **System/Hardware issues**: Use `ESP_LOG*` 
- **ArduPilot application**: Use `hal.console->printf()`
- **Performance debugging**: Use ESP32 debug macros
- **MAVLink issues**: Always use `ESP_LOG*` to avoid recursion

### 2. Avoid Logging Recursion
```cpp
// DANGEROUS - can cause deadlock:
void uart_write() {
    hal.console->printf("Writing data\n");  // Uses same UART!
    // ... actual write
}

// SAFE - uses different output:
void uart_write() {
    ESP_LOGI("UART", "Writing data");  // Uses ESP-IDF logging
    // ... actual write  
}
```

### 3. Performance Considerations
- `ESP_LOG*` functions are fast and safe
- `hal.console->printf()` can block on full buffers
- Excessive logging can affect real-time performance

### 4. Production vs Development
```cpp
#ifdef DEBUG_BUILD
    hal.console->printf("Debug: Processing %d items\n", count);
#endif

// Always available, can be filtered by level:
ESP_LOGD("MODULE", "Processing %d items", count);  // Debug level only
ESP_LOGI("MODULE", "System ready");  // Always shown
```

## Troubleshooting Common Issues

### "No console output from hal.console->printf()"
1. Check `HAL_CONSOLE_ENABLED` setting in hwdef.dat
2. Verify UART0 initialization succeeded
3. Use `ESP_LOG*` functions instead for early debugging

### "MAVLink stops working after adding debug messages"
1. Remove any `GCS_SEND_TEXT()` calls from UART/MAVLink context
2. Replace with `ESP_LOG*` functions  
3. Disable `HAL_CONSOLE_ENABLED` to prevent interference

### "Boot hangs with no output"
1. Use `ESP_LOG*` functions only during early boot
2. Check for infinite loops in static constructors
3. Monitor ESP-IDF logging for crash information

### "Inconsistent debug output"
1. Verify logging levels are set correctly
2. Check buffer sizes for console UART
3. Consider timing issues with multiple logging systems