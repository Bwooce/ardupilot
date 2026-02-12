/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32

#include <AP_Common/AP_Common.h>
#include <stdint.h>

/*
  ESP32 PSRAM-based logging backend
  
  Uses ESP32's external PSRAM (SPI RAM) for high-speed, wear-free logging.
  The ESP32-S3 typically has 8MB of PSRAM which provides plenty of space
  for in-memory logging without wearing out flash storage.
  
  Features:
  - Zero flash wear
  - Very fast read/write (no filesystem overhead)
  - Circular buffer implementation
  - Automatic old log overwrite when full
  - Survives soft resets (PSRAM content persists)
*/

class ESP32_PSRAM_Logger {
public:
    // Initialize PSRAM logging with specified buffer size
    static bool init(uint32_t buffer_size_mb = 4);
    
    // Check if PSRAM logging is available and healthy
    static bool healthy(void);
    
    // Get total and used space in bytes
    static uint32_t get_total_space(void);
    static uint32_t get_used_space(void);
    static uint32_t get_free_space(void);
    
    // Write data to the log
    static bool write(const uint8_t *data, uint32_t len);
    
    // Read data from the log
    static bool read(uint8_t *data, uint32_t len, uint32_t offset);
    
    // Clear all logs
    static void clear(void);
    
    // Get current write position
    static uint32_t get_write_offset(void);
    
    // Dump logs to MAVLink (for debugging)
    static void dump_to_mavlink(uint32_t start_offset = 0, uint32_t max_bytes = 1024);
    
    // Check if PSRAM is available on this hardware
    static bool psram_available(void);
    
    // Get PSRAM size in bytes
    static uint32_t psram_size(void);

private:
    static uint8_t *buffer_start;
    static uint32_t buffer_size;
    static uint32_t write_offset;
    static uint32_t num_wraps;  // Number of times buffer has wrapped
    static bool initialized;
    
    // Magic number to detect if PSRAM has valid log data after reset
    static const uint32_t PSRAM_LOG_MAGIC = 0x50524C47; // "PRLG"
    
    struct PACKED log_header {
        uint32_t magic;
        uint32_t buffer_size;
        uint32_t write_offset;
        uint32_t num_wraps;
        uint32_t crc32;  // CRC of the header fields
    };
};

#endif // CONFIG_HAL_BOARD == HAL_BOARD_ESP32