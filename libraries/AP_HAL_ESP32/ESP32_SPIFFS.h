/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32

// Initialize SPIFFS filesystem for internal flash logging
bool esp32_spiffs_init(void);

// Check if SPIFFS is mounted and healthy
bool esp32_spiffs_healthy(void);

// Get available space in bytes
uint32_t esp32_spiffs_available_bytes(void);

// Format SPIFFS (erase all data)
bool esp32_spiffs_format(void);

// List all files in a directory
void esp32_spiffs_list_files(const char* path);

// Delete old log files to free space
uint32_t esp32_spiffs_free_space(uint32_t required_bytes);

// Periodic SPIFFS maintenance
void esp32_spiffs_update(void);

#endif // CONFIG_HAL_BOARD == HAL_BOARD_ESP32