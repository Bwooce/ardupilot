/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  ESP32 Over-The-Air (OTA) firmware update header
  Phase 0: Safety Framework for FOTA
*/

#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32

#include <stdint.h>
#include <stddef.h>

/**
 * Validate conditions are safe for firmware update
 * Checks:
 * - Vehicle not armed
 * - Battery healthy and >50% capacity
 * - Not in autonomous mission
 * - GPS fix available (warning only)
 * 
 * @return true if conditions are safe for OTA update
 */
bool validate_update_conditions();

/**
 * Check power safety during update and abort if necessary
 * Called periodically during OTA write operations
 * Aborts update if:
 * - Battery drops below 25%
 * - Battery failsafe triggered
 * - Vehicle becomes armed
 */
void check_power_safety_during_update();

/**
 * Check if file path indicates a firmware file
 * 
 * @param path File path to check
 * @return true if path indicates firmware file
 */
bool is_firmware_file(const char* path);

/**
 * Validate ESP32 firmware header
 * 
 * @param data Firmware data buffer
 * @param len Length of data
 * @return true if header is valid ESP32 firmware
 */
bool validate_esp32_firmware_header(const uint8_t* data, size_t len);

/**
 * Begin ESP32 OTA update
 * 
 * @param filename Name of firmware file being updated
 * @return true if OTA began successfully
 */
bool esp32_ota_begin(const char* filename);

/**
 * Write data to OTA partition
 * 
 * @param data Data buffer to write
 * @param len Length of data
 * @param offset File offset (for progress tracking)
 * @return true if write successful
 */
bool esp32_ota_write(const uint8_t* data, size_t len, uint32_t offset);

/**
 * Complete OTA update and reboot to new firmware
 * 
 * @return true if completion successful (function may not return)
 */
bool esp32_ota_end();

/**
 * Abort OTA update
 */
void esp32_ota_abort();

/**
 * Get number of bytes written to OTA partition
 * 
 * @return bytes written
 */
size_t esp32_ota_get_bytes_written();

/**
 * Check if OTA update is currently in progress
 * 
 * @return true if OTA active
 */
bool esp32_ota_is_in_progress();

/**
 * Emergency factory reset - force boot from factory partition
 * Used for recovery from boot-loop situations
 * 
 * @return true if reset successful
 */
bool esp32_ota_factory_reset();

#endif // CONFIG_HAL_BOARD == HAL_BOARD_ESP32