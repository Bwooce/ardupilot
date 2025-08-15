/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include "esp_log.h"

/*
  ESP32 HAL Debug System
  
  Uses ESP-IDF native logging for reliable output even during early boot
  and when ArduPilot console system is not functional.
  
  Usage:
    ESP32_DEBUG_INFO("Scheduler initialized");  
    ESP32_DEBUG_WARNING("Failed to create task");
    ESP32_DEBUG_ERROR("Critical initialization failure");
*/

// Macro definitions using ESP-IDF logging system
// These work even when ArduPilot console is broken

#define ESP32_DEBUG_ERROR(fmt, args...)   ESP_LOGE("ESP32", fmt, ##args)  
#define ESP32_DEBUG_WARNING(fmt, args...) ESP_LOGW("ESP32", fmt, ##args)
#define ESP32_DEBUG_INFO(fmt, args...)    ESP_LOGI("ESP32", fmt, ##args)
#define ESP32_DEBUG_VERBOSE(fmt, args...) ESP_LOGD("ESP32", fmt, ##args)  
#define ESP32_DEBUG_DEBUG(fmt, args...)   ESP_LOGV("ESP32", fmt, ##args)

// TEMPORARY: Disable DEV_PRINTF to prevent console contamination
#ifdef DEV_PRINTF
#undef DEV_PRINTF
#endif
#define DEV_PRINTF(fmt, args...) ESP_LOGI("DEV_BYPASS", fmt, ##args)

// Generic debug macro with level checking (maps to ESP-IDF levels)
#define ESP32_DEBUG(level, fmt, args...) do { \
    switch(level) { \
        case 1: ESP_LOGE("ESP32", fmt, ##args); break; \
        case 2: ESP_LOGW("ESP32", fmt, ##args); break; \
        case 3: ESP_LOGI("ESP32", fmt, ##args); break; \
        case 4: ESP_LOGD("ESP32", fmt, ##args); break; \
        case 5: ESP_LOGV("ESP32", fmt, ##args); break; \
        default: break; \
    } \
} while(0)
