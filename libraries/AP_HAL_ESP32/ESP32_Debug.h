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

#if HAL_GCS_ENABLED
    #include <GCS_MAVLink/GCS.h>
    #define HAL_ESP32_DEBUG_AVAILABLE 1
#else
    #define HAL_ESP32_DEBUG_AVAILABLE 0
#endif

/*
  ESP32 HAL Debug System
  
  Integrates with ArduPilot's existing GCS_SEND_TEXT() system for 
  MAVLink-based debug output. Provides runtime-configurable debug
  levels to avoid serial port contamination.
  
  Usage:
    ESP32_DEBUG(level, "UART%d: buffer size %d", uart_num, size);
    ESP32_DEBUG_INFO("Scheduler initialized");  
    ESP32_DEBUG_WARNING("Failed to create task");
    ESP32_DEBUG_ERROR("Critical initialization failure");
*/

namespace ESP32 {

// Debug levels matching ArduPilot conventions  
enum ESP32DebugLevel {
    ESP32_DEBUG_DISABLED = 0,
    ESP32_DEBUG_ERRORS   = 1,    // Only critical errors
    ESP32_DEBUG_WARNINGS = 2,    // Warnings and errors  
    ESP32_DEBUG_INFO     = 3,    // General information
    ESP32_DEBUG_VERBOSE  = 4,    // Detailed debug info
    ESP32_DEBUG_ALL      = 5     // Everything including spam
};

class ESP32Debug {
public:
    // Get singleton instance
    static ESP32Debug* get_singleton() {
        static ESP32Debug instance;
        return &instance;
    }
    
    // Set debug level (0-5, matches enum above)
    void set_level(uint8_t level) { _debug_level = level; }
    uint8_t get_level() const { return _debug_level; }
    
    // Main debug output function
    void debug_print(uint8_t level, const char* prefix, const char* fmt, va_list args);
    
    // Convenience functions for different levels
    void error(const char* fmt, ...);
    void warning(const char* fmt, ...); 
    void info(const char* fmt, ...);
    void verbose(const char* fmt, ...);
    void debug(const char* fmt, ...);

private:
    uint8_t _debug_level = ESP32_DEBUG_INFO;  // Default to INFO level
    
    // Prevent direct construction - use singleton
    ESP32Debug() = default;
    ESP32Debug(const ESP32Debug&) = delete;
    ESP32Debug& operator=(const ESP32Debug&) = delete;
};

// Global access function
ESP32Debug* esp32_debug();

} // namespace ESP32

// Macro definitions for convenient usage
#if HAL_ESP32_DEBUG_AVAILABLE

// Generic debug macro with level checking
#define ESP32_DEBUG(level, fmt, args...) do { \
    if (ESP32::esp32_debug()->get_level() >= level) { \
        ESP32::esp32_debug()->debug(fmt, ##args); \
    } \
} while(0)

// Severity-specific macros (always check level internally)
#define ESP32_DEBUG_ERROR(fmt, args...)   ESP32::esp32_debug()->error(fmt, ##args)  
#define ESP32_DEBUG_WARNING(fmt, args...) ESP32::esp32_debug()->warning(fmt, ##args)
#define ESP32_DEBUG_INFO(fmt, args...)    ESP32::esp32_debug()->info(fmt, ##args)
#define ESP32_DEBUG_VERBOSE(fmt, args...) ESP32::esp32_debug()->verbose(fmt, ##args)  
#define ESP32_DEBUG_DEBUG(fmt, args...)   ESP32::esp32_debug()->debug(fmt, ##args)

#else

// No-op macros when GCS not available (e.g., AP_Periph builds)
#define ESP32_DEBUG(level, fmt, args...)   do { } while(0)
#define ESP32_DEBUG_ERROR(fmt, args...)    do { } while(0)
#define ESP32_DEBUG_WARNING(fmt, args...)  do { } while(0) 
#define ESP32_DEBUG_INFO(fmt, args...)     do { } while(0)
#define ESP32_DEBUG_VERBOSE(fmt, args...)  do { } while(0)
#define ESP32_DEBUG_DEBUG(fmt, args...)    do { } while(0)

#endif // HAL_ESP32_DEBUG_AVAILABLE