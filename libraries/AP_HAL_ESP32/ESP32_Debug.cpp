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

#include "ESP32_Debug.h"
#include <stdio.h>

#if HAL_ESP32_DEBUG_AVAILABLE

namespace ESP32 {

ESP32Debug* esp32_debug() {
    return ESP32Debug::get_singleton();
}

void ESP32Debug::debug_print(uint8_t level, const char* prefix, const char* fmt, va_list args) {
    // Check level threshold
    if (level > _debug_level) {
        return;
    }
    
    // Format the message with ESP32 prefix
    char buffer[200];  // MAVLink STATUSTEXT max is ~250 chars
    int prefix_len = snprintf(buffer, sizeof(buffer), "ESP32-%s: ", prefix);
    
    // Add the actual message  
    vsnprintf(buffer + prefix_len, sizeof(buffer) - prefix_len, fmt, args);
    
    // Map debug levels to MAVLink severity levels
    MAV_SEVERITY severity;
    switch (level) {
        case ESP32_DEBUG_ERRORS:
            severity = MAV_SEVERITY_ERROR;
            break;
        case ESP32_DEBUG_WARNINGS:
            severity = MAV_SEVERITY_WARNING;
            break;
        case ESP32_DEBUG_INFO:
            severity = MAV_SEVERITY_INFO;
            break;
        case ESP32_DEBUG_VERBOSE:
        case ESP32_DEBUG_ALL:
        default:
            severity = MAV_SEVERITY_DEBUG;
            break;
    }
    
    // Send via ArduPilot's MAVLink system
    GCS_SEND_TEXT(severity, "%s", buffer);
}

void ESP32Debug::error(const char* fmt, ...) {
    if (_debug_level >= ESP32_DEBUG_ERRORS) {
        va_list args;
        va_start(args, fmt);
        debug_print(ESP32_DEBUG_ERRORS, "ERROR", fmt, args);
        va_end(args);
    }
}

void ESP32Debug::warning(const char* fmt, ...) {
    if (_debug_level >= ESP32_DEBUG_WARNINGS) {
        va_list args;
        va_start(args, fmt);
        debug_print(ESP32_DEBUG_WARNINGS, "WARN", fmt, args);
        va_end(args);
    }
}

void ESP32Debug::info(const char* fmt, ...) {
    if (_debug_level >= ESP32_DEBUG_INFO) {
        va_list args;
        va_start(args, fmt);
        debug_print(ESP32_DEBUG_INFO, "INFO", fmt, args);
        va_end(args);
    }
}

void ESP32Debug::verbose(const char* fmt, ...) {
    if (_debug_level >= ESP32_DEBUG_VERBOSE) {
        va_list args;
        va_start(args, fmt);
        debug_print(ESP32_DEBUG_VERBOSE, "VERBOSE", fmt, args);
        va_end(args);
    }
}

void ESP32Debug::debug(const char* fmt, ...) {
    if (_debug_level >= ESP32_DEBUG_ALL) {
        va_list args;
        va_start(args, fmt);
        debug_print(ESP32_DEBUG_ALL, "DEBUG", fmt, args);
        va_end(args);
    }
}

} // namespace ESP32

#endif // HAL_ESP32_DEBUG_AVAILABLE