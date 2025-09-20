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

#include "ESP32_Params.h"
#include "esp_log.h"
#ifdef CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#include "hal/usb_serial_jtag_ll.h"
#endif
#include <string.h>

namespace ESP32 {

// Constructor - setup parameter defaults
ESP32Params::ESP32Params() {
    // NOTE: Cannot use ESP_LOG or USB serial here - constructor runs before system init!
    // Any output here will crash the system
    AP_Param::setup_object_defaults(this, var_info);
}

// Weak function that can be overridden by vehicle code
ESP32Params* __attribute__((weak)) esp32_params() {
    // For vehicles that don't properly register ESP32 params, use singleton
    return ESP32Params::get_singleton();
}

// Parameter table definition
const struct AP_Param::GroupInfo ESP32Params::var_info[] = {
    // @Param: DEBUG_LVL
    // @DisplayName: ESP32 HAL Debug Level
    // @Description: Controls ESP32 HAL debug output via ESP-IDF logging. 0=Disabled, 1=Errors only, 2=Warnings+Errors, 3=Info+Warnings+Errors, 4=Verbose, 5=All debug output
    // @Values: 0:Disabled, 1:Errors, 2:Warnings, 3:Info, 4:Verbose, 5:All
    // @User: Advanced
    // @RebootRequired: False
    AP_GROUPINFO("DEBUG_LVL", 1, ESP32Params, debug_level, 3), // Default to Info level
    
    // @Param: LOG_MAV
    // @DisplayName: ESP32 Log to MAVLink
    // @Description: Redirect ESP32 debug logs to MAVLink STATUSTEXT messages. Useful for debugging when console is not connected.
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    // @RebootRequired: False
#ifdef ESP32_LOG_MAV_DEFAULT
    AP_GROUPINFO("LOG_MAV", 2, ESP32Params, log_to_mavlink, ESP32_LOG_MAV_DEFAULT),
#else
    AP_GROUPINFO("LOG_MAV", 2, ESP32Params, log_to_mavlink, 0), // Default disabled
#endif
    
    AP_GROUPEND
};

void ESP32Params::init() {
    // Apply ESP-IDF logging level based on parameter
    apply_log_level();
}

// Public method to re-apply log levels (can be called after parameter loading)
void ESP32Params::update_log_levels() {
    apply_log_level();
    // Log status change
    if (log_to_mavlink.get()) {
        ESP_LOGI("ESP32", "MAVLink logging enabled");
    } else {
        ESP_LOGI("ESP32", "MAVLink logging disabled");
    }
}

void ESP32Params::apply_log_level() {
    esp_log_level_t level;
    
    switch (debug_level.get()) {
        case 0:  level = ESP_LOG_ERROR;   break;  // Level 0: Errors only (never fully disable)
        case 1:  level = ESP_LOG_ERROR;   break;  // Level 1: Errors only  
        case 2:  level = ESP_LOG_WARN;    break;  // Level 2: Warnings + Errors
        case 3:  level = ESP_LOG_INFO;    break;  // Level 3: Info + Warnings + Errors
        case 4:  level = ESP_LOG_DEBUG;   break;  // Level 4: Verbose (Debug)
        case 5:  level = ESP_LOG_VERBOSE; break;  // Level 5: All debug output
        default: level = ESP_LOG_INFO;    break;  // Default to Info
    }
    // Remove forced override - respect the configured debug level
    
    // NEVER allow ESP_LOG_NONE to preserve critical logging infrastructure
    esp_log_level_set("HAL_INIT", ESP_LOG_ERROR);	
    
    // Apply to all ESP32 HAL modules
    // UART_DROP messages are too spammy and ESP-IDF doesn't distinguish per-UART
    // Since console drops are expected when debugging is verbose, suppress all UART_DROP
    esp_log_level_set("UART_DROP", ESP_LOG_NONE);  // Always suppress
    esp_log_level_set("UART0", ESP_LOG_NONE);      // Always suppress UART0
    esp_log_level_set("uart", ESP_LOG_NONE);       // Suppress lowercase variant
    
    // Only apply level to non-suppressed UART modules
    if (level > ESP_LOG_WARN) {
        esp_log_level_set("UART_TICK", level);
        esp_log_level_set("UART_WRITE_DATA", level);
        esp_log_level_set("UART_HW", level);
    } else {
        // Keep UART internals quiet unless verbose
        esp_log_level_set("UART_TICK", ESP_LOG_WARN);
        esp_log_level_set("UART_WRITE_DATA", ESP_LOG_WARN);
        esp_log_level_set("UART_HW", ESP_LOG_WARN);
    }
    esp_log_level_set("MUTEX", level);
    esp_log_level_set("ESP32", level);
    esp_log_level_set("HAL", level);
    esp_log_level_set("SCHED", level);
    
    // Important initialization messages should always be visible
    esp_log_level_set("SCHEDULER", ESP_LOG_INFO);
    esp_log_level_set("CONSOLE", ESP_LOG_INFO);
    esp_log_level_set("CANMANAGER", ESP_LOG_INFO);
    esp_log_level_set("GPS", ESP_LOG_INFO);       // GPS detection and status
    esp_log_level_set("GPS_UART", ESP_LOG_INFO);  // GPS UART communication
    esp_log_level_set("STATUSTEXT", ESP_LOG_INFO); // STATUSTEXT corruption debugging
    esp_log_level_set("AP_FS_ESP32", ESP_LOG_INFO); // Filesystem operations debugging
    esp_log_level_set("ESP32_SPIFFS", ESP_LOG_INFO); // SPIFFS operations debugging

    // CAN/DroneCAN debugging - show only real errors
    esp_log_level_set("CAN", ESP_LOG_ERROR);       // Only show CAN errors
    esp_log_level_set("CAN_RX", ESP_LOG_ERROR);    // Only show RX errors
    esp_log_level_set("CAN_TX", ESP_LOG_ERROR);    // Only show TX errors
    esp_log_level_set("CAN_SEND", ESP_LOG_ERROR);  // Only show send queue errors
    esp_log_level_set("DRONECAN", ESP_LOG_ERROR);  // Only critical DroneCAN errors
    esp_log_level_set("CANARD", ESP_LOG_ERROR);    // Only critical Canard errors
    esp_log_level_set("DNA_SERVER", ESP_LOG_ERROR); // Only critical DNA errors
    esp_log_level_set("DNA_HEX", ESP_LOG_INFO); // Enable DNA hex debugging for manual decode

    // Suppress harmless system messages
    esp_log_level_set("system_api", ESP_LOG_NONE);  // eFuse MAC_CUSTOM messages
    
    // Apply global default for any unspecified tags
    esp_log_level_set("*", level);
    
    // Enable GCS parameter debug messages
    esp_log_level_set("GCS_PARAM", ESP_LOG_INFO);
    
    // Reduce storage and parameter debugging 
    esp_log_level_set("STORAGE", ESP_LOG_WARN);
    esp_log_level_set("PARAM", ESP_LOG_WARN);
    
    // MAVLink debugging - keep stats but at INFO level, not ERROR
    esp_log_level_set("MAVLINK", ESP_LOG_INFO);     // Channel stats as info, not errors
    esp_log_level_set("MAVLINK_RX", ESP_LOG_ERROR); // Keep RX errors for corruption detection
    
    // Debug: verify the logging configuration is applied
    printf("ESP32: ESP32_DEBUG_LVL param=%d (configured=%d) -> ESP-IDF level=%d (%s)\n", 
           debug_level.get(),
           debug_level.configured(),
           level, 
           level == ESP_LOG_ERROR ? "ERROR" :
           level == ESP_LOG_WARN ? "WARN" :
           level == ESP_LOG_INFO ? "INFO" :
           level == ESP_LOG_DEBUG ? "DEBUG" :
           level == ESP_LOG_VERBOSE ? "VERBOSE" : "UNKNOWN");
    
    // Always suppress UART_DROP messages regardless of debug level
    // These are too spammy when console buffer is full during verbose logging
    esp_log_level_set("UART_DROP", ESP_LOG_NONE);
}

} // namespace ESP32
