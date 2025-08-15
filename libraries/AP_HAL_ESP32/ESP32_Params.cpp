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

namespace ESP32 {

ESP32Params* esp32_params() {
    return ESP32Params::get_singleton();
}

// Parameter table definition
const struct AP_Param::GroupInfo ESP32Params::var_info[] = {
    // @Param: DEBUG_LEVEL
    // @DisplayName: ESP32 HAL Debug Level
    // @Description: Controls ESP32 HAL debug output via ESP-IDF logging. 0=Disabled, 1=Errors only, 2=Warnings+Errors, 3=Info+Warnings+Errors, 4=Verbose, 5=All debug output
    // @Values: 0:Disabled, 1:Errors, 2:Warnings, 3:Info, 4:Verbose, 5:All
    // @User: Advanced
    // @RebootRequired: False
    AP_GROUPINFO("DEBUG_LEVEL", 1, ESP32Params, debug_level, 3), // Default to Info level
    
    AP_GROUPEND
};

void ESP32Params::init() {
    printf("ESP32: ESP32Params::init() called, debug_level=%d (configured=%d)\n", 
           debug_level.get(), debug_level.configured());
    // Apply ESP-IDF logging level based on parameter
    apply_log_level();
}

// Public method to re-apply log levels (can be called after parameter loading)
void ESP32Params::update_log_levels() {
    apply_log_level();
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
    
    // NEVER allow ESP_LOG_NONE to preserve critical logging infrastructure
    
    // Apply to all ESP32 HAL modules
    esp_log_level_set("UART_DROP", level);
    esp_log_level_set("UART_TICK", level);
    esp_log_level_set("UART_WRITE_DATA", level);
    esp_log_level_set("UART_HW", level);
    esp_log_level_set("MUTEX", level);
    esp_log_level_set("ESP32", level);
    esp_log_level_set("HAL", level);
    esp_log_level_set("SCHED", level);
    
    // CAN/DroneCAN debugging - use INFO level minimum for visibility, or higher if set
    esp_log_level_t can_level = (level < ESP_LOG_INFO) ? ESP_LOG_INFO : level;
    esp_log_level_set("CAN", can_level);
    esp_log_level_set("CAN_RX", can_level);
    esp_log_level_set("CAN_TX", can_level);
    esp_log_level_set("DRONECAN", can_level);
    
    // Apply global default for any unspecified tags
    esp_log_level_set("*", level);
    
    // CRITICAL: Force MAVLink corruption detection to work regardless of debug level
    // Even if ESP32_DEBUG_LEVEL=0 (ESP_LOG_NONE), we need corruption detection
    esp_log_level_set("MAVLINK", ESP_LOG_ERROR);
    esp_log_level_set("MAVLINK_RX", ESP_LOG_ERROR);
    
    // Debug: verify the logging configuration is applied and investigate parameter source
    printf("ESP32: ESP32_DEBUG_LEVEL param=%d (configured=%d) -> ESP-IDF level=%d (%s), CAN level=%d (%s), MAVLINK=ERROR\n", 
           debug_level.get(),
           debug_level.configured(),
           level, 
           level == ESP_LOG_ERROR ? "ERROR" :
           level == ESP_LOG_WARN ? "WARN" :
           level == ESP_LOG_INFO ? "INFO" :
           level == ESP_LOG_DEBUG ? "DEBUG" :
           level == ESP_LOG_VERBOSE ? "VERBOSE" : "UNKNOWN",
           can_level,
           can_level == ESP_LOG_ERROR ? "ERROR" :
           can_level == ESP_LOG_WARN ? "WARN" :
           can_level == ESP_LOG_INFO ? "INFO" :
           can_level == ESP_LOG_DEBUG ? "DEBUG" :
           can_level == ESP_LOG_VERBOSE ? "VERBOSE" : "UNKNOWN");
    
    // Test if ESP-IDF logging actually works after our configuration
    ESP_LOGE("MAVLINK", "ESP-IDF logging test after configuration - you should see this!");
    printf("ESP32: If you see this printf but not the ESP_LOGE above, ESP-IDF logging is broken\n");
}

} // namespace ESP32