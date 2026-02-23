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
    AP_GROUPINFO("DEBUG_LVL", 1, ESP32Params, debug_level, ESP32_DEFAULT_LOG_LEVEL),
    
    // @Param: LOG_MAV
    // @DisplayName: ESP32 Log to MAVLink
    // @Description: Redirect ESP32 debug logs to MAVLink STATUSTEXT messages. Useful for debugging when console is not connected.
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
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
        case 0:  level = ESP_LOG_ERROR;   break;
        case 1:  level = ESP_LOG_ERROR;   break;
        case 2:  level = ESP_LOG_WARN;    break;
        case 3:  level = ESP_LOG_INFO;    break;
        case 4:  level = ESP_LOG_DEBUG;   break;
        case 5:  level = ESP_LOG_VERBOSE; break;
        default: level = ESP_LOG_INFO;    break;
    }

    // Set global default FIRST -- all tags inherit this
    esp_log_level_set("*", level);

    // Suppress known-spammy tags regardless of global level
    esp_log_level_set("UART_DROP", ESP_LOG_NONE);
    esp_log_level_set("UART_FLOW", ESP_LOG_NONE);

    // UART internals only at DEBUG or above (very high frequency)
    if (level < ESP_LOG_DEBUG) {
        esp_log_level_set("UART_TICK", ESP_LOG_WARN);
        esp_log_level_set("UART_WRITE_DATA", ESP_LOG_WARN);
    }

    // Per-module overrides from hwdef.dat (define ESP32_LOG_LEVEL_<MODULE> <0-5>)
    // These use the same 0-5 scale as ESP32_DEFAULT_LOG_LEVEL.
    // Only modules explicitly defined in hwdef.dat get overridden.
#ifdef ESP32_LOG_LEVEL_CAN
    {
        esp_log_level_t can_level = (esp_log_level_t)ESP32_LOG_LEVEL_CAN;
        esp_log_level_set("CAN", can_level);
        esp_log_level_set("TWAI_RX", can_level);
        esp_log_level_set("TWAI_TX", can_level);
        esp_log_level_set("CAN_HEALTH", can_level);
    }
#endif
#ifdef ESP32_LOG_LEVEL_UART
    {
        esp_log_level_t uart_level = (esp_log_level_t)ESP32_LOG_LEVEL_UART;
        esp_log_level_set("UART_TICK", uart_level);
        esp_log_level_set("UART_WRITE_DATA", uart_level);
    }
#endif
#ifdef ESP32_LOG_LEVEL_SCHEDULER
    {
        esp_log_level_t sched_level = (esp_log_level_t)ESP32_LOG_LEVEL_SCHEDULER;
        esp_log_level_set("SCHEDULER", sched_level);
        esp_log_level_set("MAIN", sched_level);
        esp_log_level_set("WDT", sched_level);
        esp_log_level_set("SCHED_WDT", sched_level);
        esp_log_level_set("ESP32_CPU", sched_level);
    }
#endif
#ifdef ESP32_LOG_LEVEL_STORAGE
    esp_log_level_set("STORAGE", (esp_log_level_t)ESP32_LOG_LEVEL_STORAGE);
#endif
#ifdef ESP32_LOG_LEVEL_OTA
    {
        esp_log_level_t ota_level = (esp_log_level_t)ESP32_LOG_LEVEL_OTA;
        esp_log_level_set("OTA", ota_level);
        esp_log_level_set("OTA_FTP", ota_level);
    }
#endif

    ESP_LOGI("ESP32", "Log level: %d (%s)",
             level,
             level == ESP_LOG_ERROR   ? "ERROR" :
             level == ESP_LOG_WARN    ? "WARN" :
             level == ESP_LOG_INFO    ? "INFO" :
             level == ESP_LOG_DEBUG   ? "DEBUG" :
             level == ESP_LOG_VERBOSE ? "VERBOSE" : "?");
}

} // namespace ESP32
