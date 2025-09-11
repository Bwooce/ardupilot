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

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32

#include "esp_log.h"
#include "ESP32_Params.h"
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>
#include <stdarg.h>

extern const AP_HAL::HAL& hal;

// Track messages sent (only used in debug builds)
#ifdef DEBUG_BUILD
static uint32_t last_throttle_report = 0;
#endif

// Custom ESP log output function
static int esp_log_to_mavlink(const char *fmt, va_list args)
{
    // Get ESP32 params singleton
    ESP32::ESP32Params* params = ESP32::esp32_params();
    
    // Format the message
    char buffer[256];
    int ret = vsnprintf(buffer, sizeof(buffer), fmt, args);
    
    // Always output to console (default behavior)
    printf("%s", buffer);
    
    // Additionally send to MAVLink if enabled
    if (params && params->log_to_mavlink.get() == 1) {
        // Only send if GCS is available
        if (gcs().get_singleton()) {
            // Remove trailing newline if present
            int len = strlen(buffer);
            if (len > 0 && buffer[len-1] == '\n') {
                buffer[len-1] = '\0';
            }
            
            // Send as STATUSTEXT with severity based on log level
            // Extract log level from the formatted message
            // ESP log format: "E (12345) TAG: message" or "E TAG: message"
            MAV_SEVERITY severity = MAV_SEVERITY_INFO;
            
            // Check for critical keywords that should always be CRITICAL
            if (strstr(buffer, "WDT") || strstr(buffer, "PANIC") || strstr(buffer, "FATAL")) {
                severity = MAV_SEVERITY_CRITICAL;
            } else if (strstr(buffer, "eFuse MAC_CUSTOM")) {
                // Harmless ESP-IDF message about custom MAC not being set
                severity = MAV_SEVERITY_DEBUG;
            } else if (strstr(buffer, "MAVLINK") && (strstr(buffer, "Ch0") || strstr(buffer, "Ch1") || 
                                                      strstr(buffer, "totals") || strstr(buffer, "upd="))) {
                // MAVLINK channel stats should be INFO level per user request
                severity = MAV_SEVERITY_INFO;
            } else if (buffer[0] == 'E' && (buffer[1] == ' ' || buffer[1] == '(')) {
                // ESP_LOGE messages should be CRITICAL for visibility
                severity = MAV_SEVERITY_CRITICAL;
            } else if (buffer[0] == 'W' && (buffer[1] == ' ' || buffer[1] == '(')) {
                severity = MAV_SEVERITY_WARNING;
            } else if (buffer[0] == 'I' && (buffer[1] == ' ' || buffer[1] == '(')) {
                severity = MAV_SEVERITY_INFO;
            } else if (buffer[0] == 'D' && (buffer[1] == ' ' || buffer[1] == '(')) {
                severity = MAV_SEVERITY_DEBUG;
            } else if (buffer[0] == 'V' && (buffer[1] == ' ' || buffer[1] == '(')) {
                severity = MAV_SEVERITY_DEBUG;
            }
            
            // Send to all GCS channels (may be throttled/dropped)
            gcs().send_text(severity, "%s", buffer);
            
            // Count messages for internal statistics only
            static uint32_t total_sent = 0;
            total_sent++;
            
            // Only report in debug builds if there's an issue
            #ifdef DEBUG_BUILD
            uint32_t now = AP_HAL::millis();
            if (total_sent > 1000 && now - last_throttle_report > 60000) {
                // Only warn if excessive logging detected (>1000 msgs/min)
                ESP_LOGW("ESP32", "High log rate: %lu msgs/min", 
                        (unsigned long)total_sent);
                last_throttle_report = now;
                total_sent = 0;
            }
            #endif
        }
    }
    
    return ret;
}

// Initialize ESP log redirection
void esp32_log_redirect_init()
{
    // Set custom output function
    esp_log_set_vprintf(esp_log_to_mavlink);
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_ESP32