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
  ESP32 Over-The-Air (OTA) firmware update implementation
  Phase 0: Safety Framework for FOTA
*/

#include "ESP32_OTA.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Arming/AP_Arming.h>
#include <GCS_MAVLink/GCS.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32

#include "esp_ota_ops.h"
#include "esp_log.h"

extern const AP_HAL::HAL& hal;

// OTA state variables
static const esp_partition_t* _update_partition = nullptr;
static esp_ota_handle_t _ota_handle = 0;
static size_t _bytes_written = 0;

bool validate_update_conditions() {
    // Never update when armed
    if (hal.util->get_soft_armed()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "OTA blocked: vehicle armed");
        return false;
    }
    
    // Check battery health and capacity (if battery is present)
    AP_BattMonitor &battery = AP::battery();
    float current_voltage = battery.voltage();
    
    // If no battery voltage detected, allow OTA (some vehicles don't have batteries)
    if (current_voltage <= 0.1f) {
        gcs().send_text(MAV_SEVERITY_INFO, "OTA: No battery detected - proceeding");
        // Continue with other checks
    } else {
        // Battery is present, check health and capacity
        if (!battery.healthy()) {
            gcs().send_text(MAV_SEVERITY_WARNING, "OTA blocked: battery unhealthy");
            return false;
        }
        
        // Check remaining capacity if available (more reliable than voltage)
        uint8_t remaining_pct;
        if (battery.capacity_remaining_pct(remaining_pct)) {
            if (remaining_pct < 50) {  // Need 50% minimum for update + safety margin
                gcs().send_text(MAV_SEVERITY_WARNING, 
                    "OTA blocked: battery %d%%, >50%% required", remaining_pct);
                return false;
            }
        } else {
            // No capacity monitoring - just warn about voltage but don't block
            gcs().send_text(MAV_SEVERITY_INFO, 
                "OTA: Battery voltage %.1fV - no capacity monitoring available", current_voltage);
        }
    }
    
    // Check GPS fix for position hold capability (optional but recommended)
    AP_GPS &gps = AP::gps();
    if (gps.status() < AP_GPS::GPS_OK_FIX_3D) {
        gcs().send_text(MAV_SEVERITY_INFO, 
            "OTA warning: no GPS 3D fix - proceed with caution");
        // Don't block OTA for GPS issues, just warn
    }
    
    // Check if vehicle is in an autonomous mission (block OTA during missions)
    // Note: This is a basic check - vehicle-specific implementations should override
    // if they have mission systems
    #if AP_MISSION_ENABLED
    const AP_Mission *mission = AP::mission();
    if (mission && mission->state() == AP_Mission::MISSION_RUNNING) {
        gcs().send_text(MAV_SEVERITY_WARNING, "OTA blocked: mission in progress");
        return false;
    }
    #endif
    
    return true;
}

void check_power_safety_during_update() {
    AP_BattMonitor &battery = AP::battery();
    
    // Only check battery if one is present (voltage > 0.1V)
    if (battery.voltage() > 0.1f) {
        // Abort if power drops during update
        uint8_t remaining_pct;
        if (battery.capacity_remaining_pct(remaining_pct)) {
            if (remaining_pct < 25) {  // Emergency abort threshold
                esp32_ota_abort();
                gcs().send_text(MAV_SEVERITY_CRITICAL, 
                    "OTA ABORTED: battery critical %d%%", remaining_pct);
                return;
            }
        }
        
        // Check for battery failsafe conditions
        if (battery.has_failsafed()) {
            esp32_ota_abort();
            gcs().send_text(MAV_SEVERITY_CRITICAL, "OTA ABORTED: power failsafe");
            return;
        }
    }
    
    // Check if vehicle has been armed (emergency abort)
    if (hal.util->get_soft_armed()) {
        esp32_ota_abort();
        gcs().send_text(MAV_SEVERITY_CRITICAL, "OTA ABORTED: vehicle armed");
        return;
    }
}

bool is_firmware_file(const char* path) {
    if (path == nullptr) {
        return false;
    }
    
    // Path-based detection for firmware files
    return (strstr(path, "/flash/") ||         // Special firmware directory
            strstr(path, "firmware.bin") ||     // Standard firmware name
            strstr(path, "ardupilot.bin") ||    // ArduPilot naming convention
            strstr(path, "/ota/"));             // OTA directory
}

bool validate_esp32_firmware_header(const uint8_t* data, size_t len) {
    if (data == nullptr || len < 4) {
        return false;
    }
    
    // ESP32 firmware starts with specific boot header magic number
    if (data[0] != 0xE9) {
        gcs().send_text(MAV_SEVERITY_ERROR, 
            "OTA: Invalid ESP32 firmware header (magic: 0x%02X, expected: 0xE9)", data[0]);
        return false;
    }
    
    // Additional basic validation can be added here
    // For now, just check the magic number
    return true;
}

bool esp32_ota_begin(const char* filename) {
    if (filename == nullptr) {
        return false;
    }
    
    // Validate pre-conditions before starting OTA
    if (!validate_update_conditions()) {
        return false;
    }
    
    // Check if we're in a boot-loop recovery scenario
    esp_ota_img_states_t ota_state;
    const esp_partition_t* running = esp_ota_get_running_partition();
    if (running && esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            gcs().send_text(MAV_SEVERITY_WARNING, 
                "OTA: Current firmware pending verification - may indicate boot issues");
            gcs().send_text(MAV_SEVERITY_INFO, 
                "OTA: Proceeding anyway - new update will replace current firmware");
        }
    }
    
    // Get the next available OTA partition
    _update_partition = esp_ota_get_next_update_partition(NULL);
    if (!_update_partition) {
        gcs().send_text(MAV_SEVERITY_ERROR, "OTA: No available update partition");
        return false;
    }
    
    // Begin OTA update
    esp_err_t err = esp_ota_begin(_update_partition, OTA_SIZE_UNKNOWN, &_ota_handle);
    if (err != ESP_OK) {
        gcs().send_text(MAV_SEVERITY_ERROR, "OTA: Failed to begin update (%s)", esp_err_to_name(err));
        return false;
    }
    
    _bytes_written = 0;
    
    gcs().send_text(MAV_SEVERITY_INFO, "OTA: Starting firmware update: %s", filename);
    gcs().send_text(MAV_SEVERITY_INFO, "OTA: Target partition: %s (0x%lx)", 
                   _update_partition->label, _update_partition->address);
    
    return true;
}

bool esp32_ota_write(const uint8_t* data, size_t len, uint32_t offset) {
    if (data == nullptr || len == 0 || _ota_handle == 0) {
        return false;
    }
    
    // ESP32 OTA requires sequential writes from offset 0
    // Handle non-sequential writes (resume scenarios) by restarting OTA
    if (offset != _bytes_written) {
        if (offset < _bytes_written) {
            // Retransmission of already-written data - skip it
            gcs().send_text(MAV_SEVERITY_DEBUG, "OTA: Skipping retransmitted data at offset %lu (already have %zu bytes)", 
                           (unsigned long)offset, _bytes_written);
            return true;
        } else {
            // Gap in data - ESP32 OTA can't handle this, would need to restart from beginning
            gcs().send_text(MAV_SEVERITY_ERROR, "OTA: Gap in data detected. Expected offset %zu, got %lu. ESP32 OTA requires sequential writes.", 
                           _bytes_written, (unsigned long)offset);
            gcs().send_text(MAV_SEVERITY_ERROR, "OTA: True resume not supported - must restart upload from beginning");
            esp32_ota_abort();
            return false;
        }
    }
    
    // Validate firmware header on first write
    if (_bytes_written == 0) {
        if (!validate_esp32_firmware_header(data, len)) {
            esp32_ota_abort();
            return false;
        }
    }
    
    // Write data to OTA partition
    esp_err_t err = esp_ota_write(_ota_handle, data, len);
    if (err != ESP_OK) {
        gcs().send_text(MAV_SEVERITY_ERROR, "OTA: Write failed at offset %lu (%s)", 
                       (unsigned long)offset, esp_err_to_name(err));
        esp32_ota_abort();
        return false;
    }
    
    _bytes_written += len;
    
    // Check power safety during update (every ~64KB to avoid excessive checks)
    if ((_bytes_written % 65536) == 0) {
        check_power_safety_during_update();
    }
    
    // Progress reporting every 256KB
    if ((_bytes_written % 262144) == 0) {
        gcs().send_text(MAV_SEVERITY_INFO, "OTA: %lu bytes written", 
                       (unsigned long)_bytes_written);
    }
    
    return true;
}

bool esp32_ota_end() {
    if (_ota_handle == 0) {
        gcs().send_text(MAV_SEVERITY_ERROR, "OTA: No active update to complete");
        return false;
    }
    
    // Final safety check before completing
    if (!validate_update_conditions()) {
        gcs().send_text(MAV_SEVERITY_ERROR, "OTA: Safety conditions failed at completion");
        esp32_ota_abort();
        return false;
    }
    
    // End the OTA update
    esp_err_t err = esp_ota_end(_ota_handle);
    if (err != ESP_OK) {
        gcs().send_text(MAV_SEVERITY_ERROR, "OTA: Failed to end update (%s)", esp_err_to_name(err));
        _ota_handle = 0;
        return false;
    }
    
    // Set the new partition as boot partition
    err = esp_ota_set_boot_partition(_update_partition);
    if (err != ESP_OK) {
        gcs().send_text(MAV_SEVERITY_ERROR, "OTA: Failed to set boot partition (%s)", esp_err_to_name(err));
        _ota_handle = 0;
        return false;
    }
    
    _ota_handle = 0;
    
    gcs().send_text(MAV_SEVERITY_INFO, "OTA: Update complete - %lu bytes written", 
                   (unsigned long)_bytes_written);
    gcs().send_text(MAV_SEVERITY_INFO, "OTA: Rebooting to new firmware in 3 seconds...");
    
    // Give messages time to be sent before rebooting
    hal.scheduler->delay(3000);
    
    // Reboot to new firmware
    esp_restart();
    
    // This should never be reached
    return true;
}

void esp32_ota_abort() {
    if (_ota_handle != 0) {
        esp_ota_abort(_ota_handle);
        _ota_handle = 0;
        gcs().send_text(MAV_SEVERITY_WARNING, "OTA: Update aborted");
    }
    
    _bytes_written = 0;
    _update_partition = nullptr;
}

size_t esp32_ota_get_bytes_written() {
    return _bytes_written;
}

bool esp32_ota_is_in_progress() {
    return (_ota_handle != 0);
}

bool esp32_ota_factory_reset() {
    // Get factory partition  
    const esp_partition_t* factory = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
    if (!factory) {
        gcs().send_text(MAV_SEVERITY_ERROR, "OTA: No factory partition found");
        return false;
    }
    
    // Set factory as boot partition
    esp_err_t err = esp_ota_set_boot_partition(factory);
    if (err != ESP_OK) {
        gcs().send_text(MAV_SEVERITY_ERROR, "OTA: Failed to set factory boot partition (%s)", esp_err_to_name(err));
        return false;
    }
    
    gcs().send_text(MAV_SEVERITY_INFO, "OTA: Factory reset complete - rebooting to factory firmware");
    gcs().send_text(MAV_SEVERITY_INFO, "OTA: Future USB updates will work normally");
    
    // Give messages time to be sent
    hal.scheduler->delay(2000);
    
    // Reboot to factory firmware
    esp_restart();
    
    // This should never be reached
    return true;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_ESP32