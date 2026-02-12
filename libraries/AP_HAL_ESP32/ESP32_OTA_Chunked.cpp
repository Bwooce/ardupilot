/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

/*
  ESP32 Chunked OTA implementation for improved reliability over low-bandwidth links
  
  Features:
  - Chunked transfers with acknowledgments
  - Checksum verification per chunk
  - Automatic retry on failure
  - Resume support using persistent state
*/

#include "ESP32_OTA.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_Math/crc.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32

#include "esp_ota_ops.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

extern const AP_HAL::HAL& hal;

// Chunk size optimized for MAVLink (must fit in single MAVLink message)
#define OTA_CHUNK_SIZE 200  // Small enough for reliable transmission over mLRS
#define OTA_MAX_RETRIES 3
#define OTA_CHUNK_TIMEOUT_MS 5000
#define OTA_ACK_TIMEOUT_MS 2000

// OTA state stored in NVS for resume capability
struct OTAState {
    uint32_t total_size;
    uint32_t bytes_received;
    uint32_t last_chunk_id;
    uint32_t crc32;
    char filename[64];
    bool active;
};

static OTAState ota_state;
static nvs_handle_t ota_nvs_handle;
static const esp_partition_t* update_partition = nullptr;
static esp_ota_handle_t ota_handle = 0;

// Forward declarations
bool esp32_ota_complete_chunked();

// Initialize NVS for persistent OTA state
bool esp32_ota_chunked_init() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    
    if (err != ESP_OK) {
        ESP_LOGE("OTA", "Failed to init NVS: %s", esp_err_to_name(err));
        return false;
    }
    
    // Open NVS handle
    err = nvs_open("ota_state", NVS_READWRITE, &ota_nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE("OTA", "Failed to open NVS: %s", esp_err_to_name(err));
        return false;
    }
    
    // Try to load existing OTA state
    size_t state_size = sizeof(OTAState);
    err = nvs_get_blob(ota_nvs_handle, "state", &ota_state, &state_size);
    if (err == ESP_OK && ota_state.active) {
        gcs().send_text(MAV_SEVERITY_INFO, "OTA: Resuming from chunk %lu/%lu",
                       (unsigned long)ota_state.last_chunk_id,
                       (unsigned long)(ota_state.total_size / OTA_CHUNK_SIZE));
        return true;
    }
    
    // No active OTA or error reading state
    memset(&ota_state, 0, sizeof(ota_state));
    return true;
}

// Save OTA state to NVS for resume capability
bool esp32_ota_save_state() {
    esp_err_t err = nvs_set_blob(ota_nvs_handle, "state", &ota_state, sizeof(ota_state));
    if (err != ESP_OK) {
        ESP_LOGE("OTA", "Failed to save state: %s", esp_err_to_name(err));
        return false;
    }
    
    err = nvs_commit(ota_nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE("OTA", "Failed to commit NVS: %s", esp_err_to_name(err));
        return false;
    }
    
    return true;
}

// Clear OTA state after successful update or abort
void esp32_ota_clear_state() {
    memset(&ota_state, 0, sizeof(ota_state));
    nvs_erase_key(ota_nvs_handle, "state");
    nvs_commit(ota_nvs_handle);
}

// Start or resume chunked OTA
bool esp32_ota_start_chunked(const char* filename, uint32_t total_size) {
    if (filename == nullptr || total_size == 0) {
        return false;
    }
    
    // Check if we're resuming the same file
    if (ota_state.active && strcmp(ota_state.filename, filename) == 0) {
        // Resuming previous OTA
        gcs().send_text(MAV_SEVERITY_INFO, 
                       "OTA: Resuming %s from byte %lu/%lu",
                       filename,
                       (unsigned long)ota_state.bytes_received,
                       (unsigned long)total_size);
                       
        // Re-open the OTA handle
        update_partition = esp_ota_get_next_update_partition(NULL);
        if (!update_partition) {
            gcs().send_text(MAV_SEVERITY_ERROR, "OTA: No update partition");
            return false;
        }
        
        // For ESP32, we need to restart from beginning but can skip chunks
        esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
        if (err != ESP_OK) {
            gcs().send_text(MAV_SEVERITY_ERROR, "OTA: Resume failed: %s", esp_err_to_name(err));
            return false;
        }
        
        return true;
    }
    
    // Starting new OTA
    if (!validate_update_conditions()) {
        return false;
    }
    
    // Initialize state
    memset(&ota_state, 0, sizeof(ota_state));
    strncpy(ota_state.filename, filename, sizeof(ota_state.filename) - 1);
    ota_state.total_size = total_size;
    ota_state.active = true;
    ota_state.crc32 = 0xFFFFFFFF; // Initial CRC32 value
    
    // Get update partition
    update_partition = esp_ota_get_next_update_partition(NULL);
    if (!update_partition) {
        gcs().send_text(MAV_SEVERITY_ERROR, "OTA: No update partition");
        return false;
    }
    
    // Begin OTA
    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        gcs().send_text(MAV_SEVERITY_ERROR, "OTA: Begin failed: %s", esp_err_to_name(err));
        return false;
    }
    
    // Save initial state
    esp32_ota_save_state();
    
    gcs().send_text(MAV_SEVERITY_INFO, 
                   "OTA: Starting chunked update of %s (%lu bytes)",
                   filename, (unsigned long)total_size);
    
    return true;
}

// Process a chunk of OTA data with verification
bool esp32_ota_write_chunk(uint32_t chunk_id, const uint8_t* data, size_t len, uint32_t chunk_crc) {
    if (!ota_state.active || data == nullptr || len == 0 || len > OTA_CHUNK_SIZE) {
        return false;
    }
    
    // Check if this is the expected chunk (allow re-transmission of last chunk)
    uint32_t expected_chunk = ota_state.bytes_received / OTA_CHUNK_SIZE;
    if (chunk_id < expected_chunk) {
        // Already have this chunk, just ACK it
        gcs().send_text(MAV_SEVERITY_DEBUG, "OTA: Chunk %lu already received", 
                       (unsigned long)chunk_id);
        return true;
    } else if (chunk_id > expected_chunk) {
        // Out of order chunk
        gcs().send_text(MAV_SEVERITY_WARNING, "OTA: Out of order chunk %lu (expected %lu)",
                       (unsigned long)chunk_id, (unsigned long)expected_chunk);
        return false;
    }
    
    // Verify chunk CRC
    uint32_t calculated_crc = crc32_small(0xFFFFFFFF, data, len);
    if (calculated_crc != chunk_crc) {
        gcs().send_text(MAV_SEVERITY_WARNING, "OTA: Chunk %lu CRC mismatch (got 0x%08lX, expected 0x%08lX)",
                       (unsigned long)chunk_id,
                       (unsigned long)calculated_crc,
                       (unsigned long)chunk_crc);
        return false;
    }
    
    // Write to OTA partition
    esp_err_t err = esp_ota_write(ota_handle, data, len);
    if (err != ESP_OK) {
        gcs().send_text(MAV_SEVERITY_ERROR, "OTA: Write failed: %s", esp_err_to_name(err));
        return false;
    }
    
    // Update state
    ota_state.bytes_received += len;
    ota_state.last_chunk_id = chunk_id;
    ota_state.crc32 = crc32_small(ota_state.crc32, data, len);
    
    // Save state periodically (every 10 chunks)
    if (chunk_id % 10 == 0) {
        esp32_ota_save_state();
    }
    
    // Progress report every 50 chunks
    if (chunk_id % 50 == 0 || ota_state.bytes_received >= ota_state.total_size) {
        uint8_t progress = (ota_state.bytes_received * 100) / ota_state.total_size;
        gcs().send_text(MAV_SEVERITY_INFO, "OTA: Progress %u%% (%lu/%lu bytes)",
                       progress,
                       (unsigned long)ota_state.bytes_received,
                       (unsigned long)ota_state.total_size);
    }
    
    // Check if OTA is complete
    if (ota_state.bytes_received >= ota_state.total_size) {
        return esp32_ota_complete_chunked();
    }
    
    return true;
}

// Complete the chunked OTA
bool esp32_ota_complete_chunked() {
    if (!ota_state.active) {
        return false;
    }
    
    // Finalize OTA
    esp_err_t err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        gcs().send_text(MAV_SEVERITY_ERROR, "OTA: Finalize failed: %s", esp_err_to_name(err));
        esp32_ota_clear_state();
        return false;
    }
    
    // Set as boot partition
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        gcs().send_text(MAV_SEVERITY_ERROR, "OTA: Set boot failed: %s", esp_err_to_name(err));
        esp32_ota_clear_state();
        return false;
    }
    
    gcs().send_text(MAV_SEVERITY_CRITICAL, "OTA: Update complete! CRC32: 0x%08lX", 
                   (unsigned long)ota_state.crc32);
    gcs().send_text(MAV_SEVERITY_CRITICAL, "OTA: Rebooting in 3 seconds...");
    
    // Clear state
    esp32_ota_clear_state();
    
    // Schedule reboot
    hal.scheduler->delay(3000);
    hal.scheduler->reboot(false);
    
    return true;
}

// Abort chunked OTA
void esp32_ota_abort_chunked() {
    if (ota_state.active && ota_handle != 0) {
        esp_ota_abort(ota_handle);
        ota_handle = 0;
    }
    
    esp32_ota_clear_state();
    gcs().send_text(MAV_SEVERITY_WARNING, "OTA: Update aborted");
}

// Get current OTA status for MAVLink reporting
void esp32_ota_get_status(uint32_t& bytes_received, uint32_t& total_size, bool& active) {
    bytes_received = ota_state.bytes_received;
    total_size = ota_state.total_size;
    active = ota_state.active;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_ESP32