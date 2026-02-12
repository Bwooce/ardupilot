/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

/*
  ESP32 OTA via MAVLink FILE_TRANSFER_PROTOCOL improvements
  
  This implements workarounds for unreliable FTP over low-bandwidth links:
  - Smaller block sizes for mLRS
  - Automatic retry with exponential backoff
  - Better error recovery
  - Progress tracking
*/

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Filesystem/AP_Filesystem.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32

#include "ESP32_OTA.h"
#include "esp_log.h"

extern const AP_HAL::HAL& hal;

// FTP tuning for unreliable links
#define FTP_BLOCK_SIZE_SLOW 128    // Smaller blocks for mLRS (was 512)
#define FTP_TIMEOUT_MS 10000       // Longer timeout for slow links
#define FTP_MAX_RETRIES 5          // More retries
#define FTP_RETRY_DELAY_MS 1000    // Initial retry delay

// Enhanced FTP session tracking
struct FTPSession {
    bool active;
    char filename[256];
    uint32_t offset;
    uint32_t total_size;
    uint32_t last_activity_ms;
    uint8_t retry_count;
    uint32_t retry_delay_ms;
    uint32_t crc32;
    int fd;  // File descriptor
};

static FTPSession ftp_session;

// Initialize FTP session for OTA
void esp32_ota_ftp_init() {
    memset(&ftp_session, 0, sizeof(ftp_session));
    ftp_session.fd = -1;
    
    // Configure filesystem for better reliability
    // Note: ESP32 SPIFFS doesn't support all operations
    ESP_LOGI("OTA_FTP", "FTP OTA handler initialized");
}

// Handle FTP open with OTA detection
bool esp32_ota_ftp_open(const char* filename, uint32_t size) {
    if (!filename) {
        return false;
    }
    
    // Check if this is a firmware file
    if (!is_firmware_file(filename)) {
        // Not a firmware file, handle normally
        return true;
    }
    
    ESP_LOGI("OTA_FTP", "Opening firmware file: %s (%lu bytes)", filename, (unsigned long)size);
    
    // Initialize FTP session
    ftp_session.active = true;
    strncpy(ftp_session.filename, filename, sizeof(ftp_session.filename) - 1);
    ftp_session.total_size = size;
    ftp_session.offset = 0;
    ftp_session.retry_count = 0;
    ftp_session.retry_delay_ms = FTP_RETRY_DELAY_MS;
    ftp_session.last_activity_ms = AP_HAL::millis();
    ftp_session.crc32 = 0xFFFFFFFF;
    
    // Try to open file for writing
    ftp_session.fd = AP::FS().open(filename, O_WRONLY | O_CREAT | O_TRUNC);
    if (ftp_session.fd < 0) {
        gcs().send_text(MAV_SEVERITY_ERROR, "OTA: Failed to create %s", filename);
        ftp_session.active = false;
        return false;
    }
    
    // Start OTA preparation
    if (!esp32_ota_begin(filename)) {
        AP::FS().close(ftp_session.fd);
        ftp_session.fd = -1;
        ftp_session.active = false;
        return false;
    }
    
    gcs().send_text(MAV_SEVERITY_INFO, "OTA: Ready to receive %s via FTP", filename);
    return true;
}

// Handle FTP write with retry logic
bool esp32_ota_ftp_write(uint32_t offset, const uint8_t* data, uint32_t len) {
    if (!ftp_session.active || !data || len == 0) {
        return false;
    }
    
    // Update activity timestamp
    ftp_session.last_activity_ms = AP_HAL::millis();
    
    // Check for sequential write (required for ESP32 OTA)
    if (offset != ftp_session.offset) {
        // Handle retransmission of same block
        if (offset < ftp_session.offset && 
            offset + len == ftp_session.offset) {
            // This is a retransmission of the last block, ignore it
            ESP_LOGD("OTA_FTP", "Ignoring retransmission at offset %lu", (unsigned long)offset);
            return true;
        }
        
        // Non-sequential write - try to recover
        ESP_LOGW("OTA_FTP", "Non-sequential write: expected %lu, got %lu", 
                 (unsigned long)ftp_session.offset, (unsigned long)offset);
        
        if (offset < ftp_session.offset) {
            // Request retransmission from last known good position
            gcs().send_text(MAV_SEVERITY_WARNING, 
                           "OTA: Out of order at %lu, need sequential from %lu",
                           (unsigned long)offset, (unsigned long)ftp_session.offset);
            return false;
        } else {
            // Gap in data - request missing chunk
            gcs().send_text(MAV_SEVERITY_WARNING, 
                           "OTA: Missing data from %lu to %lu",
                           (unsigned long)ftp_session.offset, (unsigned long)offset);
            return false;
        }
    }
    
    // Write to file for backup
    if (ftp_session.fd >= 0) {
        int32_t written = AP::FS().write(ftp_session.fd, data, len);
        if (written != (int32_t)len) {
            ESP_LOGE("OTA_FTP", "File write failed: %ld/%lu", (long)written, (unsigned long)len);
        }
    }
    
    // Write to OTA partition
    bool success = esp32_ota_write(data, len, offset);
    if (!success) {
        ftp_session.retry_count++;
        if (ftp_session.retry_count >= FTP_MAX_RETRIES) {
            gcs().send_text(MAV_SEVERITY_ERROR, 
                           "OTA: Write failed after %u retries at offset %lu",
                           ftp_session.retry_count, (unsigned long)offset);
            esp32_ota_abort();
            ftp_session.active = false;
            return false;
        }
        
        // Request retry with backoff
        hal.scheduler->delay(ftp_session.retry_delay_ms);
        ftp_session.retry_delay_ms = MIN(ftp_session.retry_delay_ms * 2, 10000);
        
        gcs().send_text(MAV_SEVERITY_WARNING, 
                       "OTA: Write failed at %lu, retry %u/%u",
                       (unsigned long)offset, ftp_session.retry_count, FTP_MAX_RETRIES);
        return false;
    }
    
    // Success - update state
    ftp_session.offset += len;
    ftp_session.retry_count = 0;
    ftp_session.retry_delay_ms = FTP_RETRY_DELAY_MS;
    
    // Progress reporting
    static uint32_t last_progress_report = 0;
    uint32_t progress_pct = (ftp_session.offset * 100) / ftp_session.total_size;
    if (progress_pct != last_progress_report && progress_pct % 10 == 0) {
        last_progress_report = progress_pct;
        gcs().send_text(MAV_SEVERITY_INFO, 
                       "OTA: Progress %lu%% (%lu/%lu KB)",
                       (unsigned long)progress_pct,
                       (unsigned long)(ftp_session.offset / 1024),
                       (unsigned long)(ftp_session.total_size / 1024));
    }
    
    return true;
}

// Handle FTP close/complete
bool esp32_ota_ftp_close() {
    if (!ftp_session.active) {
        return true;
    }
    
    // Close file if open
    if (ftp_session.fd >= 0) {
        AP::FS().close(ftp_session.fd);
        ftp_session.fd = -1;
    }
    
    // Check if we received all data
    if (ftp_session.offset < ftp_session.total_size) {
        gcs().send_text(MAV_SEVERITY_WARNING, 
                       "OTA: Incomplete transfer %lu/%lu bytes",
                       (unsigned long)ftp_session.offset,
                       (unsigned long)ftp_session.total_size);
        esp32_ota_abort();
        ftp_session.active = false;
        return false;
    }
    
    // Complete OTA
    if (!esp32_ota_end()) {
        ftp_session.active = false;
        return false;
    }
    
    gcs().send_text(MAV_SEVERITY_CRITICAL, 
                   "OTA: SUCCESS! Received %lu bytes. Rebooting...",
                   (unsigned long)ftp_session.offset);
    
    ftp_session.active = false;
    
    // Schedule reboot
    hal.scheduler->delay(2000);
    hal.scheduler->reboot(false);
    
    return true;
}

// Monitor FTP timeout
void esp32_ota_ftp_update() {
    if (!ftp_session.active) {
        return;
    }
    
    uint32_t now = AP_HAL::millis();
    if (now - ftp_session.last_activity_ms > FTP_TIMEOUT_MS) {
        gcs().send_text(MAV_SEVERITY_ERROR, 
                       "OTA: FTP timeout after %lu bytes",
                       (unsigned long)ftp_session.offset);
        
        // Try to save partial firmware for analysis
        if (ftp_session.fd >= 0) {
            AP::FS().fsync(ftp_session.fd);
            AP::FS().close(ftp_session.fd);
            ftp_session.fd = -1;
            
            gcs().send_text(MAV_SEVERITY_INFO, 
                           "OTA: Partial firmware saved to %s",
                           ftp_session.filename);
        }
        
        esp32_ota_abort();
        ftp_session.active = false;
    }
}

// Alternative: Simple scripting protocol for OTA
// This would use custom MAVLink messages for better control
bool esp32_ota_simple_protocol(uint8_t command, uint32_t param1, const uint8_t* data, uint32_t len) {
    enum OTACommands {
        OTA_CMD_START = 1,
        OTA_CMD_WRITE = 2,
        OTA_CMD_VERIFY = 3,
        OTA_CMD_COMPLETE = 4,
        OTA_CMD_ABORT = 5,
        OTA_CMD_STATUS = 6,
    };
    
    switch (command) {
    case OTA_CMD_START:
        // param1 = total size
        return esp32_ota_begin("mavlink_ota.bin");
        
    case OTA_CMD_WRITE:
        // param1 = offset, data = chunk data
        return esp32_ota_write(data, len, param1);
        
    case OTA_CMD_VERIFY:
        // param1 = expected CRC32
        // TODO: Implement CRC verification
        ESP_LOGI("OTA", "Verify requested with CRC32: 0x%08lX (not yet implemented)", (unsigned long)param1);
        return true;  // For now, always succeed
        
    case OTA_CMD_COMPLETE:
        return esp32_ota_end();
        
    case OTA_CMD_ABORT:
        esp32_ota_abort();
        return true;
        
    case OTA_CMD_STATUS:
        // Send back status via STATUSTEXT
        gcs().send_text(MAV_SEVERITY_INFO, "OTA: Status request - offset %lu",
                       (unsigned long)ftp_session.offset);
        return true;
        
    default:
        return false;
    }
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_ESP32