/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

/*
  ESP32 PSRAM-based logging implementation
  
  This provides a high-performance, wear-free logging backend using
  the ESP32's external PSRAM (SPI RAM). Much better than flash for logging!
*/

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32

#include "ESP32_PSRAM_Logger.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/crc.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include <string.h>

// Use heap_caps API which is always available
// This works whether PSRAM is enabled or not

extern const AP_HAL::HAL& hal;

static const char* TAG = "PSRAM_LOG";

// Static member definitions
uint8_t *ESP32_PSRAM_Logger::buffer_start = nullptr;
uint32_t ESP32_PSRAM_Logger::buffer_size = 0;
uint32_t ESP32_PSRAM_Logger::write_offset = 0;
uint32_t ESP32_PSRAM_Logger::num_wraps = 0;
bool ESP32_PSRAM_Logger::initialized = false;

// Initialize PSRAM logging
bool ESP32_PSRAM_Logger::init(uint32_t buffer_size_mb)
{
    if (initialized) {
        return true;
    }
    
    // Check if PSRAM is available
    if (!psram_available()) {
        ESP_LOGE(TAG, "PSRAM not available on this hardware");
        // Don't fail completely - just disable PSRAM logging
        initialized = false;
        buffer_start = nullptr;
        buffer_size = 0;
        return false;
    }
    
    uint32_t psram_total = psram_size();
    ESP_LOGI(TAG, "PSRAM detected: %lu MB total", (unsigned long)(psram_total / (1024 * 1024)));
    
    // Calculate buffer size (leave some PSRAM for other uses)
    buffer_size = buffer_size_mb * 1024 * 1024;
    if (buffer_size > psram_total / 2) {
        buffer_size = psram_total / 2;  // Use max 50% of PSRAM for logging
        ESP_LOGW(TAG, "Requested size too large, using %lu MB", 
                (unsigned long)(buffer_size / (1024 * 1024)));
    }
    
    // Allocate buffer in PSRAM
    buffer_start = (uint8_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    if (buffer_start == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate %lu bytes in PSRAM", (unsigned long)buffer_size);
        return false;
    }
    
    ESP_LOGI(TAG, "Allocated %lu MB at 0x%08lX for logging", 
            (unsigned long)(buffer_size / (1024 * 1024)),
            (unsigned long)buffer_start);
    
    // Check if there's existing valid log data (survives soft reset)
    // Use memcpy to avoid alignment issues
    log_header header_check;
    memcpy(&header_check, buffer_start, sizeof(log_header));
    
    if (header_check.magic == PSRAM_LOG_MAGIC) {
        // Calculate expected CRC
        uint32_t expected_crc = crc32_small(0xFFFFFFFF, (uint8_t*)&header_check, 
                                           offsetof(log_header, crc32));
        
        if (header_check.crc32 == expected_crc && header_check.buffer_size == buffer_size) {
            // Valid existing log found, resume from where we left off
            write_offset = header_check.write_offset;
            num_wraps = header_check.num_wraps;
            ESP_LOGI(TAG, "Resuming existing log at offset %lu (wrapped %lu times)",
                    (unsigned long)write_offset, (unsigned long)num_wraps);
        } else {
            // Invalid or mismatched header, start fresh
            ESP_LOGI(TAG, "Starting fresh log (invalid existing header)");
            clear();
        }
    } else {
        // No existing log, start fresh
        ESP_LOGI(TAG, "Starting fresh log buffer");
        clear();
    }
    
    initialized = true;
    
    // Report to GCS
    gcs().send_text(MAV_SEVERITY_INFO, "PSRAM Log: %lu MB ready at 0x%08lX",
                   (unsigned long)(buffer_size / (1024 * 1024)),
                   (unsigned long)buffer_start);
    
    return true;
}

// Check if PSRAM logging is healthy
bool ESP32_PSRAM_Logger::healthy(void)
{
    if (!initialized || buffer_start == nullptr) {
        return false;
    }
    
    // Do a quick write/read test to verify PSRAM is still accessible
    static uint32_t test_pattern = 0xDEADBEEF;
    static uint32_t last_test = 0;
    uint32_t now = AP_HAL::millis();
    
    // Test every 10 seconds
    if (now - last_test > 10000) {
        last_test = now;
        
        // Save current data at test location
        uint32_t saved;
        memcpy(&saved, buffer_start, sizeof(saved));
        
        // Write test pattern
        memcpy(buffer_start, &test_pattern, sizeof(test_pattern));
        
        // Read back and verify
        uint32_t readback;
        memcpy(&readback, buffer_start, sizeof(readback));
        
        // Restore original data
        memcpy(buffer_start, &saved, sizeof(saved));
        
        if (readback != test_pattern) {
            ESP_LOGE(TAG, "PSRAM health check failed!");
            initialized = false;
            return false;
        }
    }
    
    return true;
}

// Get total space
uint32_t ESP32_PSRAM_Logger::get_total_space(void)
{
    return buffer_size;
}

// Get used space
uint32_t ESP32_PSRAM_Logger::get_used_space(void)
{
    if (num_wraps > 0) {
        return buffer_size;  // Buffer is full if we've wrapped
    }
    return write_offset;
}

// Get free space
uint32_t ESP32_PSRAM_Logger::get_free_space(void)
{
    if (num_wraps > 0) {
        return 0;  // No free space if we've wrapped (circular buffer)
    }
    return buffer_size - write_offset;
}

// Write data to the log
bool ESP32_PSRAM_Logger::write(const uint8_t *data, uint32_t len)
{
    if (!initialized || buffer_start == nullptr) {
        return false;
    }
    
    // Skip header area
    uint32_t header_size = sizeof(log_header);
    uint32_t data_start = header_size;
    uint32_t data_size = buffer_size - header_size;
    
    // Handle circular buffer wrap
    while (len > 0) {
        uint32_t offset_in_data = write_offset - data_start;
        uint32_t space_until_wrap = data_size - offset_in_data;
        uint32_t bytes_to_write = MIN(len, space_until_wrap);
        
        // Write the data
        memcpy(buffer_start + write_offset, data, bytes_to_write);
        
        write_offset += bytes_to_write;
        data += bytes_to_write;
        len -= bytes_to_write;
        
        // Handle wrap
        if (write_offset >= buffer_size) {
            write_offset = data_start;
            num_wraps++;
            
            // Log wrap event periodically
            if (num_wraps % 100 == 0) {
                ESP_LOGI(TAG, "Log buffer wrapped %lu times", (unsigned long)num_wraps);
            }
        }
    }
    
    // Update header periodically (every 1MB written)
    static uint32_t last_header_update = 0;
    if (write_offset - last_header_update > 1024 * 1024 || 
        write_offset < last_header_update) {  // Handle wrap
        
        last_header_update = write_offset;
        
        log_header header;
        header.magic = PSRAM_LOG_MAGIC;
        header.buffer_size = buffer_size;
        header.write_offset = write_offset;
        header.num_wraps = num_wraps;
        header.crc32 = crc32_small(0xFFFFFFFF, (uint8_t*)&header, 
                                  offsetof(log_header, crc32));
        
        memcpy(buffer_start, &header, sizeof(header));
    }
    
    return true;
}

// Read data from the log
bool ESP32_PSRAM_Logger::read(uint8_t *data, uint32_t len, uint32_t offset)
{
    if (!initialized || buffer_start == nullptr) {
        return false;
    }
    
    uint32_t header_size = sizeof(log_header);
    uint32_t data_start = header_size;
    
    // Adjust offset to skip header
    offset += data_start;
    
    // Check bounds
    if (offset + len > buffer_size) {
        return false;
    }
    
    memcpy(data, buffer_start + offset, len);
    return true;
}

// Clear all logs
void ESP32_PSRAM_Logger::clear(void)
{
    write_offset = sizeof(log_header);
    num_wraps = 0;
    
    if (buffer_start != nullptr) {
        // Clear header
        log_header header;
        header.magic = PSRAM_LOG_MAGIC;
        header.buffer_size = buffer_size;
        header.write_offset = write_offset;
        header.num_wraps = num_wraps;
        header.crc32 = crc32_small(0xFFFFFFFF, (uint8_t*)&header, 
                                  offsetof(log_header, crc32));
        
        memcpy(buffer_start, &header, sizeof(header));
        
        // Don't log here - already logged by callers when appropriate
    }
}

// Get current write position
uint32_t ESP32_PSRAM_Logger::get_write_offset(void)
{
    return write_offset;
}

// Dump logs to MAVLink for debugging
void ESP32_PSRAM_Logger::dump_to_mavlink(uint32_t start_offset, uint32_t max_bytes)
{
    if (!initialized || buffer_start == nullptr) {
        gcs().send_text(MAV_SEVERITY_ERROR, "PSRAM Log: Not initialized");
        return;
    }
    
    uint32_t bytes_to_send = MIN(max_bytes, get_used_space());
    uint32_t bytes_sent = 0;
    
    gcs().send_text(MAV_SEVERITY_INFO, "PSRAM Log: Dumping %lu bytes from offset %lu",
                   (unsigned long)bytes_to_send, (unsigned long)start_offset);
    
    // Send in chunks via STATUSTEXT
    const uint32_t chunk_size = 50;  // Max safe size for STATUSTEXT
    uint8_t chunk[chunk_size];
    char hex_str[chunk_size * 3 + 1];  // Each byte becomes "XX " (3 chars)
    
    while (bytes_sent < bytes_to_send) {
        uint32_t this_chunk = MIN(chunk_size, bytes_to_send - bytes_sent);
        
        if (!read(chunk, this_chunk, start_offset + bytes_sent)) {
            break;
        }
        
        // Convert to hex string
        hex_str[0] = '\0';
        for (uint32_t i = 0; i < this_chunk && i < 16; i++) {  // Limit to 16 bytes per message
            char byte_str[4];
            hal.util->snprintf(byte_str, sizeof(byte_str), "%02X ", chunk[i]);
            strcat(hex_str, byte_str);
        }
        
        gcs().send_text(MAV_SEVERITY_INFO, "LOG[%04lX]: %s",
                       (unsigned long)(start_offset + bytes_sent), hex_str);
        
        bytes_sent += this_chunk;
        
        // Rate limit to avoid flooding
        hal.scheduler->delay(10);
    }
    
    gcs().send_text(MAV_SEVERITY_INFO, "PSRAM Log: Dump complete (%lu bytes)",
                   (unsigned long)bytes_sent);
}

// Check if PSRAM is available
bool ESP32_PSRAM_Logger::psram_available(void)
{
    // Check if any SPIRAM is available using heap_caps
    size_t psram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    
    // Always log the result for debugging
    ESP_LOGI(TAG, "PSRAM check: total=%lu bytes, free=%lu bytes", 
             (unsigned long)psram_size, (unsigned long)psram_free);
    
    if (psram_size == 0) {
        ESP_LOGE(TAG, "No PSRAM detected! Check ESP-IDF configuration.");
        ESP_LOGE(TAG, "Ensure CONFIG_SPIRAM=y in sdkconfig");
    }
    
    return psram_size > 0;
}

// Get PSRAM size in bytes
uint32_t ESP32_PSRAM_Logger::psram_size(void)
{
    return heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_ESP32