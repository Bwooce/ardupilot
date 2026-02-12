/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

/*
  ESP32 SPIFFS/LittleFS filesystem support for OTA updates ONLY

  WARNING: NEVER use SPIFFS for logging - causes SPI flash cache deadlocks!
  Every SPIFFS write requires disabling cache on both CPUs, which fails
  when other tasks don't yield, causing watchdog timeouts.

  This filesystem is ONLY for OTA firmware storage.
*/

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32

#include "ESP32_SPIFFS.h"
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs.h"

// Avoid conflicts with dirent.h definitions
#ifdef DT_REG
#undef DT_REG
#endif
#ifdef DT_DIR
#undef DT_DIR
#endif

#include <AP_Filesystem/AP_Filesystem.h>

// Include SPIFFS after AP_Filesystem to avoid conflicts
#ifdef __has_include
  #if __has_include("esp_spiffs.h")
    #include "esp_spiffs.h"
    #define HAS_ESP_SPIFFS 1
  #else
    #warning "esp_spiffs.h not found, SPIFFS support disabled"
    #define HAS_ESP_SPIFFS 0
  #endif
#else
  // Assume we have SPIFFS on older compilers
  #include "esp_spiffs.h"
  #define HAS_ESP_SPIFFS 1
#endif

#include <GCS_MAVLink/GCS.h>

static const char* TAG = "ESP32_SPIFFS";
#if HAS_ESP_SPIFFS
static bool spiffs_mounted = false;
#endif

// Initialize SPIFFS filesystem
bool esp32_spiffs_init(void)
{
#if HAS_ESP_SPIFFS
    if (spiffs_mounted) {
        return true;
    }

    // Use GCS message to avoid stack overflow
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ESP32: Initializing SPIFFS");
    ESP_LOGI(TAG, "Initializing SPIFFS for OTA updates only");

    // Configure SPIFFS with optimizations for fast startup
    esp_vfs_spiffs_conf_t spiffs_conf = {
        .base_path = "/APM",
        .partition_label = "ota_storage",  // For OTA firmware updates only
        .max_files = 50,  // Increased from 10 to handle more log files
        .format_if_mount_failed = false  // Don't auto-format, just fail if corrupted
    };

    esp_err_t ret = esp_vfs_spiffs_register(&spiffs_conf);

    // Handle mount failures
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "SPIFFS partition not found in partition table");
            return false;
        }

        // Only format if it's genuinely corrupted, not for other errors
        if (ret == ESP_ERR_INVALID_STATE || ret == ESP_FAIL) {
            ESP_LOGW(TAG, "SPIFFS appears corrupted (%s), formatting once", esp_err_to_name(ret));

            // Unregister first if needed
            esp_vfs_spiffs_unregister("ota_storage");

            // Format the partition
            ret = esp_spiffs_format("ota_storage");
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "SPIFFS format failed (%s)", esp_err_to_name(ret));
                return false;
            }

            // Try mounting again after format
            ret = esp_vfs_spiffs_register(&spiffs_conf);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "SPIFFS mount failed after format (%s)", esp_err_to_name(ret));
                return false;
            }

            ESP_LOGW(TAG, "SPIFFS formatted and mounted successfully");
        } else {
            ESP_LOGE(TAG, "SPIFFS mount failed (%s)", esp_err_to_name(ret));
            return false;
        }
    }
    
    // Check partition info
    size_t total = 0, used = 0;
    ret = esp_spiffs_info("ota_storage", &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "SPIFFS mounted successfully");
    ESP_LOGI(TAG, "Partition size: total: %d KB, used: %d KB (%d%%)",
             total/1024, used/1024, (used*100)/total);


    // If filesystem is nearly full, warn about OTA space
    if (used > (total * 95 / 100)) {
        ESP_LOGW(TAG, "SPIFFS nearly full (%d%%), attempting cleanup", (used*100)/total);
        // Note: Actual cleanup would be done by AP_Logger
    }
    
    // Create log directory if it doesn't exist
    // Note: SPIFFS doesn't support directories, so we just need to ensure the base path works
    // Files will be created as /APM/LOGS/00000001.BIN which SPIFFS treats as a flat filename
    ESP_LOGI(TAG, "SPIFFS ready at /APM (note: SPIFFS doesn't use directories, paths are flat)");
    
    spiffs_mounted = true;
    
    // Report to GCS
    gcs().send_text(MAV_SEVERITY_INFO, "SPIFFS: %d MB available for OTA", 
                   (total - used) / (1024 * 1024));
    
    return true;
#else
    ESP_LOGW(TAG, "SPIFFS support not available in this build");
    return false;
#endif
}

// Check if SPIFFS is mounted and healthy
bool esp32_spiffs_healthy(void)
{
#if HAS_ESP_SPIFFS
    if (!spiffs_mounted) {
        return false;
    }
    
    // Check if we can still access the filesystem
    size_t total = 0, used = 0;
    esp_err_t ret = esp_spiffs_info("ota_storage", &total, &used);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS health check failed");
        spiffs_mounted = false;
        return false;
    }
    
    // Warn if getting full
    uint8_t percent_used = (used * 100) / total;
    if (percent_used > 90) {
        static uint32_t last_warning = 0;
        uint32_t now = AP_HAL::millis();
        if (now - last_warning > 30000) { // Warn every 30 seconds
            last_warning = now;
            gcs().send_text(MAV_SEVERITY_WARNING, "SPIFFS: %d%% full, OTA may fail", percent_used);
        }
    }
    
    return true;
#else
    return false;
#endif
}

// Get available space in bytes
uint32_t esp32_spiffs_available_bytes(void)
{
#if HAS_ESP_SPIFFS
    if (!spiffs_mounted) {
        return 0;
    }
    
    size_t total = 0, used = 0;
    esp_err_t ret = esp_spiffs_info("ota_storage", &total, &used);
    
    if (ret != ESP_OK) {
        return 0;
    }
    
    return (total - used);
#else
    return 0;
#endif
}

// Format SPIFFS (erase all data)
bool esp32_spiffs_format(void)
{
#if HAS_ESP_SPIFFS
    ESP_LOGI(TAG, "Formatting SPIFFS partition...");
    
    // Unmount first if mounted
    if (spiffs_mounted) {
        esp_vfs_spiffs_unregister("ota_storage");
        spiffs_mounted = false;
    }
    
    // Format the partition
    esp_err_t ret = esp_spiffs_format("ota_storage");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to format SPIFFS (%s)", esp_err_to_name(ret));
        return false;
    }
    
    ESP_LOGI(TAG, "SPIFFS formatted successfully");
    
    // Remount
    return esp32_spiffs_init();
#else
    return false;
#endif
}

// List all files in a directory
void esp32_spiffs_list_files(const char* path)
{
#if HAS_ESP_SPIFFS
    if (!spiffs_mounted) {
        ESP_LOGE(TAG, "SPIFFS not mounted");
        return;
    }
    
    DIR* dir = opendir(path);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open directory %s", path);
        return;
    }
    
    struct dirent* entry;
    ESP_LOGI(TAG, "Files in %s:", path);
    
    while ((entry = readdir(dir)) != NULL) {
        struct stat st;
        char filepath[512];  // Increased buffer size to handle longer paths
        int ret = snprintf(filepath, sizeof(filepath), "%s/%s", path, entry->d_name);
        if (ret >= (int)sizeof(filepath)) {
            ESP_LOGW(TAG, "Path too long, skipping: %s/%s", path, entry->d_name);
            continue;
        }
        
        if (stat(filepath, &st) == 0) {
            ESP_LOGI(TAG, "  %s (%ld bytes)", entry->d_name, st.st_size);
        } else {
            ESP_LOGI(TAG, "  %s (size unknown)", entry->d_name);
        }
    }
    
    closedir(dir);
#endif
}

// Delete old log files to free space
uint32_t esp32_spiffs_free_space(uint32_t required_bytes)
{
#if HAS_ESP_SPIFFS
    if (!spiffs_mounted) {
        return 0;
    }
    
    uint32_t freed = 0;
    DIR* dir = opendir("/APM/LOGS");
    if (!dir) {
        return 0;
    }
    
    // Find and delete oldest logs
    struct dirent* entry;
    while ((entry = readdir(dir)) != NULL && freed < required_bytes) {
        // Only delete .BIN log files
        if (strstr(entry->d_name, ".BIN") != NULL) {
            char filepath[512];  // Increased buffer size
            int ret = snprintf(filepath, sizeof(filepath), "/APM/LOGS/%s", entry->d_name);
            if (ret >= (int)sizeof(filepath)) {
                ESP_LOGW(TAG, "Path too long, skipping: /APM/LOGS/%s", entry->d_name);
                continue;
            }
            
            struct stat st;
            if (stat(filepath, &st) == 0) {
                if (unlink(filepath) == 0) {
                    freed += st.st_size;
                    ESP_LOGI(TAG, "Deleted %s to free %ld bytes", entry->d_name, st.st_size);
                }
            }
        }
    }
    
    closedir(dir);
    return freed;
#else
    return 0;
#endif
}

// Periodic SPIFFS maintenance
void esp32_spiffs_update(void)
{
#if HAS_ESP_SPIFFS
    static uint32_t last_check = 0;
    uint32_t now = AP_HAL::millis();
    
    // Check every 10 seconds
    if (now - last_check < 10000) {
        return;
    }
    last_check = now;
    
    if (!esp32_spiffs_healthy()) {
        // Try to reinitialize
        esp32_spiffs_init();
    }
    
    // Check if we need to free space
    uint32_t available = esp32_spiffs_available_bytes();
    if (available < 1024 * 1024) { // Less than 1MB free
        ESP_LOGW(TAG, "Low space, attempting to free 2MB");
        esp32_spiffs_free_space(2 * 1024 * 1024);
    }
#endif
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_ESP32