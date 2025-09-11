/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

/*
  ESP32 SPIFFS/LittleFS filesystem support for internal flash logging
  Uses ESP32's internal flash memory for log storage when no SD card is available
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
    
    ESP_LOGI(TAG, "Initializing SPIFFS for internal flash logging");
    
    // Configure SPIFFS
    esp_vfs_spiffs_conf_t spiffs_conf = {
        .base_path = "/APM",
        .partition_label = "logs",  // Must match partition table
        .max_files = 10,
        .format_if_mount_failed = true
    };
    
    // Register and mount SPIFFS
    esp_err_t ret = esp_vfs_spiffs_register(&spiffs_conf);
    
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return false;
    }
    
    // Check partition info
    size_t total = 0, used = 0;
    ret = esp_spiffs_info("logs", &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
        return false;
    }
    
    ESP_LOGI(TAG, "SPIFFS mounted successfully");
    ESP_LOGI(TAG, "Partition size: total: %d KB, used: %d KB (%d%%)", 
             total/1024, used/1024, (used*100)/total);
    
    // Create log directory if it doesn't exist
    struct stat st;
    if (stat("/APM/LOGS", &st) != 0) {
        ESP_LOGI(TAG, "Creating /APM/LOGS directory");
        if (mkdir("/APM/LOGS", 0777) != 0) {
            ESP_LOGE(TAG, "Failed to create log directory");
            // Continue anyway - AP_Logger will handle this
        }
    }
    
    spiffs_mounted = true;
    
    // Report to GCS
    gcs().send_text(MAV_SEVERITY_INFO, "SPIFFS: %d MB available for logs", 
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
    esp_err_t ret = esp_spiffs_info("logs", &total, &used);
    
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
            gcs().send_text(MAV_SEVERITY_WARNING, "SPIFFS: %d%% full, logs may stop", percent_used);
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
    esp_err_t ret = esp_spiffs_info("logs", &total, &used);
    
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
        esp_vfs_spiffs_unregister("logs");
        spiffs_mounted = false;
    }
    
    // Format the partition
    esp_err_t ret = esp_spiffs_format("logs");
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
        char filepath[256];
        snprintf(filepath, sizeof(filepath), "%s/%s", path, entry->d_name);
        
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
            char filepath[256];
            snprintf(filepath, sizeof(filepath), "/APM/LOGS/%s", entry->d_name);
            
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