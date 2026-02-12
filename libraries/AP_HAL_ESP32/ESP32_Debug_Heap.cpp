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

#include "ESP32_Debug.h"

#if !ESP32_DEBUG_DISABLED && ESP32_DEBUG_MODE

#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static TaskHandle_t heap_monitor_task_handle = nullptr;
static const char* TAG = "ESP32_HEAP";

void esp32_debug_print_heap_stats(void)
{
    ESP_LOGI(TAG, "=== ESP32 Heap Statistics ===");
    
    // Print overall heap info
    multi_heap_info_t info;
    heap_caps_get_info(&info, MALLOC_CAP_DEFAULT);
    
    ESP_LOGI(TAG, "Total heap: %d bytes", info.total_allocated_bytes + info.total_free_bytes);
    ESP_LOGI(TAG, "Free heap: %d bytes", info.total_free_bytes);
    ESP_LOGI(TAG, "Allocated: %d bytes", info.total_allocated_bytes);
    ESP_LOGI(TAG, "Largest free block: %d bytes", info.largest_free_block);
    ESP_LOGI(TAG, "Minimum free: %d bytes", info.minimum_free_bytes);
    
    // Print PSRAM info if available
    if (heap_caps_get_total_size(MALLOC_CAP_SPIRAM) > 0) {
        heap_caps_get_info(&info, MALLOC_CAP_SPIRAM);
        ESP_LOGI(TAG, "PSRAM total: %d bytes", info.total_allocated_bytes + info.total_free_bytes);
        ESP_LOGI(TAG, "PSRAM free: %d bytes", info.total_free_bytes);
        ESP_LOGI(TAG, "PSRAM allocated: %d bytes", info.total_allocated_bytes);
    }
    
    // Print per-task heap allocation statistics
    #ifdef CONFIG_HEAP_TASK_TRACKING
    ESP_LOGI(TAG, "=== Per-Task Heap Usage ===");
    heap_caps_print_heap_info(MALLOC_CAP_DEFAULT);
    #endif
    
    ESP_LOGI(TAG, "==============================");
}

static void heap_monitor_task(void *parameter)
{
    const TickType_t delay_ticks = pdMS_TO_TICKS(300000); // 5 minutes
    
    ESP_LOGI(TAG, "Heap monitor task started - reporting every 5 minutes");
    
    while (1) {
        vTaskDelay(delay_ticks);
        esp32_debug_print_heap_stats();
    }
}

void esp32_debug_init_heap_monitoring(void)
{
    ESP_LOGI(TAG, "Initializing ESP32 heap monitoring");
    
    // Create heap monitoring task with low priority
    BaseType_t result = xTaskCreate(
        heap_monitor_task,
        "heap_monitor",
        2048,  // Stack size
        nullptr,
        1,     // Low priority
        &heap_monitor_task_handle
    );
    
    if (result == pdPASS) {
        ESP_LOGI(TAG, "Heap monitoring task created successfully");
        // Print initial heap stats
        esp32_debug_print_heap_stats();
    } else {
        ESP_LOGE(TAG, "Failed to create heap monitoring task");
    }
}

void esp32_debug_heap_monitor_task(void)
{
    // Manual trigger for heap stats (can be called from main loop)
    esp32_debug_print_heap_stats();
}

#endif // !ESP32_DEBUG_DISABLED && ESP32_DEBUG_MODE