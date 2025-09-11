#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "BOOT_TEST";

void app_main(void)
{
    // Most basic test possible
    ESP_LOGE(TAG, "=== MINIMAL BOOT TEST ===");
    ESP_LOGE(TAG, "app_main() reached!");
    
    printf("\n*** Minimal test successful ***\n");
    
    while (1) {
        ESP_LOGE(TAG, "Still alive...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}