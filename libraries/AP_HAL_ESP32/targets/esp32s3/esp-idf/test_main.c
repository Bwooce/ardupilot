/* Minimal test to verify app_main is being called */
#include <stdio.h>
#include "esp_log.h"
#include "esp_rom_uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    // Use ROM printf which should always work
    esp_rom_printf("\n*** TEST: app_main() reached ***\n");
    
    // Also try ESP_LOG
    ESP_LOGE("TEST", "Minimal test app_main");
    
    // Spin forever
    while (1) {
        esp_rom_printf(".");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}