/* Hello World Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stddef.h>
#include <stdio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

int main(int argc, char *argv[]);

void app_main()
{
    ESP_LOGI("APP", "ESP32-S3 app_main() entry point reached");
    ESP_LOGI("APP", "Starting ArduPilot main()");
    
    int result = main(0, NULL);
    
    // This should NEVER be reached - main() should never return!
    ESP_LOGE("APP", "CRITICAL ERROR: ArduPilot main() returned with code %d", result);
    ESP_LOGE("APP", "This indicates ArduPilot failed to start or crashed!");
    ESP_LOGE("APP", "System will halt.");
    
    // Halt the system
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGE("APP", "System halted due to ArduPilot main() exit");
    }
}
