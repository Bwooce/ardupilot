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

#include <AP_HAL/AP_HAL.h>

#include "Semaphores.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

extern const AP_HAL::HAL& hal;

using namespace ESP32;

Semaphore::Semaphore(bool recursive)
    : _is_recursive(recursive)
{
    if (_is_recursive) {
        handle = xSemaphoreCreateRecursiveMutex();
        ESP_LOGV("MUTEX", "Created RECURSIVE mutex %p", handle);
    } else {
        handle = xSemaphoreCreateMutex();
        ESP_LOGV("MUTEX", "Created NON-RECURSIVE mutex %p", handle);
    }
}

bool Semaphore::give()
{
    BaseType_t result;
    TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
    
    if (_is_recursive) {
        result = xSemaphoreGiveRecursive((QueueHandle_t)handle);
    } else {
        result = xSemaphoreGive((QueueHandle_t)handle);
    }
    
    if (result != pdTRUE) {
        ESP_LOGE("MUTEX", "Thread %p FAILED to give %s mutex %p", 
                 current_task, _is_recursive ? "recursive" : "non-recursive", handle);
    }
    
    return result == pdTRUE;
}

bool Semaphore::take(uint32_t timeout_ms)
{
    if (timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER) {
        take_blocking();
        return true;
    }
    if (take_nonblocking()) {
        return true;
    }
    uint64_t start = AP_HAL::micros64();
    do {
        hal.scheduler->delay_microseconds(200);
        if (take_nonblocking()) {
            return true;
        }
    } while ((AP_HAL::micros64() - start) < timeout_ms*1000);
    return false;
}

void Semaphore::take_blocking()
{
    BaseType_t result;
    TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
    
    if (_is_recursive) {
        result = xSemaphoreTakeRecursive((QueueHandle_t)handle, portMAX_DELAY);
    } else {
        // Check if we already own this mutex (would indicate recursion attempt)
        if (xSemaphoreGetMutexHolder((QueueHandle_t)handle) == current_task) {
            const char* task_name = pcTaskGetName(current_task);
            ESP_LOGE("MUTEX", "RECURSION DETECTED! Thread %s (%p) already owns NON-RECURSIVE mutex %p", 
                     task_name, current_task, handle);
        }
        
        result = xSemaphoreTake((QueueHandle_t)handle, portMAX_DELAY);
    }
    
    if (result != pdTRUE) {
        ESP_LOGE("MUTEX", "Thread %p FAILED to take %s mutex %p - this should never happen with portMAX_DELAY", 
                 current_task, _is_recursive ? "recursive" : "non-recursive", handle);
    }
}

bool Semaphore::take_nonblocking()
{
    if (_is_recursive) {
        return xSemaphoreTakeRecursive((QueueHandle_t)handle, 0) == pdTRUE;
    } else {
        return xSemaphoreTake((QueueHandle_t)handle, 0) == pdTRUE;
    }
}

bool Semaphore::check_owner()
{
    return xSemaphoreGetMutexHolder((QueueHandle_t)handle) == xTaskGetCurrentTaskHandle();
}


/*
  BinarySemaphore implementation
 */
BinarySemaphore::BinarySemaphore(bool initial_state)
{
    _sem = xSemaphoreCreateBinary();
    if (initial_state) {
        xSemaphoreGive(_sem);
    }
}

bool BinarySemaphore::wait(uint32_t timeout_us)
{
    TickType_t ticks = pdMS_TO_TICKS(timeout_us / 1000U);
    return xSemaphoreTake(_sem, ticks) == pdTRUE;
}

bool BinarySemaphore::wait_blocking()
{
    return xSemaphoreTake(_sem, portMAX_DELAY) == pdTRUE;
}

void BinarySemaphore::signal()
{
    xSemaphoreGive(_sem);
}

void BinarySemaphore::signal_ISR()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(_sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR_ARG(xHigherPriorityTaskWoken);
}

BinarySemaphore::~BinarySemaphore(void)
{
    if (_sem != nullptr) {
        vSemaphoreDelete(_sem);
    }
    _sem = nullptr;
}
