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
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit and David "Buzz" Bussenschutt
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "Util.h"

#include "RCOutput.h"

#include <AP_ROMFS/AP_ROMFS.h>
#include "SdCard.h"

#include <esp_timer.h>
#include <multi_heap.h>
#include <esp_heap_caps.h>

#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include <AP_Common/ExpandingString.h>

#include "esp_mac.h"
// Note: Custom MAC support temporarily disabled due to build issues with esp_efuse.h
// Will be re-enabled once ESP-IDF include paths are fixed

extern const AP_HAL::HAL& hal;

using namespace ESP32;


/**
   how much free memory do we have in bytes.
*/
uint32_t Util::available_memory(void)
{
    return heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
}

/**
   ESP32-specific memory information for MEMINFO
*/
uint32_t Util::get_heap_used_size(void) const
{
    uint32_t total_heap = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
    uint32_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    return total_heap - free_heap;
}

/*
    Special Allocation Routines
*/

void* Util::malloc_type(size_t size, AP_HAL::Util::Memory_Type mem_type)
{

    // https://docs.espressif.com/projects/esp-idf/en/v4.0.2/api-reference/system/mem_alloc.html
    // esp32 has DRAM, IRAM and D/IRAM that can be used as either

    /*
    DRAM (Data RAM) is memory used to hold data. This is the most common kind of memory accessed as heap.

    IRAM (Instruction RAM) usually holds executable data only. If accessed as generic memory, all accesses must be 32-bit aligned.

    D/IRAM is RAM which can be used as either Instruction or Data RAM.
    */

    //The ESP-IDF malloc() implementation internally calls heap_caps_malloc(size, MALLOC_CAP_8BIT) in order to allocate DRAM that is byte-addressable.

    //For most purposes, the standard libc malloc() and free() functions can be used for heap allocation without any special consideration.
    //	return malloc(size);

    if (mem_type == AP_HAL::Util::MEM_DMA_SAFE) {
        return heap_caps_calloc(1, size, MALLOC_CAP_DMA);
        //} else if (mem_type == AP_HAL::Util::MEM_FAST) {
        //   return heap_caps_calloc(1, size, MALLOC_CAP_32BIT); //WARNING 32bit memory cannot use unless 32bit access
    } else {
        return heap_caps_calloc(1, size, MALLOC_CAP_8BIT);
    }
}

void Util::free_type(void *ptr, size_t size, AP_HAL::Util::Memory_Type mem_type)
{
    if (ptr != NULL) {
        heap_caps_free(ptr);
    }
}


/*
  get safety switch state
 */
Util::safety_state Util::safety_switch_state(void)
{

#if HAL_USE_PWM == TRUE
    return ((RCOutput *)hal.rcout)->_safety_switch_state();
#else
    return SAFETY_NONE;
#endif
}

#ifdef HAL_PWM_ALARM
struct Util::ToneAlarmPwmGroup Util::_toneAlarm_pwm_group = HAL_PWM_ALARM;

bool Util::toneAlarm_init()
{
    _toneAlarm_pwm_group.pwm_cfg.period = 1000;
    pwmStart(_toneAlarm_pwm_group.pwm_drv, &_toneAlarm_pwm_group.pwm_cfg);

    return true;
}

void Util::toneAlarm_set_buzzer_tone(float frequency, float volume, uint32_t duration_ms)
{
    if (is_zero(frequency) || is_zero(volume)) {
        pwmDisableChannel(_toneAlarm_pwm_group.pwm_drv, _toneAlarm_pwm_group.chan);
    } else {
        pwmChangePeriod(_toneAlarm_pwm_group.pwm_drv,
                        roundf(_toneAlarm_pwm_group.pwm_cfg.frequency/frequency));

        pwmEnableChannel(_toneAlarm_pwm_group.pwm_drv, _toneAlarm_pwm_group.chan, roundf(volume*_toneAlarm_pwm_group.pwm_cfg.frequency/frequency)/2);
    }
}
#endif // HAL_PWM_ALARM

/*
  set HW RTC in UTC microseconds
*/
void Util::set_hw_rtc(uint64_t time_utc_usec)
{
    //stm32_set_utc_usec(time_utc_usec);
}

/*
  get system clock in UTC microseconds
*/
uint64_t Util::get_hw_rtc() const
{
    return esp_timer_get_time();
}

#if !defined(HAL_NO_FLASH_SUPPORT) && !defined(HAL_NO_ROMFS_SUPPORT)

#if !HAL_GCS_ENABLED
#define Debug(fmt, args ...)  do { hal.console->printf(fmt, ## args); } while (0)
#else
#include <GCS_MAVLink/GCS.h>
#define Debug(fmt, args ...)  do { GCS_SEND_TEXT(MAV_SEVERITY_INFO, fmt, ## args); } while (0)
#endif

Util::FlashBootloader Util::flash_bootloader()
{
    //    ....esp32 too
    return FlashBootloader::FAIL;
}
#endif // !HAL_NO_FLASH_SUPPORT && !HAL_NO_ROMFS_SUPPORT

/*
  display system identifier - board type and serial number
 */


bool Util::get_system_id(char buf[50])
{
    //uint8_t serialid[12];
    char board_name[] = HAL_ESP32_BOARD_NAME" ";

    uint8_t base_mac_addr[6] = {0};

// Custom MAC support temporarily disabled - requires esp_efuse.h
// TODO: Fix ESP-IDF include paths and re-enable custom MAC support
#if 0 && defined(ESP_EFUSE_USER_DATA_MAC_CUSTOM) && defined(HAL_ESP32_USE_CUSTOM_MAC)
    // Check if custom MAC exists before trying to read it (avoids error message)
    // Custom MACs are sometimes used for copy protection/licensing
    uint8_t custom_mac[6] = {0};
    size_t size_bits = esp_efuse_get_field_size(ESP_EFUSE_USER_DATA_MAC_CUSTOM);
    if (size_bits == 48) {  // Valid MAC is 48 bits (6 bytes)
        esp_efuse_read_field_blob(ESP_EFUSE_USER_DATA_MAC_CUSTOM, custom_mac, size_bits);
        // Check if MAC is not all zeros (empty)
        if (custom_mac[0] != 0 || memcmp(custom_mac, &custom_mac[1], 5) != 0) {
            // Custom MAC exists and is non-zero, use it for copy protection
            memcpy(base_mac_addr, custom_mac, 6);
        } else {
            // Custom MAC is empty, fall back to default
            esp_efuse_mac_get_default(base_mac_addr);
        }
    } else {
        // Invalid size, use default MAC
        esp_efuse_mac_get_default(base_mac_addr);
    }
#else
    // Not using custom MAC or not available, use default factory MAC
    esp_efuse_mac_get_default(base_mac_addr);
#endif

    char board_mac[20] = "                   ";
    snprintf(board_mac,20, "%x %x %x %x %x %x",
             base_mac_addr[0], base_mac_addr[1], base_mac_addr[2], base_mac_addr[3], base_mac_addr[4], base_mac_addr[5]);

    // null terminate both
    //board_name[13] = 0;
    board_mac[19] = 0;

    // tack strings together
    snprintf(buf, 40, "%s %s", board_name, board_mac);
    // and null terminate that too..
    buf[39] = 0;
    return true;
}

bool Util::get_system_id_unformatted(uint8_t buf[], uint8_t &len)
{
    uint8_t base_mac_addr[6] = {0};

// Custom MAC support temporarily disabled - requires esp_efuse.h
// TODO: Fix ESP-IDF include paths and re-enable custom MAC support
#if 0 && defined(ESP_EFUSE_USER_DATA_MAC_CUSTOM) && defined(HAL_ESP32_USE_CUSTOM_MAC)
    // Check if custom MAC exists before trying to read it (avoids error message)
    // Custom MACs are sometimes used for copy protection/licensing
    uint8_t custom_mac[6] = {0};
    size_t size_bits = esp_efuse_get_field_size(ESP_EFUSE_USER_DATA_MAC_CUSTOM);
    if (size_bits == 48) {  // Valid MAC is 48 bits (6 bytes)
        esp_efuse_read_field_blob(ESP_EFUSE_USER_DATA_MAC_CUSTOM, custom_mac, size_bits);
        // Check if MAC is not all zeros (empty)
        if (custom_mac[0] != 0 || memcmp(custom_mac, &custom_mac[1], 5) != 0) {
            // Custom MAC exists and is non-zero, use it for copy protection
            memcpy(base_mac_addr, custom_mac, 6);
        } else {
            // Custom MAC is empty, fall back to default
            esp_efuse_mac_get_default(base_mac_addr);
        }
    } else {
        // Invalid size, use default MAC
        esp_efuse_mac_get_default(base_mac_addr);
    }
#else
    // Not using custom MAC or not available, use default factory MAC
    esp_efuse_mac_get_default(base_mac_addr);
#endif

    len = MIN(len, ARRAY_SIZE(base_mac_addr));
    memcpy(buf, (const void *)base_mac_addr, len);

    return true;
}

// return true if the reason for the reboot was a watchdog reset
bool Util::was_watchdog_reset() const
{
    esp_reset_reason_t reason = esp_reset_reason();

    return reason == ESP_RST_PANIC
           || reason == ESP_RST_INT_WDT
           || reason == ESP_RST_TASK_WDT
           || reason == ESP_RST_WDT;
}

/*
  display stack usage as text buffer for @SYS/threads.txt
 */
void Util::thread_info(ExpandingString &str)
{
    // a header to allow for machine parsers to determine format
    str.printf("ThreadsV1\n");

    // list all FreeRTOS tasks with stack high-water marks
    UBaseType_t num_tasks = uxTaskGetNumberOfTasks();
    TaskStatus_t *task_array = (TaskStatus_t *)calloc(num_tasks, sizeof(TaskStatus_t));
    if (task_array == nullptr) {
        return;
    }
    UBaseType_t actual = uxTaskGetSystemState(task_array, num_tasks, nullptr);
    for (UBaseType_t i = 0; i < actual; i++) {
        str.printf("%-16s PRI=%2lu FREE=%5lu\n",
                   task_array[i].pcTaskName,
                   (unsigned long)task_array[i].uxCurrentPriority,
                   (unsigned long)task_array[i].usStackHighWaterMark * sizeof(StackType_t));
    }
    free(task_array);
}


