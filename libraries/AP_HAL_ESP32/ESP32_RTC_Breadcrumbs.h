/*
   Thread-safe RTC breadcrumb logging for ESP32 crash diagnostics
*/

#pragma once

#include <stdint.h>
#include "esp_attr.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#ifdef __cplusplus
extern "C" {
#endif

// Breadcrumb entry structure - keep small for RTC memory
typedef struct {
    uint32_t timestamp_ms;    // AP_HAL::millis() when logged
    uint16_t thread_id;       // Task handle hash for thread identification  
    uint8_t level;           // 0=trace, 1=info, 2=warn, 3=error, 4=critical
    uint8_t code;            // Application-specific code (0-255)
    char message[28];        // Short message (28 bytes to align to 32-byte boundary)
} rtc_breadcrumb_t;

#define RTC_BREADCRUMB_BUFFER_SIZE 32  // Power of 2 for efficient modulo

// Note: The RTC breadcrumb data structure is declared internally in the .cpp file
// to follow the same pattern as wdt_info in Scheduler.cpp and panic_info in system.cpp.
// Access the data through the provided functions only.

// Thread-safe logging functions
void rtc_breadcrumb_init(void);
void rtc_breadcrumb_log(uint8_t level, uint8_t code, const char* message);
void rtc_breadcrumb_dump_to_mavlink(void);
uint32_t rtc_breadcrumb_get_boot_count(void);

// Convenience macros for different log levels
#define RTC_BREADCRUMB_TRACE(code, msg)    rtc_breadcrumb_log(0, code, msg)
#define RTC_BREADCRUMB_INFO(code, msg)     rtc_breadcrumb_log(1, code, msg)  
#define RTC_BREADCRUMB_WARN(code, msg)     rtc_breadcrumb_log(2, code, msg)
#define RTC_BREADCRUMB_ERROR(code, msg)    rtc_breadcrumb_log(3, code, msg)
#define RTC_BREADCRUMB_CRITICAL(code, msg) rtc_breadcrumb_log(4, code, msg)

// Application-specific breadcrumb codes
enum rtc_breadcrumb_codes {
    RTC_BC_BOOT = 1,
    RTC_BC_HAL_INIT = 2,
    RTC_BC_CAN_INIT = 3,
    RTC_BC_WIFI_INIT = 4,
    RTC_BC_SENSOR_READ = 5,
    RTC_BC_MAVLINK_TX = 6,
    RTC_BC_PARAM_LOAD = 7,
    RTC_BC_FLASH_WRITE = 8,
    RTC_BC_HEAP_ALLOC = 9,
    RTC_BC_TASK_CREATE = 10,
    RTC_BC_CRITICAL_SECTION = 11,
    RTC_BC_USER_DEFINED = 100  // Start of user-defined codes
};

#ifdef __cplusplus
}
#endif