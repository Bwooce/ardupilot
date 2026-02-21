/*
   ESP32 Crash Diagnostics Integration Header
*/

#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32

#ifdef __cplusplus
extern "C" {
#endif

// Main initialization function - call from AP_HAL_ESP32 startup
void esp32_diagnostics_init(void);

// Core dump checking and reporting
void esp32_check_and_report_coredump(void);

// Delayed crash report sender
void esp32_send_crash_report_delayed(void);

// Example integration functions for different components
void esp32_can_init_with_breadcrumbs(void);
bool esp32_read_sensor_with_breadcrumbs(float* value);
void* esp32_malloc_with_breadcrumbs(size_t size);
void esp32_wifi_init_with_breadcrumbs(void);
void esp32_param_load_with_breadcrumbs(void);

// Critical section monitoring
void esp32_enter_critical_section(const char* section_name);
void esp32_exit_critical_section(const char* section_name);

#ifdef __cplusplus
}
#endif

#endif // CONFIG_HAL_BOARD == HAL_BOARD_ESP32