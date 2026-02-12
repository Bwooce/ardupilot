/*
   ESP32 Crash Diagnostics Integration Example
   Shows how to integrate breadcrumb logging and core dump analysis
*/

#include "ESP32_CrashDiagnostics.h"
#include "ESP32_RTC_Breadcrumbs.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32

// Example integration points for crash diagnostics

void esp32_diagnostics_init(void) {
    // Initialize RTC breadcrumb system
    rtc_breadcrumb_init();
    
    // Log system initialization
    RTC_BREADCRUMB_INFO(RTC_BC_BOOT, "Diagnostics init");
    
    // Note: Crash reporting will be handled elsewhere to avoid scheduler complexity
}

// Delayed crash report sender (called after MAVLink is ready)
static uint32_t crash_report_timer = 0;
void esp32_send_crash_report_delayed(void) {
    if (crash_report_timer == 0) {
        crash_report_timer = AP_HAL::millis();
        return;
    }
    
    // Wait 10 seconds after boot to send crash report
    if (AP_HAL::millis() - crash_report_timer > 10000) {
        rtc_breadcrumb_dump_to_mavlink();
        
        // Also send core dump info if available
        esp32_check_and_report_coredump();
        
        // Unregister this timer
        hal.scheduler->register_timer_process(nullptr);
    }
}

void esp32_check_and_report_coredump(void) {
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    GCS_MAVLINK* gcs_chan = gcs().chan(0);
    if (gcs_chan) {
        gcs_chan->send_text(MAV_SEVERITY_INFO, 
            "Core dump enabled. Use 'espcoredump.py info_corefile' after crash.");
    }
#else
    GCS_MAVLINK* gcs_chan = gcs().chan(0);
    if (gcs_chan) {
        gcs_chan->send_text(MAV_SEVERITY_WARNING, 
            "Core dump not enabled. Build with --debug for crash analysis.");
    }
#endif
}

// Usage examples for different system components

// Example: CAN driver initialization
void esp32_can_init_with_breadcrumbs(void) {
    RTC_BREADCRUMB_INFO(RTC_BC_CAN_INIT, "Starting CAN init");
    
    // ... actual CAN initialization code ...
    
    RTC_BREADCRUMB_INFO(RTC_BC_CAN_INIT, "CAN init complete");
}

// Example: Critical sensor reading
bool esp32_read_sensor_with_breadcrumbs(float* value) {
    RTC_BREADCRUMB_TRACE(RTC_BC_SENSOR_READ, "Reading sensor");
    
    // Placeholder sensor reading logic
    bool sensor_ok = (value != nullptr);
    
    if (!sensor_ok) {
        RTC_BREADCRUMB_ERROR(RTC_BC_SENSOR_READ, "Sensor read FAILED");
        return false;
    }
    
    RTC_BREADCRUMB_TRACE(RTC_BC_SENSOR_READ, "Sensor read OK");
    return true;
}

// Example: Memory allocation monitoring
void* esp32_malloc_with_breadcrumbs(size_t size) {
    RTC_BREADCRUMB_TRACE(RTC_BC_HEAP_ALLOC, "malloc req");
    
    void* ptr = malloc(size);
    if (!ptr) {
        char msg[28];
        snprintf(msg, sizeof(msg), "malloc fail %zu", size);
        RTC_BREADCRUMB_ERROR(RTC_BC_HEAP_ALLOC, msg);
    }
    
    return ptr;
}

// Example: WiFi initialization with breadcrumbs
void esp32_wifi_init_with_breadcrumbs(void) {
    RTC_BREADCRUMB_INFO(RTC_BC_WIFI_INIT, "WiFi starting");
    
    // ... WiFi initialization ...
    
    RTC_BREADCRUMB_INFO(RTC_BC_WIFI_INIT, "WiFi connected");
}

// Example: Parameter loading with breadcrumbs
void esp32_param_load_with_breadcrumbs(void) {
    RTC_BREADCRUMB_INFO(RTC_BC_PARAM_LOAD, "Loading params");
    
    // ... parameter loading ...
    
    RTC_BREADCRUMB_INFO(RTC_BC_PARAM_LOAD, "Params loaded");
}

// Critical section monitoring
void esp32_enter_critical_section(const char* section_name) {
    char msg[28];
    snprintf(msg, sizeof(msg), "ENTER %s", section_name);
    RTC_BREADCRUMB_WARN(RTC_BC_CRITICAL_SECTION, msg);
}

void esp32_exit_critical_section(const char* section_name) {
    char msg[28];
    snprintf(msg, sizeof(msg), "EXIT %s", section_name);
    RTC_BREADCRUMB_TRACE(RTC_BC_CRITICAL_SECTION, msg);
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_ESP32