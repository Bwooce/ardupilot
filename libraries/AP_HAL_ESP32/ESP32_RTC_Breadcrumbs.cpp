/*
   Thread-safe RTC breadcrumb logging implementation for ESP32
*/

#include "ESP32_RTC_Breadcrumbs.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// RTC memory structure for breadcrumb logging (survives resets and crashes)
// Following the same pattern as wdt_info in Scheduler.cpp and panic_info in system.cpp
#define RTC_BREADCRUMB_MAGIC 0xBEEFCAFE
static RTC_DATA_ATTR struct {
    uint32_t magic;                                // Magic number to validate data
    volatile uint32_t write_index;                 // Write pointer (volatile for thread safety)
    volatile uint32_t boot_count;                  // Number of boots/resets
    uint32_t last_crash_pc;                        // Program counter at crash
    uint32_t last_crash_task_id;                   // Task ID that crashed
    uint64_t last_crash_timestamp;                 // Timestamp of last crash
    rtc_breadcrumb_t entries[RTC_BREADCRUMB_BUFFER_SIZE];
} rtc_breadcrumb_data;

// Spinlock for thread-safe access (not in RTC memory as it gets reinitialized)
static portMUX_TYPE breadcrumb_spinlock = portMUX_INITIALIZER_UNLOCKED;

// Thread-safe hash function for task identification
static uint16_t get_task_id_hash(void) {
    TaskHandle_t handle = xTaskGetCurrentTaskHandle();
    if (handle == NULL) return 0;
    
    // Simple hash of task handle pointer
    uintptr_t addr = (uintptr_t)handle;
    return (uint16_t)((addr >> 16) ^ (addr & 0xFFFF));
}

void rtc_breadcrumb_init(void) {
    // Initialize RTC data if this is the first boot or data is corrupted
    if (rtc_breadcrumb_data.magic != RTC_BREADCRUMB_MAGIC) {
        // First boot or corrupted data - initialize structure
        memset(&rtc_breadcrumb_data, 0, sizeof(rtc_breadcrumb_data));
        rtc_breadcrumb_data.magic = RTC_BREADCRUMB_MAGIC;
    }
    
    // Increment boot count with ESP32-compatible atomic operation
    portENTER_CRITICAL(&breadcrumb_spinlock);
    rtc_breadcrumb_data.boot_count++;
    portEXIT_CRITICAL(&breadcrumb_spinlock);
    
    // Log boot event
    rtc_breadcrumb_log(RTC_BC_BOOT, 1, "System boot");
    
    // Check if we had a crash on previous boot
    if (rtc_breadcrumb_data.last_crash_pc != 0) {
        char crash_msg[28];
        snprintf(crash_msg, sizeof(crash_msg), "Prev crash PC:0x%08lX", rtc_breadcrumb_data.last_crash_pc);
        rtc_breadcrumb_log(RTC_BC_BOOT, 2, crash_msg);
        // Clear crash info after reporting
        rtc_breadcrumb_data.last_crash_pc = 0;
    }
}

void rtc_breadcrumb_log(uint8_t level, uint8_t code, const char* message) {
    if (!message) return;
    
    // Skip logging if RTC data not initialized
    if (rtc_breadcrumb_data.magic != RTC_BREADCRUMB_MAGIC) return;
    
    // Get current write index atomically and increment
    portENTER_CRITICAL(&breadcrumb_spinlock);
    uint32_t write_idx = rtc_breadcrumb_data.write_index++;
    portEXIT_CRITICAL(&breadcrumb_spinlock);
    
    uint32_t buffer_idx = write_idx % RTC_BREADCRUMB_BUFFER_SIZE;
    
    // Fill the entry
    rtc_breadcrumb_t* entry = &rtc_breadcrumb_data.entries[buffer_idx];
    entry->timestamp_ms = AP_HAL::millis();
    entry->thread_id = get_task_id_hash();
    entry->level = level;
    entry->code = code;
    
    // Safe string copy with null termination
    strncpy(entry->message, message, sizeof(entry->message) - 1);
    entry->message[sizeof(entry->message) - 1] = '\0';
}

void rtc_breadcrumb_dump_to_mavlink(void) {
    GCS_MAVLINK* gcs_chan = gcs().chan(0);
    if (!gcs_chan) return;
    
    // Skip if RTC data not initialized
    if (rtc_breadcrumb_data.magic != RTC_BREADCRUMB_MAGIC) {
        gcs_chan->send_text(MAV_SEVERITY_INFO, "RTC Breadcrumbs: Not initialized");
        return;
    }
    
    // Read values atomically
    portENTER_CRITICAL(&breadcrumb_spinlock);
    uint32_t current_write = rtc_breadcrumb_data.write_index;
    uint32_t boot_count = rtc_breadcrumb_data.boot_count;
    portEXIT_CRITICAL(&breadcrumb_spinlock);
    
    // Send header with boot count and crash info
    gcs_chan->send_text(MAV_SEVERITY_INFO, "RTC Breadcrumbs: Boot #%lu", (unsigned long)boot_count);
    
    if (rtc_breadcrumb_data.last_crash_pc != 0) {
        gcs_chan->send_text(MAV_SEVERITY_WARNING, 
            "Last crash: PC=0x%08lX Task=0x%04lX @%llu",
            (unsigned long)rtc_breadcrumb_data.last_crash_pc,
            (unsigned long)rtc_breadcrumb_data.last_crash_task_id, 
            (unsigned long long)rtc_breadcrumb_data.last_crash_timestamp);
    }
    
    // Determine range to dump (last N entries)
    uint32_t entries_to_dump = (current_write < RTC_BREADCRUMB_BUFFER_SIZE) ? 
                              current_write : RTC_BREADCRUMB_BUFFER_SIZE;
    uint32_t start_idx = (current_write >= RTC_BREADCRUMB_BUFFER_SIZE) ?
                        (current_write - RTC_BREADCRUMB_BUFFER_SIZE) : 0;
    
    // Send breadcrumb entries
    for (uint32_t i = 0; i < entries_to_dump; i++) {
        uint32_t entry_idx = (start_idx + i) % RTC_BREADCRUMB_BUFFER_SIZE;
        const rtc_breadcrumb_t* entry = &rtc_breadcrumb_data.entries[entry_idx];
        
        const char* level_str = "TRACE";
        switch (entry->level) {
            case 1: level_str = "INFO"; break;
            case 2: level_str = "WARN"; break; 
            case 3: level_str = "ERROR"; break;
            case 4: level_str = "CRIT"; break;
        }
        
        gcs_chan->send_text(MAV_SEVERITY_INFO,
            "BC[%lu] %s T%04X C%03d @%lu: %.20s",
            (unsigned long)(start_idx + i), level_str, (unsigned int)entry->thread_id, (int)entry->code,
            (unsigned long)entry->timestamp_ms, entry->message);
    }
}

uint32_t rtc_breadcrumb_get_boot_count(void) {
    if (rtc_breadcrumb_data.magic != RTC_BREADCRUMB_MAGIC) return 0;
    
    portENTER_CRITICAL(&breadcrumb_spinlock);
    uint32_t count = rtc_breadcrumb_data.boot_count;
    portEXIT_CRITICAL(&breadcrumb_spinlock);
    return count;
}

// Panic handler hook to capture crash context
extern "C" void rtc_breadcrumb_panic_handler(void* frame, bool pseudo_excause) {
    // This runs in panic context - keep it simple and fast
    // Skip if RTC data not initialized (avoid crashes in panic handler)
    if (rtc_breadcrumb_data.magic != RTC_BREADCRUMB_MAGIC) return;
    
    rtc_breadcrumb_data.last_crash_pc = 0; // Would need architecture-specific code to get PC
    rtc_breadcrumb_data.last_crash_task_id = get_task_id_hash();
    rtc_breadcrumb_data.last_crash_timestamp = AP_HAL::micros64();
    
    // Log the crash (using direct write to avoid spinlock in panic context)
    uint32_t write_idx = rtc_breadcrumb_data.write_index++;
    uint32_t buffer_idx = write_idx % RTC_BREADCRUMB_BUFFER_SIZE;
    rtc_breadcrumb_t* entry = &rtc_breadcrumb_data.entries[buffer_idx];
    entry->timestamp_ms = AP_HAL::millis();
    entry->thread_id = get_task_id_hash();
    entry->level = 4;
    entry->code = RTC_BC_CRITICAL_SECTION;
    strncpy(entry->message, "PANIC/CRASH", sizeof(entry->message) - 1);
    entry->message[sizeof(entry->message) - 1] = '\0';
}