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

#include "AP_HAL_ESP32/Scheduler.h"
#include "AP_HAL_ESP32/RCInput.h"
#include "AP_HAL_ESP32/AnalogIn.h"
#include "ESP32_Debug.h"
#include "ESP32_Params.h"
#include "AP_Math/AP_Math.h"
#include "SdCard.h"
#include "Profile.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_task_wdt.h"
#include "esp_log.h"
#include "esp_system.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>
#include <inttypes.h>

//#define SCHEDULERDEBUG 1

using namespace ESP32;

extern const AP_HAL::HAL& hal;

// RTC memory structure for WDT info (survives resets)
#define WDT_INFO_MAGIC 0xDEADBEEF
RTC_DATA_ATTR struct {
    uint32_t magic;
    char task_name[16];
    uint32_t stack_hwm;
    uint32_t timestamp;
    uint32_t heap_free;
    uint32_t heap_min;
    uint32_t last_pc;  // Last program counter if available
} wdt_info;

bool Scheduler::_initialized = true;

Scheduler::Scheduler()
{
    _initialized = false;
    _expected_delay_start_ms = 0;
    _expected_delay_duration_ms = 0;
    _priority_boosted = false;
}

Scheduler::~Scheduler()
{
    if (_initialized) {
        esp_task_wdt_deinit();
    }
}

// Note: wdt_info structure is defined above at line 44-54
// with additional fields for heap and PC tracking

#define WDT_INFO_MAGIC 0xDEADBEEF

// Helper function to register a task with watchdog and report any errors
void Scheduler::register_task_with_watchdog(const char* task_name)
{
    esp_err_t wdt_result = esp_task_wdt_add(NULL);
    if (wdt_result != ESP_OK) {
        // Report different error types
        const char* error_desc = "unknown error";
        switch (wdt_result) {
            case ESP_ERR_INVALID_ARG:
                error_desc = "invalid argument";
                break;
            case ESP_ERR_NO_MEM:
                error_desc = "out of memory";
                break;
            case ESP_ERR_NOT_FOUND:
                error_desc = "watchdog not initialized";
                break;
            case ESP_ERR_INVALID_STATE:
                error_desc = "already registered";
                break;
            default:
                break;
        }

        ESP_LOGE("WDT", "Failed to register task '%s' with watchdog: %s (%d)",
                 task_name, error_desc, wdt_result);
        hal.console->printf("WDT: Failed to register '%s': %s\n", task_name, error_desc);
    } else {
        ESP_LOGI("WDT", "Successfully registered task '%s' with watchdog", task_name);
    }
}

// External C function for use by other components like AP_Logger
extern "C" void esp32_register_thread_with_watchdog(const char* name)
{
    ESP32::Scheduler::register_task_with_watchdog(name);
}

void Scheduler::wdt_init(uint32_t timeout, uint32_t core_mask)
{
    // Try to reconfigure existing WDT first
    esp_task_wdt_config_t config = {
        .timeout_ms = timeout,
        .idle_core_mask = core_mask,
        .trigger_panic = true
    };

    // First try to deinit if already initialized
    esp_task_wdt_deinit();

    // Now initialize with our config
    esp_err_t wdt_result = esp_task_wdt_init(&config);
    if (wdt_result != ESP_OK) {
        // Only print real errors, not expected ones
        if (wdt_result != ESP_ERR_INVALID_STATE) {
            printf("esp_task_wdt_init() failed with error %d\n", wdt_result);
        }
    }

    if (ESP_OK != esp_task_wdt_add(NULL)) {
        printf("esp_task_wdt_add(NULL) failed");
    }

    // Note: ESP-IDF v5.x removed the user callback API
    // WDT info will be captured via panic handler or coredump instead
}

void Scheduler::report_reset_reason()
{
    // Try immediate report (may fail if GCS not ready)
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "WDT: Check magic=0x%lX exp=0x%lX", 
                  (unsigned long)wdt_info.magic, (unsigned long)WDT_INFO_MAGIC);
    
    // Check for saved WDT info from previous crash
    if (wdt_info.magic == WDT_INFO_MAGIC) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "WDT: Found valid saved info!");
        // Report detailed WDT info
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, 
                     "WDT: Task=%s Stack=%lu Heap=%lu/%lu",
                     wdt_info.task_name,
                     (unsigned long)wdt_info.stack_hwm,
                     (unsigned long)wdt_info.heap_free,
                     (unsigned long)wdt_info.heap_min);
        
        // Report last PC if available
        if (wdt_info.last_pc != 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                         "WDT: PC=0x%lX Tick=%lu",
                         (unsigned long)wdt_info.last_pc,
                         (unsigned long)wdt_info.timestamp);
        }
        
        // Clear the magic so we don't report it again
        wdt_info.magic = 0;
    }
    
    esp_reset_reason_t reason = esp_reset_reason();
    const char* reason_str = nullptr;
    MAV_SEVERITY severity = MAV_SEVERITY_INFO;
    
    switch(reason) {
        case ESP_RST_UNKNOWN:
            reason_str = "ESP32 Reset: Unknown reason";
            severity = MAV_SEVERITY_WARNING;
            break;
        case ESP_RST_POWERON:
            reason_str = "ESP32 Reset: Power-on reset";
            break;
        case ESP_RST_EXT:
            reason_str = "ESP32 Reset: External reset (EN pin)";
            break;
        case ESP_RST_SW:
            reason_str = "ESP32 Reset: Software reset (esp_restart)";
            break;
        case ESP_RST_PANIC:
            reason_str = "ESP32 PANIC: Stack overflow or exception";
            severity = MAV_SEVERITY_CRITICAL;
            // Note: Backtrace will be in console output
            // Use decode_esp32_backtrace.sh to analyze
            break;
        case ESP_RST_INT_WDT:
            reason_str = "ESP32 Reset: Interrupt watchdog";
            severity = MAV_SEVERITY_CRITICAL;
            break;
        case ESP_RST_TASK_WDT:
            reason_str = "ESP32 Reset: Task watchdog timeout";
            severity = MAV_SEVERITY_CRITICAL;
            break;
        case ESP_RST_WDT:
            reason_str = "ESP32 Reset: Other watchdog reset";
            severity = MAV_SEVERITY_CRITICAL;
            break;
        case ESP_RST_DEEPSLEEP:
            reason_str = "ESP32 Reset: Wake from deep sleep";
            break;
        case ESP_RST_BROWNOUT:
            reason_str = "ESP32 Reset: Brownout (low voltage)";
            severity = MAV_SEVERITY_CRITICAL;
            break;
        case ESP_RST_SDIO:
            reason_str = "ESP32 Reset: SDIO reset";
            break;
        default:
            reason_str = "ESP32 Reset: Unrecognized code";
            severity = MAV_SEVERITY_WARNING;
            break;
    }
    
    // Send reset reason via MAVLink STATUSTEXT
    if (reason_str != nullptr) {
        GCS_SEND_TEXT(severity, "%s (%d)", reason_str, (int)reason);
    }
    
    // Additional info for watchdog resets - get more details if available
    if (reason == ESP_RST_TASK_WDT || reason == ESP_RST_INT_WDT || reason == ESP_RST_WDT) {
        // Report heap status in case it's memory related
        size_t free_heap = esp_get_free_heap_size();
        size_t min_free_heap = esp_get_minimum_free_heap_size();
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Heap: free=%u min=%u", 
                     (unsigned)free_heap, (unsigned)min_free_heap);
        
        // Try to get WDT task name if available
        #if CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0 || CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1
        // Note: ESP-IDF doesn't provide direct API to get which task triggered WDT
        // but we can report that WDT was triggered
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "WDT: Check console for task details");
        #endif
        
        // Log more details via ESP_LOG which might be captured
        ESP_LOGE("WDT", "Watchdog reset detected - heap free=%u min=%u", 
                 (unsigned)free_heap, (unsigned)min_free_heap);
    }
}

void Scheduler::init()
{

#ifdef SCHEDDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif

    // Suppress task_wdt errors during startup - tasks aren't registered yet
    esp_log_level_set("task_wdt", ESP_LOG_NONE);

    // Debug via MAVLink STATUSTEXT - safe from serial contamination
    ESP32_DEBUG_INFO("Starting ESP32 Scheduler initialization");
    ESP32_DEBUG_VERBOSE("Scheduler running with CONFIG_FREERTOS_HZ=%d", CONFIG_FREERTOS_HZ);
    
    // Report ESP32 reset reason via MAVLink
    report_reset_reason();
    
    // Report debug protection status if enabled
#if defined(CONFIG_COMPILER_STACK_CHECK) || defined(CONFIG_HEAP_POISONING_COMPREHENSIVE) || defined(CONFIG_FREERTOS_WATCHPOINT_END_OF_STACK)
    // Build protection flags string
    const char* protections = "ESP32 Debug: "
#ifdef CONFIG_COMPILER_STACK_CHECK
        "STACK "
#endif
#ifdef CONFIG_HEAP_POISONING_COMPREHENSIVE
        "HEAP "
#endif
#ifdef CONFIG_FREERTOS_WATCHPOINT_END_OF_STACK
        "WATCH"
#endif
        ;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s", protections);
    
    // Also print to console for local debugging
    printf("%s protections enabled\n", protections);
#endif

    // keep main tasks that need speed on CPU 0
    // pin potentially slow stuff to CPU 1, as we have disabled the WDT on that core.
    #define FASTCPU 0
    #define SLOWCPU 1

    // pin main thread to Core 0, and we'll also pin other heavy-tasks to core 1, like wifi-related.
    if (xTaskCreatePinnedToCore(_main_thread, "APM_MAIN", Scheduler::MAIN_SS, this, Scheduler::MAIN_PRIO, &_main_task_handle,FASTCPU) != pdPASS) {
    //if (xTaskCreate(_main_thread, "APM_MAIN", Scheduler::MAIN_SS, this, Scheduler::MAIN_PRIO, &_main_task_handle) != pdPASS) {
        ESP_LOGE("SCHEDULER", "FAILED to create task _main_thread on FASTCPU");
        hal.console->printf("FAILED to create task _main_thread on FASTCPU\n");
    } else {
    	ESP_LOGI("SCHEDULER", "OK created task _main_thread on FASTCPU");
    	hal.console->printf("OK created task _main_thread on FASTCPU\n");
    }

    if (xTaskCreatePinnedToCore(_timer_thread, "APM_TIMER", TIMER_SS, this, TIMER_PRIO, &_timer_task_handle,FASTCPU) != pdPASS) {
        hal.console->printf("FAILED to create task _timer_thread on FASTCPU\n");
    } else {
    	hal.console->printf("OK created task _timer_thread on FASTCPU\n");
    }	

    if (xTaskCreatePinnedToCore(_rcout_thread, "APM_RCOUT", RCOUT_SS, this, RCOUT_PRIO, &_rcout_task_handle,SLOWCPU) != pdPASS) {
       hal.console->printf("FAILED to create task _rcout_thread on SLOWCPU\n");
    } else {
       hal.console->printf("OK created task _rcout_thread on SLOWCPU\n");
    }

    if (xTaskCreatePinnedToCore(_rcin_thread, "APM_RCIN", RCIN_SS, this, RCIN_PRIO, &_rcin_task_handle,SLOWCPU) != pdPASS) {
       hal.console->printf("FAILED to create task _rcin_thread on SLOWCPU\n");
    } else {
       hal.console->printf("OK created task _rcin_thread on SLOWCPU\n");
    }

    // pin this thread to Core 1 as it keeps all teh uart/s feed data, and we need that quick.
    if (xTaskCreatePinnedToCore(_uart_thread, "APM_UART", UART_SS, this, UART_PRIO, &_uart_task_handle,FASTCPU) != pdPASS) {
        hal.console->printf("FAILED to create task _uart_thread on FASTCPU\n");
    } else {
    	hal.console->printf("OK created task _uart_thread on FASTCPU\n");
    }	  

    // we put those on the SLOW core as it mounts the sd card, and that often isn't connected.
    if (xTaskCreatePinnedToCore(_io_thread, "SchedulerIO:APM_IO", IO_SS, this, IO_PRIO, &_io_task_handle,SLOWCPU) != pdPASS) {
        hal.console->printf("FAILED to create task _io_thread on SLOWCPU\n");
    } else {
        hal.console->printf("OK created task _io_thread on SLOWCPU\n");
    }	 

    if (xTaskCreatePinnedToCore(_storage_thread, "APM_STORAGE", STORAGE_SS, this, STORAGE_PRIO, &_storage_task_handle,SLOWCPU) != pdPASS) { //no actual flash writes without this, storage kinda appears to work, but does an erase on every boot and params don't persist over reset etc.
        hal.console->printf("FAILED to create task _storage_thread on SLOWCPU\n");
    } else {
    	hal.console->printf("OK created task _storage_thread on SLOWCPU\n");
    }

    //   xTaskCreatePinnedToCore(_print_profile, "APM_PROFILE", IO_SS, this, IO_PRIO, nullptr,SLOWCPU);
    
    printf("SCHEDULER: All tasks created successfully\n");
}

template <typename T>
void executor(T oui)
{
    oui();
}

void IRAM_ATTR Scheduler::thread_create_trampoline(void *ctx)
{
    AP_HAL::MemberProc *t = (AP_HAL::MemberProc *)ctx;
    (*t)();
    free(t);

    // delete the calling task
    vTaskDelete(NULL);
}

/*
  create a new thread
*/
bool Scheduler::thread_create(AP_HAL::MemberProc proc, const char *name, uint32_t requested_stack_size, priority_base base, int8_t priority)
{
#ifdef SCHEDDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif

    // take a copy of the MemberProc, it is freed after thread exits
    AP_HAL::MemberProc *tproc = (AP_HAL::MemberProc *)calloc(1, sizeof(proc));
    if (!tproc) {
        return false;
    }
    *tproc = proc;

    uint8_t thread_priority = IO_PRIO;
    static const struct {
        priority_base base;
        uint8_t p;
    } priority_map[] = {
        { PRIORITY_BOOST, IO_PRIO},
        { PRIORITY_MAIN, MAIN_PRIO},
        { PRIORITY_SPI, SPI_PRIORITY},
        { PRIORITY_I2C, I2C_PRIORITY},
        { PRIORITY_CAN, IO_PRIO},
        { PRIORITY_TIMER, TIMER_PRIO},
        { PRIORITY_RCIN, RCIN_PRIO},
        { PRIORITY_IO, IO_PRIO},
        { PRIORITY_UART, UART_PRIO},
        { PRIORITY_NET, WIFI_PRIO1},
        { PRIORITY_STORAGE, STORAGE_PRIO},
        { PRIORITY_SCRIPTING, UART_PRIO},
    };
    for (uint8_t i=0; i<ARRAY_SIZE(priority_map); i++) {
        if (priority_map[i].base == base) {
#ifdef SCHEDDEBUG
            printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
            thread_priority = constrain_int16(priority_map[i].p + priority, 1, 25);
            break;
        }
    }
    // chibios has a 'thread working area', we just another 1k.
    #define EXTRA_THREAD_SPACE 1024
    uint32_t actual_stack_size = requested_stack_size+EXTRA_THREAD_SPACE;

    tskTaskControlBlock* xhandle;
    BaseType_t xReturned = xTaskCreate(thread_create_trampoline, name, actual_stack_size, tproc, thread_priority, &xhandle);
    if (xReturned != pdPASS) {
        free(tproc);
        return false;
    }

    // Auto-register dynamically created tasks with watchdog
    // This fixes tasks like "FTP" that are created at runtime
    // Note: Registration happens in the creating task's context, not the new task
    esp_err_t wdt_result = esp_task_wdt_add(xhandle);
    if (wdt_result == ESP_OK) {
        ESP_LOGI("SCHED", "Auto-registered task '%s' with watchdog", name);
    } else {
        ESP_LOGW("SCHED", "Failed to auto-register task '%s' with watchdog: %d", name, wdt_result);
    }

    return true;
}

void IRAM_ATTR Scheduler::delay(uint16_t ms)
{
    uint64_t start = AP_HAL::micros64();
    while ((AP_HAL::micros64() - start)/1000 < ms) {
        delay_microseconds(1000);
        if (_min_delay_cb_ms <= ms) {
            if (in_main_thread()) {
                call_delay_cb();
            }
        }
    }
}

void IRAM_ATTR Scheduler::delay_microseconds(uint16_t us)
{
    if (in_main_thread() && us < 100) {
        esp_rom_delay_us(us);
    } else { // Minimum delay for FreeRTOS is 1ms
        uint32_t tick = portTICK_PERIOD_MS * 1000;

        // For delays >= 1ms, feed the watchdog to prevent timeout
        // Only reset watchdog for longer delays to avoid overhead
        if (us >= 1000) {
            esp_err_t wdt_err = esp_task_wdt_reset();

            // Debug unregistered tasks trying to reset watchdog
            if (wdt_err == ESP_ERR_NOT_FOUND) {
                static uint32_t last_report_ms = 0;
                uint32_t now_ms = AP_HAL::millis();

                // Report which task is failing (limit to once per second per task)
                if (now_ms - last_report_ms > 1000) {
                    TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
                    const char* task_name = pcTaskGetName(current_task);

                    // Get task details for debugging
                    TaskStatus_t task_status;
                    vTaskGetInfo(current_task, &task_status, pdTRUE, eInvalid);

                    ESP_LOGE("SCHED_WDT", "Task '%s' (handle=%p, stack_hwm=%lu) not registered with watchdog",
                             task_name ? task_name : "UNKNOWN",
                             current_task,
                             (unsigned long)task_status.usStackHighWaterMark);

                    last_report_ms = now_ms;
                }
            }
        }

        vTaskDelay((us+tick-1)/tick);
    }
}

void IRAM_ATTR Scheduler::delay_microseconds_boost(uint16_t us)
{
    // For ESP32, boost just means we should try to be more accurate
    // by avoiding task switches when possible
    if (!_priority_boosted) {
        _priority_boosted = true;
        // Could potentially increase task priority here if needed
        // vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
    }

    if (us < 100) {
        // For very short delays, use busy wait for accuracy
        esp_rom_delay_us(us);
    } else {
        // For longer delays, use normal delay
        delay_microseconds(us);
    }
}

void Scheduler::boost_end(void)
{
    if (_priority_boosted) {
        _priority_boosted = false;
        // Restore normal priority if we changed it
        // vTaskPrioritySet(NULL, original_priority);
    }
}

void Scheduler::expect_delay_ms(uint32_t ms)
{
    if (ms == 0) {
        // Cancel expected delay
        _expected_delay_duration_ms = 0;
        _expected_delay_start_ms = 0;
    } else {
        _expected_delay_start_ms = AP_HAL::millis();
        _expected_delay_duration_ms = ms;
    }
}

bool Scheduler::in_expected_delay(void) const
{
    if (_expected_delay_duration_ms == 0) {
        return false;
    }

    uint32_t now = AP_HAL::millis();
    return (now - _expected_delay_start_ms) < _expected_delay_duration_ms;
}

void IRAM_ATTR Scheduler::register_timer_process(AP_HAL::MemberProc proc)
{
#ifdef SCHEDDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }
    if (_num_timer_procs >= ESP32_SCHEDULER_MAX_TIMER_PROCS) {
        printf("Out of timer processes\n");
        return;
    }
    _timer_sem.take_blocking();
    _timer_proc[_num_timer_procs] = proc;
    _num_timer_procs++;
    _timer_sem.give();
}

void IRAM_ATTR Scheduler::register_io_process(AP_HAL::MemberProc proc)
{
#ifdef SCHEDDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    _io_sem.take_blocking();
    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i] == proc) {
            _io_sem.give();
            return;
        }
    }
    if (_num_io_procs < ESP32_SCHEDULER_MAX_IO_PROCS) {
        _io_proc[_num_io_procs] = proc;
        _num_io_procs++;
    } else {
        printf("Out of IO processes\n");
    }
    _io_sem.give();
}

void IRAM_ATTR Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    _failsafe = failsafe;
}

void Scheduler::reboot(bool hold_in_bootloader)
{
    printf("Restarting now...\n");
    hal.rcout->force_safety_on();
    unmount_sdcard();
    esp_restart();
}

bool IRAM_ATTR Scheduler::in_main_thread() const
{
    return _main_task_handle == xTaskGetCurrentTaskHandle();
}

void Scheduler::watchdog_pat()
{
    // Pat the watchdog timer for the current thread
    // This is safe to call from any thread that has been registered with the watchdog
    esp_err_t result = esp_task_wdt_reset();
    if (result != ESP_OK && result != ESP_ERR_NOT_FOUND) {
        // Only log real errors, not "not registered" which is expected for some threads
        ESP_LOGE("WDT", "Failed to pat watchdog: %d", result);
    }
}

void Scheduler::set_system_initialized()
{
#ifdef SCHEDDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (_initialized) {
        AP_HAL::panic("PANIC: scheduler::system_initialized called more than once");
    }

    _initialized = true;
}

bool Scheduler::is_system_initialized()
{
    return _initialized;
}

void IRAM_ATTR Scheduler::_timer_thread(void *arg)
{
#ifdef SCHEDDEBUG
    printf("%s:%d start\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    Scheduler *sched = (Scheduler *)arg;

    // Register this task with watchdog
    register_task_with_watchdog("APM_TIMER");

#if HAL_INS_DEFAULT != HAL_INS_NONE
    // wait to ensure INS system inits unless using HAL_INS_NONE
    while (!_initialized) {
        sched->delay_microseconds(1000);
    }
#endif

#ifdef SCHEDDEBUG
    printf("%s:%d initialised\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    while (true) {
        sched->delay_microseconds(1000);
        sched->_run_timers();
        //analog in
#ifndef HAL_DISABLE_ADC_DRIVER
        ((AnalogIn*)hal.analogin)->_timer_tick();
#endif
    }
}

void IRAM_ATTR Scheduler::_rcout_thread(void* arg)
{
    Scheduler *sched = (Scheduler *)arg;

    // Register this task with watchdog
    register_task_with_watchdog("APM_RCOUT");

    while (!_initialized) {
        sched->delay_microseconds(1000);
    }

    while (true) {
        sched->delay_microseconds(4000);
        // process any pending RC output requests
        hal.rcout->timer_tick();
    }
}

void IRAM_ATTR Scheduler::_run_timers()
{
#ifdef SCHEDULERDEBUG
    printf("%s:%d start \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (_in_timer_proc) {
        return;
    }
#ifdef SCHEDULERDEBUG
    printf("%s:%d _in_timer_proc \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    _in_timer_proc = true;

    int num_procs = 0;

    _timer_sem.take_blocking();
    num_procs = _num_timer_procs;
    _timer_sem.give();

    // now call the timer based drivers
    for (int i = 0; i < num_procs; i++) {
        if (_timer_proc[i]) {
            _timer_proc[i]();
        }
    }

    // and the failsafe, if one is setup
    if (_failsafe != nullptr) {
        _failsafe();
    }

    _in_timer_proc = false;
}

void IRAM_ATTR Scheduler::_rcin_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;

    // Register this task with watchdog
    register_task_with_watchdog("APM_RCIN");

    while (!_initialized) {
        sched->delay_microseconds(20000);
    }
    hal.rcin->init();
    while (true) {
        sched->delay_microseconds(1000);
        ((RCInput *)hal.rcin)->_timer_tick();
    }
}

void IRAM_ATTR Scheduler::_run_io(void)
{
#ifdef SCHEDULERDEBUG
    printf("%s:%d start \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (_in_io_proc) {
        return;
    }
#ifdef SCHEDULERDEBUG
    printf("%s:%d initialised \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    _in_io_proc = true;

    int num_procs = 0;
    _io_sem.take_blocking();
    num_procs = _num_io_procs;
    _io_sem.give();
    // now call the IO based drivers
    for (int i = 0; i < num_procs; i++) {
        if (_io_proc[i]) {
            _io_proc[i]();
        }
    }
    _in_io_proc = false;
}

void IRAM_ATTR Scheduler::_io_thread(void* arg)
{
#ifdef SCHEDDEBUG
    printf("%s:%d start \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    // Register this task with watchdog
    register_task_with_watchdog("APM_IO");

    mount_sdcard();
    Scheduler *sched = (Scheduler *)arg;
    while (!sched->_initialized) {
        sched->delay_microseconds(1000);
    }
#ifdef SCHEDDEBUG
    printf("%s:%d initialised \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    uint32_t last_sd_start_ms = AP_HAL::millis();
    while (true) {
        sched->delay_microseconds(1000);
        // run registered IO processes
        sched->_run_io();

        if (!hal.util->get_soft_armed()) {
            // if sdcard hasn't mounted then retry it every 3s in the IO
            // thread when disarmed
            uint32_t now = AP_HAL::millis();
            if (now - last_sd_start_ms > 3000) {
                last_sd_start_ms = now;
                sdcard_retry();
            }
        }
    }
}


void Scheduler::_storage_thread(void* arg)
{
#ifdef SCHEDDEBUG
    printf("%s:%d start \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    Scheduler *sched = (Scheduler *)arg;

    // Register this task with watchdog
    register_task_with_watchdog("APM_STORAGE");
    while (!sched->_initialized) {
        sched->delay_microseconds(10000);
    }
#ifdef SCHEDDEBUG
    printf("%s:%d initialised \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    while (true) {
        sched->delay_microseconds(1000);
        // process any pending storage writes
        hal.storage->_timer_tick();
        //print_profile();
    }
}

void Scheduler::_print_profile(void* arg)
{
    Scheduler *sched = (Scheduler *)arg;
    while (!sched->_initialized) {
        sched->delay_microseconds(10000);
    }

    while (true) {
        sched->delay(10000);
        print_profile();
    }

}

void IRAM_ATTR Scheduler::_uart_thread(void *arg)
{
#ifdef SCHEDDEBUG
    printf("%s:%d start \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    Scheduler *sched = (Scheduler *)arg;

    // Register this task with watchdog
    register_task_with_watchdog("APM_UART");
    while (!sched->_initialized) {
        sched->delay_microseconds(2000);
    }
#ifdef SCHEDDEBUG
    printf("%s:%d initialised\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    while (true) {
        sched->delay_microseconds(1000);
        for (uint8_t i=0; i<hal.num_serial; i++) {
            hal.serial(i)->_timer_tick();
        }
        hal.console->_timer_tick();
    }
}


// get the active main loop rate
uint16_t IRAM_ATTR Scheduler::get_loop_rate_hz(void)
{
    if (_active_loop_rate_hz == 0) {
        _active_loop_rate_hz = _loop_rate_hz;
    }
    return _active_loop_rate_hz;
}

// once every 60 seconds, print some stats...
void Scheduler::print_stats(void)
{
    static int64_t last_run = 0;
    if (AP_HAL::millis64() - last_run > 60000) {
        char buffer[1024];
        vTaskGetRunTimeStats(buffer);
        printf("\n\n%s\n", buffer);
        heap_caps_print_heap_info(0);
        last_run = AP_HAL::millis64();
    }

    // printf("loop_rate_hz: %d",get_loop_rate_hz());
}

// Run every 30s (was 10s - increased for mLRS compatibility)
void Scheduler::print_main_loop_rate(void)
{
    static int64_t last_run = 0;
    if (AP_HAL::millis64() - last_run > 30000) {  // Changed from 10000 to 30000 for mLRS
        last_run = AP_HAL::millis64();
        
        // Reset WDT before potentially blocking console write
        esp_task_wdt_reset();
        
        // null pointer in here...
        const float actual_loop_rate = AP::scheduler().get_filtered_loop_rate_hz();
        const uint16_t expected_loop_rate = AP::scheduler().get_loop_rate_hz();
        
        // Use GCS instead of console to avoid blocking
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Loop rate: %.1fHz / %uHz", 
                     actual_loop_rate, expected_loop_rate);
        
        // Don't use console->printf as it might block on UART
        // hal.console->printf("loop_rate: actual: %fHz, expected: %uHz\n", actual_loop_rate, expected_loop_rate);
        
        // Reset WDT after the operation
        esp_task_wdt_reset();
    }
}

void IRAM_ATTR Scheduler::_main_thread(void *arg)
{
    ESP_LOGI("MAIN", "===========================================");
    ESP_LOGI("MAIN", "ArduPilot main thread starting");
    ESP_LOGI("MAIN", "===========================================");
#ifdef SCHEDDEBUG
    printf("%s:%d start\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    Scheduler *sched = (Scheduler *)arg;

    if (sched->callbacks == nullptr) {
        ESP_LOGE("MAIN", "CRITICAL: callbacks is NULL - cannot proceed");
        vTaskDelete(NULL);
        return;
    }

#ifndef HAL_DISABLE_ADC_DRIVER
    hal.analogin->init();
#endif
    hal.rcout->init();

    // Register main thread with watchdog BEFORE calling setup()
    // This prevents "task not found" errors during initialization
    register_task_with_watchdog("APM_MAIN");

    ESP_LOGI("MAIN", "Calling ArduPilot setup() - this may take a while...");
    ESP_LOGI("SCHEDULER", "========================================");
    ESP_LOGI("SCHEDULER", "About to call callbacks->setup()");
    ESP_LOGI("SCHEDULER", "callbacks=%p", sched->callbacks);
    ESP_LOGI("SCHEDULER", "========================================");
    if (sched->callbacks != nullptr) {
        ESP_LOGI("SCHEDULER", "Callbacks valid, calling setup()");
        sched->callbacks->setup();
        ESP_LOGI("SCHEDULER", "callbacks->setup() returned");
    } else {
        ESP_LOGE("SCHEDULER", "ERROR: callbacks is NULL!");
    }
    ESP_LOGI("MAIN", "ArduPilot setup completed successfully");

    // Re-apply ESP-IDF log levels after parameter loading in setup()
    ESP32::esp32_params()->update_log_levels();

    // Re-enable task_wdt logging to catch real issues
    esp_log_level_set("task_wdt", ESP_LOG_WARN);

    sched->set_system_initialized();

#ifdef SCHEDDEBUG
    printf("%s:%d initialised\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    static uint32_t max_loop_time = 0;  // Track max loop time across all iterations
    
    while (true) {
        // Reset WDT BEFORE running the loop to ensure we don't timeout
        static uint32_t wdt_reset_count = 0;
        static uint32_t last_wdt_reset_report = 0;
        esp_err_t wdt_err = esp_task_wdt_reset();
        if (wdt_err != ESP_OK) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "WDT: Reset failed err=%d at %lu ms", 
                         (int)wdt_err, (unsigned long)AP_HAL::millis());
            ESP_LOGE("WDT", "Failed to reset WDT: error %d", wdt_err);
        } else {
            wdt_reset_count++;
            // Report WDT reset success periodically
            uint32_t now = AP_HAL::millis();
            if (now - last_wdt_reset_report > 10000) {
                last_wdt_reset_report = now;
                // Suppress this message - it's just counting successful WDT feeds, not actual resets
                // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "WDT: %lu resets OK", 
                //              (unsigned long)wdt_reset_count);
                wdt_reset_count = 0;
            }
        }
        
        // Track loop timing for debugging
        static uint32_t last_loop_start = 0;
        uint32_t loop_start = AP_HAL::millis();
        uint32_t time_since_last = loop_start - last_loop_start;
        
        // Warn if too much time passed since last loop (indicates blocking)
        if (last_loop_start != 0 && time_since_last > 2000) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "WDT: %lums since last loop!", 
                         (unsigned long)time_since_last);
        }
        last_loop_start = loop_start;
        
        sched->callbacks->loop();
        
        uint32_t loop_time = AP_HAL::millis() - loop_start;
        if (loop_time > max_loop_time) {
            max_loop_time = loop_time;
            if (loop_time > 2000) {  // Warn if loop takes > 2 seconds
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "WDT: Loop took %lums (max %lums)", 
                             (unsigned long)loop_time, (unsigned long)max_loop_time);
            }
        }
        
        sched->delay_microseconds(250);

        // run stats periodically
#ifdef SCHEDDEBUG
        sched->print_stats();
#endif
        sched->print_main_loop_rate();
        
        // Delayed WDT info report (10 seconds after boot, once only)
        static bool wdt_delayed_report_done = false;
        static uint32_t boot_time = AP_HAL::millis();
        static uint32_t loop_count = 0;
        loop_count++;
        
        // Stagger this to 15 seconds to avoid collision with other periodic messages
        if (!wdt_delayed_report_done && AP_HAL::millis() - boot_time > 15000) {
            wdt_delayed_report_done = true;
            
            // Reset WDT before multiple GCS sends
            esp_task_wdt_reset();
            
            // Combine messages to reduce mLRS traffic
            if (wdt_info.magic == WDT_INFO_MAGIC) {
                // Send one combined message instead of multiple
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "WDT: %s S=%lu H=%lu L=%d", 
                             wdt_info.task_name,
                             (unsigned long)wdt_info.stack_hwm,
                             (unsigned long)wdt_info.heap_free,
                             (int)ESP32::esp32_params()->log_to_mavlink.get());
                esp_task_wdt_reset();
                // Don't clear magic here - let report_reset_reason do it
            }
        }
        
        // Save task info to RTC memory periodically (every ~5 seconds)
        // This will be available after a WDT reset
        static uint32_t last_wdt_save = 0;
        uint32_t now = AP_HAL::millis();
        if (now - last_wdt_save > 5000) {
            last_wdt_save = now;
            
            // Save current task info
            wdt_info.magic = WDT_INFO_MAGIC;
            
            // Get current task name
            const char* task_name = pcTaskGetName(NULL);
            if (task_name) {
                strncpy(wdt_info.task_name, task_name, sizeof(wdt_info.task_name)-1);
                wdt_info.task_name[sizeof(wdt_info.task_name)-1] = '\0';
            } else {
                strcpy(wdt_info.task_name, "main");
            }
            
            // Get stack high water mark (bytes remaining)
            wdt_info.stack_hwm = uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t);
            
            // Save timestamp
            wdt_info.timestamp = xTaskGetTickCount();
            
            // Save heap info
            wdt_info.heap_free = esp_get_free_heap_size();
            wdt_info.heap_min = esp_get_minimum_free_heap_size();
            
            // Save last PC (instruction pointer) - approximate location
            // Note: This is a rough approximation, actual PC at WDT would be different
            wdt_info.last_pc = (uint32_t)__builtin_return_address(0);
            
            // Send periodic confirmation (every 30 seconds) - disabled for production
#if 0  // Enable for debugging stack/heap issues
            static uint32_t last_wdt_report = 0;
            if (now - last_wdt_report > 30000) {
                last_wdt_report = now;

                // Reset WDT before potentially blocking GCS send
                esp_task_wdt_reset();

                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "WDT: Saved %s S=%lu H=%lu/%lu MaxLoop=%lu",
                             wdt_info.task_name,
                             (unsigned long)wdt_info.stack_hwm,
                             (unsigned long)wdt_info.heap_free,
                             (unsigned long)wdt_info.heap_min,
                             (unsigned long)max_loop_time);

                // Reset WDT after GCS send
                esp_task_wdt_reset();
            }
#endif
        };
    }
}

