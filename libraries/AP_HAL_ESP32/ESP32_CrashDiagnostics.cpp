/*
   ESP32 Crash Diagnostics -- Core Dump Analysis and Reporting

   On panic (stack overflow, exception, watchdog), ESP-IDF writes an ELF
   core dump to the coredump flash partition. At next boot, this code reads
   the stored dump and reports the crash details via GCS STATUSTEXT messages
   (which also propagate over DroneCAN if a bridge node is connected).

   Full offline analysis is still available via:
     idf.py coredump-info -p /dev/ttyUSBx    (backtrace + registers)
     idf.py coredump-debug -p /dev/ttyUSBx   (launches GDB on the dump)
*/

#include "ESP32_CrashDiagnostics.h"
#include "ESP32_RTC_Breadcrumbs.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32

#include "esp_core_dump.h"
#include "esp_log.h"

extern const AP_HAL::HAL& hal;

static const char *TAG = "CRASH";

// Xtensa exception cause names (from Xtensa ISA reference, Table 4-64)
static const char *exc_cause_name(uint32_t cause)
{
    switch (cause) {
    case  0: return "IllegalInstr";
    case  2: return "InstrFetchErr";
    case  3: return "LoadStoreErr";
    case  6: return "DivByZero";
    case  9: return "Unaligned";
    case 20: return "InstrProhibit";
    case 28: return "LoadProhibit";
    case 29: return "StoreProhibit";
    default: return nullptr;
    }
}

void esp32_diagnostics_init(void)
{
    rtc_breadcrumb_init();
    RTC_BREADCRUMB_INFO(RTC_BC_BOOT, "Diagnostics init");
}

/*
  Check for a stored core dump from a previous crash and report its
  contents via GCS STATUSTEXT. Messages are sent at CRITICAL severity
  so they appear prominently in GCS logs and are forwarded over DroneCAN.

  Call this after GCS/MAVLink is initialised (e.g. from Scheduler::init
  or report_reset_reason).
*/
void esp32_check_and_report_coredump(void)
{
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    // First check if a valid core dump exists
    esp_err_t err = esp_core_dump_image_check();
    if (err == ESP_ERR_NOT_FOUND) {
        // No core dump stored -- normal boot
        return;
    }
    if (err != ESP_OK) {
        // Core dump exists but is corrupt
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "Coredump: stored but corrupt (0x%lX)",
                      (unsigned long)err);
        // Erase the corrupt dump so we don't report it every boot
        esp_core_dump_image_erase();
        return;
    }

    // Valid core dump found -- report it
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                  "Coredump: previous crash detected");

#if CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF
    // Get the human-readable panic reason
    char reason[128];
    err = esp_core_dump_get_panic_reason(reason, sizeof(reason));
    if (err == ESP_OK) {
        // STATUSTEXT text field is 50 chars; "Panic: " = 7 chars
        reason[43] = '\0';
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Panic: %s", reason);
    }

    // Get structured summary with task name, PC, and backtrace
    esp_core_dump_summary_t *summary =
        (esp_core_dump_summary_t *)malloc(sizeof(esp_core_dump_summary_t));
    if (summary == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "Coredump: no RAM for summary");
        return;
    }

    err = esp_core_dump_get_summary(summary);
    if (err == ESP_OK) {
        // Report crashing task and PC
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                      "Crash task: %s  PC: 0x%08lX",
                      summary->exc_task,
                      (unsigned long)summary->exc_pc);

        // Report exception cause and faulting address (Xtensa-specific)
        const char *cause_str = exc_cause_name(summary->ex_info.exc_cause);
        if (cause_str) {
            // "Exc: LoadProhibit(28) @0x12345678" = ~35 chars
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                          "Exc: %s(%lu) @0x%08lX",
                          cause_str,
                          (unsigned long)summary->ex_info.exc_cause,
                          (unsigned long)summary->ex_info.exc_vaddr);
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                          "Exc: cause=%lu @0x%08lX",
                          (unsigned long)summary->ex_info.exc_cause,
                          (unsigned long)summary->ex_info.exc_vaddr);
        }

        // Report backtrace (up to 8 frames to keep message count reasonable)
        uint32_t bt_depth = summary->exc_bt_info.depth;
        if (bt_depth > 8) {
            bt_depth = 8;
        }

        if (bt_depth > 0 && !summary->exc_bt_info.corrupted) {
            // Pack 3 addresses per message to fit STATUSTEXT 50-char limit
            // "BT[0]: 0x12345678 0x12345678 0x12345678" = 42 chars max
            for (uint32_t i = 0; i < bt_depth; i += 3) {
                char bt_msg[52];
                int len = snprintf(bt_msg, sizeof(bt_msg), "BT[%lu]:", (unsigned long)i);
                for (uint32_t j = i; j < i + 3 && j < bt_depth; j++) {
                    len += snprintf(bt_msg + len, sizeof(bt_msg) - len,
                                    " 0x%08lX",
                                    (unsigned long)summary->exc_bt_info.bt[j]);
                }
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "%s", bt_msg);
            }
        } else if (summary->exc_bt_info.corrupted) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "Coredump: backtrace corrupted");
        }

        // Report ELF SHA so user can match dump to binary
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "ELF SHA: %.16s",
                      (const char *)summary->app_elf_sha256);

        // Log full details to ESP_LOG for console/serial capture
        ESP_LOGE(TAG, "Core dump summary:");
        ESP_LOGE(TAG, "  Task: %s  PC: 0x%08lX  TCB: 0x%08lX",
                 summary->exc_task,
                 (unsigned long)summary->exc_pc,
                 (unsigned long)summary->exc_tcb);
        ESP_LOGE(TAG, "  ExcCause: %lu  ExcVAddr: 0x%08lX",
                 (unsigned long)summary->ex_info.exc_cause,
                 (unsigned long)summary->ex_info.exc_vaddr);
        for (uint32_t i = 0; i < summary->exc_bt_info.depth && i < 16; i++) {
            ESP_LOGE(TAG, "  BT[%lu]: 0x%08lX",
                     (unsigned long)i,
                     (unsigned long)summary->exc_bt_info.bt[i]);
        }
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "Coredump: summary read failed (0x%lX)",
                      (unsigned long)err);
    }

    free(summary);
#endif  // CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF

    // Report core dump size for reference
    size_t addr = 0, size = 0;
    if (esp_core_dump_image_get(&addr, &size) == ESP_OK) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "Coredump: %lu bytes at flash 0x%lX",
                      (unsigned long)size, (unsigned long)addr);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "Retrieve: idf.py coredump-info -p PORT");
    }

    // Erase the dump after reporting so we don't report it every boot
    esp_core_dump_image_erase();
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Coredump: erased after report");

#else  // !CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    // Core dump not enabled in this build
    ESP_LOGW(TAG, "Core dump to flash not enabled in sdkconfig");
#endif
}

// Delayed crash report sender (called from timer after MAVLink is ready)
static bool crash_report_sent = false;
void esp32_send_crash_report_delayed(void)
{
    if (crash_report_sent) {
        return;
    }

    static uint32_t first_call_ms = 0;
    if (first_call_ms == 0) {
        first_call_ms = AP_HAL::millis();
        return;
    }

    // Wait 10 seconds after boot for MAVLink link to stabilise
    if (AP_HAL::millis() - first_call_ms < 10000) {
        return;
    }

    crash_report_sent = true;

    // Send RTC breadcrumbs from previous session
    rtc_breadcrumb_dump_to_mavlink();

    // Check and report any stored core dump
    esp32_check_and_report_coredump();
}

// Stub implementations for breadcrumb integration examples
void esp32_can_init_with_breadcrumbs(void)
{
    RTC_BREADCRUMB_INFO(RTC_BC_CAN_INIT, "Starting CAN init");
}

bool esp32_read_sensor_with_breadcrumbs(float *value)
{
    if (value == nullptr) {
        RTC_BREADCRUMB_ERROR(RTC_BC_SENSOR_READ, "Sensor null ptr");
        return false;
    }
    return true;
}

void *esp32_malloc_with_breadcrumbs(size_t size)
{
    void *ptr = malloc(size);
    if (!ptr) {
        char msg[28];
        snprintf(msg, sizeof(msg), "malloc fail %zu", size);
        RTC_BREADCRUMB_ERROR(RTC_BC_HEAP_ALLOC, msg);
    }
    return ptr;
}

void esp32_wifi_init_with_breadcrumbs(void)
{
    RTC_BREADCRUMB_INFO(RTC_BC_WIFI_INIT, "WiFi starting");
}

void esp32_param_load_with_breadcrumbs(void)
{
    RTC_BREADCRUMB_INFO(RTC_BC_PARAM_LOAD, "Loading params");
}

void esp32_enter_critical_section(const char *section_name)
{
    char msg[28];
    snprintf(msg, sizeof(msg), "ENTER %s", section_name);
    RTC_BREADCRUMB_WARN(RTC_BC_CRITICAL_SECTION, msg);
}

void esp32_exit_critical_section(const char *section_name)
{
    char msg[28];
    snprintf(msg, sizeof(msg), "EXIT %s", section_name);
    RTC_BREADCRUMB_TRACE(RTC_BC_CRITICAL_SECTION, msg);
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_ESP32
