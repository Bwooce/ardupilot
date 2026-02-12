/*
 * ESP32 Startup Time Metrics
 */
#pragma once

#include <stdint.h>
#include <AP_HAL/AP_HAL.h>

namespace ESP32 {

class StartupMetrics {
public:
    static StartupMetrics& instance() {
        static StartupMetrics _instance;
        return _instance;
    }

    // Record various startup milestones
    void record_boot_start();
    void record_storage_init_start();
    void record_storage_erase_start(uint8_t sector);
    void record_storage_erase_end(uint8_t sector);
    void record_storage_init_end();
    void record_system_ready();
    
    // Log the metrics (called once when system is ready)
    void log_metrics_once();

private:
    StartupMetrics() = default;
    
    struct {
        uint32_t boot_start_ms = 0;
        uint32_t storage_init_start_ms = 0;
        uint32_t storage_init_end_ms = 0;
        uint32_t system_ready_ms = 0;
        
        // Track erase times for each sector
        struct {
            uint32_t start_ms = 0;
            uint32_t duration_ms = 0;
            bool erased = false;
        } sector_erase[4];  // Support up to 4 sectors
        
        uint32_t total_erase_time_ms = 0;
        uint8_t sectors_erased = 0;
        
        bool logged = false;  // Ensure we only log once
    } metrics;
};

} // namespace ESP32