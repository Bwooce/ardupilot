/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32 && defined(HAL_ESP32_USE_PSRAM_LOGGING)

#include "AP_Logger_Backend.h"
#include <AP_HAL_ESP32/ESP32_PSRAM_Logger.h>
#include <AP_HAL_ESP32/ESP32_PSRAM_Logger.h>

class AP_Logger_ESP32_PSRAM : public AP_Logger_Backend
{
public:
    // constructor
    AP_Logger_ESP32_PSRAM(AP_Logger &front, LoggerMessageWriter_DFLogStart *);
    
    // probe to create backend
    static AP_Logger_Backend *probe(AP_Logger &front, LoggerMessageWriter_DFLogStart *writer) {
        // Check if PSRAM is actually available before creating backend
        if (!ESP32_PSRAM_Logger::psram_available()) {
            return nullptr;  // Let system fall back to other backends
        }
        return NEW_NOTHROW AP_Logger_ESP32_PSRAM(front, writer);
    }

    // initialisation
    void Init() override;
    bool CardInserted(void) const override { return true; }  // PSRAM always "inserted"

    // erase handling (not needed for PSRAM - it's a circular buffer)
    void EraseAll() override;

    // high level interface
    uint16_t find_last_log() override;
    void get_log_boundaries(uint16_t log_num, uint32_t &start_page, uint32_t &end_page) override;
    void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc) override;
    int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) override;
    uint16_t get_num_logs() override;

    void start_new_log(void) override;
    uint32_t bufferspace_available() override;
    
    // Additional required overrides
    void end_log_transfer() override {}  // No special handling needed
    bool logging_started(void) const override { return _initialised && _num_logs > 0; }
    void stop_logging(void) override { /* PSRAM logger doesn't need explicit stop */ }
    bool logging_failed() const override { return !_initialised; }
    bool WritesOK() const override { return _initialised && ESP32_PSRAM_Logger::healthy(); }

private:
    bool _initialised;
    uint16_t _num_logs;
    uint32_t _current_log_start;
    
    // Log index structure to track logs in PSRAM
    struct log_index_entry {
        uint32_t start_offset;
        uint32_t end_offset;
        uint32_t timestamp;
    };
    
    static const uint16_t MAX_LOG_FILES = 50;
    log_index_entry _log_index[MAX_LOG_FILES];
    
    // Override the pure virtual function from AP_Logger_Backend
    bool _WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical) override;
    bool io_thread_alive() const;
    uint32_t critical_message_reserved_space() const { return 0; }
    uint32_t non_messagewriter_message_reserved_space() const { return 0; }
};

#endif // CONFIG_HAL_BOARD == HAL_BOARD_ESP32 && HAL_ESP32_USE_PSRAM_LOGGING