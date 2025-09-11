/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32 && defined(HAL_ESP32_USE_PSRAM_LOGGING)

#include "AP_Logger_ESP32_PSRAM.h"
#include <AP_HAL_ESP32/ESP32_PSRAM_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/definitions.h>

extern const AP_HAL::HAL& hal;

// Constructor
AP_Logger_ESP32_PSRAM::AP_Logger_ESP32_PSRAM(AP_Logger &front, LoggerMessageWriter_DFLogStart *writer) :
    AP_Logger_Backend(front, writer),
    _initialised(false),
    _num_logs(0),
    _current_log_start(0)
{
    memset(_log_index, 0, sizeof(_log_index));
}

// Initialize the PSRAM logger backend
void AP_Logger_ESP32_PSRAM::Init()
{
    if (_initialised) {
        return;
    }

    // Initialize PSRAM logger with configured size (default 4MB)
    uint32_t log_size_mb = 4;
#ifdef HAL_ESP32_PSRAM_LOG_SIZE_MB
    log_size_mb = HAL_ESP32_PSRAM_LOG_SIZE_MB;
#endif

    if (!ESP32_PSRAM_Logger::init(log_size_mb)) {
        // PSRAM not available - try to fall back to SPIFFS
        gcs().send_text(MAV_SEVERITY_WARNING, "PSRAM Log: Not available, trying SPIFFS");
        // For now, mark as not initialized so file backend can be used
        _initialised = false;
        return;
    }

    _initialised = true;
    
    // Clear any existing logs on startup (optional - could scan for existing logs)
    ESP32_PSRAM_Logger::clear();
    _num_logs = 0;
    
    gcs().send_text(MAV_SEVERITY_INFO, "PSRAM Log: Ready (%lu MB)", 
                   (unsigned long)log_size_mb);
}

// Erase all logs
void AP_Logger_ESP32_PSRAM::EraseAll()
{
    if (!_initialised) {
        return;
    }
    
    ESP32_PSRAM_Logger::clear();
    _num_logs = 0;
    memset(_log_index, 0, sizeof(_log_index));
    
    gcs().send_text(MAV_SEVERITY_INFO, "PSRAM Log: Erased");
}

// Find the last log number
uint16_t AP_Logger_ESP32_PSRAM::find_last_log()
{
    return _num_logs;
}

// Get log boundaries (start and end pages/offsets)
void AP_Logger_ESP32_PSRAM::get_log_boundaries(uint16_t log_num, uint32_t &start_page, uint32_t &end_page)
{
    if (log_num == 0 || log_num > _num_logs) {
        start_page = 0;
        end_page = 0;
        return;
    }
    
    start_page = _log_index[log_num - 1].start_offset;
    end_page = _log_index[log_num - 1].end_offset;
}

// Get log info
void AP_Logger_ESP32_PSRAM::get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc)
{
    if (log_num == 0 || log_num > _num_logs) {
        size = 0;
        time_utc = 0;
        return;
    }
    
    size = _log_index[log_num - 1].end_offset - _log_index[log_num - 1].start_offset;
    time_utc = _log_index[log_num - 1].timestamp;
}

// Get log data
int16_t AP_Logger_ESP32_PSRAM::get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data)
{
    if (!_initialised || log_num == 0 || log_num > _num_logs) {
        return -1;
    }
    
    uint32_t log_start = _log_index[log_num - 1].start_offset;
    uint32_t log_end = _log_index[log_num - 1].end_offset;
    uint32_t read_offset = log_start + offset;
    
    if (read_offset >= log_end) {
        return -1;
    }
    
    uint16_t bytes_to_read = MIN(len, log_end - read_offset);
    
    if (ESP32_PSRAM_Logger::read(data, bytes_to_read, read_offset)) {
        return bytes_to_read;
    }
    
    return -1;
}

// Get number of logs
uint16_t AP_Logger_ESP32_PSRAM::get_num_logs()
{
    return _num_logs;
}

// Start a new log
void AP_Logger_ESP32_PSRAM::start_new_log(void)
{
    if (!_initialised) {
        return;
    }
    
    // Close current log if any
    if (_num_logs > 0 && _log_index[_num_logs - 1].end_offset == 0) {
        _log_index[_num_logs - 1].end_offset = ESP32_PSRAM_Logger::get_write_offset();
    }
    
    // Check if we've reached max logs
    if (_num_logs >= MAX_LOG_FILES) {
        // Shift logs down, removing oldest
        memmove(&_log_index[0], &_log_index[1], sizeof(log_index_entry) * (MAX_LOG_FILES - 1));
        _num_logs = MAX_LOG_FILES - 1;
    }
    
    // Start new log
    _current_log_start = ESP32_PSRAM_Logger::get_write_offset();
    _log_index[_num_logs].start_offset = _current_log_start;
    _log_index[_num_logs].end_offset = 0;  // Will be set when log closes
    _log_index[_num_logs].timestamp = AP_HAL::millis();
    _num_logs++;
    
    gcs().send_text(MAV_SEVERITY_INFO, "PSRAM Log: Started log %u at offset %lu",
                   _num_logs, (unsigned long)_current_log_start);
}

// Get available buffer space
uint32_t AP_Logger_ESP32_PSRAM::bufferspace_available()
{
    if (!_initialised) {
        return 0;
    }
    
    return ESP32_PSRAM_Logger::get_free_space();
}

// Override the pure virtual write function from AP_Logger_Backend
bool AP_Logger_ESP32_PSRAM::_WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical)
{
    // For PSRAM, we don't need to distinguish between critical and non-critical
    // since we have plenty of space and fast access
    
    bool result = ESP32_PSRAM_Logger::write((const uint8_t*)pBuffer, size);
    
    if (result) {
        // Update current log end offset
        if (_num_logs > 0) {
            _log_index[_num_logs - 1].end_offset = ESP32_PSRAM_Logger::get_write_offset();
        }
    }
    
    return result;
}

// Check if IO thread is alive (always true for PSRAM)
bool AP_Logger_ESP32_PSRAM::io_thread_alive() const
{
    // PSRAM doesn't need an IO thread - it's direct memory access
    return _initialised;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_ESP32 && HAL_ESP32_USE_PSRAM_LOGGING