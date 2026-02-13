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

#pragma once

#include "AP_HAL_ESP32.h"

#if HAL_NUM_CAN_IFACES > 0

#include <AP_HAL/CANIface.h>
#include <AP_Common/ExpandingString.h>
#include <GCS_MAVLink/GCS.h>

namespace ESP32 {

/**
 * CAN filter configuration for ESP32 CAN implementations.
 * This was removed from AP_HAL::CANIface upstream but is still
 * needed for ESP32 hardware filter support.
 */
struct CanFilterConfig {
    uint32_t id = 0;
    uint32_t mask = 0;

    bool operator==(const CanFilterConfig& rhs) const
    {
        return rhs.id == id && rhs.mask == mask;
    }
};

/**
 * Common base class for ESP32 CAN implementations
 * Provides unified statistics, filtering, and MAVLink reporting
 * for both TWAI (native) and MCP2515 (external) controllers
 */
class ESP32_CANBase : public AP_HAL::CANIface {
public:
    ESP32_CANBase(uint8_t instance);

    // Common API implementations that both TWAI and MCP2515 will use
    bool configureFilters(const CanFilterConfig* filter_configs, uint16_t num_configs);
    uint32_t getErrorCount() const override;
    
    // Override base class get_stats method
    void get_stats(ExpandingString &str) override;
    
    // Custom stats method for direct usage
    void get_stats(char* str, size_t len);

    // MAVLink status reporting
    void send_uavcan_node_status();

protected:
    // Statistics structure shared by both implementations
    struct can_stats_t {
        uint32_t tx_success;
        uint32_t tx_failed; 
        uint32_t rx_received;
        uint32_t bus_errors;
        uint32_t last_error_code;
        uint32_t filter_rejects;  // Hardware filter rejections
        uint8_t current_health;   // 0=OK, 1=WARNING, 2=ERROR
        uint64_t last_error_time_us;
        uint32_t bus_off_count;
    } stats;

    uint8_t instance;
    uint32_t last_status_send_ms;
    
    // Send interval for MAVLink status updates (1Hz)
    static constexpr uint32_t STATUS_SEND_INTERVAL_MS = 1000;

    // Pure virtual methods that each implementation must provide
    virtual bool configure_hw_filters(const CanFilterConfig* filter_configs, uint16_t num_configs) = 0;
    virtual void collect_hw_stats() = 0;
    virtual const char* get_controller_name() const = 0;

    // Helper methods for common functionality
    void update_tx_stats(bool success);
    void update_rx_stats();
    void update_error_stats(uint32_t error_code);
    uint8_t calculate_bus_health() const;

private:
    // Internal filter management
    static constexpr uint16_t MAX_FILTERS = 16;
    CanFilterConfig active_filters[MAX_FILTERS];
    uint16_t num_active_filters;
    
    // Health assessment thresholds
    static constexpr uint32_t ERROR_WARNING_THRESHOLD = 10;
    static constexpr uint32_t ERROR_CRITICAL_THRESHOLD = 50;
};

} // namespace ESP32

#endif // HAL_NUM_CAN_IFACES > 0