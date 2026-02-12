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

#include "ESP32_CANBase.h"

#if HAL_NUM_CAN_IFACES > 0

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/ExpandingString.h>
#include <cstdio>
#include "ESP32_Debug.h"

extern const AP_HAL::HAL& hal;

using namespace ESP32;

ESP32_CANBase::ESP32_CANBase(uint8_t instance) :
    instance(instance),
    last_status_send_ms(0),
    num_active_filters(0)
{
    // Initialize statistics
    memset(&stats, 0, sizeof(stats));
}

bool ESP32_CANBase::configureFilters(const CanFilterConfig* filter_configs, uint16_t num_configs) 
{
    if (num_configs > MAX_FILTERS) {
        return false;
    }

    // Store filter configuration for reference
    memcpy(active_filters, filter_configs, num_configs * sizeof(CanFilterConfig));
    num_active_filters = num_configs;

    // Delegate to hardware-specific implementation
    return configure_hw_filters(filter_configs, num_configs);
}

void ESP32_CANBase::get_stats(ExpandingString &str)
{
    // Update hardware-specific stats first
    collect_hw_stats();

    str.printf("CAN%u (%s): TX=%lu/%lu RX=%lu ERR=%lu H=%u F=%u",
        instance, get_controller_name(),
        (unsigned long)stats.tx_success, (unsigned long)stats.tx_failed,
        (unsigned long)stats.rx_received,
        (unsigned long)stats.bus_errors,
        stats.current_health,
        num_active_filters);
}

void ESP32_CANBase::get_stats(char* str, size_t len)
{
    // Update hardware-specific stats first
    collect_hw_stats();

    snprintf(str, len, 
        "CAN%u (%s): TX=%lu/%lu RX=%lu ERR=%lu H=%u F=%u",
        instance, get_controller_name(),
        (unsigned long)stats.tx_success, (unsigned long)stats.tx_failed,
        (unsigned long)stats.rx_received,
        (unsigned long)stats.bus_errors,
        stats.current_health,
        num_active_filters);
}

uint32_t ESP32_CANBase::getErrorCount() const
{
    return stats.bus_errors;
}

void ESP32_CANBase::send_uavcan_node_status()
{
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_status_send_ms < STATUS_SEND_INTERVAL_MS) {
        return; // Don't spam
    }
    last_status_send_ms = now_ms;

    // Update hardware stats before reporting
    const_cast<ESP32_CANBase*>(this)->collect_hw_stats();

    // Report DroneCAN bus status now that recursion issue is fixed
    if (stats.bus_errors > 0 || stats.tx_failed > 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, 
                      "CAN%d: bus_err=%lu tx_fail=%lu tx_ok=%lu rx=%lu", 
                      instance,
                      (unsigned long)stats.bus_errors,
                      (unsigned long)stats.tx_failed, 
                      (unsigned long)stats.tx_success,
                      (unsigned long)stats.rx_received);
    }
    
    // Periodic status for active buses
    static uint32_t last_periodic_ms = 0;
    if (stats.rx_received > 0 && now_ms - last_periodic_ms > 30000) {
        last_periodic_ms = now_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "CAN%d: tx=%lu rx=%lu errors=%lu", 
                      instance,
                      (unsigned long)stats.tx_success,
                      (unsigned long)stats.rx_received, 
                      (unsigned long)stats.bus_errors);
    }
}

void ESP32_CANBase::update_tx_stats(bool success)
{
    if (success) {
        stats.tx_success++;
    } else {
        stats.tx_failed++;
    }
}

void ESP32_CANBase::update_rx_stats()
{
    stats.rx_received++;
    
    // Log first few RX messages to confirm CAN traffic
    static uint32_t rx_log_count = 0;
    if (rx_log_count < 5) {
        ESP32_DEBUG_INFO("CAN%d RX message #%u received", instance, (unsigned)(++rx_log_count));
    }
}

void ESP32_CANBase::update_error_stats(uint32_t error_code)
{
    stats.bus_errors++;
    stats.last_error_code = error_code;
    stats.last_error_time_us = AP_HAL::micros64();
    
    // Update health based on error patterns
    stats.current_health = calculate_bus_health();
}

uint8_t ESP32_CANBase::calculate_bus_health() const
{
    // Simple health assessment based on recent error rate
    uint32_t recent_errors = stats.bus_errors;
    
    if (recent_errors > ERROR_CRITICAL_THRESHOLD) {
        return 2; // ERROR
    } else if (recent_errors > ERROR_WARNING_THRESHOLD) {
        return 1; // WARNING
    }
    
    return 0; // OK
}

#endif // HAL_NUM_CAN_IFACES > 0