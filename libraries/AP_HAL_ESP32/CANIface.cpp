
#include "CANIface.h"

#if HAL_NUM_CAN_IFACES > 0

#include <AP_HAL/AP_HAL.h>
#include "driver/gpio.h"
#include "driver/twai.h"

extern const AP_HAL::HAL& hal;

using namespace ESP32;

// Use consistent ESP32 debug system controlled by ESP32_DEBUG_LEVEL parameter
#include "ESP32_Debug.h"

// CAN logging macros using ESP32 debug system
#define CAN_DEBUG_ERROR(fmt, ...)   ESP_LOGE("CAN", fmt, ##__VA_ARGS__)
#define CAN_DEBUG_WARN(fmt, ...)    ESP_LOGW("CAN", fmt, ##__VA_ARGS__)  
#define CAN_DEBUG_INFO(fmt, ...)    ESP_LOGI("CAN", fmt, ##__VA_ARGS__)
#define CAN_DEBUG_VERBOSE(fmt, ...) ESP_LOGD("CAN", fmt, ##__VA_ARGS__)
#define CAN_DEBUG_DEBUG(fmt, ...)   ESP_LOGV("CAN", fmt, ##__VA_ARGS__)

// Legacy compatibility
#define CAN_DEBUG_PRINTF(fmt, ...) CAN_DEBUG_INFO(fmt, ##__VA_ARGS__)

CANIface::CANIface(uint8_t instance) :
    ESP32_CANBase(instance),
    initialized(false),
    current_bitrate(500000)
{
}

bool CANIface::init(const uint32_t bitrate, const OperatingMode mode)
{
    CAN_DEBUG_INFO("CAN interface %d initialized - bitrate=%u", instance, (unsigned)bitrate);
    
    if (initialized) {
        CAN_DEBUG_INFO("CAN interface %d already initialized", instance);
        return true;
    }
    
    current_bitrate = bitrate;

    gpio_num_t tx_pin, rx_pin;

    if (instance == 0) {
#if HAL_NUM_CAN_IFACES > 0
        tx_pin = (gpio_num_t)HAL_CAN1_TX_PIN;
        rx_pin = (gpio_num_t)HAL_CAN1_RX_PIN;
        CAN_DEBUG_INFO("Initializing CAN interface %d with TX pin %d, RX pin %d, bitrate %u", 
               instance, (int)tx_pin, (int)rx_pin, (unsigned)bitrate);
#else
        CAN_DEBUG_ERROR("HAL_NUM_CAN_IFACES is 0, CAN disabled");
        return false;
#endif
    } else {
        // Only support CAN interface 0 with native TWAI
        // CAN interface 1+ should use MCP2515 or other external controllers
        CAN_DEBUG_ERROR("Instance %d not supported (only instance 0 supported)", instance);
        return false;
    }

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_pin, rx_pin, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config;
    switch (bitrate) {
    case 1000000:
        t_config = TWAI_TIMING_CONFIG_1MBITS();
        break;
    case 500000:
        t_config = TWAI_TIMING_CONFIG_500KBITS();
        break;
    case 250000:
        t_config = TWAI_TIMING_CONFIG_250KBITS();
        break;
    case 125000:
        t_config = TWAI_TIMING_CONFIG_125KBITS();
        break;
    default:
        return false;
    }

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    CAN_DEBUG_INFO("Installing TWAI driver...");
    esp_err_t install_result = twai_driver_install(&g_config, &t_config, &f_config);
    if (install_result != ESP_OK) {
        CAN_DEBUG_ERROR("TWAI driver install failed with error %d", install_result);
        return false;
    }
    CAN_DEBUG_INFO("TWAI driver installed successfully");

    CAN_DEBUG_INFO("Starting TWAI...");
    esp_err_t start_result = twai_start();
    if (start_result != ESP_OK) {
        CAN_DEBUG_ERROR("TWAI start failed with error %d", start_result);
        return false;
    }
    CAN_DEBUG_INFO("TWAI started successfully");

    CAN_DEBUG_INFO("Creating RX queue...");
    rx_queue = xQueueCreate(128, sizeof(CanRxItem));
    if (rx_queue == NULL) {
        CAN_DEBUG_ERROR("Failed to create RX queue");
        return false;
    }

    CAN_DEBUG_INFO("Creating TX queue...");
    tx_queue = xQueueCreate(128, sizeof(CanTxItem));
    if (tx_queue == NULL) {
        CAN_DEBUG_ERROR("Failed to create TX queue");
        return false;
    }

    CAN_DEBUG_INFO("Creating RX and TX tasks...");
    xTaskCreate(rx_task, "can_rx", 4096, this, 5, NULL);
    xTaskCreate(tx_task, "can_tx", 4096, this, 5, NULL);

    initialized = true;
    CAN_DEBUG_INFO("Interface %d initialization complete!", instance);

    return true;
}

int16_t CANIface::send(const AP_HAL::CANFrame &frame, uint64_t tx_deadline, AP_HAL::CANIface::CanIOFlags flags)
{
    if (!initialized) {
        return -1;
    }

    CanTxItem tx_item;
    tx_item.frame = frame;
    tx_item.deadline_us = tx_deadline;
    tx_item.flags = flags;

    if (xQueueSendToBack(tx_queue, &tx_item, 0) != pdPASS) {
        return 0;
    }

    return 1;
}

int16_t CANIface::receive(AP_HAL::CANFrame &frame, uint64_t &timestamp_us, AP_HAL::CANIface::CanIOFlags &flags)
{
    if (!initialized) {
        return -1;
    }

    CanRxItem rx_item;
    if (xQueueReceive(rx_queue, &rx_item, 0) != pdPASS) {
        return 0;
    }

    frame = rx_item.frame;
    timestamp_us = rx_item.timestamp_us;
    flags = rx_item.flags;

    return 1;
}

bool CANIface::add_to_rx_queue(const CanRxItem &rx_item)
{
    return xQueueSendToBack(rx_queue, &rx_item, 0) == pdPASS;
}

void CANIface::rx_task(void *arg)
{
    CANIface *iface = (CANIface *)arg;

    while (true) {
        twai_message_t message;
        esp_err_t rx_result = twai_receive(&message, portMAX_DELAY);
        
        if (rx_result != ESP_OK) {
            // Handle RX errors with enhanced detection
            if (rx_result == ESP_ERR_TIMEOUT) {
                // Normal timeout - no action needed
                continue;
            } else if (rx_result == ESP_FAIL) {
                // RX failure - check bus state
                twai_status_info_t twai_status;
                if (twai_get_status_info(&twai_status) == ESP_OK) {
                    if (twai_status.state == TWAI_STATE_BUS_OFF) {
                        CAN_DEBUG_ERROR("Bus-off detected during receive");
                        iface->update_error_stats(0x01); // Bus-off error
                    } else if (twai_status.state == TWAI_STATE_STOPPED) {
                        CAN_DEBUG_ERROR("TWAI stopped unexpectedly");
                        iface->update_error_stats(0x02); // Stopped error
                    }
                }
            } else {
                // Other RX errors
                iface->update_error_stats(rx_result);
            }
            continue;
        }

#if CAN_LOGLEVEL >= 4
        // Verbose frame-by-frame logging - use ESP-IDF VERBOSE level
        static uint32_t rx_count = 0;
        rx_count++;
        
        char data_str[32] = {0};
        for (int i = 0; i < message.data_length_code && i < 8; i++) {
            snprintf(data_str + (i * 3), sizeof(data_str) - (i * 3), "%02X ", message.data[i]);
        }
        
        CAN_DEBUG_VERBOSE("RX #%lu: ID=0x%08X DLC=%d DATA=[%s]", 
                         (unsigned long)rx_count, (unsigned)message.identifier, 
                         message.data_length_code, data_str);
#elif CAN_LOGLEVEL >= 3
        // INFO level - summary every 100 frames
        static uint32_t rx_count_summary = 0;
        rx_count_summary++;
        if ((rx_count_summary % 100) == 0) {
            CAN_DEBUG_INFO("Received %lu frames, latest ID=0x%08X", 
                           (unsigned long)rx_count_summary, (unsigned)message.identifier);
        }
#elif CAN_LOGLEVEL >= 2
        // WARN level - summary every 1000 frames (less verbose)
        static uint32_t rx_count_warn = 0;
        rx_count_warn++;
        if ((rx_count_warn % 1000) == 0) {
            CAN_DEBUG_WARN("CAN traffic detected: %lu frames received", (unsigned long)rx_count_warn);
        }
#endif

        CanRxItem rx_item;
        rx_item.timestamp_us = AP_HAL::micros64();
        rx_item.frame.id = message.identifier;
        rx_item.frame.dlc = message.data_length_code;
        memcpy(rx_item.frame.data, message.data, message.data_length_code);

        // Update statistics
        iface->update_rx_stats();

        iface->add_to_rx_queue(rx_item);
    }
}

void CANIface::tx_task(void *arg)
{
    CANIface *iface = (CANIface *)arg;
    CanTxItem tx_item;

    while (true) {
        // Continuously pull frames from queue until we get a non-expired one
        uint32_t expired_count = 0;
        do {
            if (xQueueReceive(iface->tx_queue, &tx_item, portMAX_DELAY) != pdPASS) {
                continue;
            }

            uint64_t now_us = AP_HAL::micros64();
            
            // Check if this frame has expired
            if (tx_item.deadline_us != 0 && now_us > tx_item.deadline_us) {
                expired_count++;
                // Only log first few expired frames to avoid spam
                if (expired_count <= 5 || expired_count % 10 == 0) {
                    CAN_DEBUG_VERBOSE("Dropped expired frame ID=0x%08X (deadline=%llu, now=%llu) [count=%lu]", 
                           (unsigned)tx_item.frame.id, tx_item.deadline_us, now_us, (unsigned long)expired_count);
                }
                continue; // Get next frame
            }
            
            // Frame is not expired, break out of loop to transmit it
            break;
            
        } while (true);

        // Verbose frame-by-frame TX logging
        char data_str[32] = {0};
        for (int i = 0; i < tx_item.frame.dlc && i < 8; i++) {
            snprintf(data_str + (i * 3), sizeof(data_str) - (i * 3), "%02X ", tx_item.frame.data[i]);
        }
        CAN_DEBUG_VERBOSE("TX: ID=0x%08X DLC=%d DATA=[%s]", (unsigned)tx_item.frame.id, tx_item.frame.dlc, data_str);

        twai_message_t message;
        message.identifier = tx_item.frame.id;
        message.data_length_code = tx_item.frame.dlc;
        message.extd = 1;  // Enable 29-bit extended CAN frames (required for DroneCAN)
        message.rtr = 0;   // Data frame, not remote transmission request
        memcpy(message.data, tx_item.frame.data, tx_item.frame.dlc);

        // Single transmission attempt for DroneCAN compliance
        // DroneCAN handles reliability at protocol layer, HAL retries violate timing assumptions
        const TickType_t timeout_ms = 1; // Minimal timeout for real-time performance
        
        esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(timeout_ms));
        
        // Update statistics and handle errors
        iface->update_tx_stats(result == ESP_OK);
        
        if (result != ESP_OK) {
            // Handle specific TWAI error conditions
            if (result == ESP_ERR_TIMEOUT) {
                // TX timeout - bus may be congested or disconnected
                CAN_DEBUG_WARN("TX timeout on message ID=0x%08X", (unsigned)tx_item.frame.id);
            } else if (result == ESP_FAIL) {
                // TX failed - check bus state
                twai_status_info_t twai_status;
                if (twai_get_status_info(&twai_status) == ESP_OK) {
                    if (twai_status.state == TWAI_STATE_BUS_OFF) {
                        // Bus-off condition detected - attempt recovery
                        CAN_DEBUG_ERROR("Bus-off detected, attempting recovery");
                        iface->update_error_stats(0x01); // Bus-off error code
                        
                        // Attempt bus recovery
                        twai_initiate_recovery();
                        // Note: Recovery is automatic in TWAI driver
                    }
                }
                CAN_DEBUG_ERROR("Failed to transmit message ID=0x%08X, error=%d", (unsigned)tx_item.frame.id, result);
            } else {
                // Other errors
                iface->update_error_stats(result);
                CAN_DEBUG_ERROR("Error %d on message ID=0x%08X", result, (unsigned)tx_item.frame.id);
            }
        }
    }
}

// ESP32_CANBase pure virtual implementations
bool CANIface::configure_hw_filters(const CanFilterConfig* filter_configs, uint16_t num_configs)
{
    if (!initialized) {
        // Store filters for later application during init
        return true;
    }
    
    // ESP32 TWAI has single filter with acceptance code and mask
    // We need to calculate least-common-denominator filter for all requested filters
    uint32_t combined_acceptance = 0x00000000;
    uint32_t combined_mask = 0x00000000;  // Start permissive, will restrict based on filters
    
    if (num_configs > 0) {
        // Find the common mask bits across all filters
        uint32_t common_mask_bits = 0x1FFFFFFF;  // Start with all bits
        
        for (uint16_t i = 0; i < num_configs; i++) {
            const auto& filter = filter_configs[i];
            // Only keep mask bits that are common across ALL filters
            common_mask_bits &= filter.mask;
        }
        
        // Use the first filter's ID with the common mask
        if (num_configs == 1) {
            // Single filter - use exact match
            combined_acceptance = filter_configs[0].id & 0x1FFFFFFF;
            combined_mask = (~filter_configs[0].mask) & 0x1FFFFFFF;  // Invert for TWAI
        } else {
            // Multiple filters - use most permissive approach
            // Accept any message that matches at least one filter's pattern
            combined_acceptance = 0x00000000;  // Don't care about specific ID
            combined_mask = (~common_mask_bits) & 0x1FFFFFFF;  // Only filter on common bits
        }
        
        hal.console->printf("CAN%d: filters=%d, accept=0x%08lx, mask=0x%08lx\n",
                           instance, num_configs, 
                           (unsigned long)combined_acceptance, 
                           (unsigned long)combined_mask);
    } else {
        // No specific filters - accept all
        combined_acceptance = 0x00000000;
        combined_mask = 0x00000000;
        hal.console->printf("CAN%d: no filters - accepting all messages\n", instance);
    }
    
    // Apply the new filter configuration
    // Note: TWAI driver must be stopped before reconfiguring filters
    if (twai_stop() != ESP_OK) {
        CAN_DEBUG_ERROR("Failed to stop TWAI for filter reconfiguration");
        return false;
    }
    
    twai_filter_config_t new_filter;
    new_filter.acceptance_code = combined_acceptance;
    new_filter.acceptance_mask = combined_mask;
    new_filter.single_filter = true;  // Use single filter mode
    
    // Reconfigure the driver with new filter
    if (twai_driver_uninstall() != ESP_OK) {
        CAN_DEBUG_ERROR("Failed to uninstall TWAI driver for filter reconfiguration");
        return false;
    }
    
    // Reinstall with new filter
    gpio_num_t tx_pin = (gpio_num_t)HAL_CAN1_TX_PIN;
    gpio_num_t rx_pin = (gpio_num_t)HAL_CAN1_RX_PIN;
    
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_pin, rx_pin, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config;
    
    // Use the current bitrate for timing configuration
    switch (current_bitrate) {
    case 1000000:
        t_config = TWAI_TIMING_CONFIG_1MBITS();
        break;
    case 500000:
        t_config = TWAI_TIMING_CONFIG_500KBITS();
        break;
    case 250000:
        t_config = TWAI_TIMING_CONFIG_250KBITS();
        break;
    case 125000:
        t_config = TWAI_TIMING_CONFIG_125KBITS();
        break;
    default:
        t_config = TWAI_TIMING_CONFIG_500KBITS();
        break;
    }
    
    if (twai_driver_install(&g_config, &t_config, &new_filter) != ESP_OK) {
        CAN_DEBUG_ERROR("Failed to reinstall TWAI driver with new filters");
        // Fall back to accept-all configuration
        twai_filter_config_t fallback = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        twai_driver_install(&g_config, &t_config, &fallback);
    }
    
    if (twai_start() != ESP_OK) {
        CAN_DEBUG_ERROR("Failed to restart TWAI after filter reconfiguration");
        return false;
    }
    
    CAN_DEBUG_INFO("Hardware filters updated - acceptance=0x%08X mask=0x%08X", 
                       (unsigned)combined_acceptance, (unsigned)combined_mask);
    
    return true;
}

void CANIface::collect_hw_stats()
{
    if (!initialized) {
        stats.current_health = 2; // ERROR - not initialized
        return;
    }
    
    static uint32_t last_stats_print_ms = 0;
    uint32_t now_ms = AP_HAL::millis();
    
    // Print periodic stats every 10 seconds if there's any activity
    if (now_ms - last_stats_print_ms > 10000) {
        if (stats.tx_success > 0 || stats.rx_received > 0) {
            CAN_DEBUG_INFO("CAN%d Stats: TX_OK=%u TX_FAIL=%u RX=%u Errors=%u Health=%d", 
                          instance, (unsigned)stats.tx_success, (unsigned)stats.tx_failed,
                          (unsigned)stats.rx_received, (unsigned)stats.bus_errors, stats.current_health);
        }
        last_stats_print_ms = now_ms;
    }
    
    // Collect ESP-IDF TWAI hardware statistics
    twai_status_info_t twai_status;
    if (twai_get_status_info(&twai_status) == ESP_OK) {
        // Update bus error statistics from TWAI hardware
        stats.bus_errors = twai_status.bus_error_count;
        
        // Track bus-off events
        if (twai_status.state == TWAI_STATE_BUS_OFF) {
            stats.bus_off_count++;
            stats.current_health = 2; // ERROR
            stats.last_error_code = 0x01; // Bus-off error code
            stats.last_error_time_us = AP_HAL::micros64();
        } else if (twai_status.state == TWAI_STATE_STOPPED) {
            stats.current_health = 1; // WARNING
        } else if (twai_status.state == TWAI_STATE_RUNNING) {
            // Calculate health based on recent error activity
            stats.current_health = calculate_bus_health();
        } else {
            stats.current_health = 1; // WARNING for other states
        }
        
        // Update filter rejection count if available
        // Note: TWAI hardware doesn't directly report filter rejections,
        // but we can infer from receive buffer status
        if (twai_status.msgs_to_rx == 0) {
            // No messages pending - could indicate filtering or no traffic
        }
        
    } else {
        // Failed to get TWAI status - mark as error
        stats.current_health = 2;
        stats.bus_errors++;
        stats.last_error_code = 0xFF; // Status read error
        stats.last_error_time_us = AP_HAL::micros64();
    }
}

void CANIface::update_status()
{
    if (!initialized) {
        return;
    }
    
    // Perform enhanced health check
    check_bus_health();
    
    // Send periodic MAVLink status
    send_uavcan_node_status();
}

void CANIface::check_bus_health()
{
    if (!initialized) {
        return;
    }
    
    twai_status_info_t twai_status;
    if (twai_get_status_info(&twai_status) != ESP_OK) {
        CAN_DEBUG_ERROR("Failed to get TWAI status");
        update_error_stats(0xFF); // Status read error
        return;
    }
    
    // Check for bus-off condition and attempt recovery
    static uint32_t last_recovery_attempt_ms = 0;
    static uint32_t consecutive_bus_off_count = 0;
    static uint32_t last_full_restart_ms = 0;
    uint32_t now_ms = AP_HAL::millis();
    
    if (twai_status.state == TWAI_STATE_BUS_OFF) {
        // Attempt basic recovery every 5 seconds
        if (now_ms - last_recovery_attempt_ms > 5000) {
            consecutive_bus_off_count++;
            CAN_DEBUG_ERROR("Bus-off detected (attempt #%u), attempting recovery", 
                           (unsigned)consecutive_bus_off_count);
            update_error_stats(0x01); // Bus-off error
            
            // If we've had persistent bus-off conditions, try full driver restart
            if (consecutive_bus_off_count > 5 && (now_ms - last_full_restart_ms > 30000)) {
                CAN_DEBUG_ERROR("Persistent bus-off condition, attempting full TWAI restart");
                
                // Store current configuration for restart
                uint32_t saved_bitrate = current_bitrate;
                
                // Perform full TWAI restart
                twai_stop();
                twai_driver_uninstall();
                
                // Brief delay to allow hardware to settle
                hal.scheduler->delay_microseconds(1000);
                
                // Reinitialize with same configuration (use NormalMode as default)
                if (init(saved_bitrate, NormalMode)) {
                    CAN_DEBUG_INFO("Full TWAI restart successful");
                    consecutive_bus_off_count = 0; // Reset counter on successful restart
                    last_full_restart_ms = now_ms;
                } else {
                    CAN_DEBUG_ERROR("Full TWAI restart failed");
                    // Continue with basic recovery attempts
                }
            } else {
                // Standard recovery attempt
                twai_initiate_recovery();
            }
            
            last_recovery_attempt_ms = now_ms;
        }
    } else {
        // Bus is healthy, reset consecutive bus-off counter
        consecutive_bus_off_count = 0;
    }
    
    // Check for excessive TX queue backlog
    if (tx_queue && uxQueueMessagesWaiting(tx_queue) > 100) {
        CAN_DEBUG_WARN("TX queue backlog detected (%u messages)", 
                           (unsigned)uxQueueMessagesWaiting(tx_queue));
        update_error_stats(0x10); // Queue backlog error
    }
    
    // Check for RX overruns
    if (twai_status.rx_missed_count > 0) {
        static uint32_t last_rx_missed = 0;
        if (twai_status.rx_missed_count > last_rx_missed) {
            CAN_DEBUG_WARN("RX overrun detected (missed %u messages)", 
                               (unsigned)(twai_status.rx_missed_count - last_rx_missed));
            update_error_stats(0x20); // RX overrun error
        }
        last_rx_missed = twai_status.rx_missed_count;
    }
}

#endif // HAL_NUM_CAN_IFACES > 0
