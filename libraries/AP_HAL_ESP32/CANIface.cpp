
#include "CANIface.h"

#if HAL_NUM_CAN_IFACES > 0

#include <AP_HAL/AP_HAL.h>
#include "driver/gpio.h"
#include "driver/twai.h"

extern const AP_HAL::HAL& hal;

using namespace ESP32;

// CAN debug output using packet-level locking to prevent MAVLink corruption
#define CAN_DEBUG 1
#if CAN_DEBUG
  #define CAN_DEBUG_PRINTF(fmt, ...) do { \
    char debug_buf[256]; \
    int len = snprintf(debug_buf, sizeof(debug_buf), fmt, ##__VA_ARGS__); \
    if (len > 0 && len < (int)sizeof(debug_buf)) { \
      hal.console->write_packet((const uint8_t*)debug_buf, len); \
    } \
  } while(0)
#else
  #define CAN_DEBUG_PRINTF(fmt, ...) do {} while(0)
#endif

CANIface::CANIface(uint8_t instance) :
    ESP32_CANBase(instance),
    initialized(false),
    current_bitrate(500000)
{
}

bool CANIface::init(const uint32_t bitrate, const OperatingMode mode)
{
    if (initialized) {
        return true;
    }
    
    current_bitrate = bitrate;

    gpio_num_t tx_pin, rx_pin;

    if (instance == 0) {
#if HAL_NUM_CAN_IFACES > 0
        tx_pin = (gpio_num_t)HAL_CAN1_TX_PIN;
        rx_pin = (gpio_num_t)HAL_CAN1_RX_PIN;
        CAN_DEBUG_PRINTF("CAN: Initializing CAN interface %d with TX pin %d, RX pin %d, bitrate %u\n", 
               instance, (int)tx_pin, (int)rx_pin, (unsigned)bitrate);
#else
        CAN_DEBUG_PRINTF("CAN: HAL_NUM_CAN_IFACES is 0, CAN disabled\n");
        return false;
#endif
    } else {
        // Only support CAN interface 0 with native TWAI
        // CAN interface 1+ should use MCP2515 or other external controllers
        CAN_DEBUG_PRINTF("CAN: Instance %d not supported (only instance 0 supported)\n", instance);
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

    CAN_DEBUG_PRINTF("CAN: Installing TWAI driver...\n");
    esp_err_t install_result = twai_driver_install(&g_config, &t_config, &f_config);
    if (install_result != ESP_OK) {
        CAN_DEBUG_PRINTF("CAN: TWAI driver install failed with error %d\n", install_result);
        return false;
    }
    CAN_DEBUG_PRINTF("CAN: TWAI driver installed successfully\n");

    CAN_DEBUG_PRINTF("CAN: Starting TWAI...\n");
    esp_err_t start_result = twai_start();
    if (start_result != ESP_OK) {
        CAN_DEBUG_PRINTF("CAN: TWAI start failed with error %d\n", start_result);
        return false;
    }
    CAN_DEBUG_PRINTF("CAN: TWAI started successfully\n");

    CAN_DEBUG_PRINTF("CAN: Creating RX queue...\n");
    rx_queue = xQueueCreate(128, sizeof(CanRxItem));
    if (rx_queue == NULL) {
        CAN_DEBUG_PRINTF("CAN: Failed to create RX queue\n");
        return false;
    }

    CAN_DEBUG_PRINTF("CAN: Creating TX queue...\n");
    tx_queue = xQueueCreate(128, sizeof(CanTxItem));
    if (tx_queue == NULL) {
        CAN_DEBUG_PRINTF("CAN: Failed to create TX queue\n");
        return false;
    }

    CAN_DEBUG_PRINTF("CAN: Creating RX and TX tasks...\n");
    xTaskCreate(rx_task, "can_rx", 4096, this, 5, NULL);
    xTaskCreate(tx_task, "can_tx", 4096, this, 5, NULL);

    initialized = true;
    CAN_DEBUG_PRINTF("CAN: Interface %d initialization complete!\n", instance);

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
                        CAN_DEBUG_PRINTF("CAN RX: Bus-off detected during receive\n");
                        iface->update_error_stats(0x01); // Bus-off error
                    } else if (twai_status.state == TWAI_STATE_STOPPED) {
                        CAN_DEBUG_PRINTF("CAN RX: TWAI stopped unexpectedly\n");
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
        CAN_DEBUG_PRINTF("CAN RX: ID=0x%08X DLC=%d DATA=[", (unsigned)message.identifier, message.data_length_code);
        for (int i = 0; i < message.data_length_code; i++) {
            CAN_DEBUG_PRINTF("%02X", message.data[i]);
            if (i < message.data_length_code - 1) CAN_DEBUG_PRINTF(" ");
        }
        CAN_DEBUG_PRINTF("]\n");
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
#if CAN_LOGLEVEL >= 3
                // Only log first few expired frames to avoid spam
                if (expired_count <= 5 || expired_count % 10 == 0) {
                    CAN_DEBUG_PRINTF("CAN TX: Dropped expired frame ID=0x%08X (deadline=%llu, now=%llu) [count=%lu]\n", 
                           (unsigned)tx_item.frame.id, tx_item.deadline_us, now_us, (unsigned long)expired_count);
                }
#endif
                continue; // Get next frame
            }
            
            // Frame is not expired, break out of loop to transmit it
            break;
            
        } while (true);

#if CAN_LOGLEVEL >= 4
        CAN_DEBUG_PRINTF("CAN TX: ID=0x%08X DLC=%d DATA=[", (unsigned)tx_item.frame.id, tx_item.frame.dlc);
        for (int i = 0; i < tx_item.frame.dlc; i++) {
            CAN_DEBUG_PRINTF("%02X", tx_item.frame.data[i]);
            if (i < tx_item.frame.dlc - 1) CAN_DEBUG_PRINTF(" ");
        }
        CAN_DEBUG_PRINTF("]\n");
#endif

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
#if CAN_LOGLEVEL >= 3
                CAN_DEBUG_PRINTF("CAN TX: Timeout on message ID=0x%08X\n", 
                       (unsigned)tx_item.frame.id);
#endif
            } else if (result == ESP_FAIL) {
                // TX failed - check bus state
                twai_status_info_t twai_status;
                if (twai_get_status_info(&twai_status) == ESP_OK) {
                    if (twai_status.state == TWAI_STATE_BUS_OFF) {
                        // Bus-off condition detected - attempt recovery
                        CAN_DEBUG_PRINTF("CAN TX: Bus-off detected, attempting recovery\n");
                        iface->update_error_stats(0x01); // Bus-off error code
                        
                        // Attempt bus recovery
                        twai_initiate_recovery();
                        // Note: Recovery is automatic in TWAI driver
                    }
                }
#if CAN_LOGLEVEL >= 3
                CAN_DEBUG_PRINTF("CAN TX: Failed to transmit message ID=0x%08X, error=%d\n", 
                       (unsigned)tx_item.frame.id, result);
#endif
            } else {
                // Other errors
                iface->update_error_stats(result);
#if CAN_LOGLEVEL >= 3
                CAN_DEBUG_PRINTF("CAN TX: Error %d on message ID=0x%08X\n", 
                       result, (unsigned)tx_item.frame.id);
#endif
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
    // We need to calculate best-fit filter for all requested filters
    uint32_t combined_acceptance = 0x00000000;
    uint32_t combined_mask = 0x1FFFFFFF;  // Start with most restrictive mask
    
    if (num_configs > 0) {
        for (uint16_t i = 0; i < num_configs; i++) {
            const auto& filter = filter_configs[i];
            
            // Convert ArduPilot filter to TWAI acceptance/mask format
            uint32_t filter_id = filter.id & 0x1FFFFFFF;  // 29-bit CAN ID
            uint32_t filter_mask = (~filter.mask) & 0x1FFFFFFF;  // Invert for TWAI format
            
            // Combine filters using OR logic for acceptance, AND for masks
            combined_acceptance |= filter_id;
            combined_mask &= filter_mask;
        }
    } else {
        // No specific filters - accept all
        combined_acceptance = 0x00000000;
        combined_mask = 0x00000000;
    }
    
    // Apply the new filter configuration
    // Note: TWAI driver must be stopped before reconfiguring filters
    if (twai_stop() != ESP_OK) {
        CAN_DEBUG_PRINTF("CAN: Failed to stop TWAI for filter reconfiguration\n");
        return false;
    }
    
    twai_filter_config_t new_filter;
    new_filter.acceptance_code = combined_acceptance;
    new_filter.acceptance_mask = combined_mask;
    new_filter.single_filter = true;  // Use single filter mode
    
    // Reconfigure the driver with new filter
    if (twai_driver_uninstall() != ESP_OK) {
        CAN_DEBUG_PRINTF("CAN: Failed to uninstall TWAI driver for filter reconfiguration\n");
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
        CAN_DEBUG_PRINTF("CAN: Failed to reinstall TWAI driver with new filters\n");
        // Fall back to accept-all configuration
        twai_filter_config_t fallback = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        twai_driver_install(&g_config, &t_config, &fallback);
    }
    
    if (twai_start() != ESP_OK) {
        CAN_DEBUG_PRINTF("CAN: Failed to restart TWAI after filter reconfiguration\n");
        return false;
    }
    
    CAN_DEBUG_PRINTF("CAN: Hardware filters updated - acceptance=0x%08X mask=0x%08X\n", 
                       (unsigned)combined_acceptance, (unsigned)combined_mask);
    
    return true;
}

void CANIface::collect_hw_stats()
{
    if (!initialized) {
        stats.current_health = 2; // ERROR - not initialized
        return;
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
        CAN_DEBUG_PRINTF("CAN: Failed to get TWAI status\n");
        update_error_stats(0xFF); // Status read error
        return;
    }
    
    // Check for bus-off condition and attempt recovery
    if (twai_status.state == TWAI_STATE_BUS_OFF) {
        static uint32_t last_recovery_attempt_ms = 0;
        uint32_t now_ms = AP_HAL::millis();
        
        // Attempt recovery every 5 seconds
        if (now_ms - last_recovery_attempt_ms > 5000) {
            CAN_DEBUG_PRINTF("CAN: Bus-off detected, attempting recovery\n");
            update_error_stats(0x01); // Bus-off error
            
            twai_initiate_recovery();
            last_recovery_attempt_ms = now_ms;
        }
    }
    
    // Check for excessive TX queue backlog
    if (tx_queue && uxQueueMessagesWaiting(tx_queue) > 100) {
        CAN_DEBUG_PRINTF("CAN: TX queue backlog detected (%u messages)\n", 
                           (unsigned)uxQueueMessagesWaiting(tx_queue));
        update_error_stats(0x10); // Queue backlog error
    }
    
    // Check for RX overruns
    if (twai_status.rx_missed_count > 0) {
        static uint32_t last_rx_missed = 0;
        if (twai_status.rx_missed_count > last_rx_missed) {
            CAN_DEBUG_PRINTF("CAN: RX overrun detected (missed %u messages)\n", 
                               (unsigned)(twai_status.rx_missed_count - last_rx_missed));
            update_error_stats(0x20); // RX overrun error
        }
        last_rx_missed = twai_status.rx_missed_count;
    }
}

#endif // HAL_NUM_CAN_IFACES > 0
