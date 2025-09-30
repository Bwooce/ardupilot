
#include "CANIface.h"

#if HAL_NUM_CAN_IFACES > 0

#include <AP_HAL/AP_HAL.h>
#include "Scheduler.h"           // For register_task_with_watchdog()
#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_debug_helpers.h"  // For esp_backtrace_print()
#include "esp_task_wdt.h"       // For esp_task_wdt_add()

extern const AP_HAL::HAL& hal;

using namespace ESP32;

// Use consistent ESP32 debug system controlled by ESP32_DEBUG_LVL parameter
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
    current_bitrate(500000),
    tx_tracker_index(0)
{
    // Initialize tx_tracker array
    memset(tx_tracker, 0, sizeof(tx_tracker));
}

bool CANIface::init(const uint32_t bitrate, const OperatingMode mode)
{
    // Log level is now controlled by ESP32_Params.cpp for consistency
    // Removed local override to avoid conflicts
    
    CAN_DEBUG_INFO("CAN interface %d initialized - bitrate=%u", instance, (unsigned)bitrate);
    CAN_DEBUG_INFO("CAN debug test - you should see this message!");
    CAN_DEBUG_VERBOSE("CAN_LOGLEVEL=%d, ESP-IDF log level set appropriately", CAN_LOGLEVEL);
    
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

    // Wait for bus to become ready (needs to see 11 recessive bits)
    // The TWAI controller may stay in STOPPED state until it detects bus activity
    twai_status_info_t init_status;
    uint32_t wait_start_ms = AP_HAL::millis();
    bool bus_ready = false;

    while ((AP_HAL::millis() - wait_start_ms) < 2000) {  // Wait up to 2 seconds
        if (twai_get_status_info(&init_status) == ESP_OK) {
            if (init_status.state == TWAI_STATE_RUNNING) {
                CAN_DEBUG_INFO("TWAI transitioned to RUNNING state after %lums",
                              (unsigned long)(AP_HAL::millis() - wait_start_ms));
                bus_ready = true;
                break;
            }
        }
        hal.scheduler->delay_microseconds(1000);  // 1ms delay with watchdog feeding
    }

    if (!bus_ready) {
        // Log warning but don't fail - the bus might become ready later when other nodes appear
        if (twai_get_status_info(&init_status) == ESP_OK) {
            CAN_DEBUG_WARN("TWAI did not reach RUNNING state after 2s (state=%d). Bus may need other nodes.",
                          init_status.state);
        } else {
            CAN_DEBUG_WARN("TWAI did not reach RUNNING state after 2s. Bus may need other nodes.");
        }
        // Continue anyway - TX will handle invalid state errors gracefully
    }

    // Only create queues and tasks if this is the first initialization
    // For recovery, the tasks keep running and will handle the new TWAI state
    if (rx_queue == NULL) {
        CAN_DEBUG_INFO("Creating RX queue...");
        rx_queue = xQueueCreate(512, sizeof(CanRxItem));  // Increased from 128 to prevent drops
        if (rx_queue == NULL) {
            CAN_DEBUG_ERROR("Failed to create RX queue");
            return false;
        }
    } else {
        // Clear existing queue for fresh start
        xQueueReset(rx_queue);
    }

    if (tx_queue == NULL) {
        CAN_DEBUG_INFO("Creating TX queue...");
        tx_queue = xQueueCreate(256, sizeof(CanTxItem));  // Also increased TX queue
        if (tx_queue == NULL) {
            CAN_DEBUG_ERROR("Failed to create TX queue");
            vQueueDelete(rx_queue);  // Clean up RX queue on failure
            rx_queue = NULL;
            return false;
        }
    } else {
        // Clear existing queue for fresh start
        xQueueReset(tx_queue);
    }

    // Only create tasks on first init, not during recovery
    if (rx_task_handle == NULL) {
        CAN_DEBUG_INFO("Creating RX and TX tasks...");
        xTaskCreate(rx_task, "can_rx", 4096, this, 5, &rx_task_handle);
        // Higher priority for TX task to reduce latency in DNA responses
        xTaskCreate(tx_task, "can_tx", 4096, this, 6, &tx_task_handle);
    } else {
        CAN_DEBUG_INFO("Tasks already running, continuing with recovery");
    }

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

    // Register this task with watchdog
    ESP32::Scheduler::register_task_with_watchdog("can_rx");

    CAN_DEBUG_INFO("CAN RX task started for interface %d", iface->instance);

    while (true) {
        // Check if restart is in progress
        if (iface->restart_in_progress) {
            // During restart, just wait without trying to receive
            // Reset watchdog during the wait to prevent timeout
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        twai_message_t message;
        // Try to receive with zero timeout first to drain any waiting messages
        // If no messages, then wait with 1ms timeout
        esp_err_t rx_result = twai_receive(&message, 0);

        // If no message available immediately, wait a bit
        if (rx_result == ESP_ERR_TIMEOUT) {
            rx_result = twai_receive(&message, pdMS_TO_TICKS(1));
        }

        // Reset watchdog periodically, not on every iteration
        static uint32_t last_wdt_reset = 0;
        uint32_t now = xTaskGetTickCount();
        if (now - last_wdt_reset > pdMS_TO_TICKS(100)) {
            esp_task_wdt_reset();
            last_wdt_reset = now;
        }

        // Check again if restart happened while we were blocked in receive
        if (iface->restart_in_progress) {
            // Discard any received message and wait for restart to complete
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }


#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
        // Check what TWAI received for GetNodeInfo responses
        if (rx_result == ESP_OK && (message.flags & TWAI_MSG_FLAG_EXTD)) {
            // Check if this looks like a GetNodeInfo response (service type 1)
            if ((message.identifier & 0xFF80) == 0x0180) { // Service frame, type 1
                bool is_response = ((message.identifier >> 15) & 1) == 0;
                if (is_response) {
                    uint8_t dest = (message.identifier >> 8) & 0x7F;
                    uint8_t src = message.identifier & 0x7F;
                    static uint32_t rx_resp_count = 0;
                    rx_resp_count++;
                    if (rx_resp_count <= 20) {
                        ESP_LOGI("TWAI_RX", "Received GetNodeInfo response #%lu: TWAI ID=0x%08lX",
                                 (unsigned long)rx_resp_count, (unsigned long)message.identifier);
                        ESP_LOGI("TWAI_RX", "  From node %d to node %d",
                                 src, dest);

                        // Log first few data bytes
                        if (message.data_length_code >= 4) {
                            ESP_LOGI("TWAI_RX", "  Data: %02X %02X %02X %02X ... %02X",
                                     message.data[0], message.data[1], message.data[2], message.data[3],
                                     message.data[message.data_length_code - 1]);
                        }
                    }
                }
            }
        }
#endif

        if (rx_result != ESP_OK) {
            // Handle RX errors with enhanced detection
            if (rx_result == ESP_ERR_TIMEOUT) {
                // Normal timeout - no action needed
                continue;
            } else if (rx_result == ESP_FAIL) {
                // RX failure - update stats without logging in hot path
                iface->update_error_stats(0x03); // RX fail error
            } else {
                // Other RX errors - just update stats
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
        if (message.extd) {
            rx_item.frame.id |= AP_HAL::CANFrame::FlagEFF;  // Set extended frame flag
        }
        rx_item.frame.dlc = message.data_length_code;
        memcpy(rx_item.frame.data, message.data, message.data_length_code);
        rx_item.flags = 0;
        
        // Check if this is a self-transmitted frame
        // Use hash of CAN ID to start search for better cache locality
        bool is_self_tx = false;
        bool loopback_requested = false;
        uint8_t start_idx = (message.identifier & 0xFF) % TX_TRACKER_SIZE;
        for (uint8_t j = 0; j < TX_TRACKER_SIZE; j++) {
            uint8_t i = (start_idx + j) % TX_TRACKER_SIZE;
            if (iface->tx_tracker[i].can_id == message.identifier &&
                iface->tx_tracker[i].timestamp_us != 0) {
                is_self_tx = true;
                loopback_requested = iface->tx_tracker[i].loopback_requested;
                iface->tx_tracker[i].timestamp_us = 0; // Clear entry
                break;
            }
        }

        if (is_self_tx && !loopback_requested) {
            // Self-transmitted frame without loopback - drop it silently
            continue;
        }
        
        if (is_self_tx && loopback_requested) {
            rx_item.flags = AP_HAL::CANIface::Loopback;
        }

        // Update statistics
        iface->update_rx_stats();

        if (!iface->add_to_rx_queue(rx_item)) {
            // Queue full - frame dropped silently to avoid slowing down RX
            iface->update_error_stats(0x04); // Queue full error
        }
    }
}

void CANIface::tx_task(void *arg)
{
    CANIface *iface = (CANIface *)arg;

    // Register this task with watchdog
    ESP32::Scheduler::register_task_with_watchdog("can_tx");

    CanTxItem tx_item;

    while (true) {
        // Check if restart is in progress
        if (iface->restart_in_progress) {
            // During restart, just wait without trying to transmit
            // Reset watchdog during the wait to prevent timeout
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Wait for a frame from the queue
        if (xQueueReceive(iface->tx_queue, &tx_item, pdMS_TO_TICKS(100)) != pdPASS) {
            // Reset watchdog after queue wait timeout
            esp_task_wdt_reset();
            continue; // No message available, go back to outer loop
        }

        // Reset watchdog after successful queue receive
        esp_task_wdt_reset();

        // Check if this frame has expired
        uint64_t now_us = AP_HAL::micros64();
        if (tx_item.deadline_us != 0 && now_us > tx_item.deadline_us) {
            static uint32_t expired_count = 0;
            expired_count++;
            // Only log first few expired frames to avoid spam
            if (expired_count <= 5 || expired_count % 10 == 0) {
                CAN_DEBUG_VERBOSE("Dropped expired frame ID=0x%08X (deadline=%llu, now=%llu) [count=%lu]",
                       (unsigned)tx_item.frame.id, tx_item.deadline_us, now_us, (unsigned long)expired_count);
            }
            continue; // Skip expired frame
        }

        // Verbose frame-by-frame TX logging
        char data_str[32] = {0};
        for (int i = 0; i < tx_item.frame.dlc && i < 8; i++) {
            snprintf(data_str + (i * 3), sizeof(data_str) - (i * 3), "%02X ", tx_item.frame.data[i]);
        }
        CAN_DEBUG_VERBOSE("TX: ID=0x%08X DLC=%d DATA=[%s]", (unsigned)tx_item.frame.id, tx_item.frame.dlc, data_str);

        twai_message_t message;
        memset(&message, 0, sizeof(message));
        
        // Check if this is an extended frame (bit 31 set means extended)
        if (tx_item.frame.isExtended()) {
            // Extended frame: strip the FlagEFF bit and use only the 29-bit ID
            message.identifier = tx_item.frame.id & AP_HAL::CANFrame::MaskExtID;
            message.extd = 1;
            
            // Validate that the ID is within 29-bit range
            if (message.identifier > 0x1FFFFFFF) {
                CAN_DEBUG_ERROR("Invalid extended CAN ID 0x%08X (exceeds 29-bit max), masking to 29 bits", 
                               (unsigned)tx_item.frame.id);
                message.identifier &= 0x1FFFFFFF;
            }
        } else {
            // Standard frame: use only the 11-bit ID
            message.identifier = tx_item.frame.id & AP_HAL::CANFrame::MaskStdID;
            message.extd = 0;
            
            // Validate that the ID is within 11-bit range
            if (message.identifier > 0x7FF) {
                CAN_DEBUG_ERROR("Invalid standard CAN ID 0x%08X (exceeds 11-bit max), masking to 11 bits", 
                               (unsigned)tx_item.frame.id);
                message.identifier &= 0x7FF;
            }
        }
        
        // Validate and clamp DLC to valid range
        if (tx_item.frame.dlc > 8) {
            CAN_DEBUG_ERROR("Invalid DLC %d for message ID=0x%08X, clamping to 8", 
                           tx_item.frame.dlc, (unsigned)tx_item.frame.id);
            message.data_length_code = 8;
        } else {
            message.data_length_code = tx_item.frame.dlc;
        }
        
        message.rtr = tx_item.frame.isRemoteTransmissionRequest() ? 1 : 0;
        message.ss = 0;    // Allow retransmission
        message.self = 0;  // Not self-reception
        memcpy(message.data, tx_item.frame.data, message.data_length_code);
        
        // Debug CAN ID issues
        // DroneCAN extended frame format: 
        // Bits 28-24: Priority (5 bits)
        // Bits 23-8: Message type (16 bits) 
        // Bit 7: Service not message
        // Bits 6-0: Source node ID (7 bits)
        //uint8_t priority = (message.identifier >> 24) & 0x1F;  // Unused for now
        //uint16_t msg_type = (message.identifier >> 8) & 0xFFFF;  // Unused after removing type 342 debug
        //bool is_service = (message.identifier >> 7) & 0x01;  // Unused for now
        //uint8_t source_node = message.identifier & 0x7F;  // Unused after removing type 342 debug
        
        // FlagEFF (bit 31) is normal for extended frame format - no need to log
        
        // CAN message counting disabled - not needed in production
        
        
        // Commented out verbose logging

        // Single transmission attempt for DroneCAN compliance
        // DroneCAN handles reliability at protocol layer, HAL retries violate timing assumptions
        const TickType_t timeout_ms = 10; // Allow sufficient time for transmission

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
        // Check if this is a GetNodeInfo response and log what we're sending
        if ((message.flags & TWAI_MSG_FLAG_EXTD) &&
            ((message.identifier & 0xFF80) == 0x0180)) { // Service frame, type 1
            bool is_response = ((message.identifier >> 15) & 1) == 0;
            if (is_response) {
                uint8_t dest = (message.identifier >> 8) & 0x7F;
                uint8_t src = message.identifier & 0x7F;
                static uint32_t resp_count = 0;
                resp_count++;
                if (resp_count <= 20) {
                    ESP_LOGI("TWAI_TX", "Sending GetNodeInfo response #%lu: CAN ID=0x%08lX",
                             (unsigned long)resp_count, (unsigned long)message.identifier);
                    ESP_LOGI("TWAI_TX", "  From node %d to node %d", src, dest);

                    // Log the actual bytes being sent to TWAI
                    ESP_LOGI("TWAI_TX", "  TWAI msg: id=0x%08lX, flags=0x%02X, dlc=%d",
                             (unsigned long)message.identifier, message.flags, message.data_length_code);
                }
            }
        }
#endif

        // Special debug for GetNodeInfo responses
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
        if ((message.identifier & 0xFF80) == 0x0180 && ((message.identifier >> 15) & 1) == 0) {
            uint8_t dest_before = (message.identifier >> 8) & 0x7F;
            ESP_LOGW("TWAI_BUG", "About to transmit GetNodeInfo response to node %d, ID=0x%08lX",
                     dest_before, (unsigned long)message.identifier);
        }
#endif

        esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(timeout_ms));

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
        if ((message.identifier & 0xFF80) == 0x0180 && ((message.identifier >> 15) & 1) == 0) {
            uint8_t dest_after = (message.identifier >> 8) & 0x7F;
            ESP_LOGW("TWAI_BUG", "After transmit GetNodeInfo response: dest=%d, result=%d",
                     dest_after, result);
            // Check if TWAI modified the message
            if (dest_after != ((message.identifier >> 8) & 0x7F)) {
                ESP_LOGE("TWAI_BUG", "TWAI MODIFIED THE CAN ID DURING TRANSMISSION!");
            }
        }
#endif
        
        if (result == ESP_OK) {
            // Track for self-reception filtering
            iface->tx_tracker[iface->tx_tracker_index].can_id = message.identifier;
            iface->tx_tracker[iface->tx_tracker_index].timestamp_us = AP_HAL::micros64();
            iface->tx_tracker[iface->tx_tracker_index].loopback_requested = (tx_item.flags & AP_HAL::CANIface::Loopback) != 0;
            iface->tx_tracker_index = (iface->tx_tracker_index + 1) % TX_TRACKER_SIZE;
        }
        
        // Update statistics and handle errors
        iface->update_tx_stats(result == ESP_OK);
        
        if (result != ESP_OK) {
            // Handle specific TWAI error conditions
            if (result == ESP_ERR_TIMEOUT) {
                // TX timeout - bus may be congested or disconnected
                // Count timeouts but don't log them - stats are tracked elsewhere
                static uint32_t timeout_count = 0;
                timeout_count++;
                // Disabled - CAN stats are available via MAVLink
                // Only enable for debugging CAN issues
#if 0
                if (timeout_count <= 5 || (timeout_count % 1000) == 0) {
                    CAN_DEBUG_WARN("TX timeout on message ID=0x%08X (total timeouts=%lu)",
                                   (unsigned)(tx_item.frame.id & AP_HAL::CANFrame::MaskExtID),
                                   (unsigned long)timeout_count);
                }
#endif
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
                // Reduce log spam - only log occasionally
                static uint32_t fail_count = 0;
                fail_count++;
                if (fail_count <= 5 || (fail_count % 100) == 0) {
                    CAN_DEBUG_ERROR("Failed to transmit message ID=0x%08X, error=%d (count=%lu)", 
                                    (unsigned)(tx_item.frame.id & AP_HAL::CANFrame::MaskExtID), 
                                    result, (unsigned long)fail_count);
                }
            } else if (result == ESP_ERR_INVALID_STATE) {
                // TWAI driver in invalid state - likely no other nodes on bus or disconnection
                // Continue processing queue to prevent buildup, just can't transmit yet
                static uint32_t last_invalid_state_log_ms = 0;
                static uint32_t invalid_state_count = 0;
                static uint32_t messages_dropped = 0;
                static uint32_t last_recovery_attempt_ms = 0;
                static uint32_t recovery_backoff_ms = 100;  // Start with 100ms backoff
                uint32_t now_ms = AP_HAL::millis();
                invalid_state_count++;
                messages_dropped++;  // Count this as a dropped message

                // Diagnostic: Check actual TWAI state when we get invalid state error
                twai_status_info_t diagnostic_status;
                bool got_status = (twai_get_status_info(&diagnostic_status) == ESP_OK);

                if (got_status) {
                    // Log the actual state to verify TX invalid state (0=STOPPED, 1=RUNNING, 2=BUS_OFF, 3=RECOVERING)
                    if (invalid_state_count == 1 || (invalid_state_count % 100) == 0) {
                        const char* state_str = "UNKNOWN";
                        switch (diagnostic_status.state) {
                            case TWAI_STATE_STOPPED: state_str = "STOPPED"; break;
                            case TWAI_STATE_RUNNING: state_str = "RUNNING"; break;
                            case TWAI_STATE_BUS_OFF: state_str = "BUS_OFF"; break;
                            case TWAI_STATE_RECOVERING: state_str = "RECOVERING"; break;
                        }
                        CAN_DEBUG_ERROR("TX ESP_ERR_INVALID_STATE: state=%s, tx_err=%u, rx_err=%u, to_tx=%lu, to_rx=%lu",
                                       state_str,
                                       (unsigned)diagnostic_status.tx_error_counter,
                                       (unsigned)diagnostic_status.rx_error_counter,
                                       (unsigned long)diagnostic_status.msgs_to_tx,
                                       (unsigned long)diagnostic_status.msgs_to_rx);
                    }
                }

                // Track recovery attempts for escalation
                static uint32_t consecutive_failures = 0;

                // Check for restart with exponential backoff to avoid hammering the bus
                bool should_attempt_recovery = false;
                if (now_ms - last_recovery_attempt_ms >= recovery_backoff_ms) {
                    should_attempt_recovery = true;
                    last_recovery_attempt_ms = now_ms;
                    consecutive_failures++;

                    // Exponential backoff: double the delay each time, max 5 seconds
                    recovery_backoff_ms = MIN(recovery_backoff_ms * 2, 5000);
                }

                if (should_attempt_recovery) {
                    iface->update_error_stats(0x08); // Invalid state error

                    // After 10 consecutive failures, escalate to full driver restart
                    if (consecutive_failures > 10) {
                        CAN_DEBUG_ERROR("TWAI recovery failed %lu times, attempting full driver restart",
                                       (unsigned long)consecutive_failures);
                        iface->attempt_driver_restart();
                        consecutive_failures = 0;
                        recovery_backoff_ms = 100;  // Reset backoff after full restart
                    } else if (got_status) {
                        // Try recovery based on current state
                        if (diagnostic_status.state == TWAI_STATE_STOPPED) {
                            // Try to start the bus again, but with backoff
                            CAN_DEBUG_INFO("TWAI is STOPPED, attempting to start (attempt %lu, backoff=%lums)",
                                          (unsigned long)consecutive_failures,
                                          (unsigned long)recovery_backoff_ms);
                            esp_err_t start_res = twai_start();
                            if (start_res != ESP_OK) {
                                CAN_DEBUG_ERROR("Failed to restart TWAI: %d", start_res);
                            }
                        } else if (diagnostic_status.state == TWAI_STATE_BUS_OFF) {
                            // Need recovery - but let it recover naturally first
                            CAN_DEBUG_INFO("TWAI is BUS_OFF, waiting for auto-recovery (attempt %lu)",
                                          (unsigned long)consecutive_failures);
                            // Don't call twai_initiate_recovery() immediately - let TWAI handle it
                            // Only force recovery after multiple attempts
                            if (consecutive_failures > 3) {
                                twai_initiate_recovery();
                            }
                        } else if (diagnostic_status.state == TWAI_STATE_RECOVERING) {
                            // Already recovering, just wait
                            CAN_DEBUG_INFO("TWAI is RECOVERING, waiting...");
                        } else if (diagnostic_status.state == TWAI_STATE_RUNNING) {
                            // Success! Reset everything
                            CAN_DEBUG_INFO("TWAI recovered to RUNNING state");
                            consecutive_failures = 0;
                            recovery_backoff_ms = 100;
                        }
                    } else {
                        // Couldn't get status, try full restart
                        CAN_DEBUG_ERROR("Can't get TWAI status, attempting full restart");
                        iface->attempt_driver_restart();
                        consecutive_failures = 0;
                        recovery_backoff_ms = 100;
                    }
                    invalid_state_count = 0;  // Reset counter after action
                }

                // Log less frequently to avoid spam
                if (now_ms - last_invalid_state_log_ms > 10000) {  // Log at most every 10 seconds
                    CAN_DEBUG_ERROR("TWAI invalid state (no other nodes?) - count=%lu, dropped=%lu",
                                   (unsigned long)invalid_state_count, (unsigned long)messages_dropped);
                    last_invalid_state_log_ms = now_ms;
                    messages_dropped = 0;  // Reset dropped count after logging
                }

                // Important: Continue loop to process next message instead of blocking
                // This prevents TX queue from filling up during invalid state
                continue;
            } else if (result == ESP_ERR_INVALID_ARG) {
                // Invalid argument - possibly corrupted message data
                // Rate limit this error to avoid spam
                static uint32_t last_invalid_arg_log_ms = 0;
                static uint32_t invalid_arg_count = 0;
                uint32_t now_ms = AP_HAL::millis();
                invalid_arg_count++;
                
                if (now_ms - last_invalid_arg_log_ms > 5000) {  // Log at most every 5 seconds
                    CAN_DEBUG_ERROR("TWAI invalid argument errors: %lu occurrences (last ID=0x%08X)", 
                                   (unsigned long)invalid_arg_count, (unsigned)tx_item.frame.id);
                    last_invalid_arg_log_ms = now_ms;
                    invalid_arg_count = 0;  // Reset counter
                }
                iface->update_error_stats(0x09); // Invalid argument error
            } else {
                // Other errors
                iface->update_error_stats(result);
                CAN_DEBUG_ERROR("Error %d on message ID=0x%08X", result, 
                                (unsigned)(tx_item.frame.id & AP_HAL::CANFrame::MaskExtID));
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
    
    // ESP-IDF 5.5 configuration
    g_config.alerts_enabled = TWAI_ALERT_ALL;  // Enable all alerts
    g_config.clkout_divider = 0;  // No clock output
    g_config.tx_queue_len = 20;   // Increase TX queue
    g_config.intr_flags = ESP_INTR_FLAG_LEVEL1;  // Level 1 interrupt
    
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

    // Periodic stats disabled - available via MAVLink instead
#if 0
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
#endif
    
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

void CANIface::attempt_driver_restart()
{
    static uint32_t last_restart_ms = 0;
    static uint32_t restart_count = 0;
    uint32_t now_ms = AP_HAL::millis();

    // Rate limit restarts to prevent restart loops
    if (now_ms - last_restart_ms < 5000) {  // Minimum 5 seconds between restarts (reduced from 30s)
        // Only log periodically to avoid spam
        static uint32_t last_log_ms = 0;
        if (now_ms - last_log_ms > 2000) {  // Log at most every 2 seconds
            CAN_DEBUG_INFO("Driver restart rate-limited (last restart %lu ms ago)", (unsigned long)(now_ms - last_restart_ms));
            last_log_ms = now_ms;
        }
        return;
    }

    // Prevent multiple simultaneous restart attempts
    if (restart_in_progress) {
        CAN_DEBUG_INFO("Driver restart already in progress, skipping");
        return;
    }
    
    restart_count++;
    last_restart_ms = now_ms;
    
    // Only log restart attempts periodically to reduce spam
    if (restart_count <= 3 || (restart_count % 10) == 0) {
        CAN_DEBUG_ERROR("TWAI driver restart #%lu (invalid state)", (unsigned long)restart_count);
    }
    
    // Store current configuration
    uint32_t saved_bitrate = current_bitrate;
    
    // Stop and uninstall driver (may fail if in invalid state, but continue anyway)
    // Set a flag to signal RX/TX tasks to pause during restart
    restart_in_progress = true;

    // Give RX/TX tasks time to see the flag and exit any blocking calls
    // The RX timeout is 100ms, so wait at least that long
    // Feed watchdog during this delay
    hal.scheduler->delay(200); // 200ms to ensure RX task sees timeout and checks flag, with watchdog feeding

    // Now attempt to stop TWAI - use non-blocking approach
    esp_err_t result = ESP_OK;

    // Check current state first
    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        CAN_DEBUG_INFO("TWAI state before stop: %d, msgs_to_tx: %lu, msgs_to_rx: %lu",
                       status.state, (unsigned long)status.msgs_to_tx, (unsigned long)status.msgs_to_rx);

        // Only try to stop if not already stopped
        if (status.state != TWAI_STATE_STOPPED) {
            result = twai_stop();
            if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
                CAN_DEBUG_WARN("TWAI stop failed (error %d), forcing uninstall", result);
            }
            // Give driver time to fully stop
            hal.scheduler->delay(50); // Allow TWAI to fully stop
        }
    }

    // Always try to uninstall, even if stop failed
    result = twai_driver_uninstall();
    if (result != ESP_OK) {
        CAN_DEBUG_WARN("TWAI uninstall failed (error %d), continuing anyway", result);
    }

    // Brief delay to let hardware settle
    hal.scheduler->delay(10); // 10ms with watchdog feeding
    
    // Clear queues (they should be empty after stop/uninstall, but be safe)
    if (rx_queue) {
        xQueueReset(rx_queue);
    }
    if (tx_queue) {
        xQueueReset(tx_queue);
    }
    
    // Reinitialize with saved configuration
    initialized = false;
    bool reinit_ok = init(saved_bitrate, NormalMode);

    if (reinit_ok) {
        // Give TWAI driver time to fully stabilize before allowing tasks to resume
        hal.scheduler->delay(50); // 50ms for TWAI to stabilize, with watchdog feeding

        // Now clear the restart flag to allow tasks to resume
        restart_in_progress = false;

        CAN_DEBUG_INFO("TWAI driver restart successful after %lu restarts", (unsigned long)restart_count);
        update_error_stats(0x00); // Clear error state
    } else {
        // Clear flag even on failure to prevent permanent task blocking
        restart_in_progress = false;

        CAN_DEBUG_ERROR("TWAI driver restart failed - interface may be unusable");
    }
}

#endif // HAL_NUM_CAN_IFACES > 0
