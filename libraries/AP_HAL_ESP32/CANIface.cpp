
#include "CANIface.h"

#if HAL_NUM_CAN_IFACES > 0

#include <AP_HAL/AP_HAL.h>
#include "Scheduler.h"           // For register_task_with_watchdog()
#include "driver/gpio.h"
#include "esp_debug_helpers.h"  // For esp_backtrace_print()
#include "esp_task_wdt.h"       // For esp_task_wdt_add()
#include "esp_timer.h"         // For esp_timer_get_time() in ISR

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

// --- ISR Callbacks (IRAM_ATTR required for cache safety) ---

bool IRAM_ATTR CANIface::on_rx_done(twai_node_handle_t handle,
                                      const twai_rx_done_event_data_t *edata,
                                      void *user_ctx)
{
    CANIface *iface = static_cast<CANIface*>(user_ctx);

    // Receive the frame from hardware in ISR context
    uint8_t rx_buf[TWAI_FRAME_MAX_LEN];
    twai_frame_t rx_frame = {};
    rx_frame.buffer = rx_buf;
    rx_frame.buffer_len = sizeof(rx_buf);

    if (twai_node_receive_from_isr(handle, &rx_frame) != ESP_OK) {
        return false;
    }

    // Build ISR-safe frame struct and push to queue
    IsrRxFrame isr_frame;
    isr_frame.id = rx_frame.header.id;
    isr_frame.dlc = (rx_frame.header.dlc > 8) ? 8 : rx_frame.header.dlc;
    isr_frame.is_extended = rx_frame.header.ide;
    isr_frame.is_rtr = rx_frame.header.rtr;
    isr_frame.timestamp_us = (uint64_t)esp_timer_get_time();
    memcpy(isr_frame.data, rx_buf, isr_frame.dlc);

    BaseType_t higher_priority_woken = pdFALSE;
    xQueueSendFromISR(iface->isr_rx_queue, &isr_frame, &higher_priority_woken);

    // Wake rx_task instantly instead of waiting for 100ms poll timeout
    if (iface->rx_task_handle != NULL) {
        vTaskNotifyGiveFromISR(iface->rx_task_handle, &higher_priority_woken);
    }

    return higher_priority_woken == pdTRUE;
}

bool IRAM_ATTR CANIface::on_tx_done(twai_node_handle_t handle,
                                      const twai_tx_done_event_data_t *edata,
                                      void *user_ctx)
{
    // TX completion notification -- currently unused since twai_node_transmit()
    // blocks until queued. Could be used for async TX tracking in future.
    (void)handle;
    (void)edata;
    (void)user_ctx;
    return false;
}

bool IRAM_ATTR CANIface::on_state_change(twai_node_handle_t handle,
                                           const twai_state_change_event_data_t *edata,
                                           void *user_ctx)
{
    CANIface *iface = static_cast<CANIface*>(user_ctx);
    iface->hw_error_state = edata->new_sta;
    return false;
}

bool IRAM_ATTR CANIface::on_error(twai_node_handle_t handle,
                                    const twai_error_event_data_t *edata,
                                    void *user_ctx)
{
    (void)handle;
    (void)edata;
    (void)user_ctx;
    return false;
}

// --- Helper ---

CANIface::BusState CANIface::error_state_to_bus_state(twai_error_state_t state) const
{
    switch (state) {
        case TWAI_ERROR_ACTIVE:
            return BusState::GOOD;
        case TWAI_ERROR_WARNING:
        case TWAI_ERROR_PASSIVE:
            return BusState::WARNING;
        case TWAI_ERROR_BUS_OFF:
            return BusState::BUS_OFF;
        default:
            return BusState::BUS_OFF;
    }
}

// --- Init ---

bool CANIface::init(const uint32_t bitrate)
{
    CAN_DEBUG_INFO("CAN interface %d initialized - bitrate=%u", instance, (unsigned)bitrate);
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

    // Configure the TWAI node using the new esp_twai API
    twai_onchip_node_config_t node_config = {};
    node_config.io_cfg.tx = tx_pin;
    node_config.io_cfg.rx = rx_pin;
    node_config.io_cfg.quanta_clk_out = (gpio_num_t)-1;
    node_config.io_cfg.bus_off_indicator = (gpio_num_t)-1;
    node_config.clk_src = (twai_clock_source_t)0;  // TWAI_CLK_SRC_DEFAULT
    node_config.bit_timing.bitrate = bitrate;
    node_config.bit_timing.sp_permill = 875;  // 87.5% sample point (CAN 2.0 standard)
    node_config.fail_retry_cnt = -1;  // Retry forever (hardware retransmission)
    node_config.tx_queue_depth = 20;
    node_config.intr_priority = 2;  // Higher priority than UART to prevent MAVLink blocking CAN

    CAN_DEBUG_INFO("Creating TWAI node...");
    esp_err_t err = twai_new_node_onchip(&node_config, &_node);
    if (err != ESP_OK) {
        CAN_DEBUG_ERROR("TWAI node creation failed with error %d", err);
        return false;
    }

    // Register ISR callbacks
    twai_event_callbacks_t cbs = {};
    cbs.on_rx_done = on_rx_done;
    cbs.on_tx_done = on_tx_done;
    cbs.on_state_change = on_state_change;
    cbs.on_error = on_error;
    err = twai_node_register_event_callbacks(_node, &cbs, this);
    if (err != ESP_OK) {
        CAN_DEBUG_ERROR("TWAI callback registration failed: %d", err);
        twai_node_delete(_node);
        _node = nullptr;
        return false;
    }

    // Configure accept-all filter
    twai_mask_filter_config_t filter_cfg = {};
    filter_cfg.id = 0;
    filter_cfg.mask = 0;  // 0 = don't care (accept all)
    filter_cfg.is_ext = 0;
    filter_cfg.no_classic = 0;
    filter_cfg.no_fd = 1;  // No FD frames (classic CAN only)
    err = twai_node_config_mask_filter(_node, 0, &filter_cfg);
    if (err != ESP_OK) {
        CAN_DEBUG_WARN("TWAI filter config failed: %d (continuing with defaults)", err);
    }

    CAN_DEBUG_INFO("Enabling TWAI node...");
    err = twai_node_enable(_node);
    if (err != ESP_OK) {
        CAN_DEBUG_ERROR("TWAI enable failed with error %d", err);
        twai_node_delete(_node);
        _node = nullptr;
        return false;
    }
    CAN_DEBUG_INFO("TWAI node enabled successfully");

    // Check initial state
    twai_node_status_t node_status;
    twai_node_record_t node_record;
    if (twai_node_get_info(_node, &node_status, &node_record) == ESP_OK) {
        CAN_DEBUG_INFO("Initial state after enable: error_state=%d, TX_ERR=%u, RX_ERR=%u",
                      node_status.state, node_status.tx_error_count, node_status.rx_error_count);
    }

    // Wait for bus to become active (needs to see 11 recessive bits)
    uint32_t wait_start_ms = AP_HAL::millis();
    bool bus_ready = false;

    while ((AP_HAL::millis() - wait_start_ms) < 2000) {
        if (twai_node_get_info(_node, &node_status, &node_record) == ESP_OK) {
            if (node_status.state == TWAI_ERROR_ACTIVE) {
                CAN_DEBUG_INFO("TWAI reached ERROR_ACTIVE after %lums, TX_ERR=%u, RX_ERR=%u",
                              (unsigned long)(AP_HAL::millis() - wait_start_ms),
                              node_status.tx_error_count, node_status.rx_error_count);
                bus_ready = true;
                break;
            }
        }
        hal.scheduler->delay_microseconds(1000);
    }

    if (!bus_ready) {
        if (twai_node_get_info(_node, &node_status, &node_record) == ESP_OK) {
            CAN_DEBUG_WARN("TWAI did not reach ERROR_ACTIVE after 2s (state=%d, TX_ERR=%u, RX_ERR=%u). Bus may need other nodes.",
                          node_status.state, node_status.tx_error_count, node_status.rx_error_count);
        } else {
            CAN_DEBUG_WARN("TWAI did not reach ERROR_ACTIVE after 2s. Bus may need other nodes.");
        }
    }

    // Create ISR RX queue (ISR callback → rx_task bridge)
    if (isr_rx_queue == NULL) {
        isr_rx_queue = xQueueCreate(128, sizeof(IsrRxFrame));
        if (isr_rx_queue == NULL) {
            CAN_DEBUG_ERROR("Failed to create ISR RX queue");
            twai_node_disable(_node);
            twai_node_delete(_node);
            _node = nullptr;
            return false;
        }
    } else {
        xQueueReset(isr_rx_queue);
    }

    // Create consumer RX queue
    if (rx_queue == NULL) {
        CAN_DEBUG_INFO("Creating RX queue...");
        rx_queue = xQueueCreate(512, sizeof(CanRxItem));
        if (rx_queue == NULL) {
            CAN_DEBUG_ERROR("Failed to create RX queue");
            return false;
        }
    } else {
        xQueueReset(rx_queue);
    }

    if (tx_queue == NULL) {
        CAN_DEBUG_INFO("Creating TX queue...");
        tx_queue = xQueueCreate(256, sizeof(CanTxItem));
        if (tx_queue == NULL) {
            CAN_DEBUG_ERROR("Failed to create TX queue");
            vQueueDelete(rx_queue);
            rx_queue = NULL;
            return false;
        }
    } else {
        xQueueReset(tx_queue);
    }

    // Only create tasks on first init, not during recovery
    if (rx_task_handle == NULL) {
        CAN_DEBUG_INFO("Creating RX and TX tasks...");
        // Pin to SLOWCPU (Core 1) to separate from Main/UART on Core 0
        // Priority 10 = same as RCOUT, above I2C/IO, below WiFi
        #define SLOWCPU 1
        xTaskCreatePinnedToCore(rx_task, "can_rx", 4096, this, 10, &rx_task_handle, SLOWCPU);
        xTaskCreatePinnedToCore(tx_task, "can_tx", 4096, this, 10, &tx_task_handle, SLOWCPU);
    } else {
        CAN_DEBUG_INFO("Tasks already running, continuing with recovery");
    }

    initialized = true;
    CAN_DEBUG_INFO("Interface %d initialization complete", instance);

    return true;
}

// --- Send / Receive ---

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

// --- RX Task ---

void CANIface::rx_task(void *arg)
{
    CANIface *iface = (CANIface *)arg;

    // Register this task with watchdog
    ESP32::Scheduler::register_task_with_watchdog("can_rx");

    CAN_DEBUG_INFO("CAN RX task started for interface %d", iface->instance);

    while (true) {
        // Check if restart is in progress
        if (iface->restart_in_progress) {
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Wait for ISR notification (instant wake) with 100ms fallback timeout
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        esp_task_wdt_reset();

        // Check again if restart happened while we were waiting
        if (iface->restart_in_progress) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Drain all queued frames (ISR may batch multiple before task runs)
        IsrRxFrame isr_frame;
        while (xQueueReceive(iface->isr_rx_queue, &isr_frame, 0) == pdPASS) {
            // Convert ISR frame to ArduPilot CAN frame
            CanRxItem rx_item;
            rx_item.timestamp_us = isr_frame.timestamp_us;
            rx_item.frame.id = isr_frame.id;
            if (isr_frame.is_extended) {
                rx_item.frame.id |= AP_HAL::CANFrame::FlagEFF;
            }
            rx_item.frame.dlc = isr_frame.dlc;
            memcpy(rx_item.frame.data, isr_frame.data, isr_frame.dlc);
            rx_item.flags = 0;

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
            // Check for GetNodeInfo responses
            if (isr_frame.is_extended) {
                if ((isr_frame.id & 0xFF80) == 0x0180) { // Service frame, type 1
                    bool is_response = ((isr_frame.id >> 15) & 1) == 0;
                    if (is_response) {
                        uint8_t dest = (isr_frame.id >> 8) & 0x7F;
                        uint8_t src = isr_frame.id & 0x7F;
                        static uint32_t rx_resp_count = 0;
                        rx_resp_count++;
                        if (rx_resp_count <= 20) {
                            ESP_LOGI("TWAI_RX", "Received GetNodeInfo response #%lu: TWAI ID=0x%08lX",
                                     (unsigned long)rx_resp_count, (unsigned long)isr_frame.id);
                            ESP_LOGI("TWAI_RX", "  From node %d to node %d", src, dest);
                            if (isr_frame.dlc >= 4) {
                                ESP_LOGI("TWAI_RX", "  Data: %02X %02X %02X %02X ... %02X",
                                         isr_frame.data[0], isr_frame.data[1],
                                         isr_frame.data[2], isr_frame.data[3],
                                         isr_frame.data[isr_frame.dlc - 1]);
                            }
                        }
                    }
                }
            }
#endif

#if CAN_LOGLEVEL >= 4
            // Verbose frame-by-frame logging
            {
                static uint32_t rx_count = 0;
                rx_count++;

                char data_str[32] = {0};
                for (int i = 0; i < isr_frame.dlc && i < 8; i++) {
                    snprintf(data_str + (i * 3), sizeof(data_str) - (i * 3), "%02X ", isr_frame.data[i]);
                }

                CAN_DEBUG_VERBOSE("RX #%lu: ID=0x%08lX DLC=%d DATA=[%s]",
                                 (unsigned long)rx_count, (unsigned long)isr_frame.id,
                                 isr_frame.dlc, data_str);
            }
#elif CAN_LOGLEVEL >= 3
            // INFO level - summary every 1000 frames
            {
                static uint32_t rx_count_summary = 0;
                rx_count_summary++;
                if ((rx_count_summary % 1000) == 0) {
                    CAN_DEBUG_DEBUG("Received %lu frames, latest ID=0x%08lX",
                                    (unsigned long)rx_count_summary, (unsigned long)isr_frame.id);
                }
            }
#endif

            // Check if this is a self-transmitted frame
            bool is_self_tx = false;
            bool loopback_requested = false;
            uint8_t start_idx = (isr_frame.id & 0xFF) % TX_TRACKER_SIZE;
            for (uint8_t j = 0; j < TX_TRACKER_SIZE; j++) {
                uint8_t i = (start_idx + j) % TX_TRACKER_SIZE;
                if (iface->tx_tracker[i].can_id == isr_frame.id &&
                    iface->tx_tracker[i].timestamp_us != 0) {
                    is_self_tx = true;
                    loopback_requested = iface->tx_tracker[i].loopback_requested;
                    iface->tx_tracker[i].timestamp_us = 0;
                    break;
                }
            }

            if (is_self_tx && !loopback_requested) {
                continue;
            }

            if (is_self_tx && loopback_requested) {
                rx_item.flags = AP_HAL::CANIface::Loopback;
            }

            // Update statistics
            iface->update_rx_stats();

            if (!iface->add_to_rx_queue(rx_item)) {
                iface->update_error_stats(0x04); // Queue full error
            }
        } // end while (xQueueReceive)
    }
}

// --- TX Task ---

void CANIface::tx_task(void *arg)
{
    CANIface *iface = (CANIface *)arg;

    // Register this task with watchdog
    ESP32::Scheduler::register_task_with_watchdog("can_tx");

    CanTxItem tx_item;

    while (true) {
        // Check if restart is in progress
        if (iface->restart_in_progress) {
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Wait for a frame from the queue
        if (xQueueReceive(iface->tx_queue, &tx_item, pdMS_TO_TICKS(100)) != pdPASS) {
            esp_task_wdt_reset();
            continue;
        }

        esp_task_wdt_reset();

        // Check if this frame has expired
        uint64_t now_us = AP_HAL::micros64();
        if (tx_item.deadline_us != 0 && now_us > tx_item.deadline_us) {
            static uint32_t expired_count = 0;
            expired_count++;
            if (expired_count <= 5 || expired_count % 10 == 0) {
                CAN_DEBUG_VERBOSE("Dropped expired frame ID=0x%08X [count=%lu]",
                       (unsigned)tx_item.frame.id, (unsigned long)expired_count);
            }
            continue;
        }

        // Build the new-API frame
        uint8_t tx_buf[TWAI_FRAME_MAX_LEN];
        twai_frame_t frame = {};
        frame.buffer = tx_buf;

        if (tx_item.frame.isExtended()) {
            frame.header.id = tx_item.frame.id & AP_HAL::CANFrame::MaskExtID;
            frame.header.ide = 1;
            if (frame.header.id > 0x1FFFFFFF) {
                CAN_DEBUG_ERROR("Invalid extended CAN ID 0x%08X, masking to 29 bits",
                               (unsigned)tx_item.frame.id);
                frame.header.id &= 0x1FFFFFFF;
            }
        } else {
            frame.header.id = tx_item.frame.id & AP_HAL::CANFrame::MaskStdID;
            frame.header.ide = 0;
            if (frame.header.id > 0x7FF) {
                CAN_DEBUG_ERROR("Invalid standard CAN ID 0x%08X, masking to 11 bits",
                               (unsigned)tx_item.frame.id);
                frame.header.id &= 0x7FF;
            }
        }

        if (tx_item.frame.dlc > 8) {
            CAN_DEBUG_ERROR("Invalid DLC %d for ID=0x%08X, clamping to 8",
                           tx_item.frame.dlc, (unsigned)tx_item.frame.id);
            frame.header.dlc = 8;
        } else {
            frame.header.dlc = tx_item.frame.dlc;
        }

        frame.header.rtr = tx_item.frame.isRemoteTransmissionRequest() ? 1 : 0;
        frame.buffer_len = frame.header.dlc;
        memcpy(tx_buf, tx_item.frame.data, frame.header.dlc);

        CAN_DEBUG_VERBOSE("TX: ID=0x%08X DLC=%d", (unsigned)frame.header.id, frame.header.dlc);

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
        // Log GetNodeInfo responses being sent
        if (frame.header.ide &&
            ((frame.header.id & 0xFF80) == 0x0180)) { // Service frame, type 1
            bool is_response = ((frame.header.id >> 15) & 1) == 0;
            if (is_response) {
                uint8_t dest = (frame.header.id >> 8) & 0x7F;
                uint8_t src = frame.header.id & 0x7F;
                static uint32_t resp_count = 0;
                resp_count++;
                if (resp_count <= 20) {
                    ESP_LOGI("TWAI_TX", "Sending GetNodeInfo response #%lu: CAN ID=0x%08lX",
                             (unsigned long)resp_count, (unsigned long)frame.header.id);
                    ESP_LOGI("TWAI_TX", "  From node %d to node %d", src, dest);
                    ESP_LOGI("TWAI_TX", "  TWAI msg: id=0x%08lX, ide=%d, dlc=%d",
                             (unsigned long)frame.header.id, frame.header.ide, frame.header.dlc);
                }
            }
        }
#endif

        const int timeout_ms = 10;
        esp_err_t result = twai_node_transmit(iface->_node, &frame, timeout_ms);

        if (result == ESP_OK) {
            // Track for self-reception filtering
            iface->tx_tracker[iface->tx_tracker_index].can_id = frame.header.id;
            iface->tx_tracker[iface->tx_tracker_index].timestamp_us = AP_HAL::micros64();
            iface->tx_tracker[iface->tx_tracker_index].loopback_requested =
                (tx_item.flags & AP_HAL::CANIface::Loopback) != 0;
            iface->tx_tracker_index = (iface->tx_tracker_index + 1) % TX_TRACKER_SIZE;
        }

#if CAN_LOGLEVEL >= 2
        // Periodic error counter monitoring
        {
            static uint32_t last_health_check_ms = 0;
            static uint32_t last_tx_err = 0;
            static uint32_t last_rx_err = 0;
            uint32_t now_ms = AP_HAL::millis();

            if (now_ms - last_health_check_ms >= 1000) {
                twai_node_status_t health_status;
                twai_node_record_t health_record;
                if (twai_node_get_info(iface->_node, &health_status, &health_record) == ESP_OK) {
                    int32_t tx_delta = (int32_t)health_status.tx_error_count - (int32_t)last_tx_err;
                    int32_t rx_delta = (int32_t)health_status.rx_error_count - (int32_t)last_rx_err;

                    bool should_log = false;
                    const char* reason = "";

                    if (health_status.tx_error_count > 0 && last_tx_err == 0) {
                        should_log = true;
                        reason = " [FIRST ERROR]";
                    } else if (health_status.tx_error_count >= 96 && last_tx_err < 96) {
                        should_log = true;
                        reason = " [WARNING: approaching bus-off at 128]";
                    } else if (health_status.tx_error_count >= 128) {
                        should_log = true;
                        reason = " [BUS-OFF THRESHOLD]";
                    } else if ((now_ms / 1000) % 5 == 0 && (now_ms - last_health_check_ms) < 2000) {
                        should_log = true;
                        reason = "";
                    }

                    if (should_log) {
                        ESP_LOGI("CAN_HEALTH", "T+%lus: TX_ERR=%u (%+ld), RX_ERR=%u (%+ld), state=%d%s",
                                 (unsigned long)(now_ms / 1000),
                                 health_status.tx_error_count, (long)tx_delta,
                                 health_status.rx_error_count, (long)rx_delta,
                                 health_status.state, reason);
                    }

                    last_tx_err = health_status.tx_error_count;
                    last_rx_err = health_status.rx_error_count;
                }
                last_health_check_ms = now_ms;
            }
        }
#endif // CAN_LOGLEVEL >= 2

        // Update statistics and handle errors
        iface->update_tx_stats(result == ESP_OK);

        if (result != ESP_OK) {
            if (result == ESP_ERR_TIMEOUT) {
                static uint32_t timeout_count = 0;
                timeout_count++;
                // TX timeout stats tracked via update_tx_stats
            } else if (result == ESP_FAIL) {
                // TX failed - check bus state
                twai_node_status_t twai_status;
                twai_node_record_t twai_record;
                if (twai_node_get_info(iface->_node, &twai_status, &twai_record) == ESP_OK) {
                    static uint32_t last_tx_err = 0;
                    static uint32_t last_rx_err = 0;
                    static uint32_t last_fail_log_ms = 0;
                    uint32_t now_ms = AP_HAL::millis();

                    uint32_t tx_delta = (twai_status.tx_error_count > last_tx_err) ?
                                       (twai_status.tx_error_count - last_tx_err) : 0;
                    uint32_t rx_delta = (twai_status.rx_error_count > last_rx_err) ?
                                       (twai_status.rx_error_count - last_rx_err) : 0;

                    if (twai_status.state == TWAI_ERROR_BUS_OFF) {
                        ESP_LOGE("CAN", "TX failed led to BUS_OFF: TX_ERR=%u (+%lu), RX_ERR=%u (+%lu)",
                                 twai_status.tx_error_count, (unsigned long)tx_delta,
                                 twai_status.rx_error_count, (unsigned long)rx_delta);
                        iface->log_bus_state(BusState::BUS_OFF);
                        iface->update_error_stats(0x01);

                        ESP_LOGI("CAN", "Initiating recovery from BUS_OFF");
                        twai_node_recover(iface->_node);
                    } else if (now_ms - last_fail_log_ms > 1000) {
                        ESP_LOGW("CAN", "TX failures: TX_ERR=%u (+%lu), RX_ERR=%u (+%lu), state=%d",
                                 twai_status.tx_error_count, (unsigned long)tx_delta,
                                 twai_status.rx_error_count, (unsigned long)rx_delta,
                                 twai_status.state);
                        last_fail_log_ms = now_ms;
                    }

                    last_tx_err = twai_status.tx_error_count;
                    last_rx_err = twai_status.rx_error_count;
                }
                static uint32_t fail_count = 0;
                fail_count++;
                if (fail_count <= 5 || (fail_count % 100) == 0) {
                    CAN_DEBUG_ERROR("Failed to transmit ID=0x%08X, error=%d (count=%lu)",
                                    (unsigned)(tx_item.frame.id & AP_HAL::CANFrame::MaskExtID),
                                    result, (unsigned long)fail_count);
                }
            } else if (result == ESP_ERR_INVALID_STATE) {
                // TWAI node in invalid state
                static uint32_t last_invalid_state_log_ms = 0;
                static uint32_t invalid_state_count = 0;
                static uint32_t last_recovery_attempt_ms = 0;
                static uint32_t recovery_backoff_ms = 100;
                static uint32_t consecutive_failures = 0;
                uint32_t now_ms = AP_HAL::millis();
                invalid_state_count++;

                // Check actual state
                twai_node_status_t diag_status;
                twai_node_record_t diag_record;
                bool got_status = (twai_node_get_info(iface->_node, &diag_status, &diag_record) == ESP_OK);

                if (got_status) {
                    BusState current_state = iface->error_state_to_bus_state(diag_status.state);
                    iface->log_bus_state(current_state);

                    if (invalid_state_count == 1 || (invalid_state_count % 100) == 0) {
                        CAN_DEBUG_VERBOSE("TX state diagnostics: error_state=%d, tx_err=%u, rx_err=%u",
                                       diag_status.state,
                                       (unsigned)diag_status.tx_error_count,
                                       (unsigned)diag_status.rx_error_count);
                    }
                }

                bool should_attempt_recovery = false;
                if (now_ms - last_recovery_attempt_ms >= recovery_backoff_ms) {
                    should_attempt_recovery = true;
                    last_recovery_attempt_ms = now_ms;
                    consecutive_failures++;
                    recovery_backoff_ms = MIN(recovery_backoff_ms * 2, 5000);
                }

                if (should_attempt_recovery) {
                    iface->update_error_stats(0x08);

                    if (consecutive_failures > 10) {
                        CAN_DEBUG_ERROR("TWAI recovery failed %lu times, attempting full restart",
                                       (unsigned long)consecutive_failures);
                        iface->attempt_driver_restart();
                        consecutive_failures = 0;
                        recovery_backoff_ms = 100;
                    } else if (got_status) {
                        if (diag_status.state == TWAI_ERROR_BUS_OFF) {
                            CAN_DEBUG_INFO("TWAI is BUS_OFF, initiating recovery (attempt %lu)",
                                          (unsigned long)consecutive_failures);
                            twai_node_recover(iface->_node);
                        } else if (diag_status.state == TWAI_ERROR_ACTIVE) {
                            CAN_DEBUG_INFO("TWAI recovered to ERROR_ACTIVE state");
                            consecutive_failures = 0;
                            recovery_backoff_ms = 100;
                        }
                    } else {
                        CAN_DEBUG_ERROR("Can't get TWAI status, attempting full restart");
                        iface->attempt_driver_restart();
                        consecutive_failures = 0;
                        recovery_backoff_ms = 100;
                    }
                    invalid_state_count = 0;
                }

                if (now_ms - last_invalid_state_log_ms > 10000) {
                    CAN_DEBUG_ERROR("TWAI invalid state - count=%lu",
                                   (unsigned long)invalid_state_count);
                    last_invalid_state_log_ms = now_ms;
                }

                continue;
            } else if (result == ESP_ERR_INVALID_ARG) {
                static uint32_t last_invalid_arg_log_ms = 0;
                static uint32_t invalid_arg_count = 0;
                uint32_t now_ms = AP_HAL::millis();
                invalid_arg_count++;

                if (now_ms - last_invalid_arg_log_ms > 5000) {
                    CAN_DEBUG_ERROR("TWAI invalid argument errors: %lu (last ID=0x%08X)",
                                   (unsigned long)invalid_arg_count, (unsigned)tx_item.frame.id);
                    last_invalid_arg_log_ms = now_ms;
                    invalid_arg_count = 0;
                }
                iface->update_error_stats(0x09);
            } else {
                iface->update_error_stats(result);
                CAN_DEBUG_ERROR("Error %d on ID=0x%08X", result,
                                (unsigned)(tx_item.frame.id & AP_HAL::CANFrame::MaskExtID));
            }
        }
    }
}

// --- Hardware Filter Configuration ---

bool CANIface::configure_hw_filters(const CanFilterConfig* filter_configs, uint16_t num_configs)
{
    if (!initialized || _node == nullptr) {
        return true;
    }

    // Calculate combined filter from all requested configs
    uint32_t combined_id = 0;
    uint32_t combined_mask = 0;

    if (num_configs > 0) {
        if (num_configs == 1) {
            combined_id = filter_configs[0].id & 0x1FFFFFFF;
            combined_mask = filter_configs[0].mask & 0x1FFFFFFF;
        } else {
            // Multiple filters - find common mask bits
            uint32_t common_mask_bits = 0x1FFFFFFF;
            for (uint16_t i = 0; i < num_configs; i++) {
                common_mask_bits &= filter_configs[i].mask;
            }
            combined_id = 0;
            combined_mask = common_mask_bits & 0x1FFFFFFF;
        }

        hal.console->printf("CAN%d: filters=%d, id=0x%08lx, mask=0x%08lx\n",
                           instance, num_configs,
                           (unsigned long)combined_id,
                           (unsigned long)combined_mask);
    } else {
        hal.console->printf("CAN%d: no filters - accepting all messages\n", instance);
    }

    // Apply filter directly -- no need to stop/uninstall/reinstall
    twai_mask_filter_config_t filter_cfg = {};
    filter_cfg.id = combined_id;
    filter_cfg.mask = combined_mask;
    filter_cfg.is_ext = 0;
    filter_cfg.no_classic = 0;
    filter_cfg.no_fd = 1;

    esp_err_t err = twai_node_config_mask_filter(_node, 0, &filter_cfg);
    if (err != ESP_OK) {
        CAN_DEBUG_ERROR("Failed to configure TWAI filter: %d", err);
        return false;
    }

    CAN_DEBUG_INFO("Hardware filters updated - id=0x%08X mask=0x%08X",
                       (unsigned)combined_id, (unsigned)combined_mask);

    return true;
}

// --- Statistics Collection ---

void CANIface::collect_hw_stats()
{
    if (!initialized || _node == nullptr) {
        stats.current_health = 2; // ERROR - not initialized
        return;
    }

    // Use hw_error_state from ISR on_state_change callback for state
    BusState current_state = error_state_to_bus_state(hw_error_state);

    // Get error counters and record from hardware (still needed for stats)
    twai_node_status_t node_status;
    twai_node_record_t node_record;
    if (twai_node_get_info(_node, &node_status, &node_record) == ESP_OK) {
        stats.bus_errors = node_record.bus_err_num;
    }

    switch (current_state) {
        case BusState::BUS_OFF:
            stats.bus_off_count++;
            stats.current_health = 2; // ERROR
            stats.last_error_code = 0x01;
            stats.last_error_time_us = AP_HAL::micros64();
            break;
        case BusState::WARNING:
            stats.current_health = 1; // WARNING
            break;
        case BusState::GOOD:
            stats.current_health = calculate_bus_health();
            break;
    }
    log_bus_state(current_state);
}

// --- Status Update ---

void CANIface::update_status()
{
    if (!initialized) {
        return;
    }

    check_bus_health();
    send_uavcan_node_status();
}

// --- Bus Health Check ---

void CANIface::check_bus_health()
{
    if (!initialized || _node == nullptr) {
        return;
    }

    // Use hw_error_state from ISR on_state_change callback (no polling needed)
    BusState current_state = error_state_to_bus_state(hw_error_state);
    log_bus_state(current_state);

    static uint32_t last_recovery_attempt_ms = 0;
    static uint32_t consecutive_bus_off_count = 0;
    static uint32_t last_full_restart_ms = 0;
    uint32_t now_ms = AP_HAL::millis();

    if (current_state == BusState::BUS_OFF) {
        update_error_stats(0x01);

        if (now_ms - last_recovery_attempt_ms > 5000) {
            consecutive_bus_off_count++;

            // Get error counters for diagnostics (only when we need to log)
            twai_node_status_t node_status;
            twai_node_record_t node_record;
            if (twai_node_get_info(_node, &node_status, &node_record) == ESP_OK) {
                ESP_LOGI("CAN", "Attempting bus recovery (attempt #%u): TX_ERR=%u, RX_ERR=%u",
                        (unsigned)consecutive_bus_off_count,
                        node_status.tx_error_count, node_status.rx_error_count);
            } else {
                ESP_LOGI("CAN", "Attempting bus recovery (attempt #%u)",
                        (unsigned)consecutive_bus_off_count);
            }

            if (consecutive_bus_off_count > 5 && (now_ms - last_full_restart_ms > 30000)) {
                CAN_DEBUG_ERROR("Persistent bus-off condition, attempting full TWAI restart");

                uint32_t saved_bitrate = current_bitrate;

                twai_node_disable(_node);
                twai_node_delete(_node);
                _node = nullptr;

                hal.scheduler->delay_microseconds(1000);

                initialized = false;
                if (init(saved_bitrate)) {
                    CAN_DEBUG_INFO("Full TWAI restart successful");
                    consecutive_bus_off_count = 0;
                    last_full_restart_ms = now_ms;
                } else {
                    CAN_DEBUG_ERROR("Full TWAI restart failed");
                }
            } else {
                twai_node_recover(_node);
            }

            last_recovery_attempt_ms = now_ms;
        }
    } else {
        consecutive_bus_off_count = 0;
    }

    // Check for excessive TX queue backlog
    if (tx_queue && uxQueueMessagesWaiting(tx_queue) > 100) {
        CAN_DEBUG_WARN("TX queue backlog detected (%u messages)",
                           (unsigned)uxQueueMessagesWaiting(tx_queue));
        update_error_stats(0x10);
    }
}

// --- Bus State Logging ---

void CANIface::log_bus_state(BusState new_state)
{
    uint32_t now_ms = AP_HAL::millis();

    if (new_state != last_bus_state) {
        if (new_state == BusState::GOOD) {
            if (bus_state_change_ms > 0) {
                float downtime_sec = (now_ms - bus_state_change_ms) / 1000.0f;
                ESP_LOGE("CAN", "Bus recovered to GOOD after %.2f seconds", downtime_sec);
            } else {
                ESP_LOGI("CAN", "Bus initialized to GOOD state");
            }
        } else if (new_state == BusState::BUS_OFF) {
            twai_node_status_t status;
            twai_node_record_t record;
            if (_node && twai_node_get_info(_node, &status, &record) == ESP_OK) {
                ESP_LOGE("CAN", "Bus entered BUS_OFF: TX_ERR=%u, RX_ERR=%u",
                         status.tx_error_count, status.rx_error_count);
            } else {
                ESP_LOGE("CAN", "Bus entered BUS_OFF state");
            }
        } else if (new_state == BusState::WARNING) {
            twai_node_status_t status;
            twai_node_record_t record;
            if (_node && twai_node_get_info(_node, &status, &record) == ESP_OK) {
                ESP_LOGW("CAN", "Bus degraded to WARNING: TX_ERR=%u, RX_ERR=%u, error_state=%d",
                         status.tx_error_count, status.rx_error_count, status.state);
            } else {
                ESP_LOGW("CAN", "Bus degraded to WARNING state");
            }
        }

        last_bus_state = new_state;
        bus_state_change_ms = now_ms;
        last_bus_error_log_ms = now_ms;
    } else if (new_state != BusState::GOOD) {
        if (now_ms - last_bus_error_log_ms >= 30000) {
            float downtime_sec = (now_ms - bus_state_change_ms) / 1000.0f;
            const char* state_str = "UNKNOWN";
            switch (new_state) {
                case BusState::BUS_OFF:     state_str = "BUS_OFF"; break;
                case BusState::WARNING:     state_str = "WARNING"; break;
                default: break;
            }
            ESP_LOGE("CAN", "Bus still %s - down for %.2f seconds", state_str, downtime_sec);
            last_bus_error_log_ms = now_ms;
        }
    }
}

// --- Driver Restart ---

void CANIface::attempt_driver_restart()
{
    static uint32_t last_restart_ms = 0;
    static uint32_t restart_count = 0;
    uint32_t now_ms = AP_HAL::millis();

    if (now_ms - last_restart_ms < 5000) {
        static uint32_t last_log_ms = 0;
        if (now_ms - last_log_ms > 2000) {
            CAN_DEBUG_INFO("Driver restart rate-limited (last restart %lu ms ago)",
                          (unsigned long)(now_ms - last_restart_ms));
            last_log_ms = now_ms;
        }
        return;
    }

    if (restart_in_progress) {
        CAN_DEBUG_INFO("Driver restart already in progress, skipping");
        return;
    }

    restart_count++;
    last_restart_ms = now_ms;

    if (restart_count <= 3 || (restart_count % 10) == 0) {
        CAN_DEBUG_ERROR("TWAI driver restart #%lu (invalid state)", (unsigned long)restart_count);
    }

    uint32_t saved_bitrate = current_bitrate;

    // Signal RX/TX tasks to pause
    restart_in_progress = true;

    // Wait for tasks to see the flag
    hal.scheduler->delay(200);

    // Disable and delete the TWAI node
    if (_node != nullptr) {
        twai_node_disable(_node);
        hal.scheduler->delay(50);
        twai_node_delete(_node);
        _node = nullptr;
    }

    hal.scheduler->delay(10);

    // Clear queues
    if (rx_queue) {
        xQueueReset(rx_queue);
    }
    if (tx_queue) {
        xQueueReset(tx_queue);
    }
    if (isr_rx_queue) {
        xQueueReset(isr_rx_queue);
    }

    // Reinitialize
    initialized = false;
    bool reinit_ok = init(saved_bitrate);

    if (reinit_ok) {
        hal.scheduler->delay(50);
        restart_in_progress = false;
        CAN_DEBUG_INFO("TWAI driver restart successful after %lu restarts", (unsigned long)restart_count);
        update_error_stats(0x00);
    } else {
        restart_in_progress = false;
        CAN_DEBUG_ERROR("TWAI driver restart failed - interface may be unusable");
    }
}

#endif // HAL_NUM_CAN_IFACES > 0
