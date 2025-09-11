void CANIface::tx_task(void *arg)
{
    CANIface *iface = (CANIface *)arg;
    CanTxItem tx_item;
    CAN_DEBUG_INFO("CAN TX task started for interface %d", iface->instance);

    while (true) {
        // Process multiple frames per iteration to keep up with high frame rates
        uint32_t frames_processed = 0;
        const uint32_t MAX_FRAMES_PER_ITERATION = 10;
        
        while (frames_processed < MAX_FRAMES_PER_ITERATION) {
            // Try to get a frame with short timeout
            if (xQueueReceive(iface->tx_queue, &tx_item, pdMS_TO_TICKS(1)) != pdPASS) {
                // No frames available
                if (frames_processed == 0) {
                    // Only wait longer if we haven't processed any frames
                    vTaskDelay(pdMS_TO_TICKS(5));
                }
                break; // Exit inner loop to yield
            }
            
            frames_processed++;
            
            // Check for expired frame
            uint64_t now_us = AP_HAL::micros64();
            if (tx_item.deadline_us != 0 && now_us > tx_item.deadline_us) {
                ESP_LOGW("CAN", "Dropped expired frame ID=0x%08X", (unsigned)tx_item.frame.id);
                continue; // Skip expired frame and get next
            }

            // Prepare TWAI message
            twai_message_t message;
            memset(&message, 0, sizeof(message));
            
            message.identifier = tx_item.frame.id & AP_HAL::CANFrame::MaskExtID;
            message.data_length_code = tx_item.frame.dlc;
            message.extd = 1;  // DroneCAN requires extended frames
            message.rtr = 0;
            message.ss = 0;    // Allow retransmission
            message.self = 0;
            memcpy(message.data, tx_item.frame.data, tx_item.frame.dlc);

            // Check TWAI status before transmission
            twai_status_info_t status;
            if (twai_get_status_info(&status) == ESP_OK) {
                if (status.state == TWAI_STATE_BUS_OFF) {
                    ESP_LOGE("CAN", "TWAI in BUS-OFF state! Initiating recovery");
                    twai_initiate_recovery();
                    vTaskDelay(pdMS_TO_TICKS(10));
                } else if (status.state == TWAI_STATE_STOPPED) {
                    ESP_LOGE("CAN", "TWAI is STOPPED! Attempting restart");
                    twai_start();
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                
                // Log status periodically
                static uint32_t last_status_log = 0;
                uint32_t now = AP_HAL::millis();
                if (now - last_status_log > 1000) {
                    ESP_LOGI("CAN", "TWAI Status: state=%d, msgs_to_tx=%ld, msgs_to_rx=%ld, tx_err=%ld, rx_err=%ld",
                            status.state, status.msgs_to_tx, status.msgs_to_rx, 
                            status.tx_error_counter, status.rx_error_counter);
                    last_status_log = now;
                }
            }
            
            // Check for TWAI alerts
            uint32_t alerts;
            if (twai_read_alerts(&alerts, 0) == ESP_OK && alerts != 0) {
                // Only log non-routine alerts
                if (alerts != 0x00000003) { // 0x03 is just TX_SUCCESS + RX_DATA
                    ESP_LOGW("CAN", "TWAI alerts detected: 0x%08X", alerts);
                }
                
                if (alerts & TWAI_ALERT_BUS_OFF) {
                    ESP_LOGE("CAN", "BUS OFF alert! Attempting recovery");
                    twai_initiate_recovery();
                }
                if (alerts & TWAI_ALERT_ERR_PASS) {
                    ESP_LOGW("CAN", "Error passive alert");
                }
                if (alerts & TWAI_ALERT_TX_FAILED) {
                    ESP_LOGW("CAN", "TX failed alert");
                }
                if (alerts & TWAI_ALERT_TX_SUCCESS) {
                    // Normal - frame was ACKed
                }
                if (alerts & TWAI_ALERT_ABOVE_ERR_WARN) {
                    ESP_LOGW("CAN", "Above error warning threshold");
                }
                if (alerts & TWAI_ALERT_BUS_RECOVERED) {
                    ESP_LOGI("CAN", "Bus recovered alert");
                }
                twai_reconfigure_alerts(TWAI_ALERT_ALL, NULL);
            }
            
            // Transmit the frame
            ESP_LOGI("CAN", "Calling twai_transmit for ID=0x%08X, DLC=%d, extd=%d, timeout=%d ticks", 
                     (unsigned)message.identifier, message.data_length_code, message.extd, (int)pdMS_TO_TICKS(10));
            
            esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(10));
            
            ESP_LOGI("CAN", "twai_transmit returned: %d (%s) for ID=0x%08X", 
                     result, esp_err_to_name(result), (unsigned)message.identifier);
            
            // Check post-TX status
            if (result == ESP_OK) {
                twai_status_info_t post_tx_status;
                if (twai_get_status_info(&post_tx_status) == ESP_OK) {
                    ESP_LOGI("CAN", "Post-TX status: msgs_to_tx=%ld, tx_err=%ld, rx_miss=%ld, state=%d", 
                             post_tx_status.msgs_to_tx, post_tx_status.tx_error_counter, 
                             post_tx_status.rx_missed_count, post_tx_status.state);
                }
                
                // Track transmitted frame for self-reception filtering
                iface->tx_tracker[iface->tx_tracker_index].can_id = message.identifier;
                iface->tx_tracker[iface->tx_tracker_index].timestamp_us = AP_HAL::micros64();
                iface->tx_tracker[iface->tx_tracker_index].loopback_requested = (tx_item.flags & AP_HAL::CANIface::Loopback) != 0;
                iface->tx_tracker_index = (iface->tx_tracker_index + 1) % TX_TRACKER_SIZE;
            }
            
            // Update statistics
            iface->update_tx_stats(result == ESP_OK);
            
            if (result != ESP_OK) {
                // Handle specific TWAI error conditions
                if (result == ESP_ERR_TIMEOUT) {
                    ESP_LOGW("CAN", "TX timeout - bus may be congested");
                } else if (result == ESP_FAIL) {
                    ESP_LOGE("CAN", "TX failed - check bus state");
                    twai_status_info_t err_status;
                    if (twai_get_status_info(&err_status) == ESP_OK) {
                        ESP_LOGE("CAN", "Error state: state=%d, tx_err=%ld, rx_err=%ld",
                                err_status.state, err_status.tx_error_counter, err_status.rx_error_counter);
                    }
                } else if (result == ESP_ERR_INVALID_STATE) {
                    ESP_LOGE("CAN", "TWAI in invalid state for TX");
                } else if (result == ESP_ERR_NOT_SUPPORTED) {
                    ESP_LOGE("CAN", "TX mode not supported");
                }
                
                iface->update_error_stats(result);
            }
        } // End of inner frame processing loop
    } // End of outer while(true) loop
}