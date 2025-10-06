#include "AP_Canard_iface.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANManager.h>
#if HAL_ENABLE_DRONECAN_DRIVERS
#include <canard/handler_list.h>
#include <canard/transfer_object.h>
#include <AP_Math/AP_Math.h>
#include <dronecan_msgs.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
#include "esp_log.h"
#include <stdio.h>
#include <string.h>
// Include for manual LogMessage decode check
extern "C" {
#include <uavcan.protocol.debug.LogMessage.h>
}
#endif
extern const AP_HAL::HAL& hal;
#define LOG_TAG "DroneCANIface"
#include <canard.h>
#include <AP_CANManager/AP_CANSensor.h>

#define DEBUG_PKTS 0

#define CANARD_MSG_TYPE_FROM_ID(x)                         ((uint16_t)(((x) >> 8U)  & 0xFFFFU))

DEFINE_HANDLER_LIST_HEADS();
DEFINE_HANDLER_LIST_SEMAPHORES();

DEFINE_TRANSFER_OBJECT_HEADS();
DEFINE_TRANSFER_OBJECT_SEMAPHORES();

#if AP_TEST_DRONECAN_DRIVERS
CanardInterface* CanardInterface::canard_ifaces[] = {nullptr, nullptr, nullptr};
CanardInterface CanardInterface::test_iface{2};
uint8_t test_node_mem_area[1024];
HAL_Semaphore test_iface_sem;
#endif

void canard_allocate_sem_take(CanardPoolAllocator *allocator) {
    if (allocator->semaphore == nullptr) {
        allocator->semaphore = NEW_NOTHROW HAL_Semaphore;
        if (allocator->semaphore == nullptr) {
            // out of memory
            CANARD_ASSERT(0);
            return;
        }
    }
    ((HAL_Semaphore*)allocator->semaphore)->take_blocking();
}

void canard_allocate_sem_give(CanardPoolAllocator *allocator) {
    if (allocator->semaphore == nullptr) {
        // it should have been allocated by canard_allocate_sem_take
        CANARD_ASSERT(0);
        return;
    }
    ((HAL_Semaphore*)allocator->semaphore)->give();
}

CanardInterface::CanardInterface(uint8_t iface_index) :
Interface(iface_index) {
#if AP_TEST_DRONECAN_DRIVERS
    if (iface_index < 3) {
        canard_ifaces[iface_index] = this;
    }
    if (iface_index == 0) {
        test_iface.init(test_node_mem_area, sizeof(test_node_mem_area), 125);
    }
    canardInitTxTransfer(&tx_transfer);
#endif
}

void CanardInterface::init(void* mem_arena, size_t mem_arena_size, uint8_t node_id) {
    canardInit(&canard, mem_arena, mem_arena_size, onTransferReception, shouldAcceptTransfer, this);
    canardSetLocalNodeID(&canard, node_id);
    initialized = true;
    
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    ESP_LOGI("CAN_RX", "CanardInterface initialized with node_id=%d - CAN_RX logging is ENABLED", node_id);
    hal.console->printf("ESP32: CanardInterface init - CAN_RX logging enabled (you should see ESP_LOGE above)\n");
#endif
}

bool CanardInterface::broadcast(const Canard::Transfer &bcast_transfer) {
    if (!initialized) {
        return false;
    }
    WITH_SEMAPHORE(_sem_tx);

#if AP_TEST_DRONECAN_DRIVERS
    if (this == &test_iface) {
        test_iface_sem.take_blocking();
    }
#endif

    // Validate and fix priority if corrupted
    uint8_t safe_priority = bcast_transfer.priority;
    if (safe_priority > 31) {
        ESP_LOGE("CANARD", "INVALID PRIORITY %u (0x%02X) for DataTypeID %u - clamping to 31",
                 (unsigned)safe_priority, (unsigned)safe_priority, 
                 (unsigned)bcast_transfer.data_type_id);
        
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
        // Enhanced diagnostic to find corruption source
        ESP_LOGE("CANARD", "broadcast() corruption detected - Transfer at %p:", &bcast_transfer);
        ESP_LOGE("CANARD", "  transfer_type=%u, signature=0x%llX", 
                 (unsigned)bcast_transfer.transfer_type, 
                 (unsigned long long)bcast_transfer.data_type_signature);
        ESP_LOGE("CANARD", "  data_type_id=%u (0x%04X), priority=%u (0x%02X)", 
                 (unsigned)bcast_transfer.data_type_id, (unsigned)bcast_transfer.data_type_id,
                 (unsigned)bcast_transfer.priority, (unsigned)bcast_transfer.priority);
        ESP_LOGE("CANARD", "  payload=%p, payload_len=%lu, iface_mask=0x%02X",
                 bcast_transfer.payload, (unsigned long)bcast_transfer.payload_len,
                 (unsigned)bcast_transfer.iface_mask);
        
        // Dump raw memory to check for corruption pattern
        const uint8_t* raw = (const uint8_t*)&bcast_transfer;
        ESP_LOGE("CANARD", "  Raw Transfer bytes [0-31]:");
        ESP_LOGE("CANARD", "    %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                 raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], raw[6], raw[7],
                 raw[8], raw[9], raw[10], raw[11], raw[12], raw[13], raw[14], raw[15]);
        ESP_LOGE("CANARD", "    %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                 raw[16], raw[17], raw[18], raw[19], raw[20], raw[21], raw[22], raw[23],
                 raw[24], raw[25], raw[26], raw[27], raw[28], raw[29], raw[30], raw[31]);
        
        // Check if it's a specific corruption pattern
        if ((bcast_transfer.priority & 0x80) != 0) {
            ESP_LOGE("CANARD", "Priority has bit 7 set - likely struct alignment/packing issue!");
        }
#endif
        
        safe_priority = 31;  // Clamp to maximum valid priority
    }
    
    // CRITICAL: Zero-initialize the entire structure first to avoid garbage values
    memset(&tx_transfer, 0, sizeof(tx_transfer));
    
    tx_transfer = {
        .transfer_type = bcast_transfer.transfer_type,
        .data_type_signature = bcast_transfer.data_type_signature,
        .data_type_id = bcast_transfer.data_type_id,
        .inout_transfer_id = bcast_transfer.inout_transfer_id,
        .priority = safe_priority,
        .payload = (const uint8_t*)bcast_transfer.payload,
        .payload_len = uint16_t(bcast_transfer.payload_len),
#if CANARD_ENABLE_CANFD
        .canfd = bcast_transfer.canfd,
#endif
        .deadline_usec = AP_HAL::micros64() + (bcast_transfer.timeout_ms * 1000),
#if CANARD_MULTI_IFACE
        .iface_mask = uint8_t((1<<num_ifaces) - 1),
#endif
    };
    
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    // Debug deadline calculation
    uint64_t now = AP_HAL::micros64();
    uint64_t deadline = now + (bcast_transfer.timeout_ms * 1000);
    
    // Log problematic deadlines
    if (deadline < now || bcast_transfer.timeout_ms > 10000) { // Past deadline or suspiciously large timeout
        ESP_LOGW("CANARD", "Bad deadline! DataTypeID=%u, now=%llu, timeout_ms=%lu, deadline=%llu (diff=%lld)",
                 bcast_transfer.data_type_id, now, (unsigned long)bcast_transfer.timeout_ms, deadline,
                 (long long)(deadline - now));
    }
    
    // Debug logging removed to reduce spam
    // ESP_LOGD("CANARD", "Allocation deadline: now=%llu, deadline=%llu", now, deadline);
#endif
    // Debug what DroneCAN thinks it's sending (only at verbose debug level)
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32 && 0  // Disabled - too verbose, enable only for debugging broadcast issues
    hal.console->printf("DRONECAN: Broadcasting: DataTypeID=%u, Priority=%u, PayloadLen=%u, TransferID=%u\n", 
                        (unsigned)bcast_transfer.data_type_id, (unsigned)bcast_transfer.priority, 
                        (unsigned)bcast_transfer.payload_len, (unsigned)*bcast_transfer.inout_transfer_id);
#endif
    
    // do canard broadcast
    int16_t ret = canardBroadcastObj(&canard, &tx_transfer);
#if AP_TEST_DRONECAN_DRIVERS
    if (this == &test_iface) {
        test_iface_sem.give();
    }
#endif
    if (ret <= 0) {
        protocol_stats.tx_errors++;
    } else {
        protocol_stats.tx_frames += ret;
    }
    return ret > 0;
}

bool CanardInterface::request(uint8_t destination_node_id, const Canard::Transfer &req_transfer) {
    if (!initialized) {
        return false;
    }
    WITH_SEMAPHORE(_sem_tx);

    // Validate and fix priority if corrupted
    uint8_t safe_priority = req_transfer.priority;
    if (safe_priority > 31) {
        ESP_LOGE("CANARD", "INVALID REQUEST PRIORITY %u (0x%02X) for DataTypeID %u - clamping to 31",
                 (unsigned)safe_priority, (unsigned)safe_priority, 
                 (unsigned)req_transfer.data_type_id);
        safe_priority = 31;  // Clamp to maximum valid priority
    }
    
    // CRITICAL: Zero-initialize the entire structure first to avoid garbage values
    memset(&tx_transfer, 0, sizeof(tx_transfer));
    
    tx_transfer = {
        .transfer_type = req_transfer.transfer_type,
        .data_type_signature = req_transfer.data_type_signature,
        .data_type_id = req_transfer.data_type_id,
        .inout_transfer_id = req_transfer.inout_transfer_id,
        .priority = safe_priority,
        .payload = (const uint8_t*)req_transfer.payload,
        .payload_len = uint16_t(req_transfer.payload_len),
#if CANARD_ENABLE_CANFD
        .canfd = req_transfer.canfd,
#endif
        .deadline_usec = AP_HAL::micros64() + (req_transfer.timeout_ms * 1000),
#if CANARD_MULTI_IFACE
        .iface_mask = uint8_t((1<<num_ifaces) - 1),
#endif
    };
    // Debug what DroneCAN thinks it's requesting (only at verbose debug level)
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32 && 0  // Disabled - too verbose, enable only for debugging service issues
    hal.console->printf("DRONECAN: Service Request: DataTypeID=%u, Priority=%u, TransferType=%u, Dest=%u, Source=%u\n", 
                        (unsigned)req_transfer.data_type_id, (unsigned)req_transfer.priority, 
                        (unsigned)req_transfer.transfer_type, destination_node_id, canard.node_id);
#endif
    
    // do canard request
    int16_t ret = canardRequestOrRespondObj(&canard, destination_node_id, &tx_transfer);
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    if (ret > 0) {
        ESP_LOGD("CAN_RX", "GetNodeInfo request queued successfully: %d frames to node %d", 
                 ret, destination_node_id);
    } else {
        ESP_LOGE("CAN_RX", "GetNodeInfo request FAILED: error %d to node %d", 
                 ret, destination_node_id);
    }
#endif
    if (ret <= 0) {
        protocol_stats.tx_errors++;
    } else {
        protocol_stats.tx_frames += ret;
    }
    return ret > 0;
}

bool CanardInterface::respond(uint8_t destination_node_id, const Canard::Transfer &res_transfer) {
    if (!initialized) {
        return false;
    }
    WITH_SEMAPHORE(_sem_tx);

    // Validate and fix priority if corrupted
    uint8_t safe_priority = res_transfer.priority;
    if (safe_priority > 31) {
        ESP_LOGE("CANARD", "INVALID RESPONSE PRIORITY %u (0x%02X) for DataTypeID %u - clamping to 31",
                 (unsigned)safe_priority, (unsigned)safe_priority, 
                 (unsigned)res_transfer.data_type_id);
        safe_priority = 31;  // Clamp to maximum valid priority
    }
    
    // CRITICAL: Zero-initialize the entire structure first to avoid garbage values
    memset(&tx_transfer, 0, sizeof(tx_transfer));
    
    tx_transfer = {
        .transfer_type = res_transfer.transfer_type,
        .data_type_signature = res_transfer.data_type_signature,
        .data_type_id = res_transfer.data_type_id,
        .inout_transfer_id = res_transfer.inout_transfer_id,
        .priority = safe_priority,
        .payload = (const uint8_t*)res_transfer.payload,
        .payload_len = uint16_t(res_transfer.payload_len),
#if CANARD_ENABLE_CANFD
        .canfd = res_transfer.canfd,
#endif
        .deadline_usec = AP_HAL::micros64() + (res_transfer.timeout_ms * 1000),
#if CANARD_MULTI_IFACE
        .iface_mask = uint8_t((1<<num_ifaces) - 1),
#endif
    };
    // do canard respond
    int16_t ret = canardRequestOrRespondObj(&canard, destination_node_id, &tx_transfer);
    if (ret <= 0) {
        protocol_stats.tx_errors++;
    } else {
        protocol_stats.tx_frames += ret;
    }
    return ret > 0;
}

void CanardInterface::onTransferReception(CanardInstance* ins, CanardRxTransfer* transfer) {
    CanardInterface* iface = (CanardInterface*) ins->user_reference;
    
    // Debug for allocation messages (only for broadcast messages with ID 1, not service responses)
    if (transfer->data_type_id == 1 && transfer->transfer_type == CanardTransferTypeBroadcast) {
        hal.console->printf("DNA: onTransferReception got Allocation message from node %d\n", 
                           transfer->source_node_id);
    }
    
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    // Debug GetNodeInfo responses (changed to INFO level for better visibility)
    if (transfer->transfer_type == CanardTransferTypeResponse && transfer->data_type_id == 1) {
        ESP_LOGI("CAN_RX", "GetNodeInfo RESPONSE received in onTransferReception from node %d! Payload len=%d",
                 transfer->source_node_id, transfer->payload_len);
        ESP_LOGI("CAN_RX", "  About to dispatch to handlers...");
    }
#endif
    
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    // Track all DroneCAN message reception
    static uint32_t dronecan_msg_count = 0;
    static uint32_t logmsg_count = 0;
    static uint32_t last_report_ms = 0;
    static bool first_can_rx = true;
    
    dronecan_msg_count++;
    
    // Verify ESP-IDF logging is working on first message
    if (first_can_rx) {
        ESP_LOGE("CAN_RX", "CAN_RX logging test - you should see this ERROR level message!");
        hal.console->printf("ESP32: If you see this printf but not the ESP_LOGE above, ESP-IDF CAN_RX logging is broken\n");
        first_can_rx = false;
    }
    
    // Log LogMessage transfers at DEBUG level (only visible when debugging)
    if (transfer->data_type_id == 16383) { // uavcan.protocol.debug.LogMessage
        logmsg_count++;

        // Special logging for Node 2 LogMessages (phantom node)
        if (transfer->source_node_id == 2) {
            ESP_LOGW("CAN_RX", "=== LogMessage from PHANTOM NODE 2 ===");
            ESP_LOGW("CAN_RX", "This should be from your real node but has corrupted source ID!");
        }

        ESP_LOGD("CAN_RX", "LogMessage RX: src_node=%d, len=%d",
                 transfer->source_node_id, transfer->payload_len);
        
        // Only log payload in verbose debug mode
        #ifdef DEBUG_BUILD
        if (transfer->payload_len > 0) {
            char hex_buf[97] = {0}; // 32 bytes * 3 chars per byte + null
            int bytes_to_log = transfer->payload_len > 32 ? 32 : transfer->payload_len;
            for (int i = 0; i < bytes_to_log; i++) {
                snprintf(&hex_buf[i*3], 4, "%02X ", ((uint8_t*)transfer->payload_head)[i]);
            }
            ESP_LOGD("CAN_RX", "  Payload preview: %s%s", hex_buf, 
                     transfer->payload_len > 32 ? "..." : "");
        }
        #endif
        
        // Try to decode it manually to check for errors
        struct uavcan_protocol_debug_LogMessage test_msg;
        bool decode_result = uavcan_protocol_debug_LogMessage_decode(transfer, &test_msg);
        if (!decode_result) {  // Fixed logic - decode_result is false on failure
            ESP_LOGE("CAN_RX", "ERROR: Failed to decode LogMessage from node %d! Payload len=%d",
                     transfer->source_node_id, transfer->payload_len);
            // Log full payload for analysis
            ESP_LOGE("CAN_RX", "  Full payload dump:");
            for (int i = 0; i < transfer->payload_len; i += 16) {
                char line_buf[49] = {0}; // 16 bytes * 3 chars + null
                int bytes_in_line = (transfer->payload_len - i) > 16 ? 16 : (transfer->payload_len - i);
                for (int j = 0; j < bytes_in_line; j++) {
                    snprintf(&line_buf[j*3], 4, "%02X ", ((uint8_t*)transfer->payload_head)[i+j]);
                }
                ESP_LOGE("CAN_RX", "    [%03d-%03d]: %s", i, i+bytes_in_line-1, line_buf);
            }
        } else {
            ESP_LOGW("CAN_RX", "  LogMessage decoded OK: level=%d, text='%.40s%s'", 
                     test_msg.level.value, test_msg.text.data,
                     strlen((char*)test_msg.text.data) > 40 ? "..." : "");
        }
    }
    
    // Report statistics every 5 seconds (use ESP_LOGD for debug-level logging only)
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_report_ms > 5000) {
        ESP_LOGD("CAN_RX", "DroneCAN stats: %lu total msgs, %lu LogMessages in last 5s",
                 dronecan_msg_count, logmsg_count);
        // Only print to console if we have LogMessages (indicating potential issues)
        if (logmsg_count > 0) {
            hal.console->printf("ESP32 CAN_RX: DroneCAN received %lu msgs (%lu LogMessages) in 5s\n",
                               dronecan_msg_count, logmsg_count);
        }
        dronecan_msg_count = 0;
        logmsg_count = 0;
        last_report_ms = now_ms;
    }
#endif
    
    iface->handle_message(*transfer);
}

bool CanardInterface::shouldAcceptTransfer(const CanardInstance* ins,
                                           uint64_t* out_data_type_signature,
                                           uint16_t data_type_id,
                                           CanardTransferType transfer_type,
                                           uint8_t source_node_id) {
    CanardInterface* iface = (CanardInterface*) ins->user_reference;
    
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    static uint32_t reject_count = 0;
    static uint32_t last_debug = 0;
#endif
    
    // Debug logging for all important message types (disabled - too verbose)
#if 1  // Enable only for DNA debugging
    bool is_important = (data_type_id == 1) || // Allocation/GetNodeInfo
                        (data_type_id == 341) || // NodeStatus
                        (data_type_id == 551); // Legacy GetNodeInfo

    if (is_important) {
        const char* msg_type = "UNKNOWN";
        if (data_type_id == 1) {
            if (source_node_id == 0) {
                msg_type = "DNA";  // Only actual allocation from node 0
            } else {
                msg_type = "GETNODEINFO";  // GetNodeInfo from assigned nodes
            }
        } else if (data_type_id == 341) {
            msg_type = "NODESTATUS";
        } else if (data_type_id == 551) {
            msg_type = "LEGACY_GETNODEINFO";
        }

        hal.console->printf("%s: shouldAccept dtid=%d from node %d, xfer_type=%d\n",
                           msg_type, data_type_id, source_node_id, transfer_type);
    }
#endif
    
    bool accepted = iface->accept_message(data_type_id, transfer_type, *out_data_type_signature);
    
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    if (!accepted) {
        reject_count++;
        // Log every 50th rejection
        if (reject_count % 50 == 1) {
            ESP_LOGD("CAN_RX", "shouldAccept REJECTED dtid=%d, xfer_type=%d, src=%d (total rejected=%d)",
                     data_type_id, transfer_type, source_node_id, reject_count);
        }
    } else {
        // Log all accepted messages
        ESP_LOGD("CAN_RX", "shouldAccept ACCEPTED dtid=%d, xfer_type=%d, src=%d, sig=0x%llx",
                 data_type_id, transfer_type, source_node_id, (unsigned long long)*out_data_type_signature);
        
        // Special logging for service responses
        if (transfer_type == CanardTransferTypeResponse) {
            ESP_LOGI("CAN_RX", "SERVICE RESPONSE ACCEPTED in shouldAccept! dtid=%d from node %d",
                     data_type_id, source_node_id);
            if (data_type_id == 1) {  // GetNodeInfo response
                ESP_LOGI("CAN_RX", "GetNodeInfo RESPONSE ACCEPTED from node %d, xfer_type=%d!",
                         source_node_id, transfer_type);
            }
        }
    }
    
    // Periodically log summary
    uint32_t now = AP_HAL::millis();
    if (now - last_debug > 10000) {
        ESP_LOGD("CAN_RX", "shouldAccept stats: %d messages rejected in last 10s", reject_count);
        reject_count = 0;
        last_debug = now;
    }
#endif
    
#if 0  // Also disabled - too verbose
    if (is_important) {
        hal.console->printf("DNA: dtid=%d accepted=%d, signature=%llx\n", 
                           data_type_id, accepted, (unsigned long long)*out_data_type_signature);
    }
#endif
    
    return accepted;
}

#if AP_TEST_DRONECAN_DRIVERS
void CanardInterface::processTestRx() {
    if (!test_iface.initialized) {
        return;
    }
    WITH_SEMAPHORE(test_iface_sem);
    for (const CanardCANFrame* txf = canardPeekTxQueue(&test_iface.canard); txf != NULL; txf = canardPeekTxQueue(&test_iface.canard)) {
        if (canard_ifaces[0]) {
            canardHandleRxFrame(&canard_ifaces[0]->canard, txf, AP_HAL::micros64());   
        }
        canardPopTxQueue(&test_iface.canard);
    }
}
#endif

void CanardInterface::processTx(bool raw_commands_only = false) {
    WITH_SEMAPHORE(_sem_tx);

    for (uint8_t iface = 0; iface < num_ifaces; iface++) {
        if (ifaces[iface] == NULL) {
            continue;
        }
        auto txq = canard.tx_queue;
        if (txq == nullptr) {
            return;
        }
        // volatile as the value can change at any time during can interrupt
        // we need to ensure that this is not optimized
        volatile const auto *stats = ifaces[iface]->get_statistics();
        uint64_t last_transmit_us = stats==nullptr?0:stats->last_transmit_us;
        bool iface_down = true;
        if (stats == nullptr || (AP_HAL::micros64() - last_transmit_us) < 200000UL) {
            /*
            We were not able to queue the frame for
            sending. Only mark the send as failing if the
            interface is active. We consider an interface as
            active if it has had successful transmits for some time.
            */
            iface_down = false;
        } 
        // scan through list of pending transfers
        while (true) {
            auto txf = &txq->frame;
            if (raw_commands_only &&
                CANARD_MSG_TYPE_FROM_ID(txf->id) != UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID &&
                CANARD_MSG_TYPE_FROM_ID(txf->id) != COM_HOBBYWING_ESC_RAWCOMMAND_ID) {
                // look at next transfer
                txq = txq->next;
                if (txq == nullptr) {
                    break;
                }
                continue;
            }
            AP_HAL::CANFrame txmsg {};
            txmsg.dlc = AP_HAL::CANFrame::dataLengthToDlc(txf->data_len);
            memcpy(txmsg.data, txf->data, txf->data_len);
            txmsg.id = (txf->id | AP_HAL::CANFrame::FlagEFF);
            
#if HAL_CANFD_SUPPORTED
            txmsg.canfd = txf->canfd;
#endif
            bool write = true;
            bool read = false;
            ifaces[iface]->select(read, write, &txmsg, 0);
            if (!write) {
                // if there is no space then we need to start from the
                // top of the queue, so wait for the next loop
                if (!iface_down) {
                    break;
                } else {
#if CANARD_MULTI_IFACE
                    txf->iface_mask &= ~(1U<<iface);
#endif
                }
            #if CANARD_MULTI_IFACE
            } else if ((txf->iface_mask & (1U<<iface)) && (AP_HAL::micros64() < txf->deadline_usec)) {
                // try sending to interfaces, clearing the mask if we succeed
                if (ifaces[iface]->send(txmsg, txf->deadline_usec, 0) > 0) {
                    txf->iface_mask &= ~(1U<<iface);
                } else {
                    // if we fail to send then we try sending on next interface
                    if (!iface_down) {
                        break;
                    } else {
                        txf->iface_mask &= ~(1U<<iface);
                    }
                }
            }
#else
            } else if (AP_HAL::micros64() < txf->deadline_usec) {
                // try sending to interfaces
                bool sent = (ifaces[iface]->send(txmsg, txf->deadline_usec, 0) > 0);
                if (sent) {
                    // Successfully sent - remove from queue
                    canardPopTxQueue(&canard);
                    // Restart from head of queue
                    txq = canard.tx_queue;
                    if (txq == nullptr) {
                        break;
                    }
                    continue;
                } else {
                    // Failed to send
                    if (!iface_down) {
                        break;  // Try again later
                    }
                }
            }
#endif
            // Move to next frame in queue
            txq = txq->next;
            if (txq == nullptr) {
                break;
            }
        }
    }

    // purge expired transfers
    for (const CanardCANFrame* txf = canardPeekTxQueue(&canard); txf != NULL; txf = canardPeekTxQueue(&canard)) {
#if CANARD_MULTI_IFACE
        if ((AP_HAL::micros64() >= txf->deadline_usec) || (txf->iface_mask == 0)) {
#else
        if (AP_HAL::micros64() >= txf->deadline_usec) {
#endif
            canardPopTxQueue(&canard);
        } else {
            break;
        }
    }
}

void CanardInterface::update_rx_protocol_stats(int16_t res)
{
    switch (-res) {
    case CANARD_OK:
        protocol_stats.rx_frames++;
        break;
    case CANARD_ERROR_OUT_OF_MEMORY:
        protocol_stats.rx_error_oom++;
        break;
    case CANARD_ERROR_INTERNAL:
        protocol_stats.rx_error_internal++;
        break;
    case CANARD_ERROR_RX_INCOMPATIBLE_PACKET:
        protocol_stats.rx_ignored_not_wanted++;
        break;
    case CANARD_ERROR_RX_WRONG_ADDRESS:
        protocol_stats.rx_ignored_wrong_address++;
        break;
    case CANARD_ERROR_RX_NOT_WANTED:
        protocol_stats.rx_ignored_not_wanted++;
        break;
    case CANARD_ERROR_RX_MISSED_START:
        protocol_stats.rx_error_missed_start++;
        break;
    case CANARD_ERROR_RX_WRONG_TOGGLE:
        protocol_stats.rx_error_wrong_toggle++;
        break;
    case CANARD_ERROR_RX_UNEXPECTED_TID:
        protocol_stats.rx_ignored_unexpected_tid++;
        break;
    case CANARD_ERROR_RX_SHORT_FRAME:
        protocol_stats.rx_error_short_frame++;
        break;
    case CANARD_ERROR_RX_BAD_CRC:
        protocol_stats.rx_error_bad_crc++;
        break;
    default:
        // mark all other errors as internal
        protocol_stats.rx_error_internal++;
        break;
    }
}

// Extract source node ID from CAN ID (bits 0-6)
// Using the same macro as canard.c uses internally
#define SOURCE_ID_FROM_ID(x) ((uint8_t)(((x) >> 0U) & 0x7FU))
static inline uint8_t extractSourceNodeID(uint32_t can_id)
{
    return SOURCE_ID_FROM_ID(can_id);
}

void CanardInterface::processRx() {
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    static uint32_t total_frames = 0;
    static uint32_t extended_frames = 0;
    static uint32_t canard_calls = 0;
    static uint32_t canard_success = 0;
    static uint32_t canard_errors = 0;
    static uint32_t last_debug_ms = 0;
    static bool first_rx = true;
    
    if (first_rx) {
        ESP_LOGD("CAN_RX", "processRx first call - debugging CAN frame processing");
        first_rx = false;
    }
#endif

    AP_HAL::CANFrame rxmsg;
    for (uint8_t i=0; i<num_ifaces; i++) {
        while(true) {
            if (ifaces[i] == NULL) {
                break;
            }
            bool read_select = true;
            bool write_select = false;
            ifaces[i]->select(read_select, write_select, nullptr, 0);
            if (!read_select) { // No data pending
                break;
            }
            CanardCANFrame rx_frame {};

            //palToggleLine(HAL_GPIO_PIN_LED);
            uint64_t timestamp;
            AP_HAL::CANIface::CanIOFlags flags;
            if (ifaces[i]->receive(rxmsg, timestamp, flags) <= 0) {
                break;
            }

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
            total_frames++;
#endif

            if (!rxmsg.isExtended()) {
                // 11 bit frame, see if we have a handler
                if (aux_11bit_driver != nullptr) {
                    aux_11bit_driver->handle_frame(rxmsg);
                }
                continue;
            }

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
            extended_frames++;
            
            // Log every 100th extended frame for debugging
            if (extended_frames % 100 == 0) {
                uint32_t masked_id = rxmsg.id & CANARD_CAN_EXT_ID_MASK;
                uint16_t data_type = extractDataType(masked_id);
                uint8_t source_node = extractSourceNodeID(masked_id);
                // Also show what the real message type is for broadcast messages
                bool is_service = (masked_id >> 7) & 0x1;
                uint16_t real_msg_type = is_service ? ((masked_id >> 16) & 0xFF) : ((masked_id >> 8) & 0x7FFF);
                ESP_LOGD("CAN_RX", "Frame %d: id=0x%08X, dtype=%d (real=%d), src=%d, svc=%d, len=%d", 
                         extended_frames, masked_id, data_type, real_msg_type, source_node, is_service, rxmsg.dlc);
            }
#endif

            rx_frame.data_len = AP_HAL::CANFrame::dlcToDataLength(rxmsg.dlc);
            memcpy(rx_frame.data, rxmsg.data, rx_frame.data_len);
#if HAL_CANFD_SUPPORTED
            rx_frame.canfd = rxmsg.canfd;
#endif
            rx_frame.id = rxmsg.id;

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
            // Check if this frame appears to be from Node 2 (which shouldn't exist)
            uint32_t masked_id = rxmsg.id & CANARD_CAN_EXT_ID_MASK;
            uint8_t source_node = extractSourceNodeID(masked_id);

            if (source_node == 2) {
                // Log detailed info about this phantom frame
                bool is_service = (masked_id >> 7) & 0x1;
                uint16_t msg_type = is_service ? ((masked_id >> 16) & 0xFF) : ((masked_id >> 8) & 0x7FFF);
                uint8_t priority = (masked_id >> 24) & 0x1F;

                ESP_LOGW("CAN_RX", "=== PHANTOM NODE 2 CAN FRAME ===");
                ESP_LOGW("CAN_RX", "Raw CAN ID: 0x%08X, Masked: 0x%08X", rxmsg.id, masked_id);
                ESP_LOGW("CAN_RX", "Decoded: Priority=%d, MsgType=%d, Service=%d, SrcNode=%d",
                         priority, msg_type, is_service, source_node);
                ESP_LOGW("CAN_RX", "DLC=%d, Data bytes:", rxmsg.dlc);

                // Log raw data bytes
                char hex_str[64] = {0};
                int offset = 0;
                for (int j = 0; j < rxmsg.dlc && j < 8; j++) {
                    offset += snprintf(hex_str + offset, sizeof(hex_str) - offset, "%02X ", rxmsg.data[j]);
                }
                ESP_LOGW("CAN_RX", "  %s", hex_str);

                // Check for ASCII text in data
                bool has_ascii = false;
                for (int j = 0; j < rxmsg.dlc && j < 8; j++) {
                    if (rxmsg.data[j] >= 0x20 && rxmsg.data[j] <= 0x7E) {
                        has_ascii = true;
                        break;
                    }
                }

                if (has_ascii) {
                    char ascii_str[32] = {0};
                    int ascii_offset = 0;
                    for (int j = 0; j < rxmsg.dlc && j < 8; j++) {
                        if (rxmsg.data[j] >= 0x20 && rxmsg.data[j] <= 0x7E) {
                            ascii_str[ascii_offset++] = rxmsg.data[j];
                        } else {
                            ascii_str[ascii_offset++] = '.';
                        }
                    }
                    ESP_LOGW("CAN_RX", "  ASCII: '%s'", ascii_str);
                }

                // Check if this looks like a NodeStatus message (type 341)
                if (msg_type == 341 && !is_service) {
                    ESP_LOGW("CAN_RX", "  This appears to be a NodeStatus message!");
                }
            }
#endif

#if CANARD_MULTI_IFACE
            rx_frame.iface_id = i;
#endif
            {
                WITH_SEMAPHORE(_sem_rx);

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
                canard_calls++;
                
                // Log our node ID periodically
                static uint32_t last_nodeid_log = 0;
                uint32_t now = AP_HAL::millis();
                if (now - last_nodeid_log > 5000) {
                    ESP_LOGD("CAN_RX", "Our node ID: %d, broadcast ID: %d", 
                             canardGetLocalNodeID(&canard), CANARD_BROADCAST_NODE_ID);
                    
                    // Show acceptance criteria for a LogMessage
                    uint64_t test_sig = 0;
                    uint16_t test_dtype = 16383; // LogMessage
                    bool would_accept_broadcast = shouldAcceptTransfer(&canard, &test_sig, test_dtype, 
                                                                      CanardTransferTypeBroadcast, 36);
                    ESP_LOGD("CAN_RX", "Would accept LogMessage broadcast from node 36? %s (sig=0x%llx)",
                             would_accept_broadcast ? "YES" : "NO", (unsigned long long)test_sig);
                    last_nodeid_log = now;
                }
#endif
                const int16_t res = canardHandleRxFrame(&canard, &rx_frame, timestamp);

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
                // Log every GetNodeInfo frame result immediately with more details
                if ((rx_frame.id & CANARD_CAN_EXT_ID_MASK) != 0) {
                    uint32_t masked_id = rx_frame.id & CANARD_CAN_EXT_ID_MASK;
                    uint16_t dtype = extractDataType(masked_id);
                    if (dtype == 1) {
                        uint8_t src = extractSourceNodeID(masked_id);
                        bool is_service = (masked_id >> 7) & 1; // SNM bit: 1=service, 0=broadcast

                        if (is_service) {
                            // Service message - has destination field
                            uint8_t dest = (masked_id >> 8) & 0x7F; // Destination node ID
                            bool is_response = ((masked_id >> 15) & 1) == 0; // bit 15=0 means response
                            uint8_t service_tid = (masked_id >> 0) & 0x7F; // service transfer ID bits 6:0
                            if (src == 125) {
                                ESP_LOGI("CANARD", "canardHandleRxFrame(dtype=1 SERVICE, src=%d, dest=%d, is_resp=%d, service_tid=%d) returned %d",
                                         src, dest, is_response, service_tid, res);
                                if (dest != canard.node_id) {
                                    ESP_LOGE("CANARD", "WARNING: Response is for node %d but we are node %d!",
                                             dest, canard.node_id);
                                }
                                if (!is_response) {
                                    ESP_LOGE("CANARD", "WARNING: Frame marked as REQUEST not RESPONSE!");
                                }
                            }
                        } else {
                            // Broadcast message - no destination field (bits 14-8 are part of message type)
                            if (src == 125) {
                                ESP_LOGI("CANARD", "canardHandleRxFrame(dtype=1 BROADCAST, src=%d, no dest) returned %d",
                                         src, res);
                            }
                        }
                    }
                }
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
                // Debug GetNodeInfo multi-frame assembly from specific nodes
                if ((rx_frame.id & CANARD_CAN_EXT_ID_MASK) != 0) {
                    uint32_t masked_id = rx_frame.id & CANARD_CAN_EXT_ID_MASK;
                    uint16_t dtype = extractDataType(masked_id);
                    uint8_t src_node = extractSourceNodeID(masked_id);

                    // Track GetNodeInfo responses from problematic nodes (123, 125)
                    if (dtype == 1 && (src_node == 123 || src_node == 125)) {
                        uint8_t tail = rx_frame.data[rx_frame.data_len - 1];
                        uint8_t sot = (tail >> 7) & 1;
                        uint8_t eot = (tail >> 6) & 1;
                        uint8_t toggle = (tail >> 5) & 1;
                        uint8_t tid = tail & 0x1F;

                        static uint8_t last_tid[128] = {0xFF};  // Track per node
                        static uint8_t last_toggle[128] = {0xFF};
                        static uint32_t frame_count[128] = {0};
                        static uint32_t debug_count = 0;

                        if (sot) {
                            // Start of new transfer
                            ESP_LOGI("GETNODEINFO", "START from node %d: TID=%d, toggle=%d, SOT=%d, EOT=%d, DLC=%d",
                                     src_node, tid, toggle, sot, eot, rx_frame.data_len);
                            frame_count[src_node] = 0;
                            last_tid[src_node] = tid;
                            last_toggle[src_node] = toggle;

                            // Check if this is a single-frame transfer (SOT and EOT both set)
                            if (eot) {
                                ESP_LOGE("GETNODEINFO", "SINGLE-FRAME GetNodeInfo.Response from node %d!",
                                         src_node);
                                ESP_LOGE("GETNODEINFO", "  This is WRONG - response is ~56 bytes, needs multi-frame!");
                                ESP_LOGE("GETNODEINFO", "  Payload in frame: %d bytes (should be start of 8-frame sequence)",
                                         rx_frame.data_len - 1);
                                // Log the data bytes
                                ESP_LOGE("GETNODEINFO", "  Data: %02X %02X %02X %02X %02X %02X (tail=%02X)",
                                         rx_frame.data[0], rx_frame.data[1], rx_frame.data[2],
                                         rx_frame.data[3], rx_frame.data[4], rx_frame.data[5],
                                         tail);
                            }
                        }

                        // Always increment frame count
                        frame_count[src_node]++;

                        // Log frame details
                        if (debug_count++ < 100) {  // Limit spam
                            ESP_LOGI("GETNODEINFO", "Frame %lu from %d: SOT=%d EOT=%d TOG=%d TID=%d (res=%d)",
                                     (unsigned long)(frame_count[src_node] - 1), src_node,
                                     sot, eot, toggle, tid, res);

                            // Special logging for the final frame
                            if (eot && res == 0) {
                                ESP_LOGI("GETNODEINFO", "EOT frame processed correctly (canard always returns 0, not 1)");
                            }

                            // Check for errors
                            if (!sot && tid != last_tid[src_node]) {
                                ESP_LOGE("GETNODEINFO", "TID CHANGED! node %d: %d->%d",
                                         src_node, last_tid[src_node], tid);
                            }
                            if (!sot && last_toggle[src_node] != 0xFF && toggle == last_toggle[src_node]) {
                                ESP_LOGE("GETNODEINFO", "TOGGLE DIDN'T FLIP! node %d: still %d",
                                         src_node, toggle);
                            }
                        }

                        last_toggle[src_node] = toggle;

                        if (eot) {
                            ESP_LOGI("GETNODEINFO", "END from node %d: %lu frames, result=%d",
                                     src_node, (unsigned long)frame_count[src_node], res);
                            last_tid[src_node] = 0xFF;  // Reset for next transfer
                            last_toggle[src_node] = 0xFF;
                        }

                        // Log canard errors
                        if (res < 0 && res != -CANARD_ERROR_RX_NOT_WANTED) {
                            ESP_LOGE("GETNODEINFO", "ERROR %d from node %d at frame %lu",
                                     res, src_node, (unsigned long)frame_count[src_node]);

                            // Decode specific errors
                            const char* error_str = "UNKNOWN";
                            switch (-res) {
                                case CANARD_ERROR_RX_WRONG_TOGGLE: error_str = "WRONG_TOGGLE"; break;
                                case CANARD_ERROR_RX_UNEXPECTED_TID: error_str = "UNEXPECTED_TID"; break;
                                case CANARD_ERROR_RX_MISSED_START: error_str = "MISSED_START"; break;
                                case CANARD_ERROR_RX_SHORT_FRAME: error_str = "SHORT_FRAME"; break;
                                case CANARD_ERROR_RX_BAD_CRC: error_str = "BAD_CRC"; break;
                            }
                            ESP_LOGE("GETNODEINFO", "  Error type: %s", error_str);
                        }
                    }
                }
#endif
                
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
                // Log frames that were accepted by shouldAccept but failed in canardHandleRxFrame
                if (res <= 0) {
                    // Extract fields using masked ID (without FlagEFF)
                    uint32_t masked_id = rx_frame.id & CANARD_CAN_EXT_ID_MASK;
                    uint16_t data_type = extractDataType(masked_id);
                    uint8_t source_node = extractSourceNodeID(masked_id);

                    // Check if this is a service frame
                    bool is_service = (masked_id & 0x80) != 0;
                    if (is_service && res == -CANARD_ERROR_RX_NOT_WANTED) {
                        // Service frame not wanted - check if it's misaddressed
                        uint8_t dest_node = (masked_id >> 8) & 0x7F;
                        uint8_t our_node = canardGetLocalNodeID(&canard);
                        bool is_response = ((masked_id >> 15) & 1) == 0;

                        if (dest_node != our_node && data_type == 1) { // GetNodeInfo = type 1
                            static uint32_t misaddr_count = 0;
                            misaddr_count++;
                            if (misaddr_count <= 10) {
                                ESP_LOGE("CAN_RX", "MISADDRESSED GetNodeInfo %s from node %d to node %d (we are node %d)!",
                                         is_response ? "response" : "request",
                                         source_node, dest_node, our_node);
                                ESP_LOGE("CAN_RX", "  This node is sending responses to wrong destination!");
                            }
                        }
                    }

                    uint64_t check_sig = 0;
                    bool should_accept = shouldAcceptTransfer(&canard, &check_sig, data_type,
                                                             extractTransferType(masked_id), source_node);
                    if (should_accept && res < 0) {
                        // Rate limit mismatch messages to reduce spam
                        static uint32_t last_mismatch_log_ms = 0;
                        static uint32_t mismatch_count = 0;
                        uint32_t now_ms = AP_HAL::millis();
                        mismatch_count++;
                        
                        if (now_ms - last_mismatch_log_ms > 5000) {  // Log at most every 5 seconds
                            ESP_LOGI("CAN_RX", "MISMATCH: %lu occurrences (last: dtype=%d node=%d res=%d)",
                                     (unsigned long)mismatch_count, data_type, source_node, res);
                            // Log one example frame for debugging
                            ESP_LOGI("CAN_RX", "  Example: ID=0x%08X, len=%d",
                                     (unsigned)(rx_frame.id & 0x1FFFFFFF), rx_frame.data_len);
                            last_mismatch_log_ms = now_ms;
                            mismatch_count = 0;
                        }
                    }
                }
                
                if (res > 0) {
                    canard_success++;
                    // Log successful message assembly
                    uint32_t masked_id = rx_frame.id & CANARD_CAN_EXT_ID_MASK;
                    uint16_t data_type = extractDataType(masked_id);
                    uint8_t source_node = extractSourceNodeID(masked_id);
                    ESP_LOGI("CAN_RX", "SUCCESS: canardHandleRxFrame returned %d for dtype=%d from node=%d",
                             res, data_type, source_node);
                    // This means a complete transfer was assembled - onTransferReception should be called
                    ESP_LOGI("CAN_RX", "COMPLETE TRANSFER ASSEMBLED! onTransferReception should fire now...");
                } else if (res < 0) {
                    canard_errors++;
                    // Log every 50th error with more details
                    if (canard_errors % 50 == 1) {
                        uint32_t masked_id = rx_frame.id & CANARD_CAN_EXT_ID_MASK;
                        uint16_t data_type = extractDataType(masked_id);
                        uint8_t source_node = extractSourceNodeID(masked_id);
                        bool is_service = (masked_id >> 7) & 1; // SNM bit: 1=service, 0=broadcast
                        uint8_t dest_node = is_service ? ((masked_id >> 8) & 0x7F) : 0; // Only extract dest for service frames
                        const char* error_name = "UNKNOWN";
                        switch(-res) {
                            case CANARD_ERROR_RX_WRONG_ADDRESS: error_name = "WRONG_ADDRESS"; break;
                            case CANARD_ERROR_RX_NOT_WANTED: error_name = "NOT_WANTED"; break;
                            case CANARD_ERROR_RX_MISSED_START: error_name = "MISSED_START"; break;
                            case CANARD_ERROR_RX_WRONG_TOGGLE: error_name = "WRONG_TOGGLE"; break;
                            case CANARD_ERROR_RX_UNEXPECTED_TID: error_name = "UNEXPECTED_TID"; break;
                            case CANARD_ERROR_RX_SHORT_FRAME: error_name = "SHORT_FRAME"; break;
                            case CANARD_ERROR_RX_BAD_CRC: error_name = "BAD_CRC"; break;
                            default: break;
                        }
                        if (is_service) {
                            ESP_LOGI("CAN_RX", "ERROR %s (%d): dtype=%d SERVICE, src=%d, dest=%d, our_id=%d",
                                     error_name, res, data_type, source_node, dest_node,
                                     canardGetLocalNodeID(&canard));
                        } else {
                            ESP_LOGI("CAN_RX", "ERROR %s (%d): dtype=%d BROADCAST, src=%d, our_id=%d",
                                     error_name, res, data_type, source_node,
                                     canardGetLocalNodeID(&canard));
                        }

                        // Log the actual frame data for debugging
                        // Log the actual 29-bit CAN ID without internal flags
                        ESP_LOGI("CAN_RX", "  Frame ID: 0x%08X, DLC: %d",
                                 (unsigned)(rx_frame.id & 0x1FFFFFFF), rx_frame.data_len);

                        if (rx_frame.data_len > 0) {
                            char hex_str[25]; // 8 bytes * 3 chars per byte + null
                            char ascii_str[9] = {0}; // ASCII representation
                            int offset = 0;
                            bool has_ascii = false;

                            for (int i = 0; i < rx_frame.data_len && i < 8; i++) {
                                uint8_t byte = rx_frame.data[i];
                                offset += snprintf(hex_str + offset, sizeof(hex_str) - offset, "%02X ", byte);

                                // Check if it's printable ASCII
                                if (byte >= 0x20 && byte <= 0x7E) {
                                    ascii_str[i] = (char)byte;
                                    has_ascii = true;
                                } else {
                                    ascii_str[i] = '.';
                                }
                            }

                            ESP_LOGI("CAN_RX", "  Frame data: %s", hex_str);

                            // If frame contains ASCII text, log it as potential debug string
                            if (has_ascii) {
                                ESP_LOGW("CAN_RX", "  ASCII detected: \"%s\" - possible debug string in CAN frame!", ascii_str);

                                // Check for known debug strings that shouldn't be in DroneCAN
                                if (strstr(ascii_str, "drone") || strstr(ascii_str, "can") || strstr(ascii_str, "ebi")) {
                                    ESP_LOGE("CAN_RX", "  ALERT: Debug text found in CAN frame from node %d! This is NOT valid DroneCAN!", source_node);
                                    ESP_LOGE("CAN_RX", "  Possible causes: Another device on bus, bus corruption, or debug output sent to CAN");
                                }
                            }
                        }
                    }
                }
#endif

                if (res == -CANARD_ERROR_RX_MISSED_START) {
                    // this might remaining frames from a message that we don't accept, so check
                    uint64_t dummy_signature;
                    uint32_t masked_id = rx_frame.id & CANARD_CAN_EXT_ID_MASK;
                    if (shouldAcceptTransfer(&canard,
                                        &dummy_signature,
                                        extractDataType(masked_id),
                                        extractTransferType(masked_id),
                                        1)) { // doesn't matter what we pass here
                        update_rx_protocol_stats(res);
                    } else {
                        protocol_stats.rx_ignored_not_wanted++;
                    }
                } else {
                    update_rx_protocol_stats(res);
                }
            }
        }
    }
    
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    // Report statistics every 5 seconds
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_debug_ms >= 5000) {
        ESP_LOGD("CAN_RX", "processRx stats: total_frames=%d, extended=%d, canard_calls=%d, success=%d, errors=%d",
                 total_frames, extended_frames, canard_calls, canard_success, canard_errors);
        last_debug_ms = now_ms;
    }
#endif
}

void CanardInterface::process(uint32_t duration_ms) {
#if AP_TEST_DRONECAN_DRIVERS
    const uint64_t deadline = AP_HAL::micros64() + duration_ms*1000;
    while (AP_HAL::micros64() < deadline) {
        processTestRx();
        hal.scheduler->delay_microseconds(1000);
    }
#else
    const uint64_t deadline = AP_HAL::micros64() + duration_ms*1000;
    while (true) {
        processRx();
        processTx();
        {
            WITH_SEMAPHORE(_sem_rx);
            WITH_SEMAPHORE(_sem_tx);
            canardCleanupStaleTransfers(&canard, AP_HAL::micros64());
        }
        const uint64_t now = AP_HAL::micros64();
        if (now < deadline) {
            IGNORE_RETURN(sem_handle.wait(deadline - now));
        } else {
            break;
        }
    }
#endif
}

bool CanardInterface::add_interface(AP_HAL::CANIface *can_iface)
{
    if (num_ifaces > HAL_NUM_CAN_IFACES) {
        AP::can().log_text(AP_CANManager::LOG_ERROR, LOG_TAG, "DroneCANIfaceMgr: Num Ifaces Exceeded\n");
        return false;
    }
    if (can_iface == nullptr) {
        AP::can().log_text(AP_CANManager::LOG_ERROR, LOG_TAG, "DroneCANIfaceMgr: Iface Null\n");
        return false;
    }
    if (ifaces[num_ifaces] != nullptr) {
        AP::can().log_text(AP_CANManager::LOG_ERROR, LOG_TAG, "DroneCANIfaceMgr: Iface already added\n");
        return false;
    }
    ifaces[num_ifaces] = can_iface;
    if (ifaces[num_ifaces] == nullptr) {
        AP::can().log_text(AP_CANManager::LOG_ERROR, LOG_TAG, "DroneCANIfaceMgr: Can't alloc uavcan::iface\n");
        return false;
    }
    if (!can_iface->set_event_handle(&sem_handle)) {
        AP::can().log_text(AP_CANManager::LOG_ERROR, LOG_TAG, "DroneCANIfaceMgr: Setting event handle failed\n");
        return false;
    }
    AP::can().log_text(AP_CANManager::LOG_INFO, LOG_TAG, "DroneCANIfaceMgr: Successfully added interface %d\n", int(num_ifaces));
    num_ifaces++;
    return true;
}

// add an 11 bit auxillary driver
bool CanardInterface::add_11bit_driver(CANSensor *sensor)
{
    if (aux_11bit_driver != nullptr) {
        // only allow one
        return false;
    }
    aux_11bit_driver = sensor;
    return true;
}

// handler for outgoing frames for auxillary drivers
bool CanardInterface::write_aux_frame(AP_HAL::CANFrame &out_frame, const uint32_t timeout_us)
{
    const uint64_t tx_deadline_us = AP_HAL::micros64() + timeout_us;
    bool ret = false;
    for (uint8_t iface = 0; iface < num_ifaces; iface++) {
        if (ifaces[iface] == NULL) {
            continue;
        }
        ret |= ifaces[iface]->send(out_frame, tx_deadline_us, 0) > 0;
    }
    return ret;
}

#endif // #if HAL_ENABLE_DRONECAN_DRIVERS
