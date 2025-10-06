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
 *
 * Author: Siddharth Bharat Purohit
 */

#include <AP_HAL/AP_HAL.h>

#if HAL_ENABLE_DRONECAN_DRIVERS

#include "AP_DroneCAN_DNA_Server.h"
#include <stddef.h>  // for offsetof
#include "AP_DroneCAN.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
// Shared node name storage for logging across functions
static char g_node_names[128][16] = {};
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
#include "esp_log.h"
#endif
#include <StorageManager/StorageManager.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <stdio.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
#include <AP_HAL_ESP32/ESP32_Debug.h>
#include "esp_log.h"
#endif
extern const AP_HAL::HAL& hal;

// FORMAT REVISION DREAMS (things to address if the NodeRecord needs to be changed substantially)
// * have DNA server accept only a 16 byte local UID to avoid overhead from variable sized hash
// * have a real empty flag for entries and/or use a CRC which is not zero for an input of all zeros
// * fix FNV-1a hash folding to be to 48 bits (6 bytes) instead of 56

#define NODERECORD_MAGIC 0xAC01
#define NODERECORD_MAGIC_LEN 2 // uint16_t
#define MAX_NODE_ID    125
#define NODERECORD_LOC(node_id) ((node_id * sizeof(NodeRecord)) + NODERECORD_MAGIC_LEN)

#define debug_dronecan(level_debug, fmt, args...) do { AP::can().log_text(level_debug, "DroneCAN", fmt, ##args); } while (0)

// database is currently shared by all DNA servers
AP_DroneCAN_DNA_Server::Database AP_DroneCAN_DNA_Server::db;

// initialize database (storage accessor is always replaced with the one supplied)
void AP_DroneCAN_DNA_Server::Database::init(StorageAccess *storage_)
{
    // storage size must be synced with StorageCANDNA entry in StorageManager.cpp
    static_assert(NODERECORD_LOC(MAX_NODE_ID+1) <= 1024, "DNA storage too small");

    // might be called from multiple threads if multiple servers use the same database
    WITH_SEMAPHORE(sem);

    storage = storage_; // use supplied accessor

    // validate magic number
    uint16_t magic = storage->read_uint16(0);
    
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    ESP_LOGD("DNA_SERVER", "Database init: magic=0x%04x (expected 0x%04x)", 
             magic, NODERECORD_MAGIC);
    hal.console->printf("DNA: Database magic=0x%04x (expected 0x%04x)\n", 
                       magic, NODERECORD_MAGIC);
#endif
    
    if (magic != NODERECORD_MAGIC) {
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
        ESP_LOGW("DNA_SERVER", "Invalid magic, resetting database!");
        hal.console->printf("DNA: Invalid magic, resetting database!\n");
#endif
        reset(); // resetting the database will put the magic back
    }

    // check and note each possible node ID's registration's presence
    uint8_t registered_count = 0;
    for (uint8_t i = 1; i <= MAX_NODE_ID; i++) {
        if (check_registration(i)) {
            node_registered.set(i);
            registered_count++;
        }
    }
    
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    ESP_LOGD("DNA_SERVER", "Found %d existing node registrations in database", registered_count);
    hal.console->printf("DNA: Found %d existing node registrations\n", registered_count);
#endif
}

// remove all registrations from the database
void AP_DroneCAN_DNA_Server::Database::reset()
{
    WITH_SEMAPHORE(sem);

    NodeRecord record;
    memset(&record, 0, sizeof(record));
    node_registered.clearall();

    // all-zero record means no registration
    // ensure node ID 0 is cleared even if we can't use it so we know the state
    for (uint8_t i = 0; i <= MAX_NODE_ID; i++) {
        write_record(record, i);
    }

    // mark the magic at the start to indicate a valid (and reset) database
    storage->write_uint16(0, NODERECORD_MAGIC);
}

// handle initializing the server with its own node ID and unique ID
void AP_DroneCAN_DNA_Server::Database::init_server(uint8_t own_node_id, const uint8_t own_unique_id[], uint8_t own_unique_id_len)
{
    WITH_SEMAPHORE(sem);

    // register server node ID and unique ID if not correctly registered. note
    // that ardupilot mixes the node ID into the unique ID so changing the node
    // ID will "leak" the old node ID
    if (own_node_id != find_node_id(own_unique_id, own_unique_id_len)) {
        register_uid(own_node_id, own_unique_id, own_unique_id_len);
    }
}

// handle processing the node info message. returns true if from a duplicate node
bool AP_DroneCAN_DNA_Server::Database::handle_node_info(uint8_t source_node_id, const uint8_t unique_id[], const Bitmask<128> *node_healthy)
{
    WITH_SEMAPHORE(sem);

    // UID is the source of truth - find what node ID this UID is registered to
    uint8_t registered_node_id = find_node_id(unique_id, 16);

    if (registered_node_id == 0) {
        // UID not in database at all - register it
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
        ESP_LOGI("DNA_DB", "New UID detected, registering node %d", source_node_id);
#endif
        register_uid(source_node_id, unique_id, 16);
        return false;
    }

    if (registered_node_id == source_node_id) {
        // UID and node ID match our records - all good
        return false;
    }

    // UID is registered to a DIFFERENT node ID
    // Check if the rightful owner is still active
    bool rightful_owner_active = node_healthy && node_healthy->get(registered_node_id);

    if (rightful_owner_active) {
        // Rightful owner still active - update database to remap UID
        // (allocation should have given this device a different ID, but handle it here too)
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
        ESP_LOGW("DNA_DB", "UID conflict: Node %d using ID registered to active node %d",
                 source_node_id, registered_node_id);
        ESP_LOGW("DNA_DB", "  Updating mapping (device got wrong ID somehow)");
#endif
    } else {
        // Rightful owner offline - allow takeover
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
        ESP_LOGW("DNA_DB", "Allowing takeover: Node %d (inactive) → Node %d (active)",
                 registered_node_id, source_node_id);
#endif
    }

    // Clear old registration and register with new node ID
    NodeRecord empty_record;
    memset(&empty_record, 0, sizeof(empty_record));
    write_record(empty_record, registered_node_id);
    node_registered.clear(registered_node_id);
    register_uid(source_node_id, unique_id, 16);

    return false; // Allow it (allocation should prevent conflicts, but handle here too)
}

// handle the allocation message. returns the allocated node ID, or 0 if allocation failed
uint8_t AP_DroneCAN_DNA_Server::Database::handle_allocation(const uint8_t unique_id[], const Bitmask<128> *node_seen)
{
    WITH_SEMAPHORE(sem);

    uint8_t resp_node_id = find_node_id(unique_id, 16);

    if (resp_node_id != 0) {
        // UID found in database with existing node ID assignment
        // Check if this node ID is currently in use by checking node_seen
        // If it's in use, there might be a conflict (different device using this ID)
        // Clear the stale registration and assign a new ID to avoid conflicts
        if (node_seen && node_seen->get(resp_node_id)) {
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
            ESP_LOGW("DNA_DB", "UID wants node %d which is in use - assigning new ID to avoid conflict", resp_node_id);
#endif
            delete_registration(resp_node_id);
            resp_node_id = 0; // Force new allocation below
        } else {
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
            ESP_LOGD("DNA_DB", "UID found with node %d (not in use), re-assigning", resp_node_id);
#endif
            return resp_node_id; // Safe to return - node not currently active
        }
    }

    if (resp_node_id == 0) {
        // find free node ID, starting at the max as prescribed by the standard
        resp_node_id = MAX_NODE_ID;
        while (resp_node_id > 0) {
            if (!node_registered.get(resp_node_id)) {
                break;
            }
            resp_node_id--;
        }

        if (resp_node_id != 0) {
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
            ESP_LOGD("DNA_DB", "Assigning new node ID %d", resp_node_id);
#endif
            create_registration(resp_node_id, unique_id, 16);
        } else {
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
            ESP_LOGE("DNA_DB", "No free node IDs available in database");
#endif
        }
    }

    return resp_node_id; // will be 0 if not found and not created
}

// retrieve node ID that matches the given unique ID. returns 0 if not found
uint8_t AP_DroneCAN_DNA_Server::Database::find_node_id(const uint8_t unique_id[], uint8_t size)
{
    NodeRecord record, cmp_record;
    compute_uid_hash(cmp_record, unique_id, size);

    for (int i = MAX_NODE_ID; i > 0; i--) {
        if (node_registered.get(i)) {
            read_record(record, i);
            if (memcmp(record.uid_hash, cmp_record.uid_hash, sizeof(NodeRecord::uid_hash)) == 0) {
                return i; // node ID found
            }
        }
    }
    return 0; // not found
}

// fill the given record with the hash of the given unique ID
void AP_DroneCAN_DNA_Server::Database::compute_uid_hash(NodeRecord &record, const uint8_t unique_id[], uint8_t size) const
{
    uint64_t hash = FNV_1_OFFSET_BASIS_64;
    hash_fnv_1a(size, unique_id, &hash);

    // xor-folding per http://www.isthe.com/chongo/tech/comp/fnv/
    hash = (hash>>56) ^ (hash&(((uint64_t)1<<56)-1)); // 56 should be 48 since we use 6 bytes

    // write it to ret
    for (uint8_t i=0; i<6; i++) {
        record.uid_hash[i] = (hash >> (8*i)) & 0xff;
    }
}

// register a given unique ID to a given node ID, deleting any existing registration for the unique ID
void AP_DroneCAN_DNA_Server::Database::register_uid(uint8_t node_id, const uint8_t unique_id[], uint8_t size)
{
    uint8_t prev_node_id = find_node_id(unique_id, size); // have we registered this unique ID under a different node ID?
    if (prev_node_id != 0) {
        delete_registration(prev_node_id); // yes, delete old node ID's registration
    }
    // overwrite an existing registration with this node ID, if any
    create_registration(node_id, unique_id, size);
}

// create the registration for the given node ID and set its record's unique ID
void AP_DroneCAN_DNA_Server::Database::create_registration(uint8_t node_id, const uint8_t unique_id[], uint8_t size)
{
    NodeRecord record;
    compute_uid_hash(record, unique_id, size);
    // compute and store CRC of the record's data to validate it
    record.crc = crc_crc8(record.uid_hash, sizeof(record.uid_hash));

    write_record(record, node_id);

    node_registered.set(node_id);
}

// delete the given node ID's registration
void AP_DroneCAN_DNA_Server::Database::delete_registration(uint8_t node_id)
{
    if (node_id > MAX_NODE_ID) {
        return;
    }

    NodeRecord record;

    // all-zero record means no registration
    memset(&record, 0, sizeof(record));
    write_record(record, node_id);
    node_registered.clear(node_id);
}

// return true if the given node ID has a registration
bool AP_DroneCAN_DNA_Server::Database::check_registration(uint8_t node_id)
{
    NodeRecord record;
    read_record(record, node_id);

    uint8_t empty_uid[sizeof(NodeRecord::uid_hash)] {};
    uint8_t crc = crc_crc8(record.uid_hash, sizeof(record.uid_hash));
    if (crc == record.crc && memcmp(&record.uid_hash[0], &empty_uid[0], sizeof(empty_uid)) != 0) {
        return true; // CRC matches and UID hash is not all zero
    }
    return false;
}

// read the given node ID's registration's record
void AP_DroneCAN_DNA_Server::Database::read_record(NodeRecord &record, uint8_t node_id)
{
    if (node_id > MAX_NODE_ID) {
        return;
    }

    storage->read_block(&record, NODERECORD_LOC(node_id), sizeof(NodeRecord));
    
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    // Debug: Check if record has valid data
    bool is_empty = true;
    for (int i = 0; i < sizeof(NodeRecord::uid_hash); i++) {
        if (record.uid_hash[i] != 0) {
            is_empty = false;
            break;
        }
    }
    if (!is_empty) {
        ESP_LOGD("DNA_SERVER", "Read record for node_id=%d from offset=%d (has data)", 
                 node_id, NODERECORD_LOC(node_id));
    }
#endif
}

// write the given node ID's registration's record
void AP_DroneCAN_DNA_Server::Database::write_record(const NodeRecord &record, uint8_t node_id)
{
    if (node_id > MAX_NODE_ID) {
        return;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    ESP_LOGD("DNA_SERVER", "Writing record for node_id=%d to storage offset=%d", 
             node_id, NODERECORD_LOC(node_id));
    hal.console->printf("DNA: Writing node %d to storage offset %d\n", 
                       node_id, NODERECORD_LOC(node_id));
#endif
    
    storage->write_block(NODERECORD_LOC(node_id), &record, sizeof(NodeRecord));
}


AP_DroneCAN_DNA_Server::AP_DroneCAN_DNA_Server(AP_DroneCAN &ap_dronecan, CanardInterface &canard_iface, uint8_t driver_index) :
    storage(StorageManager::StorageCANDNA),
    curr_verifying_node(0),
    _ap_dronecan(ap_dronecan),
    _canard_iface(canard_iface),
    allocation_sub(allocation_cb, driver_index),
    node_status_sub(node_status_cb, driver_index),
    node_info_client(_canard_iface, node_info_cb) {}

/* Initialises Publishers for respective UAVCAN Instance
Also resets the Server Record in case there is a mismatch
between specified node id and unique id against the existing
Server Record. */
bool AP_DroneCAN_DNA_Server::init(uint8_t own_unique_id[], uint8_t own_unique_id_len, uint8_t node_id)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    ESP_LOGD("DNA_SERVER", "Init called with node_id=%d", node_id);
#endif
    hal.console->printf("DNA: Server init called with node_id=%d\n", node_id);
    
    //Read the details from AP_DroneCAN
    server_state = HEALTHY;

    db.init(&storage); // initialize the database with our accessor

    if (_ap_dronecan.check_and_reset_option(AP_DroneCAN::Options::DNA_CLEAR_DATABASE)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UC DNA database reset");
        db.reset();
    }

    db.init_server(node_id, own_unique_id, own_unique_id_len);

    /* Also add to seen node id this is to verify
    if any duplicates are on the bus carrying our Node ID */
    node_seen.set(node_id);
    node_verified.set(node_id);
    node_healthy.set(node_id);
    self_node_id = node_id;
    
    // Set timeout for allocation responses (must be set before use)
    allocation_pub.set_timeout_ms(100);
    
    hal.console->printf("DNA: Server initialized successfully, self_node_id=%d\n", self_node_id);
    hal.console->printf("DNA: Allocation subscription should be active\n");
    
    return true;
}

/* Run through the list of seen node ids for verification no more
than once per 5 second. We continually verify the nodes in our 
seen list, So that we can raise issue if there are duplicates
on the bus. */
void AP_DroneCAN_DNA_Server::verify_nodes()
{
    uint32_t now = AP_HAL::millis();
    if ((now - last_verification_request) < 5000) {
        return;
    }

#if HAL_LOGGING_ENABLED
    uint8_t log_count = AP::logger().get_log_start_count();
    if (log_count != last_logging_count) {
        last_logging_count = log_count;
        node_logged.clearall();
    }
#endif

    // Track verification failures per node
    static uint32_t verification_attempt_count[128] = {0};

    //Check if we got acknowledgement from previous request
    //except for requests using our own node_id
    if (curr_verifying_node == self_node_id) {
        nodeInfo_resp_rcvd = true;
    }

    if (!nodeInfo_resp_rcvd) {
        /* Node failed to respond - could be disconnected or conflicting ID */
        node_verified.clear(curr_verifying_node);

        // Report node as potentially offline if it was previously healthy
        if (node_healthy.get(curr_verifying_node)) {
            node_healthy.clear(curr_verifying_node);
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "DroneCAN Node %d: No response - possibly OFFLINE",
                          curr_verifying_node);
        }
    } else {
        // Reset failure count on successful verification
        verification_attempt_count[curr_verifying_node] = 0;
    }

    last_verification_request = now;
    //Find the next registered Node ID to be verified.
    for (uint8_t i = 0; i <= MAX_NODE_ID; i++) {
        curr_verifying_node = (curr_verifying_node + 1) % (MAX_NODE_ID + 1);
        if ((curr_verifying_node == self_node_id) || (curr_verifying_node == 0)) {
            continue;
        }
        if (node_seen.get(curr_verifying_node)) {
            break;
        }
    }
    if (db.is_registered(curr_verifying_node)) {
        // Don't try to verify ourselves
        if (curr_verifying_node == self_node_id) {
            node_verified.set(curr_verifying_node);
            node_healthy.set(curr_verifying_node);
        } else {
            // Increment verification attempt count
            verification_attempt_count[curr_verifying_node]++;

            uavcan_protocol_GetNodeInfoRequest request;
            ESP_LOGD("DNA_SERVER", "Sending GetNodeInfo request to node %d (attempt %lu)",
                     curr_verifying_node,
                     (unsigned long)verification_attempt_count[curr_verifying_node]);
            node_info_client.request(curr_verifying_node, request);
            nodeInfo_resp_rcvd = false;

            // Warn about nodes that repeatedly fail verification
            if (verification_attempt_count[curr_verifying_node] % 10 == 0 &&
                verification_attempt_count[curr_verifying_node] > 0) {
                hal.console->printf("DNA: Node %d failed verification %lu times - possible decode or protocol issue\n",
                                  curr_verifying_node,
                                  (unsigned long)verification_attempt_count[curr_verifying_node]);
            }
        }
    }
}

/* Handles Node Status Message, adds to the Seen Node list
Also starts the Service call for Node Info to complete the
Verification process. */
void AP_DroneCAN_DNA_Server::handleNodeStatus(const CanardRxTransfer& transfer, const uavcan_protocol_NodeStatus& msg)
{
    if (transfer.source_node_id > MAX_NODE_ID || transfer.source_node_id == 0) {
        return;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    // Log raw data for Node 2 to debug corruption
    if (transfer.source_node_id == 2) {
        ESP_LOGW("DRONECAN", "=== PHANTOM NODE 2 DETECTED ===");
        ESP_LOGW("DRONECAN", "NodeStatus from Node 2 - THIS SHOULD NOT EXIST!");
        ESP_LOGW("DRONECAN", "Transfer details: priority=%d, data_type_id=%d, payload_len=%d",
                 transfer.priority, transfer.data_type_id, transfer.payload_len);

        // Log the raw payload bytes
        if (transfer.payload_head != nullptr && transfer.payload_len > 0) {
            char hex_str[256] = {0};
            int offset = 0;
            for (int i = 0; i < transfer.payload_len && i < 32; i++) {
                offset += snprintf(hex_str + offset, sizeof(hex_str) - offset, "%02X ",
                                 ((const uint8_t*)transfer.payload_head)[i]);
            }
            ESP_LOGW("DRONECAN", "Raw payload bytes: %s", hex_str);
        }

        // Log the NodeStatus content
        ESP_LOGW("DRONECAN", "NodeStatus content: uptime=%lu, health=%d, mode=%d, sub_mode=%d, vssc=%d",
                 (unsigned long)msg.uptime_sec, msg.health, msg.mode, msg.sub_mode, msg.vendor_specific_status_code);
    }

    static uint32_t last_nodestatus_time[128] = {};
    static uint32_t nodestatus_count[128] = {};
    uint32_t now = AP_HAL::millis();
    uint32_t dt = now - last_nodestatus_time[transfer.source_node_id];

    nodestatus_count[transfer.source_node_id]++;

    // Log all NodeStatus messages periodically (every 10th message)
    if (nodestatus_count[transfer.source_node_id] % 10 == 1) {
        // Get node name if available (stored in handleNodeInfo)
        const char* node_name = g_node_names[transfer.source_node_id][0] ? g_node_names[transfer.source_node_id] : "unknown";

        // Decode health and mode values per UAVCAN spec
        const char* health_str = "UNKNOWN";
        switch (msg.health) {
            case 0: health_str = "OK"; break;
            case 1: health_str = "WARNING"; break;
            case 2: health_str = "ERROR"; break;
            case 3: health_str = "CRITICAL"; break;
        }

        const char* mode_str = "UNKNOWN";
        switch (msg.mode) {
            case 0: mode_str = "OPERATIONAL"; break;
            case 1: mode_str = "INITIALIZATION"; break;
            case 2: mode_str = "MAINTENANCE"; break;
            case 3: mode_str = "SOFTWARE_UPDATE"; break;
            case 4: mode_str = "OFFLINE"; break;
            default: mode_str = "RESERVED"; break;
        }

        // Convert uptime to human-readable format
        uint32_t uptime = msg.uptime_sec;
        uint32_t days = uptime / 86400;
        uint32_t hours = (uptime % 86400) / 3600;
        uint32_t minutes = (uptime % 3600) / 60;
        uint32_t seconds = uptime % 60;

        if (days > 0) {
            ESP_LOGI("DRONECAN", "Node %d (%s) NodeStatus #%lu: uptime=%lud%luh%lum, health=%d (%s), mode=%d (%s), interval=%lums",
                     transfer.source_node_id, node_name,
                     (unsigned long)nodestatus_count[transfer.source_node_id],
                     (unsigned long)days, (unsigned long)hours, (unsigned long)minutes,
                     msg.health, health_str, msg.mode, mode_str,
                     (unsigned long)dt);
        } else if (hours > 0) {
            ESP_LOGI("DRONECAN", "Node %d (%s) NodeStatus #%lu: uptime=%luh%lum%lus, health=%d (%s), mode=%d (%s), interval=%lums",
                     transfer.source_node_id, node_name,
                     (unsigned long)nodestatus_count[transfer.source_node_id],
                     (unsigned long)hours, (unsigned long)minutes, (unsigned long)seconds,
                     msg.health, health_str, msg.mode, mode_str,
                     (unsigned long)dt);
        } else if (minutes > 0) {
            ESP_LOGI("DRONECAN", "Node %d (%s) NodeStatus #%lu: uptime=%lum%lus, health=%d (%s), mode=%d (%s), interval=%lums",
                     transfer.source_node_id, node_name,
                     (unsigned long)nodestatus_count[transfer.source_node_id],
                     (unsigned long)minutes, (unsigned long)seconds,
                     msg.health, health_str, msg.mode, mode_str,
                     (unsigned long)dt);
        } else {
            ESP_LOGI("DRONECAN", "Node %d (%s) NodeStatus #%lu: uptime=%lus, health=%d (%s), mode=%d (%s), interval=%lums",
                     transfer.source_node_id, node_name,
                     (unsigned long)nodestatus_count[transfer.source_node_id],
                     (unsigned long)seconds,
                     msg.health, health_str, msg.mode, mode_str,
                     (unsigned long)dt);
        }
    }

    // Warn about irregular intervals
    if (last_nodestatus_time[transfer.source_node_id] != 0 && (dt > 1500 || dt < 50)) {
        ESP_LOGW("DRONECAN", "Node %d NodeStatus IRREGULAR interval %lums (expected ~1000ms)",
                 transfer.source_node_id, (unsigned long)dt);
    }
    last_nodestatus_time[transfer.source_node_id] = now;
#endif
    
    // Send UAVCAN_NODE_STATUS MAVLink message for proper GCS reporting
    send_node_status_mavlink(transfer.source_node_id, msg);
    
    // Handle health state changes with severity-based reporting
    const bool was_healthy = node_healthy.get(transfer.source_node_id);
    const bool is_operational = (msg.mode == UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL);
    const bool is_healthy = (msg.health == UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK);
    
    if ((!is_healthy || !is_operational) &&
        !_ap_dronecan.option_is_set(AP_DroneCAN::Options::DNA_IGNORE_UNHEALTHY_NODE)) {
        
        // Report health status changes with appropriate severity
        if (was_healthy) {
            report_node_health_change(transfer.source_node_id, msg.health, msg.mode, false);
        }
        
        fault_node_id = transfer.source_node_id;
        server_state = NODE_STATUS_UNHEALTHY;
        node_healthy.clear(transfer.source_node_id);
    } else {
        // Node is now healthy
        if (!was_healthy && is_healthy && is_operational) {
            report_node_health_change(transfer.source_node_id, msg.health, msg.mode, true);
        }
        
        node_healthy.set(transfer.source_node_id);
        if (node_healthy == node_verified) {
            server_state = HEALTHY;
        }
    }
    if (!node_verified.get(transfer.source_node_id)) {
        // Don't try to verify ourselves - we already know our own info
        if (transfer.source_node_id == self_node_id) {
            node_verified.set(transfer.source_node_id);
            node_healthy.set(transfer.source_node_id);
        } else {
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
            ESP_LOGI("DRONECAN", "Node %d not verified, sending GetNodeInfo request from our node %d", 
                     transfer.source_node_id, _ap_dronecan.canard_iface.get_node_id());
#endif
            //immediately begin verification of the node_id
            uavcan_protocol_GetNodeInfoRequest request;
            ESP_LOGI("DNA_SERVER", "Sending GetNodeInfo request to node %d (after allocation)", transfer.source_node_id);
            node_info_client.request(transfer.source_node_id, request);
        }
    }
    //Add node to seen list if not seen before
    node_seen.set(transfer.source_node_id);
}

void AP_DroneCAN_DNA_Server::send_node_status_mavlink(uint8_t node_id, const uavcan_protocol_NodeStatus& msg)
{
#if AP_HAVE_GCS_SEND_MAVLINK
    // Send UAVCAN_NODE_STATUS MAVLink message (ID 310)
    mavlink_message_t mavlink_msg;
    mavlink_msg_uavcan_node_status_pack(
        mavlink_system.sysid,
        node_id,  // Use node_id as component_id per MAVLink-UAVCAN bridge spec
        &mavlink_msg,
        AP_HAL::micros64(),  // time_usec
        msg.uptime_sec,      // uptime_sec  
        msg.health,          // health
        msg.mode,            // mode
        msg.sub_mode,        // sub_mode
        msg.vendor_specific_status_code  // vendor_specific_status_code
    );
    
    // Send to all active MAVLink channels
    GCS_MAVLINK::send_to_active_channels(&mavlink_msg);
#endif
}

void AP_DroneCAN_DNA_Server::report_node_health_change(uint8_t node_id, uint8_t health, uint8_t mode, bool recovered)
{
#if AP_HAVE_GCS_SEND_TEXT
    const char* health_str;
    MAV_SEVERITY severity;
    
    // Map DroneCAN health states to MAVLink severity and text
    switch (health) {
        case UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK:
            health_str = "OK";
            severity = MAV_SEVERITY_INFO;
            break;
        case UAVCAN_PROTOCOL_NODESTATUS_HEALTH_WARNING:
            health_str = "WARNING";
            severity = MAV_SEVERITY_WARNING;
            break;
        case UAVCAN_PROTOCOL_NODESTATUS_HEALTH_ERROR:
            health_str = "ERROR";
            severity = MAV_SEVERITY_ERROR;
            break;
        case UAVCAN_PROTOCOL_NODESTATUS_HEALTH_CRITICAL:
            health_str = "CRITICAL";
            severity = MAV_SEVERITY_CRITICAL;
            break;
        default:
            health_str = "UNKNOWN";
            severity = MAV_SEVERITY_WARNING;
            break;
    }
    
    const char* mode_str;
    switch (mode) {
        case UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL:
            mode_str = "OPERATIONAL";
            break;
        case UAVCAN_PROTOCOL_NODESTATUS_MODE_INITIALIZATION:
            mode_str = "INITIALIZING";
            break;
        case UAVCAN_PROTOCOL_NODESTATUS_MODE_MAINTENANCE:
            mode_str = "MAINTENANCE";
            break;
        case UAVCAN_PROTOCOL_NODESTATUS_MODE_SOFTWARE_UPDATE:
            mode_str = "UPDATING";
            break;
        case UAVCAN_PROTOCOL_NODESTATUS_MODE_OFFLINE:
            mode_str = "OFFLINE";
            break;
        default:
            mode_str = "UNKNOWN";
            break;
    }
    
    if (recovered) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "DroneCAN Node %d: %s (%s) - RECOVERED", 
                      node_id, health_str, mode_str);
    } else {
        GCS_SEND_TEXT(severity, "DroneCAN Node %d: %s (%s)", 
                      node_id, health_str, mode_str);
    }
#endif
}

void AP_DroneCAN_DNA_Server::send_node_info_mavlink(uint8_t node_id, const uavcan_protocol_GetNodeInfoResponse& msg)
{
#if AP_HAVE_GCS_SEND_MAVLINK
    // Send UAVCAN_NODE_INFO MAVLink message (ID 311) 
    mavlink_message_t mavlink_msg;
    mavlink_msg_uavcan_node_info_pack(
        mavlink_system.sysid,
        node_id,  // Use node_id as component_id per MAVLink-UAVCAN bridge spec
        &mavlink_msg,
        AP_HAL::micros64(),           // time_usec
        msg.node_status.uptime_sec,   // uptime_sec
        (char*)msg.name.data,         // name
        msg.hardware_version.major,   // hw_version_major
        msg.hardware_version.minor,   // hw_version_minor  
        msg.hardware_version.unique_id, // hw_unique_id (first 16 bytes)
        msg.software_version.major,   // sw_version_major
        msg.software_version.minor,   // sw_version_minor
        msg.software_version.vcs_commit  // sw_vcs_commit
    );
    
    // Send to all active MAVLink channels
    GCS_MAVLINK::send_to_active_channels(&mavlink_msg);
    
    // Also send a text message for human-readable notification
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "DroneCAN Node %d: %s v%d.%d online", 
                  node_id, msg.name.data, 
                  msg.software_version.major, msg.software_version.minor);
#endif
}

/* Node Info message handler
Handle responses from GetNodeInfo Request. We verify the node info
against our records. Marks Verification mask if already recorded,
Or register if the node id is available and not recorded for the
received Unique ID */
void AP_DroneCAN_DNA_Server::handleNodeInfo(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoResponse& rsp)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    ESP_LOGD("DNA_SERVER", "handleNodeInfo CALLED for node %d", transfer.source_node_id);
#endif
    if (transfer.source_node_id > MAX_NODE_ID || transfer.source_node_id == 0) {
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
#endif
        return;
    }
    
    // Send UAVCAN_NODE_INFO MAVLink message for node discovery
    send_node_info_mavlink(transfer.source_node_id, rsp);
    /*
      if we haven't logged this node then log it now
     */
#if HAL_LOGGING_ENABLED
    if (!node_logged.get(transfer.source_node_id) && AP::logger().logging_started()) {
        node_logged.set(transfer.source_node_id);
        uint64_t uid[2];
        memcpy(uid, rsp.hardware_version.unique_id, sizeof(rsp.hardware_version.unique_id));
        // @LoggerMessage: CAND
        // @Description: Info from GetNodeInfo request
        // @Field: TimeUS: Time since system startup
        // @Field: Driver: Driver index
        // @Field: NodeId: Node ID
        // @Field: UID1: Hardware ID, part 1
        // @Field: UID2: Hardware ID, part 2
        // @Field: Name: Name string
        // @Field: Major: major revision id
        // @Field: Minor: minor revision id
        // @Field: Version: AP_Periph git hash
        AP::logger().Write("CAND", "TimeUS,Driver,NodeId,UID1,UID2,Name,Major,Minor,Version",
                           "s-#------", "F--------", "QBBQQZBBI",
                           AP_HAL::micros64(),
                           _ap_dronecan.get_driver_index(),
                           transfer.source_node_id,
                           uid[0], uid[1],
                           rsp.name.data,
                           rsp.software_version.major,
                           rsp.software_version.minor,
                           rsp.software_version.vcs_commit);
    }
#endif

    bool duplicate = db.handle_node_info(transfer.source_node_id, rsp.hardware_version.unique_id, &node_healthy);
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    ESP_LOGD("DNA_SERVER", "Node %d: duplicate=%s, name='%s'",
             transfer.source_node_id, duplicate ? "YES" : "NO", rsp.name.data);
#endif
    if (duplicate) {
        if (!_ap_dronecan.option_is_set(AP_DroneCAN::Options::DNA_IGNORE_DUPLICATE_NODE)) {
            /* This is a device with node_id already registered
            for another device */
            server_state = DUPLICATE_NODES;
            fault_node_id = transfer.source_node_id;
            memcpy(fault_node_name, rsp.name.data, sizeof(fault_node_name));
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
            ESP_LOGE("DNA_SERVER", "DUPLICATE NODE DETECTED! Node %d", transfer.source_node_id);
#endif
        }
    } else {
        //Verify as well
        node_verified.set(transfer.source_node_id);
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
        // Store node name for later use in NodeStatus logging
        strncpy(g_node_names[transfer.source_node_id], (const char*)rsp.name.data, 15);
        g_node_names[transfer.source_node_id][15] = 0;

        ESP_LOGD("DNA_SERVER", "Node %d VERIFIED successfully", transfer.source_node_id);
#endif
        if (transfer.source_node_id == curr_verifying_node) {
            nodeInfo_resp_rcvd = true;
        }
    }
}

// process node ID allocation messages for DNA
void AP_DroneCAN_DNA_Server::handle_allocation(const CanardRxTransfer& transfer, const uavcan_protocol_dynamic_node_id_Allocation& msg)
{
    // Always log that we entered the handler
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    uint32_t start_ms = AP_HAL::millis();
    
    // Track how often we're being called
    static uint32_t last_call_log = 0;
    static uint32_t call_count = 0;
    call_count++;
    if (start_ms - last_call_log > 1000) {
        if (call_count > 10) {
            ESP_LOGE("DNA_SERVER", "handle_allocation called %lu times/sec! source=%d, first=%d, uid_len=%d", 
                     call_count, transfer.source_node_id, msg.first_part_of_unique_id, msg.unique_id.len);
        } else {
            ESP_LOGD("DNA_SERVER", "handle_allocation at T+%lums! source=%d, first=%d, uid_len=%d", 
                     (unsigned long)start_ms, transfer.source_node_id, msg.first_part_of_unique_id, msg.unique_id.len);
        }
        call_count = 0;
        last_call_log = start_ms;
    }
#else
    hal.console->printf("DNA: handle_allocation at T+%lums! source=%d, first=%d, uid_len=%d\n", 
                       (unsigned long)AP_HAL::millis(), transfer.source_node_id, msg.first_part_of_unique_id, msg.unique_id.len);
#endif
    
    if (transfer.source_node_id != 0) {
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
        ESP32_DEBUG_WARNING("DNA: Ignoring allocation - not from anonymous node (source=%d)", transfer.source_node_id);
#endif
        hal.console->printf("DNA: Ignoring allocation - not from anonymous node (source=%d)\n", transfer.source_node_id);
        return; // ignore allocation messages that are not DNA requests
    }
    uint32_t now = AP_HAL::millis();

    if ((now - last_alloc_msg_ms) > UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_FOLLOWUP_TIMEOUT_MS) {
        rcvd_unique_id_offset = 0; // reset state, timed out
        memset(rcvd_unique_id, 0, sizeof(rcvd_unique_id)); // Clear buffer on timeout
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
        if (rcvd_unique_id_offset > 0) {
            ESP32_DEBUG_WARNING("DNA: Allocation timeout - resetting state after %ums", 
                            (unsigned)(now - last_alloc_msg_ms));
        }
#endif
    }

    if (msg.first_part_of_unique_id) {
        // nodes waiting to send a follow up will instead send a first part
        // again if they see another allocation message. therefore we should
        // always reset and process a received first part, since any node we
        // were allocating will never send its follow up and we'd just time out
        rcvd_unique_id_offset = 0;
        memset(rcvd_unique_id, 0, sizeof(rcvd_unique_id)); // Clear entire buffer to avoid garbage
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
        ESP_LOGD("DNA_SERVER", "First part received, resetting state and clearing buffer");
#endif
    } else if (rcvd_unique_id_offset == 0) {
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
        ESP_LOGW("DNA_SERVER", "WARNING: Follow-up without first part (offset=0) - treating as first part");
        // Workaround: Some nodes send follow-ups without proper first_part flag
        // We'll treat this as a first part to allow allocation to proceed
#else
        return; // not first part but we are expecting one, ignore
#endif
    }

    if (rcvd_unique_id_offset) {
        debug_dronecan(AP_CANManager::LOG_DEBUG, "TIME: %lu  -- Accepting Followup part! %u\n",
                     (unsigned long)now,
                     unsigned((now - last_alloc_msg_ms)));
    } else {
        debug_dronecan(AP_CANManager::LOG_DEBUG, "TIME: %lu  -- Accepting First part! %u\n",
                     (unsigned long)now,
                     unsigned((now - last_alloc_msg_ms)));
    }

    last_alloc_msg_ms = now;
    if ((rcvd_unique_id_offset + msg.unique_id.len) > sizeof(rcvd_unique_id)) {
        rcvd_unique_id_offset = 0; // reset state, request contains an over-long ID
        return;
    }

    // save the new portion of the unique ID
    memcpy(&rcvd_unique_id[rcvd_unique_id_offset], msg.unique_id.data, msg.unique_id.len);
    rcvd_unique_id_offset += msg.unique_id.len;
    
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    ESP_LOGD("DNA_SERVER", "Accumulated %d bytes total (added %d), first_part=%d", 
             rcvd_unique_id_offset, msg.unique_id.len, msg.first_part_of_unique_id);
    
    // Debug: Show what we've accumulated (only occasionally to reduce spam)
    static uint32_t last_uid_debug_ms = 0;
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_uid_debug_ms > 10000) {  // Only log every 10 seconds
        ESP_LOGI("DNA_SERVER", "Processing DNA requests (rcvd_unique_id_offset=%d)", rcvd_unique_id_offset);
        last_uid_debug_ms = now_ms;
    }
#endif

    // respond with the message containing the received unique ID so far, or
    // with the node ID if we successfully allocated one
    uavcan_protocol_dynamic_node_id_Allocation rsp {};
    
    // Initialize the entire structure to avoid encoding uninitialized memory
    memset(&rsp, 0, sizeof(rsp));
    
    // Copy only the received bytes
    memcpy(rsp.unique_id.data, rcvd_unique_id, rcvd_unique_id_offset);
    
    // CRITICAL: Set the length to what we actually received, not the full array size
    // The encoder will only encode rsp.unique_id.len bytes, not the entire array
    rsp.unique_id.len = rcvd_unique_id_offset;
    
    // WORKAROUND for potential bool packing issue on ESP32
    // Ensure first_part_of_unique_id is properly set as 0 or 1
    rsp.first_part_of_unique_id = msg.first_part_of_unique_id ? 1 : 0;
    
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    // Debug: Log response summary (detailed logging removed to reduce spam)
    ESP_LOGD("DNA_SERVER", "Preparing response with %d UID bytes", rsp.unique_id.len);
#endif
    
    // Per DroneCAN spec: Echo the first_part flag from the request when responding to partial allocations
    // Only set to 0 when we're sending the final response with allocated node_id
    rsp.first_part_of_unique_id = msg.first_part_of_unique_id;
    rsp.node_id = 0; // Will be set if allocation succeeds
    
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    ESP_LOGD("DNA_SERVER", "Preparing response: accumulated_uid_len=%d bytes", rcvd_unique_id_offset);
#endif

    // Check if this looks like a complete UID:
    // - Standard UIDs are 16 bytes, but shorter UIDs are valid
    // - If first_part_of_unique_id is false and we have data, treat as complete
    bool uid_looks_complete = (rcvd_unique_id_offset == sizeof(rcvd_unique_id)) ||
                              (!msg.first_part_of_unique_id && rcvd_unique_id_offset > 0);
    
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    ESP_LOGD("DNA_SERVER", "UID completeness check: offset=%d, sizeof=%d, first_part=%d, looks_complete=%d",
             rcvd_unique_id_offset, (int)sizeof(rcvd_unique_id), msg.first_part_of_unique_id, uid_looks_complete);
#endif
    
    if (uid_looks_complete) { // full unique ID received, allocate it!
        // we ignore the preferred node ID as it seems nobody uses the feature
        // and we couldn't guarantee it anyway. we will always remember and
        // re-assign node IDs consistently, so the node could send a status
        // with a particular ID once then switch back to no preference for DNA
        rsp.node_id = db.handle_allocation(rcvd_unique_id, &node_seen);
        
        // For the final allocation response with the echoed UID:
        // If the UID fits in one frame (≤6 bytes), mark it as first_part=1
        // If it's a continuation from multi-frame (>6 bytes), keep first_part=0
        rsp.first_part_of_unique_id = (rcvd_unique_id_offset <= 6) ? 1 : 0;
        
        rcvd_unique_id_offset = 0; // reset state for next allocation
        memset(rcvd_unique_id, 0, sizeof(rcvd_unique_id)); // Clear buffer for next allocation
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
        ESP_LOGD("DNA", "Full UID received, allocated node ID %d", rsp.node_id);
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
        ESP_LOGD("DNA_SERVER", "Full UID received, allocated node ID %d", rsp.node_id);
#endif
        // Log only important allocations (non-zero node IDs)
        if (rsp.node_id != 0) {
            hal.console->printf("DNA: Allocated node ID %d\n", rsp.node_id);
        }
        if (rsp.node_id == 0) { // allocation failed
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
            ESP32_DEBUG_ERROR("DNA: Allocation failed - database full");
#endif
            hal.console->printf("DNA: Allocation failed - database full\n");
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DroneCAN DNA allocation failed; database full");
            // don't send reply with a failed ID in case the allocatee does
            // silly things, though it is technically legal. the allocatee will
            // then time out and try again (though we still won't have an ID!)
            return;
        }
    } else {
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
        ESP_LOGD("DNA_SERVER", "Partial UID received (%d/%d bytes), sending echo", 
                 rcvd_unique_id_offset, (int)sizeof(rcvd_unique_id));
#endif
        hal.console->printf("DNA: Partial UID received (%d/%d bytes), sending echo\n", 
                           rcvd_unique_id_offset, (int)sizeof(rcvd_unique_id));
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    ESP_LOGD("DNA_SERVER", "Broadcasting allocation response, node_id=%d, uid_len=%d", 
             rsp.node_id, rsp.unique_id.len);
    // Dump the full response message
    ESP_LOGD("DNA_SERVER", "Response details:");
    ESP_LOGD("DNA_SERVER", "  node_id: %d", rsp.node_id);
    ESP_LOGD("DNA_SERVER", "  first_part_of_unique_id: %d", rsp.first_part_of_unique_id);
    ESP_LOGD("DNA_SERVER", "  unique_id.len: %d", rsp.unique_id.len);
    ESP_LOGD("DNA_SERVER", "  unique_id data: %02X %02X %02X %02X %02X %02X %02X %02X",
             rsp.unique_id.data[0], rsp.unique_id.data[1], rsp.unique_id.data[2], rsp.unique_id.data[3],
             rsp.unique_id.data[4], rsp.unique_id.data[5], rsp.unique_id.data[6], rsp.unique_id.data[7]);
    if (rsp.unique_id.len > 8) {
        ESP_LOGD("DNA_SERVER", "            cont: %02X %02X %02X %02X %02X %02X %02X %02X",
                 rsp.unique_id.data[8], rsp.unique_id.data[9], rsp.unique_id.data[10], rsp.unique_id.data[11],
                 rsp.unique_id.data[12], rsp.unique_id.data[13], rsp.unique_id.data[14], rsp.unique_id.data[15]);
    }
#endif
    hal.console->printf("DNA: Broadcasting allocation response, node_id=%d, uid_len=%d\n", 
                       rsp.node_id, rsp.unique_id.len);
    
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    ESP_LOGD("DNA_SERVER", "Allocation publisher timeout_ms = %lu", 
             (unsigned long)allocation_pub.get_timeout_ms());
#endif
    
    // Debug: verify response structure before sending
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    uint32_t broadcast_ms = AP_HAL::millis();
    ESP_LOGD("DNA_SERVER", "Broadcasting response at T+%lums (delay from request: %lums)",
             (unsigned long)broadcast_ms, (unsigned long)(broadcast_ms - start_ms));
    ESP_LOGD("DNA_SERVER", "Final response: sizeof=%d, node_id=%d, first_part=%d, uid.len=%d",
             (int)sizeof(rsp), rsp.node_id, rsp.first_part_of_unique_id, rsp.unique_id.len);
    
    // Calculate expected encoded size
    // node_id: 7 bits, first_part: 1 bit, unique_id: variable length array
    int expected_bits = 7 + 1 + 5 + (rsp.unique_id.len * 8); // 7+1 for fields, 5 for array len, then data
    ESP_LOGD("DNA_SERVER", "  Expected encoded bits=%d, bytes=%d", expected_bits, (expected_bits + 7) / 8);
#endif
    
    // Final verification before broadcast
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    // ESP_LOGI("DNA_SERVER", "About to broadcast: node_id=%d (0x%02X), first_part=%d, uid_len=%d", 
    //          rsp.node_id, rsp.node_id, rsp.first_part_of_unique_id, rsp.unique_id.len);
    
    // CRITICAL: Verify the struct values RIGHT before encoding
    // ESP_LOGI("DNA_SERVER", "PRE-ENCODE CHECK:");
    // ESP_LOGI("DNA_SERVER", "  rsp.node_id = %d (should be 0 for echo, non-zero for allocation)", rsp.node_id);
    // ESP_LOGI("DNA_SERVER", "  rsp.first_part_of_unique_id = %d (should match request for echo)", rsp.first_part_of_unique_id);
    // ESP_LOGI("DNA_SERVER", "  rsp.unique_id.len = %d", rsp.unique_id.len);
    // ESP_LOGI("DNA_SERVER", "  rsp.unique_id.data[0-5] = %02X %02X %02X %02X %02X %02X",
    //          rsp.unique_id.data[0], rsp.unique_id.data[1], rsp.unique_id.data[2],
    //          rsp.unique_id.data[3], rsp.unique_id.data[4], rsp.unique_id.data[5]);
    
    // Check struct size and alignment
    ESP_LOGD("DNA_SERVER", "Struct info: sizeof(rsp)=%d, sizeof(bool)=%d, alignof(rsp)=%d",
             (int)sizeof(rsp), (int)sizeof(bool), (int)__alignof__(rsp));
    ESP_LOGD("DNA_SERVER", "Field offsets: node_id=%d, first_part=%d, unique_id=%d",
             (int)offsetof(struct uavcan_protocol_dynamic_node_id_Allocation, node_id),
             (int)offsetof(struct uavcan_protocol_dynamic_node_id_Allocation, first_part_of_unique_id),
             (int)offsetof(struct uavcan_protocol_dynamic_node_id_Allocation, unique_id));
    
    // Log actual struct memory to debug encoding issue
    ESP_LOGD("DNA_SERVER", "Response struct memory dump (first 32 bytes):");
    uint8_t* rsp_bytes = (uint8_t*)&rsp;
    for (int i = 0; i < 32 && i < (int)sizeof(rsp); i += 8) {
        ESP_LOGD("DNA_SERVER", "  [%02d]: %02X %02X %02X %02X %02X %02X %02X %02X",
                 i, rsp_bytes[i], rsp_bytes[i+1], rsp_bytes[i+2], rsp_bytes[i+3],
                 rsp_bytes[i+4], rsp_bytes[i+5], rsp_bytes[i+6], rsp_bytes[i+7]);
    }
    
    // Check if we're sending the expected data
    if (rsp.unique_id.len == 6) {
        ESP_LOGD("DNA_SERVER", "Sending 6-byte UID response for node %d", rsp.node_id);
        // Expected encoded size: 7 bits (node_id) + 1 bit (first_part) + 5 bits (len) + 48 bits (6 bytes) = 61 bits = 8 bytes
        ESP_LOGD("DNA_SERVER", "Expected encoded size = 8 bytes, actual struct size = %d bytes", (int)sizeof(rsp));
    }
#endif
    
    // Add guard against sending duplicate responses too quickly
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    static uint32_t last_response_time = 0;
    static uint8_t last_response_node_id = 0;
    static uint8_t last_uid_hash = 0;
    static uint32_t response_count = 0;
    static uint32_t last_rate_limit_log = 0;
    
    // Calculate simple hash of UID for comparison
    uint8_t uid_hash = 0;
    for (int i = 0; i < rsp.unique_id.len; i++) {
        uid_hash ^= rsp.unique_id.data[i];
    }
    
    uint32_t now_guard = AP_HAL::millis();
    
    // Rate limiting - don't send more than 10 responses per second total
    response_count++;
    if ((now_guard - last_rate_limit_log) >= 1000) {
        if (response_count > 10) {
            ESP_LOGE("DNA_SERVER", "RATE LIMIT: Sent %lu DNA responses in 1 second - TX queue likely flooded!", response_count);
        }
        response_count = 0;
        last_rate_limit_log = now_guard;
    }
    
    // Don't send if we've sent too many recently (prevent TX queue flooding)
    if (response_count > 10) {
        ESP_LOGW("DNA_SERVER", "Dropping DNA response due to rate limit (>10/sec)");
        return;
    }
    
    // Also check for duplicate responses
    if (rsp.node_id == last_response_node_id && 
        uid_hash == last_uid_hash &&
        (now_guard - last_response_time) < 500) {  // Increased to 500ms
        ESP_LOGW("DNA_SERVER", "Suppressing duplicate DNA response for node %d (sent %lums ago)",
                 rsp.node_id, (now_guard - last_response_time));
        return; // Don't send duplicate response within 500ms
    }
    
    last_response_time = now_guard;
    last_response_node_id = rsp.node_id;
    last_uid_hash = uid_hash;
    
    ESP_LOGD("DNA_SERVER", "CALLING allocation_pub.broadcast() for node_id=%d, uid_len=%d", 
             rsp.node_id, rsp.unique_id.len);
    
    
    // Log what we're about to transmit
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    ESP_LOGD("DNA_SERVER", "TRANSMITTING DNA Response:");
    ESP_LOGD("DNA_SERVER", "  node_id: %d (0x%02X)", rsp.node_id, rsp.node_id);
    ESP_LOGD("DNA_SERVER", "  first_part_of_unique_id: %d", rsp.first_part_of_unique_id);
    ESP_LOGD("DNA_SERVER", "  unique_id.len: %d bytes", rsp.unique_id.len);
    
    // Log the complete UID being transmitted
    if (rsp.unique_id.len > 0) {
        char uid_str[128];
        int offset = 0;
        for (int i = 0; i < rsp.unique_id.len && i < 16; i++) {
            offset += snprintf(uid_str + offset, sizeof(uid_str) - offset, "%02X ", rsp.unique_id.data[i]);
        }
        ESP_LOGD("DNA_SERVER", "  unique_id data: %s", uid_str);
    }
    
    // Track what we're actually sending
    static uint32_t tx_sequence = 0;
    tx_sequence++;
    ESP_LOGD("DNA_SERVER", "TX Sequence #%lu: Broadcasting allocation response", tx_sequence);
#else
    hal.console->printf("DNA: TRANSMITTING Response: node_id=%d, first_part=%d, uid_len=%d\n",
                       rsp.node_id, rsp.first_part_of_unique_id, rsp.unique_id.len);
    if (rsp.unique_id.len > 0) {
        hal.console->printf("DNA: TX UID data:");
        for (int i = 0; i < rsp.unique_id.len && i < 16; i++) {
            hal.console->printf(" %02X", rsp.unique_id.data[i]);
        }
        hal.console->printf("\n");
    }
#endif
    
    // Make a defensive copy to ensure struct doesn't get modified during broadcast
    struct uavcan_protocol_dynamic_node_id_Allocation rsp_copy;
    memcpy(&rsp_copy, &rsp, sizeof(rsp_copy));

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    // Log DNA response bytes for manual decoding
    // Throttle to every 10th response to reduce spam
    static uint32_t dna_hex_counter = 0;
    dna_hex_counter++;

    // Only log every 10th response, or if it's an allocation (node_id != 0)
    if ((dna_hex_counter % 10) == 0 || rsp_copy.node_id != 0) {
        // Log the response structure using ESP_LOGI for console output
        ESP_LOGI("DNA_HEX", "DNA RSP: node=%d first=%d uid_len=%d",
                 rsp_copy.node_id,
                 rsp_copy.first_part_of_unique_id ? 1 : 0,
                 rsp_copy.unique_id.len);

        // Log first 8 bytes of UID for manual decode
        if (rsp_copy.unique_id.len > 0) {
            ESP_LOGI("DNA_HEX", "DNA UID: %02X %02X %02X %02X %02X %02X %02X %02X",
                     rsp_copy.unique_id.data[0], rsp_copy.unique_id.data[1],
                     rsp_copy.unique_id.data[2], rsp_copy.unique_id.data[3],
                     rsp_copy.unique_id.data[4], rsp_copy.unique_id.data[5],
                     rsp_copy.unique_id.data[6], rsp_copy.unique_id.data[7]);

            // If more than 8 bytes, log the rest
            if (rsp_copy.unique_id.len > 8 && rsp_copy.unique_id.len <= 16) {
                ESP_LOGI("DNA_HEX", "DNA UID2: %02X %02X %02X %02X %02X %02X %02X %02X",
                         rsp_copy.unique_id.data[8],
                         (rsp_copy.unique_id.len > 9) ? rsp_copy.unique_id.data[9] : 0,
                         (rsp_copy.unique_id.len > 10) ? rsp_copy.unique_id.data[10] : 0,
                         (rsp_copy.unique_id.len > 11) ? rsp_copy.unique_id.data[11] : 0,
                         (rsp_copy.unique_id.len > 12) ? rsp_copy.unique_id.data[12] : 0,
                         (rsp_copy.unique_id.len > 13) ? rsp_copy.unique_id.data[13] : 0,
                         (rsp_copy.unique_id.len > 14) ? rsp_copy.unique_id.data[14] : 0,
                         (rsp_copy.unique_id.len > 15) ? rsp_copy.unique_id.data[15] : 0);
            }
        }
    }
#endif

    allocation_pub.broadcast(rsp_copy, false); // never publish allocation message with CAN FD
    
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    ESP_LOGD("DNA_SERVER", "RETURNED from allocation_pub.broadcast() - TX should be queued");
    
    // Log TX status
    ESP_LOGD("DNA_SERVER", "TX Status: Response queued for transmission to anonymous node");
    if (rsp.node_id != 0) {
        ESP_LOGD("DNA_SERVER", "TX Result: Allocated node ID %d to device", rsp.node_id);
    } else {
        ESP_LOGD("DNA_SERVER", "TX Result: Echoing partial UID (%d bytes)", rsp.unique_id.len);
    }
#else
    hal.console->printf("DNA: Response broadcast queued for transmission\n");
#endif
    
    // Immediately push the frame to CAN bus to reduce latency
    _canard_iface.processTx(false);
    
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    ESP_LOGD("DNA_SERVER", "Called processTx() to send frame immediately");
    
    // Log that we sent a response to help debug continuous sending
    static uint32_t last_broadcast_log = 0;
    static uint32_t broadcast_count = 0;
    broadcast_count++;
    uint32_t now_bc = AP_HAL::millis();
    if (now_bc - last_broadcast_log > 1000) {
        if (broadcast_count > 10) {  // Only warn if excessive
            ESP_LOGW("DNA_SERVER", "High DNA activity: %lu responses/sec", broadcast_count);
        }
        broadcast_count = 0;
        last_broadcast_log = now_bc;
    }
#endif
}

//report the server state, along with failure message if any
bool AP_DroneCAN_DNA_Server::prearm_check(char* fail_msg, uint8_t fail_msg_len) const
{
    switch (server_state) {
    case HEALTHY:
        return true;
    case DUPLICATE_NODES: {
        if (_ap_dronecan.option_is_set(AP_DroneCAN::Options::DNA_IGNORE_DUPLICATE_NODE)) {
            // ignore error
            return true;
        }
        snprintf(fail_msg, fail_msg_len, "Duplicate Node %s../%d!", fault_node_name, fault_node_id);
        return false;
    }
    case NODE_STATUS_UNHEALTHY: {
        if (_ap_dronecan.option_is_set(AP_DroneCAN::Options::DNA_IGNORE_UNHEALTHY_NODE)) {
            // ignore error
            return true;
        }
        snprintf(fail_msg, fail_msg_len, "Node %d unhealthy!", fault_node_id);
        return false;
    }
    }
    // should never get; compiler should enforce all server_states are covered
    return false;
}

#endif //HAL_NUM_CAN_IFACES
#endif //HAL_ENABLE_DRONECAN_DRIVERS
