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
#include "AP_DroneCAN.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
// Shared node name storage for logging across functions
// UAVCAN standard allows node names up to 80 characters (uint8[<=80])
static char g_node_names[128][81] = {};  // 80 chars + null terminator
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

// Storage format is platform-dependent:
// ESP32: Full 16-byte UIDs (no hash) for debugging, MAX_NODE_ID=62 to fit 1KB storage
//        Also avoids MAVLink component ID collisions (camera=100, servo=140+, gimbal=154+)
//        since PARAM_EXT bridge uses node_id == component_id per MAVLink UAVCAN spec.
// Others: Upstream hash+CRC format, MAX_NODE_ID=125
#define NODERECORD_MAGIC_LEN 2 // uint16_t
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
#define NODERECORD_MAGIC 0xAC02  // Different magic to force reset on format change
#define MAX_NODE_ID    62        // 62*16+2 = 994 bytes fits 1KB storage
#else
#define NODERECORD_MAGIC 0xAC01
#define MAX_NODE_ID    125       // 125*7+2 = 877 bytes fits 1KB storage
#endif
#define NODERECORD_LOC(node_id) ((node_id * sizeof(NodeRecord)) + NODERECORD_MAGIC_LEN)

#define debug_dronecan(level_debug, fmt, args...) do { AP::can().log_text(level_debug, "DroneCAN", fmt, ##args); } while (0)

// database is currently shared by all DNA servers
AP_DroneCAN_DNA_Server::Database AP_DroneCAN_DNA_Server::db;

// initialize database (storage accessor is always replaced with the one supplied)
void AP_DroneCAN_DNA_Server::Database::init(StorageAccess *storage_)
{
    // storage size must be synced with StorageCANDNA entry in StorageManager.cpp
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    // ESP32: 62 nodes * 16 bytes + 2 magic = 994 bytes
    static_assert(NODERECORD_LOC(MAX_NODE_ID) + sizeof(NodeRecord) <= 1024, "DNA storage too small");
#else
    // Upstream: 125 nodes * 7 bytes + 2 magic = 877 bytes (uses MAX_NODE_ID+1 to include slot 0)
    static_assert(NODERECORD_LOC(MAX_NODE_ID+1) <= 1024, "DNA storage too small");
#endif

    // might be called from multiple threads if multiple servers use the same database
    WITH_SEMAPHORE(sem);

    storage = storage_; // use supplied accessor

    // validate magic number
    uint16_t magic = storage->read_uint16(0);

    debug_dronecan(AP_CANManager::LOG_DEBUG, "DNA: magic=0x%04x (expected 0x%04x)", magic, NODERECORD_MAGIC);

    if (magic != NODERECORD_MAGIC) {
        debug_dronecan(AP_CANManager::LOG_WARNING, "DNA: invalid magic, resetting database");
        reset(); // resetting the database will put the magic back
    }

    // check and note each possible node ID's registration's presence
    for (uint8_t i = 1; i <= MAX_NODE_ID; i++) {
        if (check_registration(i)) {
            node_registered.set(i);
        }
    }

    // Mark database as initialized - operations are now safe
    initialized = true;
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
    uint16_t magic = NODERECORD_MAGIC;
    debug_dronecan(AP_CANManager::LOG_INFO, "DNA: writing magic 0x%04X to storage", NODERECORD_MAGIC);
    if (!storage->write_block(0, &magic, sizeof(magic))) {
        debug_dronecan(AP_CANManager::LOG_ERROR, "DNA: failed to write magic number during reset");
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "DroneCAN DNA storage write failed");
    } else {
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
        // Verify the write by reading back
        uint16_t verify_magic = storage->read_uint16(0);
        if (verify_magic != NODERECORD_MAGIC) {
            debug_dronecan(AP_CANManager::LOG_ERROR, "DNA: magic readback mismatch 0x%04X != 0x%04X",
                           verify_magic, NODERECORD_MAGIC);
        }

        // Force immediate flush to flash
        // ESP32 Storage uses delayed writes via _timer_tick(). The magic number write
        // goes to RAM buffer but won't persist to flash until timer tick runs.
        // _timer_tick() only flushes ONE dirty line per call. After clearing all node
        // records, many lines are dirty. We call it enough times to flush everything.
        const uint16_t max_flush_cycles = 1000; // More than enough for 512 lines
        for (uint16_t i = 0; i < max_flush_cycles; i++) {
            hal.storage->_timer_tick();
            hal.scheduler->delay_microseconds(100);
        }

        if (!hal.storage->healthy()) {
            debug_dronecan(AP_CANManager::LOG_ERROR, "DNA: storage unhealthy after flush");
        }
#endif
    }
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
bool AP_DroneCAN_DNA_Server::Database::handle_node_info(uint8_t source_node_id, const uint8_t unique_id[], const Bitmask<128> *healthy_mask)
{
    WITH_SEMAPHORE(sem);

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    // ESP32: UID-based conflict resolution with takeover support
    uint8_t registered_node_id = find_node_id(unique_id, 16);

    if (registered_node_id == 0) {
        // UID not in database - new node
        ESP_LOGI("DNA_DB", "UID not found in DB (new node), registering node %d, UID=%02X%02X%02X%02X%02X%02X...",
                 source_node_id,
                 unique_id[0], unique_id[1], unique_id[2],
                 unique_id[3], unique_id[4], unique_id[5]);
        register_uid(source_node_id, unique_id, 16);
        return false;
    }

    if (registered_node_id == source_node_id) {
        return false;
    }

    // UID is registered to a DIFFERENT node ID -- allow takeover
    bool rightful_owner_active = healthy_mask && healthy_mask->get(registered_node_id);
    if (rightful_owner_active) {
        ESP_LOGW("DNA_DB", "UID conflict: Node %d using ID registered to active node %d",
                 source_node_id, registered_node_id);
    } else {
        ESP_LOGW("DNA_DB", "Allowing takeover: Node %d (inactive) -> Node %d (active)",
                 registered_node_id, source_node_id);
    }

    NodeRecord empty_record;
    memset(&empty_record, 0, sizeof(empty_record));
    write_record(empty_record, registered_node_id);
    node_registered.clear(registered_node_id);
    register_uid(source_node_id, unique_id, 16);
    return false;
#else
    // Non-ESP32: upstream behavior -- report duplicates, register unknowns
    (void)healthy_mask;
    if (is_registered(source_node_id)) {
        if (source_node_id != find_node_id(unique_id, 16)) {
            return true; // duplicate: this node ID belongs to a different UID
        }
    } else {
        register_uid(source_node_id, unique_id, 16);
    }
    return false;
#endif
}

// handle the allocation message. returns the allocated node ID, or 0 if allocation failed
uint8_t AP_DroneCAN_DNA_Server::Database::handle_allocation(const uint8_t unique_id[], uint8_t uid_len, const Bitmask<128> *seen_mask)
{
    WITH_SEMAPHORE(sem);

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    // Reject operations before database is initialized
    if (!initialized) {
        ESP_LOGW("DNA_DB", "Allocation requested before database initialized - ignoring");
        return 0;
    }
#endif

    uint8_t resp_node_id = find_node_id(unique_id, uid_len);

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    if (resp_node_id == 0) {
        ESP_LOGD("DNA_DB", "Allocation: UID not in DB, UID=%02X%02X%02X%02X%02X%02X... (may be partial)",
                 unique_id[0], unique_id[1], unique_id[2],
                 unique_id[3], unique_id[4], unique_id[5]);
    }
#endif

    if (resp_node_id == 0) {
        // find free node ID, starting at the max as prescribed by the standard
        resp_node_id = MAX_NODE_ID;
        while (resp_node_id > 0) {
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
            // ESP32: also check seen_mask to avoid allocating IDs used by active nodes
            if (!node_registered.get(resp_node_id) &&
                !(seen_mask && seen_mask->get(resp_node_id))) {
                break;
            }
#else
            // Non-ESP32: upstream behavior -- only check registered
            (void)seen_mask;
            if (!node_registered.get(resp_node_id)) {
                break;
            }
#endif
            resp_node_id--;
        }

        if (resp_node_id != 0) {
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
            ESP_LOGD("DNA_DB", "Assigning new node ID %d, UID=%02X%02X%02X%02X%02X%02X... (len=%d)", resp_node_id,
                     unique_id[0], unique_id[1], unique_id[2],
                     unique_id[3], unique_id[4], unique_id[5], uid_len);
#endif
            create_registration(resp_node_id, unique_id, uid_len);
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DroneCAN DNA allocation failed - database full");
        }
    }

    return resp_node_id; // will be 0 if not found and not created
}

// retrieve node ID that matches the given unique ID. returns 0 if not found
uint8_t AP_DroneCAN_DNA_Server::Database::find_node_id(const uint8_t unique_id[], uint8_t size)
{
    NodeRecord record;

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    // Direct UID comparison (full 16-byte storage)
    uint8_t search_len = (size > 16) ? 16 : size;

    for (int i = MAX_NODE_ID; i > 0; i--) {
        if (node_registered.get(i)) {
            read_record(record, i);
            if (memcmp(record.uid, unique_id, search_len) == 0) {
                ESP_LOGD("DNA_DB", "find_node_id: Found match at node %d (search_len=%d)", i, search_len);
                return i;
            }
        }
    }
#else
    // Hash-based comparison (upstream format)
    NodeRecord cmp_record;
    compute_uid_hash(cmp_record, unique_id, size);

    for (int i = MAX_NODE_ID; i > 0; i--) {
        if (node_registered.get(i)) {
            read_record(record, i);
            if (memcmp(record.uid_hash, cmp_record.uid_hash, sizeof(NodeRecord::uid_hash)) == 0) {
                return i;
            }
        }
    }
#endif
    return 0; // not found
}

#if CONFIG_HAL_BOARD != HAL_BOARD_ESP32
// fill the given record with the hash of the given unique ID (upstream format)
void AP_DroneCAN_DNA_Server::Database::compute_uid_hash(NodeRecord &record, const uint8_t unique_id[], uint8_t size) const
{
    uint64_t hash = FNV_1_OFFSET_BASIS_64;
    hash_fnv_1a(size, unique_id, &hash);

    // xor-folding per http://www.isthe.com/chongo/tech/comp/fnv/
    hash = (hash>>56) ^ (hash&(((uint64_t)1<<56)-1));

    for (uint8_t i=0; i<6; i++) {
        record.uid_hash[i] = (hash >> (8*i)) & 0xff;
    }
}
#endif

// register a given unique ID to a given node ID, deleting any existing registration for the unique ID
void AP_DroneCAN_DNA_Server::Database::register_uid(uint8_t node_id, const uint8_t unique_id[], uint8_t size)
{
    uint8_t prev_node_id = find_node_id(unique_id, size); // have we registered this unique ID under a different node ID?
    if (prev_node_id != 0) {
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
        ESP_LOGD("DNA_DB", "register_uid: Deleting old registration at node %d before creating new at node %d", prev_node_id, node_id);
#endif
        delete_registration(prev_node_id); // yes, delete old node ID's registration
    }
    // overwrite an existing registration with this node ID, if any
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    if (prev_node_id == 0 && node_registered.get(node_id)) {
        ESP_LOGD("DNA_DB", "register_uid: Updating registration for node %d (e.g., partial→full UID hash)", node_id);
    }
#endif
    create_registration(node_id, unique_id, size);
}

// create the registration for the given node ID and set its record's unique ID
void AP_DroneCAN_DNA_Server::Database::create_registration(uint8_t node_id, const uint8_t unique_id[], uint8_t size)
{
    NodeRecord record;
    memset(&record, 0, sizeof(record));

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    // Store full UID directly
    uint8_t copy_size = (size > 16) ? 16 : size;
    memcpy(record.uid, unique_id, copy_size);
#else
    // Compute hash and CRC (upstream format)
    compute_uid_hash(record, unique_id, size);
    record.crc = crc_crc8(record.uid_hash, sizeof(record.uid_hash));
#endif

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

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    // Check if UID is all zeros (no registration)
    for (uint8_t i = 0; i < sizeof(record.uid); i++) {
        if (record.uid[i] != 0) {
            return true;
        }
    }
    return false;
#else
    // Check hash CRC (upstream format)
    uint8_t empty_uid[sizeof(NodeRecord::uid_hash)] {};
    uint8_t crc = crc_crc8(record.uid_hash, sizeof(record.uid_hash));
    if (crc == record.crc && memcmp(&record.uid_hash[0], &empty_uid[0], sizeof(empty_uid)) != 0) {
        return true;
    }
    return false;
#endif
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
    for (int i = 0; i < 16; i++) {
        if (record.uid[i] != 0) {
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

    // Read existing record and only write if different (preserve flash write lifetime)
    NodeRecord existing_record;
    storage->read_block(&existing_record, NODERECORD_LOC(node_id), sizeof(NodeRecord));

    if (memcmp(&existing_record, &record, sizeof(NodeRecord)) == 0) {
        // Record unchanged, skip write to preserve flash lifetime
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
        ESP_LOGD("DNA_SERVER", "Skipping write to node %d - record unchanged", node_id);
#endif
        return;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    // Check if we're writing a registration (non-zero UID) or clearing (all-zero UID)
    bool is_clearing = true;
    for (uint8_t i = 0; i < 16; i++) {
        if (record.uid[i] != 0) {
            is_clearing = false;
            break;
        }
    }
    if (is_clearing) {
        debug_dronecan(AP_CANManager::LOG_DEBUG, "DNA: clearing node %d record", node_id);
    } else {
        debug_dronecan(AP_CANManager::LOG_DEBUG, "DNA: writing node %d record", node_id);
    }
#endif

    if (!storage->write_block(NODERECORD_LOC(node_id), &record, sizeof(NodeRecord))) {
        debug_dronecan(AP_CANManager::LOG_ERROR, "DNA_DB CRITICAL: Failed to write record for node %d to storage", node_id);
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DroneCAN DNA storage write failed for node %d", node_id);
    }
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
    debug_dronecan(AP_CANManager::LOG_DEBUG, "DNA: init node_id=%d", node_id);

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
    
    debug_dronecan(AP_CANManager::LOG_INFO, "DNA: server initialized, self_node_id=%d", self_node_id);

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

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    // Track verification failures per node (ESP32 debug diagnostics only)
    static uint32_t verification_attempt_count[128] = {0};
    static uint32_t verification_start_time[128] = {0};
#endif

    //Check if we got acknowledgement from previous request
    //except for requests using our own node_id
    if (curr_verifying_node == self_node_id) {
        nodeInfo_resp_rcvd = true;
    }

    if (!nodeInfo_resp_rcvd) {
        /* Node failed to respond - could be disconnected or conflicting ID */
        node_verified.clear(curr_verifying_node);

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
        // ESP32 diagnostics: mark node as unhealthy if it stops responding to GetNodeInfo
        // Not enabled on other platforms as test_iface (SITL) doesn't respond to GetNodeInfo
        if (node_healthy.get(curr_verifying_node)) {
            node_healthy.clear(curr_verifying_node);
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "DroneCAN Node %d: No response - possibly OFFLINE",
                          curr_verifying_node);
        }
#endif
    } else {
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
        // Reset failure count and start time on successful verification
        verification_attempt_count[curr_verifying_node] = 0;
        verification_start_time[curr_verifying_node] = 0;
#endif
    }

    last_verification_request = now;
    //Find the next registered Node ID to be verified.
    for (uint8_t i = 0; i <= MAX_NODE_ID; i++) {
        curr_verifying_node = (curr_verifying_node + 1) % (MAX_NODE_ID + 1);
        if ((curr_verifying_node == self_node_id) || (curr_verifying_node == 0)) {
            continue;
        }
        if (db.is_registered(curr_verifying_node)) {
            break;
        }
    }
    if (db.is_registered(curr_verifying_node)) {
        // Don't try to verify ourselves
        if (curr_verifying_node == self_node_id) {
            node_verified.set(curr_verifying_node);
            node_healthy.set(curr_verifying_node);
        } else {
            uavcan_protocol_GetNodeInfoRequest request;
            node_info_client.request(curr_verifying_node, request);
            nodeInfo_resp_rcvd = false;

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
            // Track and warn about nodes that repeatedly fail verification
            if (verification_attempt_count[curr_verifying_node] == 0) {
                verification_start_time[curr_verifying_node] = now;
            }
            verification_attempt_count[curr_verifying_node]++;

            if (verification_attempt_count[curr_verifying_node] % 10 == 0 &&
                verification_attempt_count[curr_verifying_node] > 0) {
                uint32_t duration_ms = now - verification_start_time[curr_verifying_node];
                float duration_sec = duration_ms / 1000.0f;
                debug_dronecan(AP_CANManager::LOG_WARNING, "DNA: node %d: %lu GetNodeInfo requests over %.1fs with no response",
                               curr_verifying_node,
                               (unsigned long)verification_attempt_count[curr_verifying_node],
                               duration_sec);
            }
#endif
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
    
    // Handle health state changes with severity-based reporting
    const bool is_operational = (msg.mode == UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL);
    const bool is_healthy = (msg.health == UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK);
#if AP_DRONECAN_MAVLINK_REPORTING_ENABLED || CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    const bool was_healthy = node_healthy.get(transfer.source_node_id);
#endif
#if AP_DRONECAN_MAVLINK_REPORTING_ENABLED
    const bool first_contact = !node_seen.get(transfer.source_node_id);
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    // Debug: Track node_healthy changes for troubleshooting LED flipping
    static bool first_log[128] = {true};
#endif
    bool ignore_unhealthy = _ap_dronecan.option_is_set(AP_DroneCAN::Options::DNA_IGNORE_UNHEALTHY_NODE);

#if AP_DRONECAN_MAVLINK_REPORTING_ENABLED
    // Send UAVCAN_NODE_STATUS MAVLink message on first contact or health change only
    // (not every heartbeat, which would flood MAVLink channels)
    if (first_contact || (was_healthy != is_healthy)) {
        send_node_status_mavlink(transfer.source_node_id, msg);
    }
#endif

    if ((!is_healthy || !is_operational) && !ignore_unhealthy) {

#if AP_DRONECAN_MAVLINK_REPORTING_ENABLED
        if (was_healthy) {
            report_node_health_change(transfer.source_node_id, msg.health, msg.mode, false);
        }
#endif

        fault_node_id = transfer.source_node_id;
        server_state = NODE_STATUS_UNHEALTHY;
        node_healthy.clear(transfer.source_node_id);

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
        // Log when node_healthy changes from true to false
        if (was_healthy || first_log[transfer.source_node_id]) {
            ESP_LOGW("DNA_HEALTH", "Node %d: node_healthy CLEARED (was=%d, health=%d, mode=%d, operational=%d)",
                     transfer.source_node_id, was_healthy, is_healthy, msg.mode, is_operational);
            first_log[transfer.source_node_id] = false;
        }
#endif
    } else {
#if AP_DRONECAN_MAVLINK_REPORTING_ENABLED
        // Report recovery only if previously seen (not first contact)
        if (!was_healthy && is_healthy && is_operational && !first_contact) {
            report_node_health_change(transfer.source_node_id, msg.health, msg.mode, true);
        }
#endif

        node_healthy.set(transfer.source_node_id);
        if (node_healthy == node_verified) {
            server_state = HEALTHY;
        }

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
        // Log when node_healthy changes from false to true
        if (!was_healthy || first_log[transfer.source_node_id]) {
            ESP_LOGW("DNA_HEALTH", "Node %d: node_healthy SET (was=%d, health=%d, mode=%d, operational=%d, ignore_opt=%d)",
                     transfer.source_node_id, was_healthy, is_healthy, msg.mode, is_operational, ignore_unhealthy);
            first_log[transfer.source_node_id] = false;
        }
#endif
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
            node_info_client.request(transfer.source_node_id, request);
        }
    }
    //Add node to seen list if not seen before
    node_seen.set(transfer.source_node_id);
}

#if AP_DRONECAN_MAVLINK_REPORTING_ENABLED
void AP_DroneCAN_DNA_Server::send_node_status_mavlink(uint8_t node_id, const uavcan_protocol_NodeStatus& msg)
{
#if HAL_GCS_ENABLED
    // Pack the message with the node ID as the component ID (per MAVLink UAVCAN spec)
    mavlink_message_t mavlink_msg;
    mavlink_msg_uavcan_node_status_pack(
        mavlink_system.sysid,
        node_id,  // Source component = node ID (1:1 per MAVLink UAVCAN spec)
        &mavlink_msg,
        AP_HAL::micros64(),
        msg.uptime_sec,
        msg.health,
        msg.mode,
        msg.sub_mode,
        msg.vendor_specific_status_code
    );

    // Send to all active MAVLink channels using lock protocol
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &mavlink_msg);
    for (uint8_t i=0; i<gcs().num_gcs(); i++) {
        GCS_MAVLINK *c = gcs().chan(i);
        if (c == nullptr || c->is_private() || !c->is_active()) {
            continue;
        }
        comm_send_lock((mavlink_channel_t)i, len);
        comm_send_buffer((mavlink_channel_t)i, buf, len);
        comm_send_unlock((mavlink_channel_t)i);
    }
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
#if HAL_GCS_ENABLED
    // Prepare node name with null termination
    char node_name[81];
    size_t name_len = MIN(msg.name.len, sizeof(node_name) - 1);
    memcpy(node_name, msg.name.data, name_len);
    node_name[name_len] = '\0';

    // Pack the message with the node ID as the component ID (per MAVLink UAVCAN spec)
    mavlink_message_t mavlink_msg;
    mavlink_msg_uavcan_node_info_pack(
        mavlink_system.sysid,
        node_id,  // Source component = node ID (1:1 per MAVLink UAVCAN spec)
        &mavlink_msg,
        AP_HAL::micros64(),
        msg.status.uptime_sec,
        node_name,
        msg.hardware_version.major,
        msg.hardware_version.minor,
        msg.hardware_version.unique_id,
        msg.software_version.major,
        msg.software_version.minor,
        msg.software_version.vcs_commit
    );

    // Send to all active MAVLink channels using lock protocol
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &mavlink_msg);
    for (uint8_t i=0; i<gcs().num_gcs(); i++) {
        GCS_MAVLINK *c = gcs().chan(i);
        if (c == nullptr || c->is_private() || !c->is_active()) {
            continue;
        }
        comm_send_lock((mavlink_channel_t)i, len);
        comm_send_buffer((mavlink_channel_t)i, buf, len);
        comm_send_unlock((mavlink_channel_t)i);
    }

    // Also send a text message for human-readable notification
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "DroneCAN Node %d: %s v%d.%d online",
                  node_id, node_name,
                  msg.software_version.major, msg.software_version.minor);
#endif
}
#endif  // AP_DRONECAN_MAVLINK_REPORTING_ENABLED

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
    
#if AP_DRONECAN_MAVLINK_REPORTING_ENABLED
    // Send UAVCAN_NODE_INFO MAVLink message for node discovery
    send_node_info_mavlink(transfer.source_node_id, rsp);
#endif

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
    // Log GetNodeInfo responses at DEBUG level
    ESP_LOGD("DNA_SERVER", "Node %d GetNodeInfo: duplicate=%s, name='%s'",
             transfer.source_node_id, duplicate ? "YES" : "NO", rsp.name.data);
#endif
    if (duplicate) {
        if (!_ap_dronecan.option_is_set(AP_DroneCAN::Options::DNA_IGNORE_DUPLICATE_NODE)) {
            /* This is a device with node_id already registered
            for another device */
            server_state = DUPLICATE_NODES;
            fault_node_id = transfer.source_node_id;
            memcpy(fault_node_name, rsp.name.data, sizeof(fault_node_name));
            debug_dronecan(AP_CANManager::LOG_ERROR, "DNA_SERVER ERROR: DUPLICATE NODE DETECTED on Node %d", transfer.source_node_id);
        }
    } else {
        //Verify as well
        node_verified.set(transfer.source_node_id);
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
        // Store node name for later use in NodeStatus logging
        const char* old_name = g_node_names[transfer.source_node_id][0] ? g_node_names[transfer.source_node_id] : "none";
        const char* new_name = (const char*)rsp.name.data;

        // Warn if name is changing (possible UID conflict or node re-registration)
        if (g_node_names[transfer.source_node_id][0] && strcmp(old_name, new_name) != 0) {
            ESP_LOGW("DNA_SERVER", "Node %d name CHANGED: '%s' -> '%s' (possible UID conflict!)",
                     transfer.source_node_id, old_name, new_name);
        }

        strncpy(g_node_names[transfer.source_node_id], new_name, 80);
        g_node_names[transfer.source_node_id][80] = 0;

        ESP_LOGD("DNA_SERVER", "Node %d VERIFIED: name='%s'", transfer.source_node_id, g_node_names[transfer.source_node_id]);
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
#endif

    if (transfer.source_node_id != 0) {
        debug_dronecan(AP_CANManager::LOG_WARNING, "DNA: ignoring allocation from non-anonymous node %d", transfer.source_node_id);
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
        // not first part but we are expecting one, reject per DroneCAN standard
        return;
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

    // Full 16-byte UID required before allocation.
    // The DNA protocol client expects unique_id.len == 16 in the response to
    // recognize a completed allocation. Allocating with fewer bytes causes the
    // client to treat the response as a partial echo, creating an infinite loop.
    bool uid_looks_complete = (rcvd_unique_id_offset == sizeof(rcvd_unique_id));

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    ESP_LOGD("DNA_SERVER", "UID completeness check: offset=%d, required=%d, looks_complete=%d",
             rcvd_unique_id_offset, (int)sizeof(rcvd_unique_id), uid_looks_complete);
#endif

    if (uid_looks_complete) { // full unique ID received, allocate it!
        // we ignore the preferred node ID as it seems nobody uses the feature
        // and we couldn't guarantee it anyway. we will always remember and
        // re-assign node IDs consistently, so the node could send a status
        // with a particular ID once then switch back to no preference for DNA
        rsp.node_id = db.handle_allocation(rcvd_unique_id, rcvd_unique_id_offset, &node_seen);

        // For the final allocation response with the echoed UID:
        // Since we require ≥12 bytes, this is always a multi-frame transfer
        // Set first_part=0 to indicate this is a continuation/final frame
        rsp.first_part_of_unique_id = 0;
        
        rcvd_unique_id_offset = 0; // reset state for next allocation
        memset(rcvd_unique_id, 0, sizeof(rcvd_unique_id)); // Clear buffer for next allocation
        if (rsp.node_id != 0) {
            debug_dronecan(AP_CANManager::LOG_INFO, "DNA: allocated node ID %d", rsp.node_id);
        }
        if (rsp.node_id == 0) { // allocation failed
            debug_dronecan(AP_CANManager::LOG_ERROR, "DNA: allocation failed - database full");
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DroneCAN DNA allocation failed; database full");
            // don't send reply with a failed ID in case the allocatee does
            // silly things, though it is technically legal. the allocatee will
            // then time out and try again (though we still won't have an ID!)
            return;
        }
    } else {
        debug_dronecan(AP_CANManager::LOG_DEBUG, "DNA: partial UID (%d bytes), sending echo", rcvd_unique_id_offset);
    }

    debug_dronecan(AP_CANManager::LOG_DEBUG, "DNA: broadcasting allocation response, node_id=%d, uid_len=%d",
                   rsp.node_id, rsp.unique_id.len);

    allocation_pub.broadcast(rsp, false); // never publish allocation message with CAN FD

    // Immediately push the frame to CAN bus to reduce latency
    _canard_iface.processTx(false);
}

// Request node info for all seen nodes (for MAV_CMD_UAVCAN_GET_NODE_INFO)
void AP_DroneCAN_DNA_Server::request_all_node_info()
{
    // NOTE: The Client class can only track ONE outstanding request at a time
    // (it has a single server_node_id and transfer_id member). If we send multiple
    // requests in rapid succession, only the LAST request's response will be accepted.
    //
    // Solution: Mark all nodes as unverified and trigger immediate verification.
    // The verify_nodes() function will send ONE request per 5-second cycle.
    // This ensures responses are properly tracked and UAVCAN_NODE_INFO messages
    // are sent to the GCS as each response arrives.

    // Mark all seen nodes as unverified to ensure fresh requests are sent
    for (uint8_t i = 1; i <= MAX_NODE_ID; i++) {
        if (node_seen.get(i) && i != self_node_id) {
            node_verified.clear(i);
        }
    }

    // Force immediate verification cycle by resetting the timer
    // This will cause verify_nodes() to run on the next main loop iteration
    last_verification_request = 0;

    uint8_t node_count = node_seen.count() - (node_seen.get(self_node_id) ? 1 : 0);
    debug_dronecan(AP_CANManager::LOG_INFO, "DNA: triggered verification for %d nodes", node_count);
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

#endif //HAL_ENABLE_DRONECAN_DRIVERS
