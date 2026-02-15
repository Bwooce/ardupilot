#pragma once
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>

#if HAL_ENABLE_DRONECAN_DRIVERS
#include <AP_Common/Bitmask.h>
#include <StorageManager/StorageManager.h>
#include <AP_CANManager/AP_CANManager.h>
#include <canard/publisher.h>
#include <canard/subscriber.h>
#include <canard/service_client.h>
#include "AP_Canard_iface.h"
#include <dronecan_msgs.h>

class AP_DroneCAN;
//Forward declaring classes
class AP_DroneCAN_DNA_Server
{
    StorageAccess storage;

    // ESP32 stores full 16-byte UIDs for debugging/diagnostics; fits 62 nodes in 1KB
    // Other platforms use upstream hash+CRC format; fits 125 nodes in 1KB
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    struct PACKED NodeRecord {
        uint8_t uid[16];  // Full 16-byte unique ID (no hash, no CRC)
    };
#else
    struct NodeRecord {
        uint8_t uid_hash[6];
        uint8_t crc;
    };
#endif

    /*
     * For each node ID (1 through MAX_NODE_ID), the database can have one
     * registration for it. Each registration consists of a NodeRecord which
     * contains the (hash of the) unique ID reported by that node ID. Other
     * info could be added to the registration in the future.
     *
     * Physically, the database is stored as a header and format version,
     * followed by an array of NodeRecords indexed by node ID. If a particular
     * NodeRecord has an all-zero unique ID hash or an invalid CRC, then that
     * node ID isn't considerd to have a registration.
     *
     * The database has public methods which handle the server behavior for the
     * relevant message. The methods can be used by multiple servers in
     * different threads, so each holds a lock for its duration.
     */
    class Database {
    public:
        Database() {};

        // initialize database (storage accessor is always replaced with the one supplied)
        void init(StorageAccess *storage_);

        // remove all registrations from the database
        void reset();

        // return true if the given node ID is registered
        bool is_registered(uint8_t node_id) {
            return node_registered.get(node_id);
        }

        // handle initializing the server with its own node ID and unique ID
        void init_server(uint8_t own_node_id, const uint8_t own_unique_id[], uint8_t own_unique_id_len);

        // handle processing the node info message. returns true if from a duplicate node
        bool handle_node_info(uint8_t source_node_id, const uint8_t unique_id[], const Bitmask<128> *healthy_mask);

        // handle the allocation message. returns the allocated node ID, or 0 if allocation failed
        uint8_t handle_allocation(const uint8_t unique_id[], uint8_t uid_len, const Bitmask<128> *seen_mask);

    private:
        // retrieve node ID that matches the given unique ID. returns 0 if not found
        uint8_t find_node_id(const uint8_t unique_id[], uint8_t size);

#if CONFIG_HAL_BOARD != HAL_BOARD_ESP32
        // fill the given record with the hash of the given unique ID (upstream format)
        void compute_uid_hash(NodeRecord &record, const uint8_t unique_id[], uint8_t size) const;
#endif

        // register a given unique ID to a given node ID, deleting any existing registration for the unique ID
        void register_uid(uint8_t node_id, const uint8_t unique_id[], uint8_t size);

        // create the registration for the given node ID and set its record's unique ID
        void create_registration(uint8_t node_id, const uint8_t unique_id[], uint8_t size);

        // delete the given node ID's registration
        void delete_registration(uint8_t node_id);

        // return true if the given node ID has a registration
        bool check_registration(uint8_t node_id);

        // read the given node ID's registration's record
        void read_record(NodeRecord &record, uint8_t node_id);

        // write the given node ID's registration's record
        void write_record(const NodeRecord &record, uint8_t node_id);

        // bitmasks containing a status for each possible node ID (except 0 and > MAX_NODE_ID)
        Bitmask<128> node_registered; // have a registration for this node ID

        StorageAccess *storage;
        HAL_Semaphore sem;
        bool initialized;  // true after init() validates/resets database
    };

    static Database db;

    enum ServerState {
        NODE_STATUS_UNHEALTHY = -5,
        DUPLICATE_NODES = -2,
        HEALTHY = 0
    };

    uint32_t last_verification_request;
    uint8_t curr_verifying_node;
    uint8_t self_node_id;
    bool nodeInfo_resp_rcvd;

    // bitmasks containing a status for each possible node ID (except 0 and > MAX_NODE_ID)
    Bitmask<128> node_verified; // node seen and unique ID matches stored
    Bitmask<128> node_seen; // received NodeStatus
    Bitmask<128> node_logged; // written to log fle
    Bitmask<128> node_healthy; // reports healthy

    uint8_t last_logging_count;

    //Error State
    enum ServerState server_state;
    uint8_t fault_node_id;
    char fault_node_name[15];


    // dynamic node ID allocation state variables
    uint8_t rcvd_unique_id[16];
    uint8_t rcvd_unique_id_offset;
    uint32_t last_alloc_msg_ms;

    AP_DroneCAN &_ap_dronecan;
    CanardInterface &_canard_iface;

    Canard::Publisher<uavcan_protocol_dynamic_node_id_Allocation> allocation_pub{_canard_iface};

    Canard::ObjCallback<AP_DroneCAN_DNA_Server, uavcan_protocol_dynamic_node_id_Allocation> allocation_cb{this, &AP_DroneCAN_DNA_Server::handle_allocation};
    Canard::Subscriber<uavcan_protocol_dynamic_node_id_Allocation> allocation_sub;

    Canard::ObjCallback<AP_DroneCAN_DNA_Server, uavcan_protocol_NodeStatus> node_status_cb{this, &AP_DroneCAN_DNA_Server::handleNodeStatus};
    Canard::Subscriber<uavcan_protocol_NodeStatus> node_status_sub;

    Canard::ObjCallback<AP_DroneCAN_DNA_Server, uavcan_protocol_GetNodeInfoResponse> node_info_cb{this, &AP_DroneCAN_DNA_Server::handleNodeInfo};
    Canard::Client<uavcan_protocol_GetNodeInfoResponse> node_info_client;

public:
    AP_DroneCAN_DNA_Server(AP_DroneCAN &ap_dronecan, CanardInterface &canard_iface, uint8_t driver_index);


    // Do not allow copies
    CLASS_NO_COPY(AP_DroneCAN_DNA_Server);

    //Initialises publisher and Server Record for specified uavcan driver
    bool init(uint8_t own_unique_id[], uint8_t own_unique_id_len, uint8_t node_id);

    //report the server state, along with failure message if any
    bool prearm_check(char* fail_msg, uint8_t fail_msg_len) const;

    // Get node health statistics for LED display
    uint8_t get_healthy_node_count() { return node_healthy.count(); }
    uint8_t get_verified_node_count() { return node_verified.count(); }
    uint8_t get_seen_node_count() { return node_seen.count(); }
    bool has_healthy_nodes() { return node_healthy.count() > 0; }
    bool all_nodes_healthy() { return node_healthy == node_verified && node_verified.count() > 0; }

    // Check if a specific node has been seen (for parameter access via MAVLink)
    bool is_node_seen(uint8_t node_id) { return node_seen.get(node_id); }

    // Get node counts excluding local node (for display purposes)
    uint8_t get_remote_healthy_count() {
        uint8_t count = node_healthy.count();
        return (node_healthy.get(self_node_id) && count > 0) ? count - 1 : count;
    }
    uint8_t get_remote_verified_count() {
        uint8_t count = node_verified.count();
        return (node_verified.get(self_node_id) && count > 0) ? count - 1 : count;
    }

    // canard message handler callbacks
    void handle_allocation(const CanardRxTransfer& transfer, const uavcan_protocol_dynamic_node_id_Allocation& msg);
    void handleNodeStatus(const CanardRxTransfer& transfer, const uavcan_protocol_NodeStatus& msg);
    void handleNodeInfo(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoResponse& rsp);

    //Run through the list of seen node ids for verification
    void verify_nodes();

    // Request node info for all seen nodes (for MAV_CMD_UAVCAN_GET_NODE_INFO)
    void request_all_node_info();

private:
    // MAVLink reporting functions
    void send_node_status_mavlink(uint8_t node_id, const uavcan_protocol_NodeStatus& msg);
    void report_node_health_change(uint8_t node_id, uint8_t health, uint8_t mode, bool recovered);
    void send_node_info_mavlink(uint8_t node_id, const uavcan_protocol_GetNodeInfoResponse& msg);
};

#endif
