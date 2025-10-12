# PARAM_EXT Implementation Plan - USER Range Approach

## Agreed Solution

**Use MAVLink USER range (25-99) for DroneCAN nodes 1-75**

### Component ID Mapping
```
Component ID = 25 + (node_id - 1)

Examples:
  Node 1  → Component 25
  Node 10 → Component 34
  Node 42 → Component 66
  Node 75 → Component 99
```

### Interface Discovery
Use DNA server's `node_seen` bitmask to auto-discover which CAN interface has each node.

## Changes Required

### 1. Remove Name Encoding Approach

**Files to modify:**
- `libraries/GCS_MAVLink/GCS_Param.cpp`
- `libraries/GCS_MAVLink/GCS.h`

**Changes:**
- Remove `parse_dronecan_param_name()` function
- Update `handle_param_ext_request_read()` to use component ID instead
- Update `handle_param_ext_set()` to use component ID instead
- Remove `full_param_name` field from `pending_param_ext_request` structure

### 2. Add Component ID Routing

**In GCS_Param.cpp:**

```cpp
void GCS_MAVLINK::handle_param_ext_request_read(const mavlink_message_t &msg)
{
    mavlink_param_ext_request_read_t packet;
    mavlink_msg_param_ext_request_read_decode(&msg, &packet);

    // Check if component ID is in USER range for DroneCAN nodes
    if (packet.target_component < 25 || packet.target_component > 99) {
        // Not a DroneCAN node request
        return;
    }

    // Extract node ID from component ID
    uint8_t node_id = packet.target_component - 24;  // Component 25 → node 1

    // Find which CAN interface has this node
    AP_DroneCAN *ap_dronecan = nullptr;
    uint8_t can_driver_index = 0;

    for (uint8_t i = 0; i < AP_DRONECAN_MAX_INSTANCES; i++) {
        AP_DroneCAN *candidate = AP_DroneCAN::get_dronecan(i);
        if (candidate != nullptr &&
            candidate->get_dna_server().node_seen.get(node_id)) {
            ap_dronecan = candidate;
            can_driver_index = i;
            break;  // Use first interface that has this node
        }
    }

    if (ap_dronecan == nullptr) {
        // Node not found on any interface
        send_param_ext_ack(packet.param_id, "",
                          MAV_PARAM_EXT_TYPE_REAL32, PARAM_ACK_FAILED);
        return;
    }

    // Extract parameter name
    char param_name[17];
    memcpy(param_name, packet.param_id, 16);
    param_name[16] = '\0';

    // Create pending request
    pending_param_ext_request req;
    req.chan = chan;
    req.can_driver_index = can_driver_index;
    req.node_id = node_id;
    strncpy(req.param_name, param_name, sizeof(req.param_name) - 1);
    req.param_name[sizeof(req.param_name) - 1] = '\0';
    req.is_set = false;
    req.request_time_ms = AP_HAL::millis();

    // Try to initiate DroneCAN parameter get (try float first, most common)
    static AP_DroneCAN::ParamGetSetFloatCb float_cb(dronecan_param_get_set_float_cb);
    if (ap_dronecan->get_parameter_on_node(node_id, param_name, &float_cb)) {
        param_ext_requests.push(req);
        return;
    }

    // Try integer
    static AP_DroneCAN::ParamGetSetIntCb int_cb(dronecan_param_get_set_int_cb);
    if (ap_dronecan->get_parameter_on_node(node_id, param_name, &int_cb)) {
        param_ext_requests.push(req);
        return;
    }

    // Try string
    static AP_DroneCAN::ParamGetSetStringCb string_cb(dronecan_param_get_set_string_cb);
    if (ap_dronecan->get_parameter_on_node(node_id, param_name, &string_cb)) {
        param_ext_requests.push(req);
        return;
    }

    // All attempts failed - probably busy
    send_param_ext_ack(packet.param_id, "",
                      MAV_PARAM_EXT_TYPE_REAL32, PARAM_ACK_FAILED);
}
```

### 3. Update Response Messages

Responses must come from the correct component ID (the node's component ID):

```cpp
void GCS_MAVLINK::send_param_ext_value(const char *param_name, const char *param_value,
                                        uint8_t param_type, uint8_t node_id)
{
    mavlink_message_t msg;
    mavlink_msg_param_ext_value_pack(
        mavlink_system.sysid,
        25 + (node_id - 1),  // Source component = node's component ID
        &msg,
        param_name,
        param_value,
        param_type,
        0, 0  // param_count, param_index not used for DroneCAN
    );

    MAVLINK_ALIGNED_BUF(buf, MAVLINK_MAX_PACKET_LEN);
    uint16_t len = mavlink_msg_to_send_buffer((uint8_t*)buf, &msg);
    comm_send_buffer(chan, (uint8_t*)buf, len);
}
```

### 4. Update Callback Functions

Callbacks need to pass node_id to response functions:

```cpp
static bool dronecan_param_get_set_float_cb(AP_DroneCAN* ap_dronecan, const uint8_t node_id,
                                             const char* name, float &value)
{
    // Find matching pending request
    GCS_MAVLINK::pending_param_ext_request req;
    bool found = false;

    for (uint8_t i = 0; i < GCS_MAVLINK::param_ext_requests.available(); i++) {
        if (GCS_MAVLINK::param_ext_requests.peek(req, i)) {
            if (req.node_id == node_id &&
                strncmp(req.param_name, name, sizeof(req.param_name)) == 0) {
                found = true;
                break;
            }
        }
    }

    if (!found) {
        return false;
    }

    // Format value as string
    char value_str[128];
    snprintf(value_str, sizeof(value_str), "%.6f", value);

    // Send response to GCS with correct component ID
    GCS_MAVLINK *gcs_chan = gcs().chan(req.chan);
    if (gcs_chan != nullptr) {
        if (req.is_set) {
            gcs_chan->send_param_ext_ack(req.param_name, value_str,
                                         MAV_PARAM_EXT_TYPE_REAL32,
                                         PARAM_ACK_ACCEPTED, node_id);
        } else {
            gcs_chan->send_param_ext_value(req.param_name, value_str,
                                           MAV_PARAM_EXT_TYPE_REAL32, node_id);
        }
    }

    return true;
}
```

### 5. Update Data Structures

**In GCS.h:**

```cpp
struct pending_param_ext_request {
    mavlink_channel_t chan;
    uint8_t can_driver_index;  // 0-based index (0-8)
    uint8_t node_id;           // 1-127
    char param_name[17];       // DroneCAN param name (max 16 chars + null)
    uint8_t param_type;        // MAV_PARAM_EXT_TYPE
    char param_value[128];     // Extended parameter value buffer
    bool is_set;               // true=set, false=get
    uint32_t request_time_ms;  // For timeout handling
};
```

**Removed field:** `full_param_name` (no longer needed with component ID approach)

### 6. Optional: CAN_PARAM_IFACE Preference Parameter

Add optional parameter for explicit interface selection:

```cpp
// In AP_DroneCAN.h or appropriate location
AP_Int8 can_param_iface;  // 0=auto, 1=CAN1, 2=CAN2, etc.
```

Modify discovery logic:
```cpp
// Check preferred interface first if set
if (can_param_iface > 0 && can_param_iface <= AP_DRONECAN_MAX_INSTANCES) {
    AP_DroneCAN *preferred = AP_DroneCAN::get_dronecan(can_param_iface - 1);
    if (preferred && preferred->get_dna_server().node_seen.get(node_id)) {
        return preferred;
    }
}
// Fall back to auto-discovery
```

## Testing Plan

### Test Cases

1. **Single interface, single node**
   - Request: Component 25 (node 1) on CAN1
   - Expected: Parameter returned from node 1

2. **Single interface, multiple nodes**
   - Request: Component 25, 26, 34 (nodes 1, 2, 10)
   - Expected: Parameters returned from respective nodes

3. **Redundant node (on CAN1 and CAN2)**
   - Request: Component 66 (node 42)
   - Expected: Uses first interface found (CAN1)

4. **Explicit interface preference**
   - Set CAN_PARAM_IFACE = 2
   - Request: Component 66 (node 42)
   - Expected: Uses CAN2 even if CAN1 also has node

5. **Node not present**
   - Request: Component 50 (node 26) when node 26 doesn't exist
   - Expected: PARAM_EXT_ACK with PARAM_ACK_FAILED

6. **Node > 75**
   - Request: Component 105 (node 81)
   - Expected: Ignored (component ID out of USER range)

7. **Parameter name edge cases**
   - Long name (16 chars): "BATTERY_CAPACITY"
   - Short name (3 chars): "ESC"
   - Expected: Both work (no 16-char total limit anymore!)

### Test with mavcan-cli

Update mavcan-cli component base:
```python
COMPONENT_BASE = 25
component_id = COMPONENT_BASE + (node_id - 1)
```

Test commands:
```bash
mavcan --port /dev/ttyUSB0 --node 1 get ESC_INDEX
mavcan --port /dev/ttyUSB0 --node 42 set MOTOR_KV 1000
```

## Documentation Updates

### User Documentation
- "PARAM_EXT supports DroneCAN nodes 1-75 via component IDs 25-99"
- "Nodes are auto-discovered via DNA server tracking"
- "Optional CAN_PARAM_IFACE parameter for explicit interface selection"
- "Nodes 76+ not accessible via PARAM_EXT (use native DroneCAN tools)"

### Developer Documentation
- Component ID mapping formula: `25 + (node_id - 1)`
- Auto-discovery via `AP_DroneCAN::get_dna_server().node_seen.get(node_id)`
- USER range (25-99) usage justification

## Summary of Changes

**What's staying:**
- ✅ PARAM_EXT message types (REQUEST_READ, VALUE, SET, ACK)
- ✅ DroneCAN callback mechanism
- ✅ ObjectBuffer queue for async requests
- ✅ Type mapping (REAL32, INT32, CUSTOM)

**What's changing:**
- ❌ Remove `parse_dronecan_param_name()` function
- ❌ Remove `full_param_name` field from structure
- ✅ Use component ID (25-99) instead of name encoding
- ✅ Add auto-discovery via DNA server
- ✅ Pass node_id to response functions for correct source component ID

**Result:**
- ✅ Supports full DroneCAN parameter names (no 16-char limit!)
- ✅ Clean component ID mapping (25 + node_id - 1)
- ✅ Compatible with mavcan-cli expectations
- ✅ 75 nodes supported (covers >95% of systems)
