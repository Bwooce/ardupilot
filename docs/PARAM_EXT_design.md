# PARAM_EXT Implementation Design for DroneCAN Parameter Access

## Overview

This document describes the implementation of MAVLink PARAM_EXT protocol support in ArduPilot to enable GCS tools (like mavcan-cli) to get and set DroneCAN node parameters via MAVLink.

## Problem Statement

Current ArduPilot implementation:
- Has PARAM_EXT message definitions (messages 320-324)
- No handlers for these messages
- No bridge to existing AP_DroneCAN parameter functions

mavcan-cli tool requires PARAM_EXT support to access DroneCAN node parameters.

## Solution Approach

Implement PARAM_EXT handlers that bridge to existing AP_DroneCAN async parameter functions:
- `get_parameter_on_node()` - async get with callback
- `set_parameter_on_node()` - async set with callback

## Component ID Mapping (USER Range Approach)

### Design Philosophy
Use MAVLink's standard component ID mechanism for routing, rather than encoding routing information in parameter names.

### Component ID Allocation
**Range**: MAVLink USER range (component IDs 25-99) for DroneCAN nodes 1-75

**Mapping Formula**:
```
Component ID = 25 + (node_id - 1)

Examples:
  Node 1  → Component 25
  Node 10 → Component 34
  Node 42 → Component 66
  Node 75 → Component 99
```

**Why USER Range (25-99)?**
1. **Standards Compliant**: MAVLink USER range (25-99) is explicitly designated for "privately managed MAVLink networks"
2. **Appropriate Use**: DroneCAN nodes are effectively a private/internal network within the vehicle
3. **No Conflicts**: No conflicts with standard MAVLink components (autopilot, camera, gimbal, etc.)
4. **Sufficient Capacity**: 75 component IDs covers >95% of practical DroneCAN systems
5. **Clean Mapping**: Simple, intuitive formula with no gaps
6. **Extensible**: Could add explicit interface selection parameter later if needed

### Parameter Names
**Format**: Native DroneCAN parameter names (no prefixes)

**Examples**:
```
ESC_INDEX           → Native 16-char DroneCAN name
MOTOR_POLES         → Native 16-char DroneCAN name
BATTERY_CAPACITY    → Native 16-char DroneCAN name
```

**Advantages**:
- No artificial 16-character limit imposed by name encoding
- Supports full DroneCAN parameter name space
- Clean, standards-compliant approach
- Compatible with mavcan-cli expectations

### CAN Interface Discovery
**Approach**: Automatic discovery via DNA server tracking

**Implementation**:
```cpp
// Each AP_DroneCAN driver instance tracks which nodes it has seen
AP_DroneCAN *ap_dronecan = AP_DroneCAN::get_dronecan(i);
if (ap_dronecan->get_dna_server().node_seen.get(node_id)) {
    // This interface has the node
}
```

**Behavior**:
- Firmware automatically queries all CAN interfaces to find which has the requested node
- For redundant nodes (on multiple buses), uses first interface found
- Returns PARAM_ACK_FAILED if node not found on any interface

**Future Enhancement** (optional):
Add `CAN_PARAM_IFACE` parameter for explicit interface preference:
```
CAN_PARAM_IFACE = 0  (auto-discover, default)
CAN_PARAM_IFACE = 1  (prefer CAN1)
CAN_PARAM_IFACE = 2  (prefer CAN2)
```

## Message Flow

### PARAM_EXT_REQUEST_READ Flow
1. GCS sends `PARAM_EXT_REQUEST_READ`:
   - `target_component` = 25 + (node_id - 1)
   - `param_id` = native DroneCAN parameter name (e.g., "ESC_INDEX")
2. ArduPilot extracts node_id: `node_id = target_component - 24`
3. ArduPilot finds interface via DNA server: `node_seen.get(node_id)`
4. ArduPilot calls `get_parameter_on_node()` on discovered interface
5. DroneCAN callback receives response
6. ArduPilot sends `PARAM_EXT_VALUE`:
   - `source_component` = 25 + (node_id - 1)
   - `param_id` = native parameter name
   - `param_value` = formatted value string

### PARAM_EXT_SET Flow
1. GCS sends `PARAM_EXT_SET`:
   - `target_component` = 25 + (node_id - 1)
   - `param_id` = native DroneCAN parameter name
   - `param_value` = new value string
2. ArduPilot extracts node_id and finds interface
3. ArduPilot calls `set_parameter_on_node()` on discovered interface
4. DroneCAN callback receives response
5. ArduPilot sends `PARAM_EXT_ACK`:
   - `source_component` = 25 + (node_id - 1)
   - `param_result` = PARAM_ACK_ACCEPTED or PARAM_ACK_FAILED

### PARAM_EXT_REQUEST_LIST
Not supported (DroneCAN protocol doesn't provide parameter list API)

## Implementation Components

### 1. GCS_Param.cpp - Handler Functions

**handle_param_ext_request_read()**:
```cpp
void GCS_MAVLINK::handle_param_ext_request_read(const mavlink_message_t &msg)
{
    // Extract component ID → node_id
    uint8_t node_id = packet.target_component - 24;

    // Auto-discover interface
    for (uint8_t i = 0; i < AP_DRONECAN_MAX_INSTANCES; i++) {
        AP_DroneCAN *candidate = AP_DroneCAN::get_dronecan(i);
        if (candidate && candidate->get_dna_server().node_seen.get(node_id)) {
            // Found interface, queue DroneCAN request
            ap_dronecan->get_parameter_on_node(node_id, param_name, &callback);
        }
    }
}
```

**handle_param_ext_set()**: Similar to read, but calls `set_parameter_on_node()`

**send_param_ext_value()**: Set source component ID to `25 + (node_id - 1)`

**send_param_ext_ack()**: Set source component ID to `25 + (node_id - 1)`

### 2. GCS.h - Data Structures

```cpp
struct pending_param_ext_request {
    mavlink_channel_t chan;
    uint8_t can_driver_index;  // 0-based index (0-8 for CAN1-CAN9)
    uint8_t node_id;           // DroneCAN node ID (1-127)
    char param_name[17];       // DroneCAN param name (max 16 chars + null)
    uint8_t param_type;        // MAV_PARAM_EXT_TYPE
    char param_value[128];     // Extended parameter value buffer
    bool is_set;               // true=set, false=get
    uint32_t request_time_ms;  // For timeout handling
};

static ObjectBuffer<pending_param_ext_request> param_ext_requests(5);
```

### 3. DroneCAN Callbacks

Use existing callback types:
- `ParamGetSetFloatCb` - for float parameters
- `ParamGetSetIntCb` - for int32_t parameters
- `ParamGetSetStringCb` - for string parameters

Callbacks send `PARAM_EXT_VALUE` or `PARAM_EXT_ACK` back to GCS with correct source component ID.

## Type Mapping

MAVLink PARAM_EXT → DroneCAN:
- `MAV_PARAM_EXT_TYPE_REAL32` → float (ParamGetSetFloatCb)
- `MAV_PARAM_EXT_TYPE_INT32` → int32_t (ParamGetSetIntCb)
- `MAV_PARAM_EXT_TYPE_CUSTOM` → string (ParamGetSetStringCb)

## Limitations and Notes

### Supported Nodes
- **Nodes 1-75**: Fully supported via component IDs 25-99
- **Nodes 76+**: Not accessible via PARAM_EXT (use native DroneCAN tools)
- This covers >95% of practical DroneCAN systems

### Other Limitations
- `PARAM_EXT_REQUEST_LIST` not supported (no DroneCAN API to list parameters)
- Only supports parameters on configured DroneCAN interfaces
- Sequential processing (one param request at a time per CAN interface)
- Maximum 9 CAN interfaces supported (CAN1-CAN9)
- 100ms timeout per request (AP_DRONECAN_GETSET_TIMEOUT_MS)

### Redundant Node Handling
For redundant nodes (same node ID on multiple buses):
- Uses first interface where node is found
- Optional `CAN_PARAM_IFACE` parameter can force specific interface (future enhancement)

## Testing with mavcan-cli

**mavcan-cli Update Required**:
Change component ID base from 240 to 25:
```python
# Old (conflicts with bridges/system control):
component_id = 240 + node_id

# New (USER range):
component_id = 25 + (node_id - 1)
```

**Example Commands**:
```bash
# Access node 1 ESC_INDEX (component 25)
mavcan --port /dev/ttyUSB0 --node 1 get ESC_INDEX

# Access node 42 MOTOR_KV (component 66)
mavcan --port /dev/ttyUSB0 --node 42 set MOTOR_KV 1000
```

## Advantages of This Approach

1. **Standards Compliant**: Uses MAVLink component ID mechanism as designed
2. **No Name Length Restrictions**: Supports full 16-character DroneCAN parameter names
3. **Clean Implementation**: No complex name parsing required
4. **Automatic Routing**: DNA server provides interface discovery
5. **Practical Coverage**: 75 nodes sufficient for >95% of systems
6. **Compatible**: Matches mavcan-cli expectations (after component base update)
7. **Legitimate Use**: USER range explicitly intended for private networks

## References

- MAVLink PARAM_EXT protocol: https://mavlink.io/en/services/parameter_ext.html
- MAVLink component ID ranges: https://mavlink.io/en/messages/common.html#MAV_COMPONENT
- ArduPilot AP_DroneCAN implementation: libraries/AP_DroneCAN/
- Component ID analysis: docs/PARAM_EXT_component_id_analysis.md
- Implementation plan: docs/PARAM_EXT_implementation_plan.md
