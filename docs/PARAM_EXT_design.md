# PARAM_EXT Implementation Design

## Architecture Overview

### Parameter Naming Convention
Format: `CANn.Nxxx.PARAM_NAME`
- `n` = CAN interface number (1-9, supports up to 9 CAN interfaces)
- `xxx` = Node ID (001-127, zero-padded 3 digits to support full DroneCAN range)
- `PARAM_NAME` = DroneCAN parameter name (e.g., ESC_INDEX)

Examples:
- `CAN1.N001.ESC_INDEX`
- `CAN1.N042.MOTOR_KV`
- `CAN2.N127.PARAM_NAME`

#### Why Interface Number is Required (Redundant Node Support)

In redundant DroneCAN configurations, a **single physical node** has the **same node ID** on multiple CAN buses. Our naming convention requires specifying **which CAN interface** to use to access it.

**Redundant Node Example:**
- Physical ESC with node ID 42 is connected to both CAN1 and CAN2 for redundancy
- `CAN1.N042.ESC_INDEX` → Access node 42's parameter via CAN1 interface
- `CAN2.N042.ESC_INDEX` → Access the same physical node via CAN2 interface

**Why this is necessary:**

1. **ArduPilot Architecture**: Each CAN interface has a separate `AP_DroneCAN` driver instance. When calling `get_parameter_on_node()`, you must specify which driver instance to use, which determines which physical bus the request is sent on.

2. **Testing/Debugging**: You may want to test each interface independently:
   - "Is CAN1 working? Try `CAN1.N042.ESC_INDEX`"
   - "Is CAN2 working? Try `CAN2.N042.ESC_INDEX`"

3. **Interface-Specific Issues**: If one bus is experiencing problems, you can explicitly use the other:
   - "CAN1 is lossy, use CAN2: `CAN2.N042.ESC_INDEX`"

4. **DroneCAN Protocol Requirement**: The protocol requires sending on a specific bus. Even with redundant nodes listening on multiple buses, the **request** must be sent on **one specific interface**.

5. **Explicit Routing Topology**: The naming reflects reality - you're not just addressing "node X", you're addressing "node X **via** interface Y". This makes the routing path explicit and deterministic.

**Alternative Would Be Problematic:**
If we used just `N042.ESC_INDEX` without the interface number:
- ArduPilot wouldn't know which `AP_DroneCAN` driver instance to use
- No way to specify which physical bus to send the request on
- No way to test/debug individual interfaces in redundant setups
- Ambiguous behavior when node is on multiple interfaces

**Implementation:**
```cpp
// Parse "CAN1.N042.ESC_INDEX" → can_driver_index=0, node_id=42, param="ESC_INDEX"
AP_DroneCAN *ap_dronecan = AP_DroneCAN::get_dronecan(can_driver_index);
ap_dronecan->get_parameter_on_node(node_id, param_name, callback);
```

The interface number directly maps to the driver instance, making routing explicit and unambiguous.

### Message Flow

1. **PARAM_EXT_REQUEST_READ** → Parse name → DroneCAN get_parameter_on_node() → Callback → **PARAM_EXT_VALUE**
2. **PARAM_EXT_SET** → Parse name → DroneCAN set_parameter_on_node() → Callback → **PARAM_EXT_ACK**
3. **PARAM_EXT_REQUEST_LIST** → Not supported (return empty/error - DroneCAN has no list API)

### Key Constraints
- DroneCAN only allows ONE pending parameter operation per interface
- Must queue PARAM_EXT requests and process sequentially
- Timeout after 100ms (AP_DRONECAN_GETSET_TIMEOUT_MS)

### Implementation Components

#### 1. GCS_Param.cpp - New Functions
- `parse_dronecan_param_name()` - Extract CAN interface, node ID, param name
  - Parse format: `CAN[1-9].N[0-1][0-9][0-9].PARAM_NAME`
  - Returns: can_driver_index (0-8), node_id (1-127), param_name
- `handle_param_ext_request_read()` - Queue DroneCAN get request
- `handle_param_ext_set()` - Queue DroneCAN set request  
- `handle_param_ext_request_list()` - Return not supported
- `send_param_ext_value()` - Send PARAM_EXT_VALUE response
- `send_param_ext_ack()` - Send PARAM_EXT_ACK response

#### 2. GCS.h - New Structures
```cpp
struct pending_param_ext_request {
    mavlink_channel_t chan;
    uint8_t can_driver_index;  // 0-based index (0-8)
    uint8_t node_id;           // 1-127
    char param_name[17];       // DroneCAN param name (max 16 chars + null)
    char full_param_name[AP_MAX_NAME_SIZE+1];  // Full CANn.Nxxx.PARAM for response
    uint8_t param_type;        // MAV_PARAM_EXT_TYPE
    char param_value[128];     // Extended parameter value buffer
    bool is_set;               // true=set, false=get
    uint32_t request_time_ms;  // For timeout handling
};

static ObjectBuffer<pending_param_ext_request> param_ext_requests(5);
```

#### 3. DroneCAN Callbacks
Use existing callback types:
- ParamGetSetFloatCb
- ParamGetSetIntCb  
- ParamGetSetStringCb

Callbacks will send PARAM_EXT_VALUE or PARAM_EXT_ACK back to GCS.

### Type Mapping
MAVLink PARAM_EXT → DroneCAN:
- MAV_PARAM_EXT_TYPE_REAL32 → float (ParamGetSetFloatCb)
- MAV_PARAM_EXT_TYPE_INT32 → int32_t (ParamGetSetIntCb)
- MAV_PARAM_EXT_TYPE_CUSTOM → string (ParamGetSetStringCb)

### Limitations
- PARAM_EXT_REQUEST_LIST not supported (no DroneCAN API to list parameters)
- Only supports parameters on configured DroneCAN interfaces
- Sequential processing (one param request at a time per CAN interface)
- Maximum 9 CAN interfaces supported (CAN1-CAN9)
- Node IDs 1-127 supported (full DroneCAN range)

### Parsing Logic
```
Input: "CAN2.N042.ESC_INDEX"

Step 1: Check prefix "CAN" (bytes 0-2)
Step 2: Parse interface number (byte 3): '2' → driver_index = 1 (0-based)
Step 3: Check dot separator (byte 4)
Step 4: Check 'N' prefix (byte 5)
Step 5: Parse 3-digit node ID (bytes 6-8): "042" → node_id = 42
Step 6: Check dot separator (byte 9)
Step 7: Remaining string is parameter name: "ESC_INDEX"

Validation:
- Interface number 1-9 (maps to driver_index 0-8)
- Node ID 001-127 (leading zeros required)
- Parameter name max 16 chars
- Total length fits in AP_MAX_NAME_SIZE
```
