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
