# PARAM_EXT Component ID Allocation Analysis

## Current MAVLink Component ID Allocations (Standard)

### Allocated Ranges
```
0:     MAV_COMP_ID_ALL (broadcast)
1:     MAV_COMP_ID_AUTOPILOT1
25-99: MAV_COMP_ID_USER1 through USER75 (75 IDs for private networks)
100-105: MAV_COMP_ID_CAMERA through CAMERA6
110-112: MAV_COMP_ID_RADIO through RADIO3
140-153: MAV_COMP_ID_SERVO1 through SERVO14
154:   MAV_COMP_ID_GIMBAL
156-161: ADSB, OSD, PERIPHERAL, QX1_GIMBAL, FLARM, PARACHUTE
169:   WINCH
171-175: MAV_COMP_ID_GIMBAL2 through GIMBAL6
180-181: BATTERY, BATTERY2
191-194: ONBOARD_COMPUTER through ONBOARD_COMPUTER4
200-202: IMU through IMU3
220-221: GPS, GPS2
236-238: ODID (Open Drone ID)
240-242: UDP_BRIDGE, UART_BRIDGE, TUNNEL_NODE
250:   SYSTEM_CONTROL
```

### Available Gaps
```
2-24:    23 IDs  (too small)
106-109:  4 IDs  (too small)
113-139: 27 IDs  (largest contiguous gap, still too small)
176-179:  4 IDs  (too small)
182-190:  9 IDs  (too small)
195-199:  5 IDs  (too small)
203-219: 17 IDs  (too small)
222-235: 14 IDs  (too small)
239:      1 ID   (too small)
243-249:  7 IDs  (too small)
251-255:  5 IDs  (too small)
```

**Total available IDs in gaps: ~116 IDs**
**DroneCAN nodes needed: 128 IDs (node 0-127, though node 0 not used)**

## Problem Statement

**We need 127 component IDs for DroneCAN nodes (1-127) but there is NO contiguous block large enough in the MAVLink standard component ID space.**

The largest available gap is only 27 IDs (113-139), which is insufficient.

## Existing Conflicts

### mavcan-cli Approach (240 + node_id)
```
Node 1   → Component 241 ❌ CONFLICTS with UART_BRIDGE (241)
Node 2   → Component 242 ❌ CONFLICTS with TUNNEL_NODE (242)
Node 3   → Component 243 ✅ Available
...
Node 10  → Component 250 ❌ CONFLICTS with SYSTEM_CONTROL (250)
Node 11  → Component 251 ✅ Available
...
Node 15  → Component 255 ✅ Available (max component ID)
Node 16+ → Component 256+ ❌ EXCEEDS 8-bit limit
```

**mavcan-cli approach only works for nodes 3-9, 11-15 (11 nodes max!)**

### Our Proposal (100 + node_id)
```
Node 1   → Component 101 ✅ Available
Node 2   → Component 102 ✅ Available
Node 3   → Component 103 ✅ Available
Node 4   → Component 104 ✅ Available
Node 5   → Component 105 ✅ Available
Node 6   → Component 106 ❌ CONFLICTS with cameras?
...
Node 10  → Component 110 ❌ CONFLICTS with RADIO (110)
Node 11  → Component 111 ❌ CONFLICTS with RADIO2 (111)
Node 12  → Component 112 ❌ CONFLICTS with RADIO3 (112)
Node 13  → Component 113 ✅ Available
...
Node 39  → Component 139 ✅ Available (end of largest gap)
Node 40  → Component 140 ❌ CONFLICTS with SERVO1 (140)
```

**Our approach only works for nodes 1-5, 13-39 (32 nodes max!)**

## The Fundamental Problem

**There is NO way to map 128 DroneCAN nodes to MAVLink component IDs without conflicts or protocol violations.**

Component ID is an 8-bit field (0-255), and the standard allocates most of it already.

## Possible Solutions

### Solution 1: Use USER Range (25-99) ❌ INSUFFICIENT
```
Component ID = 25 + node_id

Node 1   → Component 26  ✅
Node 2   → Component 27  ✅
...
Node 74  → Component 99  ✅
Node 75  → Component 100 ❌ CONFLICTS with CAMERA
```

**Only supports 74 nodes (1-74), not enough for full DroneCAN range (1-127).**

### Solution 2: Sparse Mapping (Use Gaps) ❌ COMPLEX AND CONFUSING
```
Nodes 1-5    → Components 101-105
Nodes 6-32   → Components 113-139
Nodes 33-36  → Components 176-179
Nodes 37-45  → Components 182-190
... (continue through all gaps)
```

**Problems:**
- Non-intuitive mapping (node 6 = component 113??)
- Complex lookup tables required
- Still doesn't fit all 127 nodes
- Nightmare to debug

### Solution 3: Segment by CAN Interface ❌ WASTEFUL
```
CAN1 nodes: Components 113-139 (27 nodes max)
CAN2 nodes: Components 203-219 (17 nodes max)
CAN3 nodes: Components 222-235 (14 nodes max)
```

**Problems:**
- Arbitrary limits per interface
- Doesn't use component ID space efficiently
- Still doesn't support full node range

### Solution 4: Extend MAVLink Protocol ✅ PROPER SOLUTION
Add a new message type specifically for DroneCAN parameter access:

```c
// New message: DRONECAN_PARAM_REQUEST_READ
uint8_t can_interface;      // Which CAN interface (1-9)
uint8_t node_id;            // DroneCAN node ID (1-127)
char param_name[16];        // Parameter name
int16_t param_index;        // Or use index

// Response: DRONECAN_PARAM_VALUE
uint8_t can_interface;
uint8_t node_id;
char param_name[16];
char param_value[128];      // Extended parameter value
uint8_t param_type;
```

**Advantages:**
- ✅ Proper encoding of all routing information
- ✅ Supports full node range (1-127)
- ✅ Supports multiple CAN interfaces explicitly
- ✅ No component ID conflicts
- ✅ Clean protocol design

**Disadvantages:**
- ⚠️ Requires MAVLink protocol change (community approval needed)
- ⚠️ Not backward compatible
- ⚠️ Requires updates to all GCS tools

### Solution 5: Hybrid - Component ID + State Parameter ⚠️ WORKABLE COMPROMISE

Use available component ID ranges with explicit interface selection:

```
Component ID = 113 + (node_id - 1)  [for nodes 1-27 only]

Node 1  → Component 113
Node 2  → Component 114
...
Node 27 → Component 139

For nodes 28+: Return error "Node not supported via PARAM_EXT"
```

Add ArduPilot parameter: `CAN_PARAM_IFACE` (default=1, range 1-9)

**Advantages:**
- ✅ No standard conflicts for first 27 nodes
- ✅ Interface selection via state parameter
- ✅ Works today with existing protocol

**Disadvantages:**
- ❌ Only supports 27 nodes (not full 127 range)
- ⚠️ Arbitrary limitation
- ⚠️ Stateful interface selection

### Solution 6: PARAM_EXT with Name Encoding (Original Approach) ⚠️ FALLBACK

Revert to `CANn.Nxxx.PARAM_NAME` with component 0:

**Advantages:**
- ✅ No component ID conflicts
- ✅ Explicit routing in name
- ✅ Supports all nodes and interfaces

**Disadvantages:**
- ❌ **16-character limit kills most DroneCAN parameters**
- ❌ Not compatible with mavcan-cli expectations
- ❌ Non-standard approach

## Recommendation

### Short Term: Solution 5 (Hybrid)
Use component IDs 113-139 for nodes 1-27:
```cpp
if (target_component >= 113 && target_component <= 139) {
    uint8_t node_id = target_component - 112;  // 113 → node 1
    // Use CAN_PARAM_IFACE parameter to select interface
    // Handle PARAM_EXT for this node
}
```

**Document clearly**: "PARAM_EXT support limited to first 27 DroneCAN nodes"

### Long Term: Solution 4 (Protocol Extension)
Work with MAVLink community to add proper DroneCAN parameter messages:
- `DRONECAN_PARAM_EXT_REQUEST_READ` (msgid TBD)
- `DRONECAN_PARAM_EXT_VALUE` (msgid TBD)
- `DRONECAN_PARAM_EXT_SET` (msgid TBD)
- `DRONECAN_PARAM_EXT_ACK` (msgid TBD)

These messages would properly encode:
- CAN interface number
- Node ID
- Parameter name (no length restriction)
- Parameter value

## Impact on mavcan-cli

**Current mavcan-cli assumption (240 + node_id) CANNOT work** due to conflicts:
- Component 240 = UDP_BRIDGE
- Component 241 = UART_BRIDGE
- Component 242 = TUNNEL_NODE
- Component 250 = SYSTEM_CONTROL

**Recommendations for mavcan-cli:**

1. **Short term**: Support configurable component base
   ```python
   COMPONENT_BASE = 113  # Use 113-139 range
   component_id = COMPONENT_BASE + (node_id - 1)
   # Only works for nodes 1-27
   ```

2. **Medium term**: Add interface state parameter
   ```python
   # Set interface before parameter access
   mavlink.param_set("CAN_PARAM_IFACE", 1)
   mavlink.param_ext_request_read(component_id, "ESC_INDEX")
   ```

3. **Long term**: Support new DRONECAN_PARAM_* messages
   ```python
   # Once MAVLink protocol extended
   mavlink.dronecan_param_request_read(
       can_interface=1,
       node_id=42,
       param_name="ESC_INDEX"
   )
   ```

## Conclusion

**The component ID space in MAVLink is insufficient for 128 DroneCAN nodes.**

We have three options:
1. ✅ **Accept limitations** (27 nodes max via PARAM_EXT)
2. ⚠️ **Revert to name encoding** (16-char limit problem)
3. ✅ **Extend MAVLink protocol** (proper long-term solution)

**Recommended path forward:**
1. Implement hybrid solution (components 113-139, nodes 1-27)
2. Document limitations clearly
3. Begin MAVLink protocol extension proposal
4. Coordinate with mavcan-cli developer on limitations and long-term solution
