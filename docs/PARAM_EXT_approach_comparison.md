# PARAM_EXT Implementation Approaches Comparison

## Overview

Two different architectural approaches for accessing DroneCAN node parameters via MAVLink have been proposed. This document analyzes both approaches to facilitate discussion with the mavcan-cli tool developer.

## Approach 1: Parameter Name Encoding (Current ArduPilot Implementation)

### Design
- **Routing encoded in**: Parameter name
- **Format**: `CANn.Nxxx.PARAM_NAME`
- **Target component**: 0 (autopilot)
- **Message types**: PARAM_EXT_REQUEST_READ, PARAM_EXT_VALUE, PARAM_EXT_SET, PARAM_EXT_ACK

### Example
```python
# Access node 42's ESC_INDEX parameter on CAN1
msg = connection.param_ext_request_read_send(
    target_system=1,
    target_component=0,       # Autopilot
    param_id=b"CAN1.N042.ESC_INDEX",
    param_index=-1
)
# Response: PARAM_EXT_VALUE with param_id="CAN1.N042.ESC_INDEX"
```

### Advantages
1. **Explicit routing**: Interface selection is visible in parameter name
2. **Multi-interface support**: Can specify which CAN bus to use (CAN1 vs CAN2)
3. **Debugging friendly**: "CAN1.N042.ESC_INDEX" clearly shows routing path
4. **Simpler firmware**: All messages go to component 0, no component ID multiplexing
5. **Compatible with redundant configs**: Different physical buses explicitly addressable

### Disadvantages
1. **⚠️ CRITICAL: Parameter name length limit**: MAVLink PARAM_EXT param_id is 16 characters
   - Prefix `CAN1.N042.` = 10 characters
   - Only **6 characters remaining** for actual parameter name
   - Many DroneCAN parameters exceed this (e.g., `BATTERY_CAPACITY` = 16 chars)
   - **This is a showstopper for long parameter names**

2. **Name pollution**: DroneCAN parameter names are modified with prefix
3. **Parsing overhead**: Firmware must parse every parameter name
4. **Not standards-compliant**: Doesn't use MAVLink's component ID mechanism for routing

## Approach 2: Component ID Encoding (mavcan-cli Expected Design)

### Design
- **Routing encoded in**: Component ID
- **Format**: Native DroneCAN parameter names
- **Target component**: 240 + node_id (e.g., 250 for node 10)
- **Message types**: PARAM_EXT_REQUEST_READ, PARAM_EXT_VALUE, PARAM_EXT_SET, PARAM_EXT_ACK

### Example
```python
# Access node 42's ESC_INDEX parameter
msg = connection.param_ext_request_read_send(
    target_system=1,
    target_component=240 + 42,  # Component 282 = Node 42
    param_id=b"ESC_INDEX",      # Native name, no prefix
    param_index=-1
)
# Response: PARAM_EXT_VALUE from component 282 with param_id="ESC_INDEX"
```

### Advantages
1. **✅ No name length restrictions**: Native DroneCAN parameter names preserved
2. **Standards-compliant**: Uses MAVLink component ID for routing (as designed)
3. **Clean parameter names**: No prefixes or encoding
4. **Efficient**: No string parsing needed
5. **Scalable**: Component ID space supports up to 128 nodes (240-367)

### Disadvantages
1. **❌ CRITICAL: No multi-interface routing**: Component ID only encodes node ID, not which CAN interface
   - How to specify "node 42 on CAN1 vs CAN2"?
   - Redundant configurations become ambiguous
   - Cannot test/debug individual interfaces

2. **Component ID namespace management**:
   - Range 240-367 for DroneCAN nodes
   - Must ensure no conflicts with other ArduPilot components
   - MAVLink component ID semantics: nodes aren't really "components"

3. **More complex firmware**:
   - Must multiplex PARAM_EXT messages by component ID
   - Route messages to correct CAN interface (but which one?)
   - Need heuristic or default interface selection

## Critical Issue Analysis

### Issue 1: Parameter Name Length (Approach 1)

**Problem**: `CAN1.N042.` uses 10 chars, leaving only 6 for parameter name.

**Example failures**:
```
CAN1.N042.BATTERY_CAPACITY    → 26 chars, exceeds 16-char limit
CAN1.N042.MOTOR_POLES          → 21 chars, exceeds limit
CAN1.N042.ESC_INDEX            → 19 chars, exceeds limit
```

**Mitigation attempts**:
- Use shorter prefix? `C1.N42.BATTERY_CAPACITY` = 23 chars (still too long)
- Abbreviate param names? Breaks compatibility, non-standard
- **No good solution without breaking DroneCAN parameter naming**

### Issue 2: Multi-Interface Routing (Approach 2)

**Problem**: Component ID cannot encode which CAN interface to use.

**Scenario**:
```
Node 42 is connected to both CAN1 and CAN2 (redundancy)
Message to component 282 (node 42) arrives
Question: Which interface should handle it?
```

**Mitigation attempts**:

**Option A: Always use first available interface**
```cpp
// Problem: No way to specify interface explicitly
AP_DroneCAN *ap_dronecan = find_first_dronecan_with_node(node_id);
```
- Cons: Cannot test specific interfaces, debugging impossible

**Option B: Try all interfaces**
```cpp
// Send request to all interfaces that have this node
for (auto *dronecan : all_interfaces) {
    if (dronecan->has_node(node_id)) {
        dronecan->get_parameter_on_node(node_id, ...);
    }
}
```
- Cons: Multiple responses, race conditions, wasted bandwidth

**Option C: Extend component ID encoding**
```
Component ID = 240 + (can_interface * 128) + node_id
CAN1 Node 42 → 240 + (0 * 128) + 42 = 282
CAN2 Node 42 → 240 + (1 * 128) + 42 = 410
```
- Cons: Exceeds 8-bit component ID range (max 255)

**Option D: Add CAN interface field to PARAM_EXT messages**
- Cons: Requires MAVLink protocol change, not backward compatible

## Hybrid Approach (Potential Solution)

### Design
Use both component ID AND a CAN interface selector:

**Component ID**: 240 + node_id (for node selection)
**Parameter name prefix**: `Cn.` where n=interface (for interface selection)

### Example
```python
# Access node 42's ESC_INDEX on CAN1
msg = connection.param_ext_request_read_send(
    target_system=1,
    target_component=240 + 42,    # Node 42
    param_id=b"C1.ESC_INDEX",     # CAN1, parameter ESC_INDEX
    param_index=-1
)
```

### Analysis
- Component ID: 240 + node_id → 3 chars (`282`)
- Interface prefix: `C1.` → 3 chars
- Remaining for param name: **13 chars**
- Examples that fit:
  - `C1.BATTERY_CAPACITY` → 19 chars (still too long!)
  - `C1.ESC_INDEX` → 13 chars (fits)
  - `C1.MOTOR_POLES` → 14 chars (too long)

**Verdict**: Still insufficient for long parameter names.

## Recommended Solution: Enhanced Component ID Approach

### Proposal
Encode **both** CAN interface and node ID in a structured component ID scheme using upper bits:

```
Component ID (8 bits):
- Bits 0-6: Node ID (0-127)
- Bit 7: Set to 1 (marks as DroneCAN range)
- Use MAV_COMPONENT_ID_USER base (25) + offset

Base: 240 (0xF0)
Format: 240 + (can_interface << 7) + node_id

CAN1 Node 42: 240 + (0 << 7) + 42 = 282
CAN2 Node 42: 240 + (1 << 7) + 42 = 410 (❌ exceeds 255)
```

**Problem**: Still exceeds 8-bit component ID limit for CAN2+.

### Alternative: Use Reserved Bits in PARAM_EXT

MAVLink PARAM_EXT messages have 128-byte value field. Could encode metadata:

```
param_value[0-1]: CAN interface selector (if needed)
param_value[2-127]: Actual parameter value
```

**Problem**: Requires protocol extension, breaks compatibility.

## Conclusion and Recommendation

### Current State
- **Approach 1 (Implemented)**: Blocked by 16-char parameter name limit
- **Approach 2 (mavcan-cli)**: Blocked by multi-interface routing ambiguity

### Both approaches have critical limitations that prevent full functionality.

### Path Forward Options

**Option 1: Accept Limitations of Approach 1**
- Document maximum parameter name length (6 chars after `CAN1.N042.`)
- Only support short DroneCAN parameter names
- **Unacceptable**: Breaks compatibility with standard DroneCAN devices

**Option 2: Accept Limitations of Approach 2**
- Always use first/default CAN interface
- Add manual interface selection parameter: `CAN_PARAM_IFACE`
- Users set `CAN_PARAM_IFACE=1` or `2` to select interface before sending PARAM_EXT
- **Workable but clunky**: Extra state management required

**Option 3: Extend MAVLink Protocol**
- Add `can_interface` field to PARAM_EXT messages
- Submit MAVLink protocol change proposal
- **Long-term solution but requires community consensus**

**Option 4: Use PARAM_EXT value field for routing metadata**
- Encode CAN interface in first bytes of param_value
- **Hacky but backward compatible for get operations**

### Immediate Recommendation

**Implement Approach 2 with Option 2 workaround:**

1. Use component ID encoding (240 + node_id)
2. Add ArduPilot parameter: `CAN_PARAM_IFACE` (default=1)
3. When PARAM_EXT message arrives:
   - Extract node_id from component_id
   - Use CAN interface from `CAN_PARAM_IFACE` parameter
   - Route to `AP_DroneCAN::get_dronecan(CAN_PARAM_IFACE - 1)`

4. For mavcan-cli:
   ```bash
   # Set interface first (one-time)
   mavcan param set CAN_PARAM_IFACE 1

   # Now access node parameters (uses CAN1)
   mavcan --node 42 get ESC_INDEX
   ```

**Advantages**:
- ✅ No parameter name length restrictions
- ✅ Standards-compliant component ID usage
- ✅ Multi-interface support (via state parameter)
- ✅ Works with mavcan-cli's expected design
- ⚠️ Requires users to set interface selector

**Disadvantages**:
- Interface selection is stateful (not per-request)
- Cannot access multiple interfaces simultaneously
- Extra parameter to manage

### Long-Term Recommendation

Work with MAVLink community to add CAN interface field to PARAM_EXT protocol, or create new DroneCAN-specific MAVLink messages that properly encode routing information.

## Questions for mavcan-cli Developer

1. How does mavcan-cli handle multi-interface DroneCAN systems?
2. Is the 240 + node_id component ID mapping acceptable?
3. Would a stateful interface selector (`CAN_PARAM_IFACE`) be acceptable?
4. Are there other MAVLink protocol extensions to consider?
5. Would you consider contributing to a MAVLink protocol proposal for proper DroneCAN parameter routing?
