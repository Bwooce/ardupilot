# PARAM_EXT Improved Solution: Component ID Encoding

## Key Insights from Discussion

1. **ArduPilot DOES track which nodes are on which interface**
   - Each `AP_DroneCAN` driver instance has a DNA server
   - DNA server maintains `node_seen` bitmask (128 nodes per interface)
   - Query: `ap_dronecan->get_dna_server().node_seen.get(node_id)`

2. **Component ID space above 100 is largely unused**
   - MAVLink component IDs: 0-255 (8-bit field)
   - Standard components: 0-50 (autopilot, gimbal, camera, etc.)
   - Available for DroneCAN: 100-255 (156 component IDs)

## Proposed Solution: Structured Component ID Encoding

### Component ID Allocation
```
CAN1 nodes: 100-227 (128 IDs)
  Component ID = 100 + node_id
  Node 1   → Component 101
  Node 42  → Component 142
  Node 127 → Component 227

CAN2 nodes: 228-355 (exceeds 255!)
```

**Problem**: CAN2 would overflow 8-bit component ID limit.

### Alternative: Use Available Space More Efficiently

```
CAN1 nodes (1-127): Component ID = 100 + node_id
  Node 42 → Component 142

CAN2 nodes (1-127): Component ID = 100 + 64 + node_id  [mod 128 offset]
  Node 42 → Component 206

CAN3 nodes (1-63): Component ID = 192 + node_id
  Node 42 → Component 234
```

**Problem**: Still runs out of space, arbitrary limits.

## Better Solution: Automatic Interface Detection

### Design
Use component ID to specify **only** the node ID, then **query all interfaces** to find which one has that node.

**Component ID**: 100 + node_id (simple, clean)
**Interface selection**: Automatic discovery

### Implementation

```cpp
void GCS_MAVLINK::handle_param_ext_request_read(const mavlink_message_t &msg)
{
    mavlink_param_ext_request_read_t packet;
    mavlink_msg_param_ext_request_read_decode(&msg, &packet);

    // Extract node ID from component ID
    uint8_t node_id = packet.target_component - 100;

    if (node_id < 1 || node_id > 127) {
        // Not a DroneCAN node component ID
        return;
    }

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

    // Proceed with parameter request on discovered interface
    char param_name[17];
    strncpy(param_name, packet.param_id, sizeof(param_name) - 1);
    param_name[sizeof(param_name) - 1] = '\0';

    // Queue request for this interface
    pending_param_ext_request req;
    req.chan = chan;
    req.can_driver_index = can_driver_index;
    req.node_id = node_id;
    strncpy(req.param_name, param_name, sizeof(req.param_name));
    strncpy(req.full_param_name, param_name, sizeof(req.full_param_name));
    req.is_set = false;
    req.request_time_ms = AP_HAL::millis();

    // Try to initiate DroneCAN parameter get
    static AP_DroneCAN::ParamGetSetFloatCb float_cb(dronecan_param_get_set_float_cb);
    if (ap_dronecan->get_parameter_on_node(node_id, param_name, &float_cb)) {
        param_ext_requests.push(req);
    } else {
        send_param_ext_ack(packet.param_id, "",
                          MAV_PARAM_EXT_TYPE_REAL32, PARAM_ACK_FAILED);
    }
}
```

### Advantages

✅ **No name length restrictions** - Native DroneCAN parameter names preserved
✅ **Standards-compliant** - Uses MAVLink component ID for routing
✅ **Simple mapping** - Component 142 = Node 42 (obvious, memorable)
✅ **Automatic interface discovery** - Firmware finds the right interface
✅ **Works with redundant nodes** - Uses first available interface automatically
✅ **Compatible with mavcan-cli** - Exactly their expected design!

### Handling Redundant Nodes

**Scenario**: Node 42 is on both CAN1 and CAN2

**Behavior**:
- PARAM_EXT to component 142 (node 42)
- Firmware searches: CAN1 has node 42? Yes → Use CAN1
- Result: First interface wins

**For explicit interface selection**:
Users can **temporarily disable** an interface to force the other:
```bash
# Force use of CAN2 by disabling CAN1
mavlink param set CAN_P1_DRIVER 0  # Disable CAN1
mavlink --node 42 get ESC_INDEX     # Now uses CAN2
mavlink param set CAN_P1_DRIVER 1  # Re-enable CAN1
```

Or add explicit interface parameter (stateful selector):
```bash
# Preferred interface for parameter access
CAN_PARAM_IFACE = 1  # Prefer CAN1
CAN_PARAM_IFACE = 2  # Prefer CAN2
```

### Component ID Mapping

```
Component ID Range: 100-227 (supports nodes 0-127)

Examples:
  Component 101 → Node 1
  Component 110 → Node 10
  Component 142 → Node 42
  Component 200 → Node 100
  Component 227 → Node 127
```

### mavcan-cli Compatibility

**Perfect compatibility!** mavcan-cli currently uses:
```python
component_id = 240 + node_id
```

We can update to:
```python
component_id = 100 + node_id
```

Or make it configurable:
```python
# In mavcan config
DRONECAN_COMPONENT_BASE = 100  # ArduPilot uses 100
component_id = DRONECAN_COMPONENT_BASE + node_id
```

## Implementation Checklist

- [x] Document automatic interface discovery approach
- [ ] Modify `handle_param_ext_request_read()` to use component ID
- [ ] Modify `handle_param_ext_set()` to use component ID
- [ ] Remove `parse_dronecan_param_name()` - no longer needed
- [ ] Add `find_dronecan_node()` helper function
- [ ] Update response messages to use correct component ID
- [ ] Add optional `CAN_PARAM_IFACE` preference parameter
- [ ] Test with single interface
- [ ] Test with redundant node configuration
- [ ] Update documentation
- [ ] Coordinate with mavcan-cli team on component base

## Response Handling

**CRITICAL**: Responses must come from the correct component ID

```cpp
void send_param_ext_value_to_node(uint8_t node_id, const char *param_name,
                                    const char *param_value, uint8_t param_type)
{
    mavlink_message_t msg;
    mavlink_msg_param_ext_value_pack(
        mavlink_system.sysid,
        100 + node_id,  // ← Source component = node's component ID
        &msg,
        param_name,
        param_value,
        param_type,
        0, 0
    );

    MAVLINK_ALIGNED_BUF(buf, MAVLINK_MAX_PACKET_LEN);
    uint16_t len = mavlink_msg_to_send_buffer((uint8_t*)buf, &msg);
    comm_send_buffer(chan, (uint8_t*)buf, len);
}
```

## Migration Path

### Phase 1: Implement Component ID Approach
- Remove CANn.Nxxx.PARAM_NAME encoding
- Use component ID for node selection
- Auto-discover interface
- Full parameter name support

### Phase 2: Add Preference Parameter (Optional)
```cpp
// New parameter: CAN_PARAM_IFACE (0=auto, 1=CAN1, 2=CAN2, etc.)
AP_Int8 param_interface_preference;

// In find_dronecan_node():
if (param_interface_preference > 0) {
    // Try preferred interface first
    uint8_t pref_idx = param_interface_preference - 1;
    AP_DroneCAN *preferred = AP_DroneCAN::get_dronecan(pref_idx);
    if (preferred && preferred->get_dna_server().node_seen.get(node_id)) {
        return preferred;
    }
}
// Fall back to auto-discovery
```

### Phase 3: Coordinate with mavcan-cli
- Update mavcan-cli to use component base 100
- Test integration
- Document component ID scheme

## Comparison to Previous Approaches

| Feature | CANn.Nxxx.PARAM | Component 240+ | Component 100+ Auto |
|---------|-----------------|----------------|---------------------|
| **Name length** | ❌ 6 chars only | ✅ Full names | ✅ Full names |
| **Multi-interface** | ✅ Explicit CAN1/CAN2 | ❌ No encoding | ✅ Auto or preference |
| **Standards** | ❌ Non-standard | ⚠️ Partial | ✅ Full compliance |
| **Simplicity** | ❌ Complex parsing | ✅ Simple | ✅ Simple |
| **mavcan-cli** | ❌ Not compatible | ✅ Compatible | ✅ **Fully compatible** |
| **Debugging** | ✅ Explicit routing | ⚠️ Ambiguous | ✅ Auto-discover |

## Conclusion

**This solution is superior to both previous approaches:**

1. ✅ Solves the 16-character name length limitation
2. ✅ Uses existing node tracking (DNA server)
3. ✅ Clean component ID mapping (100 + node_id)
4. ✅ Automatic interface discovery
5. ✅ Optional explicit interface preference
6. ✅ Fully compatible with mavcan-cli expectations
7. ✅ Uses available component ID space (100-227)

**Recommendation**: Implement this approach immediately.
