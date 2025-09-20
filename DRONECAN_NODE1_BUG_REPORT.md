# DroneCAN Node 1 Protocol Violation Bug Report

## Summary
Node 1 on the CAN bus is transmitting malformed DroneCAN messages containing ASCII text fragments instead of properly formatted binary protocol data. This causes continuous communication errors with the ArduPilot flight controller (Node 10).

## Affected System
- **Flight Controller**: ArduPilot ESP32 (Node 10)
- **Problem Node**: E-bike Controller (Node 1)
- **Protocol**: DroneCAN v0 (UAVCAN)
- **CAN Bus**: 1 Mbps

## Observed Behavior

### Error Messages from ArduPilot
```
I (52978) CAN_RX: ERROR UNEXPECTED_TID (-15): dtype=1, src=1, dest=10, our_id=10
I (68418) CAN_RX: ERROR WRONG_TOGGLE (-14): dtype=1, src=1, dest=10, our_id=10
I (92358) CAN_RX: ERROR NOT_WANTED (-12): dtype=65500, src=1, dest=92, our_id=10
```

### Captured Malformed Frames

#### Frame 1: Text in NodeStatus Message
```
CAN ID: 0x10010A81 (NodeStatus from Node 1)
Data:   00 0E 64 72 6F 6E 65 0E
Decoded: Tail=0x00, Payload=".drone."
```
**Issue**: NodeStatus (dtype=1) should contain binary status data, not ASCII text.

#### Frame 2: More Text Fragments
```
CAN ID: 0x10010A81 (NodeStatus from Node 1)
Data:   63 61 6E 2E 65 62 69 38
Decoded: "can.ebi8" (likely "can.ebike")
```
**Issue**: Missing tail byte, raw ASCII text instead of DroneCAN framing.

#### Frame 3: Vendor-Specific Message
```
CAN ID: 0x10FFDC01 (dtype=65500, vendor-specific)
Data:   00 69
```
**Issue**: Undefined vendor-specific message type not recognized by ArduPilot.

## Technical Analysis

### Protocol Violations Identified

1. **Invalid NodeStatus Content**
   - Expected: `[uint32 uptime][uint8 health][uint8 mode][uint8 submode][uint16 vendor_status]`
   - Received: ASCII text fragments ("drone", "can.ebike")

2. **Missing DroneCAN Framing**
   - Some frames lack proper tail bytes
   - No valid transfer ID management
   - Toggle bit not alternating correctly

3. **Text Debug Output on CAN Bus**
   - Node appears to be sending debug strings directly to CAN
   - Text includes: "drone", "can.ebike" suggesting e-bike controller context

### Root Cause Hypothesis

The e-bike controller (Node 1) appears to be:
1. Sending debug printf() output directly to the CAN interface
2. Using an incompatible DroneCAN implementation
3. Mixing text protocol with binary DroneCAN frames

## Impact

- **Communication Failure**: Node 1 cannot properly communicate with flight controller
- **Log Spam**: Continuous error messages (18+ per 5 seconds)
- **Timing Issues**: NodeStatus interval 2164ms instead of expected 1000ms
- **System Instability**: Potential for cascading failures in multi-node systems

## Reproduction Steps

1. Connect Node 1 (e-bike controller) to CAN bus
2. Start ArduPilot with DroneCAN enabled
3. Monitor CAN bus traffic
4. Observe malformed frames and error messages

## Recommended Fix

### For Node 1 Developer Team:

1. **Remove Debug Output from CAN Interface**
   ```c
   // BAD - Don't do this:
   can_send("can.ebike status");

   // GOOD - Use proper DroneCAN messages:
   uavcan_protocol_NodeStatus status = {
       .uptime_sec = uptime,
       .health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK,
       .mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL,
       .sub_mode = 0,
       .vendor_specific_status_code = 0
   };
   broadcast_node_status(&status);
   ```

2. **Implement Proper DroneCAN Framing**
   - Always include tail bytes
   - Manage transfer IDs correctly
   - Alternate toggle bits in multi-frame transfers

3. **Validate Message Types**
   - Only send standard DroneCAN message types
   - Document any vendor-specific messages (dtype >16000)

4. **Test Protocol Compliance**
   - Use a DroneCAN analyzer to verify frame format
   - Test with reference implementation (pyuavcan, libcanard)

## Workaround (Temporary)

For ArduPilot users experiencing this issue:
1. Disconnect Node 1 from CAN bus
2. Or add message filtering to ignore Node 1:
   ```cpp
   if (transfer.source_node_id == 1) {
       return; // Ignore malformed messages from Node 1
   }
   ```

## Severity
**HIGH** - Prevents proper CAN communication and causes system instability

## Attachments
- Raw CAN trace showing malformed frames
- Error log from ArduPilot
- Expected vs Actual frame comparison

---
*Report generated from ArduPilot ESP32 diagnostic session*
*Date: 2025-09-20*