# DNA Allocation Encoding Fix Summary

## The Problem
We were encoding the DNA allocation response incorrectly. The node team was correct.

### What We Were Doing
- Using `canardEncodeScalar` for the 7-bit node_id field
- This function MSB-aligns partial bytes by shifting left
- For node_id=64, we sent 0x80 (shifted left by 1 bit)

### What Nodes Expected
- Direct bit placement without shifting
- For node_id=64, they expected 0x40
- Bits 0-6: node_id (no shift)
- Bit 7: first_part_of_unique_id flag

## The Root Cause
We incorrectly used `canardEncodeScalar` which MSB-aligns partial bytes by shifting them left. This was wrong for the DNA allocation message where the spec requires direct bit placement without shifting.

## The Fix
Modified the generated encoder in `uavcan.protocol.dynamic_node_id.Allocation.h` to:
1. Check if we're encoding from bit offset 0 (first byte)
2. If so, directly place node_id in bits 0-6 and first_part in bit 7
3. Skip the canardEncodeScalar shifting for this special case

```c
// Direct placement for first byte
buffer[0] = (msg->node_id & 0x7F) | (msg->first_part_of_unique_id ? 0x80 : 0x00);
```

## Result
- For node_id=64, first_part=0: Now sends 0x40 (correct)
- Previously sent 0x80 (wrong due to left shift)
- Nodes can now correctly decode the allocation response

## Lessons Learned
1. The DroneCAN/UAVCAN v0 spec expects direct bit placement for single-byte packed fields
2. `canardEncodeScalar`'s MSB alignment is not always appropriate
3. Generated code may need adjustments for specific message types
4. Always verify bit-level encoding against what receivers expect