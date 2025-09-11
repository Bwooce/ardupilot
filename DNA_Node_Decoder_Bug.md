# DNA Allocation Node Decoder Bug Analysis

## Summary
The remote node is incorrectly decoding DroneCAN DNA allocation responses due to not properly handling bit-aligned fields per the DroneCAN/UAVCAN specification.

## The Issue
When ArduPilot sends a DNA allocation response with:
- node_id = 64 (0x40)
- first_part_of_unique_id = 0

The encoded first byte is 0x80, but the node decodes this as:
- node_id = 0
- first_part_of_unique_id = 1

## Root Cause
The DroneCAN/UAVCAN protocol uses bit-aligned encoding. For fields that don't align to byte boundaries:

1. **Encoding**: Values are left-shifted to align with the MSB side of the byte
2. **Decoding**: Values must be right-shifted back to get the original value

For a 7-bit node_id field:
- Raw value: 64 = 0x40 = 0b01000000
- After encoding (left shift by 1): 0x80 = 0b10000000
- After decoding (right shift by 1): 0x40 = 0b01000000 = 64

## The Bug
The remote node is directly reading the byte without performing the required right-shift operation. This causes:
- Byte 0x80 to be interpreted as node_id=0, first_part=1
- Instead of the correct node_id=64, first_part=0

## Evidence
```
[INFO] ALLOC: Raw byte[0]=0x80, Decoded: node_id=0, first_part=1
[ERROR] ALLOC: Invalid node ID 0 in allocation response!
```

The node is reading bits directly:
- Bits 0-6 of 0x80 = 0b0000000 = 0 (incorrect)
- Bit 7 of 0x80 = 0b1 = 1 (incorrect)

Instead of using canardDecodeScalar which would:
- Right-shift 0x80 by 1 bit to get 0x40
- Then read bits 0-6 as 64 (correct)
- And bit 7 as 0 (correct)

## Solution
The remote node needs to:
1. Use the standard canardDecodeScalar function from libcanard
2. Or if using custom decoding, implement the proper bit alignment shifts

## Verification
Our encoding is correct as verified by:
1. Manual encoding test produces expected values
2. The encoded bytes match the DroneCAN specification
3. Standard canardDecodeScalar would correctly decode our messages

## Workaround Options
1. **Not recommended**: Modify canardEncodeScalar to skip shifting for 7-bit values (breaks spec compliance)
2. **Better**: Have the node fix their decoder to comply with DroneCAN specification
3. **Alternative**: Use a different node ID that doesn't have bit 6 set (e.g., IDs below 64)