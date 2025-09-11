#!/usr/bin/env python3

# Quick analysis of the latest corruption
msgid = 9614016
print(f"Message ID: {msgid} (0x{msgid:06x})")
print(f"Hex: {msgid:x}")

# Check if this could be a shifted/corrupted known message ID
for shift in [8, 16, 24]:
    shifted = msgid >> shift
    if shifted < 300:
        print(f"Possible source: {shifted} (right shift {shift} bits)")

# Look for patterns in the raw header
header = "fd 47 00 00 6d ca 16 c0 b2 92"
print(f"Raw header: {header}")

# The message ID in the header appears as "16 c0" which is 0xc016 = 49174
# But we're seeing 9614016 which suggests memory corruption during processing
print("The corruption is happening during message processing, not in the raw header")