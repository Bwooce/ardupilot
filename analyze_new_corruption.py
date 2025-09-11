#!/usr/bin/env python3
"""
Analyze the new corruption: Message ID 9645760 
Raw header: fd 47 00 00 1d ca 16 c0 2e 93
"""

def analyze_corruption():
    print("=== New Corruption Analysis ===\n")
    
    # Message details from log
    corrupted_msgid = 9645760
    raw_header = [0xfd, 0x47, 0x00, 0x00, 0x1d, 0xca, 0x16, 0xc0, 0x2e, 0x93]
    
    print(f"Corrupted Message ID: {corrupted_msgid} (0x{corrupted_msgid:x})")
    
    # Analyze the message ID bytes
    msgid_bytes = corrupted_msgid.to_bytes(4, 'little')
    print(f"Message ID bytes (little-endian): {' '.join(f'0x{b:02x}' for b in msgid_bytes)}")
    
    # Check raw header structure
    print(f"\nRaw header analysis:")
    print(f"  Magic: 0x{raw_header[0]:02x} (should be 0xFD)")
    print(f"  Length: {raw_header[1]} bytes")
    print(f"  Incompat flags: 0x{raw_header[2]:02x}")
    print(f"  Compat flags: 0x{raw_header[3]:02x}")
    print(f"  Sequence: {raw_header[4]}")
    print(f"  System ID: {raw_header[5]}")
    print(f"  Component ID: {raw_header[6]}")
    
    # Message ID from header (bytes 7-9)
    header_msgid = (raw_header[9] << 16) | (raw_header[8] << 8) | raw_header[7]
    print(f"  Message ID from header: {header_msgid} (0x{header_msgid:06x})")
    print(f"  Message ID bytes: 0x{raw_header[7]:02x} 0x{raw_header[8]:02x} 0x{raw_header[9]:02x}")
    
    # Check if this matches the reported corrupted ID
    print(f"  Matches reported ID: {header_msgid == corrupted_msgid}")
    
    # Stack trace shows send_attitude(), so check ATTITUDE message ID
    attitude_msgid = 30  # ATTITUDE message ID
    print(f"\nExpected ATTITUDE message ID: {attitude_msgid} (0x{attitude_msgid:02x})")
    
    # Check for pattern in corruption
    print(f"\nCorruption pattern analysis:")
    print(f"  Expected: 0x{attitude_msgid:06x} (ATTITUDE)")
    print(f"  Got:      0x{header_msgid:06x}")
    print(f"  XOR:      0x{header_msgid ^ attitude_msgid:06x}")
    
    # Check individual bytes for known message IDs
    known_messages = {
        0: "HEARTBEAT",
        1: "SYS_STATUS", 
        30: "ATTITUDE",
        125: "POWER_STATUS",
        147: "BATTERY_STATUS",
        253: "STATUSTEXT"
    }
    
    for i, byte_val in enumerate([raw_header[7], raw_header[8], raw_header[9]]):
        if byte_val in known_messages:
            print(f"  Byte {i}: 0x{byte_val:02x} = {byte_val} ({known_messages[byte_val]})")
    
    # Check if our msgid=0 fix worked
    print(f"\nChecking if msgid padding fix worked:")
    if header_msgid & 0xFF000000 == 0:
        print("  ✓ High byte is clear (fix appears to work)")
    else:
        print(f"  ✗ High byte contaminated: 0x{(header_msgid >> 24) & 0xFF:02x}")

if __name__ == "__main__":
    analyze_corruption()