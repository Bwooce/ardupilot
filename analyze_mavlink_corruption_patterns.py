#!/usr/bin/env python3
"""
Analyze corrupted MAVLink message bytes for ASCII and offset patterns
"""

import struct

def analyze_corruption(header_hex, msg_id, description=""):
    """Analyze a corrupted MAVLink header"""
    print(f"\n{'='*60}")
    print(f"Corruption: {description}")
    print(f"Raw header: {header_hex}")
    print(f"Message ID: {msg_id} (0x{msg_id:06X})")
    
    # Parse the header bytes
    header_bytes = bytes.fromhex(header_hex.replace(" ", ""))
    
    print(f"\nByte breakdown:")
    print(f"  [0] Magic: 0x{header_bytes[0]:02X} ({'MAVLink v2' if header_bytes[0] == 0xFD else 'INVALID'})")
    print(f"  [1-2] Payload len: {struct.unpack('<H', header_bytes[1:3])[0]} bytes")
    print(f"  [3] Incompat flags: 0x{header_bytes[3]:02X}")
    print(f"  [4] Compat flags: 0x{header_bytes[4]:02X}")
    print(f"  [5] Sequence: {header_bytes[5]}")
    print(f"  [6] System ID: {header_bytes[6]} (0x{header_bytes[6]:02X})")
    print(f"  [7] Component ID: {header_bytes[7]} (0x{header_bytes[7]:02X})")
    if len(header_bytes) >= 11:
        print(f"  [8-10] Message ID: {struct.unpack('<I', header_bytes[8:11] + b'\x00')[0]}")
    elif len(header_bytes) >= 10:
        print(f"  [8-9] Partial Message ID bytes: {header_bytes[8]:02X} {header_bytes[9]:02X}")
    
    # Check for ASCII characters
    print(f"\nASCII analysis:")
    for i, byte in enumerate(header_bytes):
        if 0x20 <= byte <= 0x7E:  # Printable ASCII range
            print(f"  [{i}] 0x{byte:02X} = '{chr(byte)}' (ASCII)")
    
    # Look for potential pointers or offsets
    print(f"\nPotential patterns:")
    
    # Check if bytes 6-9 could be a memory address or pointer
    if len(header_bytes) >= 10:
        # Little-endian 32-bit value from bytes 6-9
        potential_addr = struct.unpack('<I', header_bytes[6:10])[0]
        print(f"  Bytes [6-9] as uint32: 0x{potential_addr:08X} ({potential_addr})")
        
        # Check if it looks like an ESP32 memory address
        if 0x3FC00000 <= potential_addr <= 0x3FCFFFFF:
            print(f"    -> Looks like ESP32 DRAM address!")
        elif 0x42000000 <= potential_addr <= 0x42FFFFFF:
            print(f"    -> Looks like ESP32 IRAM/Flash address!")
        elif 0x40370000 <= potential_addr <= 0x403DFFFF:
            print(f"    -> Looks like ESP32 ROM address!")
            
    # Check for repeating patterns
    for i in range(len(header_bytes) - 1):
        if header_bytes[i] == header_bytes[i+1]:
            print(f"  Repeating byte at [{i}]-[{i+1}]: 0x{header_bytes[i]:02X}")
    
    # Check if any bytes look like offsets into a buffer
    for i, byte in enumerate(header_bytes[6:], 6):  # Start from corrupted part
        if byte < 32:
            print(f"  [{i}] Small value 0x{byte:02X} ({byte}) - could be offset/length")

# Analyze the corruptions from the log
print("MAVLink Corruption Analysis")
print("="*60)

# From the current log
analyze_corruption("FD 61 00 00 65 C1 18 C0 F0 D3", 13889728, "Current corruption #1")

# From historical logs mentioned
analyze_corruption("fd 47 00 00 6d ca 16 c0 b2 92", 9614016, "Historical corruption #1")
analyze_corruption("fd eb 1f ba dc bc 1f 3a 01 ee", 15597882, "Historical corruption #2")
analyze_corruption("fd e5 04 00 dc 05 00 00 dc 05", 384000, "Historical corruption #3")

# Additional analysis
print("\n" + "="*60)
print("PATTERN ANALYSIS:")
print("="*60)

# Check what valid ATTITUDE message should look like
print("\nExpected ATTITUDE message pattern:")
print("  Magic: 0xFD (MAVLink v2)")
print("  Payload: 28 bytes (0x1C 0x00 in little-endian)")
print("  System ID: Usually 1-255")
print("  Component ID: Usually 1-250")  
print("  Message ID: 30 (0x1E 0x00 0x00 in little-endian)")

# Look for specific patterns in corrupted data
corrupted_sections = [
    ("18 C0 F0 D3", "Current #1 bytes [6-9]"),
    ("16 c0 b2 92", "Historical #1 bytes [6-9]"),
    ("1f 3a 01 ee", "Historical #2 bytes [6-9]"),
    ("00 00 dc 05", "Historical #3 bytes [6-9]"),
]

print("\nCorrupted section analysis (where sysid/compid/msgid should be):")
for hex_str, desc in corrupted_sections:
    bytes_data = bytes.fromhex(hex_str.replace(" ", ""))
    print(f"\n{desc}: {hex_str}")
    
    # Check for ASCII
    ascii_chars = []
    for b in bytes_data:
        if 0x20 <= b <= 0x7E:
            ascii_chars.append(chr(b))
        else:
            ascii_chars.append('.')
    print(f"  ASCII: '{''.join(ascii_chars)}'")
    
    # As 32-bit value
    val32 = struct.unpack('<I', bytes_data)[0]
    print(f"  As uint32: 0x{val32:08X} ({val32})")
    
    # Check if it's a plausible memory address
    if 0x3FC00000 <= val32 <= 0x3FCFFFFF:
        print(f"    -> ESP32 DRAM region (0x3FC00000-0x3FCFFFFF)")
    elif 0x42000000 <= val32 <= 0x42FFFFFF:
        print(f"    -> ESP32 Flash/IRAM region (0x42000000-0x42FFFFFF)")
    elif 0x40370000 <= val32 <= 0x403DFFFF:
        print(f"    -> ESP32 ROM region (0x40370000-0x403DFFFF)")
    
    # Check if first two bytes could be a counter/offset
    if len(bytes_data) >= 2:
        val16 = struct.unpack('<H', bytes_data[:2])[0]
        print(f"  First 2 bytes as uint16: {val16} (0x{val16:04X})")