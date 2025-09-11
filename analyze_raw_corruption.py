#!/usr/bin/env python3
"""
Analyze the raw corruption data from the debug output
"""

def analyze_corruption(raw_hex, expected_msgid=22):
    """Analyze a corrupted MAVLink packet"""
    # Convert hex string to bytes
    raw_bytes = bytes.fromhex(raw_hex.replace(' ', ''))
    
    if len(raw_bytes) < 12:
        print(f"Packet too short: {len(raw_bytes)} bytes")
        return
    
    # Parse MAVLink v2 header
    if raw_bytes[0] != 0xFD:
        print(f"Not MAVLink v2: start byte 0x{raw_bytes[0]:02X}")
        return
    
    payload_len = raw_bytes[1]
    incompat_flags = raw_bytes[2]
    compat_flags = raw_bytes[3]
    seq = raw_bytes[4]
    sysid = raw_bytes[5]
    compid = raw_bytes[6]
    
    # Message ID is 3 bytes little endian
    msgid = int.from_bytes(raw_bytes[7:10], 'little')
    
    print(f"Raw hex: {raw_hex}")
    print(f"Parsed header:")
    print(f"  Start: 0x{raw_bytes[0]:02X}")
    print(f"  Payload length: {payload_len}")
    print(f"  Flags: incompat=0x{incompat_flags:02X}, compat=0x{compat_flags:02X}")
    print(f"  Sequence: {seq}")
    print(f"  System ID: {sysid}")
    print(f"  Component ID: {compid}")
    print(f"  Message ID: {msgid} (0x{msgid:06X})")
    print(f"  Expected: {expected_msgid} (0x{expected_msgid:06X})")
    
    # Show payload
    payload = raw_bytes[10:10+payload_len]
    print(f"  Payload ({len(payload)} bytes): {payload.hex()}")
    
    # Try to interpret as ASCII
    try:
        ascii_part = payload[:16].decode('ascii', errors='replace')
        print(f"  Payload as ASCII: '{ascii_part}'")
    except:
        pass
    
    # If this should be PARAM_VALUE (22), check if payload looks like param data
    if expected_msgid == 22 and len(payload) >= 25:
        # PARAM_VALUE structure: float(4) + uint16(2) + uint16(2) + char[16](16) + uint8(1) = 25 bytes
        param_value = int.from_bytes(payload[0:4], 'little')  # Interpret as int for now
        param_count = int.from_bytes(payload[4:6], 'little')
        param_index = int.from_bytes(payload[6:8], 'little')
        param_id_bytes = payload[8:24]
        param_type = payload[24] if len(payload) > 24 else 0
        
        # Try to decode parameter name
        try:
            param_id = param_id_bytes.decode('ascii', errors='replace').rstrip('\x00')
        except:
            param_id = param_id_bytes.hex()
        
        print(f"  If PARAM_VALUE:")
        print(f"    Value: 0x{param_value:08X}")
        print(f"    Count: {param_count}")
        print(f"    Index: {param_index}")
        print(f"    Param ID: '{param_id}'")
        print(f"    Type: {param_type}")
    
    print()

# Analyze the corruption examples from the user
print("=== Analyzing corruption examples ===\n")

# Example 1: UNKNOWN_5194496
analyze_corruption("fd19000024032700434f4d504153535f")

# Example 2: UNKNOWN_5194496 (different compid)
analyze_corruption("fd190000b8035900434f4d504153535f")

# Example 3: UNKNOWN_4543233
analyze_corruption("fd1900000b010101534552564f315f4d")

# Example 4: UNKNOWN_4543254
analyze_corruption("fd1900003e010116534552564f31315f")

# Example 5: UNKNOWN_6247503
analyze_corruption("fd1900006701484f545f455343000214")