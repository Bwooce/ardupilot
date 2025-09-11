#!/usr/bin/env python3
"""
Analyze the corrupted message IDs to understand the pattern
"""

# Raw packet data from debug output
packets = [
    ("fd19000083010048421703524f54544c", 1524296, "ROTTL"),
    ("fd190000ac01033000434f590000090f", 4390960, "COY"),
    ("fd19001600000000434f4d504153535f", 5194496, "COMPASS_"),
    ("fd1900000000000000ee010116000000", 15597568, ""),
    ("fd19000036000000803f3a0101160000", 4161536, ""),
    ("fd1900003b010100494e535f00090189", 5130496, "INS_"),
    ("fd190000550101c60041485200000997", 4260038, "AHR"),
    ("fd19000069010116c8421703da544852", 4376598, ""),
    ("fd19000083010180401703f35352565f", 1523840, ""),
    ("fd190000970100803f1703434f4e4649", 1523584, "CONFI"),
]

def analyze_packet(hex_data, reported_msgid, param_name):
    data = bytes.fromhex(hex_data)
    print(f"\nPacket with param '{param_name}' (reported msgid {reported_msgid}):")
    print(f"Raw: {hex_data}")
    
    if len(data) >= 12:
        # MAVLink v2 header breakdown
        magic = data[0]
        payload_len = data[1]
        incompat_flags = data[2]
        compat_flags = data[3]
        seq = data[4]
        sysid = data[5]
        compid = data[6]
        msgid_bytes = data[7:10]
        msgid = int.from_bytes(msgid_bytes, 'little')
        
        print(f"  Magic: 0x{magic:02x}")
        print(f"  Payload len: {payload_len}")
        print(f"  Seq: {seq}, SysID: {sysid}, CompID: {compid}")
        print(f"  MsgID bytes: {msgid_bytes.hex()}")
        print(f"  MsgID decoded: {msgid} (should be 22 for PARAM_VALUE)")
        print(f"  Reported MsgID: {reported_msgid}")
        
        # Check if this should be message ID 22 (PARAM_VALUE)
        correct_msgid = 22  # 0x16
        correct_bytes = correct_msgid.to_bytes(3, 'little')
        print(f"  Expected MsgID bytes: {correct_bytes.hex()}")
        
        # Compare what we got vs what we expected
        diff = []
        for i, (got, expected) in enumerate(zip(msgid_bytes, correct_bytes)):
            if got != expected:
                diff.append(f"byte{i}: got 0x{got:02x}, expected 0x{expected:02x}")
        
        if diff:
            print(f"  Corruption: {', '.join(diff)}")
        
        # Look at payload to verify this is a PARAM_VALUE
        if len(data) >= 12 + 4:  # header + at least param name start
            payload = data[10:10+payload_len]
            print(f"  Payload ({len(payload)} bytes): {payload[:16].hex()}")
            
            # Try to decode as PARAM_VALUE structure
            if len(payload) >= 25:  # PARAM_VALUE is 25 bytes
                param_value = int.from_bytes(payload[0:4], 'little', signed=False)
                param_count = int.from_bytes(payload[4:6], 'little')
                param_index = int.from_bytes(payload[6:8], 'little')
                param_id = payload[8:24].rstrip(b'\x00').decode('ascii', errors='ignore')
                param_type = payload[24]
                
                print(f"  Decoded PARAM_VALUE:")
                print(f"    Value: {param_value} (as uint32)")
                print(f"    Count: {param_count}")
                print(f"    Index: {param_index}")
                print(f"    Param ID: '{param_id}'")
                print(f"    Type: {param_type}")

print("=== Message ID Corruption Analysis ===")
for hex_data, reported_msgid, param_name in packets:
    analyze_packet(hex_data, reported_msgid, param_name)