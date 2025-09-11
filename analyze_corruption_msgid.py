#!/usr/bin/env python3
"""
Analyze specific corruption patterns in MAVLink message IDs
Focus on UNKNOWN_205824 ("COMP") and UNKNOWN_300603 patterns
"""

def analyze_msgid_corruption():
    print("=== MAVLink Message ID Corruption Analysis ===\n")
    
    # Known corruption examples from user logs
    corruptions = [
        {"unknown_id": 205824, "content": "COMP", "description": "Likely COMPASS message corruption"},
        {"unknown_id": 300603, "content": "", "description": "Unknown corruption pattern"},
        {"unknown_id": 2359338, "content": "", "description": "Fixed: was HEARTBEAT(0) -> 0x240020A"},
        {"unknown_id": 5505149, "content": "", "description": "Previous corruption pattern"},
    ]
    
    # Analyze each corruption pattern
    for corruption in corruptions:
        unknown_id = corruption["unknown_id"]
        content = corruption["content"]
        desc = corruption["description"]
        
        print(f"Analyzing UNKNOWN_{unknown_id}:")
        print(f"  Description: {desc}")
        print(f"  Content: '{content}'" if content else "  Content: (no ASCII content provided)")
        
        # Convert to hex for bit analysis
        hex_val = hex(unknown_id)
        print(f"  Hex: {hex_val}")
        
        # Check for common MAVLink message IDs that could be corrupted
        print(f"  Possible sources:")
        
        # Check if it's a shifted/corrupted version of known message IDs
        common_msgids = {
            0: "HEARTBEAT",
            24: "GPS_RAW_INT", 
            30: "ATTITUDE",
            33: "GLOBAL_POSITION_INT",
            65: "RC_CHANNELS",
            87: "POSITION_TARGET_GLOBAL_INT",
            125: "POWER_STATUS",
            147: "BATTERY_STATUS",
            181: "LANDING_TARGET",
            253: "STATUSTEXT"
        }
        
        # Check for bit shifts, byte swaps, etc.
        for shift in range(1, 25):  # Check bit shifts
            shifted = unknown_id >> shift
            if shifted in common_msgids:
                print(f"    - Right shift {shift} bits: {shifted} ({common_msgids[shifted]})")
            
            shifted = unknown_id << shift
            if shifted < 1000000 and shifted in common_msgids:
                print(f"    - Left shift {shift} bits: {shifted} ({common_msgids[shifted]})")
        
        # Check for byte corruption patterns
        bytes_arr = unknown_id.to_bytes(4, 'little', signed=False)
        print(f"  Little-endian bytes: {' '.join(f'0x{b:02x}' for b in bytes_arr)}")
        
        # Check if any individual bytes match known message IDs
        for i, byte_val in enumerate(bytes_arr):
            if byte_val in common_msgids:
                print(f"    - Byte {i}: 0x{byte_val:02x} = {byte_val} ({common_msgids[byte_val]})")
        
        # For COMP content, check compass-related message IDs
        if content == "COMP":
            compass_msgs = {
                24: "GPS_RAW_INT (has compass heading)",
                30: "ATTITUDE (compass-related)", 
                147: "BATTERY_STATUS (might have compass sensor data)",
                33: "GLOBAL_POSITION_INT (has compass heading)"
            }
            print(f"  Possible compass-related messages:")
            for msgid, desc in compass_msgs.items():
                print(f"    - {msgid}: {desc}")
        
        print()

def analyze_memory_corruption():
    print("=== Memory Corruption Pattern Analysis ===\n")
    
    # Check if corruption follows specific patterns
    unknown_ids = [205824, 300603, 2359338, 5505149]
    
    print("Checking for patterns in corrupted IDs:")
    for i, id1 in enumerate(unknown_ids):
        for j, id2 in enumerate(unknown_ids[i+1:], i+1):
            xor = id1 ^ id2
            print(f"  {id1} XOR {id2} = {xor} (0x{xor:x})")
    
    print()
    
    # Check for common bit patterns
    print("Bit patterns in corrupted IDs:")
    for unknown_id in unknown_ids:
        binary = bin(unknown_id)[2:].zfill(32)
        print(f"  {unknown_id:>7}: {binary}")
        
        # Count bit positions that are set
        set_bits = [i for i, bit in enumerate(binary[::-1]) if bit == '1']
        print(f"           Set bits: {set_bits}")
    
    print()

if __name__ == "__main__":
    analyze_msgid_corruption()
    analyze_memory_corruption()