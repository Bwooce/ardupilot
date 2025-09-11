#!/usr/bin/env python3
"""
Analyze the UNKNOWN message IDs from the log to find patterns
"""

def analyze_unknown_messages():
    # UNKNOWN messages from the log
    unknown_messages = [
        4543233, 2359296, 13500797, 5791539, 2359318, 7471229, 4342094, 
        5111933, 5857100, 2425213, 4672607, 15925629, 21580, 9699709, 
        15860093, 5521735, 8192359, 9568637, 4543314, 12583293, 4522006, 
        4718973, 5521746, 3670788, 4411926, 5111830, 1704317, 5374464, 
        13304167, 2687357, 22849, 12648829, 13238653, 22593, 9961853, 
        10289511, 13566333
    ]
    
    # Known MAVLink message IDs to check against
    known_messages = {
        0: "HEARTBEAT",
        1: "SYS_STATUS", 
        2: "SYSTEM_TIME",
        24: "GPS_RAW_INT",
        27: "RAW_IMU",
        30: "ATTITUDE", 
        33: "GLOBAL_POSITION_INT",
        35: "RC_CHANNELS_SCALED",
        36: "SERVO_OUTPUT_RAW",
        65: "RC_CHANNELS",
        74: "VFR_HUD",
        87: "POSITION_TARGET_GLOBAL_INT",
        125: "POWER_STATUS",
        147: "BATTERY_STATUS",
        163: "HIGH_LATENCY",
        181: "LANDING_TARGET",
        235: "HIGH_LATENCY2",
        241: "VIBRATION",
        253: "STATUSTEXT",
        22: "PARAM_VALUE"
    }
    
    print("=== Analyzing UNKNOWN Message IDs ===\n")
    
    statustext_candidates = []
    servo_candidates = []
    
    for unknown_id in sorted(unknown_messages):
        print(f"UNKNOWN_{unknown_id} (0x{unknown_id:x}):")
        
        # Check bytes for known message IDs
        bytes_arr = unknown_id.to_bytes(4, 'little', signed=False)
        print(f"  Bytes: {' '.join(f'0x{b:02x}' for b in bytes_arr)}")
        
        found_matches = []
        for i, byte_val in enumerate(bytes_arr):
            if byte_val in known_messages:
                found_matches.append(f"Byte {i}={byte_val} ({known_messages[byte_val]})")
        
        if found_matches:
            print(f"  Sources: {', '.join(found_matches)}")
            
            # Collect candidates
            if any(b == 253 for b in bytes_arr):
                statustext_candidates.append(unknown_id)
                print(f"  *** STATUSTEXT CANDIDATE ***")
            
            if any(b == 36 for b in bytes_arr):
                servo_candidates.append(unknown_id)
                print(f"  *** SERVO_OUTPUT_RAW CANDIDATE ***")
        
        print()
    
    print(f"=== Summary ===")
    print(f"Likely corrupted STATUSTEXT messages: {len(statustext_candidates)}")
    print(f"Likely corrupted SERVO_OUTPUT_RAW messages: {len(servo_candidates)}")
    print(f"Total UNKNOWN messages: {len(unknown_messages)}")

if __name__ == "__main__":
    analyze_unknown_messages()
