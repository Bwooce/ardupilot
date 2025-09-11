#!/usr/bin/env python3
"""
Analyze the specific corruption pattern to find commonality
"""

def analyze_message_id_corruption():
    # Corrupted message IDs from the user's output
    corrupted_messages = [
        {
            'msgid': 5194496,
            'msgid_hex': '0x4F4300', 
            'raw_bytes': 'fd19000024032700434f4d504153535f',
            'payload_ascii': 'MPASS_',
            'likely_param': 'COMPASS_*'
        },
        {
            'msgid': 5194496,  # Same ID, different system/component
            'msgid_hex': '0x4F4300',
            'raw_bytes': 'fd190000b8035900434f4d504153535f', 
            'payload_ascii': 'MPASS_',
            'likely_param': 'COMPASS_*'
        },
        {
            'msgid': 4543233,
            'msgid_hex': '0x455301',
            'raw_bytes': 'fd1900000b010101534552564f315f4d',
            'payload_ascii': 'RVO1_M',
            'likely_param': 'SERVO1_*'
        },
        {
            'msgid': 4543254,
            'msgid_hex': '0x455316', 
            'raw_bytes': 'fd1900003e010116534552564f31315f',
            'payload_ascii': 'RVO11_',
            'likely_param': 'SERVO11_*'
        },
        {
            'msgid': 6247503,
            'msgid_hex': '0x5F544F',
            'raw_bytes': 'fd1900006701484f545f455343000214',
            'payload_ascii': 'ESC ',
            'likely_param': 'HOT_ESC*'
        }
    ]
    
    print("=== Analysis of Corrupted Message IDs ===\n")
    
    # Extract the corrupted bytes that became message IDs
    print("Corrupted Message ID bytes (should be 0x000016 for PARAM_VALUE):")
    for msg in corrupted_messages:
        msgid_bytes = msg['msgid'].to_bytes(3, 'little')
        print(f"  {msg['msgid_hex']}: {msgid_bytes} -> '{msgid_bytes.decode('ascii', errors='replace')}'")
        print(f"    Likely from parameter: {msg['likely_param']}")
    
    print("\n=== Pattern Analysis ===")
    
    # Look for patterns in parameter names
    param_prefixes = []
    for msg in corrupted_messages:
        param = msg['likely_param']
        if param.endswith('*'):
            prefix = param[:-1]
            param_prefixes.append(prefix)
    
    print(f"Parameter prefixes affected: {param_prefixes}")
    
    # Check if these are long parameter names
    print("\nParameter name length analysis:")
    for prefix in set(param_prefixes):
        print(f"  {prefix}: {len(prefix)} characters")
        if len(prefix) >= 14:  # Close to 16-byte limit
            print(f"    *** {prefix} is {len(prefix)} chars - very close to 16-byte limit! ***")
        elif len(prefix) >= 12:
            print(f"    *** {prefix} is {len(prefix)} chars - getting close to 16-byte limit ***")
    
    # The key insight: these might be parameters whose names are exactly 16 bytes
    # or close to it, causing issues with null termination
    print("\n=== HYPOTHESIS ===")
    print("Corrupted messages all seem to involve parameter names that are:")
    print("1. Long parameter names (12+ characters)")  
    print("2. Possibly reaching or exceeding the 16-byte param_id limit")
    print("3. May have null termination issues")
    
    print("\nSpecific parameter families affected:")
    print("- COMPASS_* parameters (compass configuration)")
    print("- SERVO1_*, SERVO11_* parameters (servo configuration)")  
    print("- HOT_ESC* parameters (ESC temperature monitoring)")
    
    print("\nCommonality: These are all **configuration parameters with long names**")
    print("that may be causing buffer overflow in the 16-byte param_id field!")

if __name__ == "__main__":
    analyze_message_id_corruption()