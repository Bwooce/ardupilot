#!/usr/bin/env python3
"""
Diagnose serial corruption by analyzing raw byte stream
"""

import time
import serial
import struct

def find_mavlink_patterns(data):
    """Find MAVLink sync bytes and analyze structure"""
    patterns = {
        'mavlink_v1': [],  # 0xFE
        'mavlink_v2': [],  # 0xFD
        'corrupted': [],
        'valid_looking': []
    }
    
    for i in range(len(data) - 10):
        if data[i] == 0xFE:  # MAVLink v1
            patterns['mavlink_v1'].append(i)
            # Check if it looks valid
            if i + 8 < len(data):
                payload_len = data[i+1]
                if payload_len <= 255:  # Reasonable payload
                    patterns['valid_looking'].append((i, 'v1', payload_len))
                    
        elif data[i] == 0xFD:  # MAVLink v2
            patterns['mavlink_v2'].append(i)
            # Check if it looks valid
            if i + 12 < len(data):
                payload_len = data[i+1]
                incompat_flags = data[i+2]
                compat_flags = data[i+3]
                if payload_len <= 255 and incompat_flags <= 1:  # Reasonable values
                    patterns['valid_looking'].append((i, 'v2', payload_len))
                else:
                    patterns['corrupted'].append(i)
    
    return patterns

def analyze_corruption_pattern(data):
    """Look for specific corruption patterns"""
    analysis = {
        'bit_flips': 0,
        'byte_shifts': 0,
        'duplicates': 0,
        'insertions': 0
    }
    
    # Look for repeated sequences (might indicate duplication)
    for length in [8, 16, 32]:
        for i in range(len(data) - length * 2):
            if data[i:i+length] == data[i+length:i+length*2]:
                analysis['duplicates'] += 1
                break
    
    # Look for bit flips (bytes that differ by single bit)
    for i in range(len(data) - 1):
        xor = data[i] ^ data[i+1]
        # Check if XOR result is a power of 2 (single bit difference)
        if xor != 0 and (xor & (xor - 1)) == 0:
            analysis['bit_flips'] += 1
    
    return analysis

def diagnose_serial():
    print("=== Serial Corruption Diagnosis ===")
    
    try:
        port = serial.Serial('/dev/cu.usbserial-0001', 460800, timeout=0.1)
        print("Connected at 460800 baud")
    except Exception as e:
        print(f"Failed to connect: {e}")
        return
    
    print("Capturing 10 seconds of raw data...")
    
    # Capture raw data
    raw_chunks = []
    start_time = time.time()
    total_bytes = 0
    
    while time.time() - start_time < 10:
        chunk = port.read(1024)
        if chunk:
            raw_chunks.append(chunk)
            total_bytes += len(chunk)
            
            # Show progress
            if len(raw_chunks) % 10 == 0:
                print(f"  Captured {total_bytes} bytes...")
    
    # Combine all chunks
    raw_data = b''.join(raw_chunks)
    print(f"\nTotal captured: {len(raw_data)} bytes")
    
    # Find MAVLink patterns
    patterns = find_mavlink_patterns(raw_data)
    
    print(f"\n=== MAVLink Sync Bytes Found ===")
    print(f"MAVLink v1 (0xFE): {len(patterns['mavlink_v1'])} occurrences")
    print(f"MAVLink v2 (0xFD): {len(patterns['mavlink_v2'])} occurrences")
    print(f"Valid-looking frames: {len(patterns['valid_looking'])}")
    print(f"Corrupted frames: {len(patterns['corrupted'])}")
    
    # Show first few valid-looking frames
    if patterns['valid_looking']:
        print(f"\n=== First 5 Valid-Looking Frames ===")
        for i, (pos, version, payload_len) in enumerate(patterns['valid_looking'][:5]):
            frame_start = raw_data[pos:pos+20]
            hex_str = ' '.join(f'{b:02x}' for b in frame_start)
            print(f"  Frame {i+1} at byte {pos} ({version}, payload={payload_len}): {hex_str}")
            
            # Decode frame header
            if version == 'v2' and pos + 12 < len(raw_data):
                sync = frame_start[0]
                payload = frame_start[1]
                incompat = frame_start[2]
                compat = frame_start[3]
                seq = frame_start[4]
                sysid = frame_start[5]
                compid = frame_start[6]
                msgid = frame_start[7] | (frame_start[8] << 8) | (frame_start[9] << 16)
                print(f"       Decoded: sys={sysid} comp={compid} msgid={msgid} seq={seq}")
    
    # Analyze byte distribution
    print(f"\n=== Byte Distribution ===")
    byte_counts = {}
    for byte in raw_data:
        if byte not in byte_counts:
            byte_counts[byte] = 0
        byte_counts[byte] += 1
    
    # Show most common bytes
    sorted_bytes = sorted(byte_counts.items(), key=lambda x: x[1], reverse=True)
    print("Top 10 most common bytes:")
    for byte_val, count in sorted_bytes[:10]:
        percentage = (count / len(raw_data)) * 100
        char_repr = chr(byte_val) if 32 <= byte_val < 127 else '.'
        print(f"  0x{byte_val:02X} ('{char_repr}'): {count} times ({percentage:.1f}%)")
    
    # Check for specific patterns
    corruption = analyze_corruption_pattern(raw_data)
    print(f"\n=== Corruption Analysis ===")
    print(f"Duplicate sequences: {corruption['duplicates']}")
    print(f"Potential bit flips: {corruption['bit_flips']}")
    
    # Look for gaps between valid frames
    if len(patterns['valid_looking']) > 1:
        print(f"\n=== Frame Spacing ===")
        gaps = []
        for i in range(len(patterns['valid_looking']) - 1):
            gap = patterns['valid_looking'][i+1][0] - patterns['valid_looking'][i][0]
            gaps.append(gap)
        
        if gaps:
            avg_gap = sum(gaps) / len(gaps)
            min_gap = min(gaps)
            max_gap = max(gaps)
            print(f"Average gap between frames: {avg_gap:.1f} bytes")
            print(f"Min gap: {min_gap}, Max gap: {max_gap}")
            
            # Check if gaps are consistent (should be for periodic messages)
            if max_gap > avg_gap * 2:
                print("⚠️  Irregular gaps - possible data loss or corruption")
    
    # Check for ELRS-specific patterns
    print(f"\n=== ELRS Pattern Check ===")
    # ELRS might insert specific byte sequences
    elrs_patterns = [
        (b'\x00\x00\x00\x00', 'Null padding'),
        (b'\xFF\xFF\xFF\xFF', 'FF padding'),
        (b'\xC0\xC0', 'CRSF sync'),  # ELRS uses CRSF protocol
    ]
    
    for pattern, name in elrs_patterns:
        count = raw_data.count(pattern)
        if count > 0:
            print(f"  Found {name}: {count} occurrences")
    
    # Show a hex dump of suspicious areas
    print(f"\n=== Sample of Raw Data (first 200 bytes) ===")
    for i in range(0, min(200, len(raw_data)), 16):
        hex_str = ' '.join(f'{b:02x}' for b in raw_data[i:i+16])
        ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in raw_data[i:i+16])
        print(f"  {i:04x}: {hex_str:<48} |{ascii_str}|")
    
    port.close()
    print("\n=== Diagnosis Complete ===")
    
    # Final assessment
    if len(patterns['valid_looking']) == 0:
        print("\n❌ CRITICAL: No valid MAVLink frames found!")
        print("   Possible causes:")
        print("   - Wrong baud rate")
        print("   - Serial parameters mismatch (data bits, stop bits, parity)")
        print("   - Physical layer corruption")
    elif len(patterns['corrupted']) > len(patterns['valid_looking']):
        print("\n⚠️  WARNING: More corrupted frames than valid ones")
        print("   Likely causes:")
        print("   - Buffer overruns")
        print("   - DMA conflicts")
        print("   - Timing issues")
    elif corruption['duplicates'] > 10:
        print("\n⚠️  WARNING: Many duplicate sequences found")
        print("   Possible buffer management issue")

if __name__ == "__main__":
    diagnose_serial()