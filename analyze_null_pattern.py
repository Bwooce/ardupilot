#!/usr/bin/env python3
"""
Analyze where null bytes appear relative to MAVLink frames
"""

import serial
import time

def analyze_nulls():
    print("=== Null Byte Pattern Analysis ===")
    
    port = serial.Serial('/dev/cu.usbserial-0001', 4800, timeout=0.1)
    print("Capturing 5 seconds of data...\n")
    
    # Capture raw data
    raw_data = bytearray()
    start = time.time()
    while time.time() - start < 5:
        chunk = port.read(1024)
        if chunk:
            raw_data.extend(chunk)
    
    print(f"Captured {len(raw_data)} bytes\n")
    
    # Find MAVLink frames and analyze null positions
    frames = []
    i = 0
    while i < len(raw_data) - 10:
        if raw_data[i] == 0xFD:  # MAVLink v2
            payload_len = raw_data[i+1] if i+1 < len(raw_data) else 0
            frame_len = payload_len + 12 + 2  # header + payload + checksum
            
            if i + frame_len <= len(raw_data):
                frames.append({
                    'start': i,
                    'end': i + frame_len,
                    'len': frame_len,
                    'payload_len': payload_len
                })
                i += frame_len
            else:
                i += 1
        else:
            i += 1
    
    print(f"Found {len(frames)} MAVLink frames\n")
    
    # Analyze gaps between frames
    print("=== Inter-frame Analysis ===")
    gaps = []
    for i in range(len(frames) - 1):
        gap_start = frames[i]['end']
        gap_end = frames[i+1]['start']
        gap_size = gap_end - gap_start
        
        if gap_size > 0:
            gap_data = raw_data[gap_start:gap_end]
            null_count = gap_data.count(0x00)
            gaps.append({
                'size': gap_size,
                'nulls': null_count,
                'percent': (null_count/gap_size)*100 if gap_size > 0 else 0
            })
    
    if gaps:
        avg_gap = sum(g['size'] for g in gaps) / len(gaps)
        avg_null_percent = sum(g['percent'] for g in gaps) / len(gaps)
        
        print(f"Gaps between frames: {len(gaps)}")
        print(f"Average gap size: {avg_gap:.1f} bytes")
        print(f"Average null percentage in gaps: {avg_null_percent:.1f}%")
        
        # Show distribution
        print("\nGap size distribution:")
        for size in [0, 10, 20, 50, 100, 200]:
            count = sum(1 for g in gaps if g['size'] >= size)
            print(f"  >= {size:3d} bytes: {count} gaps")
        
        # Check if gaps are mostly nulls
        mostly_null_gaps = sum(1 for g in gaps if g['percent'] > 80)
        print(f"\nGaps that are >80% nulls: {mostly_null_gaps}/{len(gaps)}")
    
    # Check for nulls within frames (should be rare)
    print("\n=== Intra-frame Analysis ===")
    nulls_in_frames = 0
    for frame in frames[:10]:  # Check first 10 frames
        frame_data = raw_data[frame['start']:frame['end']]
        null_count = frame_data.count(0x00)
        if null_count > 0:
            nulls_in_frames += null_count
            # Nulls in payload are normal, but not in header
            header_nulls = frame_data[:10].count(0x00)
            if header_nulls > 2:  # sys_id, comp_id might be 0
                print(f"  WARNING: Frame at {frame['start']} has {header_nulls} nulls in header!")
    
    print(f"Total nulls within first 10 frames: {nulls_in_frames}")
    
    # Show a visual representation
    print("\n=== Visual Pattern (first 500 bytes) ===")
    print("Legend: M=MAVLink start, .=null, x=other")
    visual = []
    for i in range(min(500, len(raw_data))):
        if raw_data[i] == 0xFD:
            visual.append('M')
        elif raw_data[i] == 0x00:
            visual.append('.')
        else:
            visual.append('x')
    
    # Print in lines of 80 chars
    for i in range(0, len(visual), 80):
        print(''.join(visual[i:i+80]))
    
    port.close()
    
    print("\n=== Conclusion ===")
    if gaps and avg_null_percent > 70:
        print("✓ Nulls are BETWEEN frames (ELRS padding)")
        print("  ELRS is adding padding between MAVLink messages")
        print("  This is normal for RF protocols but wasteful")
        print("\nRecommendations:")
        print("  1. Check ELRS serial rate matches ArduPilot rate (460800)")
        print("  2. Increase ELRS packet rate if possible")
        print("  3. Consider MAVLink message rate tuning")
    else:
        print("✗ Nulls are mixed with data (possible corruption)")

if __name__ == "__main__":
    analyze_nulls()
