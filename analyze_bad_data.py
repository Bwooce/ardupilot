#!/usr/bin/env python3
"""
Analyze BAD_DATA messages from MAVLink connection
"""

import time
import struct
from pymavlink import mavutil

def analyze_bad_data():
    print("=== MAVLink BAD_DATA Analysis ===")
    
    try:
        # Use robust_parsing to try to recover from bad data
        connection = mavutil.mavlink_connection('/dev/cu.usbserial-0001', baud=460800, 
                                                autoreconnect=True, robust_parsing=True)
        print("Connected to ESP32 at 460800 baud")
    except Exception as e:
        print(f"Failed to connect: {e}")
        return
    
    print("Monitoring for 30 seconds...")
    start_time = time.time()
    
    stats = {
        'good_messages': {},
        'bad_data_count': 0,
        'bad_data_samples': [],
        'total_bytes': 0,
        'sync_errors': 0,
        'crc_errors': 0,
        'statustext_count': 0
    }
    
    # Also track raw bytes to look for patterns
    raw_buffer = bytearray()
    
    while time.time() - start_time < 30:
        # Try to read raw bytes
        try:
            raw_bytes = connection.port.read(1024)
            if raw_bytes:
                stats['total_bytes'] += len(raw_bytes)
                raw_buffer.extend(raw_bytes)
                
                # Keep buffer size manageable
                if len(raw_buffer) > 10000:
                    raw_buffer = raw_buffer[-5000:]
        except:
            pass
        
        # Process MAVLink messages
        msg = connection.recv_match(blocking=False, timeout=0.01)
        if msg:
            msg_type = msg.get_type()
            
            if msg_type == 'BAD_DATA':
                stats['bad_data_count'] += 1
                
                # Capture first 10 bad data samples
                if len(stats['bad_data_samples']) < 10:
                    # Try to get the raw data that caused this
                    if hasattr(msg, 'data'):
                        stats['bad_data_samples'].append(msg.data[:50])  # First 50 bytes
                    else:
                        stats['bad_data_samples'].append(b'<no data available>')
                        
                # Check MAVLink parser state
                if hasattr(connection.mav, 'total_packets_sent'):
                    if stats['bad_data_count'] % 50 == 1:  # Every 50th bad data
                        print(f"  BAD_DATA #{stats['bad_data_count']}: "
                              f"parser state={connection.mav.parse_state if hasattr(connection.mav, 'parse_state') else 'unknown'}")
                        
            elif msg_type == 'STATUSTEXT':
                stats['statustext_count'] += 1
                severity_map = {
                    0: "EMERGENCY", 1: "ALERT", 2: "CRITICAL", 3: "ERROR",
                    4: "WARNING", 5: "NOTICE", 6: "INFO", 7: "DEBUG"
                }
                severity_name = severity_map.get(msg.severity, f"UNKNOWN({msg.severity})")
                print(f"  STATUSTEXT: [{severity_name}] {msg.text}")
            else:
                # Good message
                if msg_type in stats['good_messages']:
                    stats['good_messages'][msg_type] += 1
                else:
                    stats['good_messages'][msg_type] = 1
    
    # Analyze results
    print(f"\n=== Analysis Results ===")
    print(f"Total bytes received: {stats['total_bytes']}")
    print(f"BAD_DATA count: {stats['bad_data_count']}")
    print(f"STATUSTEXT count: {stats['statustext_count']}")
    print(f"Good message types: {len(stats['good_messages'])}")
    
    # Calculate bad data ratio
    total_messages = stats['bad_data_count'] + sum(stats['good_messages'].values())
    if total_messages > 0:
        bad_ratio = (stats['bad_data_count'] / total_messages) * 100
        print(f"Bad data ratio: {bad_ratio:.1f}%")
    
    # Analyze bad data samples
    if stats['bad_data_samples']:
        print(f"\n=== Bad Data Samples (first {len(stats['bad_data_samples'])}) ===")
        for i, sample in enumerate(stats['bad_data_samples'], 1):
            if isinstance(sample, bytes):
                # Show hex and try to decode as ASCII
                hex_str = ' '.join(f'{b:02x}' for b in sample[:20])
                ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in sample[:20])
                print(f"  Sample {i}: {hex_str}")
                print(f"           ASCII: {ascii_str}")
                
                # Check for common patterns
                if all(b == 0x00 for b in sample[:10]):
                    print(f"           Pattern: All zeros (idle/padding)")
                elif all(b == 0xFF for b in sample[:10]):
                    print(f"           Pattern: All 0xFF (no data)")
                elif sample.startswith(b'\xFE') or sample.startswith(b'\xFD'):
                    print(f"           Pattern: Looks like MAVLink but corrupted")
    
    # Look for ELRS padding patterns in raw buffer
    print(f"\n=== Raw Buffer Analysis ===")
    if raw_buffer:
        # Count consecutive identical bytes (common in padding)
        consecutive_counts = {}
        prev_byte = None
        count = 0
        
        for byte in raw_buffer[-1000:]:  # Check last 1000 bytes
            if byte == prev_byte:
                count += 1
            else:
                if prev_byte is not None and count > 10:
                    if prev_byte not in consecutive_counts:
                        consecutive_counts[prev_byte] = []
                    consecutive_counts[prev_byte].append(count)
                prev_byte = byte
                count = 1
        
        if consecutive_counts:
            print("  Found padding patterns:")
            for byte_val, counts in consecutive_counts.items():
                max_count = max(counts)
                print(f"    Byte 0x{byte_val:02X}: max run of {max_count} consecutive bytes")
    
    # Show top message types
    if stats['good_messages']:
        print(f"\n=== Top 10 Good Message Types ===")
        sorted_msgs = sorted(stats['good_messages'].items(), key=lambda x: x[1], reverse=True)
        for msg_type, count in sorted_msgs[:10]:
            print(f"  {msg_type}: {count}")
    
    connection.close()

if __name__ == "__main__":
    analyze_bad_data()