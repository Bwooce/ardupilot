#!/usr/bin/env python3
"""
Raw serial byte monitor
Shows hex and ASCII of all serial data
"""

import serial
import time
import sys
import glob

def monitor_raw(port="/dev/cu.usbmodem*", baudrate=115200, duration=10):
    # Find device
    devices = glob.glob(port)
    if not devices:
        print(f"No device found matching {port}")
        sys.exit(1)
    port = devices[0]
    
    print(f"Opening {port} at {baudrate} baud...")
    ser = serial.Serial(port, baudrate, timeout=0.1)
    
    print(f"Monitoring for {duration} seconds...")
    print("=" * 80)
    
    total_bytes = 0
    start_time = time.time()
    buffer = bytearray()
    
    while time.time() - start_time < duration:
        data = ser.read(1000)
        if data:
            total_bytes += len(data)
            buffer.extend(data)
            
            # Print hex and ASCII side by side
            for i in range(0, len(data), 32):
                chunk = data[i:i+32]
                hex_str = ' '.join(f'{b:02x}' for b in chunk)
                ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
                print(f"{total_bytes-len(data)+i:06x}: {hex_str:<96} {ascii_str}")
    
    print("=" * 80)
    print(f"\nTotal bytes received: {total_bytes}")
    
    # Protocol detection
    if buffer:
        print("\nProtocol signatures detected:")
        if b'\xfe' in buffer:
            print(f"  MAVLink v1 (0xFE): {buffer.count(0xfe)} occurrences")
        if b'\xfd' in buffer:
            print(f"  MAVLink v2 (0xFD): {buffer.count(0xfd)} occurrences")
        if b'\xc8' in buffer:
            print(f"  CRSF (0xC8): {buffer.count(0xc8)} occurrences")
        if b'$' in buffer:
            print(f"  MSP ($): {buffer.count(ord('$'))} occurrences")
        
        # Check for text patterns
        ascii_percent = sum(1 for b in buffer if 32 <= b < 127) / len(buffer) * 100
        print(f"\nASCII printable: {ascii_percent:.1f}%")
        
        if ascii_percent > 50:
            print("\nLikely text content detected. Lines:")
            text = buffer.decode('ascii', errors='ignore')
            for line in text.split('\n')[:10]:
                line = line.strip()
                if line:
                    print(f"  > {line}")
    
    ser.close()

if __name__ == "__main__":
    baudrate = int(sys.argv[1]) if len(sys.argv) > 1 else 115200
    duration = int(sys.argv[2]) if len(sys.argv) > 2 else 10
    
    print("=" * 80)
    print("Raw Serial Monitor")
    print("=" * 80)
    
    monitor_raw("/dev/cu.usbmodem*", baudrate, duration)