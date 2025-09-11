#!/usr/bin/env python3
"""
Check mLRS stats including TX packet counts
"""

import serial
import time
import sys
import glob

def check_stats(port="/dev/cu.usbmodem*", baudrate=115200):
    devices = glob.glob(port)
    if not devices:
        print("No device found!")
        return
    port = devices[0]
    
    ser = serial.Serial(port, baudrate, timeout=1)
    
    # Send exit to ensure we're in CLI mode
    ser.write(b"exit\r")
    time.sleep(0.5)
    ser.reset_input_buffer()
    
    # Send empty line to get into CLI
    ser.write(b"\r")
    time.sleep(0.2)
    response = ser.read(100)
    
    print("Getting mLRS statistics...")
    print("=" * 60)
    
    # Get version
    ser.write(b"v\r")
    time.sleep(0.3)
    response = ser.read(500)
    if response:
        text = response.decode('ascii', errors='ignore')
        for line in text.split('\n'):
            if 'Tx:' in line or 'Rx:' in line or 'v1.' in line:
                print(line.strip())
    
    # Get parameter list to check config
    print("\nChecking configuration...")
    ser.write(b"pl tx\r")
    time.sleep(0.5)
    response = ser.read(2000)
    if response:
        text = response.decode('ascii', errors='ignore')
        for line in text.split('\n'):
            if 'Ser Dest' in line or 'Ser Baudrate' in line or 'Mav Component' in line:
                print(f"  {line.strip()}")
    
    # Try stats command
    print("\nRequesting live statistics...")
    ser.write(b"stats\r")
    
    # Collect stats for 5 seconds
    stats_lines = []
    start = time.time()
    while time.time() - start < 5:
        data = ser.read(500)
        if data:
            text = data.decode('ascii', errors='ignore')
            lines = text.replace('\r', '').split('\n')
            for line in lines:
                if line and '|' in line and line not in stats_lines:
                    stats_lines.append(line)
                    # Parse the stats line
                    parts = line.split('|')
                    if len(parts) >= 4:
                        print(f"\nStats update:")
                        for part in parts:
                            part = part.strip()
                            if 'LQ' in part:
                                print(f"  Link Quality: {part}")
                            elif 'tx' in part.lower():
                                print(f"  TX: {part}")
                            elif 'rx' in part.lower():
                                print(f"  RX: {part}")
                            elif 'bytes' in part.lower() or 'pkt' in part.lower():
                                print(f"  Data: {part}")
        time.sleep(0.1)
    
    # Stop stats
    ser.write(b"\r")
    time.sleep(0.2)
    
    # Check for connected state
    print("\n" + "=" * 60)
    print("Analysis:")
    
    if stats_lines:
        # Look for packet counts
        has_tx = any('tx' in line.lower() for line in stats_lines)
        has_rx = any('rx' in line.lower() for line in stats_lines)
        
        if has_tx and has_rx:
            print("✓ Bidirectional communication detected")
        elif has_rx:
            print("⚠ Only receiving packets (no TX)")
        elif has_tx:
            print("⚠ Only transmitting packets (no RX)")
        else:
            print("✗ No packet activity detected")
    else:
        print("✗ Could not get statistics")
        print("  mLRS might not be in the right mode")
    
    ser.close()

if __name__ == "__main__":
    print("=" * 60)
    print("mLRS Stats Check")
    print("=" * 60)
    
    check_stats("/dev/cu.usbmodem*", 115200)
