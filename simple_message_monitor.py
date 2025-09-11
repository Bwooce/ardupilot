#!/usr/bin/env python3
"""
Simple MAVLink message monitor - shows all message types received
"""

import sys
import time
from datetime import datetime
from pymavlink import mavutil

def main():
    port = '/dev/cu.usbserial-0001'
    baud = 460800
    
    print(f"=== Simple Message Monitor ===")
    print(f"Connecting to {port} at {baud} baud...")
    
    try:
        master = mavutil.mavlink_connection(port, baud=baud, timeout=1)
        print("✓ Connected! Monitoring all messages...")
        print("Press Ctrl+C to exit\n")
        
        msg_counts = {}
        statustext_msgs = []
        start_time = time.time()
        
        while True:
            try:
                msg = master.recv_match(blocking=False)
                if msg is None:
                    time.sleep(0.01)
                    continue
                
                msg_type = msg.get_type()
                msg_counts[msg_type] = msg_counts.get(msg_type, 0) + 1
                
                # Special handling for STATUSTEXT
                if msg_type == 'STATUSTEXT':
                    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    text = msg.text.decode('utf-8', errors='replace').rstrip('\x00')
                    statustext_msgs.append(f"[{timestamp}] {text}")
                    print(f"✓ STATUSTEXT: {text}")
                
                # Special handling for UNKNOWN messages
                elif msg_type.startswith('UNKNOWN_'):
                    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    print(f"⚠ {msg_type} at {timestamp}")
                    
                    # Try to extract any readable text from raw data
                    if hasattr(msg, '_raw_data') and msg._raw_data:
                        try:
                            readable = ''.join(chr(b) if 32 <= b <= 126 else '.' for b in msg._raw_data[:50])
                            if any(c.isalpha() for c in readable):
                                print(f"   Raw text: {readable}")
                        except:
                            pass
                
                # Print summary every 50 messages
                total_msgs = sum(msg_counts.values())
                if total_msgs % 50 == 0:
                    print(f"\n--- Message Summary (Total: {total_msgs}) ---")
                    for msg_type, count in sorted(msg_counts.items(), key=lambda x: x[1], reverse=True)[:10]:
                        print(f"  {msg_type}: {count}")
                    print()
            
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error: {e}")
                continue
    
    except Exception as e:
        print(f"Connection failed: {e}")
        return 1
    
    # Final summary
    elapsed = time.time() - start_time
    total_msgs = sum(msg_counts.values())
    print(f"\n=== Final Summary ===")
    print(f"Total messages: {total_msgs}")
    print(f"Duration: {elapsed:.1f}s")
    print(f"Rate: {total_msgs/elapsed:.1f} msg/s")
    print(f"STATUSTEXT messages: {len(statustext_msgs)}")
    
    print(f"\nMessage types received:")
    for msg_type, count in sorted(msg_counts.items(), key=lambda x: x[1], reverse=True):
        print(f"  {msg_type}: {count}")
    
    if statustext_msgs:
        print(f"\nSTATUSTEXT messages:")
        for msg in statustext_msgs:
            print(f"  {msg}")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())