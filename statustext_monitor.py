#!/usr/bin/env python3
"""
ESP32 STATUSTEXT Monitor - Display STATUSTEXT messages with metadata
Simple script to monitor and display STATUSTEXT messages from ArduPilot
"""

import serial
import sys
import time
from datetime import datetime
from pymavlink import mavutil

def main():
    # Configuration
    port = '/dev/cu.usbmodem20883687524E1'
    baud = 115200
    
    print("=== ESP32 STATUSTEXT Monitor ===")
    print(f"Connecting to {port} at {baud} baud...")
    
    try:
        # Connect to ESP32
        master = mavutil.mavlink_connection(port, baud=baud, timeout=1)
        print("✓ Connected! Monitoring STATUSTEXT messages...")
        print("Press Ctrl+C to exit\n")
        
        # Wait for first heartbeat to confirm connection
        print("Waiting for heartbeat...")
        heartbeat = master.wait_heartbeat(timeout=10)
        if heartbeat:
            print(f"✓ Heartbeat received from system {heartbeat.get_srcSystem()}, component {heartbeat.get_srcComponent()}")
        else:
            print("⚠ No heartbeat received, but continuing...")
        print()
        
        statustext_count = 0
        all_msg_count = 0
        start_time = time.time()
        
        while True:
            try:
                # Get next message
                msg = master.recv_match(blocking=False)
                if msg is None:
                    time.sleep(0.01)  # Small delay to prevent CPU spinning
                    continue
                
                all_msg_count += 1
                
                # Show periodic status of all messages received
                if all_msg_count % 100 == 0:
                    print(f"[DEBUG] Received {all_msg_count} total messages, {statustext_count} STATUSTEXT messages")
                
                # Check if it's a STATUSTEXT message (or unknown that might be STATUSTEXT)
                msg_type = msg.get_type()
                
                # Show all unknown messages that might be corrupted STATUSTEXT
                if msg_type.startswith('UNKNOWN_'):
                    print(f"[DEBUG] {msg_type} - raw data: {getattr(msg, '_raw_data', 'no raw data')[:20]}...")
                
                if msg_type == 'STATUSTEXT':
                    statustext_count += 1
                    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    
                    # Extract STATUSTEXT fields
                    severity = msg.severity
                    text = msg.text.decode('utf-8', errors='replace').rstrip('\x00')
                elif msg_type.startswith('UNKNOWN_') and hasattr(msg, '_raw_data'):
                    # Check if unknown message contains text that looks like STATUSTEXT
                    try:
                        raw_data = msg._raw_data
                        if len(raw_data) > 10:
                            # Look for text patterns in the raw data
                            text_portion = raw_data[2:].decode('utf-8', errors='replace').rstrip('\x00')
                            if any(keyword in text_portion.upper() for keyword in ['ESP32', 'CAN', 'INIT', 'ERROR', 'WARN', 'READY']):
                                statustext_count += 1
                                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                                severity = raw_data[1] if len(raw_data) > 1 else 6  # Default to INFO
                                text = text_portion[:50]  # Limit length
                                print(f"[RECOVERED] Found likely STATUSTEXT in {msg_type}")
                            else:
                                continue
                    except:
                        continue
                else:
                    continue
                    
                # Map severity levels to names (MAVLink severity levels)
                severity_names = {
                        0: "EMERGENCY",
                        1: "ALERT", 
                        2: "CRITICAL",
                        3: "ERROR",
                        4: "WARNING", 
                        5: "NOTICE",
                        6: "INFO",
                        7: "DEBUG"
                    }
                    
                severity_name = severity_names.get(severity, f"UNKNOWN({severity})")
                    
                # Color coding for severity levels
                if severity <= 2:      # EMERGENCY, ALERT, CRITICAL
                    color = "\033[91m"  # Red
                elif severity == 3:    # ERROR  
                    color = "\033[31m"  # Dark red
                elif severity == 4:    # WARNING
                    color = "\033[93m"  # Yellow
                elif severity == 5:    # NOTICE
                    color = "\033[96m"  # Cyan
                elif severity == 6:    # INFO
                    color = "\033[92m"  # Green
                else:                  # DEBUG
                    color = "\033[37m"  # White/gray
                    
                    reset_color = "\033[0m"
                    
                    # Display the message
                    print(f"{color}[{timestamp}] #{statustext_count:03d} {severity_name:>9}: {text}{reset_color}")
                    
                    # Show additional metadata for certain severities
                    if severity <= 4:  # EMERGENCY through WARNING
                        print(f"  └─ System: {msg.get_srcSystem()}, Component: {msg.get_srcComponent()}")
            
            except KeyboardInterrupt:
                break
            except UnicodeDecodeError as e:
                print(f"⚠ Unicode decode error in STATUSTEXT: {e}")
            except Exception as e:
                print(f"⚠ Error processing message: {e}")
                continue
    
    except serial.SerialException as e:
        print(f"✗ Serial connection failed: {e}")
        return 1
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
        return 1
    finally:
        try:
            master.close()
        except:
            pass
    
    # Summary
    elapsed_time = time.time() - start_time
    print(f"\n=== Session Summary ===")
    print(f"Total messages received: {all_msg_count}")
    print(f"Total STATUSTEXT messages: {statustext_count}")
    print(f"Session duration: {elapsed_time:.1f} seconds")
    if elapsed_time > 0:
        print(f"Overall message rate: {all_msg_count/elapsed_time:.1f} msg/sec")
        if statustext_count > 0:
            print(f"STATUSTEXT rate: {statustext_count/elapsed_time:.1f} msg/sec")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
