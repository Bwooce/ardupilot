#!/usr/bin/env python3
"""
Comprehensive MAVLink debugging tool
Sends initial heartbeat with component 191 and monitors all MAVLink traffic
"""

import time
import sys
import glob
import serial
from pymavlink import mavutil

def debug_mavlink(port="/dev/cu.usbmodem*", baudrate=115200, duration=20):
    # Find device
    devices = glob.glob(port)
    if not devices:
        print(f"No device found matching {port}")
        sys.exit(1)
    port = devices[0]
    
    print(f"Connecting to {port} at {baudrate} baud...")
    
    # Also open raw serial for BAD_DATA capture and CLI commands
    ser = serial.Serial(port, baudrate, timeout=0.5)
    
    # First check mLRS configuration
    print("Checking mLRS configuration...")
    ser.write(b"\r")  # Get into CLI
    time.sleep(0.2)
    ser.reset_input_buffer()
    
    # Get TX parameters
    ser.write(b"pl tx\r")
    time.sleep(0.5)
    response = ser.read(2000)
    if response:
        config_text = response.decode('ascii', errors='ignore')
        # Check critical parameters
        if 'Tx Ser Dest = serial' in config_text:
            print("  ✓ Tx Ser Dest = serial (correct)")
        elif 'Tx Ser Dest = mbridge' in config_text:
            print("  ✗ Tx Ser Dest = mbridge (WRONG - should be 'serial')")
        
        if 'Tx Mav Component = enabled' in config_text:
            print("  ✓ Tx Mav Component = enabled")
        elif 'Tx Mav Component = disabled' in config_text:
            print("  ✗ Tx Mav Component = disabled (should be enabled)")
    
    # Quick stats check to see if RF packets are arriving
    print("\nChecking RF link status...")
    ser.write(b"stats\r")
    time.sleep(2)  # Collect a few samples
    ser.write(b"\r")  # Stop stats
    time.sleep(0.2)
    stats_response = ser.read(2000)
    if stats_response:
        stats_text = stats_response.decode('ascii', errors='ignore')
        # Parse stats lines (format: LQ(LQ),LQ, RSSI1,RSSI2, SNR; TX, RX;)
        lines = stats_text.split('\n')
        rx_counts = []
        tx_counts = []
        for line in lines:
            if ';' in line and ',' in line:
                try:
                    # Extract TX and RX counts
                    parts = line.split(';')
                    if len(parts) >= 2:
                        counts = parts[1].strip()
                        if ',' in counts:
                            tx_rx = counts.split(',')
                            tx_val = int(tx_rx[0].strip())
                            rx_val = int(tx_rx[1].strip()) if len(tx_rx) > 1 else 0
                            tx_counts.append(tx_val)
                            rx_counts.append(rx_val)
                except:
                    pass
        
        if rx_counts:
            print(f"  TX counts: {tx_counts[:5]}")  # Show first 5
            print(f"  RX counts: {rx_counts[:5]}")
            
            # Check for anomalies
            if any(rx_counts[i] > rx_counts[i+1] for i in range(len(rx_counts)-1) if rx_counts[i+1] > 0):
                print("  ⚠ RX count DECREASING - counter overflow or reset!")
            elif max(rx_counts) > 0:
                print("  ✓ RF packets being received")
                print("  ⚠ But NOT forwarded to USB (we only see our own messages)")
    
    # Switch to MAVLink mode - mLRS automatically switches when it receives MAVLink
    print("\nSwitching to MAVLink mode...")
    # Just send a MAVLink message - mLRS will auto-switch from CLI to MAVLink
    # No explicit command needed
    ser.reset_input_buffer()
    
    # Connect with component ID 191
    master = mavutil.mavlink_connection(
        port, 
        baud=baudrate,
        source_system=255,
        source_component=191,
        autoreconnect=True,
        robust_parsing=True
    )
    
    print(f"Connected as System=255, Component=191")
    print("Note: mLRS TX sends as component 190 when 'Tx Mav Component' is enabled")
    print("Sending initial heartbeats to trigger streaming...")
    
    # Send initial heartbeats - try different autopilot types
    # mLRS might need a specific autopilot type to establish routing
    for i in range(5):
        if i == 0:
            # Try as GCS with MAV_AUTOPILOT_INVALID (standard GCS)
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            print(f"  Sent heartbeat #{i+1} (GCS/INVALID)")
        elif i == 1:
            # Try as GCS with MAV_AUTOPILOT_GENERIC (some GCS use this)
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                0,  # MAV_AUTOPILOT_GENERIC
                0, 0, 0
            )
            print(f"  Sent heartbeat #{i+1} (GCS/GENERIC)")
        else:
            # Standard GCS heartbeat
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            print(f"  Sent heartbeat #{i+1}")
        time.sleep(0.2)
    
    # Also request data from System 1 to trigger routing
    print("Requesting data from System 1 (ArduPilot)...")
    master.mav.param_request_list_send(1, 0)
    master.mav.request_data_stream_send(
        1, 0,  # target system 1, component 0
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        4, 1  # 4 Hz, start
    )
    
    print(f"\nMonitoring for {duration} seconds...\n")
    
    # Track statistics
    systems = {}
    messages = {}
    bad_data_count = 0
    bad_data_bytes = bytearray()
    component_190_count = 0
    start_time = time.time()
    last_heartbeat = time.time()
    
    while time.time() - start_time < duration:
        # Send periodic heartbeat
        if time.time() - last_heartbeat >= 1.0:
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            last_heartbeat = time.time()
        
        # Receive messages
        msg = master.recv_match(blocking=False, timeout=0.1)
        if msg:
            msg_type = msg.get_type()
            
            if msg_type == 'BAD_DATA':
                bad_data_count += 1
                # Capture raw bytes
                raw = ser.read(ser.in_waiting or 1)
                if raw:
                    bad_data_bytes.extend(raw)
                    if bad_data_count <= 10:  # Show first 10 BAD_DATA
                        hex_str = ' '.join(f'{b:02x}' for b in raw[:50])
                        ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in raw[:50])
                        print(f"BAD_DATA #{bad_data_count}: {hex_str}")
                        
                        # Analyze structure
                        if len(raw) >= 16:
                            # Check for patterns
                            if raw[0:3] == b'd\r\n' or raw[0:3] == b'id\r':
                                print(f"  Starts with CLI response: '{raw[0:3].decode('ascii', errors='ignore')}'")
                                if len(raw) > 3:
                                    # Analyze the binary part after CLI text
                                    binary_part = raw[3:]
                                    if len(binary_part) >= 2:
                                        print(f"  Binary after CLI: sys={binary_part[0]:02x} comp={binary_part[1]:02x}")
                                        if binary_part[0] == 0xff and binary_part[1] == 0xbf:
                                            print("  → Looks like our System 255, Component 191!")
                            elif raw[0] == 0xfe:
                                print("  MAVLink v1 start byte detected")
                                if len(raw) >= 6:
                                    print(f"  MAVLink v1: len={raw[1]} seq={raw[2]} sys={raw[3]} comp={raw[4]} msg={raw[5]}")
                            elif raw[0] == 0xfd:
                                print("  MAVLink v2 start byte detected")
                                if len(raw) >= 10:
                                    print(f"  MAVLink v2: len={raw[1]} incompat={raw[2]} compat={raw[3]} seq={raw[4]} sys={raw[5]} comp={raw[6]} msg_id={raw[7:10].hex()}")
                            else:
                                # Look for repeating patterns
                                if len(raw) >= 8 and raw[3:5] == b'\xff\xbf':
                                    print(f"  Contains 0xFF 0xBF at offset 3-4 (System 255, Component 191)")
                                if b'\x00\x00\x00\x00\x00\x06\x08' in raw:
                                    print("  Contains pattern: 00 00 00 00 00 06 08")
                        
                        if any(32 <= b < 127 for b in raw):
                            print(f"  ASCII readable: {ascii_str}")
            else:
                sys_id = msg.get_srcSystem()
                comp_id = msg.get_srcComponent()
                
                # Special check for component 190
                if comp_id == 190:
                    component_190_count += 1
                    if component_190_count == 1:
                        print(f"*** COMPONENT 190 DETECTED! System={sys_id} ***")
                        print("  This is mLRS TX component heartbeat")
                
                # Track systems
                key = (sys_id, comp_id)
                if key not in systems:
                    systems[key] = {'first_seen': time.time() - start_time, 'messages': {}}
                    print(f"*** NEW SYSTEM: {sys_id}/{comp_id} ***")
                    
                    if msg_type == 'HEARTBEAT':
                        print(f"  Type: {msg.type}, Autopilot: {msg.autopilot}")
                        if msg.autopilot == 3:
                            print("  ✓ This is ArduPilot!")
                        elif sys_id == 255 and comp_id == 191:
                            print("  (Our own heartbeat)")
                        elif comp_id == 190:
                            print("  ✓ This is mLRS TX component (Tx Mav Component enabled)")
                        elif sys_id == 51:
                            print("  ✓ This might be mLRS system ID")
                
                # Track messages
                if msg_type not in messages:
                    messages[msg_type] = 0
                    print(f"NEW MESSAGE TYPE: {msg_type}")
                    
                    # Check if this is our own message echoed back
                    if sys_id == 255 and comp_id == 191:
                        if msg_type in ['PARAM_REQUEST_LIST', 'REQUEST_DATA_STREAM', 'COMMAND_LONG']:
                            print(f"  ⚠ This is our own {msg_type} echoed back!")
                    
                    # Show details for important messages
                    if msg_type == 'STATUSTEXT':
                        print(f"  Text: {msg.text}")
                    elif msg_type == 'SYS_STATUS':
                        print(f"  Voltage: {msg.voltage_battery}mV")
                    elif msg_type == 'GPS_RAW_INT':
                        print(f"  GPS Fix: {msg.fix_type}, Sats: {msg.satellites_visible}")
                    elif msg_type == 'ATTITUDE':
                        print(f"  Roll: {msg.roll:.2f}, Pitch: {msg.pitch:.2f}")
                    elif msg_type == 'PARAM_VALUE':
                        print(f"  {msg.param_id} = {msg.param_value}")
                
                messages[msg_type] += 1
                systems[key]['messages'][msg_type] = systems[key]['messages'].get(msg_type, 0) + 1
        
        time.sleep(0.01)
    
    # Print summary
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    
    print(f"\nSystems detected: {len(systems)}")
    for (sys_id, comp_id), info in systems.items():
        total_msgs = sum(info['messages'].values())
        print(f"  System {sys_id}/{comp_id}: {total_msgs} messages")
        if len(info['messages']) <= 5:
            for msg_type, count in info['messages'].items():
                print(f"    {msg_type}: {count}")
        else:
            print(f"    Types: {', '.join(info['messages'].keys())}")
    
    print(f"\nMessage types: {len(messages)}")
    for msg_type, count in sorted(messages.items(), key=lambda x: x[1], reverse=True)[:10]:
        print(f"  {msg_type}: {count}")
    
    if bad_data_count:
        print(f"\nBAD_DATA messages: {bad_data_count}")
        if bad_data_bytes:
            print(f"Total BAD_DATA bytes captured: {len(bad_data_bytes)}")
            # Check for ASCII text
            ascii_chars = sum(1 for b in bad_data_bytes if 32 <= b < 127)
            if ascii_chars > len(bad_data_bytes) * 0.5:
                print("BAD_DATA appears to be text:")
                text = ''.join(chr(b) if 32 <= b < 127 else '.' for b in bad_data_bytes[:200])
                print(f"  {text}")
    
    if component_190_count > 0:
        print(f"\nComponent 190 messages: {component_190_count}")
        print("  mLRS TX component is active but may not be forwarding")
    
    # Determine connection status
    print("\n" + "=" * 60)
    our_sys = (255, 191)
    other_systems = [s for s in systems.keys() if s != our_sys]
    
    if any(systems[s]['messages'].get('HEARTBEAT', 0) > 0 for s in other_systems if s in systems):
        if 1 in [s[0] for s in other_systems]:
            print("✓ ArduPilot detected and communicating!")
        else:
            print("✓ MAVLink communication established with remote system")
    elif len(systems) == 1 and our_sys in systems:
        print("✗ mLRS is in LOOPBACK MODE - only echoing our own messages!")
        print("\nThis means:")
        print("  - USB→RF: Our messages are NOT being transmitted")
        print("  - RF→USB: Received packets are NOT being forwarded to USB")
        print("  - All messages seen are just local echo")
        print("\nThis indicates an mLRS forwarding issue:")
        print("  - Stats show RF packets arriving (good RF link)")
        print("  - But those packets aren't forwarded to serial/USB")
        print("  - Instead, serial input is just echoed back")
        print("  - RX counter may be resetting (firmware issue?)")
        print("\nPossible fixes:")
        print("  1. Wrong serial destination (should be 'serial' not 'mbridge')")
        print("  2. MAVLink routing not established (needs bidirectional heartbeats)")
        print("  3. Firmware bug in packet forwarding")
        print("  4. Try 'Tx Ser Dest = serial2' if available")
        print("\nTo check mLRS status:")
        print("  - Enter CLI: send empty line")
        print("  - View params: pl tx")
        print("  - Check stats: stats")
        print("  - Exit to MAVLink: exit")
    else:
        print("⚠ Partial communication - check connection")
    
    ser.close()

if __name__ == "__main__":
    baudrate = int(sys.argv[1]) if len(sys.argv) > 1 else 115200
    duration = int(sys.argv[2]) if len(sys.argv) > 2 else 20
    
    print("=" * 60)
    print("MAVLink Debug Tool")
    print("=" * 60)
    
    debug_mavlink("/dev/cu.usbmodem*", baudrate, duration)