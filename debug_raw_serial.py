#!/usr/bin/env python3
"""
Raw serial test for ESP32 TConnect rover MAVLink communication
Tests UART RX functionality without requiring pymavlink
"""

import serial
import time
import struct

# MAVLink v2 constants
MAVLINK_STX_V2 = 0xFD
MAV_SYSTEM_ID = 255  # GCS system ID
MAV_COMPONENT_ID = 1
TARGET_SYSTEM = 1    # Rover system ID
TARGET_COMPONENT = 1

def calculate_crc(data):
    """Calculate MAVLink CRC-16/MCRF4XX"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc & 0xFFFF

def create_heartbeat():
    """Create a MAVLink v2 HEARTBEAT message"""
    # HEARTBEAT payload: type, autopilot, base_mode, custom_mode, system_status, mavlink_version
    payload = struct.pack('<BBBBI', 
                         6,   # MAV_TYPE_GCS
                         8,   # MAV_AUTOPILOT_INVALID  
                         0,   # base_mode
                         0,   # custom_mode
                         4,   # MAV_STATE_ACTIVE
                         )
    
    msg_id = 0  # HEARTBEAT message ID
    seq = 0
    
    # Create header
    header = struct.pack('<BBBBBB', 
                        MAVLINK_STX_V2,  # start byte
                        len(payload),    # payload length
                        0,               # incompatible flags
                        0,               # compatible flags
                        seq,             # sequence
                        MAV_SYSTEM_ID)   # system ID
    
    header += struct.pack('<BB', MAV_COMPONENT_ID, msg_id & 0xFF)
    header += struct.pack('<HB', (msg_id >> 8) & 0xFFFF, msg_id >> 16)
    
    # Calculate CRC
    crc_data = header[1:] + payload  # All except STX
    crc = calculate_crc(crc_data + bytes([50]))  # HEARTBEAT CRC_EXTRA = 50
    
    return header + payload + struct.pack('<H', crc)

def create_param_request_list():
    """Create PARAM_REQUEST_LIST message"""
    payload = struct.pack('<BB', TARGET_SYSTEM, TARGET_COMPONENT)
    msg_id = 21  # PARAM_REQUEST_LIST
    seq = 1
    
    header = struct.pack('<BBBBBBBBB', 
                        MAVLINK_STX_V2,  # start byte
                        len(payload),    # payload length  
                        0,               # incompatible flags
                        0,               # compatible flags
                        seq,             # sequence
                        MAV_SYSTEM_ID,   # system ID
                        MAV_COMPONENT_ID, # component ID
                        msg_id & 0xFF,   # msg_id low
                        (msg_id >> 8) & 0xFF) # msg_id mid
    header += struct.pack('<B', msg_id >> 16)  # msg_id high
    
    crc_data = header[1:] + payload
    crc = calculate_crc(crc_data + bytes([159]))  # PARAM_REQUEST_LIST CRC_EXTRA = 159
    
    return header + payload + struct.pack('<H', crc)

def create_param_request_read(param_name):
    """Create PARAM_REQUEST_READ message"""
    # Pad param name to 16 bytes
    param_bytes = param_name.encode('ascii')[:16].ljust(16, b'\x00')
    payload = struct.pack('<BB', TARGET_SYSTEM, TARGET_COMPONENT) + param_bytes + struct.pack('<h', -1)
    
    msg_id = 20  # PARAM_REQUEST_READ
    seq = 2
    
    header = struct.pack('<BBBBBBBBB',
                        MAVLINK_STX_V2,
                        len(payload),
                        0, 0, seq,
                        MAV_SYSTEM_ID,
                        MAV_COMPONENT_ID,
                        msg_id & 0xFF,
                        (msg_id >> 8) & 0xFF)
    header += struct.pack('<B', msg_id >> 16)
    
    crc_data = header[1:] + payload
    crc = calculate_crc(crc_data + bytes([214]))  # PARAM_REQUEST_READ CRC_EXTRA = 214
    
    return header + payload + struct.pack('<H', crc)

def parse_mavlink_message(data):
    """Parse incoming MAVLink v2 message"""
    if len(data) < 12:  # Minimum header size
        return None
        
    if data[0] != MAVLINK_STX_V2:
        return None
    
    payload_len = data[1]
    if len(data) < 12 + payload_len:
        return None
    
    seq = data[4]
    system_id = data[5]
    component_id = data[6]
    msg_id = data[7] | (data[8] << 8) | (data[9] << 16)
    
    payload = data[10:10+payload_len]
    
    return {
        'seq': seq,
        'system_id': system_id,
        'component_id': component_id,
        'msg_id': msg_id,
        'payload': payload
    }

def test_serial_communication():
    print("=== ESP32 TConnect Raw Serial MAVLink Test ===")
    
    # Connect to serial port
    try:
        ser = serial.Serial('/dev/cu.usbserial-0001', 460800, timeout=1)
        print(f"✓ Connected to /dev/cu.usbserial-0001 at 460800 baud")
    except Exception as e:
        print(f"✗ Failed to connect: {e}")
        return
    
    time.sleep(2)  # Let connection stabilize
    
    # Clear any existing data
    ser.flushInput()
    ser.flushOutput()
    
    # Test 1: Check for existing traffic
    print("\n1. Monitoring existing traffic for 3 seconds...")
    start_time = time.time()
    bytes_received = 0
    messages_received = 0
    
    buffer = b''
    while time.time() - start_time < 3:
        data = ser.read(ser.in_waiting or 1)
        if data:
            bytes_received += len(data)
            buffer += data
            
            # Look for MAVLink messages - handle partial packets correctly
            while len(buffer) >= 12:
                if buffer[0] == MAVLINK_STX_V2:
                    payload_len = buffer[1]
                    msg_len = 12 + payload_len
                    
                    if len(buffer) >= msg_len:
                        msg = parse_mavlink_message(buffer[:msg_len])
                        if msg:
                            messages_received += 1
                            if messages_received <= 5:  # Show first 5 messages
                                print(f"  RX message: ID={msg['msg_id']}, sys={msg['system_id']}, len={payload_len}")
                        buffer = buffer[msg_len:]
                    else:
                        break  # Keep partial message in buffer for next iteration
                else:
                    # Invalid start byte - find next potential MAVLink start
                    next_fd = buffer.find(MAVLINK_STX_V2, 1)
                    if next_fd > 0:
                        print(f"Warning: Skipped {next_fd} invalid bytes: {buffer[:next_fd].hex()}")
                        buffer = buffer[next_fd:]
                    else:
                        buffer = buffer[1:]  # Remove one invalid byte
    
    print(f"✓ Baseline traffic: {bytes_received} bytes, {messages_received} MAVLink messages")
    
    # Test 2: Send heartbeat and look for response
    print("\n2. Testing heartbeat transmission...")
    heartbeat = create_heartbeat()
    
    ser.write(heartbeat)
    ser.flush()
    print(f"✓ Sent HEARTBEAT ({len(heartbeat)} bytes)")
    
    # Look for heartbeat response
    time.sleep(1)
    response_data = ser.read(ser.in_waiting or 100)
    if response_data:
        print(f"✓ Received {len(response_data)} bytes response")
        
        # Try to parse response
        msg = parse_mavlink_message(response_data)
        if msg and msg['msg_id'] == 0:  # HEARTBEAT response
            print(f"✓ HEARTBEAT response: sys={msg['system_id']}, comp={msg['component_id']}")
        else:
            print(f"? Response not a HEARTBEAT: {response_data[:20].hex()}")
    else:
        print("✗ No response to HEARTBEAT")
    
    # Test 3: Request specific parameter
    print("\n3. Testing parameter request...")
    param_req = create_param_request_read("SYSID_MYGCS")
    
    ser.write(param_req)
    ser.flush()
    print(f"✓ Sent PARAM_REQUEST_READ for SYSID_MYGCS ({len(param_req)} bytes)")
    
    # Wait for parameter response
    start_time = time.time()
    param_response = False
    
    while time.time() - start_time < 3:
        data = ser.read(ser.in_waiting or 50)
        if data:
            msg = parse_mavlink_message(data)
            if msg and msg['msg_id'] == 22:  # PARAM_VALUE
                print(f"✓ PARAM_VALUE response received ({len(data)} bytes)")
                param_response = True
                break
            elif msg:
                print(f"? Other message: ID={msg['msg_id']}")
    
    if not param_response:
        print("✗ No PARAM_VALUE response")
    
    # Test 4: Request parameter list
    print("\n4. Testing parameter list request...")
    param_list_req = create_param_request_list()
    
    ser.write(param_list_req)
    ser.flush()
    print(f"✓ Sent PARAM_REQUEST_LIST ({len(param_list_req)} bytes)")
    
    # Monitor parameter responses
    start_time = time.time()
    param_count = 0
    
    while time.time() - start_time < 5:  # 5 second window
        data = ser.read(ser.in_waiting or 100)
        if data:
            msg = parse_mavlink_message(data)
            if msg and msg['msg_id'] == 22:  # PARAM_VALUE
                param_count += 1
                if param_count <= 10 or param_count % 50 == 0:
                    print(f"  Parameter {param_count} received")
    
    print(f"✓ Parameter list: {param_count} parameters received")
    
    # Test 5: Raw transmission test
    print("\n5. Testing raw data transmission...")
    
    # Send some test bytes and see what comes back
    test_data = b"MAVLINK_TEST_12345\n"
    ser.write(test_data)
    ser.flush()
    
    time.sleep(0.5)
    response = ser.read(ser.in_waiting or 100)
    
    if response:
        print(f"✓ Raw response: {len(response)} bytes: {response[:50]}")
    else:
        print("✗ No raw response")
    
    # Final statistics
    print(f"\n=== UART Statistics ===")
    print(f"Bytes in waiting: {ser.in_waiting}")
    print(f"Connection: {ser.is_open}")
    
    ser.close()
    
    print(f"\n=== SUMMARY ===")
    print(f"- Baseline RX traffic: {messages_received} messages")
    print(f"- Parameter request response: {'✓' if param_response else '✗'}")
    print(f"- Parameter list response: {param_count} parameters")
    print(f"- UART RX functionality: {'✓' if param_count > 0 else '?' if messages_received > 0 else '✗'}")

if __name__ == "__main__":
    test_serial_communication()