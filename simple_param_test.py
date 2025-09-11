#!/usr/bin/env python3
"""
Simple parameter request test to verify basic communication
"""

import serial
import time
import struct

# MAVLink v2 constants
MAVLINK_STX_V2 = 0xFD
MAV_SYSTEM_ID = 255
MAV_COMPONENT_ID = 1
TARGET_SYSTEM = 1
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

def create_param_request_read(param_id, seq_num):
    """Create PARAM_REQUEST_READ message for specific parameter"""
    # Pad parameter ID to 16 bytes
    param_bytes = param_id.encode('ascii')[:16].ljust(16, b'\x00')
    payload = struct.pack('<BB', TARGET_SYSTEM, TARGET_COMPONENT) + param_bytes + struct.pack('<h', -1)
    msg_id = 20  # PARAM_REQUEST_READ
    
    header = struct.pack('<BBBBBBBBB', 
                        MAVLINK_STX_V2,
                        len(payload),
                        0, 0, seq_num & 0xFF,
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
    if len(data) < 12:
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
        'payload': payload,
        'raw': data
    }

def test_single_param_request():
    """Test single parameter request with detailed monitoring"""
    
    print("=== Simple Parameter Request Test ===")
    
    try:
        ser = serial.Serial('/dev/cu.usbserial-0001', 460800, timeout=10.0)  # Longer timeout
        print(f"✓ Connected to /dev/cu.usbserial-0001 at 460800 baud")
    except Exception as e:
        print(f"✗ Failed to connect: {e}")
        return
    
    time.sleep(2)  # Give more time for connection
    ser.flushInput()
    ser.flushOutput()
    
    print("\n=== Monitoring Baseline Traffic ===")
    
    # Monitor baseline traffic for 5 seconds
    baseline_start = time.time()
    baseline_messages = []
    buffer = b''
    
    while time.time() - baseline_start < 5:
        data = ser.read(ser.in_waiting or 1)
        if data:
            buffer += data
            
            while len(buffer) >= 12:
                if buffer[0] == MAVLINK_STX_V2:
                    payload_len = buffer[1]
                    msg_len = 12 + payload_len
                    
                    if len(buffer) >= msg_len:
                        msg = parse_mavlink_message(buffer[:msg_len])
                        if msg:
                            baseline_messages.append(msg)
                        buffer = buffer[msg_len:]
                    else:
                        break
                else:
                    buffer = buffer[1:]
    
    print(f"Baseline traffic: {len(baseline_messages)} messages in 5 seconds")
    message_types = {}
    for msg in baseline_messages:
        message_types[msg['msg_id']] = message_types.get(msg['msg_id'], 0) + 1
    
    for msg_id, count in sorted(message_types.items()):
        print(f"  Message ID {msg_id}: {count} messages")
    
    if len(baseline_messages) == 0:
        print("❌ No baseline traffic - connection may be broken")
        ser.close()
        return
    
    print(f"\n=== Testing Single Parameter Requests ===")
    
    # Test common parameters
    test_params = ['SYSID_THISMAV', 'RC1_TRIM', 'RC2_TRIM', 'BATT_MONITOR', 'FRAME_CLASS']
    
    for param_name in test_params:
        print(f"\nTesting parameter: {param_name}")
        
        # Clear buffer
        buffer = b''
        ser.flushInput()
        
        # Send request
        param_request = create_param_request_read(param_name, 1)
        ser.write(param_request)
        ser.flush()
        
        print(f"  ✓ Sent PARAM_REQUEST_READ for '{param_name}' ({len(param_request)} bytes)")
        
        # Wait for response
        response_received = False
        timeout = 5.0  # 5 second timeout per parameter
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            data = ser.read(ser.in_waiting or 1)
            if data:
                buffer += data
                
                while len(buffer) >= 12:
                    if buffer[0] == MAVLINK_STX_V2:
                        payload_len = buffer[1]
                        msg_len = 12 + payload_len
                        
                        if len(buffer) >= msg_len:
                            msg = parse_mavlink_message(buffer[:msg_len])
                            if msg:
                                if msg['msg_id'] == 22:  # PARAM_VALUE
                                    payload = msg['payload']
                                    if len(payload) >= 25:
                                        try:
                                            param_value = struct.unpack('<f', payload[0:4])[0]
                                            param_id = payload[8:24].decode('ascii', errors='ignore').rstrip('\x00')
                                            param_type = payload[24]
                                            
                                            if param_id == param_name:
                                                print(f"  ✅ SUCCESS: {param_name} = {param_value} (type {param_type})")
                                                response_received = True
                                                break
                                            else:
                                                print(f"  📨 Got different parameter: {param_id} = {param_value}")
                                        except Exception as e:
                                            print(f"  ❌ Failed to parse parameter: {e}")
                                elif msg['msg_id'] == 0:
                                    # Heartbeat - normal
                                    pass
                                else:
                                    print(f"  📨 Got message ID {msg['msg_id']} instead of PARAM_VALUE")
                            
                            buffer = buffer[msg_len:]
                        else:
                            break
                    else:
                        buffer = buffer[1:]
            
            if response_received:
                break
        
        if not response_received:
            print(f"  ❌ TIMEOUT: No response for '{param_name}' after {timeout} seconds")
    
    ser.close()
    print(f"\n=== Test Complete ===")

if __name__ == "__main__":
    test_single_param_request()