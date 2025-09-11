#!/usr/bin/env python3
"""
High-volume MAVLink stress test for ESP32 TConnect rover
Tests packet loss under different transmission rates and patterns
"""

import serial
import time
import struct
import threading
from collections import defaultdict

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

def create_param_request_read(param_name, seq_num):
    """Create PARAM_REQUEST_READ message with sequence number"""
    param_bytes = param_name.encode('ascii')[:16].ljust(16, b'\x00')
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

def create_ping(seq_num, timestamp):
    """Create PING message"""
    payload = struct.pack('<QHBB', timestamp, seq_num, TARGET_SYSTEM, TARGET_COMPONENT)
    msg_id = 4  # PING
    
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
    crc = calculate_crc(crc_data + bytes([237]))  # PING CRC_EXTRA = 237
    
    return header + payload + struct.pack('<H', crc)

def create_heartbeat(seq_num):
    """Create HEARTBEAT message"""
    payload = struct.pack('<BBBBI', 
                         6,   # MAV_TYPE_GCS
                         8,   # MAV_AUTOPILOT_INVALID  
                         0,   # base_mode
                         0,   # custom_mode
                         4,   # MAV_STATE_ACTIVE
                         )
    
    msg_id = 0  # HEARTBEAT
    
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
    crc = calculate_crc(crc_data + bytes([50]))  # HEARTBEAT CRC_EXTRA = 50
    
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
        'payload': payload
    }

class MAVLinkReceiver(threading.Thread):
    """Background thread to receive and count MAVLink messages"""
    
    def __init__(self, serial_port):
        super().__init__(daemon=True)
        self.ser = serial_port
        self.running = True
        self.stats = {
            'total_bytes': 0,
            'total_messages': 0,
            'message_types': defaultdict(int),
            'responses_received': defaultdict(int),  # Track responses by sequence
            'last_sequences': defaultdict(int),      # Track last sequence per message type
            'sequence_gaps': defaultdict(list),      # Track missing sequences
        }
        self.buffer = b''
        
    def run(self):
        while self.running:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
                if data:
                    self.stats['total_bytes'] += len(data)
                    self.buffer += data
                    
                    # Process complete messages
                    while len(self.buffer) >= 12:
                        if self.buffer[0] == MAVLINK_STX_V2:
                            payload_len = self.buffer[1]
                            msg_len = 12 + payload_len
                            
                            if len(self.buffer) >= msg_len:
                                msg = parse_mavlink_message(self.buffer[:msg_len])
                                if msg:
                                    self.stats['total_messages'] += 1
                                    self.stats['message_types'][msg['msg_id']] += 1
                                    
                                    # Track responses to our requests
                                    if msg['msg_id'] == 22:  # PARAM_VALUE response
                                        self.stats['responses_received']['param'] += 1
                                    elif msg['msg_id'] == 5:  # PING response  
                                        self.stats['responses_received']['ping'] += 1
                                    elif msg['msg_id'] == 0:  # HEARTBEAT
                                        self.stats['responses_received']['heartbeat'] += 1
                                    
                                    # Check for sequence gaps in rover messages
                                    msg_type = msg['msg_id'] 
                                    seq = msg['seq']
                                    last_seq = self.stats['last_sequences'][msg_type]
                                    
                                    if last_seq > 0 and seq != (last_seq + 1) % 256:
                                        # Found sequence gap
                                        gap_start = (last_seq + 1) % 256
                                        gap_end = seq
                                        self.stats['sequence_gaps'][msg_type].append((gap_start, gap_end))
                                    
                                    self.stats['last_sequences'][msg_type] = seq
                                
                                self.buffer = self.buffer[msg_len:]
                            else:
                                break
                        else:
                            self.buffer = self.buffer[1:]  # Skip invalid byte
                            
            except Exception as e:
                print(f"Receiver error: {e}")
                
    def stop(self):
        self.running = False

def stress_test_serial():
    print("=== ESP32 TConnect MAVLink Stress Test ===")
    
    # Connect to serial port
    try:
        ser = serial.Serial('/dev/cu.usbserial-0001', 460800, timeout=0.1)
        print(f"✓ Connected to /dev/cu.usbserial-0001 at 460800 baud")
    except Exception as e:
        print(f"✗ Failed to connect: {e}")
        return
    
    time.sleep(1)
    ser.flushInput()
    ser.flushOutput()
    
    # Start receiver thread
    receiver = MAVLinkReceiver(ser)
    receiver.start()
    
    print("\n=== Test 1: Baseline Traffic Analysis ===")
    print("Monitoring normal traffic for 5 seconds...")
    
    time.sleep(5)
    baseline_stats = dict(receiver.stats)
    
    print(f"Baseline: {baseline_stats['total_messages']} messages, {baseline_stats['total_bytes']} bytes")
    print("Message distribution:")
    for msg_id, count in sorted(baseline_stats['message_types'].items()):
        print(f"  ID {msg_id}: {count} messages")
    
    # Test 2: Low-rate parameter requests
    print(f"\n=== Test 2: Low-Rate Parameter Requests (1 Hz) ===")
    
    requests_sent = 0
    start_time = time.time()
    
    test_params = ["SYSID_MYGCS", "SERIAL0_BAUD", "FRAME_CLASS", "CAN_D1_PROTOCOL", "COMPASS_ENABLE"]
    
    for i in range(10):  # 10 requests at 1 Hz
        param_name = test_params[i % len(test_params)]
        msg = create_param_request_read(param_name, requests_sent)
        ser.write(msg)
        ser.flush()
        requests_sent += 1
        
        if i < 9:  # Don't sleep after last request
            time.sleep(1.0)
    
    time.sleep(2)  # Wait for responses
    low_rate_stats = dict(receiver.stats)
    
    responses = low_rate_stats['responses_received']['param'] - baseline_stats.get('responses_received', {}).get('param', 0)
    print(f"Low rate: {requests_sent} requests sent, {responses} responses received")
    print(f"Success rate: {responses/requests_sent*100:.1f}%")
    
    # Test 3: High-rate parameter requests
    print(f"\n=== Test 3: High-Rate Parameter Requests (20 Hz) ===")
    
    requests_sent_hr = 0
    start_time = time.time()
    
    for i in range(100):  # 100 requests at 20 Hz (5 seconds)
        param_name = test_params[i % len(test_params)]
        msg = create_param_request_read(param_name, requests_sent + requests_sent_hr)
        ser.write(msg)
        ser.flush()
        requests_sent_hr += 1
        
        if i < 99:
            time.sleep(0.05)  # 20 Hz = 50ms interval
    
    time.sleep(3)  # Wait for responses
    high_rate_stats = dict(receiver.stats)
    
    responses_hr = high_rate_stats['responses_received']['param'] - low_rate_stats['responses_received']['param']
    print(f"High rate: {requests_sent_hr} requests sent, {responses_hr} responses received")
    print(f"Success rate: {responses_hr/requests_sent_hr*100:.1f}%")
    
    # Test 4: Burst transmission
    print(f"\n=== Test 4: Burst Transmission Test ===")
    
    burst_count = 50
    burst_start = time.time()
    
    # Send 50 pings as fast as possible
    for i in range(burst_count):
        timestamp = int(time.time() * 1000000) & 0xFFFFFFFFFFFFFFFF
        msg = create_ping(requests_sent + requests_sent_hr + i, timestamp)
        ser.write(msg)
    
    ser.flush()  # Ensure all data is sent
    burst_end = time.time()
    
    time.sleep(2)  # Wait for responses
    burst_stats = dict(receiver.stats)
    
    ping_responses = burst_stats['responses_received']['ping'] - high_rate_stats.get('responses_received', {}).get('ping', 0)
    
    print(f"Burst: {burst_count} pings sent in {burst_end-burst_start:.3f}s")
    print(f"Ping responses: {ping_responses}")
    print(f"Burst success rate: {ping_responses/burst_count*100:.1f}%")
    
    # Test 5: Sustained high-rate mixed traffic  
    print(f"\n=== Test 5: Sustained Mixed Traffic (10 seconds) ===")
    
    mixed_start = time.time()
    mixed_requests = 0
    mixed_heartbeats = 0
    
    for i in range(200):  # 20 Hz for 10 seconds
        # Alternate between parameter requests and heartbeats
        if i % 2 == 0:
            param_name = test_params[i % len(test_params)]
            msg = create_param_request_read(param_name, (requests_sent + requests_sent_hr + burst_count + i) & 0xFF)
            mixed_requests += 1
        else:
            msg = create_heartbeat((requests_sent + requests_sent_hr + burst_count + i) & 0xFF)
            mixed_heartbeats += 1
        
        ser.write(msg)
        
        if i % 10 == 0:  # Flush every 10 messages
            ser.flush()
        
        if i < 199:
            time.sleep(0.05)  # 20 Hz
    
    ser.flush()
    time.sleep(3)  # Wait for responses
    
    final_stats = dict(receiver.stats)
    
    mixed_param_responses = final_stats['responses_received']['param'] - burst_stats['responses_received']['param']
    mixed_hb_responses = final_stats['responses_received']['heartbeat'] - burst_stats.get('responses_received', {}).get('heartbeat', 0)
    
    print(f"Mixed traffic: {mixed_requests} param requests, {mixed_heartbeats} heartbeats")
    print(f"Param responses: {mixed_param_responses} ({mixed_param_responses/mixed_requests*100:.1f}%)")
    print(f"Heartbeat responses: {mixed_hb_responses}")
    
    # Final analysis
    print(f"\n=== FINAL ANALYSIS ===")
    
    total_outbound = requests_sent + requests_sent_hr + burst_count + mixed_requests
    total_responses = final_stats['responses_received']['param']
    
    print(f"Total parameter requests sent: {total_outbound}")
    print(f"Total parameter responses: {total_responses}")
    print(f"Overall success rate: {total_responses/total_outbound*100:.1f}%")
    
    print(f"\nReceive statistics:")
    print(f"  Total bytes received: {final_stats['total_bytes']}")
    print(f"  Total messages received: {final_stats['total_messages']}")
    print(f"  Calculated packet loss: {(1 - total_responses/total_outbound)*100:.1f}%")
    
    # Check for sequence gaps in received telemetry
    if final_stats['sequence_gaps']:
        print(f"\nSequence gaps detected (possible RX issues):")
        for msg_type, gaps in final_stats['sequence_gaps'].items():
            if gaps:
                print(f"  Message ID {msg_type}: {len(gaps)} gaps")
    else:
        print(f"\nNo sequence gaps detected in received telemetry")
    
    receiver.stop()
    ser.close()
    
    return {
        'total_requests': total_outbound,
        'total_responses': total_responses, 
        'success_rate': total_responses/total_outbound*100,
        'rx_bytes': final_stats['total_bytes'],
        'rx_messages': final_stats['total_messages']
    }

if __name__ == "__main__":
    results = stress_test_serial()
    
    print(f"\n=== SUMMARY ===")
    print(f"Success Rate: {results['success_rate']:.1f}%")
    print(f"Calculated Loss: {100-results['success_rate']:.1f}%")
    
    if results['success_rate'] < 50:
        print("HIGH PACKET LOSS - Likely UART or processing issue")
    elif results['success_rate'] < 90:
        print("MODERATE PACKET LOSS - Possible buffer/timing issue") 
    else:
        print("LOW PACKET LOSS - Likely application-level issue")