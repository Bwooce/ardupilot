#!/usr/bin/env python3
"""
Debug CRSF telemetry from RadioMaster Bandit
Figure out why we're getting zeros when it shows "Connected"
"""

import serial
import time
import sys
import struct

def crc8_dvb_s2(data):
    """Calculate CRC8 for CRSF"""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0xD5
            else:
                crc = crc << 1
            crc &= 0xFF
    return crc

def build_rc_packet():
    """Build CRSF RC packet"""
    channels = [992] * 16  # All centered
    packed = 0
    for i, ch in enumerate(channels):
        packed |= (ch & 0x7FF) << (i * 11)
    payload = packed.to_bytes(22, 'little')
    
    frame = bytearray([0xC8, 24, 0x16])
    frame.extend(payload)
    crc = crc8_dvb_s2(bytes([0x16]) + payload)
    frame.append(crc)
    return bytes(frame)

def parse_crsf_frame(data, offset):
    """Try to parse a CRSF frame at offset"""
    if offset + 2 >= len(data):
        return None, 0
    
    sync = data[offset]
    length = data[offset + 1]
    
    if offset + length + 2 > len(data):
        return None, 0
    
    frame_type = data[offset + 2]
    payload = data[offset + 3:offset + 1 + length]
    crc = data[offset + 1 + length]
    
    # Verify CRC
    calc_crc = crc8_dvb_s2(bytes([frame_type]) + payload)
    
    return {
        'sync': sync,
        'length': length,
        'type': frame_type,
        'payload': payload,
        'crc': crc,
        'crc_valid': crc == calc_crc
    }, offset + length + 2

def debug_telemetry(port="/dev/cu.usbserial-0001"):
    """Debug CRSF telemetry"""
    
    print("="*60)
    print("CRSF Telemetry Debug")
    print("="*60)
    print("Bandit shows: Connected")
    print("Testing telemetry reception...")
    print("-"*60)
    
    try:
        ser = serial.Serial(
            port=port, 
            baudrate=420000, 
            timeout=0.1,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            rtscts=False,
            dsrdtr=False
        )
        rc_packet = build_rc_packet()
        
        # Clear buffer
        ser.reset_input_buffer()
        
        print("\n1. Sending RC packets and monitoring response...")
        
        # Collect data for analysis
        all_data = bytearray()
        non_zero_bytes = bytearray()
        frame_types_seen = set()
        
        for i in range(500):  # ~3 seconds at 150Hz
            ser.write(rc_packet)
            
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                all_data.extend(data)
                
                # Find non-zero bytes
                for b in data:
                    if b != 0:
                        non_zero_bytes.append(b)
                
                # Try to parse CRSF frames
                offset = 0
                while offset < len(data):
                    # Look for sync bytes
                    if data[offset] in [0xC8, 0xEC, 0xEE, 0xEA]:
                        frame, next_offset = parse_crsf_frame(data, offset)
                        if frame and frame['crc_valid']:
                            frame_types_seen.add(frame['type'])
                            if i == 0 or len(frame_types_seen) == 1:  # First frame or new type
                                print(f"\n  ✓ Valid CRSF frame:")
                                print(f"    Type: 0x{frame['type']:02X}")
                                print(f"    Length: {frame['length']}")
                                print(f"    Payload: {frame['payload'].hex()[:40]}...")
                            offset = next_offset
                            continue
                    offset += 1
            
            time.sleep(0.00667)
        
        print(f"\n2. Analysis of {len(all_data)} bytes received:")
        print(f"   - Non-zero bytes: {len(non_zero_bytes)}")
        print(f"   - Zero bytes: {len(all_data) - len(non_zero_bytes)}")
        
        if non_zero_bytes:
            print(f"   - Non-zero values: {' '.join(f'{b:02X}' for b in non_zero_bytes[:50])}")
            print(f"   - CRSF frame types seen: {', '.join(f'0x{t:02X}' for t in frame_types_seen)}")
        
        # Check for specific patterns
        print("\n3. Checking for MAVLink markers...")
        mavlink_v1 = all_data.find(b'\xFE')
        mavlink_v2 = all_data.find(b'\xFD')
        
        if mavlink_v1 >= 0:
            print(f"   ✓ Found MAVLink v1 marker at offset {mavlink_v1}")
            print(f"     Context: {all_data[mavlink_v1:mavlink_v1+20].hex()}")
        if mavlink_v2 >= 0:
            print(f"   ✓ Found MAVLink v2 marker at offset {mavlink_v2}")
            print(f"     Context: {all_data[mavlink_v2:mavlink_v2+20].hex()}")
        
        if mavlink_v1 < 0 and mavlink_v2 < 0:
            print("   ✗ No MAVLink markers found")
        
        # Test requesting telemetry
        print("\n4. Testing telemetry request frames...")
        
        # CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY (0x2B)
        # Request telemetry settings
        request_frame = bytearray([0xEA, 4, 0x2B, 0x00, 0x00])
        crc = crc8_dvb_s2(request_frame[2:])
        request_frame.append(crc)
        
        ser.write(request_frame)
        time.sleep(0.1)
        
        if ser.in_waiting:
            response = ser.read(ser.in_waiting)
            if response != b'\x00' * len(response):
                print(f"   Response to settings request: {response.hex()[:60]}")
        
        ser.close()
        
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return
    except Exception as e:
        print(f"Error: {e}")
        return
    
    print("\n" + "="*60)
    print("Diagnosis:")
    print("-"*60)
    
    if len(non_zero_bytes) == 0:
        print("""
The Bandit is NOT sending telemetry data despite showing "Connected".

Possible causes:
1. The RX is not sending telemetry to the flight controller
   → Check that RX UART is connected to FC and configured for MAVLink
   
2. The Bandit is not configured for telemetry passthrough
   → May need to enable telemetry in ELRS settings
   
3. The link is in wrong mode
   → Should be in MAVLink mode, not Normal mode
   
4. Telemetry ratio is set too low
   → Check telemetry ratio in ELRS settings (should be 1:2 or better)
""")
    else:
        print(f"""
The Bandit IS sending data:
- Frame types: {', '.join(f'0x{t:02X}' for t in frame_types_seen)}
- Non-zero bytes: {len(non_zero_bytes)}

Check CRSF frame type meanings:
- 0x14: Link statistics
- 0x1E: Attitude
- 0x21: Flight mode
- 0x29: Device info
- 0x7F: MAVLink telemetry
""")
    
    print("="*60)

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbserial-0001"
    debug_telemetry(port)