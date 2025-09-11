#!/usr/bin/env python3
"""
Test different methods to establish and maintain connection with RadioMaster Bandit
Find what makes it connect and stay connected
"""

import serial
import time
import sys
import threading

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

class BanditConnectionTest:
    def __init__(self, port="/dev/cu.usbserial-0001"):
        self.port = port
        self.ser = None
        self.connected = False
        self.running = False
        
    def build_crsf_rc_packet(self, throttle=992):
        """Build CRSF RC packet with variable throttle"""
        channels = [992] * 16  # All centered
        channels[2] = throttle  # Throttle channel
        
        packed = 0
        for i, ch in enumerate(channels):
            packed |= (ch & 0x7FF) << (i * 11)
        payload = packed.to_bytes(22, 'little')
        
        frame = bytearray([0xC8, 24, 0x16])
        frame.extend(payload)
        crc = crc8_dvb_s2(bytes([0x16]) + payload)
        frame.append(crc)
        return bytes(frame)
    
    def build_radio_id_packet(self):
        """Build CRSF radio ID packet"""
        frame = bytearray()
        frame.append(0xC8)  # Sync
        frame.append(6)     # Length
        frame.append(0x3A)  # CRSF_FRAMETYPE_RADIO_ID
        frame.extend([0xEA, 0xEE, 0x00, 0x00])  # Radio ID
        crc = crc8_dvb_s2(frame[2:-1])
        frame.append(crc)
        return bytes(frame)
    
    def build_msp_packet(self):
        """Build MSP packet wrapped in CRSF"""
        # MSP ping command
        msp = bytearray([0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        
        frame = bytearray()
        frame.append(0xEA)  # Sync (TX module)
        frame.append(len(msp) + 2)  # Length
        frame.append(0x7A)  # CRSF_FRAMETYPE_MSP_REQ
        frame.extend(msp)
        crc = crc8_dvb_s2(bytes([0x7A]) + msp)
        frame.append(crc)
        return bytes(frame)
    
    def test_method_1_basic_crsf(self):
        """Method 1: Basic CRSF at different rates"""
        print("\n1. Testing basic CRSF at different rates...")
        
        rates = [50, 100, 150, 200, 250]
        for rate in rates:
            print(f"   Testing {rate}Hz...")
            packet = self.build_crsf_rc_packet()
            
            for _ in range(rate):
                self.ser.write(packet)
                time.sleep(1.0/rate)
                
                if self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting)
                    if data != b'\x00' * len(data):
                        print(f"   ✓ Got response at {rate}Hz!")
                        return True
        return False
    
    def test_method_2_radio_id(self):
        """Method 2: Send radio ID first"""
        print("\n2. Testing with radio ID packet...")
        
        # Send radio ID
        id_packet = self.build_radio_id_packet()
        self.ser.write(id_packet)
        time.sleep(0.1)
        
        # Then send RC packets
        rc_packet = self.build_crsf_rc_packet()
        for _ in range(100):
            self.ser.write(rc_packet)
            time.sleep(0.00667)
            
            if self.ser.in_waiting:
                data = self.ser.read(self.ser.in_waiting)
                if data != b'\x00' * len(data):
                    print("   ✓ Got response after radio ID!")
                    return True
        return False
    
    def test_method_3_arm_sequence(self):
        """Method 3: Simulate arming sequence"""
        print("\n3. Testing arming sequence...")
        
        # Low throttle
        print("   Sending low throttle...")
        packet = self.build_crsf_rc_packet(throttle=172)  # Min throttle
        for _ in range(50):
            self.ser.write(packet)
            time.sleep(0.00667)
        
        # Arm (typically yaw right)
        print("   Sending arm command (yaw right)...")
        channels = [992] * 16
        channels[2] = 172   # Throttle low
        channels[3] = 1811  # Yaw right
        
        packed = 0
        for i, ch in enumerate(channels):
            packed |= (ch & 0x7FF) << (i * 11)
        payload = packed.to_bytes(22, 'little')
        
        frame = bytearray([0xC8, 24, 0x16])
        frame.extend(payload)
        crc = crc8_dvb_s2(bytes([0x16]) + payload)
        frame.append(crc)
        
        for _ in range(150):  # 1 second
            self.ser.write(frame)
            time.sleep(0.00667)
            
            if self.ser.in_waiting:
                data = self.ser.read(self.ser.in_waiting)
                if data != b'\x00' * len(data):
                    print("   ✓ Got response during arm sequence!")
                    return True
        return False
    
    def test_method_4_msp_commands(self):
        """Method 4: Send MSP commands"""
        print("\n4. Testing MSP commands...")
        
        msp_packet = self.build_msp_packet()
        self.ser.write(msp_packet)
        time.sleep(0.1)
        
        # Alternate between MSP and RC
        rc_packet = self.build_crsf_rc_packet()
        for i in range(100):
            if i % 10 == 0:
                self.ser.write(msp_packet)
            else:
                self.ser.write(rc_packet)
            time.sleep(0.00667)
            
            if self.ser.in_waiting:
                data = self.ser.read(self.ser.in_waiting)
                if data != b'\x00' * len(data):
                    print("   ✓ Got response with MSP!")
                    return True
        return False
    
    def test_method_5_burst_then_steady(self):
        """Method 5: Burst of packets then steady rate"""
        print("\n5. Testing burst then steady...")
        
        packet = self.build_crsf_rc_packet()
        
        # Send burst
        print("   Sending burst...")
        for _ in range(50):
            self.ser.write(packet)
        time.sleep(0.1)
        
        # Then steady rate
        print("   Sending steady 150Hz...")
        for _ in range(300):
            self.ser.write(packet)
            time.sleep(0.00667)
            
            if self.ser.in_waiting:
                data = self.ser.read(self.ser.in_waiting)
                if data != b'\x00' * len(data):
                    print("   ✓ Got response after burst!")
                    return True
        return False
    
    def maintain_connection(self):
        """Once connected, maintain the connection"""
        print("\n✓ CONNECTION ESTABLISHED!")
        print("Maintaining connection...")
        print("Watch the Bandit screen - should show 'Connected'")
        print("Press Ctrl+C to stop")
        
        packet = self.build_crsf_rc_packet()
        packet_count = 0
        last_status = time.time()
        
        try:
            while True:
                self.ser.write(packet)
                packet_count += 1
                
                if self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting)
                    # Only show non-zero responses
                    if data != b'\x00' * len(data):
                        print(f"Telemetry: {data[:20].hex()}...")
                
                if time.time() - last_status > 1.0:
                    print(f"Status: Sent {packet_count} packets/sec")
                    packet_count = 0
                    last_status = time.time()
                
                time.sleep(0.00667)  # 150Hz
                
        except KeyboardInterrupt:
            print("\nStopped by user")
    
    def run(self):
        """Run all connection tests"""
        print("="*60)
        print("RadioMaster Bandit Connection Test")
        print("="*60)
        print("Testing different methods to establish connection...")
        print("Watch the Bandit screen for status changes")
        print("-"*60)
        
        # Try different baud rates
        for baud in [420000, 400000, 115200]:
            print(f"\nTrying {baud} baud...")
            try:
                self.ser = serial.Serial(
                    port=self.port,
                    baudrate=baud,
                    timeout=0.01,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    rtscts=False,
                    dsrdtr=False
                )
                print(f"✓ Opened port at {baud}")
                
                # Clear buffer
                self.ser.reset_input_buffer()
                
                # Try each method
                methods = [
                    self.test_method_1_basic_crsf,
                    self.test_method_2_radio_id,
                    self.test_method_3_arm_sequence,
                    self.test_method_4_msp_commands,
                    self.test_method_5_burst_then_steady
                ]
                
                for method in methods:
                    if method():
                        self.maintain_connection()
                        return
                
                self.ser.close()
                
            except serial.SerialException as e:
                print(f"✗ Failed at {baud}: {e}")
                continue
        
        print("\n" + "="*60)
        print("No method established connection")
        print("\nPossible issues:")
        print("1. TX and RX not properly bound")
        print("2. Binding phrase mismatch")
        print("3. TX firmware not configured for USB CRSF")
        print("4. Need to use ELRS Configurator for setup")
        print("="*60)

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbserial-0001"
    tester = BanditConnectionTest(port)
    tester.run()