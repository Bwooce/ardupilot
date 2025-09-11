#!/usr/bin/env python3
"""
Check what mode the RadioMaster Bandit is in
Try different communication methods to identify its state
"""

import serial
import time
import sys

def check_bandit(port="/dev/cu.usbserial-0001"):
    """Try to identify Bandit's mode"""
    
    print("="*60)
    print("RadioMaster Bandit Mode Check")
    print("="*60)
    
    # Test 1: Check for ELRS AT commands
    print("\n1. Testing AT commands (configuration mode)...")
    for baud in [115200, 420000]:
        try:
            ser = serial.Serial(port, baud, timeout=1)
            
            # Try ELRS AT commands
            ser.write(b'AT\r\n')
            time.sleep(0.1)
            response = ser.read(100)
            if response and response != b'\x00' * len(response):
                print(f"  ✓ Got response at {baud}: {response}")
                
            # Try to get version
            ser.write(b'AT+VERSION\r\n')
            time.sleep(0.1)
            response = ser.read(100)
            if response and response != b'\x00' * len(response):
                print(f"  Version: {response}")
                
            ser.close()
        except:
            pass
    
    # Test 2: Check if it's in bootloader mode
    print("\n2. Checking for bootloader/DFU mode...")
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        # STM32 bootloader typically responds to 0x7F
        ser.write(b'\x7F')
        time.sleep(0.1)
        response = ser.read(10)
        if response == b'\x79' or response == b'y':
            print("  ⚠ Device is in bootloader mode!")
            print("  Need to reset to normal mode")
        else:
            print(f"  Not in bootloader (got: {response.hex()})")
        ser.close()
    except:
        pass
    
    # Test 3: Send CRSF and check LED behavior
    print("\n3. Testing CRSF with LED observation...")
    print("  Watch the Bandit's LED while this runs:")
    try:
        ser = serial.Serial(port, 420000, timeout=0.1)
        
        # Build valid CRSF RC packet
        channels = [992] * 16  # All centered
        packed = 0
        for i, ch in enumerate(channels):
            packed |= (ch & 0x7FF) << (i * 11)
        payload = packed.to_bytes(22, 'little')
        
        frame = bytearray([0xC8, 24, 0x16])
        frame.extend(payload)
        crc = 0
        for byte in bytes([0x16]) + payload:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0xD5
                else:
                    crc = crc << 1
                crc &= 0xFF
        frame.append(crc)
        
        print("  Sending CRSF for 5 seconds...")
        print("  LED states:")
        print("    - Solid = Connected to RX")
        print("    - Fast blink = Searching for RX")
        print("    - Slow blink = No handset detected")
        print("    - Off = Not powered or wrong mode")
        
        start = time.time()
        responses = set()
        while time.time() - start < 5:
            ser.write(frame)
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                # Check if response is not all zeros
                if data != b'\x00' * len(data):
                    responses.add(data[:10].hex())
            time.sleep(0.00667)
        
        if responses:
            print(f"\n  Got non-zero responses:")
            for r in responses:
                print(f"    {r}")
        else:
            print("\n  Only zero responses - TX not transmitting telemetry")
            
        ser.close()
    except Exception as e:
        print(f"  Error: {e}")
    
    # Test 4: Check binding
    print("\n4. Binding state check...")
    print("  If LED is blinking slowly (2x pattern):")
    print("    → TX is in bind mode")
    print("  If LED is solid:")
    print("    → TX is bound and connected to RX")
    
    print("\n" + "="*60)
    print("Diagnosis:")
    print("-"*60)
    
    print("""
Possible issues:
1. TX and RX not bound
   → Solution: Bind them using button sequences
   
2. TX in wrong mode (not MAVLink)
   → Solution: Use ELRS Configurator to set TX to MAVLink mode
   
3. RX not powered or out of range
   → Solution: Power the RX and bring it closer
   
4. Binding phrase mismatch
   → Solution: Flash both TX and RX with same binding phrase
   
5. Firmware version mismatch
   → Solution: Update both to ELRS 3.x with MAVLink support

To properly configure the Bandit for MAVLink:
1. Use ELRS Configurator (not via USB serial)
2. Flash with MAVLink-enabled firmware
3. Set binding phrase (same on TX and RX)
4. Configure RX for MAVLink output
""")
    print("="*60)

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbserial-0001"
    check_bandit(port)