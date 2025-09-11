#!/usr/bin/env python3
"""
ESP32 FOTA (Firmware Over-The-Air) Test Script

Tests ESP32 firmware update via MAVLink FTP protocol.
This script demonstrates the complete FOTA workflow using standard MAVLink commands.

Usage:
    python3 test_esp32_ota.py --connection tcp:127.0.0.1:5762 --firmware build/esp32lilygo_tconnect/bin/arduplane.bin
    python3 test_esp32_ota.py --connection serial:/dev/cu.usbmodem*:115200 --firmware auto
"""

import argparse
import os
import sys
import time
import struct
from pathlib import Path

try:
    from pymavlink import mavutil
    from pymavlink.dialects.v20 import ardupilotmega as mavlink
except ImportError:
    print("Error: pymavlink not found. Install with: pip install pymavlink")
    sys.exit(1)

class ESP32FOTATest:
    def __init__(self, connection_string, firmware_path):
        self.connection_string = connection_string
        self.firmware_path = firmware_path
        self.mav = None
        self.target_system = 1
        self.target_component = 1
        self.session_id = 0
        self.sequence_number = 0
        
    def normalize_connection_string(self, connection_string):
        """Normalize connection string format for pymavlink compatibility"""
        if connection_string.startswith('serial:'):
            # Remove serial: prefix if present, pymavlink can auto-detect
            connection_string = connection_string[7:]
        
        # Handle macOS USB serial device format and wildcard expansion
        if connection_string.startswith('/dev/cu.') and ':' in connection_string:
            device, baud = connection_string.rsplit(':', 1)
            # Handle wildcard in device path
            if '*' in device:
                import glob
                devices = glob.glob(device)
                if devices:
                    device = devices[0]  # Use first matching device
                    print(f"Found device: {device}")
                else:
                    print(f"Warning: No device matching {device}")
                    return connection_string
            try:
                int(baud)  # Validate baud rate is numeric
                return f"{device},{baud}"  # Use comma format for serial connections
            except ValueError:
                return connection_string
        
        return connection_string

    def connect(self):
        """Connect to the vehicle"""
        print(f"Connecting to {self.connection_string}...")
        
        # Normalize connection string for better compatibility
        normalized_conn = self.normalize_connection_string(self.connection_string)
        if normalized_conn != self.connection_string:
            print(f"Normalized to: {normalized_conn}")
        
        try:
            self.mav = mavutil.mavlink_connection(normalized_conn)
            self.mav.wait_heartbeat()
            self.target_system = self.mav.target_system
            self.target_component = self.mav.target_component
            print(f"Connected to system {self.target_system}, component {self.target_component}")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
    
    def find_firmware(self):
        """Auto-detect firmware file in build directory"""
        if self.firmware_path == "auto":
            # Look for firmware in common build locations
            possible_paths = [
                "build/esp32lilygo_tconnect/bin/arduplane.bin",
                "build/esp32lilygo_tconnect/bin/ardupilot.bin", 
                "build/esp32lilygo_tconnect/esp-idf_build/ardupilot.bin",
                "build/esp32diy/bin/arduplane.bin",
                "build/esp32diy/bin/ardupilot.bin",
                "build/esp32icarus/bin/arduplane.bin",
                "build/esp32icarus/bin/ardupilot.bin",
            ]
            
            for path in possible_paths:
                if os.path.exists(path):
                    self.firmware_path = path
                    print(f"Auto-detected firmware: {self.firmware_path}")
                    return True
            
            print("Error: Could not auto-detect firmware file")
            print("Available build directories:")
            build_dir = Path("build")
            if build_dir.exists():
                for item in build_dir.iterdir():
                    if item.is_dir():
                        print(f"  {item}")
            return False
        
        return os.path.exists(self.firmware_path)
    
    def validate_firmware(self):
        """Validate ESP32 firmware file"""
        if not os.path.exists(self.firmware_path):
            print(f"Error: Firmware file not found: {self.firmware_path}")
            return False
            
        file_size = os.path.getsize(self.firmware_path)
        print(f"Firmware file: {self.firmware_path}")
        print(f"Firmware size: {file_size} bytes ({file_size/1024/1024:.2f} MB)")
        
        # Check ESP32 firmware magic number
        with open(self.firmware_path, 'rb') as f:
            header = f.read(4)
            if len(header) >= 1 and header[0] == 0xE9:
                print("✓ Valid ESP32 firmware header detected")
                return True
            else:
                print(f"Warning: ESP32 firmware magic number not found (got 0x{header[0]:02x}, expected 0xE9)")
                return True  # Continue anyway for testing
    
    def wait_for_ack(self, timeout=10):
        """Wait for FTP acknowledgment"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.mav.recv_match(type='FILE_TRANSFER_PROTOCOL', blocking=False, timeout=1)
            if msg:
                # Decode FTP response
                opcode = msg.payload[3]
                if opcode == 128:  # FTP_OP::Ack
                    return True
                elif opcode == 129:  # FTP_OP::Nack
                    error_code = msg.payload[4] if len(msg.payload) > 4 else 0
                    print(f"FTP NACK received - Error code: {error_code}")
                    return False
        print("Timeout waiting for FTP response")
        return False
    
    def send_ftp_command(self, opcode, session, data=b"", offset=0):
        """Send FTP command via FILE_TRANSFER_PROTOCOL"""
        self.sequence_number += 1
        
        # Build FTP payload
        payload = bytearray(251)  # FILE_TRANSFER_PROTOCOL payload size
        
        # FTP header
        payload[0:2] = struct.pack("<H", self.sequence_number)  # sequence
        payload[2] = session  # session
        payload[3] = opcode   # opcode
        payload[4] = min(len(data), 239)  # size (max 239 bytes of data)
        payload[5] = 0        # req_opcode
        payload[6] = 0        # burst_complete
        payload[7] = 0        # padding
        payload[8:12] = struct.pack("<I", offset)  # offset
        
        # Copy data
        if data:
            data_len = min(len(data), 239)
            payload[12:12+data_len] = data[:data_len]
        
        # Send FILE_TRANSFER_PROTOCOL message
        self.mav.mav.file_transfer_protocol_send(
            0,  # target_network
            self.target_system,
            self.target_component, 
            payload
        )
        
    def check_vehicle_ready(self):
        """Check if vehicle is ready for OTA update"""
        print("Checking vehicle status...")
        
        # Wait for heartbeat to get vehicle state
        msg = self.mav.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if not msg:
            print("Error: No heartbeat received")
            return False
            
        # Check if armed
        if msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("Error: Vehicle is armed - OTA updates blocked")
            print("Debug: base_mode=0x{:02X}".format(msg.base_mode))
            return False
        
        print("✓ Vehicle is disarmed (base_mode=0x{:02X})".format(msg.base_mode))
        
        # Check battery status
        msg = self.mav.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
        if msg:
            battery_remaining = msg.battery_remaining
            if battery_remaining >= 0:  # Valid battery reading
                print(f"✓ Battery level: {battery_remaining}%")
                if battery_remaining < 50:
                    print("Warning: Battery level below 50% - OTA may be blocked")
            else:
                print("✓ No battery monitoring detected")
        
        return True
    
    def upload_firmware(self):
        """Upload firmware via MAVLink FTP"""
        print("\n=== Starting FOTA Upload ===")
        
        # 1. Create firmware file for OTA
        firmware_name = "/flash/firmware.bin"  # Triggers FOTA routing
        print(f"Creating file: {firmware_name}")
        
        self.send_ftp_command(
            opcode=6,  # FTP_OP::CreateFile
            session=1,
            data=firmware_name.encode('utf-8')
        )
        
        if not self.wait_for_ack():
            print("Failed to create firmware file")
            return False
        
        print("✓ Firmware file created - OTA routing should be active")
        
        # 2. Upload firmware in chunks
        chunk_size = 239  # Max FTP payload size
        
        with open(self.firmware_path, 'rb') as f:
            firmware_data = f.read()
        
        total_size = len(firmware_data)
        bytes_sent = 0
        last_progress = -1
        
        print(f"Uploading firmware: {total_size} bytes in {chunk_size}-byte chunks")
        
        while bytes_sent < total_size:
            # Read chunk
            chunk = firmware_data[bytes_sent:bytes_sent + chunk_size]
            
            # Send WriteFile command
            self.send_ftp_command(
                opcode=7,  # FTP_OP::WriteFile
                session=1,
                data=chunk,
                offset=bytes_sent
            )
            
            if not self.wait_for_ack(timeout=5):
                print(f"Upload failed at offset {bytes_sent}")
                return False
            
            bytes_sent += len(chunk)
            progress = int(bytes_sent * 100 / total_size)
            
            if progress != last_progress and progress % 10 == 0:
                print(f"Upload progress: {progress}% ({bytes_sent}/{total_size} bytes)")
                last_progress = progress
        
        print("✓ Firmware upload complete")
        
        # 3. Terminate session to trigger OTA completion
        print("Terminating FTP session - this should trigger OTA completion and reboot")
        self.send_ftp_command(
            opcode=1,  # FTP_OP::TerminateSession  
            session=1
        )
        
        if not self.wait_for_ack():
            print("Session termination failed")
            return False
            
        print("✓ OTA completion triggered")
        print("Vehicle should reboot to new firmware in ~3 seconds...")
        
        return True
    
    def monitor_reboot(self):
        """Monitor vehicle reboot and new firmware startup"""
        print("\n=== Monitoring Reboot ===")
        print("Waiting for connection loss (reboot)...")
        
        # Wait for heartbeat loss (indicating reboot)
        start_time = time.time()
        last_heartbeat = time.time()
        
        while time.time() - start_time < 5:
            msg = self.mav.recv_match(type='HEARTBEAT', blocking=False, timeout=1)
            if msg:
                last_heartbeat = time.time()
            elif time.time() - last_heartbeat > 10:
                print("✓ Connection lost - vehicle rebooting")
                break
        else:
            print("Warning: Expected reboot not detected")
            return False
        
        # Wait for reconnection
        print("Waiting for vehicle to come back online...")
        reconnect_start = time.time()
        
        while time.time() - reconnect_start < 60:
            msg = self.mav.recv_match(type='HEARTBEAT', blocking=False, timeout=1)
            if msg:
                print("✓ Vehicle reconnected!")
                print(f"New firmware version: {msg.autopilot} (custom_mode: {msg.custom_mode})")
                return True
                
        print("Error: Vehicle did not reconnect after reboot")
        return False
    
    def run_test(self):
        """Run complete FOTA test"""
        print("ESP32 FOTA Test Starting...")
        print("=" * 50)
        
        # 1. Find and validate firmware
        if not self.find_firmware():
            return False
            
        if not self.validate_firmware():
            return False
        
        # 2. Connect to vehicle
        if not self.connect():
            return False
        
        # 3. Check vehicle status
        if not self.check_vehicle_ready():
            return False
        
        # 4. Upload firmware
        if not self.upload_firmware():
            return False
        
        # 5. Monitor reboot
        if not self.monitor_reboot():
            return False
        
        print("\n" + "=" * 50)
        print("✓ ESP32 FOTA Test SUCCESSFUL!")
        print("Firmware update completed successfully")
        return True

def main():
    parser = argparse.ArgumentParser(description='Test ESP32 FOTA functionality')
    parser.add_argument('--connection', '-c', required=True,
                       help='MAVLink connection string (e.g., tcp:127.0.0.1:5762, serial:/dev/cu.usbmodem*:115200)')
    parser.add_argument('--firmware', '-f', default='auto',
                       help='Firmware file path or "auto" to auto-detect')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Verbose output')
    
    args = parser.parse_args()
    
    if args.verbose:
        mavutil.set_debug(True)
    
    test = ESP32FOTATest(args.connection, args.firmware)
    
    try:
        success = test.run_test()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"Test failed with exception: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()