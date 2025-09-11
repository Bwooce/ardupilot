#!/usr/bin/env python3
"""
Simple ESP32 FOTA Test Script

Basic test to verify firmware file and connection without performing actual upload.
Useful for development and debugging.

Usage:
    python3 test_esp32_ota_simple.py --connection tcp:127.0.0.1:5762
    python3 test_esp32_ota_simple.py --connection serial:/dev/cu.usbmodem*:115200 --check-only
"""

import argparse
import os
import sys
import time
from pathlib import Path

try:
    from pymavlink import mavutil
    from pymavlink.dialects.v20 import ardupilotmega as mavlink
except ImportError:
    print("Error: pymavlink not found. Install with: pip install pymavlink")
    print("Install with: pip install pymavlink")
    sys.exit(1)

def find_firmware_files():
    """Find all available ESP32 firmware files"""
    print("Searching for ESP32 firmware files...")
    
    build_dir = Path("build")
    firmware_files = []
    
    if not build_dir.exists():
        print("No build directory found")
        return firmware_files
    
    # Search patterns for ESP32 firmware
    patterns = ["**/ardupilot.bin", "**/arduplane.bin", "**/arducopter.bin", "**/ardusub.bin"]
    
    for pattern in patterns:
        for file_path in build_dir.glob(pattern):
            if "esp32" in str(file_path):
                size_mb = file_path.stat().st_size / (1024 * 1024)
                
                # Check ESP32 magic number
                try:
                    with open(file_path, 'rb') as f:
                        magic = f.read(1)[0]
                        is_esp32 = (magic == 0xE9)
                except:
                    is_esp32 = False
                
                firmware_files.append({
                    'path': file_path,
                    'size_mb': size_mb,
                    'is_esp32': is_esp32
                })
    
    return firmware_files

def normalize_connection_string(connection_string):
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

def test_connection(connection_string):
    """Test MAVLink connection"""
    print(f"Testing connection: {connection_string}")
    
    # Normalize connection string for better compatibility
    normalized_conn = normalize_connection_string(connection_string)
    if normalized_conn != connection_string:
        print(f"Normalized to: {normalized_conn}")
    
    try:
        mav = mavutil.mavlink_connection(normalized_conn)
        print("Waiting for heartbeat...")
        
        # Wait for heartbeat with timeout
        msg = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=10)
        if not msg:
            print("❌ No heartbeat received")
            return False
        
        print(f"✅ Connected to system {mav.target_system}, component {mav.target_component}")
        print(f"   Vehicle type: {msg.type}")
        print(f"   Autopilot: {msg.autopilot}")
        print(f"   Mode: {msg.custom_mode}")
        print(f"   Armed: {'Yes' if msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED else 'No'}")
        
        # Test FTP capability
        print("\nTesting FTP capability...")
        # Request autopilot version to see if FTP is supported
        mav.mav.command_long_send(
            mav.target_system, mav.target_component,
            mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, 0,
            1, 0, 0, 0, 0, 0, 0
        )
        
        # Look for response
        start_time = time.time()
        while time.time() - start_time < 5:
            msg = mav.recv_match(type='AUTOPILOT_VERSION', blocking=False, timeout=1)
            if msg:
                capabilities = msg.capabilities
                ftp_supported = bool(capabilities & mavlink.MAV_PROTOCOL_CAPABILITY_FTP)
                print(f"   FTP Support: {'Yes' if ftp_supported else 'No'}")
                break
        else:
            print("   FTP Support: Unknown (no response)")
        
        return True
        
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        return False

def test_ota_conditions(connection_string):
    """Test if vehicle is ready for OTA update"""
    print(f"\nTesting OTA readiness...")
    
    # Normalize connection string for better compatibility
    normalized_conn = normalize_connection_string(connection_string)
    
    try:
        mav = mavutil.mavlink_connection(normalized_conn)
        mav.wait_heartbeat(timeout=5)
        
        # Check arming state
        msg = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("❌ Vehicle is ARMED - OTA blocked")
            return False
        print("✅ Vehicle is disarmed")
        
        # Check battery
        print("Checking battery status...")
        msg = mav.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
        if msg and msg.battery_remaining >= 0:
            battery_pct = msg.battery_remaining
            print(f"   Battery: {battery_pct}%")
            if battery_pct < 50:
                print("⚠️  Battery below 50% - OTA may be blocked")
            else:
                print("✅ Battery level sufficient")
        else:
            print("✅ No battery monitoring (allowed for bench testing)")
        
        # Check GPS (optional)
        msg = mav.recv_match(type='GPS_RAW_INT', blocking=True, timeout=3)
        if msg:
            gps_fix = msg.fix_type
            print(f"   GPS Fix: {gps_fix} ({'OK' if gps_fix >= 3 else 'No 3D fix'})")
        else:
            print("   GPS: No data (non-blocking for OTA)")
        
        print("✅ Vehicle appears ready for OTA update")
        return True
        
    except Exception as e:
        print(f"❌ OTA readiness check failed: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description='Simple ESP32 FOTA test')
    parser.add_argument('--connection', '-c', 
                       help='MAVLink connection string (e.g., tcp:127.0.0.1:5762)')
    parser.add_argument('--check-only', action='store_true',
                       help='Only check firmware files, do not test connection')
    
    args = parser.parse_args()
    
    print("ESP32 FOTA Simple Test")
    print("=" * 30)
    
    # 1. Find firmware files
    firmware_files = find_firmware_files()
    
    if not firmware_files:
        print("❌ No ESP32 firmware files found")
        print("   Make sure you've built for an ESP32 target:")
        print("   ./waf configure --board esp32lilygo_tconnect")
        print("   ./waf build")
        return False
    
    print(f"✅ Found {len(firmware_files)} ESP32 firmware file(s):")
    for fw in firmware_files:
        status = "✅" if fw['is_esp32'] else "⚠️"
        print(f"   {status} {fw['path']} ({fw['size_mb']:.1f} MB)")
    
    if args.check_only:
        print("\nCheck-only mode - skipping connection test")
        return True
    
    if not args.connection:
        print("\nNo connection specified - use --connection to test vehicle")
        print("Example connections:")
        print("  --connection tcp:127.0.0.1:5762     (SITL)")
        print("  --connection serial:/dev/cu.usbmodem*:115200  (Serial)")
        return True
    
    # 2. Test connection
    if not test_connection(args.connection):
        return False
    
    # 3. Test OTA readiness
    if not test_ota_conditions(args.connection):
        print("\n⚠️  Vehicle may not be ready for OTA update")
        print("   This is expected during development/testing")
    
    print("\n" + "=" * 30)
    print("✅ Simple test completed")
    print("\nTo perform actual FOTA test:")
    print(f"   python3 test_esp32_ota.py --connection {args.connection}")
    
    return True

if __name__ == "__main__":
    try:
        success = main()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\nTest interrupted")
        sys.exit(1)
    except Exception as e:
        print(f"Test failed: {e}")
        sys.exit(1)