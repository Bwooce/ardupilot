#!/usr/bin/env python3
"""
ESP32 Upload Wrapper with Serial Port Lock

This script wraps the ESP32 upload process with proper lock file coordination
to prevent conflicts with monitoring tools.
"""

import os
import sys
import subprocess
import argparse
from pathlib import Path

# Add current directory to path for imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from serial_port_lock import SerialPortLock, wait_for_port

def find_serial_port():
    """Find the first available ESP32 serial port"""
    common_ports = [
        '/dev/ttyACM0',
        '/dev/ttyUSB0',
        '/dev/ttyACM1',
        '/dev/ttyUSB1'
    ]

    for port in common_ports:
        if Path(port).exists():
            return port

    # Check for any ttyUSB or ttyACM
    for port in Path('/dev').glob('ttyUSB*'):
        return str(port)
    for port in Path('/dev').glob('ttyACM*'):
        return str(port)

    return None

def upload_firmware(port, firmware_path=None, baudrate=921600, restart_monitor=True):
    """
    Upload firmware to ESP32 with serial port locking

    Args:
        port: Serial port path
        firmware_path: Path to firmware file (optional, uses waf default)
        baudrate: Upload baud rate
        restart_monitor: Automatically restart monitor after upload

    Returns:
        True if successful, False otherwise
    """
    # Acquire lock for upload
    print(f"Acquiring lock for {port}...")
    try:
        with SerialPortLock(port, "esp32_upload") as lock:
            print(f"Lock acquired, starting upload...")

            if firmware_path:
                # Direct esptool upload
                cmd = [
                    'esptool.py',
                    '--port', port,
                    '--baud', str(baudrate),
                    '--before', 'default_reset',
                    '--after', 'hard_reset',
                    'write_flash',
                    '-z',
                    '0x0', firmware_path
                ]
            else:
                # Use waf upload for rover (assumes we're in ardupilot directory)
                # The --upload flag tells waf to upload after building
                cmd = ['./waf', 'rover', '--upload', f'--upload-port={port}']

            print(f"Running: {' '.join(cmd)}")
            result = subprocess.run(cmd, capture_output=False, text=True)

            if result.returncode == 0:
                print("✓ Upload successful!")

                # Automatically restart monitor if requested
                if restart_monitor:
                    print("\n🔄 Restarting serial monitor...")
                    # Wait for device to reset after upload
                    print("   Waiting for device reset...")
                    time.sleep(3)

                    # Start monitor in background
                    monitor_cmd = [
                        'python3',
                        '/home/bruce/ardupilot/esp32_monitor_agent.py',
                        '--port', port,
                        '--interval', '30'
                    ]

                    try:
                        # Open log file for monitor output
                        log_file = open('/tmp/esp32_monitor.log', 'w')

                        # Use subprocess.Popen to start in background
                        monitor_process = subprocess.Popen(
                            monitor_cmd,
                            stdout=log_file,
                            stderr=subprocess.STDOUT,
                            start_new_session=True
                        )
                        print(f"✓ Monitor restarted in background (PID {monitor_process.pid})")
                        print(f"  To view output: tail -f /tmp/esp32_monitor.log")
                        print(f"  To stop monitor: kill {monitor_process.pid}")
                    except Exception as e:
                        print(f"⚠️  Could not restart monitor: {e}")
                        print(f"  Start manually: python3 /home/bruce/ardupilot/esp32_monitor_agent.py --port {port}")

                return True
            else:
                print(f"✗ Upload failed with code {result.returncode}")
                return False

    except RuntimeError as e:
        print(f"✗ Failed to acquire lock: {e}")

        # Check who has the lock
        temp_lock = SerialPortLock(port, "check")
        info = temp_lock.get_lock_info()
        if info:
            pid, tool = info
            print(f"Port {port} is currently locked by {tool} (PID {pid})")
            print(f"You may need to stop the monitoring agent first:")
            print(f"  kill {pid}")

        return False
    except Exception as e:
        print(f"✗ Upload error: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description='ESP32 firmware upload with port locking')
    parser.add_argument('--port', help='Serial port (auto-detect if not specified)')
    parser.add_argument('--firmware', help='Firmware file path (use waf if not specified)')
    parser.add_argument('--baud', type=int, default=921600, help='Upload baud rate')
    parser.add_argument('--force', action='store_true', help='Force kill existing lock holder')
    parser.add_argument('--no-monitor', action='store_true', help='Do not restart monitor after upload')

    args = parser.parse_args()

    # Find serial port
    port = args.port
    if not port:
        port = find_serial_port()
        if not port:
            print("✗ No ESP32 serial port found!")
            print("Please connect your ESP32 and try again.")
            sys.exit(1)
        print(f"Auto-detected port: {port}")

    # Check if port exists
    if not Path(port).exists():
        print(f"✗ Port {port} does not exist!")
        sys.exit(1)

    # Handle force option
    if args.force:
        lock = SerialPortLock(port, "force_check")
        info = lock.get_lock_info()
        if info:
            pid, tool = info
            print(f"Force killing {tool} (PID {pid})...")
            try:
                os.kill(pid, 9)
                time.sleep(1)
            except ProcessLookupError:
                pass

    # Perform upload
    success = upload_firmware(port, args.firmware, args.baud, restart_monitor=not args.no_monitor)
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    import time
    main()