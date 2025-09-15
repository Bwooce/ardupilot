#!/usr/bin/env python3
"""
ArduPilot ESP32 Serial Monitor
Monitors ESP32 serial output for boot status, crashes, and filesystem messages
"""

import serial
import time
import sys
import signal
from datetime import datetime

class ESP32SerialMonitor:
    def __init__(self, port='/dev/ttyACM0', timeout=30):
        self.port = port
        self.timeout = timeout
        self.serial_conn = None
        self.start_time = time.time()
        self.boot_detected = False
        self.crashes_detected = []
        self.filesystem_messages = []
        self.error_count = 0

    def try_connect(self, baudrate):
        """Try to connect at specified baud rate"""
        try:
            if self.serial_conn:
                self.serial_conn.close()

            print(f"[INFO] Trying to connect at {baudrate} baud...")
            self.serial_conn = serial.Serial(
                self.port,
                baudrate,
                timeout=2,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            time.sleep(0.5)  # Allow connection to stabilize
            return True
        except Exception as e:
            print(f"[ERROR] Failed to connect at {baudrate}: {e}")
            return False

    def analyze_line(self, line):
        """Analyze a line of serial output for important patterns"""
        line_lower = line.lower()
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]

        # Boot detection patterns
        if any(pattern in line_lower for pattern in [
            'ardupilot rover', 'boot complete', 'ready to fly',
            'armable:', 'gps:', 'starting rover'
        ]):
            self.boot_detected = True
            print(f"[BOOT] {timestamp}: {line}")

        # Crash detection patterns
        elif any(pattern in line_lower for pattern in [
            'guru meditation', 'panic', 'abort', 'watchdog', 'stack overflow',
            'hard fault', 'exception', 'crash', 'core dump', 'backtrace'
        ]):
            self.crashes_detected.append(f"{timestamp}: {line}")
            print(f"[CRITICAL] {timestamp}: CRASH DETECTED - {line}")

        # Filesystem patterns
        elif any(pattern in line_lower for pattern in [
            'spiffs', 'fatfs', 'filesystem', 'mount', 'log file', 'dataflash',
            'logger', 'hal_os_fatfs'
        ]):
            self.filesystem_messages.append(f"{timestamp}: {line}")
            print(f"[FILESYSTEM] {timestamp}: {line}")

        # Error patterns
        elif any(pattern in line_lower for pattern in [
            'error', 'fail', 'timeout', 'invalid', 'cannot', 'unable'
        ]):
            self.error_count += 1
            print(f"[ERROR] {timestamp}: {line}")

        # Info/debug patterns
        elif any(pattern in line_lower for pattern in [
            'info', 'debug', 'init', 'setup', 'config'
        ]):
            print(f"[INFO] {timestamp}: {line}")

        # All other output
        else:
            print(f"[DATA] {timestamp}: {line}")

    def monitor(self):
        """Main monitoring loop"""
        baud_rates = [115200, 921600, 57600, 9600]

        for baudrate in baud_rates:
            if self.try_connect(baudrate):
                print(f"[INFO] Connected successfully at {baudrate} baud")
                break
        else:
            print("[ERROR] Failed to connect at any baud rate")
            return False

        print(f"[INFO] Starting monitoring for {self.timeout} seconds...")
        print(f"[INFO] Monitoring {self.port} for ArduPilot ESP32 output...")
        print("-" * 80)

        try:
            while time.time() - self.start_time < self.timeout:
                try:
                    if self.serial_conn.in_waiting > 0:
                        line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            self.analyze_line(line)
                    else:
                        time.sleep(0.1)

                except UnicodeDecodeError:
                    print("[WARNING] Serial data decode error - possible corruption")
                except serial.SerialException as e:
                    print(f"[ERROR] Serial error: {e}")
                    break

        except KeyboardInterrupt:
            print("\n[INFO] Monitoring interrupted by user")

        finally:
            if self.serial_conn:
                self.serial_conn.close()

        return True

    def report_summary(self):
        """Generate final monitoring report"""
        print("\n" + "=" * 80)
        print("MONITORING SUMMARY")
        print("=" * 80)

        print(f"1. Boot Status: {'✅ SUCCESS' if self.boot_detected else '❌ NO BOOT DETECTED'}")

        print(f"2. Crash Messages: {len(self.crashes_detected)} detected")
        if self.crashes_detected:
            for crash in self.crashes_detected:
                print(f"   - {crash}")
        else:
            print("   ✅ No crashes detected")

        print(f"3. Filesystem Messages: {len(self.filesystem_messages)} detected")
        if self.filesystem_messages:
            for msg in self.filesystem_messages[-5:]:  # Show last 5
                print(f"   - {msg}")
            if len(self.filesystem_messages) > 5:
                print(f"   ... and {len(self.filesystem_messages) - 5} more")
        else:
            print("   ⚠️  No filesystem messages detected")

        print(f"4. Error Count: {self.error_count}")

        # Recommendations
        print("\nRECOMMENDations:")
        if not self.boot_detected:
            print("- Device may need hardware reset")
            print("- Check power supply and connections")
            print("- Verify firmware was uploaded successfully")

        if self.crashes_detected:
            print("- CRITICAL: Investigate crash causes immediately")
            print("- Check stack usage and memory allocation")

        if not self.filesystem_messages:
            print("- Check if HAL_OS_FATFS_IO=0 setting is working")
            print("- Verify ESP32 filesystem backend configuration")

def main():
    if len(sys.argv) > 1:
        timeout = int(sys.argv[1])
    else:
        timeout = 30

    monitor = ESP32SerialMonitor(timeout=timeout)

    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        print('\n[INFO] Stopping monitor...')
        monitor.report_summary()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    if monitor.monitor():
        monitor.report_summary()

    return 0

if __name__ == "__main__":
    sys.exit(main())