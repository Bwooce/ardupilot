#!/usr/bin/env python3
"""
ESP32 ArduPilot Serial Monitor - Filesystem Fix Verification
Monitor for HAL_OS_FATFS_IO fix effectiveness
"""

import serial
import time
import sys
import re
from datetime import datetime

class ESP32FilesystemMonitor:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, timeout=30):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.start_time = time.time()

        # Patterns to monitor for
        self.patterns = {
            'filesystem_constructor': r'AP_Filesystem_ESP32 constructor called',
            'backend_verification': r'ESP32 Backend verification',
            'open_entry': r'!!! AP_Filesystem_ESP32::open ENTRY - WE MADE IT !!!',
            'log_open_fail': r'Log open fail',
            'log_created': r'[Ll]og.*created|[Ll]og.*opened successfully',
            'spiffs_init': r'SPIFFS.*init|mount.*SPIFFS|SPIFFS.*mount',
            'filesystem_init': r'Filesystem.*init|Init.*filesystem',
            'hal_init': r'HAL.*init|Init.*HAL'
        }

        self.found_messages = {key: [] for key in self.patterns.keys()}
        self.all_output = []

    def monitor(self):
        print(f"[{datetime.now().strftime('%H:%M:%S')}] Starting ESP32 filesystem monitoring on {self.port}")
        print("Looking for HAL_OS_FATFS_IO fix effectiveness...")
        print("=" * 80)

        try:
            with serial.Serial(self.port, self.baudrate, timeout=1) as ser:
                # Clear any existing data
                ser.reset_input_buffer()

                while time.time() - self.start_time < self.timeout:
                    try:
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                            timestamped_line = f"[{timestamp}] {line}"
                            self.all_output.append(timestamped_line)

                            # Check against our patterns
                            self.check_patterns(line, timestamp)

                            # Print important messages immediately
                            if any(keyword in line.lower() for keyword in ['filesystem', 'spiffs', 'log', 'open', 'mount', 'backend']):
                                print(f"[FILESYSTEM] {timestamped_line}")

                    except UnicodeDecodeError:
                        # Handle corrupted serial data
                        print(f"[{datetime.now().strftime('%H:%M:%S')}] [WARNING] Serial data corruption detected")
                        continue
                    except serial.SerialException as e:
                        print(f"[ERROR] Serial communication error: {e}")
                        break

        except serial.SerialException as e:
            print(f"[ERROR] Cannot open serial port {self.port}: {e}")
            return False
        except KeyboardInterrupt:
            print("\n[INFO] Monitoring interrupted by user")

        return True

    def check_patterns(self, line, timestamp):
        """Check line against all monitored patterns"""
        for pattern_name, pattern in self.patterns.items():
            if re.search(pattern, line, re.IGNORECASE):
                self.found_messages[pattern_name].append((timestamp, line))

    def generate_report(self):
        """Generate comprehensive monitoring report"""
        print("\n" + "=" * 80)
        print("ESP32 FILESYSTEM MONITORING REPORT")
        print("=" * 80)

        # Critical success indicators
        print("\n🔍 FILESYSTEM FIX VERIFICATION:")

        if self.found_messages['filesystem_constructor']:
            print("✅ AP_Filesystem_ESP32 constructor called - BACKEND SELECTION FIXED")
            for ts, msg in self.found_messages['filesystem_constructor']:
                print(f"   [{ts}] {msg}")
        else:
            print("❌ AP_Filesystem_ESP32 constructor NOT detected - Fix may not be working")

        if self.found_messages['backend_verification']:
            print("✅ ESP32 Backend verification messages found")
            for ts, msg in self.found_messages['backend_verification']:
                print(f"   [{ts}] {msg}")
        else:
            print("⚠️  ESP32 Backend verification messages not found")

        if self.found_messages['open_entry']:
            print("✅ File open operations reaching ESP32 backend - MAJOR SUCCESS")
            for ts, msg in self.found_messages['open_entry']:
                print(f"   [{ts}] {msg}")
        else:
            print("❌ File open operations not reaching ESP32 backend")

        # Error indicators
        print("\n🚨 ERROR INDICATORS:")
        if self.found_messages['log_open_fail']:
            print("❌ Log open failures detected - Fix incomplete")
            for ts, msg in self.found_messages['log_open_fail']:
                print(f"   [{ts}] {msg}")
        else:
            print("✅ No 'Log open fail' errors detected")

        # Success indicators
        print("\n🎯 SUCCESS INDICATORS:")
        if self.found_messages['log_created']:
            print("✅ Successful log operations detected")
            for ts, msg in self.found_messages['log_created']:
                print(f"   [{ts}] {msg}")
        else:
            print("⚠️  No explicit log creation success messages found")

        if self.found_messages['spiffs_init']:
            print("✅ SPIFFS initialization detected")
            for ts, msg in self.found_messages['spiffs_init']:
                print(f"   [{ts}] {msg}")
        else:
            print("⚠️  SPIFFS initialization messages not found")

        # Overall assessment
        print("\n📊 OVERALL ASSESSMENT:")

        critical_success = bool(self.found_messages['filesystem_constructor'] and
                              not self.found_messages['log_open_fail'])

        if critical_success:
            if self.found_messages['open_entry']:
                print("🎉 EXCELLENT: HAL_OS_FATFS_IO fix appears to be working perfectly!")
                print("   - ESP32 filesystem backend is being selected")
                print("   - File operations are reaching the ESP32 backend")
                print("   - No log open failures detected")
            else:
                print("✅ GOOD: HAL_OS_FATFS_IO fix appears to be working")
                print("   - ESP32 filesystem backend is being selected")
                print("   - No log open failures detected")
                print("   - File operations may not have occurred yet during boot")
        else:
            if not self.found_messages['filesystem_constructor']:
                print("❌ CRITICAL: Fix not working - ESP32 filesystem backend not being selected")
            elif self.found_messages['log_open_fail']:
                print("⚠️  PARTIAL: Backend selection fixed but file operations still failing")

        # Additional context
        print(f"\n📈 MONITORING STATISTICS:")
        print(f"   - Monitoring duration: {self.timeout} seconds")
        print(f"   - Total lines captured: {len(self.all_output)}")
        print(f"   - Filesystem-related messages: {sum(len(msgs) for msgs in self.found_messages.values())}")

        # Show recent output for context
        print(f"\n📋 RECENT OUTPUT (last 10 lines):")
        for line in self.all_output[-10:]:
            print(f"   {line}")

def main():
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = '/dev/ttyACM0'

    monitor = ESP32FilesystemMonitor(port=port, timeout=30)

    if monitor.monitor():
        monitor.generate_report()
    else:
        print("Monitoring failed - check serial connection")
        sys.exit(1)

if __name__ == "__main__":
    main()