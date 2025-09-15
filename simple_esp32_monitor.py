#!/usr/bin/env python3
"""
Simple ESP32 monitor with better error handling
"""

import serial
import time
import sys
import signal
from datetime import datetime

class SimpleESP32Monitor:
    def __init__(self):
        self.running = True
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, signum, frame):
        print("\nReceived interrupt signal, shutting down...")
        self.running = False

    def monitor(self, port='/dev/ttyACM0', baud=115200, timeout=30):
        print(f"[{datetime.now().strftime('%H:%M:%S')}] ESP32 Monitor starting")
        print(f"Port: {port}, Baud: {baud}, Timeout: {timeout}s")
        print("=" * 60)

        filesystem_messages = []
        all_messages = []
        start_time = time.time()

        try:
            ser = serial.Serial(port, baud, timeout=0.5)
            print(f"[{datetime.now().strftime('%H:%M:%S')}] Serial port opened successfully")

            # Clear buffer
            ser.reset_input_buffer()

            while self.running and (time.time() - start_time) < timeout:
                try:
                    if ser.in_waiting > 0:
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                            timestamped_line = f"[{timestamp}] {line}"
                            all_messages.append(timestamped_line)

                            # Check for filesystem-related messages
                            line_lower = line.lower()
                            if any(keyword in line_lower for keyword in [
                                'filesystem', 'spiffs', 'log', 'open', 'mount',
                                'backend', 'ap_filesystem', 'fatfs'
                            ]):
                                filesystem_messages.append(timestamped_line)
                                print(f"[FS] {timestamped_line}")

                            # Check for critical patterns
                            if 'AP_Filesystem_ESP32 constructor called' in line:
                                print(f"[CRITICAL SUCCESS] {timestamped_line}")
                            elif '!!! AP_Filesystem_ESP32::open ENTRY' in line:
                                print(f"[MAJOR SUCCESS] {timestamped_line}")
                            elif 'Log open fail' in line:
                                print(f"[ERROR] {timestamped_line}")

                            # Print all non-empty lines for debugging
                            if len(all_messages) <= 20:  # First 20 messages
                                print(f"[RAW] {timestamped_line}")

                    else:
                        time.sleep(0.1)

                except UnicodeDecodeError:
                    print(f"[{datetime.now().strftime('%H:%M:%S')}] [WARNING] Serial decode error")
                except Exception as e:
                    print(f"[{datetime.now().strftime('%H:%M:%S')}] [ERROR] {e}")

            ser.close()

        except serial.SerialException as e:
            print(f"[ERROR] Serial port error: {e}")
            return False, [], []
        except Exception as e:
            print(f"[ERROR] Unexpected error: {e}")
            return False, [], []

        return True, filesystem_messages, all_messages

    def generate_report(self, success, filesystem_messages, all_messages):
        print("\n" + "=" * 60)
        print("ESP32 MONITORING REPORT")
        print("=" * 60)

        print(f"\nTotal messages received: {len(all_messages)}")
        print(f"Filesystem-related messages: {len(filesystem_messages)}")

        if not success:
            print("❌ Monitoring failed - check serial connection")
            return

        if not all_messages:
            print("❌ No serial data received")
            print("Possible causes:")
            print("  - ESP32 not connected or powered")
            print("  - Firmware not running")
            print("  - Wrong baud rate")
            print("  - Device in boot loop or crashed")
            return

        # Look for specific patterns
        constructor_found = any('AP_Filesystem_ESP32 constructor called' in msg for msg in filesystem_messages)
        open_entry_found = any('!!! AP_Filesystem_ESP32::open ENTRY' in msg for msg in filesystem_messages)
        log_fail_found = any('Log open fail' in msg for msg in all_messages)

        print("\n🔍 FILESYSTEM FIX STATUS:")
        if constructor_found:
            print("✅ ESP32 filesystem constructor detected - Backend selection working!")
        else:
            print("❌ ESP32 filesystem constructor NOT detected")

        if open_entry_found:
            print("✅ File operations reaching ESP32 backend - Fix successful!")
        else:
            print("❌ File operations not detected in ESP32 backend")

        if log_fail_found:
            print("❌ Log open failures still occurring")
        else:
            print("✅ No log open failures detected")

        print("\n📋 FILESYSTEM MESSAGES:")
        for msg in filesystem_messages[-10:]:  # Last 10 filesystem messages
            print(f"  {msg}")

        print("\n📋 RECENT OUTPUT:")
        for msg in all_messages[-5:]:  # Last 5 messages
            print(f"  {msg}")

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'

    monitor = SimpleESP32Monitor()
    success, fs_msgs, all_msgs = monitor.monitor(port, timeout=30)
    monitor.generate_report(success, fs_msgs, all_msgs)

if __name__ == "__main__":
    main()