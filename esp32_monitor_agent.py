#!/usr/bin/env python3
"""
ESP32 Error Monitor Agent
Monitors ESP32 serial output, collects errors, and provides summaries
"""

import serial
import sys
import re
import time
from datetime import datetime
from collections import defaultdict, deque
import signal
import argparse
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from serial_port_lock import SerialPortLock

class ESP32ErrorMonitor:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.port_lock = None
        self.running = True
        self.graceful_exit = False  # Flag for graceful exit on SIGUSR1

        # Error tracking
        self.errors = defaultdict(list)  # category -> list of (timestamp, message)
        self.warnings = defaultdict(list)
        self.recent_lines = deque(maxlen=100)  # Keep last 100 lines for context
        self.error_counts = defaultdict(int)
        self.warning_counts = defaultdict(int)

        # Patterns to detect issues
        self.error_patterns = [
            (r'Log open fail.*No space left', 'SPIFFS_SPACE'),
            (r'E \(\d+\) ([^:]+):', 'ESP_ERROR'),
            (r'assertion.*failed', 'ASSERT'),
            (r'Guru Meditation Error', 'CRASH'),
            (r'abort\(\) was called', 'ABORT'),
            (r'Stack overflow', 'STACK_OVERFLOW'),
            (r'failed.*errno=(\d+)', 'ERRNO'),
            (r'SPIFFS.*failed', 'SPIFFS_FAIL'),
            (r'malloc.*failed', 'MEMORY'),
            (r'heap.*corrupt', 'HEAP_CORRUPT'),
        ]

        self.warning_patterns = [
            (r'W \(\d+\) ([^:]+):', 'ESP_WARN'),
            (r'TX timeout', 'CAN_TIMEOUT'),
            (r'UART.*drop', 'UART_DROP'),
            (r'buffer.*full', 'BUFFER_FULL'),
        ]

        # Key status patterns
        self.status_patterns = {
            'SPIFFS_MOUNTED': r'SPIFFS mounted successfully',
            'SPIFFS_INFO': r'Partition size:.*total:.*KB.*used:.*KB',
            'FILESYSTEM_CALL': r'AP_Filesystem.*called',
            'BACKEND_CALL': r'backend.fs.open',
            'ESP32_FS_CALL': r'AP_Filesystem_ESP32::open called',
            'ESP32_FS_CONSTRUCTOR': r'AP_Filesystem_ESP32 constructor called',
            'ESP32_FS_ENTRY': r'!!! AP_Filesystem_ESP32::open ENTRY - WE MADE IT !!!',
            'LOG_OPEN_SUCCESS': r'Log open for /APM/\d+\.BIN',
            'LOG_OPEN_FAIL': r'Log open fail',
            'APM_DIRECTORY': r'/APM',
            'MAVLINK_LOG_TRANSFER': r'MAVLink.*log.*transfer',
            'HAL_ENABLED': r'AP_FILESYSTEM_ESP32_ENABLED=(\d+)',
            'PSRAM_SIZE': r'PSRAM detected: (\d+) MB',
            'FREE_RAM': r'Free RAM: (\d+)',
        }

        self.last_summary_time = time.time()
        self.summary_interval = 30  # seconds

    def connect(self):
        """Connect to serial port with lock file coordination"""
        try:
            # Acquire lock first
            self.port_lock = SerialPortLock(self.port, "esp32_monitor_agent")
            if not self.port_lock.acquire(timeout=30):
                print(f"✗ Could not acquire lock for {self.port} (timeout)")
                return False

            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"✓ Connected to {self.port} at {self.baudrate} baud (with lock)")
            return True
        except Exception as e:
            print(f"✗ Failed to connect to {self.port}: {e}")
            if self.port_lock:
                self.port_lock.release()
                self.port_lock = None
            return False

    def disconnect(self):
        """Disconnect and release port lock"""
        if self.serial:
            self.serial.close()
            self.serial = None
        if self.port_lock:
            self.port_lock.release()
            self.port_lock = None
            print(f"Released lock on {self.port}")

    def parse_line(self, line):
        """Parse a line for errors, warnings, and status"""
        timestamp = datetime.now().strftime("%H:%M:%S")

        # Check for errors
        for pattern, category in self.error_patterns:
            if re.search(pattern, line, re.IGNORECASE):
                self.errors[category].append((timestamp, line.strip()))
                self.error_counts[category] += 1
                return 'ERROR', category

        # Check for warnings
        for pattern, category in self.warning_patterns:
            if re.search(pattern, line, re.IGNORECASE):
                self.warnings[category].append((timestamp, line.strip()))
                self.warning_counts[category] += 1
                return 'WARNING', category

        # Check status patterns
        for name, pattern in self.status_patterns.items():
            match = re.search(pattern, line)
            if match:
                return 'STATUS', name

        return None, None

    def print_summary(self):
        """Print a summary of collected errors and warnings"""
        print("\n" + "="*60)
        print(f"ESP32 Error Monitor Summary - {datetime.now().strftime('%H:%M:%S')}")
        print("="*60)

        # Critical Issues
        critical_issues = []

        # Check for SPIFFS space issue
        if 'SPIFFS_SPACE' in self.errors:
            critical_issues.append("🔴 SPIFFS reports 'No space left on device' despite having free space")

        # Check if ESP32 filesystem backend is being called
        if 'BACKEND_CALL' in self.error_counts and 'ESP32_FS_CALL' not in self.error_counts:
            critical_issues.append("🔴 AP_Filesystem_ESP32::open() is NOT being called - backend not working!")

        # Check for crashes
        if 'CRASH' in self.errors or 'ABORT' in self.errors:
            critical_issues.append("🔴 System crash detected!")

        if critical_issues:
            print("\n🚨 CRITICAL ISSUES:")
            for issue in critical_issues:
                print(f"  {issue}")

        # Error Summary
        if self.error_counts:
            print("\n❌ ERRORS (last 30s):")
            for category, count in sorted(self.error_counts.items(), key=lambda x: x[1], reverse=True):
                print(f"  {category}: {count} occurrences")
                # Show last error
                if self.errors[category]:
                    last_time, last_msg = self.errors[category][-1]
                    print(f"    Last: [{last_time}] {last_msg[:80]}...")

        # Warning Summary
        if self.warning_counts:
            print("\n⚠️  WARNINGS (last 30s):")
            for category, count in sorted(self.warning_counts.items(), key=lambda x: x[1], reverse=True):
                if count > 10:  # Only show if significant
                    print(f"  {category}: {count} occurrences")

        # Status Information
        print("\n📊 STATUS:")

        # Check for key logging functionality status
        constructor_seen = False
        entry_seen = False
        log_open_success_seen = False
        log_open_fail_seen = False
        spiffs_mounted = False

        # Check recent lines for status info
        for line in list(self.recent_lines):
            for name, pattern in self.status_patterns.items():
                match = re.search(pattern, line)
                if match:
                    if name == 'ESP32_FS_CONSTRUCTOR':
                        constructor_seen = True
                    elif name == 'ESP32_FS_ENTRY':
                        entry_seen = True
                    elif name == 'LOG_OPEN_SUCCESS':
                        log_open_success_seen = True
                    elif name == 'LOG_OPEN_FAIL':
                        log_open_fail_seen = True
                    elif name == 'SPIFFS_MOUNTED':
                        spiffs_mounted = True
                    elif name == 'HAL_ENABLED':
                        enabled = match.group(1)
                        status = "✓ ENABLED" if enabled == "1" else "✗ DISABLED"
                        print(f"  AP_FILESYSTEM_ESP32: {status}")
                    elif name == 'PSRAM_SIZE':
                        print(f"  PSRAM: {match.group(1)} MB detected")
                    elif name == 'FREE_RAM':
                        ram_bytes = int(match.group(1))
                        print(f"  Free RAM: {ram_bytes/1024/1024:.1f} MB")
                    elif name == 'SPIFFS_INFO':
                        print(f"  SPIFFS: {line.strip()}")

        # Report ESP32 filesystem logging status
        print(f"  ESP32 FS Constructor Called: {'✓ YES' if constructor_seen else '✗ NO'}")
        print(f"  ESP32 FS open() Entry Detected: {'✓ YES' if entry_seen else '✗ NO'}")
        print(f"  SPIFFS Mounted: {'✓ YES' if spiffs_mounted else '✗ NO'}")
        print(f"  Log File Creation Success: {'✓ YES' if log_open_success_seen else '✗ NO'}")
        print(f"  Log File Creation Failures: {'⚠️ YES' if log_open_fail_seen else '✓ NO'}")

        # Recommendations
        print("\n💡 RECOMMENDATIONS:")
        if 'ESP32_FS_CALL' not in self.error_counts and 'BACKEND_CALL' in self.error_counts:
            print("  1. ESP32 filesystem backend is not being called - check virtual function linking")
            print("  2. Verify AP_Filesystem_ESP32 object is being instantiated")
            print("  3. Check if constructor message appears: 'AP_Filesystem_ESP32 constructor called'")

        if 'SPIFFS_SPACE' in self.errors:
            print("  1. SPIFFS file creation failing despite free space")
            print("  2. Check if HAL_ESP32_USE_SPIFFS is defined during compilation")
            print("  3. Verify SPIFFS mount point and file paths are correct")

        print("="*60)

    def monitor(self, duration=None):
        """Main monitoring loop"""
        if not self.connect():
            return

        print(f"Monitoring ESP32 output... (Press Ctrl+C to stop)")
        print(f"Summary every {self.summary_interval} seconds\n")

        start_time = time.time()

        try:
            while self.running:
                if duration and (time.time() - start_time) > duration:
                    break

                if self.serial.in_waiting:
                    try:
                        line = self.serial.readline().decode('utf-8', errors='replace')
                        self.recent_lines.append(line)

                        # Print the line
                        print(line, end='')

                        # Parse for issues
                        level, category = self.parse_line(line)

                        # Highlight critical errors immediately
                        if level == 'ERROR' and category in ['CRASH', 'ABORT', 'SPIFFS_SPACE']:
                            print(f"\n🚨 CRITICAL: {category} detected!\n")

                    except Exception as e:
                        print(f"Parse error: {e}")

                # Print summary periodically
                if time.time() - self.last_summary_time > self.summary_interval:
                    self.print_summary()
                    self.last_summary_time = time.time()

                    # Reset counts for next period
                    self.error_counts.clear()
                    self.warning_counts.clear()

        except KeyboardInterrupt:
            print("\n\nMonitoring stopped by user")
        finally:
            if self.graceful_exit:
                print("\n✓ Gracefully releasing port for upload agent")
                print("  Monitor will resume automatically after upload completes")
            else:
                self.print_summary()
            self.disconnect()  # Release lock and close serial

    def signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully"""
        self.running = False
        self.disconnect()  # Ensure port lock is released

    def release_handler(self, sig, frame):
        """Handle SIGUSR1 for graceful release request"""
        print("\n📤 Received release request from upload agent")
        print("   Gracefully disconnecting to allow firmware upload...")
        self.graceful_exit = True
        self.running = False

def main():
    parser = argparse.ArgumentParser(description='ESP32 Error Monitor Agent')
    parser.add_argument('--port', default='/dev/ttyACM0', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--duration', type=int, help='Monitor duration in seconds')
    parser.add_argument('--interval', type=int, default=30, help='Summary interval in seconds')

    args = parser.parse_args()

    monitor = ESP32ErrorMonitor(args.port, args.baud)
    monitor.summary_interval = args.interval

    # Set up signal handlers
    signal.signal(signal.SIGINT, monitor.signal_handler)
    signal.signal(signal.SIGTERM, monitor.signal_handler)
    signal.signal(signal.SIGUSR1, monitor.release_handler)  # For graceful release requests

    # Run monitor
    monitor.monitor(args.duration)

if __name__ == '__main__':
    main()