#!/usr/bin/env python3
"""
ArduPilot Rover Monitor Agent
Monitors ArduPilot rover serial output for critical issues and diagnostics
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

class ArduPilotRoverMonitor:
    def __init__(self, port='/dev/ttyACM1', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.port_lock = None
        self.running = True

        # Error tracking
        self.critical_errors = defaultdict(list)
        self.errors = defaultdict(list)
        self.warnings = defaultdict(list)
        self.recent_lines = deque(maxlen=200)  # Keep more lines for ArduPilot context
        self.error_counts = defaultdict(int)
        self.warning_counts = defaultdict(int)
        self.critical_counts = defaultdict(int)

        # ArduPilot-specific critical error patterns
        self.critical_patterns = [
            (r'PANIC:', 'PANIC'),
            (r'Internal Error', 'INTERNAL_ERROR'),
            (r'Watchdog', 'WATCHDOG_RESET'),
            (r'Stack overflow', 'STACK_OVERFLOW'),
            (r'Hard fault', 'HARD_FAULT'),
            (r'Memory fault', 'MEMORY_FAULT'),
            (r'Usage fault', 'USAGE_FAULT'),
            (r'Bus fault', 'BUS_FAULT'),
            (r'assert failed', 'ASSERTION_FAILED'),
            (r'EMERGENCY.*STOP', 'EMERGENCY_STOP'),
            (r'Brownout', 'BROWNOUT'),
            (r'Reset.*unexpected', 'UNEXPECTED_RESET'),
            (r'EKF.*primary.*fail', 'EKF_PRIMARY_FAIL'),
            (r'MAVLINK.*CORRUPTION', 'MAVLINK_CORRUPTION'),
            (r'Guru Meditation Error', 'GURU_MEDITATION'),
        ]

        # ArduPilot error patterns
        self.error_patterns = [
            (r'ERROR|FAIL|Failed', 'GENERAL_ERROR'),
            (r'GPS.*lost|GPS.*timeout', 'GPS_LOST'),
            (r'RC.*failsafe|RC.*lost', 'RC_FAILSAFE'),
            (r'Battery.*low|Battery.*critical', 'BATTERY_LOW'),
            (r'Compass.*error|Compass.*fail', 'COMPASS_ERROR'),
            (r'IMU.*error|IMU.*fail', 'IMU_ERROR'),
            (r'CAN.*error|CAN.*timeout', 'CAN_ERROR'),
            (r'UART.*error|UART.*overflow', 'UART_ERROR'),
            (r'Mission.*error|Mission.*fail', 'MISSION_ERROR'),
            (r'Steering.*error|Throttle.*error', 'ACTUATOR_ERROR'),
            (r'Sensor.*error|Sensor.*fail', 'SENSOR_ERROR'),
            (r'Parameter.*error|Param.*fail', 'PARAMETER_ERROR'),
            (r'EKF.*error|EKF.*warn', 'EKF_ERROR'),
            (r'Navigation.*error|Nav.*fail', 'NAVIGATION_ERROR'),
            (r'eFuse.*empty', 'EFUSE_ERROR'),
            (r'system_api.*error', 'SYSTEM_API_ERROR'),
        ]

        # ArduPilot warning patterns
        self.warning_patterns = [
            (r'WARN|Warning', 'GENERAL_WARNING'),
            (r'GPS.*glitch|GPS.*jump', 'GPS_GLITCH'),
            (r'Velocity.*error|Position.*error', 'VELOCITY_ERROR'),
            (r'Mode.*change.*fail', 'MODE_CHANGE_FAIL'),
            (r'Fence.*breach', 'FENCE_BREACH'),
            (r'Yaw.*error|Heading.*error', 'YAW_ERROR'),
            (r'Speed.*limit|Throttle.*limit', 'SPEED_LIMIT'),
            (r'Obstacle.*detect', 'OBSTACLE_DETECTED'),
        ]

        # Status patterns to track rover state
        self.status_patterns = {
            'MODE': r'Mode\s+(\w+)',
            'ARM_STATE': r'(ARMED|DISARMED)',
            'GPS_FIX': r'GPS.*fix.*(\d+)',
            'BATTERY_VOLTAGE': r'Battery.*(\d+\.\d+)V',
            'MISSION_STATE': r'Mission.*(\w+)',
            'FENCE_STATE': r'Fence.*(\w+)',
            'EKF_STATE': r'EKF.*(\w+)',
            'RC_CHANNELS': r'RC.*Ch(\d+):(\d+)',
        }

        self.last_summary_time = time.time()
        self.summary_interval = 15  # seconds

        # Track rover state
        self.current_mode = "UNKNOWN"
        self.armed_state = "UNKNOWN"
        self.gps_fix = "UNKNOWN"
        self.battery_voltage = "UNKNOWN"

    def connect(self):
        """Connect to serial port with lock file coordination"""
        try:
            # Acquire lock first
            self.port_lock = SerialPortLock(self.port, "ardupilot_rover_monitor")
            if not self.port_lock.acquire(timeout=30):
                print(f"✗ Could not acquire lock for {self.port} (timeout)")
                return False

            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"✓ Connected to {self.port} at {self.baudrate} baud (with lock)")
            print(f"🤖 ArduPilot Rover Monitor - Watching for critical issues...")
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
        """Parse a line for ArduPilot-specific errors, warnings, and status"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]  # Include milliseconds

        # Check for critical errors first
        for pattern, category in self.critical_patterns:
            if re.search(pattern, line, re.IGNORECASE):
                self.critical_errors[category].append((timestamp, line.strip()))
                self.critical_counts[category] += 1
                return 'CRITICAL', category

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

        # Check status patterns and update state
        for name, pattern in self.status_patterns.items():
            match = re.search(pattern, line, re.IGNORECASE)
            if match:
                if name == 'MODE':
                    self.current_mode = match.group(1)
                elif name == 'ARM_STATE':
                    self.armed_state = match.group(1)
                elif name == 'GPS_FIX':
                    self.gps_fix = match.group(1)
                elif name == 'BATTERY_VOLTAGE':
                    self.battery_voltage = match.group(1) + "V"
                return 'STATUS', name

        return None, None

    def print_rover_status(self):
        """Print current rover status"""
        print(f"🚁 ROVER STATUS:")
        print(f"  Mode: {self.current_mode}")
        print(f"  Armed: {self.armed_state}")
        print(f"  GPS Fix: {self.gps_fix}")
        print(f"  Battery: {self.battery_voltage}")

    def print_summary(self):
        """Print a summary of collected errors and warnings"""
        print("\n" + "="*70)
        print(f"ArduPilot Rover Monitor Summary - {datetime.now().strftime('%H:%M:%S')}")
        print("="*70)

        # Print rover status
        self.print_rover_status()

        # Critical Issues (always show these)
        if self.critical_counts:
            print("\n🚨 CRITICAL ISSUES:")
            for category, count in sorted(self.critical_counts.items(), key=lambda x: x[1], reverse=True):
                print(f"  {category}: {count} occurrences")
                if self.critical_errors[category]:
                    last_time, last_msg = self.critical_errors[category][-1]
                    print(f"    Last: [{last_time}] {last_msg}")

        # Error Summary
        if self.error_counts:
            print("\n❌ ERRORS (last period):")
            for category, count in sorted(self.error_counts.items(), key=lambda x: x[1], reverse=True):
                print(f"  {category}: {count} occurrences")
                if self.errors[category]:
                    last_time, last_msg = self.errors[category][-1]
                    print(f"    Last: [{last_time}] {last_msg[:100]}...")

        # Warning Summary (only if significant)
        if self.warning_counts:
            print("\n⚠️  WARNINGS (last period):")
            for category, count in sorted(self.warning_counts.items(), key=lambda x: x[1], reverse=True):
                if count > 5:  # Only show if frequent
                    print(f"  {category}: {count} occurrences")

        # Recommendations based on issues found
        print("\n💡 RECOMMENDATIONS:")
        if any(self.critical_counts.values()):
            print("  🔴 CRITICAL: System stability compromised - investigate immediately!")
            if 'WATCHDOG_RESET' in self.critical_counts:
                print("     - Watchdog resets indicate main loop blocking or infinite loops")
            if 'STACK_OVERFLOW' in self.critical_counts:
                print("     - Stack overflow - reduce recursion or increase stack size")
            if 'EKF_PRIMARY_FAIL' in self.critical_counts:
                print("     - EKF failure affects navigation - check sensors and calibration")

        if 'GPS_LOST' in self.error_counts:
            print("  📡 GPS issues detected - check antenna and satellite visibility")
        if 'RC_FAILSAFE' in self.error_counts:
            print("  📻 RC failsafe - check transmitter and receiver")
        if 'BATTERY_LOW' in self.error_counts:
            print("  🔋 Battery issues - monitor power consumption and charging")

        if not any(self.critical_counts.values()) and not any(self.error_counts.values()):
            print("  ✅ No significant issues detected - rover operating normally")

        print("="*70)

    def monitor(self, duration=None):
        """Main monitoring loop"""
        if not self.connect():
            return

        print(f"Monitoring ArduPilot rover output... (Press Ctrl+C to stop)")
        print(f"Summary every {self.summary_interval} seconds\n")

        start_time = time.time()
        line_count = 0

        try:
            while self.running:
                if duration and (time.time() - start_time) > duration:
                    break

                if self.serial.in_waiting:
                    try:
                        line = self.serial.readline().decode('utf-8', errors='replace')
                        if line.strip():  # Only process non-empty lines
                            self.recent_lines.append(line)
                            line_count += 1

                            # Print the line with timestamp
                            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                            print(f"[{timestamp}] {line}", end='')

                            # Parse for issues
                            level, category = self.parse_line(line)

                            # Immediately highlight critical errors
                            if level == 'CRITICAL':
                                print(f"\n🚨 CRITICAL ALERT: {category} detected!")
                                print(f"    {line.strip()}\n")

                            # Highlight important errors
                            elif level == 'ERROR' and category in ['GPS_LOST', 'RC_FAILSAFE', 'BATTERY_LOW', 'EKF_ERROR']:
                                print(f"\n⚠️  IMPORTANT: {category} - {line.strip()}\n")

                    except Exception as e:
                        print(f"Parse error: {e}")

                # Print summary periodically
                if time.time() - self.last_summary_time > self.summary_interval:
                    print(f"\n📊 Processed {line_count} lines in last {self.summary_interval}s")
                    self.print_summary()
                    self.last_summary_time = time.time()
                    line_count = 0

                    # Reset counts for next period
                    self.error_counts.clear()
                    self.warning_counts.clear()
                    # Keep critical counts accumulating

                time.sleep(0.01)  # Small delay to prevent 100% CPU usage

        except KeyboardInterrupt:
            print("\n\nMonitoring stopped by user")
        finally:
            self.print_summary()
            self.disconnect()

    def signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully"""
        self.running = False

def main():
    parser = argparse.ArgumentParser(description='ArduPilot Rover Monitor Agent')
    parser.add_argument('--port', default='/dev/ttyACM1', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--duration', type=int, help='Monitor duration in seconds')
    parser.add_argument('--interval', type=int, default=15, help='Summary interval in seconds')

    args = parser.parse_args()

    monitor = ArduPilotRoverMonitor(args.port, args.baud)
    monitor.summary_interval = args.interval

    # Set up signal handlers
    signal.signal(signal.SIGINT, monitor.signal_handler)

    # Run monitor
    monitor.monitor(args.duration)

if __name__ == '__main__':
    main()