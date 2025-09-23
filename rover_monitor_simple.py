#!/usr/bin/env python3
"""
Simple ArduPilot Rover Monitor
Continuously monitors rover serial output and reports issues
"""

import serial
import time
import sys
from datetime import datetime
import signal

class RoverMonitor:
    def __init__(self, port='/dev/ttyACM1', baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = True
        self.line_count = 0
        self.mavlink_corruptions = 0
        self.errors = 0
        self.warnings = 0

    def signal_handler(self, sig, frame):
        print(f'\n\n=== MONITORING SUMMARY ===')
        print(f'Total lines processed: {self.line_count}')
        print(f'MAVLink corruptions: {self.mavlink_corruptions}')
        print(f'Errors detected: {self.errors}')
        print(f'Warnings detected: {self.warnings}')
        print('Monitor stopped.')
        self.running = False
        if self.ser:
            self.ser.close()
        sys.exit(0)

    def monitor(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            print(f'🤖 ArduPilot Rover Monitor Started')
            print(f'📡 Connected to {self.port} at {self.baud} baud')
            print(f'⏰ Started at {datetime.now().strftime("%H:%M:%S")}')
            print(f'📊 Monitoring for critical issues...\n')

            while self.running:
                if self.ser.in_waiting:
                    line = self.ser.readline()
                    if line:
                        try:
                            line_str = line.decode('utf-8', errors='replace').strip()
                            if line_str:  # Only process non-empty lines
                                self.line_count += 1
                                timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]

                                # Always print the line
                                print(f'[{timestamp}] {line_str}')

                                # Check for critical issues
                                line_upper = line_str.upper()

                                if 'MAVLINK' in line_upper and 'CORRUPTION' in line_upper:
                                    self.mavlink_corruptions += 1
                                    print(f'🚨 CRITICAL ALERT: MAVLink Corruption #{self.mavlink_corruptions}!')

                                if 'PANIC' in line_upper:
                                    print(f'🚨 CRITICAL ALERT: PANIC detected!')

                                if 'WATCHDOG' in line_upper:
                                    print(f'🚨 CRITICAL ALERT: Watchdog reset!')

                                if 'STACK OVERFLOW' in line_upper:
                                    print(f'🚨 CRITICAL ALERT: Stack overflow!')

                                if 'GURU MEDITATION' in line_upper:
                                    print(f'🚨 CRITICAL ALERT: Guru Meditation Error!')

                                if any(word in line_upper for word in ['ERROR', 'FAIL', 'FAILED']):
                                    self.errors += 1
                                    if self.errors <= 10:  # Only highlight first 10 to avoid spam
                                        print(f'❌ ERROR #{self.errors} detected')

                                if any(word in line_upper for word in ['WARN', 'WARNING']):
                                    self.warnings += 1
                                    if self.warnings <= 5:  # Only highlight first 5
                                        print(f'⚠️  WARNING #{self.warnings} detected')

                        except Exception as e:
                            print(f'Parse error: {e}')

                # Print periodic summary
                if self.line_count % 100 == 0 and self.line_count > 0:
                    print(f'\n📊 Status: {self.line_count} lines, {self.mavlink_corruptions} corruptions, {self.errors} errors, {self.warnings} warnings\n')

                time.sleep(0.001)  # Small delay

        except serial.SerialException as e:
            print(f'Serial error: {e}')
        except KeyboardInterrupt:
            self.signal_handler(signal.SIGINT, None)
        except Exception as e:
            print(f'Unexpected error: {e}')
        finally:
            if self.ser:
                self.ser.close()

if __name__ == '__main__':
    monitor = RoverMonitor()
    signal.signal(signal.SIGINT, monitor.signal_handler)
    signal.signal(signal.SIGTERM, monitor.signal_handler)
    signal.signal(signal.SIGUSR1, monitor.signal_handler)
    monitor.monitor()