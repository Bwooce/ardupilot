#!/usr/bin/env python3
"""
Simple serial connection test for ESP32
"""

import serial
import time
import sys

def test_serial(port='/dev/ttyACM0', timeout=10):
    baud_rates = [115200, 921600, 460800, 230400, 57600, 9600]

    for baud in baud_rates:
        print(f"Testing {port} at {baud} baud...")
        try:
            with serial.Serial(port, baud, timeout=1) as ser:
                # Send a newline to trigger any prompt
                ser.write(b'\r\n')
                ser.flush()

                # Read for a few seconds
                start_time = time.time()
                data_received = False

                while time.time() - start_time < 3:
                    if ser.in_waiting > 0:
                        data = ser.readline()
                        if data:
                            try:
                                decoded = data.decode('utf-8', errors='ignore').strip()
                                if decoded:
                                    print(f"  SUCCESS at {baud} baud: {decoded}")
                                    data_received = True
                            except:
                                print(f"  Data received at {baud} baud (non-UTF8)")
                                data_received = True

                if not data_received:
                    print(f"  No data at {baud} baud")

        except serial.SerialException as e:
            print(f"  Error at {baud} baud: {e}")
        except Exception as e:
            print(f"  Unexpected error at {baud} baud: {e}")

    print("\nTesting complete. If no data was found, the device may be:")
    print("- Not connected or powered")
    print("- Not running firmware")
    print("- Using a different communication protocol")
    print("- In a boot loop or crashed state")

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    test_serial(port)