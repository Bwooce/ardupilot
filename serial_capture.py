#!/usr/bin/env python3
import serial
import time
import sys

def capture_serial():
    try:
        # Open serial port
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        print("Starting 60-second serial capture...")

        start_time = time.time()
        end_time = start_time + 60

        while time.time() < end_time:
            try:
                if ser.in_waiting > 0:
                    data = ser.readline().decode('utf-8', errors='ignore').strip()
                    if data:
                        print(data)
                time.sleep(0.01)  # Small delay to prevent excessive CPU usage
            except Exception as e:
                print(f"Read error: {e}")
                break

        ser.close()
        print("Serial capture completed.")

    except Exception as e:
        print(f"Failed to open serial port: {e}")
        sys.exit(1)

if __name__ == "__main__":
    capture_serial()