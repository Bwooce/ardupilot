#!/usr/bin/env python3
"""
Check ESP32 status and attempt to communicate
"""

import serial
import time
from datetime import datetime

def check_esp32_connection(port='/dev/ttyACM0'):
    print(f"Checking ESP32 connection on {port}")
    print("=" * 50)

    baud_rates = [115200, 921600, 74880]  # Including ESP32 boot baud rate

    for baud in baud_rates:
        print(f"\nTrying baud rate: {baud}")
        try:
            with serial.Serial(port, baud, timeout=2) as ser:
                print(f"  Port opened at {baud} baud")

                # Check for data
                ser.reset_input_buffer()

                # Try sending some commands that might trigger a response
                commands = [b'\r\n', b'help\r\n', b'version\r\n', b'status\r\n']

                for cmd in commands:
                    print(f"  Sending: {cmd}")
                    ser.write(cmd)
                    ser.flush()
                    time.sleep(0.5)

                    # Check for response
                    time.sleep(1)
                    response_data = []

                    while ser.in_waiting > 0:
                        try:
                            data = ser.read(ser.in_waiting)
                            response_data.append(data)
                        except:
                            break

                    if response_data:
                        for data in response_data:
                            try:
                                decoded = data.decode('utf-8', errors='ignore')
                                if decoded.strip():
                                    print(f"  RESPONSE: {repr(decoded)}")
                                    return True, baud, decoded
                            except:
                                print(f"  RAW RESPONSE: {data.hex()}")
                                return True, baud, f"Raw data: {data.hex()}"

                    print(f"  No response to {cmd}")

                print(f"  No response at {baud} baud")

        except serial.SerialException as e:
            print(f"  Serial error at {baud} baud: {e}")
        except Exception as e:
            print(f"  Error at {baud} baud: {e}")

    return False, None, None

def main():
    connected, baud, data = check_esp32_connection()

    print("\n" + "=" * 50)
    print("ESP32 CONNECTION TEST RESULTS")
    print("=" * 50)

    if connected:
        print(f"✅ ESP32 responding at {baud} baud")
        print(f"Data received: {data}")
        print("\nThe ESP32 is connected and responding.")
        print("If no ArduPilot messages are seen, the firmware may need to be uploaded.")
    else:
        print("❌ No response from ESP32")
        print("\nPossible issues:")
        print("- ESP32 not connected or powered")
        print("- ESP32 in bootloader mode (try pressing reset)")
        print("- Hardware failure")
        print("- Wrong serial port")
        print("\nTroubleshooting:")
        print("1. Check USB cable connection")
        print("2. Press RESET button on ESP32")
        print("3. Check if device appears in lsusb output")
        print("4. Try different USB port")

if __name__ == "__main__":
    main()