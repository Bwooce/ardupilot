#!/usr/bin/env python3
import serial
import time
from pymavlink import mavutil
import struct
import binascii

# Configuration
SERIAL_PORT = "/dev/cu.usbserial-0001"  # Replace with your USB port
BAUD_RATE = 460800  # Common baud rate for ELRS MAVLink (adjust if needed)
#BAUD_RATE = 420000
LOG_FILE = "serial_data.log"

def decode_mavlink(connection):
    """Attempt to decode MAVLink messages from the connection."""
    try:
        msg = connection.recv_match(blocking=False)
        if msg:
            print(f"MAVLink Message: {msg.get_type()} - {msg.to_dict()}")
            with open(LOG_FILE, "a") as f:
                f.write(f"MAVLink: {msg.get_type()} - {msg.to_dict()}\n")
            return True
        return False
    except Exception as e:
        print(f"MAVLink decode error: {e}")
        return False

def decode_crsf(data):
    """Attempt to decode CRSF (Crossfire) packets."""
    try:
        # CRSF packets start with 0xC8 (address) and have a length byte
        if len(data) < 3 or data[0] != 0xC8:
            return False
        length = data[1]
        if len(data) < length + 2:
            return False
        packet_type = data[2]
        # Example: Handle specific CRSF packet types (e.g., 0x14 for RC channels)
        if packet_type == 0x14:
            channels = struct.unpack("<HHHHHHHH", data[3:19])  # 8 channels, 16-bit each
            print(f"CRSF RC Channels: {channels}")
            with open(LOG_FILE, "a") as f:
                f.write(f"CRSF RC Channels: {channels}\n")
        else:
            print(f"CRSF Packet Type: 0x{packet_type:02X}, Data: {data.hex()}")
            with open(LOG_FILE, "a") as f:
                f.write(f"CRSF Packet Type: 0x{packet_type:02X}, Data: {data.hex()}\n")
        return True
    except Exception as e:
        print(f"CRSF decode error: {e}")
        return False

def main():
    # Initialize serial connection
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud")
    except serial.SerialException as e:
        print(f"Failed to open serial port: {e}")
        return

    # Initialize MAVLink connection
    try:
        mav = mavutil.mavlink_connection(f'udpin:localhost:14550')
        # Alternatively, use serial directly: mav = mavutil.mavlink_connection(ser, baud=BAUD_RATE)
        print("MAVLink connection initialized")
    except Exception as e:
        print(f"MAVLink initialization error: {e}")
        ser.close()
        return

    # Buffer for raw data
    buffer = bytearray()

    print("Receiving data... Press Ctrl+C to stop.")
    with open(LOG_FILE, "w") as f:
        f.write("Starting log...\n")

    try:
        while True:
            # Read raw data from serial port
            data = ser.read(100)  # Read up to 100 bytes
            if data:
                buffer.extend(data)
                print(f"Raw data: {data.hex()}")
                with open(LOG_FILE, "a") as f:
                    f.write(f"Raw: {data.hex()}\n")

                # Attempt MAVLink decoding
                decode_mavlink(mav)

                # Attempt CRSF decoding
                while len(buffer) > 0:
                    if decode_crsf(buffer):
                        # Remove processed CRSF packet
                        length = buffer[1] + 2 if buffer[0] == 0xC8 else 1
                        buffer = buffer[length:]
                    else:
                        # Remove first byte if not a valid CRSF packet
                        buffer = buffer[1:]

            time.sleep(0.01)  # Prevent CPU overload

    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        ser.close()
        print("Serial port closed")

if __name__ == "__main__":
    main()

