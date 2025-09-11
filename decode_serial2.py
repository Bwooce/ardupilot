import serial
import time
from pymavlink import mavutil

# Configuration
SERIAL_PORT = "/dev/cu.usbserial-0001"  # Your port
BAUD_RATES = [460800, 115200, 57600]  # Common baud rates to try
LOG_FILE = "serial_data.log"

def decode_mavlink(connection):
    """Attempt to decode MAVLink messages."""
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

def check_pattern(data):
    """Check if data is a repeating 0xAA pattern."""
    return all(byte == 0xAA for byte in data)

def try_baud_rate(port, baud):
    """Attempt to connect and read data at the given baud rate."""
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        print(f"Connected to {port} at {baud} baud")
        return ser
    except serial.SerialException as e:
        print(f"Failed to connect at {baud} baud: {e}")
        return None

def main():
    # Initialize logging
    with open(LOG_FILE, "w") as f:
        f.write("Starting log...\n")

    # Try different baud rates
    ser = None
    for baud in BAUD_RATES:
        ser = try_baud_rate(SERIAL_PORT, baud)
        if ser:
            break
    if not ser:
        print("Failed to connect at any baud rate. Check port and connections.")
        return

    # Initialize MAVLink connection
    try:
        # Create a proper MAVLink connection string for serial
        connection_string = f"serial:{SERIAL_PORT}:{ser.baudrate}"
        mav = mavutil.mavlink_connection(connection_string)
        print("MAVLink connection initialized")
    except Exception as e:
        print(f"MAVLink initialization error: {e}")
        mav = None

    print("Receiving data... Press Ctrl+C to stop.")
    try:
        while True:
            # Read raw data
            data = ser.read(100)
            if data:
                # Check for 0xAA pattern
                if check_pattern(data):
                    print(f"Warning: Received repeating 0xAA pattern. Check baud rate or ELRS/flight controller settings.")
                else:
                    print(f"Raw data: {data.hex()}")
                    with open(LOG_FILE, "a") as f:
                        f.write(f"Raw: {data.hex()}\n")
                    # Attempt MAVLink decoding
                    if mav:
                        decode_mavlink(mav)
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        ser.close()
        print("Serial port closed")

if __name__ == "__main__":
    main()

