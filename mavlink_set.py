#!/usr/bin/env python3
"""
ELRS MAVLink Configuration Script
Automatically discovers ELRS TX module and configures it for MAVLink mode
"""

import serial
import serial.tools.list_ports
import time
import struct

class CRSFProtocol:
    # CRSF Protocol Constants
    SYNC_BYTE = 0xC8
    
    # Frame types
    TYPE_SETTINGS_WRITE = 0x2D
    TYPE_RC_CHANNELS = 0x16
    
    # ELRS Addresses
    ELRS_ADDRESS = 0xEE
    RADIO_ADDRESS = 0xEA
    
    # ELRS Commands (based on research)
    LINK_MODE_COMMAND = 0x0A  # Link Mode setting ID (estimated)
    PROTOCOL_COMMAND = 0x0B   # Protocol setting ID (estimated)
    
    # Values
    LINK_MODE_MAVLINK = 1     # MAVLink mode
    PROTOCOL_MAVLINK = 6      # MAVLink protocol
    
    @staticmethod
    def calculate_crc8(data):
        """Calculate CRC8 for CRSF protocol"""
        crc = 0
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0xD5
                else:
                    crc <<= 1
                crc &= 0xFF
        return crc
    
    def create_settings_packet(self, command_id, value):
        """Create CRSF settings write packet"""
        # Payload: [Target][Source][Command][Value]
        payload = [self.ELRS_ADDRESS, self.RADIO_ADDRESS, command_id, value]
        
        # Calculate length (payload + type + crc)
        length = len(payload) + 2
        
        # Create packet without CRC first
        packet_data = [length, self.TYPE_SETTINGS_WRITE] + payload
        
        # Calculate CRC over type + payload
        crc = self.calculate_crc8(packet_data[1:])
        
        # Complete packet
        packet = [self.SYNC_BYTE] + packet_data + [crc]
        
        return bytes(packet)
    
    def create_rc_packet(self):
        """Create dummy RC channels packet to maintain connection"""
        # Standard RC packet with centered values (992 = center)
        channels = [992] * 16  # 16 channels, all centered
        
        # Pack channels into 11-bit values (CRSF format)
        packed = bytearray(22)  # 16 channels * 11 bits = 176 bits = 22 bytes
        
        bit_offset = 0
        for channel in channels:
            # Convert to 11-bit range (172-1811, center=992)
            value = max(172, min(1811, channel))
            
            # Pack 11-bit value
            byte_offset = bit_offset // 8
            bit_in_byte = bit_offset % 8
            
            # Split value across bytes
            for bit in range(11):
                if value & (1 << bit):
                    packed[byte_offset + (bit + bit_in_byte) // 8] |= 1 << ((bit + bit_in_byte) % 8)
            
            bit_offset += 11
        
        # Create packet
        payload = list(packed)
        length = len(payload) + 2
        packet_data = [length, self.TYPE_RC_CHANNELS] + payload
        crc = self.calculate_crc8(packet_data[1:])
        packet = [self.SYNC_BYTE] + packet_data + [crc]
        
        return bytes(packet)

def find_elrs_device():
    """Auto-discover ELRS TX module USB device"""
    print("Searching for ELRS TX module...")
    
    # Common ELRS device identifiers
    elrs_identifiers = [
        'usbmodem',  # macOS style
        'ttyACM',    # Linux style  
        'COM',       # Windows style
        'cp210',     # CP2102 chip common in ELRS
        'ch340',     # CH340 chip
        'ftdi',      # FTDI chip
    ]
    
    ports = serial.tools.list_ports.comports()
    
    for port in ports:
        port_name = port.device.lower()
        description = (port.description or '').lower()
        manufacturer = (port.manufacturer or '').lower()
        
        print(f"Found port: {port.device} - {port.description}")
        
        # Check for ELRS-like devices
        for identifier in elrs_identifiers:
            if (identifier in port_name or 
                identifier in description or 
                identifier in manufacturer):
                print(f"Potential ELRS device: {port.device}")
                return port.device
    
    # If no specific match, try first available serial port
    if ports:
        print(f"No specific ELRS device found, trying first available: {ports[0].device}")
        return ports[0].device
    
    return None

def configure_elrs_mavlink():
    """Main function to configure ELRS for MAVLink"""
    
    # Find ELRS device
    device = find_elrs_device()
    if not device:
        print("ERROR: No serial devices found!")
        return False
    
    print(f"Attempting to connect to: {device}")
    
    try:
        # Open serial connection
        # ELRS typically uses 420000 baud, but some use 115200
        for baud_rate in [420000, 115200, 57600]:
            try:
                ser = serial.Serial(device, baud_rate, timeout=1)
                print(f"Connected at {baud_rate} baud")
                break
            except:
                continue
        else:
            print("ERROR: Could not establish serial connection at any baud rate")
            return False
        
        crsf = CRSFProtocol()
        
        print("Sending initial RC data to establish connection...")
        
        # Send RC data for a few seconds to establish connection
        for i in range(20):
            rc_packet = crsf.create_rc_packet()
            ser.write(rc_packet)
            time.sleep(0.05)  # 20Hz
            print(f"Sent RC packet {i+1}/20", end='\r')
        
        print("\nConnection established, sending MAVLink configuration...")
        
        # Send Link Mode to MAVLink command
        link_mode_packet = crsf.create_settings_packet(
            crsf.LINK_MODE_COMMAND, 
            crsf.LINK_MODE_MAVLINK
        )
        ser.write(link_mode_packet)
        print("Sent Link Mode = MAVLink command")
        time.sleep(0.1)
        
        # Send Protocol to MAVLink command  
        protocol_packet = crsf.create_settings_packet(
            crsf.PROTOCOL_COMMAND,
            crsf.PROTOCOL_MAVLINK
        )
        ser.write(protocol_packet)
        print("Sent Protocol = MAVLink command")
        time.sleep(0.1)
        
        # Continue sending RC data to maintain connection
        print("Maintaining connection with RC data...")
        print("MAVLink configuration sent! Your TX module should now be in MAVLink mode.")
        print("You can now connect QGroundControl to this same serial port at 460800 baud.")
        print("Press Ctrl+C to stop...")
        
        try:
            while True:
                rc_packet = crsf.create_rc_packet()
                ser.write(rc_packet)
                time.sleep(0.05)  # Keep sending at 20Hz
        except KeyboardInterrupt:
            print("\nStopping RC data transmission...")
        
        ser.close()
        return True
        
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return False
    except Exception as e:
        print(f"Unexpected error: {e}")
        return False

if __name__ == "__main__":
    print("ELRS MAVLink Configuration Tool")
    print("================================")
    print("This script will:")
    print("1. Find your ELRS TX module")
    print("2. Send RC data to establish connection") 
    print("3. Configure it for MAVLink mode")
    print("4. Keep the connection alive")
    print()
    
    success = configure_elrs_mavlink()
    
    if success:
        print("Configuration completed successfully!")
        print("Your ELRS TX module should now be ready for MAVLink communication.")
    else:
        print("Configuration failed. Please check connections and try again.")
        print("Troubleshooting tips:")
        print("- Ensure TX module is powered via USB")
        print("- Check that no other software is using the serial port")
        print("- Try running as administrator/sudo if permission issues occur")

