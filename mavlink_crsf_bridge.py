#!/usr/bin/env python3
"""
MAVLink to CRSF Bridge for ELRS TX Module
Allows QGroundControl to communicate through ELRS TX in MAVLink mode

Architecture:
QGC <--MAVLink--> Bridge <--CRSF--> TX <--ELRS/MAVLink--> RX <--MAVLink--> FC
"""

import serial
import socket
import time
import struct
import threading
import sys
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2

class MAVLinkCRSFBridge:
    """Bridge between MAVLink (from QGC) and CRSF (to ELRS TX)"""
    
    def __init__(self, tx_port, tx_baud=420000, mavlink_port=14550):
        """Initialize bridge
        
        Args:
            tx_port: Serial port for ELRS TX module
            tx_baud: Baud rate for CRSF (typically 420000 for ELRS)
            mavlink_port: UDP port for MAVLink from QGC
        """
        self.tx_port = tx_port
        self.tx_baud = tx_baud
        self.mavlink_port = mavlink_port
        self.running = False
        
        # Serial connection to TX module
        self.tx_serial = None
        
        # MAVLink connection
        self.mav = None
        
        # RC channels (default centered)
        self.rc_channels = [1500] * 16
        self.last_rc_time = 0
        
        # Statistics
        self.stats = {
            'mav_rx': 0,
            'mav_tx': 0,
            'crsf_tx': 0,
            'crsf_rx': 0
        }
        
    def crc8_dvb_s2(self, data):
        """Calculate CRC8 for CRSF"""
        crc = 0
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0xD5
                else:
                    crc = crc << 1
                crc &= 0xFF
        return crc
    
    def build_crsf_rc_packet(self):
        """Build CRSF RC channels packet"""
        # Convert PWM (1000-2000) to CRSF (0-1984)
        crsf_channels = []
        for pwm in self.rc_channels:
            # CRSF: 0=988us, 992=1500us, 1984=2012us
            val = int((pwm - 988) * 1.96)
            val = max(0, min(1984, val))
            crsf_channels.append(val)
        
        # Pack 16 channels (11 bits each) into 22 bytes
        packed = 0
        for i, ch in enumerate(crsf_channels):
            packed |= (ch & 0x7FF) << (i * 11)
        
        payload = packed.to_bytes(22, 'little')
        
        # Build CRSF frame
        frame = bytearray()
        frame.append(0xC8)  # CRSF_SYNC_BYTE
        frame.append(24)    # Length
        frame.append(0x16)  # CRSF_FRAMETYPE_RC_CHANNELS_PACKED
        frame.extend(payload)
        
        crc_data = bytes([0x16]) + payload
        frame.append(self.crc8_dvb_s2(crc_data))
        
        return bytes(frame)
    
    def parse_crsf_telemetry(self, data):
        """Parse CRSF telemetry frames and extract MAVLink"""
        # CRSF telemetry format: [sync][len][type][payload][crc]
        # ELRS embeds MAVLink in various ways
        
        extracted = bytearray()
        i = 0
        
        while i < len(data):
            # Look for MAVLink start bytes directly (ELRS might pass through)
            if i < len(data) - 8:
                if data[i] == 0xFE:  # MAVLink v1
                    # Try to extract MAVLink v1 packet
                    payload_len = data[i + 1] if i + 1 < len(data) else 0
                    packet_len = payload_len + 8  # header + payload + checksum
                    if i + packet_len <= len(data):
                        extracted.extend(data[i:i + packet_len])
                        i += packet_len
                        continue
                elif data[i] == 0xFD:  # MAVLink v2
                    # Try to extract MAVLink v2 packet
                    payload_len = data[i + 1] if i + 1 < len(data) else 0
                    incompat_flags = data[i + 2] if i + 2 < len(data) else 0
                    packet_len = payload_len + 12  # header + payload + checksum
                    if incompat_flags & 0x01:  # Signed
                        packet_len += 13
                    if i + packet_len <= len(data):
                        extracted.extend(data[i:i + packet_len])
                        i += packet_len
                        continue
            
            # Look for CRSF frame with embedded MAVLink
            if data[i] in [0xC8, 0xEC, 0xEE, 0xEA]:
                if i + 2 < len(data):
                    frame_len = data[i + 1]
                    if i + frame_len + 2 <= len(data):
                        frame_type = data[i + 2]
                        
                        # Various CRSF telemetry frame types that might contain MAVLink
                        if frame_type in [0x7F, 0x7E, 0x32, 0x7B, 0x7C]:
                            # Extract payload (skip sync, len, type, and crc)
                            payload = data[i + 3:i + 1 + frame_len]
                            
                            # Check if payload contains MAVLink
                            if len(payload) > 0:
                                if payload[0] in [0xFE, 0xFD]:  # MAVLink start
                                    extracted.extend(payload)
                                # Also check for CRSF extended frames with MAVLink
                                elif frame_type == 0x32 and len(payload) > 2:
                                    # Extended frame might have address bytes first
                                    for j in range(len(payload) - 1):
                                        if payload[j] in [0xFE, 0xFD]:
                                            extracted.extend(payload[j:])
                                            break
                        
                        i += frame_len + 2
                        continue
            i += 1
        
        return extracted if len(extracted) > 0 else None
    
    def connect_tx(self):
        """Connect to ELRS TX module"""
        try:
            if self.tx_serial:
                try:
                    self.tx_serial.close()
                except:
                    pass
                self.tx_serial = None
                
            self.tx_serial = serial.Serial(
                port=self.tx_port,
                baudrate=self.tx_baud,
                timeout=0.01,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            print(f"✓ Connected to TX module at {self.tx_port} ({self.tx_baud} baud)")
            return True
        except Exception as e:
            if self.running:  # Only print if we're still running
                print(f"✗ Failed to connect to TX: {e}")
            return False
    
    def setup_mavlink(self):
        """Setup MAVLink UDP connection for QGC"""
        try:
            # Create MAVLink UDP connection - let pymavlink handle the socket
            self.mav = mavutil.mavlink_connection(
                f'udpin:0.0.0.0:{self.mavlink_port}',
                source_system=255,
                source_component=0
            )
            
            print(f"✓ MAVLink UDP listening on port {self.mavlink_port}")
            print(f"  Configure QGC to connect to: udp://localhost:{self.mavlink_port}")
            return True
            
        except Exception as e:
            print(f"✗ Failed to setup MAVLink: {e}")
            return False
    
    def mavlink_thread(self):
        """Thread to handle MAVLink from QGC"""
        qgc_address = None
        
        while self.running:
            try:
                # Receive MAVLink from QGC
                msg = self.mav.recv_match(blocking=False)
                if msg:
                    self.stats['mav_rx'] += 1
                    
                    # Extract RC override commands if present
                    if msg.get_type() == 'RC_CHANNELS_OVERRIDE':
                        # Update RC channels from MAVLink
                        for i in range(8):
                            chan_val = getattr(msg, f'chan{i+1}_raw', 0)
                            if chan_val > 0:  # 0 means no change
                                self.rc_channels[i] = chan_val
                        self.last_rc_time = time.time()
                    
                    # Forward other MAVLink messages to TX as telemetry
                    # This would need CRSF encapsulation for telemetry uplink
                    # For now, focusing on RC control
                    
            except socket.timeout:
                pass
            except Exception as e:
                if self.running:
                    print(f"MAVLink error: {e}")
            
            time.sleep(0.001)
    
    def crsf_thread(self):
        """Thread to handle CRSF communication with TX"""
        last_rc_send = 0
        last_error_time = 0
        reconnect_delay = 1.0
        
        while self.running:
            try:
                # Check if serial is connected
                if not self.tx_serial or not self.tx_serial.is_open:
                    if time.time() - last_error_time > reconnect_delay:
                        print(f"⟳ Attempting to reconnect to TX module...")
                        if self.connect_tx():
                            print("✓ TX module reconnected!")
                            reconnect_delay = 1.0
                        else:
                            reconnect_delay = min(reconnect_delay * 2, 10.0)  # Exponential backoff
                            last_error_time = time.time()
                    time.sleep(0.1)
                    continue
                
                now = time.time()
                
                # Send RC packets at 150Hz
                if now - last_rc_send > 0.00667:  # 150Hz
                    rc_packet = self.build_crsf_rc_packet()
                    self.tx_serial.write(rc_packet)
                    self.stats['crsf_tx'] += 1
                    last_rc_send = now
                
                # Read telemetry from TX
                if self.tx_serial.in_waiting:
                    data = self.tx_serial.read(self.tx_serial.in_waiting)
                    self.stats['crsf_rx'] += len(data)
                    
                    # Debug: Show first few bytes periodically
                    if self.stats['crsf_rx'] % 1000 < len(data):
                        preview = ' '.join(f'{b:02X}' for b in data[:min(20, len(data))])
                        print(f"  CRSF RX sample: {preview}...")
                    
                    # Parse CRSF telemetry and extract MAVLink
                    mavlink_data = self.parse_crsf_telemetry(data)
                    if mavlink_data:
                        print(f"  ✓ Extracted {len(mavlink_data)} MAVLink bytes")
                        if self.mav:
                            # Forward to QGC
                            self.mav.write(mavlink_data)
                            self.stats['mav_tx'] += 1
                
            except (serial.SerialException, OSError) as e:
                if self.running:
                    if "Device not configured" in str(e) or "write failed" in str(e):
                        print(f"⚠ TX module disconnected: {e}")
                        if self.tx_serial:
                            try:
                                self.tx_serial.close()
                            except:
                                pass
                            self.tx_serial = None
                        last_error_time = time.time()
                    else:
                        print(f"CRSF error: {e}")
            except Exception as e:
                if self.running:
                    print(f"CRSF unexpected error: {e}")
            
            time.sleep(0.001)
    
    def status_thread(self):
        """Thread to print status updates"""
        last_print = time.time()
        
        while self.running:
            if time.time() - last_print > 2.0:
                print(f"Bridge Stats - MAV RX:{self.stats['mav_rx']} TX:{self.stats['mav_tx']} | "
                      f"CRSF TX:{self.stats['crsf_tx']} RX:{self.stats['crsf_rx']}")
                last_print = time.time()
            time.sleep(1)
    
    def run(self):
        """Main bridge operation"""
        print("\n" + "="*60)
        print("MAVLink to CRSF Bridge for ELRS")
        print("="*60)
        
        # Try to connect to TX (but don't fail if not available yet)
        if not self.connect_tx():
            print("⚠ TX module not connected yet - will retry automatically")
        
        # Setup MAVLink
        if not self.setup_mavlink():
            return False
        
        print("\n✓ Bridge ready!")
        print("-"*60)
        print("QGroundControl Setup:")
        print(f"1. Add UDP connection to localhost:{self.mavlink_port}")
        print("2. QGC will see the vehicle through ELRS link")
        print("\nData flow:")
        print("QGC → MAVLink/UDP → Bridge → CRSF → TX → ELRS → RX → FC")
        print("-"*60)
        print("Press Ctrl+C to stop\n")
        
        # Start threads
        self.running = True
        threads = [
            threading.Thread(target=self.mavlink_thread, name="MAVLink"),
            threading.Thread(target=self.crsf_thread, name="CRSF"),
            threading.Thread(target=self.status_thread, name="Status")
        ]
        
        for t in threads:
            t.daemon = True
            t.start()
        
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n✓ Stopping bridge...")
        
        self.running = False
        time.sleep(0.5)
        
        if self.tx_serial:
            self.tx_serial.close()
        if self.mav:
            self.mav.close()
        
        print("✓ Bridge stopped")
        return True

def main():
    """Main entry point"""
    
    # Parse arguments
    if len(sys.argv) > 1:
        tx_port = sys.argv[1]
    else:
        import glob
        ports = glob.glob('/dev/cu.usb*') + glob.glob('/dev/ttyUSB*')
        if ports:
            print(f"Available ports: {', '.join(ports)}")
            tx_port = input("Enter TX port: ").strip()
        else:
            print("No USB serial ports found")
            sys.exit(1)
    
    # Optional: UDP port for MAVLink
    mavlink_port = int(sys.argv[2]) if len(sys.argv) > 2 else 14550
    
    # Create and run bridge
    bridge = MAVLinkCRSFBridge(tx_port, mavlink_port=mavlink_port)
    bridge.run()

if __name__ == "__main__":
    main()