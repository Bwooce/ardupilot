#!/usr/bin/env python3
"""
Raw MAVLink packet analyzer - no pymavlink required
Directly analyzes serial data to identify packet types
"""

import serial
import time
import struct

class MAVLinkAnalyzer:
    def __init__(self, port, baudrate=460800):
        self.port = port
        self.baudrate = baudrate
        self.buffer = b''
        
    def parse_mavlink_v2(self, data, start_idx):
        """Parse MAVLink v2 packet starting at start_idx"""
        if start_idx + 12 > len(data):
            return None, "Incomplete header"
            
        header = data[start_idx:start_idx + 12]
        magic, payload_len, incompat_flags, compat_flags, seq, sysid, compid = struct.unpack('<BBBBBBB', header[:7])
        msgid = struct.unpack('<I', header[7:10] + b'\x00')[0]  # 24-bit little-endian
        
        # Calculate total packet length
        total_len = 12 + payload_len + 2  # header + payload + checksum
        if incompat_flags & 0x01:  # signed
            total_len += 13
            
        if start_idx + total_len > len(data):
            return None, f"Incomplete packet: need {total_len}, have {len(data) - start_idx}"
            
        packet = data[start_idx:start_idx + total_len]
        
        return {
            'version': 2,
            'msgid': msgid,
            'payload_len': payload_len,
            'sysid': sysid,
            'compid': compid,
            'seq': seq,
            'packet_len': total_len,
            'incompat_flags': incompat_flags,
            'compat_flags': compat_flags,
            'raw_packet': packet
        }, None
        
    def parse_mavlink_v1(self, data, start_idx):
        """Parse MAVLink v1 packet starting at start_idx"""
        if start_idx + 8 > len(data):
            return None, "Incomplete header"
            
        header = data[start_idx:start_idx + 8]
        magic, payload_len, seq, sysid, compid, msgid = struct.unpack('<BBBBBB', header[:6])
        
        total_len = 8 + payload_len + 2  # header + payload + checksum
        
        if start_idx + total_len > len(data):
            return None, f"Incomplete packet: need {total_len}, have {len(data) - start_idx}"
            
        packet = data[start_idx:start_idx + total_len]
        
        return {
            'version': 1,
            'msgid': msgid,
            'payload_len': payload_len,
            'sysid': sysid,
            'compid': compid,
            'seq': seq,
            'packet_len': total_len,
            'raw_packet': packet
        }, None
    
    def analyze_unknown_data(self, data):
        """Analyze data that doesn't parse as valid MAVLink"""
        if len(data) == 0:
            return "EMPTY"
            
        # Check for padding patterns
        if all(b == 0x00 for b in data):
            return f"NULL_PADDING ({len(data)} bytes)"
        if all(b == 0xFF for b in data):
            return f"FF_PADDING ({len(data)} bytes)"
            
        # Check for CRSF frame
        if data[0] == 0xC8:
            return f"CRSF_FRAME ({data[:min(8, len(data))].hex()})"
            
        # Check for ASCII
        try:
            text = data.decode('ascii', errors='strict')
            if all(32 <= ord(c) <= 126 or c in '\r\n\t' for c in text):
                return f"ASCII_TEXT ('{text[:20]}{'...' if len(text) > 20 else ''}')"
        except:
            pass
            
        # Unknown binary data
        return f"UNKNOWN_BINARY (starts 0x{data[0]:02x}, {data[:8].hex()})"
    
    def find_packets_in_buffer(self):
        """Find and parse all packets in the current buffer"""
        packets = []
        unknown_segments = []
        i = 0
        
        while i < len(self.buffer):
            if self.buffer[i] == 0xFD:  # MAVLink v2
                packet, error = self.parse_mavlink_v2(self.buffer, i)
                if packet:
                    packets.append(packet)
                    i += packet['packet_len']
                else:
                    # Incomplete or corrupted, collect as unknown
                    unknown_segments.append((i, 1, f"Bad MAVLink2: {error}"))
                    i += 1
                    
            elif self.buffer[i] == 0xFE:  # MAVLink v1
                packet, error = self.parse_mavlink_v1(self.buffer, i)
                if packet:
                    packets.append(packet)
                    i += packet['packet_len']
                else:
                    unknown_segments.append((i, 1, f"Bad MAVLink1: {error}"))
                    i += 1
                    
            else:
                # Not a MAVLink start byte, find the next one or end of buffer
                start_unknown = i
                while i < len(self.buffer) and self.buffer[i] not in [0xFD, 0xFE]:
                    i += 1
                    
                unknown_data = self.buffer[start_unknown:i]
                analysis = self.analyze_unknown_data(unknown_data)
                unknown_segments.append((start_unknown, len(unknown_data), analysis))
                
        return packets, unknown_segments
    
    def analyze_stream(self, duration=30):
        """Analyze MAVLink stream for specified duration"""
        print(f"=== Raw MAVLink Stream Analysis ({duration}s) ===")
        
        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"Connected to {self.port} at {self.baudrate} baud")
        except Exception as e:
            print(f"Failed to connect: {e}")
            return
            
        start_time = time.time()
        total_bytes = 0
        valid_packets = 0
        unknown_segments_count = 0
        message_id_counts = {}
        unknown_types = {}
        
        while time.time() - start_time < duration:
            data = ser.read(1024)  # Read up to 1KB at a time
            if data:
                total_bytes += len(data)
                self.buffer += data
                
                # Process complete packets
                packets, unknown_segments = self.find_packets_in_buffer()
                
                for packet in packets:
                    valid_packets += 1
                    msgid = packet['msgid']
                    if msgid in message_id_counts:
                        message_id_counts[msgid] += 1
                    else:
                        message_id_counts[msgid] = 1
                        
                for start, length, analysis in unknown_segments:
                    unknown_segments_count += 1
                    if unknown_segments_count <= 10:  # Show first 10 unknown segments
                        print(f"  Unknown #{unknown_segments_count}: {analysis}")
                        
                    # Categorize unknown types
                    category = analysis.split('(')[0].strip()
                    if category in unknown_types:
                        unknown_types[category] += 1
                    else:
                        unknown_types[category] = 1
                
                # Remove processed data from buffer, keep last 100 bytes for incomplete packets
                if len(self.buffer) > 100:
                    self.buffer = self.buffer[-100:]
        
        ser.close()
        
        print(f"\n=== Analysis Results ===")
        print(f"Total bytes received: {total_bytes}")
        print(f"Valid MAVLink packets: {valid_packets}")
        print(f"Unknown data segments: {unknown_segments_count}")
        
        if message_id_counts:
            print(f"\nTop MAVLink Message IDs:")
            sorted_msgs = sorted(message_id_counts.items(), key=lambda x: x[1], reverse=True)
            for msgid, count in sorted_msgs[:10]:
                print(f"  ID {msgid}: {count} packets")
                
        if unknown_types:
            print(f"\nUnknown Data Types:")
            for category, count in unknown_types.items():
                print(f"  {category}: {count} segments")

if __name__ == "__main__":
    analyzer = MAVLinkAnalyzer('/dev/cu.usbserial-0001', 460800)
    analyzer.analyze_stream(30)