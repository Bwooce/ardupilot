#!/usr/bin/env python3
"""
MAVLink packet parser for fragmented data
Looks for 'fd' start bytes and attempts to decode valid packets
"""

def parse_mavlink_data(hex_string):
    """Parse hex string and extract MAVLink packets"""
    # Convert hex string to bytes
    data = bytes.fromhex(hex_string.replace(' ', ''))
    packets = []
    
    def find_and_parse_packet(data, offset=0, depth=0):
        """Recursively find and parse MAVLink packets"""
        if offset >= len(data):
            return []
        
        # Look for MAVLink v2 start byte (0xfd)
        fd_pos = data.find(b'\xfd', offset)
        if fd_pos == -1:
            return []
        
        if fd_pos + 10 >= len(data):  # Need at least header
            return []
            
        # Parse MAVLink v2 header
        start = fd_pos
        payload_len = data[fd_pos + 1]
        incompat_flags = data[fd_pos + 2]
        compat_flags = data[fd_pos + 3]
        seq = data[fd_pos + 4]
        sysid = data[fd_pos + 5]
        compid = data[fd_pos + 6]
        
        # Message ID is 3 bytes (little-endian)
        msgid = (data[fd_pos + 9] << 16) | (data[fd_pos + 8] << 8) | data[fd_pos + 7]
        
        # Calculate total packet length
        header_len = 10  # MAVLink v2 header
        total_len = header_len + payload_len + 2  # header + payload + checksum
        
        # Check if we have enough data for complete packet
        if fd_pos + total_len > len(data):
            # Incomplete packet - look for next fd
            result = find_and_parse_packet(data, fd_pos + 1, depth + 1)
            return result
        
        # Extract the packet
        packet_data = data[fd_pos:fd_pos + total_len]
        
        # Look for embedded fd bytes within this supposed packet (excluding the start)
        embedded_fd = packet_data.find(b'\xfd', 1)
        if embedded_fd != -1 and embedded_fd < payload_len + 10:
            # Found embedded fd - this might be fragmented
            # Parse the embedded packet instead
            result = find_and_parse_packet(data, fd_pos + embedded_fd, depth + 1)
            # Also continue searching after this position
            result.extend(find_and_parse_packet(data, fd_pos + 1, depth + 1))
            return result
        
        # Validate message ID (should be reasonable)
        if msgid > 50000:  # Arbitrary upper limit for sanity
            # Invalid message ID - keep searching
            result = find_and_parse_packet(data, fd_pos + 1, depth + 1)
            return result
        
        # Extract payload and checksum
        payload = packet_data[header_len:header_len + payload_len] if payload_len > 0 else b''
        checksum = packet_data[header_len + payload_len:header_len + payload_len + 2]
        
        packet_info = {
            'offset': fd_pos,
            'length': total_len,
            'payload_len': payload_len,
            'seq': seq,
            'sysid': sysid,
            'compid': compid,
            'msgid': msgid,
            'payload': payload.hex() if payload else '',
            'checksum': checksum.hex() if len(checksum) == 2 else 'incomplete',
            'raw': packet_data.hex(),
            'depth': depth
        }
        
        result = [packet_info]
        
        # Continue searching for more packets after this one
        result.extend(find_and_parse_packet(data, fd_pos + total_len, depth))
        
        return result
    
    return find_and_parse_packet(data)

def get_message_name(msgid):
    """Get human-readable message name"""
    message_names = {
        0: "HEARTBEAT",
        1: "SYS_STATUS", 
        24: "GPS_RAW_INT",
        30: "ATTITUDE",
        33: "GLOBAL_POSITION_INT",
        74: "VFR_HUD",
        147: "BATTERY_STATUS",
        230: "ESTIMATOR_STATUS",
        # Add more as needed
    }
    return message_names.get(msgid, f"UNKNOWN_{msgid}")

def analyze_data(raw_data):
    """Analyze the provided MAVLink data"""
    print("Analyzing MAVLink data...")
    print("=" * 60)
    
    packets = parse_mavlink_data(raw_data)
    
    if not packets:
        print("No valid MAVLink packets found!")
        return
    
    for i, packet in enumerate(packets):
        print(f"\nPacket {i+1}:")
        print(f"  Offset: {packet['offset']} bytes")
        print(f"  Message ID: {packet['msgid']} ({get_message_name(packet['msgid'])})")
        print(f"  Sequence: {packet['seq']}")
        print(f"  System/Component: {packet['sysid']}/{packet['compid']}")
        print(f"  Payload Length: {packet['payload_len']} bytes")
        print(f"  Total Length: {packet['length']} bytes")
        print(f"  Checksum: {packet['checksum']}")
        print(f"  Parse depth: {packet['depth']}")
        if packet['payload_len'] <= 20:  # Show payload for small packets
            print(f"  Payload: {packet['payload']}")
        print(f"  Raw packet: {packet['raw'][:40]}{'...' if len(packet['raw']) > 40 else ''}")

if __name__ == "__main__":
    # Your provided data
    test_data = """
    00c701019300000000000000000000ff7fffffffffffffffffffffffffffffffffffffffff0000000000ff0000000006e77f
    fd090000c80101000000000000000a03410403cd1efd1c0000c90101a30000dcfc8736
    aff0d4367465a1ab0000000000000000d94d6e370000803f2955fd1c0000ca01011e0000563201000406d0b60000000000ff00000000010000000000
    000000000000000000000000000000000000000000000000000000000000000000ffe449fd1d0000d301011b00002bf3ac040000000000
    00000019fc00000000000000000000000000fc085c78fd180000d401011800000000000000000000000000000000000000000000fffffffffe5efd0b
    0000d5010102000000000000000000007e3201a503fd160000d601012200007e320100000000000000000000000000000000
    0000ff90b8fd140000d70101f10000c840ad04000000000500c0290500402805004036d043fd290000d801019300000000000000000000
    ff7fffffffffffffffffffffffffffffffffffffffff0000000000ff00000000068619
    fd090000d90101000000000000000a034104035531fd1c0000da0101a30000dcfc8736aff0d4367465a1ab000000000000000062d13237
    0000803fe89cfd1c0000db01011e00003f360100f5f033340000000000ff000000000100000000000000000000000000000000000000000000000000
    00000000000000000000000000000000000000000000fff8dffd1d0000e401011b00009e35bc04000000000000000019fc00000000000000000000000000fc08
    ccbcfd180000e501011800000000000000000000000000000000000000000000ffffffff5ecffd0b0000e6010102000000000000000000
    006636010554fd160000e70101220000663601000000000000000000000000000000000000ffc372fd140000e80101f10000f582bc0400
    0000000500c02905004028050040361b7bfd290000e901019300000000000000000000ff7fffffffffffffffffffffffffffffffffffffffff000000
    0000ff0000000006c144fd0d0000ea01016f00000000000000000000e110e0a812b11c
    fd090000eb0101000000000000000a
    03410403edcffd1c0000ec0101a300000da083361cd5e136f1fe7aab000000000000000031c819370000803ff4adfd1c0000ed01011e0000273a0100
    f26537340000000000ff00000000010000000000000000000000000000000000000000000000000000000000000000000000000000ff25
    dcfd1d0000f601011b00006b77cb04000000000000000019fc00000000000000000000000000fc08bbd6fd180000f701011800000000000000000000
    000000000000000000000000ffffffffab7efd0b0000f8010102000000000000000000004e3a019d90fd160000f901012200004e3a0100
    0000000000000000000000000000000000ff9167fd140000fa0101f1000058c5cb04000000000500c0290500402805004036
    0acefd290000fb01019300000000000000000000ff7fffffffffffffffffffffffffffffffffffffffff0000000000ff0000000006d129
    fd090000fc0101000000000000000a0341040326dcfd1c0000fd0101a300000da08336
    1cd5e136f1fe7aab0000000000000000e86b77370000803f2c49fd1c0000fe01011e00000f3e010001b6aab60000000000ff00000000010000000000
    000000000000000000000000000000000000000000000000000000000000000000ffbedafd1d00000701011b00009fb9da040000000000
    00000019fc00000000000000000000000000fc083e43fd1800000801011800000000000000000000000000000000000000000000ffffffff1622fd0b
    00000901010200000000000000000000363e0106e9fd1600000a0101220000363e0100000000000000000000000000000000
    0000ff435ffd1400000b0101f10000bb07db04000000000500c02905004028050040362be5fd2900000c01019300000000000000000000ff7fffffff
    ffffffffffffffffffffffffffffffffff0000000000ff00000000066f93
    fd0900000d0101000000000000000a03410403a80afd1c00000e0101a300000da083361cd5e136f1fe7aab00000000000000006ce37a37
    0000803f70ebfd1c00000f01011e0000f7410100a72c2ab70000000000ff00000000010000000000000000000000000000000000000000
    """
    
    # Remove newlines and clean up
    clean_data = ''.join(test_data.strip().split())
    analyze_data(clean_data)