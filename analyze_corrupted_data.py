#!/usr/bin/env python3
"""
Extract valid MAVLink packets and analyze remaining corrupted data
"""

def extract_valid_packets(hex_string):
    """Extract only valid MAVLink packets and return remaining data"""
    data = bytes.fromhex(hex_string.replace(' ', ''))
    valid_packets = []
    used_ranges = []
    
    def is_valid_message_id(msgid):
        """Check if message ID exists in MAVLink spec"""
        # Complete MAVLink message ID set including ArduPilot-specific messages
        # Based on https://mavlink.io/en/messages/ardupilotmega.html and common.xml
        valid_ids = {
            # Common MAVLink messages (0-299)
            0, 1, 2, 4, 5, 6, 7, 11, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
            40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 54, 55, 61, 62, 63, 64, 65, 66, 67, 69, 70, 73, 74, 75, 76, 77,
            81, 82, 83, 84, 85, 86, 87, 89, 90, 91, 92, 93, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112,
            113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135,
            136, 137, 138, 139, 140, 141, 142, 143, 144, 146, 147, 148, 149, 162, 192, 225, 230, 231, 232, 233, 234, 235,
            241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 256, 257, 258, 259, 260, 261, 262, 263,
            264, 265, 266, 267, 268, 269, 270, 271, 275, 276, 277, 280, 281, 282, 283, 284, 285, 286, 287, 288, 299,
            
            # ArduPilot-specific messages (150-180)
            150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180,
            
            # ArduPilot-specific messages (300-399)  
            300, 301, 310, 311, 320, 321, 322, 323, 324, 330, 331, 332, 333, 335, 339, 340, 350, 360, 370, 373, 375, 376, 385, 386, 387, 388,
            
            # Additional ArduPilot messages
            11000, 11001, 11002, 11003, 11010, 11011, 11020, 11030, 11031, 11032, 11033, 11034, 11035, 11036, 11037, 11038, 11039, 11040, 11041, 11042,
            11050, 11051, 11052, 11053, 11054, 11055, 11056, 11057, 11058, 11059, 11060, 11061, 11062, 11063, 11064, 11065, 11066, 11067, 11068, 11069,
            11070, 11071, 11072, 11073, 11074, 11075, 11076, 11077, 11078, 11079, 11080, 11081, 11082, 11083, 11084, 11085, 11086, 11087, 11088, 11089,
            11090, 11091, 11092, 11093, 11094, 11095, 11096, 11097, 11098, 11099, 11100, 11101, 11102, 11103, 11104, 11105, 11106, 11107, 11108, 11109,
            11110, 11111, 11112, 11113, 11114, 11115, 11116, 11117, 11118, 11119, 11120, 11121, 11122, 11123, 11124, 11125, 11126, 11127, 11128, 11129,
            11130, 11131, 11132, 11133, 11134, 11135, 11136, 11137, 11138, 11139, 11140, 11141, 11142, 11143, 11144, 11145, 11146, 11147, 11148, 11149,
        }
        return msgid in valid_ids
    
    def find_valid_packets(data, offset=0):
        """Find valid MAVLink packets with improved parsing"""
        packets = []
        pos = offset
        
        while pos < len(data):
            # Look for MAVLink v2 start byte (0xfd)
            fd_pos = data.find(b'\xfd', pos)
            if fd_pos == -1:
                break
                
            # Need at least 10 bytes for header
            if fd_pos + 10 > len(data):
                break
                
            # Parse MAVLink v2 header
            payload_len = data[fd_pos + 1]
            incompat_flags = data[fd_pos + 2]
            compat_flags = data[fd_pos + 3]
            seq = data[fd_pos + 4]
            sysid = data[fd_pos + 5]
            compid = data[fd_pos + 6]
            
            # Message ID is 3 bytes (little-endian)
            msgid = (data[fd_pos + 9] << 16) | (data[fd_pos + 8] << 8) | data[fd_pos + 7]
            
            # Calculate total packet length
            header_len = 10
            total_len = header_len + payload_len + 2
            
            # Check if we have enough data for complete packet
            if fd_pos + total_len > len(data):
                # Incomplete packet at end of data - skip
                pos = fd_pos + 1
                continue
            
            # Basic validation - reasonable payload length and message ID
            if payload_len > 255 or msgid > 50000:
                # Invalid packet - continue searching
                pos = fd_pos + 1
                continue
                
            # Validate message ID against known set (optional - can be removed for broader acceptance)
            if not is_valid_message_id(msgid):
                # Unknown message ID - continue searching  
                pos = fd_pos + 1
                continue
            
            # Extract the packet
            packet_data = data[fd_pos:fd_pos + total_len]
            
            packet_info = {
                'offset': fd_pos,
                'length': total_len,
                'msgid': msgid,
                'seq': seq,
                'sysid': sysid,
                'compid': compid,
                'payload_len': payload_len,
                'data': packet_data
            }
            
            packets.append(packet_info)
            
            # Move to the end of this packet to continue searching
            pos = fd_pos + total_len
        
        return packets
    
    packets = find_valid_packets(data)
    
    # Create list of used byte ranges
    for packet in packets:
        used_ranges.append((packet['offset'], packet['offset'] + packet['length']))
    
    # Sort ranges by offset
    used_ranges.sort()
    
    return packets, used_ranges, data

def get_unused_data(data, used_ranges):
    """Extract data not used by valid packets"""
    unused_chunks = []
    last_end = 0
    
    for start, end in used_ranges:
        if start > last_end:
            # There's unused data between packets
            unused_chunks.append((last_end, start, data[last_end:start]))
        last_end = end
    
    # Check for unused data at the end
    if last_end < len(data):
        unused_chunks.append((last_end, len(data), data[last_end:]))
    
    return unused_chunks

def get_message_name(msgid):
    """Get human-readable message name"""
    message_names = {
        # Common MAVLink messages
        0: "HEARTBEAT",
        1: "SYS_STATUS", 
        2: "SYSTEM_TIME",
        4: "PING",
        5: "CHANGE_OPERATOR_CONTROL",
        6: "CHANGE_OPERATOR_CONTROL_ACK",
        7: "AUTH_KEY",
        11: "SET_MODE",
        20: "PARAM_REQUEST_READ",
        21: "PARAM_REQUEST_LIST", 
        22: "PARAM_VALUE",
        23: "PARAM_SET",
        24: "GPS_RAW_INT",
        25: "GPS_STATUS",
        26: "SCALED_IMU",
        27: "RAW_IMU", 
        28: "RAW_PRESSURE",
        29: "SCALED_PRESSURE",
        30: "ATTITUDE",
        31: "ATTITUDE_QUATERNION",
        32: "LOCAL_POSITION_NED",
        33: "GLOBAL_POSITION_INT",
        34: "RC_CHANNELS_SCALED",
        35: "RC_CHANNELS_RAW",
        36: "SERVO_OUTPUT_RAW",
        37: "MISSION_REQUEST_PARTIAL_LIST",
        38: "MISSION_WRITE_PARTIAL_LIST",
        39: "MISSION_ITEM",
        40: "MISSION_REQUEST",
        41: "MISSION_SET_CURRENT",
        42: "MISSION_CURRENT",
        43: "MISSION_REQUEST_LIST",
        44: "MISSION_COUNT",
        45: "MISSION_CLEAR_ALL",
        46: "MISSION_ITEM_REACHED",
        47: "MISSION_ACK",
        48: "SET_GPS_GLOBAL_ORIGIN",
        49: "GPS_GLOBAL_ORIGIN",
        50: "PARAM_MAP_RC",
        51: "MISSION_REQUEST_INT",
        54: "SAFETY_SET_ALLOWED_AREA",
        55: "SAFETY_ALLOWED_AREA",
        61: "ATTITUDE_QUATERNION_COV",
        62: "NAV_CONTROLLER_OUTPUT",
        63: "GLOBAL_POSITION_INT_COV",
        64: "LOCAL_POSITION_NED_COV",
        65: "RC_CHANNELS",
        66: "REQUEST_DATA_STREAM",
        67: "DATA_STREAM",
        69: "MANUAL_CONTROL",
        70: "RC_CHANNELS_OVERRIDE",
        73: "MISSION_ITEM_INT",
        74: "VFR_HUD",
        75: "COMMAND_INT",
        76: "COMMAND_LONG",
        77: "COMMAND_ACK",
        81: "MANUAL_SETPOINT",
        82: "SET_ATTITUDE_TARGET",
        83: "ATTITUDE_TARGET",
        84: "SET_POSITION_TARGET_LOCAL_NED",
        85: "POSITION_TARGET_LOCAL_NED",
        86: "SET_POSITION_TARGET_GLOBAL_INT",
        87: "POSITION_TARGET_GLOBAL_INT",
        89: "LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET",
        90: "HIL_STATE",
        91: "HIL_CONTROLS",
        92: "HIL_RC_INPUTS_RAW",
        93: "HIL_ACTUATOR_CONTROLS",
        100: "OPTICAL_FLOW",
        101: "GLOBAL_VISION_POSITION_ESTIMATE",
        102: "VISION_POSITION_ESTIMATE",
        103: "VISION_SPEED_ESTIMATE",
        104: "VICON_POSITION_ESTIMATE",
        105: "HIGHRES_IMU",
        106: "OPTICAL_FLOW_RAD",
        107: "HIL_SENSOR",
        108: "SIM_STATE",
        109: "RADIO_STATUS",
        110: "FILE_TRANSFER_PROTOCOL",
        111: "TIMESYNC",
        112: "CAMERA_TRIGGER",
        113: "HIL_GPS",
        114: "HIL_OPTICAL_FLOW",
        115: "HIL_STATE_QUATERNION",
        116: "SCALED_IMU2",
        117: "LOG_REQUEST_LIST",
        118: "LOG_ENTRY",
        119: "LOG_REQUEST_DATA",
        120: "LOG_DATA",
        121: "LOG_ERASE",
        122: "LOG_REQUEST_END",
        123: "GPS_INJECT_DATA",
        124: "GPS2_RAW",
        125: "POWER_STATUS",
        126: "SERIAL_CONTROL",
        127: "GPS_RTK",
        128: "GPS2_RTK",
        129: "SCALED_IMU3",
        130: "DATA_TRANSMISSION_HANDSHAKE",
        131: "ENCAPSULATED_DATA",
        132: "DISTANCE_SENSOR",
        133: "TERRAIN_REQUEST",
        134: "TERRAIN_DATA",
        135: "TERRAIN_CHECK",
        136: "TERRAIN_REPORT",
        137: "SCALED_PRESSURE2",
        138: "ATT_POS_MOCAP",
        139: "SET_ACTUATOR_CONTROL_TARGET",
        140: "ACTUATOR_CONTROL_TARGET",
        141: "ALTITUDE",
        142: "RESOURCE_REQUEST",
        143: "SCALED_PRESSURE3",
        144: "FOLLOW_TARGET",
        146: "CONTROL_SYSTEM_STATE",
        147: "BATTERY_STATUS",
        148: "AUTOPILOT_VERSION",
        149: "LANDING_TARGET",
        
        # ArduPilot-specific messages
        150: "SENSOR_OFFSETS",
        151: "SET_MAG_OFFSETS", 
        152: "MEMINFO",
        153: "AP_ADC",
        154: "DIGICAM_CONFIGURE",
        155: "DIGICAM_CONTROL",
        156: "MOUNT_CONFIGURE",
        157: "MOUNT_CONTROL",
        158: "MOUNT_STATUS",
        159: "FENCE_POINT",
        160: "FENCE_FETCH_POINT",
        161: "FENCE_STATUS",
        162: "MAG_CAL_PROGRESS",
        163: "AHRS",
        164: "SIMSTATE",
        165: "HWSTATUS",
        166: "RADIO",
        167: "LIMITS_STATUS",
        168: "WIND",
        169: "DATA16",
        170: "DATA32",
        171: "DATA64",
        172: "DATA96",
        173: "RANGEFINDER",
        174: "AIRSPEED_AUTOCAL",
        175: "RALLY_POINT",
        176: "RALLY_FETCH_POINT",
        177: "COMPASSMOT_STATUS",
        178: "AHRS2",
        179: "CAMERA_STATUS",
        180: "CAMERA_FEEDBACK",
        
        230: "ESTIMATOR_STATUS",
        231: "WIND_COV",
        232: "GPS_INPUT",
        233: "GPS_RTCM_DATA",
        234: "HIGH_LATENCY",
        235: "HIGH_LATENCY2",
        241: "VIBRATION",
        242: "HOME_POSITION",
        243: "SET_HOME_POSITION",
        244: "MESSAGE_INTERVAL",
        245: "EXTENDED_SYS_STATE",
        246: "ADSB_VEHICLE",
        247: "COLLISION",
        248: "V2_EXTENSION",
        249: "MEMORY_VECT",
        250: "DEBUG_VECT",
        251: "NAMED_VALUE_FLOAT",
        252: "NAMED_VALUE_INT",
        253: "STATUSTEXT",
        254: "DEBUG",
        256: "SETUP_SIGNING",
        257: "BUTTON_CHANGE",
        258: "PLAY_TUNE",
        259: "CAMERA_INFORMATION",
        260: "CAMERA_SETTINGS",
        261: "STORAGE_INFORMATION",
        262: "CAMERA_CAPTURE_STATUS",
        263: "CAMERA_IMAGE_CAPTURED",
        264: "FLIGHT_INFORMATION",
        265: "MOUNT_ORIENTATION",
        266: "LOGGING_DATA",
        267: "LOGGING_DATA_ACKED",
        268: "LOGGING_ACK",
        269: "VIDEO_STREAM_INFORMATION",
        270: "VIDEO_STREAM_STATUS",
        271: "CAMERA_FOV_STATUS",
        275: "CAMERA_TRACKING_IMAGE_STATUS",
        276: "CAMERA_TRACKING_GEO_STATUS",
        277: "GIMBAL_MANAGER_INFORMATION",
        280: "GIMBAL_MANAGER_STATUS",
        281: "GIMBAL_MANAGER_SET_ATTITUDE",
        282: "GIMBAL_MANAGER_SET_PITCHYAW",
        283: "GIMBAL_DEVICE_INFORMATION",
        284: "GIMBAL_DEVICE_SET_ATTITUDE",
        285: "GIMBAL_DEVICE_ATTITUDE_STATUS",
        286: "AUTOPILOT_STATE_FOR_GIMBAL_DEVICE",
        287: "GIMBAL_MANAGER_SET_TILTPAN",
        288: "WIFI_CONFIG_AP",
        299: "PROTOCOL_VERSION",
        
        # More ArduPilot messages
        300: "AQ_TELEMETRY_F",
        301: "AQ_ESC_TELEMETRY",
        310: "ESC_TELEMETRY_1_TO_4",
        311: "ESC_TELEMETRY_5_TO_8",
        320: "OSD_PARAM_CONFIG",
        321: "OSD_PARAM_CONFIG_REPLY",
        322: "OSD_PARAM_SHOW_CONFIG",
        323: "OSD_PARAM_SHOW_CONFIG_REPLY",
        324: "OBSTACLE_DISTANCE_3D",
        330: "WATER_DEPTH",
        331: "MCU_STATUS",
        
        # High-numbered ArduPilot messages
        11000: "DEVICE_OP_READ",
        11001: "DEVICE_OP_READ_REPLY", 
        11002: "DEVICE_OP_WRITE",
        11003: "DEVICE_OP_WRITE_REPLY",
        11010: "ADAP_TUNING",
        11011: "VISION_POSITION_DELTA",
        11020: "AOA_SSA",
        11030: "ESC_TELEMETRY_1_TO_4",
        11031: "ESC_TELEMETRY_5_TO_8",
        11032: "ESC_TELEMETRY_9_TO_12",
    }
    return message_names.get(msgid, f"MSG_{msgid}")

def analyze_unused_data(unused_chunks):
    """Analyze unused data for patterns"""
    print("=== UNUSED/CORRUPTED DATA ANALYSIS ===")
    print()
    
    all_unused = b''.join([chunk[2] for chunk in unused_chunks])
    
    print(f"Total unused data: {len(all_unused)} bytes")
    print(f"Number of unused chunks: {len(unused_chunks)}")
    print()
    
    for i, (start, end, chunk) in enumerate(unused_chunks):
        print(f"Chunk {i+1}: Bytes {start}-{end} ({len(chunk)} bytes)")
        
        # Look for ASCII text
        try:
            ascii_text = chunk.decode('ascii', errors='ignore')
            printable_chars = ''.join(c for c in ascii_text if c.isprintable())
            if len(printable_chars) > 3:
                print(f"  ASCII text found: '{printable_chars}'")
        except:
            pass
        
        # Look for common patterns
        hex_str = chunk.hex()
        print(f"  Hex: {hex_str[:60]}{'...' if len(hex_str) > 60 else ''}")
        
        # Look for repeated patterns
        if len(chunk) >= 4:
            for pattern_len in [1, 2, 4]:
                if len(chunk) >= pattern_len * 3:  # Need at least 3 repetitions
                    pattern = chunk[:pattern_len]
                    repeats = 0
                    pos = 0
                    while pos + pattern_len <= len(chunk):
                        if chunk[pos:pos + pattern_len] == pattern:
                            repeats += 1
                            pos += pattern_len
                        else:
                            break
                    if repeats >= 3:
                        print(f"  Repeated pattern ({pattern_len} bytes, {repeats} times): {pattern.hex()}")
        
        # Look for potential MAVLink fragments
        if b'\xfd' in chunk:
            fd_positions = []
            pos = 0
            while True:
                fd_pos = chunk.find(b'\xfd', pos)
                if fd_pos == -1:
                    break
                fd_positions.append(fd_pos)
                pos = fd_pos + 1
            print(f"  MAVLink start bytes found at positions: {fd_positions}")
        
        # Look for NULL bytes
        null_count = chunk.count(b'\x00')
        if null_count > len(chunk) // 4:  # More than 25% null bytes
            print(f"  High NULL byte content: {null_count}/{len(chunk)} ({null_count/len(chunk)*100:.1f}%)")
        
        # Look for 0xFF bytes (often indicates padding or errors)
        ff_count = chunk.count(b'\xff')
        if ff_count > len(chunk) // 4:  # More than 25% 0xFF bytes
            print(f"  High 0xFF byte content: {ff_count}/{len(chunk)} ({ff_count/len(chunk)*100:.1f}%)")
        
        print()

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
    0000ff435ffd1400000b0101f10000bb07db04000000000500c0290500402805004036362be5fd2900000c01019300000000000000000000ff7fffffff
    ffffffffffffffffffffffffffffffffff0000000000ff00000000066f93
    fd0900000d0101000000000000000a03410403a80afd1c00000e0101a300000da083361cd5e136f1fe7aab00000000000000006ce37a37
    0000803f70ebfd1c00000f01011e0000f7410100a72c2ab70000000000ff00000000010000000000000000000000000000000000000000
    """
    
    # Clean up data
    clean_data = ''.join(test_data.strip().split())
    
    print("Extracting valid MAVLink packets...")
    packets, used_ranges, raw_data = extract_valid_packets(clean_data)
    
    print(f"Found {len(packets)} valid MAVLink packets:")
    for i, packet in enumerate(packets[:10]):  # Show first 10
        msg_name = get_message_name(packet['msgid'])
        print(f"  {i+1}: Offset {packet['offset']}, {msg_name} (ID {packet['msgid']}), Seq {packet['seq']}, Len {packet['length']}")
    if len(packets) > 10:
        print(f"  ... and {len(packets) - 10} more")
    print()
    
    # Show message type summary
    msg_counts = {}
    for packet in packets:
        msg_name = get_message_name(packet['msgid'])
        msg_counts[msg_name] = msg_counts.get(msg_name, 0) + 1
    
    print("Message type summary:")
    for msg_name, count in sorted(msg_counts.items()):
        print(f"  {msg_name}: {count} packets")
    print()
    
    # Get unused data
    unused_chunks = get_unused_data(raw_data, used_ranges)
    
    # Analyze unused data
    analyze_unused_data(unused_chunks)