#include <stdio.h>
#include <stdint.h>
#include <string.h>

// DroneCAN NodeStatus message structure (7 bytes):
// - uint32_t uptime_sec (4 bytes)
// - uint2_t health (2 bits)
// - uint3_t mode (3 bits)  
// - uint3_t sub_mode (3 bits)
// - uint16_t vendor_specific_status_code (2 bytes)

void decode_can_id(uint32_t can_id) {
    // CAN ID format for DroneCAN:
    // bits 0-6: Source Node ID (7 bits)
    // bits 7: Service not message flag
    // bits 8-13: Reserved (6 bits)
    // bits 14-15: Transfer type (2 bits)
    // bits 16-20: Message type ID low (5 bits)
    // bits 21-27: Message type ID high (7 bits)
    // bit 28: Frame type (0=data, 1=service)
    
    int source_node_id = can_id & 0x7F;
    int service_flag = (can_id >> 7) & 0x01;
    int message_type = (can_id >> 8) & 0xFFFF;
    int priority = (can_id >> 24) & 0x1F;
    
    printf("CAN ID 0x%08X breakdown:\n", can_id);
    printf("  Source Node ID: %d (0x%02X)\n", source_node_id, source_node_id);
    printf("  Service Flag: %d\n", service_flag);
    printf("  Message Type: %d (0x%04X)\n", message_type, message_type);
    printf("  Priority: %d\n", priority);
}

void decode_nodestatus(uint8_t* data, int len) {
    if (len < 7) {
        printf("ERROR: NodeStatus requires 7 bytes, got %d\n", len);
        return;
    }
    
    // Extract fields according to DroneCAN encoding
    uint32_t uptime_sec = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
    
    // Byte 4 contains health (2 bits), mode (3 bits), sub_mode (3 bits)
    uint8_t health = data[4] & 0x03;
    uint8_t mode = (data[4] >> 2) & 0x07;
    uint8_t sub_mode = (data[4] >> 5) & 0x07;
    
    // Bytes 5-6 contain vendor_specific_status_code
    uint16_t vendor_code = data[5] | (data[6] << 8);
    
    printf("NodeStatus decode:\n");
    printf("  Uptime: %u seconds\n", uptime_sec);
    printf("  Health: %u (", health);
    switch(health) {
        case 0: printf("OK"); break;
        case 1: printf("WARNING"); break;
        case 2: printf("ERROR"); break;
        case 3: printf("CRITICAL"); break;
    }
    printf(")\n");
    printf("  Mode: %u (", mode);
    switch(mode) {
        case 0: printf("OPERATIONAL"); break;
        case 1: printf("INITIALIZATION"); break;
        case 2: printf("MAINTENANCE"); break;
        case 3: printf("SOFTWARE_UPDATE"); break;
        case 7: printf("OFFLINE"); break;
        default: printf("UNKNOWN");
    }
    printf(")\n");
    printf("  Sub-mode: %u\n", sub_mode);
    printf("  Vendor code: 0x%04X (%u)\n", vendor_code, vendor_code);
}

void decode_multi_frame(uint8_t* data, int len) {
    if (len < 2) {
        printf("ERROR: Multi-frame needs at least 2 bytes\n");
        return;
    }
    
    // Last 2 bytes are the tail byte
    uint8_t tail = data[len-1];
    
    // Tail byte format:
    // bits 0-1: start of transfer (1), middle (0), end (2), single frame (3)
    // bit 2: toggle bit
    // bits 3-7: transfer ID
    
    int sot_eot = tail & 0x03;
    int toggle = (tail >> 2) & 0x01;
    int transfer_id = (tail >> 3) & 0x1F;
    
    printf("Multi-frame tail byte: 0x%02X\n", tail);
    printf("  Frame type: ");
    switch(sot_eot) {
        case 0: printf("Middle frame\n"); break;
        case 1: printf("Start of transfer\n"); break;
        case 2: printf("End of transfer\n"); break;
        case 3: printf("Single frame transfer\n"); break;
    }
    printf("  Toggle: %d\n", toggle);
    printf("  Transfer ID: %d\n", transfer_id);
    
    // Payload is everything except tail byte
    printf("  Payload (%d bytes):", len-1);
    for (int i = 0; i < len-1; i++) {
        printf(" %02X", data[i]);
    }
    printf("\n");
}

int main() {
    printf("=== Decoding CAN messages from node 122 ===\n\n");
    
    // From the logs:
    // CAN_RX ID=0x10040A7A, len=8, data: 52 00 00 A9 5C 00 00 36
    printf("Frame 1: ID=0x10040A7A, len=8\n");
    decode_can_id(0x10040A7A);
    uint8_t frame1[] = {0x52, 0x00, 0x00, 0xA9, 0x5C, 0x00, 0x00, 0x36};
    decode_multi_frame(frame1, 8);
    
    // Check if this is a valid NodeStatus (should be 7 bytes + tail)
    if ((frame1[7] & 0x03) == 3) { // Single frame
        printf("Attempting to decode as NodeStatus:\n");
        decode_nodestatus(frame1, 7);
    }
    
    printf("\n");
    
    // CAN_RX ID=0x10040A7A, len=2, data: 00 56
    printf("Frame 2: ID=0x10040A7A, len=2\n");
    decode_can_id(0x10040A7A);
    uint8_t frame2[] = {0x00, 0x56};
    decode_multi_frame(frame2, 2);
    
    printf("\n");
    
    // Let's also check what NodeStatus ID 341 (0x155) should look like in CAN ID
    printf("Expected CAN ID for NodeStatus (341) from node 122:\n");
    // NodeStatus is message type 341 (0x155)
    // From node 122 (0x7A)
    // Priority 10 (0x0A) is typical for NodeStatus
    // Format: Priority(5) | MsgType(16) | ServiceNotMsg(1) | SourceNode(7)
    uint32_t expected_id = (10 << 24) | (341 << 8) | 122;
    printf("  Expected: 0x%08X\n", expected_id);
    decode_can_id(expected_id);
    
    printf("\n=== Analysis ===\n");
    printf("The received CAN ID 0x10040A7A appears to be malformed:\n");
    printf("  - Message type extracted: 0x0040 (64) not 0x0155 (341)\n");
    printf("  - This suggests corrupted CAN ID or wrong encoding\n");
    
    printf("\nThe data also appears problematic:\n");
    printf("  - Frame 1 tail byte 0x36 indicates single frame (bits 0-1 = 0x2?)\n");
    printf("  - Frame 2 with only 2 bytes is too short for NodeStatus\n");
    
    return 0;
}