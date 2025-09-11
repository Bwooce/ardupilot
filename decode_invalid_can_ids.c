#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

void decode_can_id(uint32_t can_id) {
    // DroneCAN CAN ID format (29-bit extended):
    // bits 0-6: Source Node ID (7 bits)
    // bit 7: Service not message flag
    // bits 8-23: Message Type ID (16 bits)
    // bits 24-28: Priority (5 bits)
    
    int source_node_id = can_id & 0x7F;
    int service_flag = (can_id >> 7) & 0x01;
    int message_type = (can_id >> 8) & 0xFFFF;
    int priority = (can_id >> 24) & 0x1F;
    
    // Check if ID exceeds 29-bit limit
    bool exceeds_29bit = (can_id > 0x1FFFFFFF);
    
    printf("CAN ID 0x%08X breakdown:\n", can_id);
    if (exceeds_29bit) {
        printf("  ERROR: ID exceeds 29-bit extended frame limit (0x1FFFFFFF)\n");
        printf("  Bit 29 is set: %d\n", (can_id >> 29) & 0x01);
        printf("  Bit 30 is set: %d\n", (can_id >> 30) & 0x01);
        printf("  Bit 31 is set: %d\n", (can_id >> 31) & 0x01);
    }
    printf("  Priority: %d (0x%02X)\n", priority, priority);
    printf("  Message Type: %d (0x%04X)\n", message_type, message_type);
    printf("  Service Flag: %d\n", service_flag);
    printf("  Source Node ID: %d (0x%02X)\n", source_node_id, source_node_id);
    
    // Look up known message types
    switch(message_type) {
        case 341: printf("    -> NodeStatus\n"); break;
        case 342: printf("    -> GetNodeInfo request\n"); break;
        case 343: printf("    -> GetNodeInfo response\n"); break;
        case 1034: printf("    -> ESC Status\n"); break;
        case 0x03F2: printf("    -> Unknown (0x03F2 = %d)\n", 0x03F2); break;
    }
}

int main() {
    printf("=== Analyzing Invalid CAN IDs ===\n\n");
    
    printf("Invalid ID #1:\n");
    decode_can_id(0x8803F20A);
    
    printf("\nInvalid ID #2:\n");
    decode_can_id(0x9F01560A);
    
    printf("\n=== Analysis ===\n");
    printf("Both IDs have bit 31 set, making them invalid for CAN extended frames.\n");
    printf("This suggests memory corruption or incorrect pointer casting.\n");
    
    // Let's see what these would be without the high bit
    printf("\nWithout bit 31:\n");
    decode_can_id(0x8803F20A & 0x7FFFFFFF);
    printf("\n");
    decode_can_id(0x9F01560A & 0x1FFFFFFF);
    
    return 0;
}