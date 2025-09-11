// Test DNA allocation response encoding on ESP32
// Compile with: xtensa-esp32s3-elf-gcc -I./build/esp32lilygo_tconnect/modules/DroneCAN/libcanard/dsdlc_generated/include -I./modules/DroneCAN/libcanard -o test_dna_response test_dna_response_encoding.cpp

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// Simplified structure matching the DroneCAN definition
struct uavcan_protocol_dynamic_node_id_Allocation {
    uint8_t node_id;
    bool first_part_of_unique_id;
    struct { uint8_t len; uint8_t data[16]; } unique_id;
};

// Simulate the encoding function
void encode_allocation(uint8_t* buffer, struct uavcan_protocol_dynamic_node_id_Allocation* msg) {
    // First byte should contain:
    // Bits 0-6: node_id (7 bits)
    // Bit 7: first_part_of_unique_id (1 bit)
    buffer[0] = (msg->node_id & 0x7F) | ((msg->first_part_of_unique_id ? 1 : 0) << 7);
    
    // For echoing request (node_id=0), we should copy the UID bytes
    // The remaining bytes are the UID data
    memcpy(&buffer[1], msg->unique_id.data, msg->unique_id.len);
}

int main() {
    printf("Testing DNA allocation response encoding on ESP32\n");
    printf("sizeof(bool) = %zu\n", sizeof(bool));
    printf("sizeof(struct) = %zu\n", sizeof(struct uavcan_protocol_dynamic_node_id_Allocation));
    
    // Test case 1: Echo response (node_id=0)
    printf("\nTest 1: Echo response (node_id=0)\n");
    struct uavcan_protocol_dynamic_node_id_Allocation echo_rsp;
    memset(&echo_rsp, 0, sizeof(echo_rsp));
    echo_rsp.node_id = 0;
    echo_rsp.first_part_of_unique_id = true;  // Echo the first_part flag
    echo_rsp.unique_id.len = 6;
    // Set the UID to match what ESC sends: 80 80 65 99 5B E6 20
    uint8_t test_uid[6] = {0x80, 0x65, 0x99, 0x5B, 0xE6, 0x20};
    memcpy(echo_rsp.unique_id.data, test_uid, 6);
    
    printf("Input structure:\n");
    printf("  node_id = %d (0x%02X)\n", echo_rsp.node_id, echo_rsp.node_id);
    printf("  first_part = %d\n", echo_rsp.first_part_of_unique_id);
    printf("  uid.len = %d\n", echo_rsp.unique_id.len);
    printf("  uid.data = ");
    for (int i = 0; i < echo_rsp.unique_id.len; i++) {
        printf("%02X ", echo_rsp.unique_id.data[i]);
    }
    printf("\n");
    
    // Encode it
    uint8_t buffer[32];
    memset(buffer, 0xFF, sizeof(buffer));
    encode_allocation(buffer, &echo_rsp);
    
    printf("Encoded output (first 8 bytes):\n  ");
    for (int i = 0; i < 8; i++) {
        printf("%02X ", buffer[i]);
    }
    printf("\n");
    
    // Expected: 80 80 65 99 5B E6 20 (first byte 0x80 = node_id:0 | first_part:1)
    // But ESC sees: 80 80 65 99 5B E6 20 (exactly what was sent!)
    
    // Test case 2: Allocation response (node_id=64)
    printf("\nTest 2: Allocation response (node_id=64)\n");
    struct uavcan_protocol_dynamic_node_id_Allocation alloc_rsp;
    memset(&alloc_rsp, 0, sizeof(alloc_rsp));
    alloc_rsp.node_id = 64;
    alloc_rsp.first_part_of_unique_id = false;  // Clear for final response
    alloc_rsp.unique_id.len = 6;
    memcpy(alloc_rsp.unique_id.data, test_uid, 6);
    
    printf("Input structure:\n");
    printf("  node_id = %d (0x%02X)\n", alloc_rsp.node_id, alloc_rsp.node_id);
    printf("  first_part = %d\n", alloc_rsp.first_part_of_unique_id);
    printf("  uid.len = %d\n", alloc_rsp.unique_id.len);
    
    memset(buffer, 0xFF, sizeof(buffer));
    encode_allocation(buffer, &alloc_rsp);
    
    printf("Encoded output (first 8 bytes):\n  ");
    for (int i = 0; i < 8; i++) {
        printf("%02X ", buffer[i]);
    }
    printf("\n");
    
    // Expected first byte: 0x40 (node_id:64 | first_part:0)
    // Then the 6 UID bytes
    
    printf("\nAnalysis:\n");
    printf("The problem is that the DNA server is sending the UID bytes starting at byte[0]\n");
    printf("instead of encoding (node_id | first_part<<7) in byte[0] and UID starting at byte[1]\n");
    printf("This suggests the encoder is either:\n");
    printf("1. Not being called at all (raw struct being sent)\n");
    printf("2. Being bypassed somehow\n");
    printf("3. The response struct has wrong values when passed to encoder\n");
    
    return 0;
}