// Test to verify bool size and struct layout on ESP32
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

struct test_allocation {
    uint8_t node_id;
    bool first_part_of_unique_id;
    struct { uint8_t len; uint8_t data[16]; } unique_id;
};

struct test_allocation_uint8 {
    uint8_t node_id;
    uint8_t first_part_of_unique_id;  // Use uint8_t instead of bool
    struct { uint8_t len; uint8_t data[16]; } unique_id;
};

int main() {
    printf("Testing struct layout for DNA allocation on ESP32:\n\n");
    
    printf("Size comparison:\n");
    printf("  sizeof(bool) = %zu\n", sizeof(bool));
    printf("  sizeof(uint8_t) = %zu\n", sizeof(uint8_t));
    printf("  sizeof(struct test_allocation) = %zu\n", sizeof(struct test_allocation));
    printf("  sizeof(struct test_allocation_uint8) = %zu\n", sizeof(struct test_allocation_uint8));
    
    printf("\nField offsets with bool:\n");
    printf("  offsetof(node_id) = %zu\n", offsetof(struct test_allocation, node_id));
    printf("  offsetof(first_part_of_unique_id) = %zu\n", offsetof(struct test_allocation, first_part_of_unique_id));
    printf("  offsetof(unique_id) = %zu\n", offsetof(struct test_allocation, unique_id));
    printf("  offsetof(unique_id.len) = %zu\n", offsetof(struct test_allocation, unique_id) + offsetof(struct { uint8_t len; uint8_t data[16]; }, len));
    printf("  offsetof(unique_id.data) = %zu\n", offsetof(struct test_allocation, unique_id) + offsetof(struct { uint8_t len; uint8_t data[16]; }, data));
    
    printf("\nField offsets with uint8_t:\n");
    printf("  offsetof(node_id) = %zu\n", offsetof(struct test_allocation_uint8, node_id));
    printf("  offsetof(first_part_of_unique_id) = %zu\n", offsetof(struct test_allocation_uint8, first_part_of_unique_id));
    printf("  offsetof(unique_id) = %zu\n", offsetof(struct test_allocation_uint8, unique_id));
    
    printf("\nPotential issue:\n");
    if (sizeof(bool) != 1) {
        printf("  WARNING: bool is %zu bytes, not 1 byte!\n", sizeof(bool));
        printf("  This could cause struct packing issues.\n");
    }
    
    // Test actual encoding behavior
    struct test_allocation msg = {0};
    msg.node_id = 64;
    msg.first_part_of_unique_id = false;
    msg.unique_id.len = 6;
    msg.unique_id.data[0] = 0x80;
    msg.unique_id.data[1] = 0x65;
    
    printf("\nMemory dump of struct (first 8 bytes):\n");
    uint8_t* ptr = (uint8_t*)&msg;
    for (int i = 0; i < 8; i++) {
        printf("  [%d] = 0x%02X\n", i, ptr[i]);
    }
    
    return 0;
}