#include <stdio.h>
#include <stdint.h>

// Test the encoding from battery_node.c (official libcanard example)
void test_libcanard_encoding(uint8_t node_id, uint8_t first_part) {
    uint8_t byte = (node_id << 1);
    if (first_part) {
        byte |= 1;
    }

    printf("Libcanard encoding (node_id=%d, first_part=%d):\n", node_id, first_part);
    printf("  Result: 0x%02X (binary: ", byte);
    for (int i = 7; i >= 0; i--) {
        printf("%d", (byte >> i) & 1);
    }
    printf(")\n");
    printf("  Bit 0 (LSB): %d = first_part\n", byte & 1);
    printf("  Bits 1-7: 0x%02X = %d = node_id\n", (byte >> 1) & 0x7F, (byte >> 1) & 0x7F);
    printf("\n");
}

// Test the node team's claimed encoding
void test_node_team_encoding(uint8_t node_id, uint8_t first_part) {
    uint8_t byte = (node_id & 0x7F);  // node_id in bits 0-6
    if (first_part) {
        byte |= 0x80;  // first_part in bit 7
    }

    printf("Node team encoding (node_id=%d, first_part=%d):\n", node_id, first_part);
    printf("  Result: 0x%02X (binary: ", byte);
    for (int i = 7; i >= 0; i--) {
        printf("%d", (byte >> i) & 1);
    }
    printf(")\n");
    printf("  Bits 0-6 (lower 7): 0x%02X = %d = node_id\n", byte & 0x7F, byte & 0x7F);
    printf("  Bit 7 (MSB): %d = first_part\n", (byte >> 7) & 1);
    printf("\n");
}

int main() {
    printf("=== DNA Allocation Byte 0 Encoding Test ===\n\n");

    printf("Test case: node_id=0, first_part=1 (first message)\n");
    printf("--------------------------------------------\n");
    test_libcanard_encoding(0, 1);
    test_node_team_encoding(0, 1);

    printf("\nTest case: node_id=0, first_part=0 (follow-up message)\n");
    printf("-------------------------------------------------------\n");
    test_libcanard_encoding(0, 0);
    test_node_team_encoding(0, 0);

    printf("\nTest case: node_id=73, first_part=1 (battery_node.c example)\n");
    printf("-------------------------------------------------------------\n");
    test_libcanard_encoding(73, 1);
    test_node_team_encoding(73, 1);

    printf("\n=== Analysis of 0x80 ===\n");
    uint8_t test_byte = 0x80;
    printf("0x80 = binary 10000000\n");
    printf("  Libcanard interpretation: node_id=%d, first_part=%d\n",
           (test_byte >> 1) & 0x7F, test_byte & 1);
    printf("  Node team interpretation: node_id=%d, first_part=%d\n",
           test_byte & 0x7F, (test_byte >> 7) & 1);

    return 0;
}
