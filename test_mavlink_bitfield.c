#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>

// Simulate MAVLink's packed structure with 24-bit bitfield
typedef struct __attribute__((packed)) {
    uint16_t checksum;
    uint8_t magic;
    uint8_t len;
    uint8_t incompat_flags;
    uint8_t compat_flags;
    uint8_t seq;
    uint8_t sysid;
    uint8_t compid;
    uint32_t msgid:24;  // 24-bit bitfield - the suspect
    uint64_t payload64[33];  // MAVLINK_MAX_PAYLOAD_LEN=255 bytes
} test_mavlink_message_t;

// Test functions to see how compiler accesses the bitfield
uint32_t read_msgid(test_mavlink_message_t *msg) {
    return msg->msgid;
}

void write_msgid(test_mavlink_message_t *msg, uint32_t id) {
    msg->msgid = id;
}

// Test with VIBRATION message ID
void test_vibration_id(test_mavlink_message_t *msg) {
    msg->msgid = 241;  // MAVLINK_MSG_ID_VIBRATION
}

// Test reading after writing
uint32_t test_round_trip(test_mavlink_message_t *msg, uint32_t id) {
    msg->msgid = id;
    return msg->msgid;
}

// Test what happens with adjacent memory
void test_with_adjacent_corruption(test_mavlink_message_t *msg) {
    // Fill structure with pattern
    memset(msg, 0xAA, sizeof(*msg));
    
    // Set known values
    msg->magic = 0xFD;
    msg->len = 32;
    msg->sysid = 1;
    msg->compid = 1;
    msg->msgid = 241;  // Should be 0x0000F1
    
    printf("After setting msgid=241:\n");
    printf("  msgid field = 0x%06X\n", msg->msgid);
    printf("  Raw bytes at msgid offset: ");
    
    // Show raw bytes where msgid is stored (after compid)
    uint8_t *ptr = (uint8_t*)msg;
    int msgid_offset = 9;  // After 2+1+1+1+1+1+1+1 = 9 bytes
    for (int i = 0; i < 4; i++) {
        printf("%02X ", ptr[msgid_offset + i]);
    }
    printf("\n");
}

int main() {
    test_mavlink_message_t msg;
    
    printf("Structure info:\n");
    printf("  sizeof(msg) = %lu\n", sizeof(msg));
    printf("  offsetof(compid) = %lu (msgid follows this)\n", offsetof(test_mavlink_message_t, compid));
    printf("  alignof(msg) = %lu\n", __alignof__(msg));
    
    // Test basic read/write
    printf("\nBasic tests:\n");
    write_msgid(&msg, 241);
    printf("  Written 241, read back: %u (0x%06X)\n", read_msgid(&msg), read_msgid(&msg));
    
    // Test with potential corruption value
    write_msgid(&msg, 0x2F7C00);
    printf("  Written 0x2F7C00, read back: 0x%06X\n", read_msgid(&msg));
    
    // Test with adjacent memory pattern
    printf("\nAdjacent memory test:\n");
    test_with_adjacent_corruption(&msg);
    
    return 0;
}