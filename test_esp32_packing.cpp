// Test program to verify struct packing on ESP32
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>

// Simulate __XTENSA__ for testing
#define __XTENSA__ 1

// Test both packed and unpacked versions
struct UnpackedTransfer {
    uint32_t transfer_type;
    uint64_t data_type_signature;
    uint16_t data_type_id;
    uint8_t* inout_transfer_id;
    uint8_t priority;
    const void* payload;
    uint32_t payload_len;
    uint8_t iface_mask;
    bool canfd;
    uint32_t timeout_ms;
};

struct __attribute__((packed)) PackedTransfer {
    uint32_t transfer_type;
    uint64_t data_type_signature;
    uint16_t data_type_id;
    uint8_t* inout_transfer_id;
    uint8_t priority;
    const void* payload;
    uint32_t payload_len;
    uint8_t iface_mask;
    bool canfd;
    uint32_t timeout_ms;
};

int main() {
    printf("=== ESP32 Struct Packing Test ===\n\n");
    
    printf("Unpacked struct:\n");
    printf("  sizeof(UnpackedTransfer) = %zu bytes\n", sizeof(UnpackedTransfer));
    printf("  alignof(UnpackedTransfer) = %zu bytes\n", alignof(UnpackedTransfer));
    printf("  priority offset = %zu\n", offsetof(UnpackedTransfer, priority));
    printf("  payload offset = %zu\n", offsetof(UnpackedTransfer, payload));
    printf("  payload_len offset = %zu\n", offsetof(UnpackedTransfer, payload_len));
    printf("\n");
    
    printf("Packed struct:\n");
    printf("  sizeof(PackedTransfer) = %zu bytes\n", sizeof(PackedTransfer));
    printf("  alignof(PackedTransfer) = %zu bytes\n", alignof(PackedTransfer));
    printf("  priority offset = %zu\n", offsetof(PackedTransfer, priority));
    printf("  payload offset = %zu\n", offsetof(PackedTransfer, payload));
    printf("  payload_len offset = %zu\n", offsetof(PackedTransfer, payload_len));
    printf("\n");
    
    // Check if packing actually removes padding
    size_t unpacked_expected_priority = offsetof(UnpackedTransfer, inout_transfer_id) + sizeof(uint8_t*);
    size_t packed_expected_priority = offsetof(PackedTransfer, inout_transfer_id) + sizeof(uint8_t*);
    
    printf("Analysis:\n");
    printf("  Unpacked: Expected priority at %zu, actual at %zu (padding = %zu bytes)\n",
           unpacked_expected_priority, offsetof(UnpackedTransfer, priority),
           offsetof(UnpackedTransfer, priority) - unpacked_expected_priority);
    printf("  Packed: Expected priority at %zu, actual at %zu (padding = %zu bytes)\n",
           packed_expected_priority, offsetof(PackedTransfer, priority),
           offsetof(PackedTransfer, priority) - packed_expected_priority);
    printf("\n");
    
    if (sizeof(PackedTransfer) < sizeof(UnpackedTransfer)) {
        printf("SUCCESS: Packing reduced struct size by %zu bytes\n",
               sizeof(UnpackedTransfer) - sizeof(PackedTransfer));
    } else if (sizeof(PackedTransfer) == sizeof(UnpackedTransfer)) {
        printf("INFO: Packing had no effect on size (may already be optimal)\n");
    } else {
        printf("ERROR: Packed struct is larger than unpacked!\n");
    }
    
    return 0;
}