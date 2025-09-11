// Test program to verify Canard::Transfer struct packing on ESP32
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>

// Simulate the CanardTransferType enum
typedef enum {
    CanardTransferTypeBroadcast = 0,
    CanardTransferTypeRequest = 1,
    CanardTransferTypeResponse = 2
} CanardTransferType;

// The Canard::Transfer structure from interface.h
struct Transfer {
    CanardTransferType transfer_type; ///< Type of transfer
    uint64_t data_type_signature; ///< Signature of the message/service
    uint16_t data_type_id; ///< ID of the message/service
    uint8_t* inout_transfer_id; ///< Transfer ID reference
    uint8_t priority; ///< Priority of the transfer
    const void* payload; ///< Pointer to the payload
    uint32_t payload_len; ///< Length of the payload
    uint8_t iface_mask; ///< Bitmask of interfaces
    bool canfd; ///< true if CAN FD
    uint32_t timeout_ms; ///< timeout in ms
};

// The CanardTxTransfer structure from canard.h
typedef struct {
    CanardTransferType transfer_type;
    uint64_t data_type_signature;
    uint16_t data_type_id;
    uint8_t* inout_transfer_id;
    uint8_t priority;
    const uint8_t* payload;
    uint16_t payload_len;  // Note: This is uint16_t in CanardTxTransfer!
#if CANARD_ENABLE_CANFD
    bool canfd;
#endif
#if CANARD_ENABLE_DEADLINE
    uint64_t deadline_usec;
#endif
#if CANARD_MULTI_IFACE
    uint8_t iface_mask;
#endif
} CanardTxTransfer;

int main() {
    printf("=== Struct Packing Analysis for Canard Transfers ===\n\n");
    
    printf("Canard::Transfer struct:\n");
    printf("  sizeof(Transfer) = %zu bytes\n", sizeof(Transfer));
    printf("  alignof(Transfer) = %zu bytes\n", alignof(Transfer));
    printf("\n");
    
    printf("Field offsets in Canard::Transfer:\n");
    printf("  transfer_type:        offset %zu, size %zu\n", offsetof(Transfer, transfer_type), sizeof(CanardTransferType));
    printf("  data_type_signature:  offset %zu, size %zu\n", offsetof(Transfer, data_type_signature), sizeof(uint64_t));
    printf("  data_type_id:         offset %zu, size %zu\n", offsetof(Transfer, data_type_id), sizeof(uint16_t));
    printf("  inout_transfer_id:    offset %zu, size %zu (pointer)\n", offsetof(Transfer, inout_transfer_id), sizeof(uint8_t*));
    printf("  priority:             offset %zu, size %zu\n", offsetof(Transfer, priority), sizeof(uint8_t));
    printf("  payload:              offset %zu, size %zu (pointer)\n", offsetof(Transfer, payload), sizeof(void*));
    printf("  payload_len:          offset %zu, size %zu\n", offsetof(Transfer, payload_len), sizeof(uint32_t));
    printf("  iface_mask:           offset %zu, size %zu\n", offsetof(Transfer, iface_mask), sizeof(uint8_t));
    printf("  canfd:                offset %zu, size %zu\n", offsetof(Transfer, canfd), sizeof(bool));
    printf("  timeout_ms:           offset %zu, size %zu\n", offsetof(Transfer, timeout_ms), sizeof(uint32_t));
    printf("\n");
    
    // Check for potential alignment issues
    size_t expected_priority_offset = offsetof(Transfer, inout_transfer_id) + sizeof(uint8_t*);
    size_t actual_priority_offset = offsetof(Transfer, priority);
    
    if (expected_priority_offset != actual_priority_offset) {
        printf("WARNING: Padding detected before priority field!\n");
        printf("  Expected offset: %zu\n", expected_priority_offset);
        printf("  Actual offset:   %zu\n", actual_priority_offset);
        printf("  Padding bytes:   %zu\n", actual_priority_offset - expected_priority_offset);
    } else {
        printf("No padding before priority field - layout looks correct.\n");
    }
    
    printf("\n");
    printf("CanardTxTransfer struct:\n");
    printf("  sizeof(CanardTxTransfer) = %zu bytes\n", sizeof(CanardTxTransfer));
    printf("  alignof(CanardTxTransfer) = %zu bytes\n", alignof(CanardTxTransfer));
    printf("\n");
    
    printf("Key difference: Canard::Transfer has payload_len as uint32_t,\n");
    printf("                CanardTxTransfer has payload_len as uint16_t!\n");
    printf("\nThis mismatch could cause corruption when copying between structures.\n");
    
    return 0;
}