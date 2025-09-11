#include <stdio.h>
#include <stdint.h>

int main() {
    // The invalid IDs from the log
    uint32_t invalid_id1 = 0x8803F20A;
    uint32_t invalid_id2 = 0x9F01560A;
    
    // FlagEFF is bit 31
    uint32_t FlagEFF = 1U << 31;  // 0x80000000
    
    printf("=== Analyzing CAN ID Construction Issue ===\n\n");
    
    printf("Invalid ID 1: 0x%08X\n", invalid_id1);
    printf("  Has bit 31 set: %s\n", (invalid_id1 & FlagEFF) ? "YES" : "NO");
    printf("  Without bit 31: 0x%08X\n", invalid_id1 & 0x7FFFFFFF);
    
    // What if txf->id already had FlagEFF and we OR'd it again?
    // That wouldn't change anything since it's already set
    
    // But what if there's a sign extension issue?
    // If txf->id is treated as signed somewhere...
    int32_t signed_id = (int32_t)(invalid_id1 & 0x1FFFFFFF);
    printf("  As signed 29-bit then extended: 0x%08X\n", (uint32_t)signed_id);
    
    printf("\nInvalid ID 2: 0x%08X\n", invalid_id2);
    printf("  Has bit 31 set: %s\n", (invalid_id2 & FlagEFF) ? "YES" : "NO");
    printf("  Without bit 31: 0x%08X\n", invalid_id2 & 0x7FFFFFFF);
    
    // Let's check if these could be from priority field overflow
    // Priority should be 5 bits (0-31), but what if it's larger?
    uint32_t priority1 = (invalid_id1 >> 24) & 0xFF;  // Get 8 bits instead of 5
    uint32_t priority2 = (invalid_id2 >> 24) & 0xFF;
    
    printf("\nExtended priority field (8 bits instead of 5):\n");
    printf("  ID1 priority: 0x%02X (%d) - bit 7=%d, bit 6=%d, bit 5=%d\n", 
           priority1, priority1, 
           (priority1 >> 7) & 1, (priority1 >> 6) & 1, (priority1 >> 5) & 1);
    printf("  ID2 priority: 0x%02X (%d) - bit 7=%d, bit 6=%d, bit 5=%d\n", 
           priority2, priority2,
           (priority2 >> 7) & 1, (priority2 >> 6) & 1, (priority2 >> 5) & 1);
    
    // What would these IDs be if constructed with bad priority?
    // DroneCAN ID format: Priority(5) | MsgType(16) | Service(1) | NodeID(7)
    
    printf("\n=== Hypothesis: Priority field corruption ===\n");
    printf("If priority accidentally had bit 7 set (value >= 128):\n");
    
    // Reconstruct what the original might have been
    uint32_t msg_type1 = (invalid_id1 >> 8) & 0xFFFF;
    uint32_t node_id1 = invalid_id1 & 0x7F;
    printf("  ID1: MsgType=0x%04X (%d), NodeID=%d\n", msg_type1, msg_type1, node_id1);
    
    // If priority was supposed to be 8 but got corrupted to 0x88
    uint32_t correct_id1 = (8 << 24) | (msg_type1 << 8) | node_id1;
    printf("  Correct ID1 would be: 0x%08X\n", correct_id1);
    
    uint32_t msg_type2 = (invalid_id2 >> 8) & 0xFFFF;
    uint32_t node_id2 = invalid_id2 & 0x7F;
    printf("  ID2: MsgType=0x%04X (%d), NodeID=%d\n", msg_type2, msg_type2, node_id2);
    
    // If priority was supposed to be 31 but got corrupted to 0x9F
    uint32_t correct_id2 = (31 << 24) | (msg_type2 << 8) | node_id2;
    printf("  Correct ID2 would be: 0x%08X\n", correct_id2);
    
    return 0;
}