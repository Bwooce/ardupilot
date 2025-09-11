// Test to verify stack protection is enabled
// This should trigger a stack canary error if protection is working

#include <stdio.h>
#include <string.h>

__attribute__((noinline))
void trigger_stack_overflow() {
    char buffer[8];
    // Intentionally overflow the buffer
    // This should trigger stack protection if enabled
    memset(buffer, 'A', 32);  // Write 32 bytes to 8-byte buffer
    printf("If you see this, stack protection might not be working: %s\n", buffer);
}

void test_stack_protection() {
    printf("Testing stack protection...\n");
    trigger_stack_overflow();
    printf("Stack protection test completed (should not reach here if working)\n");
}

// To use in ArduPilot:
// Add this to your test code and call test_stack_protection()
// If stack protection is working, you should see:
// "Stack smashing protect failure!" or
// "Debug exception reason: Stack canary watchpoint triggered"