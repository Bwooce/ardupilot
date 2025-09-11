// Test to check bool size on ESP32 with xtensa compiler
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

int main() {
    printf("ESP32 type sizes:\n");
    printf("sizeof(bool) = %zu bytes\n", sizeof(bool));
    printf("sizeof(uint8_t) = %zu bytes\n", sizeof(uint8_t));
    printf("sizeof(int) = %zu bytes\n", sizeof(int));
    
    // Test bool values
    bool test_true = true;
    bool test_false = false;
    
    printf("\nBool value test:\n");
    printf("true = %d (0x%02X)\n", test_true, test_true);
    printf("false = %d (0x%02X)\n", test_false, test_false);
    
    // Check memory representation
    printf("\nMemory representation:\n");
    uint8_t* ptr_true = (uint8_t*)&test_true;
    uint8_t* ptr_false = (uint8_t*)&test_false;
    
    printf("&test_true bytes: ");
    for (size_t i = 0; i < sizeof(bool); i++) {
        printf("0x%02X ", ptr_true[i]);
    }
    printf("\n");
    
    printf("&test_false bytes: ");
    for (size_t i = 0; i < sizeof(bool); i++) {
        printf("0x%02X ", ptr_false[i]);
    }
    printf("\n");
    
    return 0;
}