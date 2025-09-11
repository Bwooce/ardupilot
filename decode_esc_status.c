#include <stdio.h>
#include <stdint.h>
#include <string.h>

// Convert float16 to float32 (half precision to single precision)
float float16_to_float32(uint16_t h) {
    uint32_t sign = (h & 0x8000) << 16;
    uint32_t exponent = (h & 0x7C00) >> 10;
    uint32_t mantissa = h & 0x03FF;
    
    if (exponent == 0) {
        // Subnormal or zero
        if (mantissa == 0) return 0.0f;
        // Subnormal - normalize it
        exponent = 1;
        while ((mantissa & 0x0400) == 0) {
            mantissa <<= 1;
            exponent--;
        }
        mantissa &= 0x03FF;
    } else if (exponent == 31) {
        // Infinity or NaN
        exponent = 255;
    }
    
    // Convert to float32 format
    exponent = exponent - 15 + 127;
    mantissa = mantissa << 13;
    
    union {
        uint32_t i;
        float f;
    } result;
    result.i = sign | (exponent << 23) | mantissa;
    return result.f;
}

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
    
    printf("CAN ID 0x%08X breakdown:\n", can_id);
    printf("  Priority: %d\n", priority);
    printf("  Message Type: %d (0x%04X)", message_type, message_type);
    if (message_type == 341) printf(" - NodeStatus");
    else if (message_type == 1034) printf(" - ESC Status");
    printf("\n");
    printf("  Service Flag: %d\n", service_flag);
    printf("  Source Node ID: %d (0x%02X)\n", source_node_id, source_node_id);
}

void decode_esc_status(uint8_t* data, int len) {
    printf("ESC Status decode (need 14 bytes, have %d):\n", len);
    
    if (len < 14) {
        printf("  ERROR: Insufficient data for ESC Status\n");
        // Try to decode what we have
        if (len >= 4) {
            uint32_t error_count = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
            printf("  Error count: %u\n", error_count);
        }
        if (len >= 6) {
            uint16_t voltage_raw = data[4] | (data[5] << 8);
            float voltage = float16_to_float32(voltage_raw);
            printf("  Voltage (float16): 0x%04X = %.2f V\n", voltage_raw, voltage);
        }
        if (len >= 8) {
            uint16_t current_raw = data[6] | (data[7] << 8);
            float current = float16_to_float32(current_raw);
            printf("  Current (float16): 0x%04X = %.2f A\n", current_raw, current);
        }
        return;
    }
    
    // Full decode
    uint32_t error_count = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
    uint16_t voltage_raw = data[4] | (data[5] << 8);
    uint16_t current_raw = data[6] | (data[7] << 8);
    uint16_t temperature_raw = data[8] | (data[9] << 8);
    int32_t rpm = (int32_t)(data[10] | (data[11] << 8) | (data[12] << 16) | (data[13] << 24));
    
    float voltage = float16_to_float32(voltage_raw);
    float current = float16_to_float32(current_raw);
    float temperature = float16_to_float32(temperature_raw);
    
    printf("  Error count: %u\n", error_count);
    printf("  Voltage: %.2f V (raw: 0x%04X)\n", voltage, voltage_raw);
    printf("  Current: %.2f A (raw: 0x%04X)\n", current, current_raw);
    printf("  Temperature: %.2f C (raw: 0x%04X)\n", temperature, temperature_raw);
    printf("  RPM: %d\n", rpm);
    
    if (len > 14) {
        printf("  Power rating: %u%%\n", data[14]);
    }
    if (len > 15) {
        printf("  ESC index: %u\n", data[15]);
    }
}

void analyze_tail_byte(uint8_t tail) {
    // Tail byte format:
    // bits 0-1: SOT/EOT (start=1, end=2, single=3, middle=0)
    // bit 2: toggle bit
    // bits 3-7: transfer ID
    
    int sot_eot = tail & 0x03;
    int toggle = (tail >> 2) & 0x01;
    int transfer_id = (tail >> 3) & 0x1F;
    
    printf("Tail byte 0x%02X:\n", tail);
    printf("  Transfer type: ");
    switch(sot_eot) {
        case 0: printf("Middle frame"); break;
        case 1: printf("Start of transfer"); break;
        case 2: printf("End of transfer"); break;
        case 3: printf("Single frame transfer"); break;
    }
    printf("\n");
    printf("  Toggle bit: %d\n", toggle);
    printf("  Transfer ID: %d\n", transfer_id);
}

int main() {
    printf("=== Decoding ESC Status messages from node 122 ===\n\n");
    
    // Frame 1: ID=0x10040A7A, len=8, data: 52 00 00 A9 5C 00 00 36
    printf("Frame 1: ID=0x10040A7A, len=8\n");
    decode_can_id(0x10040A7A);
    uint8_t frame1[] = {0x52, 0x00, 0x00, 0xA9, 0x5C, 0x00, 0x00, 0x36};
    printf("Data:");
    for (int i = 0; i < 8; i++) printf(" %02X", frame1[i]);
    printf("\n");
    
    analyze_tail_byte(frame1[7]);
    
    // This is "end of transfer" so it's the end of a multi-frame message
    // The payload is 7 bytes (excluding tail)
    printf("\nPayload (7 bytes): ");
    for (int i = 0; i < 7; i++) printf(" %02X", frame1[i]);
    printf("\n");
    
    printf("\n");
    
    // Frame 2: ID=0x10040A7A, len=2, data: 00 56
    printf("Frame 2: ID=0x10040A7A, len=2\n");
    decode_can_id(0x10040A7A);
    uint8_t frame2[] = {0x00, 0x56};
    printf("Data:");
    for (int i = 0; i < 2; i++) printf(" %02X", frame2[i]);
    printf("\n");
    
    analyze_tail_byte(frame2[1]);
    
    printf("\nPayload (1 byte): %02X\n", frame2[0]);
    
    printf("\n=== Analysis ===\n");
    printf("Both frames are ESC Status messages (ID 1034) from node 122.\n");
    printf("Frame 1: End of multi-frame transfer #6 with toggle=1\n");
    printf("Frame 2: End of multi-frame transfer #10 with toggle=1\n");
    printf("\nThese appear to be incomplete multi-frame transfers.\n");
    printf("We're only seeing the END frames, missing the START and any MIDDLE frames.\n");
    printf("This explains the BAD_CRC and WRONG_TOGGLE errors - we don't have complete transfers.\n");
    
    printf("\n=== Attempting partial decode of Frame 1 payload ===\n");
    printf("If this were the last 7 bytes of an ESC Status message:\n");
    // ESC Status is 14+ bytes, so these would be bytes 7-13
    // That would be: current(2), temperature(2), rpm(4) minus 1 byte
    uint16_t possible_current = frame1[0] | (frame1[1] << 8);
    uint16_t possible_temp = frame1[2] | (frame1[3] << 8);
    
    printf("  Possible current (if bytes 6-7): %.2f A (raw: 0x%04X)\n", 
           float16_to_float32(possible_current), possible_current);
    printf("  Possible temperature (if bytes 8-9): %.2f C (raw: 0x%04X)\n",
           float16_to_float32(possible_temp), possible_temp);
    
    // But 0x0052 as float16 = very small value, 0xA900 has wrong bit pattern
    // Let's try different interpretation
    
    printf("\nOr if Frame 1 contains error_count + voltage + current (first 8 bytes):\n");
    uint32_t error_count = frame1[0] | (frame1[1] << 8) | (frame1[2] << 16) | (frame1[3] << 24);
    uint16_t voltage = frame1[4] | (frame1[5] << 8);
    // Would need 2 more bytes for current
    
    printf("  Error count: %u (0x%08X)\n", error_count, error_count);
    printf("  Voltage: %.2f V (raw: 0x%04X)\n", float16_to_float32(voltage), voltage);
    
    return 0;
}