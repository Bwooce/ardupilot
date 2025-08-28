#pragma once

#include "NotifyDevice.h"
#include <AP_HAL/AP_HAL.h>

/**
 * Enhanced APA102 LED driver supporting multiple individual LEDs
 * Can operate in two modes:
 * 1. Single RGB mode (backward compatible) - all LEDs show same color
 * 2. Multi-LED mode - each LED can show different status information
 */
class AP_Notify_APA102_LED : public NotifyDevice
{
public:
    AP_Notify_APA102_LED();

    bool init() override;
    void update() override;

private:
    struct LED_Color {
        uint8_t r, g, b;
        LED_Color(uint8_t red = 0, uint8_t green = 0, uint8_t blue = 0) : r(red), g(green), b(blue) {}
        bool operator!=(const LED_Color& other) const {
            return r != other.r || g != other.g || b != other.b;
        }
    };

    void send_led_data(uint8_t r, uint8_t g, uint8_t b);
    void send_start_frame();
    void send_end_frame();
    void update_leds();
    
    // Status determination functions for multi-LED mode
    LED_Color get_system_status_color();
    LED_Color get_can_status_color();
    LED_Color get_gps_status_color();
    LED_Color get_armed_status_color();
    LED_Color get_single_led_color(); // For single RGB mode
    
    // LED mode determination
    bool use_multi_led_mode() const;

    AP_HAL::DigitalSource *data_pin;
    AP_HAL::DigitalSource *clock_pin;
    
    // LED configuration
    uint8_t num_leds;
    LED_Color *led_colors;
    LED_Color *last_led_colors;
    
    uint32_t last_update_ms;
    bool initialized;
    
    // Color definitions (const instead of constexpr for C++ compatibility)
    static const LED_Color COLOR_OFF;
    static const LED_Color COLOR_RED;
    static const LED_Color COLOR_GREEN;
    static const LED_Color COLOR_BLUE;
    static const LED_Color COLOR_YELLOW;
    static const LED_Color COLOR_ORANGE;
    static const LED_Color COLOR_PURPLE;
    static const LED_Color COLOR_WHITE;
};
