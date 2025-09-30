#include "APA102_LED.h"

#if HAL_APA102_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_CANManager/AP_CAN.h>

extern const AP_HAL::HAL& hal;

// LED color definitions
const AP_Notify_APA102_LED::LED_Color AP_Notify_APA102_LED::COLOR_OFF(0, 0, 0);
const AP_Notify_APA102_LED::LED_Color AP_Notify_APA102_LED::COLOR_RED(128, 0, 0);
const AP_Notify_APA102_LED::LED_Color AP_Notify_APA102_LED::COLOR_GREEN(0, 128, 0);
const AP_Notify_APA102_LED::LED_Color AP_Notify_APA102_LED::COLOR_BLUE(0, 0, 128);
const AP_Notify_APA102_LED::LED_Color AP_Notify_APA102_LED::COLOR_YELLOW(128, 128, 0);
const AP_Notify_APA102_LED::LED_Color AP_Notify_APA102_LED::COLOR_ORANGE(128, 82, 0);
const AP_Notify_APA102_LED::LED_Color AP_Notify_APA102_LED::COLOR_PURPLE(64, 0, 64);
const AP_Notify_APA102_LED::LED_Color AP_Notify_APA102_LED::COLOR_WHITE(128, 128, 128);

AP_Notify_APA102_LED::AP_Notify_APA102_LED() :
    data_pin(nullptr),
    clock_pin(nullptr),
    num_leds(1),
    led_colors(nullptr),
    last_led_colors(nullptr),
    last_update_ms(0),
    initialized(false)
{
}

bool AP_Notify_APA102_LED::init()
{
    data_pin = hal.gpio->channel(HAL_APA102_DATA_PIN);
    clock_pin = hal.gpio->channel(HAL_APA102_CLOCK_PIN);

    if (data_pin == nullptr || clock_pin == nullptr) {
        return false;
    }

    data_pin->mode(HAL_GPIO_OUTPUT);
    clock_pin->mode(HAL_GPIO_OUTPUT);

    // Get number of LEDs from configuration (default to 4 for backward compatibility)
#ifdef HAL_APA102_NUM_LEDS
    num_leds = HAL_APA102_NUM_LEDS;
#else
    num_leds = 4; // Default for backward compatibility
#endif

    // Allocate memory for LED colors
    led_colors = new(std::nothrow) LED_Color[num_leds];
    last_led_colors = new(std::nothrow) LED_Color[num_leds];
    
    if (led_colors == nullptr || last_led_colors == nullptr) {
        return false;
    }

    // Initialize all LEDs to off first
    for (uint8_t i = 0; i < num_leds; i++) {
        led_colors[i] = COLOR_OFF;
        last_led_colors[i] = COLOR_OFF;
    }

    // Send all LEDs off to ensure clean state
    update_leds();

    hal.console->printf("APA102: Initialized %d LEDs\n", num_leds);

    // Debug initial notify flag states
    hal.console->printf("APA102: Initial flags - init:%d prearm:%d gyro:%d gps_st:%d armed:%d\n",
                        AP_Notify::flags.initialising,
                        AP_Notify::flags.pre_arm_check,
                        AP_Notify::flags.gyro_calibrated,
                        AP_Notify::flags.gps_status,
                        AP_Notify::flags.armed);

    initialized = true;
    return true;
}

void AP_Notify_APA102_LED::update()
{
    if (!initialized) {
        return;
    }

    uint32_t now = AP_HAL::millis();
    if (now - last_update_ms < 100) { // Update at 10Hz
        return;
    }
    last_update_ms = now;

    if (use_multi_led_mode()) {
        // Multi-LED mode: each LED shows different status
        if (num_leds >= 1) led_colors[0] = get_system_status_color();  // System Status
        if (num_leds >= 2) led_colors[1] = get_can_status_color();     // CAN Status
        if (num_leds >= 3) led_colors[2] = get_gps_status_color();     // GPS Status
        if (num_leds >= 4) led_colors[3] = get_armed_status_color();   // Armed Status
        
        // For additional LEDs, repeat the pattern or show system status
        for (uint8_t i = 4; i < num_leds; i++) {
            led_colors[i] = get_system_status_color();
        }
    } else {
        // Single RGB mode: all LEDs show same color (backward compatibility)
        LED_Color single_color = get_single_led_color();
        for (uint8_t i = 0; i < num_leds; i++) {
            led_colors[i] = single_color;
        }
    }

    // Check if any LEDs need updating
    bool needs_update = false;
    for (uint8_t i = 0; i < num_leds; i++) {
        if (led_colors[i] != last_led_colors[i]) {
            needs_update = true;
            break;
        }
    }

    // Debug output on changes or every 10 minutes
    static uint32_t debug_counter = 0;
    debug_counter++;

    // Log immediately on change, or every 600 seconds (6000 * 100ms)
    if (needs_update || (debug_counter % 6000) == 0) {
        hal.console->printf("APA102: Status - ");
        hal.console->printf("init:%d prearm:%d gps:%d armed:%d | Colors: ",
                            AP_Notify::flags.initialising,
                            AP_Notify::flags.pre_arm_check,
                            AP_Notify::flags.gps_status,
                            AP_Notify::flags.armed);
        for (uint8_t i = 0; i < num_leds; i++) {
            hal.console->printf("[%d]R%d,G%d,B%d ", i, led_colors[i].r, led_colors[i].g, led_colors[i].b);
        }
        hal.console->printf("%s\n", needs_update ? "(changed)" : "(periodic)");
    }

    if (needs_update) {
        update_leds();
        for (uint8_t i = 0; i < num_leds; i++) {
            last_led_colors[i] = led_colors[i];
        }
    }
}

bool AP_Notify_APA102_LED::use_multi_led_mode() const
{
    // Use multi-LED mode if we have more than 1 LED
    return num_leds > 1;
}

void AP_Notify_APA102_LED::send_led_data(uint8_t r, uint8_t g, uint8_t b)
{
    // APA102 requires a start byte of 0b11100000 + 5 bits of brightness
    // Reduced from 31 (max) to 4 (much dimmer) for comfortable viewing
    uint8_t brightness = 0b00000100; // ~13% brightness
    uint8_t header = 0b11100000 | brightness;

    // Send header
    for (int8_t i=7; i>=0; i--) {
        clock_pin->write(0);
        hal.scheduler->delay_microseconds(1);
        data_pin->write((header >> i) & 1);
        hal.scheduler->delay_microseconds(1);
        clock_pin->write(1);
        hal.scheduler->delay_microseconds(1);
    }

    // Send blue
    for (int8_t i=7; i>=0; i--) {
        clock_pin->write(0);
        hal.scheduler->delay_microseconds(1);
        data_pin->write((b >> i) & 1);
        hal.scheduler->delay_microseconds(1);
        clock_pin->write(1);
        hal.scheduler->delay_microseconds(1);
    }

    // Send green
    for (int8_t i=7; i>=0; i--) {
        clock_pin->write(0);
        hal.scheduler->delay_microseconds(1);
        data_pin->write((g >> i) & 1);
        hal.scheduler->delay_microseconds(1);
        clock_pin->write(1);
        hal.scheduler->delay_microseconds(1);
    }

    // Send red
    for (int8_t i=7; i>=0; i--) {
        clock_pin->write(0);
        hal.scheduler->delay_microseconds(1);
        data_pin->write((r >> i) & 1);
        hal.scheduler->delay_microseconds(1);
        clock_pin->write(1);
        hal.scheduler->delay_microseconds(1);
    }
}

void AP_Notify_APA102_LED::send_start_frame()
{
    for (uint8_t i=0; i<32; i++) {
        clock_pin->write(0);
        data_pin->write(0);
        clock_pin->write(1);
    }
}

void AP_Notify_APA102_LED::send_end_frame()
{
    for (uint8_t i=0; i<32; i++) {
        clock_pin->write(0);
        data_pin->write(1);
        clock_pin->write(1);
    }
}

void AP_Notify_APA102_LED::update_leds()
{
    send_start_frame();
    for (uint8_t i = 0; i < num_leds; i++) {
        send_led_data(led_colors[i].r, led_colors[i].g, led_colors[i].b);
    }
    send_end_frame();
}

AP_Notify_APA102_LED::LED_Color AP_Notify_APA102_LED::get_system_status_color()
{
    // Priority order: Errors > Warnings > Normal

    // Critical errors - Red
    if (AP_Notify::flags.failsafe_radio ||
        AP_Notify::flags.failsafe_battery ||
        AP_Notify::flags.failsafe_gcs ||
        AP_Notify::flags.failsafe_ekf ||
        AP_Notify::flags.ekf_bad) {
        return COLOR_RED;
    }

    // Warnings - Yellow/Orange
    if (!AP_Notify::flags.pre_arm_check ||
        !AP_Notify::flags.gyro_calibrated ||
        AP_Notify::flags.gps_glitching) {
        return COLOR_YELLOW;
    }

    // Initializing - Blue
    if (AP_Notify::flags.initialising) {
        return COLOR_BLUE;
    }

    // Normal operation - Green
    return COLOR_GREEN;
}

AP_Notify_APA102_LED::LED_Color AP_Notify_APA102_LED::get_can_status_color()
{
#if HAL_NUM_CAN_IFACES > 0
    const AP_CANManager& can_mgr = AP::can();
    
    // Check if any CAN interface is enabled and has a driver
    bool can_enabled = false;
    bool can_healthy = false;
    
    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        if (can_mgr.get_driver_type(i) != AP_CAN::Protocol::None) {
            can_enabled = true;
            // Note: CAN health checking would need additional API
            // For now, assume healthy if enabled
            can_healthy = true;
            break;
        }
    }
    
    if (!can_enabled) {
        return COLOR_OFF;  // CAN disabled
    }
    
    if (!can_healthy) {
        return COLOR_RED;  // CAN bus-off or errors
    }
    
    return COLOR_GREEN;  // CAN active and healthy
#else
    return COLOR_OFF;  // No CAN support
#endif
}

AP_Notify_APA102_LED::LED_Color AP_Notify_APA102_LED::get_gps_status_color()
{
    switch (AP_Notify::flags.gps_status) {
        case 0:  // No GPS
            return COLOR_RED;
        case 1:  // No lock
            return COLOR_YELLOW;
        case 2:  // 2D lock
            return COLOR_ORANGE;
        case 3:  // 3D lock
            return COLOR_GREEN;
        case 4:  // DGPS lock
            return COLOR_GREEN;
        case 5:  // RTK lock
            return COLOR_BLUE;
        default:
            return COLOR_OFF;
    }
}

AP_Notify_APA102_LED::LED_Color AP_Notify_APA102_LED::get_armed_status_color()
{
    if (AP_Notify::flags.armed) {
        if (AP_Notify::flags.flying) {
            return COLOR_RED;     // Armed and flying - bright red
        } else {
            return COLOR_ORANGE;  // Armed but not flying - orange
        }
    } else {
        if (AP_Notify::flags.pre_arm_check) {
            return COLOR_GREEN;   // Disarmed and pre-arm passed - green
        } else {
            return COLOR_BLUE;    // Disarmed with pre-arm issues - blue
        }
    }
}

AP_Notify_APA102_LED::LED_Color AP_Notify_APA102_LED::get_single_led_color()
{
    // Single LED mode - use traditional RGB LED logic
    
    // Armed = Red
    if (AP_Notify::flags.armed) {
        return COLOR_RED;
    }
    
    // GPS lock = Green  
    if (AP_Notify::flags.gps_status >= 3) {
        return COLOR_GREEN;
    }
    
    // GPS acquiring = Yellow
    if (AP_Notify::flags.gps_status >= 1) {
        return COLOR_YELLOW;
    }
    
    // Initializing = Blue
    if (AP_Notify::flags.initialising) {
        return COLOR_BLUE;
    }
    
    // Default = White
    return COLOR_WHITE;
}

#endif // HAL_APA102_ENABLED