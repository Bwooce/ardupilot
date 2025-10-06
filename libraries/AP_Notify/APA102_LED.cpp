#include "APA102_LED.h"

#if HAL_APA102_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_CANManager/AP_CAN.h>
#if HAL_ENABLE_DRONECAN_DRIVERS
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_DroneCAN/AP_DroneCAN_DNA_Server.h>
#endif

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
        if (num_leds >= 1) led_colors[0] = get_battery_status_color();  // Battery Status
        if (num_leds >= 2) led_colors[1] = get_can_status_color();      // CAN Status
        if (num_leds >= 3) led_colors[2] = get_gps_status_color();      // GPS Status
        if (num_leds >= 4) led_colors[3] = get_armed_status_color();    // Armed Status

        // For additional LEDs, repeat the pattern or show battery status
        for (uint8_t i = 4; i < num_leds; i++) {
            led_colors[i] = get_battery_status_color();
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
        hal.console->printf("APA102: ");

        // [0] Battery status
        if (num_leds >= 1) {
            const AP_BattMonitor &battery = AP::battery();
            uint8_t battery_pct = 0;
            bool has_battery = battery.capacity_remaining_pct(battery_pct);

            float current = 0;
            bool is_charging = false;
            if (battery.current_amps(current)) {
                is_charging = (current < -0.1f);
            }

            if (!has_battery) {
                hal.console->printf("[0]batt=none:off ");
            } else if (is_charging) {
                hal.console->printf("[0]batt=%d%%_chrg:green ", battery_pct);
            } else {
                hal.console->printf("[0]batt=%d%%:", battery_pct);
                // Determine color
                if (battery_pct >= 50) hal.console->printf("green ");
                else if (battery_pct >= 20) hal.console->printf("yellow ");
                else if (battery_pct >= 10) hal.console->printf("orange ");
                else hal.console->printf("red ");
            }
        }

        // [1] CAN status
        if (num_leds >= 2) {
#if HAL_NUM_CAN_IFACES > 0
            const AP_CANManager& can_mgr = AP::can();
            uint8_t total_verified = 0;
            uint8_t total_healthy = 0;
            bool has_dronecan = false;
            bool has_can = false;

            for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
#if HAL_ENABLE_DRONECAN_DRIVERS
                if (can_mgr.get_driver_type(i) == AP_CAN::Protocol::DroneCAN) {
                    has_dronecan = true;
                    has_can = true;
                    AP_DroneCAN* dronecan = static_cast<AP_DroneCAN*>(can_mgr.get_driver(i));
                    if (dronecan != nullptr) {
                        AP_DroneCAN_DNA_Server& dna = dronecan->get_dna_server();
                        total_verified += dna.get_verified_node_count();
                        total_healthy += dna.get_healthy_node_count();
                    }
                } else
#endif
                if (can_mgr.get_driver_type(i) != AP_CAN::Protocol::None) {
                    has_can = true;
                }
            }

            if (!has_can) {
                hal.console->printf("[1]can=off:off ");
            } else if (!has_dronecan) {
                hal.console->printf("[1]can=other:yellow ");
            } else if (total_verified == 0) {
                hal.console->printf("[1]can=no_nodes:red ");
            } else if (total_healthy == 0) {
                hal.console->printf("[1]can=0/%d:red ", total_verified);
            } else if (total_healthy < total_verified) {
                hal.console->printf("[1]can=%d/%d:orange ", total_healthy, total_verified);
            } else {
                hal.console->printf("[1]can=%d/%d:green ", total_healthy, total_verified);
            }
#else
            hal.console->printf("[1]can=disabled:off ");
#endif
        }

        // [2] GPS status
        if (num_leds >= 3) {
            const char* gps_str = "unknown";
            const char* gps_color = "off";
            switch (AP_Notify::flags.gps_status) {
                case 0:
                    gps_str = "no_gps";
                    gps_color = "red";
                    break;
                case 1:
                    gps_str = "no_lock";
                    gps_color = "yellow";
                    break;
                case 2:
                    gps_str = "2d";
                    gps_color = "orange";
                    break;
                case 3:
                    gps_str = "3d";
                    gps_color = "green";
                    break;
                case 4:
                    gps_str = "dgps";
                    gps_color = "green";
                    break;
                case 5:
                    gps_str = "rtk";
                    gps_color = "blue";
                    break;
            }
            hal.console->printf("[2]gps=%s:%s ", gps_str, gps_color);
        }

        // [3] Armed status
        if (num_leds >= 4) {
            if (AP_Notify::flags.armed) {
                if (AP_Notify::flags.flying) {
                    hal.console->printf("[3]armed=flying:red ");
                } else {
                    hal.console->printf("[3]armed=yes:orange ");
                }
            } else {
                if (AP_Notify::flags.pre_arm_check) {
                    hal.console->printf("[3]armed=ready:green ");
                } else {
                    hal.console->printf("[3]armed=not_ready:blue ");
                }
            }
        }

        // Any additional LEDs beyond 4
        for (uint8_t i = 4; i < num_leds; i++) {
            uint8_t r = led_colors[i].r;
            uint8_t g = led_colors[i].g;
            uint8_t b = led_colors[i].b;
            hal.console->printf("[%d]R%d,G%d,B%d ", i, r, g, b);
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

AP_Notify_APA102_LED::LED_Color AP_Notify_APA102_LED::get_battery_status_color()
{
    const AP_BattMonitor &battery = AP::battery();

    // Check all battery instances and use the lowest percentage (most conservative)
    uint8_t lowest_battery_pct = 100;
    bool any_battery_valid = false;
    bool any_charging = false;

    for (uint8_t i = 0; i < battery.num_instances(); i++) {
        uint8_t pct = 0;
        if (battery.capacity_remaining_pct(pct, i)) {
            any_battery_valid = true;
            if (pct < lowest_battery_pct) {
                lowest_battery_pct = pct;
            }
        }

        // Check if any battery is charging (negative current)
        float current = 0;
        if (battery.current_amps(current, i)) {
            if (current < -0.1f) {  // Negative current = charging (with small deadband)
                any_charging = true;
            }
        }
    }

    if (!any_battery_valid) {
        // No battery monitor configured - show off
        return COLOR_OFF;
    }

    uint8_t battery_pct = lowest_battery_pct;

    // Flash green when charging (2Hz = 500ms period, 250ms on/off)
    if (any_charging) {
        static uint32_t last_flash_ms = 0;
        static bool flash_state = false;
        uint32_t now = AP_HAL::millis();
        if (now - last_flash_ms > 250) {
            flash_state = !flash_state;
            last_flash_ms = now;
        }
        if (!flash_state) {
            return COLOR_OFF;
        }
    }

    // Calculate intensity based on battery percentage
    if (battery_pct >= 50) {
        // 100-50%: Green with varying intensity (128 at 100%, 64 at 50%)
        // Map 100-50% to 128-64 brightness
        uint8_t intensity = 64 + ((battery_pct - 50) * 64) / 50;
        return LED_Color(0, intensity, 0);
    } else if (battery_pct >= 20) {
        // 50-20%: Yellow with varying intensity (128 at 50%, 64 at 20%)
        // Map 50-20% to 128-64 brightness
        uint8_t intensity = 64 + ((battery_pct - 20) * 64) / 30;
        return LED_Color(intensity, intensity, 0);
    } else if (battery_pct >= 10) {
        // 20-10%: Orange (warning)
        return COLOR_ORANGE;
    } else {
        // <10%: Red (critical)
        return COLOR_RED;
    }
}

AP_Notify_APA102_LED::LED_Color AP_Notify_APA102_LED::get_can_status_color()
{
#if HAL_NUM_CAN_IFACES > 0
    const AP_CANManager& can_mgr = AP::can();

    // Check all CAN interfaces for DroneCAN status
    bool can_enabled = false;
    uint8_t total_verified_nodes = 0;
    uint8_t total_healthy_nodes = 0;
    bool has_dronecan = false;

    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
#if HAL_ENABLE_DRONECAN_DRIVERS
        if (can_mgr.get_driver_type(i) == AP_CAN::Protocol::DroneCAN) {
            can_enabled = true;
            has_dronecan = true;

            // Get DroneCAN driver and check node health
            AP_DroneCAN* dronecan = static_cast<AP_DroneCAN*>(can_mgr.get_driver(i));
            if (dronecan != nullptr) {
                AP_DroneCAN_DNA_Server& dna = dronecan->get_dna_server();
                total_verified_nodes += dna.get_verified_node_count();
                total_healthy_nodes += dna.get_healthy_node_count();
            }
        } else
#endif
        if (can_mgr.get_driver_type(i) != AP_CAN::Protocol::None) {
            can_enabled = true;
        }
    }

    if (!can_enabled) {
        return COLOR_OFF;  // CAN disabled
    }

    if (!has_dronecan) {
        // Non-DroneCAN protocol enabled - show yellow (bus active but not DroneCAN)
        return COLOR_YELLOW;
    }

    if (total_verified_nodes == 0) {
        // DroneCAN enabled but no nodes verified yet - red
        return COLOR_RED;
    }

    if (total_healthy_nodes == 0) {
        // Nodes exist but none healthy - red
        return COLOR_RED;
    }

    if (total_healthy_nodes < total_verified_nodes) {
        // Some nodes unhealthy - orange
        return COLOR_ORANGE;
    }

    // All verified nodes are healthy - green
    return COLOR_GREEN;
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