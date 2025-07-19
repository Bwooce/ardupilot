#include "APA102_LED.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

AP_Notify_APA102_LED::AP_Notify_APA102_LED()
{
}

void AP_Notify_APA102_LED::init()
{
    data_pin = hal.gpio->channel(hal.board_config->get_param("APA102_DATA"));
    clock_pin = hal.gpio->channel(hal.board_config->get_param("APA102_CLOCK"));

    data_pin->mode(HAL_GPIO_OUTPUT);
    clock_pin->mode(HAL_GPIO_OUTPUT);
}

void AP_Notify_APA102_LED::update()
{
    uint8_t r[4] = {0};
    uint8_t g[4] = {0};
    uint8_t b[4] = {0};

    // LED 1: System / Arming
    if (AP_Notify::flags.failsafe_radio || AP_Notify::flags.failsafe_battery || AP_Notify::flags.failsafe_gcs || AP_Notify::flags.failsafe_ekf) {
        b[0] = 255; // Pulsing Blue for Failsafe
    } else if (AP_Notify::flags.armed) {
        g[0] = 255; // Solid Green for Armed
    } else if (AP_Notify::flags.pre_arm_check) {
        r[0] = 255; g[0] = 255; // Pulsing Yellow for Ready to Arm
    } else {
        r[0] = 255; // Solid Red for Pre-arm Checks Failed
    }

    // LED 2: GPS
    auto *gps = AP::gps();
    if (gps && gps->num_sensors() > 0) {
        if (gps->status() >= AP_GPS::GPS_OK_FIX_3D) {
            b[1] = 255; // Solid Blue for 3D Fix or better
        } else {
            b[1] = (AP_HAL::millis() % 1000) < 500 ? 255 : 0; // Blinking Blue for No Fix
        }
    }

    // LED 3: Telemetry
    if (GCS_MAVLINK::is_connected()) {
        b[2] = 255; r[2] = 255; // Solid Cyan for GCS Link Active
    }

    // LED 4: Battery
    auto *battery = AP::battery();
    if (battery && battery->num_instances() > 0) {
        if (battery->has_failsafed()) {
             r[3] = (AP_HAL::millis() % 250) < 125 ? 255 : 0; // Blinking Red (Fast) for Failsafe
        } else if (battery->get_voltage() > 0) {
            float charge_level = battery->capacity_remaining_pct();
            if (charge_level < 25) {
                r[3] = 255; // Solid Red for Critical
            } else if (charge_level < 50) {
                r[3] = 255; g[3] = 255; // Solid Yellow for Low
            } else {
                g[3] = 255; // Solid Green for Good
            }
        }
    }

    send_start_frame();
    for (uint8_t i=0; i<4; i++) {
        send_led_data(r[i], g[i], b[i]);
    }
    send_end_frame();
}

void AP_Notify_APA102_LED::send_led_data(uint8_t r, uint8_t g, uint8_t b)
{
    // APA102 requires a start byte of 0b11100000 + 5 bits of brightness
    uint8_t brightness = 0b00011111; // Full brightness
    uint8_t header = 0b11100000 | brightness;

    // Send header
    for (int8_t i=7; i>=0; i--) {
        clock_pin->write(0);
        data_pin->write((header >> i) & 1);
        clock_pin->write(1);
    }

    // Send blue
    for (int8_t i=7; i>=0; i--) {
        clock_pin->write(0);
        data_pin->write((b >> i) & 1);
        clock_pin->write(1);
    }

    // Send green
    for (int8_t i=7; i>=0; i--) {
        clock_pin->write(0);
        data_pin->write((g >> i) & 1);
        clock_pin->write(1);
    }

    // Send red
    for (int8_t i=7; i>=0; i--) {
        clock_pin->write(0);
        data_pin->write((r >> i) & 1);
        clock_pin->write(1);
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