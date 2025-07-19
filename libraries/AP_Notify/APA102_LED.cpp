#include "APA102_LED.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <GCS_MAVLink/GCS.h>

#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL& hal;

AP_Notify_APA102_LED::AP_Notify_APA102_LED() : RGBLed(1, 255, 128, 64)
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

    return true;
}

bool AP_Notify_APA102_LED::hw_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    send_start_frame();
    for (uint8_t i=0; i<4; i++) {
        send_led_data(r, g, b);
    }
    send_end_frame();
    return true;
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