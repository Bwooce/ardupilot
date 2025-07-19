#pragma once

#include "RGBLed.h"
#include <AP_HAL/AP_HAL.h>

class AP_Notify_APA102_LED : public RGBLed
{
public:
    AP_Notify_APA102_LED();

    bool init() override;

protected:
    bool hw_set_rgb(uint8_t r, uint8_t g, uint8_t b) override;

private:
    void send_led_data(uint8_t r, uint8_t g, uint8_t b);
    void send_start_frame();
    void send_end_frame();

    AP_HAL::DigitalSource *data_pin;
    AP_HAL::DigitalSource *clock_pin;
};
