#pragma once

#include "Display.h"

#define NOTIFY_DISPLAY_I2C_ADDR 0x3C
#define NOTIFY_DISPLAY_HD44780_I2C_ADDR 0x27

class Display_Backend {

public:

    virtual void hw_update() = 0;
    virtual void set_pixel(uint16_t x, uint16_t y) = 0;
    virtual void clear_pixel(uint16_t x, uint16_t y) = 0;
    virtual void clear_screen() = 0;
    
    // Display type identification for character vs pixel displays
    enum DisplayType {
        DISPLAY_TYPE_PIXEL,      // OLED displays (SSD1306, SH1106)
        DISPLAY_TYPE_CHARACTER   // Character displays (HD44780)
    };
    virtual DisplayType get_display_type() const = 0;
    
    // Optional method for character displays to handle text directly
    virtual void draw_text(uint16_t x, uint16_t y, const char* c) { (void)x; (void)y; (void)c; }

protected:

    virtual ~Display_Backend() {}

    virtual bool hw_init() = 0;

};
