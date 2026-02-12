/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "Display_Backend.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/I2CDevice.h>

#define HD44780_COLUMNS 16
#define HD44780_ROWS 2

// HD44780 LCD commands
#define HD44780_CLEARDISPLAY 0x01
#define HD44780_RETURNHOME 0x02
#define HD44780_ENTRYMODESET 0x04
#define HD44780_DISPLAYCONTROL 0x08
#define HD44780_CURSORSHIFT 0x10
#define HD44780_FUNCTIONSET 0x20
#define HD44780_SETCGRAMADDR 0x40
#define HD44780_SETDDRAMADDR 0x80

// Flags for display on/off control
#define HD44780_DISPLAYON 0x04
#define HD44780_DISPLAYOFF 0x00
#define HD44780_CURSORON 0x02
#define HD44780_CURSOROFF 0x00
#define HD44780_BLINKON 0x01
#define HD44780_BLINKOFF 0x00

// Flags for display/cursor shift
#define HD44780_DISPLAYMOVE 0x08
#define HD44780_CURSORMOVE 0x00
#define HD44780_MOVERIGHT 0x04
#define HD44780_MOVELEFT 0x00

// Flags for function set
#define HD44780_8BITMODE 0x10
#define HD44780_4BITMODE 0x00
#define HD44780_2LINE 0x08
#define HD44780_1LINE 0x00
#define HD44780_5x10DOTS 0x04
#define HD44780_5x8DOTS 0x00

// I2C LCD backpack bit positions
#define HD44780_BACKLIGHT 0x08
#define HD44780_ENABLE 0x04
#define HD44780_READWRITE 0x02
#define HD44780_REGISTER_SELECT 0x01

class Display_HD44780_I2C: public Display_Backend
{
private:
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    
    bool _need_hw_update;
    char _text_buffer[HD44780_ROWS][HD44780_COLUMNS + 1];
    bool _buffer_dirty[HD44780_ROWS];
    
    void _write_4bits(uint8_t value);
    void _expand_write(uint8_t data);
    void _pulse_enable(uint8_t data);
    void _write_command(uint8_t cmd);
    void _write_data(uint8_t data);
    void _timer();
    void _update_display();

public:
    Display_HD44780_I2C(AP_HAL::OwnPtr<AP_HAL::Device> dev);
    ~Display_HD44780_I2C();

    static Display_HD44780_I2C *probe(AP_HAL::OwnPtr<AP_HAL::Device> dev);

    bool hw_init() override;
    void hw_update() override;
    void set_pixel(uint16_t x, uint16_t y) override;
    void clear_pixel(uint16_t x, uint16_t y) override;
    void clear_screen() override;
    DisplayType get_display_type() const override { return DISPLAY_TYPE_CHARACTER; }
    void draw_text(uint16_t x, uint16_t y, const char* c) override;
    
    // HD44780-specific methods
    void set_cursor(uint8_t col, uint8_t row);
    void write_char(uint8_t col, uint8_t row, char c);
    void write_string(uint8_t col, uint8_t row, const char* str);
};