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

#include "Display_HD44780_I2C.h"

#include <utility>
#include <cstring>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

extern const AP_HAL::HAL& hal;

// Constructor
Display_HD44780_I2C::Display_HD44780_I2C(AP_HAL::OwnPtr<AP_HAL::Device> dev) :
    _dev(std::move(dev)),
    _need_hw_update(false)
{
    // Initialize text buffer to spaces
    for (uint8_t row = 0; row < HD44780_ROWS; row++) {
        memset(_text_buffer[row], ' ', HD44780_COLUMNS);
        _text_buffer[row][HD44780_COLUMNS] = '\0';
        _buffer_dirty[row] = false;
    }
}

Display_HD44780_I2C::~Display_HD44780_I2C()
{
}

Display_HD44780_I2C *Display_HD44780_I2C::probe(AP_HAL::OwnPtr<AP_HAL::Device> dev)
{
    Display_HD44780_I2C *driver = NEW_NOTHROW Display_HD44780_I2C(std::move(dev));
    if (!driver || !driver->hw_init()) {
        delete driver;
        return nullptr;
    }
    return driver;
}

bool Display_HD44780_I2C::hw_init()
{
    if (!_dev) {
        return false;
    }

    // Take I2C bus semaphore with timeout
    if (!_dev->get_semaphore()->take(100)) {
        return false;
    }

    // First check if device is present by trying a simple write
    uint8_t test_data = HD44780_BACKLIGHT;
    if (!_dev->transfer(&test_data, 1, nullptr, 0)) {
        // Device not responding, release semaphore and fail gracefully
        _dev->get_semaphore()->give();
        hal.console->printf("HD44780: Display not found at I2C address\n");
        return false;
    }

    // LCD initialization sequence for HD44780 with I2C backpack
    // Wait for LCD to power up
    hal.scheduler->delay(50);

    // Initialize in 4-bit mode according to datasheet
    _expand_write(HD44780_BACKLIGHT); // Turn on backlight
    hal.scheduler->delay(1000);

    // Put LCD into 4-bit mode - this is according to the HD44780 datasheet
    // We start in 8-bit mode, try to set 4-bit mode
    _write_4bits(0x03 << 4);
    hal.scheduler->delay(5); // Wait min 4.1ms

    // Second try
    _write_4bits(0x03 << 4);
    hal.scheduler->delay(5); // Wait min 4.1ms

    // Third try
    _write_4bits(0x03 << 4);
    hal.scheduler->delay(1);

    // Finally, set to 4-bit interface
    _write_4bits(0x02 << 4);

    // Now we can start sending commands
    _write_command(HD44780_FUNCTIONSET | HD44780_4BITMODE | HD44780_2LINE | HD44780_5x8DOTS);
    _write_command(HD44780_DISPLAYCONTROL | HD44780_DISPLAYON | HD44780_CURSOROFF | HD44780_BLINKOFF);
    _write_command(HD44780_ENTRYMODESET | 0x02); // Set entry mode: increment cursor, no shift
    _write_command(HD44780_CLEARDISPLAY);
    hal.scheduler->delay(2); // Clear command takes longer

    // Give back I2C semaphore
    _dev->get_semaphore()->give();

    // Clear our buffer
    clear_screen();

    // Register periodic callback for updates
    _dev->register_periodic_callback(50000, FUNCTOR_BIND_MEMBER(&Display_HD44780_I2C::_timer, void));

    return true;
}

void Display_HD44780_I2C::_expand_write(uint8_t data)
{
    uint8_t buf = data | HD44780_BACKLIGHT;
    _dev->transfer(&buf, 1, nullptr, 0);
}

void Display_HD44780_I2C::_write_4bits(uint8_t value)
{
    _expand_write(value);
    _pulse_enable(value);
}

void Display_HD44780_I2C::_pulse_enable(uint8_t data)
{
    _expand_write(data | HD44780_ENABLE);
    hal.scheduler->delay_microseconds(1);
    _expand_write(data & ~HD44780_ENABLE);
    hal.scheduler->delay_microseconds(50);
}

void Display_HD44780_I2C::_write_command(uint8_t cmd)
{
    uint8_t high = cmd & 0xF0;
    uint8_t low = (cmd << 4) & 0xF0;
    _write_4bits(high);
    _write_4bits(low);
}

void Display_HD44780_I2C::_write_data(uint8_t data)
{
    uint8_t high = data & 0xF0;
    uint8_t low = (data << 4) & 0xF0;
    _write_4bits(high | HD44780_REGISTER_SELECT);
    _write_4bits(low | HD44780_REGISTER_SELECT);
}

void Display_HD44780_I2C::set_cursor(uint8_t col, uint8_t row)
{
    static const uint8_t row_offsets[] = { 0x00, 0x40 };
    if (row >= HD44780_ROWS) {
        row = HD44780_ROWS - 1;
    }
    if (col >= HD44780_COLUMNS) {
        col = HD44780_COLUMNS - 1;
    }
    _write_command(HD44780_SETDDRAMADDR | (col + row_offsets[row]));
}

void Display_HD44780_I2C::write_char(uint8_t col, uint8_t row, char c)
{
    if (row >= HD44780_ROWS || col >= HD44780_COLUMNS) {
        return;
    }
    
    if (_text_buffer[row][col] != c) {
        _text_buffer[row][col] = c;
        _buffer_dirty[row] = true;
        _need_hw_update = true;
    }
}

void Display_HD44780_I2C::write_string(uint8_t col, uint8_t row, const char* str)
{
    if (row >= HD44780_ROWS || !str) {
        return;
    }
    
    uint8_t pos = col;
    while (*str && pos < HD44780_COLUMNS) {
        write_char(pos, row, *str);
        str++;
        pos++;
    }
}

void Display_HD44780_I2C::_update_display()
{
    for (uint8_t row = 0; row < HD44780_ROWS; row++) {
        if (_buffer_dirty[row]) {
            set_cursor(0, row);
            for (uint8_t col = 0; col < HD44780_COLUMNS; col++) {
                _write_data(_text_buffer[row][col]);
            }
            _buffer_dirty[row] = false;
        }
    }
}

void Display_HD44780_I2C::_timer()
{
    if (!_need_hw_update) {
        return;
    }
    _need_hw_update = false;

    _dev->get_semaphore()->take_blocking();
    _update_display();
    _dev->get_semaphore()->give();
}

void Display_HD44780_I2C::hw_update()
{
    _need_hw_update = true;
}

void Display_HD44780_I2C::set_pixel(uint16_t x, uint16_t y)
{
    // HD44780 is character-based, not pixel-based
    // This maps pixel coordinates to character positions for compatibility
    uint8_t col = x / 8; // Assume 8 pixels per character
    uint8_t row = y / 8;
    if (col < HD44780_COLUMNS && row < HD44780_ROWS) {
        write_char(col, row, '#'); // Use '#' to represent a "set" pixel
    }
}

void Display_HD44780_I2C::clear_pixel(uint16_t x, uint16_t y)
{
    // Clear pixel by writing space
    uint8_t col = x / 8;
    uint8_t row = y / 8;
    if (col < HD44780_COLUMNS && row < HD44780_ROWS) {
        write_char(col, row, ' ');
    }
}

void Display_HD44780_I2C::clear_screen()
{
    for (uint8_t row = 0; row < HD44780_ROWS; row++) {
        for (uint8_t col = 0; col < HD44780_COLUMNS; col++) {
            _text_buffer[row][col] = ' ';
        }
        _text_buffer[row][HD44780_COLUMNS] = '\0';
        _buffer_dirty[row] = true;
    }
    _need_hw_update = true;
}

void Display_HD44780_I2C::draw_text(uint16_t x, uint16_t y, const char* c)
{
    if (nullptr == c) {
        return;
    }
    
    // Convert pixel coordinates to character coordinates
    // ArduPilot uses specific pixel spacing: COLUMN(X) = ((X *  7) + 0), ROW(Y) = ((Y * 10) + 6)
    // So reverse calculation: col = x / 7, row = (y - 6) / 10
    uint8_t col = x / 7;
    uint8_t row = (y >= 6) ? ((y - 6) / 10) : 0;
    
    // Ensure we're within bounds
    if (row >= HD44780_ROWS) {
        row = HD44780_ROWS - 1;
    }
    
    // Write the string directly to the LCD
    write_string(col, row, c);
}