/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <AP_HAL/UARTDriver.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_HAL_ESP32/AP_HAL_ESP32.h>
#include <AP_HAL_ESP32/Semaphores.h>

#include "driver/gpio.h"
#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace ESP32
{

class UARTDriver : public AP_HAL::UARTDriver
{
public:

    UARTDriver(uint8_t serial_num);  // Moved to .cpp file

    virtual ~UARTDriver() = default;

    void vprintf(const char *fmt, va_list ap) override;

    bool is_initialized() override;
    bool tx_pending() override;

    uint32_t txspace() override;

    void _timer_tick(void) override;

    uint32_t bw_in_bytes_per_second() const override
    {
        return 10*1024;
    }

    /*
      return timestamp estimate in microseconds for when the start of
      a nbytes packet arrived on the uart. This should be treated as a
      time constraint, not an exact time. It is guaranteed that the
      packet did not start being received after this time, but it
      could have been in a system buffer before the returned time.
      This takes account of the baudrate of the link. For transports
      that have no baudrate (such as USB) the time estimate may be
      less accurate.
      A return value of zero means the HAL does not support this API */
     
    uint64_t receive_time_constraint_us(uint16_t nbytes) override; 

    uint32_t get_baud_rate() const override { return _baudrate; }
    
    // Debug: Get buffer sizes for verification
    uint16_t get_rx_buffer_size() const { return RX_BUF_SIZE; }
    uint16_t get_tx_buffer_size() const { return TX_BUF_SIZE; }
    
    // write() now handles atomicity - no need for separate write_packet()
    

private:
    bool _initialized;
    size_t TX_BUF_SIZE = 1024;
    size_t RX_BUF_SIZE = 1024;
    uint8_t _buffer[32];
    ByteBuffer _readbuf{0};
    ByteBuffer _writebuf{0};
    Semaphore _write_mutex;
    Semaphore _read_mutex;
    void read_data();
    void write_data();
    void calculate_buffer_sizes(uint32_t baudrate, uint16_t &rxS, uint16_t &txS);

    uint8_t uart_num;

    // timestamp for receiving data on the UART, avoiding a lock
    uint64_t _receive_timestamp[2];
    uint8_t _receive_timestamp_idx;
    uint32_t _baudrate;

    const tskTaskControlBlock* _uart_owner_thd;
    
    // Allow multiple threads to access UART for MAVLink processing
    bool _allow_multithread_access;
    
    // Event-driven processing instead of polling
    QueueHandle_t _uart_event_queue;

    void _receive_timestamp_update(void);

protected:
    void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    void _end() override;
    void _flush() override;
    uint32_t _available() override;
    ssize_t _read(uint8_t *buffer, uint16_t count) override;
    size_t _write(const uint8_t *buffer, size_t size) override;
    bool _discard_input() override; // discard all bytes available for reading
};

}
