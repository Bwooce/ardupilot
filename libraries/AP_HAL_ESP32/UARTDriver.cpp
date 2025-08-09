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

#include <AP_HAL_ESP32/UARTDriver.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "esp_log.h"
#include "driver/usb_serial_jtag.h"
#include "hal/usb_serial_jtag_ll.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define HAL_ESP32_UART_MIN_TX_SIZE 512
#define HAL_ESP32_UART_MIN_RX_SIZE 512

extern const AP_HAL::HAL& hal;

namespace ESP32
{

UARTDesc uart_desc[] = {HAL_ESP32_UART_DEVICES};

void UARTDriver::vprintf(const char *fmt, va_list ap)
{
    uart_port_t p = uart_desc[uart_num].port;
    if (p == 0) {
        // Always use ESP32 logging system for USB-Serial/JTAG interface
        // vprintf() only handles formatted text, never binary MAVLink data
        esp_log_writev(ESP_LOG_INFO, "", fmt, ap);
    } else {
        AP_HAL::UARTDriver::vprintf(fmt, ap);
    }
}

void UARTDriver::_begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    if (b == 0 && txS == 0 && rxS == 0 && _initialized) {
        // the thread owning this port has changed
        _uart_owner_thd = xTaskGetCurrentTaskHandle();
        return;
    }

    if (uart_num < ARRAY_SIZE(uart_desc)) {
        uart_port_t p = uart_desc[uart_num].port;
        if (!_initialized) {
            // Calculate optimal buffer sizes based on baud rate
            calculate_buffer_sizes(b, rxS, txS);
            
            // Update instance buffer sizes
            RX_BUF_SIZE = rxS;
            TX_BUF_SIZE = txS;
            
            // Debug: Log buffer sizes (can be removed later)
            printf("ESP32 UART%d: baud=%d, RX=%d, TX=%d\n", uart_num, (int)b, (int)rxS, (int)txS);
            
            if (p == 0) {
                // Initialize USB-Serial/JTAG driver for port 0 (ESP32-S3)
                _uart_event_queue = nullptr;  // USB doesn't use event queue
                usb_serial_jtag_driver_config_t usb_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
                usb_config.rx_buffer_size = RX_BUF_SIZE;
                usb_config.tx_buffer_size = TX_BUF_SIZE;
                usb_serial_jtag_driver_install(&usb_config);
            } else {
                // Initialize regular UART for other ports with DMA support
                uart_config_t config = {
                    .baud_rate = (int)b,
                    .data_bits = UART_DATA_8_BITS,
                    .parity = UART_PARITY_DISABLE,
                    .stop_bits = UART_STOP_BITS_1,
                    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                    .rx_flow_ctrl_thresh = 120,
                    .source_clk = UART_SCLK_APB,
                };
                uart_param_config(p, &config);
                uart_set_pin(p,
                             uart_desc[uart_num].tx,
                             uart_desc[uart_num].rx,
                             UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
                
                // Install UART driver with large buffers and DMA support
                // Use event queue for interrupt-driven processing
                uart_driver_install(p, rxS, txS, 100, &_uart_event_queue, ESP_INTR_FLAG_IRAM);
            }
            _readbuf.set_size(RX_BUF_SIZE);
            _writebuf.set_size(TX_BUF_SIZE);
            _uart_owner_thd = xTaskGetCurrentTaskHandle();
            
            // Enable multithread access for MAVLink processing
            _allow_multithread_access = true;

            _initialized = true;
        } else {
            if (p != 0) {
                // Only set baudrate for regular UARTs, not USB-Serial/JTAG
                flush();
                uart_set_baudrate(p, b);
            }
        }
    }
    _baudrate = b;
}

void UARTDriver::_end()
{
    if (_initialized) {
        uart_port_t p = uart_desc[uart_num].port;
        if (p == 0) {
            // Uninstall USB-Serial/JTAG driver for port 0
            usb_serial_jtag_driver_uninstall();
        } else {
            // Uninstall regular UART driver for other ports
            uart_driver_delete(p);
        }
        _readbuf.set_size(0);
        _writebuf.set_size(0);
    }
    _initialized = false;
}

void UARTDriver::_flush()
{
    uart_port_t p = uart_desc[uart_num].port;
    if (p == 0) {
        // Flush USB-Serial/JTAG interface
        // Note: usb_serial_jtag driver auto-flushes, but we ensure it here
        usb_serial_jtag_ll_txfifo_flush();
    } else {
        // Flush regular UART
        uart_flush(p);
    }
}

bool UARTDriver::is_initialized()
{
    return _initialized;
}

bool UARTDriver::tx_pending()
{
    return (_writebuf.available() > 0);
}


uint32_t UARTDriver::_available()
{
    if (!_initialized) {
        return 0;
    }
    
    // MAVLink processing requires access from multiple threads - remove restriction
    
    // Thread-safe access to buffer - use non-blocking to avoid deadlock
    if (!_read_mutex.take_nonblocking()) {
        // If mutex is held, return cached value or 0 to avoid blocking parameter processing
        return 0;
    }
    uint32_t result = _readbuf.available();
    _read_mutex.give();
    return result;
}

uint32_t UARTDriver::txspace()
{
    if (!_initialized) {
        return 0;
    }
    int result =  _writebuf.space();
    result -= TX_BUF_SIZE / 4;
    return MAX(result, 0);

}

ssize_t IRAM_ATTR UARTDriver::_read(uint8_t *buffer, uint16_t count)
{
    if (!_initialized) {
        return -1;
    }
    
    // MAVLink processing requires access from multiple threads - remove restriction

    // Thread-safe buffer read - use non-blocking to avoid deadlock with timer
    if (!_read_mutex.take_nonblocking()) {
        // If mutex is held by timer_tick, return 0 to avoid blocking
        return 0;
    }
    const uint32_t ret = _readbuf.read(buffer, count);
    _read_mutex.give();
    
    if (ret == 0) {
        return 0;
    }

    _receive_timestamp_update();
    return ret;
}

void IRAM_ATTR UARTDriver::_timer_tick(void)
{
    if (!_initialized) {
        return;
    }
    read_data();
    write_data();
}

void IRAM_ATTR UARTDriver::read_data()
{
    uart_port_t p = uart_desc[uart_num].port;
    
    // Critical section: protect buffer writes from race conditions
    _read_mutex.take_blocking();
    
    if (p == 0) {
        // USB-Serial/JTAG interface uses polling (no events)
        int count = usb_serial_jtag_read_bytes(_buffer, sizeof(_buffer), 0);
        if (count > 0) {
            _readbuf.write(_buffer, count);
            _receive_timestamp_update();
        }
    } else {
        // Use event-driven processing for regular UARTs
        if (_uart_event_queue == nullptr) {
            return;  // No event queue configured
        }
        
        uart_event_t event;
        
        // Process events one at a time to avoid race conditions
        // Only process one event per call to prevent buffer corruption
        if (xQueueReceive(_uart_event_queue, &event, 0) == pdTRUE) {
            switch (event.type) {
                case UART_DATA: {
                    // Read available data in chunks with better error handling
                    size_t available = 0;
                    uart_get_buffered_data_len(p, &available);
                    
                    if (available > 0) {
                        size_t to_read = MIN(available, sizeof(_buffer));
                        int count = uart_read_bytes(p, _buffer, to_read, 0);
                        if (count > 0) {
                            // Atomic buffer write - all or nothing
                            if (_readbuf.space() >= count) {
                                _readbuf.write(_buffer, count);
                                _receive_timestamp_update();
                            } else {
                                // Buffer full - drop this data to prevent corruption
                                uart_flush_input(p);
                            }
                        }
                    }
                    break;
                }
                case UART_BUFFER_FULL:
                    // Handle buffer overflow - emergency read
                    uart_flush_input(p);
                    break;
                    
                case UART_FIFO_OVF:
                    // FIFO overflow - clear and continue
                    uart_flush_input(p);
                    break;
                    
                case UART_FRAME_ERR:
                case UART_PARITY_ERR:
                    // Error recovery - just log and continue
                    break;
                    
                default:
                    break;
            }
        }
    }
    
    // Release mutex at end of function
    _read_mutex.give();
}

void IRAM_ATTR UARTDriver::write_data()
{
    uart_port_t p = uart_desc[uart_num].port;
    int count = 0;
    _write_mutex.take_blocking();
    do {
        count = _writebuf.peekbytes(_buffer, sizeof(_buffer));
        if (count > 0) {
            if (p == 0) {
                // Use USB-Serial/JTAG interface for port 0 (ESP32-S3)
                count = usb_serial_jtag_write_bytes(_buffer, count, 0);
            } else {
                // Use regular UART for other ports
                count = uart_tx_chars(p, (const char*) _buffer, count);
            }
            _writebuf.advance(count);
        }
    } while (count > 0);
    _write_mutex.give();
}

size_t IRAM_ATTR UARTDriver::_write(const uint8_t *buffer, size_t size)
{
    if (!_initialized) {
        return 0;
    }

    _write_mutex.take_blocking();


    size_t ret = _writebuf.write(buffer, size);
    _write_mutex.give();
    return ret;
}

bool UARTDriver::_discard_input()
{
    if (_uart_owner_thd != xTaskGetCurrentTaskHandle()) {
        return false;
    }
    if (!_initialized) {
        return false;
    }

    _readbuf.clear();

    return true;
}

// record timestamp of new incoming data
void IRAM_ATTR UARTDriver::_receive_timestamp_update(void)
{
    _receive_timestamp[_receive_timestamp_idx^1] = AP_HAL::micros64();
    _receive_timestamp_idx ^= 1;
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
  A return value of zero means the HAL does not support this API
*/
uint64_t UARTDriver::receive_time_constraint_us(uint16_t nbytes)
{
    uint64_t last_receive_us = _receive_timestamp[_receive_timestamp_idx];
    if (_baudrate > 0) {
        // assume 10 bits per byte. For USB we assume zero transport delay
        uint32_t transport_time_us = (1000000UL * 10UL / _baudrate) * (nbytes + available());
        last_receive_us -= transport_time_us;
    }
    return last_receive_us;
}

void UARTDriver::calculate_buffer_sizes(uint32_t baudrate, uint16_t &rxS, uint16_t &txS)
{
    // Calculate minimum buffer sizes based on ChibiOS approach
    uint16_t min_tx_buffer = HAL_ESP32_UART_MIN_TX_SIZE;
    uint16_t min_rx_buffer = HAL_ESP32_UART_MIN_RX_SIZE;
    
    // Scale RX buffer with baud rate to handle burst traffic
    // Ensure we can buffer at least 100ms of data at full rate
    if (baudrate > 0) {
        // Convert baud (bits/sec) to bytes/sec, then to bytes per 100ms
        uint32_t bytes_per_100ms = (baudrate / 10) / 10; // /10 for bits->bytes, /10 for 100ms
        min_rx_buffer = MAX(min_rx_buffer, bytes_per_100ms);
    }
    
    // Double buffers for high-speed connections
    if (baudrate >= 460800) {
        min_tx_buffer *= 2;
        min_rx_buffer *= 2;
    }
    
#if CONFIG_IDF_TARGET_ESP32S3
    // ESP32-S3 has plenty of memory - use larger buffers for reliability
    min_tx_buffer *= 2;
    min_rx_buffer *= 2;
#endif
    
    // Apply calculated minimums
    if (txS < min_tx_buffer) {
        txS = min_tx_buffer;
    }
    if (rxS < min_rx_buffer) {
        rxS = min_rx_buffer;
    }
    
    // Cap at reasonable maximums
    txS = MIN(txS, 16384U);
    rxS = MIN(rxS, 16384U);
}

}
