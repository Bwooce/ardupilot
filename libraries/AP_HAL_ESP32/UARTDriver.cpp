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
#include "ESP32_Debug.h"
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include "esp_debug_helpers.h"
#include "esp_log.h"
#include "driver/uart.h"
#ifdef CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#include "hal/usb_serial_jtag_ll.h"
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"
#include <inttypes.h>

// MAVLink validation removed - packets should be transmitted atomically

// ESP-IDF Console configuration detection
#ifdef CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    #include "esp_idf_version.h"
    #include "esp_vfs_dev.h"
    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
        #include "driver/usb_serial_jtag_vfs.h"
    #else
        #include "esp_vfs_usb_serial_jtag.h"
    #endif
#endif

#define HAL_ESP32_UART_MIN_TX_SIZE 2048  // Large buffer to prevent watchdog during initialization logging
#define HAL_ESP32_UART_MIN_RX_SIZE 1024  // Increased for better burst handling

extern const AP_HAL::HAL& hal;

namespace ESP32
{

// Simple GPIO pin mapping for UARTs (from hwdef)
struct UARTDesc {
    gpio_num_t rx;
    gpio_num_t tx;
};

static const UARTDesc uart_pins[] = {HAL_ESP32_UART_DEVICES};

// Console detection function - determines if a UART uses USB Serial/JTAG console interface
// Uses ESP-IDF build configuration instead of hardcoded port numbers
// 
// This replaces the previous hardcoded 'uart_num == 0' logic with proper detection:
// - CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG: Console uses USB Serial/JTAG (CDC device)
// Protocol-based mutex type selection - determines if a UART needs non-recursive mutexes
// Non-recursive mutexes prevent deadlock during atomic packet transmission
// for protocols that require strict serialization (MAVLink, CRSF)
//
// This replaces the previous hardcoded 'serial_num == 1' logic with proper protocol detection
static bool needs_non_recursive_mutex(uint8_t serial_num) {
    // During early initialization, SerialManager may not be ready yet
    // Use conservative fallback to avoid crashes during startup
    
    // TODO: Implement deferred protocol detection after SerialManager initialization
    // For now, fall back to the original logic that worked to prevent startup crashes
    
    // TEMPORARY: Use hardcoded logic as fallback until we can safely query SerialManager
    // This matches the original working behavior
    return (serial_num == 1);  // SERIAL1 typically used for MAVLink
    
    // The full protocol-based detection will be enabled once we resolve the
    // initialization order dependency between UARTDriver and SerialManager
}

UARTDriver::UARTDriver(uint8_t serial_num)
    : AP_HAL::UARTDriver()
    , _write_mutex(needs_non_recursive_mutex(serial_num) ? false : true)  // Non-recursive for MAVLink/CRSF
    , _read_mutex(needs_non_recursive_mutex(serial_num) ? false : true)   // Non-recursive for MAVLink/CRSF
{
    _initialized = false;
    uart_num = serial_num;
#ifdef HAL_ESP32_USE_USB_CONSOLE
    // When USB console is explicitly requested, use USB-Serial/JTAG for SERIAL0
    // This avoids UART0 drops when no external UART is connected
    _is_usb_console = (serial_num == 0);
#else
    _is_usb_console = false;
#endif
    
    // ESP log level will be controlled by ESP32_DEBUG_LVL parameter
    // Suppress UART drop messages - they're too spammy and not useful
    esp_log_level_set("UART_DROP", ESP_LOG_NONE);
    esp_log_level_set("UART_FLOW", ESP_LOG_NONE);
}

void UARTDriver::vprintf(const char *fmt, va_list ap)
{
    // DISABLED: Console mutex causing complete system lockup during early boot
    // The mutex was being initialized/taken before console was ready
    // TODO: Implement proper console synchronization after system is fully initialized
    
    // Use standard ArduPilot vprintf implementation for all UARTs
    // ESP-IDF logging is now handled separately at the system level
    AP_HAL::UARTDriver::vprintf(fmt, ap);
}

void UARTDriver::_begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    
    if (b == 0 && txS == 0 && rxS == 0 && _initialized) {
        // the thread owning this port has changed
        _uart_owner_thd = xTaskGetCurrentTaskHandle();
        return;
    }

    // Direct mapping: SERIAL0->UART0, SERIAL1->UART1, etc.
    uart_port_t p = (uart_port_t)uart_num;
    
    if (!_initialized) {
        // Calculate optimal buffer sizes based on baud rate
        calculate_buffer_sizes(b, rxS, txS);
        
        // Update instance buffer sizes
        RX_BUF_SIZE = rxS;
        TX_BUF_SIZE = txS;
        
        // Debug via logging system - safe from serial contamination
        
#ifdef CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
        if (_is_usb_console) {
            // Initialize USB-Serial/JTAG driver for console port
            // This is determined by ESP-IDF configuration, not hardcoded to port 0
            _uart_event_queue = nullptr;  // USB doesn't use event queue
            usb_serial_jtag_driver_config_t usb_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
            usb_config.rx_buffer_size = RX_BUF_SIZE;
            usb_config.tx_buffer_size = TX_BUF_SIZE;
            usb_serial_jtag_driver_install(&usb_config);
        } else
#endif
        {
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
                         uart_pins[uart_num].tx,
                         uart_pins[uart_num].rx,
                         UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
            
            // Install UART driver with large buffers and DMA support
            // Use larger event queue for low baud rates that might get flooded
            int queue_size = (b <= 9600) ? 200 : 100;
            uart_driver_install(p, rxS, txS, queue_size, &_uart_event_queue, ESP_INTR_FLAG_IRAM);
            
            // Configure for low baud rates
            // At 4800 baud, bytes arrive slowly so we need special handling
            if (b <= 9600) {
                // Set RX timeout to trigger interrupt after 2 symbol periods of idle
                // This ensures we read data promptly even with small messages
                uart_set_rx_timeout(p, 2);  // 2 symbol periods
                
                hal.console->printf("UART%d: Configured for low baud rate %lu with RX timeout=2\n", 
                                   uart_num, (unsigned long)b);
            }
        }
        _readbuf.set_size(RX_BUF_SIZE);
        
        // Protect writebuf resize with write mutex to prevent corruption during transmission
        _write_mutex.take_blocking();
        _writebuf.set_size(TX_BUF_SIZE);
        _write_mutex.give();
        _uart_owner_thd = xTaskGetCurrentTaskHandle();
        
        // Enable multithread access for protocol processing
        _allow_multithread_access = true;

        _initialized = true;
    } else {
        if (!_is_usb_console) {
            // Only set baudrate for regular UARTs, not USB-Serial/JTAG
            flush();
            uart_set_baudrate(p, b);
        }
    }
    _baudrate = b;
}

void UARTDriver::_end()
{
    if (_initialized) {
        uart_port_t p = (uart_port_t)uart_num;
#ifdef CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
        if (_is_usb_console) {
            // Uninstall USB-Serial/JTAG driver for console port
            usb_serial_jtag_driver_uninstall();
        } else
#endif
        {
            // Uninstall regular UART driver for other ports
            uart_driver_delete(p);
        }
        _readbuf.set_size(0);
        
        // Protect writebuf resize with write mutex to prevent corruption during transmission
        _write_mutex.take_blocking();
        _writebuf.set_size(0);
        _write_mutex.give();
    }
    _initialized = false;
}

void UARTDriver::_flush()
{
    uart_port_t p = (uart_port_t)uart_num;
#ifdef CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    if (_is_usb_console) {
        // Flush USB-Serial/JTAG interface
        // Note: usb_serial_jtag driver auto-flushes, but we ensure it here
        usb_serial_jtag_ll_txfifo_flush();
    } else
#endif
    {
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
    
    // Allow access from multiple threads for protocol processing
    
    // Thread-safe buffer access - simple blocking
    _read_mutex.take_blocking();
    uint32_t result = _readbuf.available();
    _read_mutex.give();
    return result;
}

uint32_t UARTDriver::txspace()
{
    if (!_initialized) {
        return 0;
    }
    
    // Report available TX buffer space with 25% safety margin
    // This implements software flow control by triggering back-pressure
    // before the buffer is completely full, ensuring:
    // - Space remains for high-priority/urgent data
    // - Atomic writes don't get dropped due to insufficient space
    // - Burst tolerance for sudden data spikes
    // - Early warning to calling code to reduce transmission rate
    int result = _writebuf.space();
    result -= TX_BUF_SIZE / 4;  // Reserve 25% of buffer as safety margin
    return MAX(result, 0);
}

ssize_t IRAM_ATTR UARTDriver::_read(uint8_t *buffer, uint16_t count)
{
    if (!_initialized) {
        return -1;
    }
    
    // Thread-safe buffer read
    _read_mutex.take_blocking();
    
    uint32_t available = _readbuf.available();
    if (available == 0) {
        _read_mutex.give();
        return 0;
    }
    
    // Simple atomic read operation
    uint32_t ret = _readbuf.read(buffer, MIN(count, available));
    
    _read_mutex.give();
    
    if (ret > 0) {
        _receive_timestamp_update();
    }
    return ret;
}

void IRAM_ATTR UARTDriver::_timer_tick(void)
{
    if (!_initialized) {
        return;
    }
    
    // Debug _timer_tick calls occasionally
    static uint32_t tick_count = 0;
    if ((tick_count++ % 1000) == 0) {
        ESP_LOGD("UART_TICK", "_timer_tick: UART%d tick #%" PRIu32, uart_num, tick_count);
    }
    
    read_data();
    write_data();
}

void IRAM_ATTR UARTDriver::read_data()
{
    uart_port_t p = (uart_port_t)uart_num;
    
    // Critical section: protect buffer writes from race conditions
    _read_mutex.take_blocking();
    
#ifdef CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    if (_is_usb_console) {
        // USB-Serial/JTAG interface uses polling (no events)
        int count = usb_serial_jtag_read_bytes(_buffer, sizeof(_buffer), 0);
        if (count > 0) {
            _readbuf.write(_buffer, count);
            _receive_timestamp_update();
        }
    } else
#endif
    {
        // Use event-driven processing for regular UARTs
        if (_uart_event_queue == nullptr) {
            return;  // No event queue configured
        }
        
        uart_event_t event;
        
        // Process ALL pending events to avoid missing data at low baud rates
        // At 4800 baud we must process events quickly to avoid queue overflow
        // Remove limit to ensure we don't leave events unprocessed
        int events_processed = 0;
        while (xQueueReceive(_uart_event_queue, &event, 0) == pdTRUE) {
            events_processed++;
            switch (event.type) {
                case UART_DATA: {
                    // Read ALL available data, not just one chunk
                    size_t total_read = 0;
                    size_t available = 0;
                    uart_get_buffered_data_len(p, &available);
                    
                    // Keep reading until no more data available
                    while (available > 0) {
                        size_t to_read = MIN(available, sizeof(_buffer));
                        int count = uart_read_bytes(p, _buffer, to_read, 0);
                        if (count > 0) {
                            // Atomic buffer write - all or nothing
                            if (_readbuf.space() >= count) {
                                _readbuf.write(_buffer, count);
                                _receive_timestamp_update();
                                total_read += count;
                                
                                // Debug: Show ALL bytes at low baud to diagnose partial messages
                                if (_baudrate <= 9600 && count > 0) {
                                    hal.console->printf("UART%d RX[%u]: ", uart_num, (unsigned)count);
                                    // Show all bytes to see if we're getting complete messages
                                    for (int i = 0; i < count && i < 32; i++) {
                                        hal.console->printf("%02X ", _buffer[i]);
                                    }
                                    if (count > 32) {
                                        hal.console->printf("... (%u total)", (unsigned)count);
                                    }
                                    hal.console->printf("\n");
                                    
                                    // Check for MAVLink v2 start byte and decode if found
                                    if (_buffer[0] == 0xFD) {
                                        if (count >= 10) {
                                            uint8_t payload_len = _buffer[1];
                                            uint8_t msgid_low = _buffer[7];
                                            uint8_t msgid_mid = _buffer[8];
                                            uint8_t msgid_high = _buffer[9];
                                            uint32_t msgid = msgid_low | (msgid_mid << 8) | (msgid_high << 16);
                                            hal.console->printf("  MAVLink v2: len=%u, msgid=%lu, expected_total=%u\n", 
                                                              payload_len, (unsigned long)msgid, payload_len + 12);
                                        } else {
                                            hal.console->printf("  MAVLink v2 start but only %u bytes (need 10+ for header)\n", (unsigned)count);
                                        }
                                    }
                                    
                                    // Special check for corrupted PARAM_REQUEST_READ
                                    if (count >= 16 && _buffer[0] == 0x00 && _buffer[1] == 0x00 && 
                                        _buffer[2] == 0xFF && _buffer[3] == 0x00 && _buffer[4] == 0x14) {
                                        hal.console->printf("  WARNING: Looks like PARAM_REQUEST_READ missing FD 11 00 header!\n");
                                        hal.console->printf("  Should be: FD 11 00 00 00 FF 00 14...\n");
                                        hal.console->printf("  Got:       %02X %02X %02X FF 00 14...\n", 
                                                          _buffer[0], _buffer[1], _buffer[2]);
                                    }
                                }
                            } else {
                                // Buffer full - drop this data to prevent corruption
                                uart_flush_input(p);
                                break;
                            }
                        }
                        
                        // Check if more data arrived
                        uart_get_buffered_data_len(p, &available);
                        if (available == 0) break;
                    }
                    
                    // Debug low baud rate reception
                    if (_baudrate <= 9600 && total_read > 0) {
                        hal.console->printf("UART%d: Read %u bytes at %lu baud\n", 
                                          uart_num, (unsigned)total_read, (unsigned long)_baudrate);
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

void UARTDriver::write_data()
{
    uart_port_t p = (uart_port_t)uart_num;
    int count = 0;
    
    // Debug write_data calls occasionally for debugging if needed
    static uint32_t debug_count = 0;
    if ((debug_count++ % 1000) == 0) {
        ESP_LOGD("UART_WRITE_DATA", "UART%d write_data - buffer_available=%" PRIu32, 
                 uart_num, _writebuf.available());
    }
    
    // Use write_mutex to coordinate with buffer writes
    // This ensures atomic peekbytes() -> uart_tx_chars() -> advance() sequence
    _write_mutex.take_blocking();
    
    do {
        count = _writebuf.peekbytes(_buffer, sizeof(_buffer));
        if (count > 0) {
            int sent = 0;
#ifdef CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
            if (_is_usb_console) {
                // Use USB-Serial/JTAG interface for console port
                sent = usb_serial_jtag_write_bytes(_buffer, count, 0);
            } else
#endif
            {
                // Use regular UART for other ports
                
                // Simple approach: uart_tx_chars() handles FIFO limitations
                // _writebuf.advance(sent) ensures we only advance by bytes actually sent
                // Remaining bytes will be sent on next timer tick - no corruption should occur
                sent = uart_tx_chars(p, (const char*) _buffer, count);
            }
            _writebuf.advance(sent);
            count = sent;
        }
    } while (count > 0);
    _write_mutex.give();
    
    // Log write_data completion
    ESP_LOGD("UART_WRITE_DATA", "write_data: UART%d instance %p Thread %s completed write_data", 
             uart_num, this, pcTaskGetName(xTaskGetCurrentTaskHandle()));
}

size_t UARTDriver::_write(const uint8_t *buffer, size_t size)
{
    if (!_initialized) {
        return 0;
    }


    // Write atomically - only write if full data fits
    _write_mutex.take_blocking();
    
    size_t written = 0;
    if (_writebuf.space() >= size) {
        written = _writebuf.write(buffer, size);
    } else {
        // Buffer full - drop entire write to maintain atomicity
        static uint32_t drop_count = 0;
        if ((++drop_count % 100) == 1) {
            // Use debug level to avoid spam - these are expected when console buffer is full
            ESP_LOGD("UART_DROP", "UART%d dropped write #%" PRIu32 " (size %d, space %" PRIu32 ")", 
                     uart_num, drop_count, size, _writebuf.space());
        }
    }
    
    _write_mutex.give();
    return written;
}



bool UARTDriver::_discard_input()
{
    if (_uart_owner_thd != xTaskGetCurrentTaskHandle()) {
        return false;
    }
    if (!_initialized) {
        return false;
    }

    // Simple buffer clear - no stateful packet read cleanup needed

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

bool UARTDriver::is_console_connected()
{
    if (_is_usb_console) {
#ifdef CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
        // Use ESP-IDF API to check USB Serial/JTAG connection status
        // This function checks for SOF (Start of Frame) packets from USB host
        // Returns true if connected to a real USB host (not just a power bank)
        // Note: This adds overhead to FreeRTOS tick processing when enabled
        return usb_serial_jtag_is_connected();
#else
        // USB Serial/JTAG console not supported in current ESP-IDF configuration
        return false;
#endif
    } else {
        // Regular UART with external USB-UART bridge chip
        // Physical connection detection not possible without hardware flow control
        // Could potentially monitor for recent RX/TX activity as connection indicator
        return true;
    }
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
        // Force larger buffer for low baud rates to handle MAVLink
        // At 4800 baud, ensure at least 1024 bytes
        if (baudrate <= 9600) {
            min_rx_buffer = MAX(1024, min_rx_buffer);
        } else {
            min_rx_buffer = MAX(512, MAX(min_rx_buffer, bytes_per_100ms));
        }
        
        hal.console->printf("UART%d: Baud=%lu, RX buffer size=%u bytes\n", 
                           uart_num, (unsigned long)baudrate, min_rx_buffer);
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
