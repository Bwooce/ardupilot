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

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_ESP32/HAL_ESP32_Class.h>
#include <AP_Math/div1000.h>
#include "SdCard.h"
#include "esp_rom_uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#ifdef CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#include "hal/usb_serial_jtag_ll.h"
#endif

#include <stdint.h>
#include "esp_timer.h"

namespace AP_HAL
{

void panic(const char *errormsg, ...)
{
    va_list ap;
    char panic_buf[256];
    
    // Format the panic message
    va_start(ap, errormsg);
    vsnprintf(panic_buf, sizeof(panic_buf), errormsg, ap);
    va_end(ap);

#ifdef CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    // For USB Serial/JTAG console, try to write directly
    // This might work even when normal console isn't initialized
    const char *prefix = "\n\n!!! PANIC: ";
    const char *suffix = " !!!\n\n";
    
    // Try to write directly to USB Serial/JTAG
    usb_serial_jtag_write_bytes(prefix, strlen(prefix), 100);
    usb_serial_jtag_write_bytes(panic_buf, strlen(panic_buf), 100);
    usb_serial_jtag_write_bytes(suffix, strlen(suffix), 100);
    
    // Wait for it to be sent
    usb_serial_jtag_ll_txfifo_flush();
#endif

    // Also try standard output methods
    fflush(stdout);
    printf("\n\n!!! PANIC !!!\n%s\n!!! PANIC END !!!\n\n", panic_buf);
    fflush(stdout);
    
    // For non-USB consoles, ROM printf might work
    esp_rom_printf("\n\n*** PANIC: %s ***\n\n", panic_buf);
    esp_rom_output_tx_wait_idle(0);
    
    // Save to RTC memory for debugging
    static RTC_DATA_ATTR struct {
        uint32_t magic;
        uint32_t counter;
        char message[248];
    } panic_info;
    panic_info.magic = 0xDEADBEEF;
    panic_info.counter++;  // Increment to see if this is a new panic
    strncpy(panic_info.message, panic_buf, sizeof(panic_info.message)-1);
    panic_info.message[sizeof(panic_info.message)-1] = '\0';

    while (1) {
        // Feed watchdog occasionally to prevent WDT from firing
        // This helps us see the panic message before WDT resets
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

uint32_t micros()
{
    return micros64();
}

uint32_t millis()
{
    return millis64();
}

uint64_t micros64()
{
    return esp_timer_get_time();
}

uint64_t millis64()
{
    return uint64_div1000(micros64());
}

} // namespace AP_HAL

static HAL_ESP32 hal_esp32;

const AP_HAL::HAL& AP_HAL::get_HAL()
{
    return hal_esp32;
}

AP_HAL::HAL& AP_HAL::get_HAL_mutable()
{
    return hal_esp32;
}
