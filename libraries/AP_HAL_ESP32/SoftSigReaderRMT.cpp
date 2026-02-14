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
 *
 * Code by David "Buzz" Bussenschutt and others
 *
 */

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32

#include "SoftSigReaderRMT.h"

using namespace ESP32;

extern const AP_HAL::HAL& hal;

bool IRAM_ATTR SoftSigReaderRMT::rx_done_callback(rmt_channel_handle_t channel,
                                                     const rmt_rx_done_event_data_t *edata,
                                                     void *user_ctx)
{
    SoftSigReaderRMT *self = static_cast<SoftSigReaderRMT*>(user_ctx);
    self->num_received = edata->num_symbols;
    self->data_ready = true;
    return false;
}

void SoftSigReaderRMT::start_receive()
{
    rmt_receive_config_t rx_config = {};
    rx_config.signal_range_min_ns = 1250;
    rx_config.signal_range_max_ns = idle_threshold_ns;
    rmt_receive(rx_channel, rx_symbols, sizeof(rx_symbols), &rx_config);
}

void SoftSigReaderRMT::init()
{
#ifndef HAL_ESP32_RMT_RX_PIN_NUMBER
    #error HAL_ESP32_RMT_RX_PIN_NUMBER undefined in hwdef
#endif

    rmt_rx_channel_config_t rx_chan_config = {};
    rx_chan_config.gpio_num = (gpio_num_t)HAL_ESP32_RMT_RX_PIN_NUMBER;
    rx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rx_chan_config.resolution_hz = resolution_hz;
    rx_chan_config.mem_block_symbols = 64;

    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &rx_channel));

    rmt_rx_event_callbacks_t cbs = {};
    cbs.on_recv_done = rx_done_callback;
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, this));

    ESP_ERROR_CHECK(rmt_enable(rx_channel));
    start_receive();

    channelpointer = -1;
    memset(channeldata0, 0, sizeof(channeldata0));
    memset(channeldata1, 0, sizeof(channeldata1));
}

bool SoftSigReaderRMT::read(uint32_t &widths0, uint32_t &widths1)
{
    // Check for new data from ISR callback
    if (data_ready) {
        data_ready = false;

        channels = (num_received > 0) ? (int)(num_received - 1) : 0;
        // Convert RMT ticks to microseconds (resolution is 8MHz, so divide by 8)
        const uint32_t tick_to_us = resolution_hz / 1000000;
        for (int i = 0; i < channels && i < 16; i++) {
            channeldata0[i] = rx_symbols[i].duration0 / tick_to_us;
            channeldata1[i] = rx_symbols[i].duration1 / tick_to_us;
            if (channelpointer < 0) {
                channelpointer = 0;
            }
        }

        // Re-arm reception
        start_receive();
    }

    // Return one channel per call
    if (channelpointer >= 0) {
        widths0 = uint16_t(channeldata0[channelpointer]);
        widths1 = uint16_t(channeldata1[channelpointer]);

        channelpointer++;

        // After the 8th channel, insert the wide idle pulse for PPM sum
        if (channelpointer == 9) {
            widths0 = 3000;
            widths1 = 1000;
        }
        if (channelpointer > 9) {
            channelpointer = 0;
            return false;
        }
        return true;
    }

    return false;
}

#endif //CONFIG_HAL_BOARD == HAL_BOARD_ESP32
