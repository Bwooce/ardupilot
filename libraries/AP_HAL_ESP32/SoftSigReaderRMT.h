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
#pragma once

#include "AP_HAL_ESP32.h"
#include "driver/rmt_rx.h"

namespace ESP32
{

class SoftSigReaderRMT
{
public:
    // get singleton
    static SoftSigReaderRMT *get_instance(void)
    {
        return _instance;
    }

    void init();
    bool read(uint32_t &widths0, uint32_t &widths1);
private:
    static const int max_pulses = 64;
    static const uint32_t resolution_hz = 8000000;  // 8MHz (was 80MHz/10)
    static const uint32_t idle_threshold_ns = 3500000;  // 3.5ms PPM frame gap

    rmt_channel_handle_t rx_channel;
    rmt_symbol_word_t rx_symbols[max_pulses];
    volatile size_t num_received;
    volatile bool data_ready;

    static SoftSigReaderRMT *_instance;

    uint32_t channeldata0[16];
    uint32_t channeldata1[16];
    int channelpointer;
    int channels;

    static bool IRAM_ATTR rx_done_callback(rmt_channel_handle_t channel,
                                            const rmt_rx_done_event_data_t *edata,
                                            void *user_ctx);
    void start_receive();
};
}
