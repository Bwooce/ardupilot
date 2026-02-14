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
 */
#pragma once

#include "AP_HAL_ESP32.h"
#include "driver/rmt_rx.h"

class ESP32::RmtSigReader
{
public:
    static const uint32_t frequency = 1000000;  // 1MHz resolution
    static const int max_pulses = 128;
    static const int idle_threshold = 3000;  // 3ms gap between frames (in ticks at 1MHz = us)
    void init();
    void disable();
    bool read(uint32_t &width_high, uint32_t &width_low);
private:
    bool add_item(uint32_t duration, bool level);
    void start_receive();

    rmt_channel_handle_t rx_channel;
    rmt_symbol_word_t rx_symbols[max_pulses];

    volatile size_t num_received;
    volatile bool data_ready;
    size_t current_item;
    size_t item_count;

    uint32_t last_high;
    uint32_t ready_high;
    uint32_t ready_low;
    bool pulse_ready;

    static bool IRAM_ATTR rx_done_callback(rmt_channel_handle_t channel,
                                            const rmt_rx_done_event_data_t *edata,
                                            void *user_ctx);
};
