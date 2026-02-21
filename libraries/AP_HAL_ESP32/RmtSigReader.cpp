#include <AP_HAL/HAL.h>
#include "RmtSigReader.h"

#ifdef HAL_ESP32_RCIN

using namespace ESP32;

bool IRAM_ATTR RmtSigReader::rx_done_callback(rmt_channel_handle_t channel,
                                                const rmt_rx_done_event_data_t *edata,
                                                void *user_ctx)
{
    RmtSigReader *self = static_cast<RmtSigReader*>(user_ctx);
    self->num_received = edata->num_symbols;
    self->data_ready = true;
    return false;
}

void RmtSigReader::start_receive()
{
    rmt_receive_config_t rx_config = {};
    rx_config.signal_range_min_ns = 1250;
    rx_config.signal_range_max_ns = (uint32_t)idle_threshold * 1000;
    rmt_receive(rx_channel, rx_symbols, sizeof(rx_symbols), &rx_config);
}

void RmtSigReader::init()
{
    rmt_rx_channel_config_t rx_chan_config = {};
    rx_chan_config.gpio_num = HAL_ESP32_RCIN;
    rx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rx_chan_config.resolution_hz = frequency;
    rx_chan_config.mem_block_symbols = 128;

    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &rx_channel));

    rmt_rx_event_callbacks_t cbs = {};
    cbs.on_recv_done = rx_done_callback;
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, this));

    ESP_ERROR_CHECK(rmt_enable(rx_channel));
    start_receive();
}

void RmtSigReader::disable()
{
    rmt_disable(rx_channel);
}

bool RmtSigReader::add_item(uint32_t duration, bool level)
{
    bool has_more = true;
    if (duration == 0) {
        has_more = false;
        duration = idle_threshold;
    }
    if (level) {
        if (last_high == 0) {
            last_high = duration;
        }
    } else {
        if (last_high != 0) {
            ready_high = last_high;
            ready_low = duration;
            pulse_ready = true;
            last_high = 0;
        }
    }
    return has_more;
}

bool RmtSigReader::read(uint32_t &width_high, uint32_t &width_low)
{
    // Pick up new data from ISR callback
    if (current_item >= item_count) {
        if (!data_ready) {
            return false;
        }
        data_ready = false;
        item_count = num_received;
        current_item = 0;
    }

    bool buffer_empty = (current_item >= item_count);
    buffer_empty = buffer_empty ||
                   !add_item(rx_symbols[current_item].duration0, rx_symbols[current_item].level0);
    buffer_empty = buffer_empty ||
                   !add_item(rx_symbols[current_item].duration1, rx_symbols[current_item].level1);
    current_item++;

    if (buffer_empty) {
        // Batch exhausted, re-arm reception
        current_item = 0;
        item_count = 0;
        start_receive();
    }

    if (pulse_ready) {
        width_high = ready_high;
        width_low = ready_low;
        pulse_ready = false;
        return true;
    }
    return false;
}
#endif
