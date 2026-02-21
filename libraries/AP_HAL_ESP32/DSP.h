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

#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_DSP

#include "HAL_ESP32_Namespace.h"

class ESP32::DSP : public AP_HAL::DSP {
public:
    virtual FFTWindowState* fft_init(uint16_t window_size, uint16_t sample_rate, uint8_t sliding_window_size) override;
    virtual void fft_start(FFTWindowState* state, FloatBuffer& samples, uint16_t advance) override;
    virtual uint16_t fft_analyse(FFTWindowState* state, uint16_t start_bin, uint16_t end_bin, float noise_att_cutoff) override;

    class FFTWindowStateESP32 : public AP_HAL::DSP::FFTWindowState {
        friend class ESP32::DSP;
    public:
        FFTWindowStateESP32(uint16_t window_size, uint16_t sample_rate, uint8_t sliding_window_size);
        virtual ~FFTWindowStateESP32();
    };

protected:
    void vector_max_float(const float* vin, uint16_t len, float* maxValue, uint16_t* maxIndex) const override;
    void vector_scale_float(const float* vin, float scale, float* vout, uint16_t len) const override;
    float vector_mean_float(const float* vin, uint16_t len) const override;
    void vector_add_float(const float* vin1, const float* vin2, float* vout, uint16_t len) const override;

private:
    void step_hanning(FFTWindowStateESP32* fft, FloatBuffer& samples, uint16_t advance);
    void step_fft(FFTWindowStateESP32* fft);
    void step_cmplx_mag_esp32(FFTWindowStateESP32* fft, uint16_t start_bin, uint16_t end_bin, float noise_att_cutoff);
    uint16_t step_calc_frequencies_esp32(FFTWindowStateESP32* fft, uint16_t start_bin, uint16_t end_bin);

    static bool _twiddle_initialized;
    static uint16_t _twiddle_fft_size;
};

#endif // HAL_WITH_DSP
