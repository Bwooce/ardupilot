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

#if HAL_WITH_DSP

#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include "DSP.h"

#include "esp_dsp.h"

using namespace ESP32;

extern const AP_HAL::HAL& hal;

bool DSP::_twiddle_initialized = false;
uint16_t DSP::_twiddle_fft_size = 0;

AP_HAL::DSP::FFTWindowState* DSP::fft_init(uint16_t window_size, uint16_t sample_rate, uint8_t sliding_window_size)
{
    DSP::FFTWindowStateESP32* fft = NEW_NOTHROW DSP::FFTWindowStateESP32(window_size, sample_rate, sliding_window_size);
    if (fft == nullptr || fft->_hanning_window == nullptr || fft->_rfft_data == nullptr
        || fft->_freq_bins == nullptr || fft->_derivative_freq_bins == nullptr) {
        delete fft;
        return nullptr;
    }

    const uint16_t fft_size = window_size / 2;
    if (!_twiddle_initialized || _twiddle_fft_size < fft_size) {
        // initialize radix-2 twiddle factors for N/2-point complex FFT
        esp_err_t ret = dsps_fft2r_init_fc32(nullptr, fft_size);
        if (ret != ESP_OK) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ESP32 DSP: fft2r init failed");
            delete fft;
            return nullptr;
        }
        // dsps_cplx2real_fc32 uses fft4r internal tables
        ret = dsps_fft4r_init_fc32(nullptr, fft_size);
        if (ret != ESP_OK) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ESP32 DSP: fft4r init failed");
            delete fft;
            return nullptr;
        }
        _twiddle_fft_size = fft_size;
        _twiddle_initialized = true;
    }

    return fft;
}

void DSP::fft_start(FFTWindowState* state, FloatBuffer& samples, uint16_t advance)
{
    step_hanning((FFTWindowStateESP32*)state, samples, advance);
}

uint16_t DSP::fft_analyse(AP_HAL::DSP::FFTWindowState* state, uint16_t start_bin, uint16_t end_bin, float noise_att_cutoff)
{
    FFTWindowStateESP32* fft = (FFTWindowStateESP32*)state;
    step_fft(fft);
    step_cmplx_mag_esp32(fft, start_bin, end_bin, noise_att_cutoff);
    return step_calc_frequencies_esp32(fft, start_bin, end_bin);
}

DSP::FFTWindowStateESP32::FFTWindowStateESP32(uint16_t window_size, uint16_t sample_rate, uint8_t sliding_window_size)
    : AP_HAL::DSP::FFTWindowState::FFTWindowState(window_size, sample_rate, sliding_window_size)
{
    if (_freq_bins == nullptr || _hanning_window == nullptr || _rfft_data == nullptr || _derivative_freq_bins == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ESP32 DSP: alloc failed w=%u",
            (unsigned)window_size);
    }
    // no additional allocations needed -- we use _freq_bins for in-place FFT
    // and _rfft_data for preserving complex data for the interpolator
}

DSP::FFTWindowStateESP32::~FFTWindowStateESP32()
{
    // base class destructor handles all deallocation
}

// step 1: apply Hanning window to incoming samples
void DSP::step_hanning(FFTWindowStateESP32* fft, FloatBuffer& samples, uint16_t advance)
{
    // peek samples into _freq_bins
    uint32_t read_window = samples.peek(&fft->_freq_bins[0], fft->_window_size);
    if (read_window != fft->_window_size) {
        return;
    }
    samples.advance(advance);

    // element-wise multiply by Hanning window using esp-dsp
    dsps_mul_f32(fft->_freq_bins, fft->_hanning_window, fft->_freq_bins, fft->_window_size, 1, 1, 1);
}

// step 2: perform real FFT using esp-dsp
// All operations are in-place in _freq_bins (N floats = N/2 complex pairs).
// After the FFT, complex data is copied to _rfft_data for Quinn's estimator,
// then _freq_bins is overwritten with magnitude-squared values.
void DSP::step_fft(FFTWindowStateESP32* fft)
{
    const uint16_t N = fft->_window_size;
    const uint16_t Nhalf = N / 2;

    // _freq_bins already contains N windowed real samples from step_hanning.
    // esp-dsp treats these N reals as N/2 interleaved complex pairs.

    // N/2-point complex FFT (in-place)
    dsps_fft2r_fc32(fft->_freq_bins, Nhalf);

    // bit-reverse the output (in-place, radix-2 real variant)
    dsps_bit_rev2r_fc32(fft->_freq_bins, Nhalf);

    // convert to real FFT spectrum (in-place)
    // output: [0]=DC, [1]=Nyquist, [2*k]=Re(k), [2*k+1]=Im(k) for k=1..N/2-1
    dsps_cplx2real_fc32(fft->_freq_bins, Nhalf);

    // copy complex data to _rfft_data for Quinn's frequency interpolator
    // _rfft_data layout matches CMSIS stage_rfft_f32 output
    memcpy(fft->_rfft_data, fft->_freq_bins, sizeof(float) * N);
    // Nyquist bin at end for the interpolator (matches ChibiOS DSP.cpp:230-231)
    fft->_rfft_data[N] = fft->_freq_bins[1];  // Nyquist real
    fft->_rfft_data[N + 1] = 0;               // Nyquist imag = 0

    // compute magnitude squared into _freq_bins, overwriting the complex data
    // save bins whose complex data would be overwritten before being read
    const float dc = fft->_freq_bins[0];
    const float nyquist = fft->_freq_bins[1];
    const float bin1_re = fft->_freq_bins[2];
    const float bin1_im = fft->_freq_bins[3];

    // bins N/2-1 down to 2 (output index i < input index i*2, so safe)
    for (uint16_t i = fft->_bin_count - 1; i >= 2; i--) {
        const float re = fft->_freq_bins[i * 2];
        const float im = fft->_freq_bins[i * 2 + 1];
        fft->_freq_bins[i] = re * re + im * im;
    }
    fft->_freq_bins[1] = bin1_re * bin1_re + bin1_im * bin1_im;
    fft->_freq_bins[0] = sq(dc);
    fft->_freq_bins[fft->_bin_count] = sq(nyquist);
}

// step 3: process magnitudes and find peaks (delegates to base class)
void DSP::step_cmplx_mag_esp32(FFTWindowStateESP32* fft, uint16_t start_bin, uint16_t end_bin, float noise_att_cutoff)
{
    step_cmplx_mag(fft, start_bin, end_bin, noise_att_cutoff);
}

// step 4: calculate peak frequencies (delegates to base class)
uint16_t DSP::step_calc_frequencies_esp32(FFTWindowStateESP32* fft, uint16_t start_bin, uint16_t end_bin)
{
    step_calc_frequencies(fft, start_bin, end_bin);
    return fft->_peak_data[CENTER]._bin;
}

// vector operations using esp-dsp where available

void DSP::vector_max_float(const float* vin, uint16_t len, float* maxValue, uint16_t* maxIndex) const
{
    *maxValue = vin[0];
    *maxIndex = 0;
    for (uint16_t i = 1; i < len; i++) {
        if (vin[i] > *maxValue) {
            *maxValue = vin[i];
            *maxIndex = i;
        }
    }
}

void DSP::vector_scale_float(const float* vin, float scale, float* vout, uint16_t len) const
{
    dsps_mulc_f32(vin, vout, len, scale, 1, 1);
}

float DSP::vector_mean_float(const float* vin, uint16_t len) const
{
    float sum = 0.0f;
    for (uint16_t i = 0; i < len; i++) {
        sum += vin[i];
    }
    return sum / len;
}

void DSP::vector_add_float(const float* vin1, const float* vin2, float* vout, uint16_t len) const
{
    dsps_add_f32(vin1, vin2, vout, len, 1, 1, 1);
}

#endif // HAL_WITH_DSP
