# ESP32 esp-dsp Optimization Opportunities

Future optimizations using Espressif's esp-dsp library beyond the current FFT backend.
Speedup estimates are for ESP32-S3 with vector extensions at 240MHz.
Plain ESP32 (Xtensa LX6) will see smaller gains (no vector extensions).

## 1. Hanning Window Generation -- `dsps_wind_hann_f32()`

**Current:** Base class `AP_HAL::DSP::FFTWindowState` constructor generates the Hanning
window coefficients in a manual loop (`0.5 - 0.5 * cosf(2*PI*i/N)`).

**Optimization:** Replace with `dsps_wind_hann_f32(window, N)` which uses optimized
math and S3 vector instructions.

**Speedup:** ~2x for window generation. This is a one-time init cost per FFT instance,
so the real-world impact is minimal (saves ~50us on a 256-point init).

**Effort:** Low. Single function call replacement. Requires overriding the base class
window generation or adding an ESP32-specific post-init step.

**Files:** `libraries/AP_HAL_ESP32/DSP.cpp` (in FFTWindowStateESP32 constructor)

---

## 2. Tone Generation for Self-Test -- `dsps_tone_gen_f32()`

**Current:** DSP_test and AP_GyroFFT::self_test() generate test signals using `sinf()`
in a loop. Each `sinf()` call takes ~100-200 cycles on ESP32.

**Optimization:** Use `dsps_tone_gen_f32()` which generates sine waves using a
recursive oscillator (2 multiplies + 1 add per sample vs trigonometric computation).

**Speedup:** ~3-5x for signal generation on S3. For a 256-sample buffer:
- Current: ~25-50us (256 * sinf calls)
- Optimized: ~5-10us (recursive oscillator)

**Effort:** Low. Replace sinf loop with dsps_tone_gen_f32() call. The API is:
`dsps_tone_gen_f32(output, len, amplitude, frequency/sample_rate, phase)`

**Files:** `libraries/AP_HAL/examples/DSP_test/DSP_test.cpp`,
`libraries/AP_GyroFFT/AP_GyroFFT.cpp` (self_test method)

---

## 3. SNR Measurement for FFT Validation -- `dsps_snr_f32()`

**Current:** No quantitative FFT quality metric. Tests only check peak frequency
detection, not the overall spectral quality.

**Optimization:** Use `dsps_snr_f32(signal, reference, len)` to compute
signal-to-noise ratio of FFT output against a known reference spectrum.

**Speedup:** N/A -- this is a new diagnostic capability, not a speed optimization.
Useful for validating FFT accuracy across platforms and window sizes.

**Effort:** Low. Add SNR computation to DSP_test after FFT analysis. Compare
against expected spectrum of known test signals. Expected SNR > 40dB for
clean single-frequency input.

**Files:** `libraries/AP_HAL/examples/DSP_test/DSP_test.cpp`

---

## 4. Batch Biquad Filtering -- `dsps_biquad_f32()`

**Current:** ArduPilot's `DigitalBiquadFilter::apply()` processes one sample at a time.
The notch filter in AP_InertialSensor calls `apply()` per gyro sample as they arrive.
Each call has function overhead that dominates the actual filter math.

**Optimization:** Buffer N samples (8-32), then process as a batch with
`dsps_biquad_f32(input, output, len, coeffs, delay_line)`. esp-dsp's implementation
uses S3 vector instructions to process 4 samples per cycle.

**Speedup:** ~5-10x per filter operation. For a 16-sample batch on S3:
- Current: ~16 * 200ns = 3.2us (16 individual apply() calls)
- Optimized: ~0.3-0.6us (vectorized batch processing)
With 3 axes * 2-3 notch filters = 6-9 filters running, this saves ~15-25us
per IMU batch, which is meaningful at 1kHz+ sample rates.

**Effort:** HIGH. Requires architectural changes:
1. Add `apply_batch(float* samples, uint16_t count)` to `DigitalBiquadFilter`
2. Modify IMU driver to accumulate samples before filtering
3. Handle filter state (delay line) correctly across batches
4. Ensure latency is acceptable (buffering adds ~8-32 sample delay)
ArduPilot's filter pipeline is fundamentally per-sample. This is a significant
redesign, not a drop-in replacement. Only worthwhile if ESP32 CPU budget is tight.

**Files:** `libraries/Filter/DigitalBiquadFilter.h/.cpp`,
`libraries/AP_InertialSensor/AP_InertialSensor.cpp`

---

## 5. Vector Dot Product for Magnitude -- `dsps_dotprod_f32()`

**Current:** FFT magnitude-squared is computed with a manual loop:
`freq_bins[i] = re*re + im*im` iterating over N/2 bins.

**Optimization:** Use `dsps_dotprod_f32()` to compute dot products of
real and imaginary components, or restructure as a single `dsps_mulc_f32` +
`dsps_add_f32` pipeline on split real/imaginary arrays.

**Speedup:** ~2-3x for the magnitude loop on S3. For 128 bins:
- Current: ~2-3us (manual loop, 128 iterations)
- Optimized: ~0.8-1.2us (vectorized)

**Effort:** Medium. The current in-place layout (interleaved re/im) doesn't
map directly to dot product. Options:
a) Deinterleave to separate re[] and im[] arrays, dot-product each, then add
   (extra memory + copy overhead may negate gains)
b) Use `dsps_mul_f32` with step=2 to square in-place, then sum adjacent pairs
   (needs careful stride handling)

**Files:** `libraries/AP_HAL_ESP32/DSP.cpp` (step_fft method)

---

## 6. FIR Decimation Filter -- `dsps_fir_f32()`

**Current:** No decimation filter in the FFT pipeline. The sample rate fed to FFT
equals the IMU sample rate (typically 1kHz on ESP32). Frequency resolution is
determined by window_size / sample_rate.

**Optimization:** Add a FIR anti-aliasing + decimation filter before FFT. This
allows using a smaller FFT window while maintaining frequency resolution in the
band of interest (typically 50-500Hz for vibration analysis).

Example: 1kHz input, 2:1 decimation -> 500Hz effective rate. A 64-point FFT
at 500Hz gives the same resolution as 128-point at 1kHz, using half the memory.

**Speedup:** N/A -- enables better memory/resolution tradeoff, not raw speed.
esp-dsp's FIR is ~3-5x faster than a manual convolution loop on S3.
A 32-tap FIR at 1kHz costs ~5-10us per sample batch (negligible).

**Effort:** Medium. Requires:
1. Design appropriate FIR filter coefficients for the target band
2. Add decimation stage between sample buffer and FFT input
3. Integrate with AP_GyroFFT's sample collection logic
4. Adjust frequency bin mapping for the decimated rate

**Files:** `libraries/AP_HAL_ESP32/DSP.cpp` (new decimation step),
`libraries/AP_GyroFFT/AP_GyroFFT.cpp` (sample collection)
