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

---

## 7. S3 Vector Optimization for `dsps_cplx2real_fc32` -- the last scalar FFT step

### Summary

The real-FFT pipeline on ESP32-S3 is: `fft2r` (aes3 vectorized) -> `bit_rev2r` (aes3 vectorized) ->
`cplx2real` (ae32 scalar only). The cplx2real step is the remaining non-vectorized link in the
chain. Upstream esp-dsp has an aes3 version of `dsps_fft4r_fc32` (the radix-4 butterfly), but has
**not** written an aes3 version of `dsps_cplx2real_fc32`.

### Current Implementation: ae32 Scalar Assembly

The function `dsps_cplx2real_fc32_ae32_()` lives in:
`managed_components/espressif__esp-dsp/modules/fft/float/dsps_fft4r_fc32_ae32.c` (lines 142-251)

The ANSI C reference is in:
`managed_components/espressif__esp-dsp/modules/fft/float/dsps_fft4r_fc32_ansi.c` (lines 218-256)

Platform dispatch is in:
`managed_components/espressif__esp-dsp/modules/fft/include/dsps_fft4r_platform.h`
`managed_components/espressif__esp-dsp/modules/fft/include/dsps_fft4r.h`

All paths are relative to `libraries/AP_HAL_ESP32/targets/esp32s3/esp-idf/`.

On ESP32-S3, the ae32 variant is selected because `dsps_fft4r_platform.h` defines
`dsps_cplx2real_fc32_ae32_enabled` when `XCHAL_HAVE_FP && XCHAL_HAVE_LOOPS` (both true on S3).
There is **no** `dsps_cplx2real_fc32_aes3_enabled` define anywhere in the upstream codebase, and
no `dsps_cplx2real_fc32_aes3_()` function declaration or implementation exists.

### What the Algorithm Does

`dsps_cplx2real_fc32` converts the output of an N/2-point complex FFT into the N-point real FFT
spectrum. It is the final post-processing step that "unfolds" the Hermitian-symmetric result.

The algorithm processes N/4 iterations (the loop counter is `fft_points >> 2` where
`fft_points = N/2`, so iterations = N/8 ... but the loop body processes 2 pairs per iteration,
so it covers N/4 complex bins total). Each iteration:

1. **Loads 8 floats**: 4 from the forward pointer (`data[k]` region) and 4 from the reverse
   pointer (`data[N-k]` region), reading 2 consecutive complex pairs from each end.
2. **Computes symmetric/antisymmetric decomposition**: For each pair (k, N-k):
   - `f1k.re = data[k].re + data[N-k].re`  (even part, real)
   - `f1k.im = data[k].im - data[N-k].im`  (even part, imag)
   - `f2k.re = data[k].re - data[N-k].re`  (odd part, real)
   - `f2k.im = data[k].im + data[N-k].im`  (odd part, imag)
3. **Applies twiddle factor** (complex multiply of f2k by table entry):
   - `tw.re = c * f2k.re - s * f2k.im`  (2x mul.s + 1x msub.s)
   - `tw.im = s * f2k.re + c * f2k.im`  (2x mul.s + 1x madd.s)
4. **Combines and scales by 0.5**:
   - `result[k]   = 0.5 * (f1k + tw)`
   - `result[N-k] = 0.5 * (f1k - tw)*`  (conjugated)
5. **Stores 8 floats** back to both forward and reverse positions.

The ae32 assembly uses scalar Xtensa FPU instructions: `lsi`/`ssi`/`ssip` for load/store,
`add.s`/`sub.s`/`mul.s`/`madd.s`/`msub.s` for arithmetic, and `loopnez` for the hardware loop.
It processes 2 complex pairs (4 bins: k, k+1, N-k-1, N-k) per loop iteration.

### Inner Loop Instruction Count (ae32)

Per loop iteration (2 complex pairs processed):
- 8 `lsi` loads (scalar, 32-bit each)
- 8 `ssi`/`ssip` stores (scalar, 32-bit each)
- 4 `add.s` + 4 `sub.s` = 8 add/sub (symmetric decomposition)
- 2 `lsi` twiddle loads per pair = 4 total
- 4 `mul.s` + 2 `madd.s` + 2 `msub.s` = 8 multiply-accumulate (twiddle application)
- 8 `mul.s` for the 0.5 scaling
- 4 `add.s` + 4 `sub.s` = 8 final combine
- 2 address updates (`addx8` for twiddle pointer, `addi` for data pointers)

Total: ~12 loads + 8 stores + ~24 arithmetic + ~4 address ops = ~48 instructions per 4 bins.

### S3 Vectorization Opportunity

The ESP32-S3 `EE.LDF.64.IP` instruction loads two adjacent 32-bit floats in one cycle (a complex
pair). `EE.STF.64.IP` stores two floats in one cycle. The ae32 code already operates on pairs
of complex values per iteration, making the data layout naturally amenable to 64-bit load/stores.

**What S3 vector instructions would help:**

1. **`EE.LDF.64.IP` / `EE.STF.64.IP`**: Replace pairs of `lsi`/`ssi` scalar loads/stores
   with single 64-bit operations. This cuts 8+8=16 scalar memory ops down to 4+4=8 vector
   memory ops (the reverse-pointer loads are non-sequential so gain is partial; forward loads
   and stores are contiguous and fully benefit).

2. The arithmetic (`add.s`, `sub.s`, `mul.s`, `madd.s`, `msub.s`) remains scalar -- the S3 does
   not have SIMD float add/mul that processes two independent floats in parallel (unlike
   integer SIMD). The `EE.MULA.F32.LD` instruction can fuse a load with a multiply-accumulate,
   potentially saving cycles by overlapping memory access with computation.

**What would NOT help:**

The S3 vector extensions are primarily 128-bit integer SIMD (Q registers for int8/int16
operations, useful for neural networks). For 32-bit float, the vector benefit is limited to
wider load/store and load-with-compute fusion. There is no float SIMD add that processes
two f32 values simultaneously. The butterfly math is inherently sequential per complex element.

### Expected Speedup

**Conservative estimate: 15-30% cycle reduction** on the cplx2real step alone.

The gain comes from:
- ~2x memory bandwidth improvement on contiguous loads/stores (saves ~8 cycles per iteration)
- Potential `EE.MULA.F32.LD` fusion saving ~2-4 cycles per iteration on twiddle application
- Better pipeline scheduling with fewer instructions

For context, the fft2r aes3 vs ae32 showed ~13% improvement on ESP32-S3 (8,999 vs ~10,342
cycles for N=128). The fft4r aes3 showed 14% improvement (13,213 vs ~15,348 for N=256).
The cplx2real function has a simpler loop structure with more contiguous memory access,
so it may benefit slightly more from 64-bit loads.

**Impact on ArduPilot's FFT pipeline:**

ArduPilot uses window sizes of 32-256 (default 32-64 on ESP32). The cplx2real step processes
N/2 complex points, so for N=256 that is 128 complex points = 32 loop iterations. At an
estimated 48 cycles/iteration (ae32), the total is ~1,536 cycles = ~6.4us at 240MHz.

A 25% improvement saves ~384 cycles = ~1.6us per FFT call. With FFT running at ~10-20Hz
per axis and 3 axes, that is ~48-96us/sec saved -- negligible in absolute terms but it
eliminates the last scalar bottleneck in the pipeline.

The real value is that it makes cplx2real consistent with the rest of the FFT pipeline
(all aes3-optimized), and any upstream contribution benefits the broader ESP32 ecosystem.

### Upstream Status

- **esp-dsp issue #98** ([link](https://github.com/espressif/esp-dsp/issues/98)): Added
  `dsps_fft4r_fc32_aes3_.S` (radix-4 butterfly) but did NOT add `dsps_cplx2real_fc32_aes3`.
  Status: Done/Merged.
- **No open issues or PRs** exist for S3 vectorization of cplx2real as of Feb 2026.
- **`dsps_fft4r_platform.h` (upstream master)**: Defines `dsps_fft4r_fc32_aes3_enabled` for
  S3 but has NO `dsps_cplx2real_fc32_aes3_enabled` define.
- **`dsps_fft4r.h` (upstream master)**: The dispatch for `dsps_cplx2real_fc32` only checks
  `dsps_cplx2real_fc32_ae32_enabled`, falling back to ANSI. No aes3 path exists.
- The ESP32-P4 (RISC-V) has its own `_arp4` variants for fft4r but also lacks a cplx2real
  optimized variant, suggesting Espressif considers cplx2real low priority.

### Effort: Medium (as an upstream contribution)

Writing `dsps_cplx2real_fc32_aes3_.S` would require:
1. Translating the ae32 inline assembly to standalone `.S` format using `EE.LDF.64.IP` /
   `EE.STF.64.IP` for paired loads/stores
2. Investigating `EE.MULA.F32.LD` fusion for the twiddle multiply section
3. Adding the `dsps_cplx2real_fc32_aes3_enabled` define to `dsps_fft4r_platform.h`
4. Declaring `dsps_cplx2real_fc32_aes3_()` in `dsps_fft4r.h` and updating the dispatch chain
5. Testing against ANSI reference for correctness (existing test suite covers this)

The fft4r_aes3 assembly file (`dsps_fft4r_fc32_aes3_.S`) on upstream master serves as a
direct template -- it uses the same instruction patterns needed for cplx2real.

This would be best contributed upstream to esp-dsp rather than maintained as a local patch.
