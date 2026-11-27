#include "rx_processing.h"
#include "rx_processing_tables.h"
#include "kaiser_window_q15.h"
#include <stddef.h>

#define CHIRP_HALF_FFT (CHIRP_DSP_FFT_SIZE / 2u)
#define CHIRP_MIN_INPUT_CODE 0u
#define CHIRP_MAX_INPUT_CODE 4095u
#define CHIRP_HARDWARE_CLIP_MARGIN 15u

RxStats rx_analyze(const uint16_t *samples, uint32_t count,
                   uint16_t clip_low, uint16_t clip_high)
{
    RxStats result = {0, 0, 0, 0, 0, false};
    if (samples == 0 || count == 0) return result;

    uint16_t min_code = samples[0];
    uint16_t max_code = samples[0];
    uint64_t sum = 0;
    uint64_t square_sum = 0;
    for (uint32_t i = 0; i < count; ++i) {
        const uint16_t code = samples[i];
        if (code < min_code) min_code = code;
        if (code > max_code) max_code = code;
        if (code <= clip_low || code >= clip_high) result.clipped = true;
        sum += code;
    }

    const uint32_t mean = (uint32_t)(sum / count);
    for (uint32_t i = 0; i < count; ++i) {
        const int32_t centered = (int32_t)samples[i] - (int32_t)mean;
        square_sum += (uint64_t)(centered * (int64_t)centered);
    }

    result.min_code = min_code;
    result.max_code = max_code;
    result.mean_code = mean;
    result.peak_to_peak = (uint32_t)max_code - (uint32_t)min_code;
    result.rms_centered = 0;
    if (count != 0) {
        uint64_t mean_square = square_sum / count;
        uint32_t root = 0;
        while ((uint64_t)(root + 1) * (root + 1) <= mean_square) ++root;
        result.rms_centered = root;
    }
    return result;
}

void rx_remove_bias(const uint16_t *input, int32_t *output,
                    uint32_t count, uint16_t bias_code)
{
    if (input == 0 || output == 0) return;
    for (uint32_t i = 0; i < count; ++i)
        output[i] = (int32_t)input[i] - (int32_t)bias_code;
}

/* Q15 I/Q dechirp -> 129-tap low-pass -> x16 decimation -> 1024-point Q15 FFT.
   Static workspace is about 12.8 KB; no 16k-sample float/complex array is allocated. */
static int16_t chirp_i_history[CHIRP_DSP_FIR_TAPS];
static int16_t chirp_q_history[CHIRP_DSP_FIR_TAPS];
static int16_t chirp_baseband_iq[2u * CHIRP_DSP_FFT_SIZE];
static int32_t chirp_fft_interleaved[2u * CHIRP_DSP_FFT_SIZE];
static uint64_t chirp_noise_powers[CHIRP_HALF_FFT];

#define CHIRP_CZT_POINTS 33u
#define CHIRP_METHODS_PER_WINDOW 3u

static int16_t chirp_lookup_sin(uint32_t phase)
{
    const uint32_t index = phase >> 24;
    const uint32_t fraction = (phase >> 8) & 0xFFFFu;
    const int32_t a = chirp_sine_q15[index];
    const int32_t b = chirp_sine_q15[(index + 1u) & 0xFFu];
    const int64_t delta = (int64_t)(b - a) * fraction;
    int32_t value = a + (int32_t)((delta + 32768) >> 16);
    if (value > 32767) value = 32767;
    if (value < -32768) value = -32768;
    return (int16_t)value;
}

static int16_t chirp_lookup_cos(uint32_t phase)
{
    return chirp_lookup_sin(phase + 0x40000000u);
}

static int32_t chirp_window_q15(uint32_t index, bool kaiser)
{
    if (kaiser) return chirp_kaiser_beta4_q15[index];
    const uint32_t phase = (uint32_t)(((uint64_t)index << 32u) /
                                      (CHIRP_DSP_FFT_SIZE - 1u));
    return (32767 - chirp_lookup_cos(phase)) / 2;
}

static uint64_t chirp_fft_power(uint32_t bin)
{
    const int64_t re = chirp_fft_interleaved[2u * bin];
    const int64_t im = chirp_fft_interleaved[2u * bin + 1u];
    return (uint64_t)(re * re) + (uint64_t)(im * im);
}

static void chirp_fft_q15_1024(int32_t *data)
{
    uint32_t j = 0u;
    for (uint32_t i = 1u; i < CHIRP_DSP_FFT_SIZE; ++i) {
        uint32_t bit = CHIRP_DSP_FFT_SIZE >> 1u;
        while ((j & bit) != 0u) { j ^= bit; bit >>= 1u; }
        j ^= bit;
        if (i < j) {
            int32_t tmp = data[2u*i]; data[2u*i] = data[2u*j]; data[2u*j] = tmp;
            tmp = data[2u*i+1u]; data[2u*i+1u] = data[2u*j+1u]; data[2u*j+1u] = tmp;
        }
    }

    for (uint32_t length = 2u; length <= CHIRP_DSP_FFT_SIZE; length <<= 1u) {
        const uint32_t half = length >> 1u;
        for (uint32_t block = 0u; block < CHIRP_DSP_FFT_SIZE; block += length) {
            for (uint32_t k = 0u; k < half; ++k) {
                const uint32_t phase = (uint32_t)(0ull - (((uint64_t)k << 32u) / length));
                const int32_t wr = chirp_lookup_cos(phase);
                const int32_t wi = chirp_lookup_sin(phase);
                const uint32_t even = block + k, odd = even + half;
                const int32_t ar = data[2u*even], ai = data[2u*even+1u];
                const int32_t br = data[2u*odd], bi = data[2u*odd+1u];
                const int32_t tr = (int32_t)(((int64_t)wr*br - (int64_t)wi*bi) >> 15u);
                const int32_t ti = (int32_t)(((int64_t)wr*bi + (int64_t)wi*br) >> 15u);
                data[2u*even] = ar + tr; data[2u*even+1u] = ai + ti;
                data[2u*odd] = ar - tr; data[2u*odd+1u] = ai - ti;
            }
        }
    }
}

static void chirp_load_fft_window(bool kaiser)
{
    for (uint32_t i = 0u; i < CHIRP_DSP_FFT_SIZE; ++i) {
        const int32_t window = chirp_window_q15(i, kaiser);
        const int32_t in_i = chirp_baseband_iq[2u * i];
        const int32_t in_q = chirp_baseband_iq[2u * i + 1u];
        chirp_fft_interleaved[2u * i] = (int32_t)(((int64_t)in_i * window) >> 11u);
        chirp_fft_interleaved[2u * i + 1u] = (int32_t)(((int64_t)in_q * window) >> 11u);
    }
}

static uint32_t chirp_log2_power_q12(uint64_t value)
{
    if (value == 0u) return 0u;
    uint32_t exponent = 0u;
    uint64_t scan = value;
    while (scan >>= 1u) ++exponent;
    uint64_t normalized = (exponent >= 31u) ? (value >> (exponent - 31u)) :
                           (value << (31u - exponent));
    uint32_t fraction = 0u;
    for (uint32_t bit = 0u; bit < 12u; ++bit) {
        normalized = (normalized * normalized) >> 31u;
        if (normalized >= (2ull << 31u)) {
            normalized >>= 1u;
            fraction |= 1u << (11u - bit);
        }
    }
    return (exponent << 12u) | fraction;
}

static bool chirp_bin_in_band(uint32_t bin, uint32_t fft_rate_hz,
                              uint32_t min_beat_millihz,
                              uint32_t max_beat_millihz)
{
    const uint32_t frequency_millihz = (uint32_t)(((uint64_t)bin * fft_rate_hz * 1000u) /
                                                   CHIRP_DSP_FFT_SIZE);
    return (frequency_millihz >= min_beat_millihz) &&
           (frequency_millihz <= max_beat_millihz);
}

static bool chirp_find_peak(uint32_t fft_rate_hz, uint32_t min_beat_millihz,
                            uint32_t max_beat_millihz, int32_t *signed_bin,
                            uint64_t *peak_power, uint32_t *band_count)
{
    *signed_bin = 0;
    *peak_power = 0u;
    *band_count = 0u;
    for (uint32_t k = 1u; k < CHIRP_HALF_FFT; ++k) {
        if (!chirp_bin_in_band(k, fft_rate_hz, min_beat_millihz, max_beat_millihz)) continue;
        const uint64_t pos = chirp_fft_power(k);
        const uint64_t neg = chirp_fft_power(CHIRP_DSP_FFT_SIZE - k);
        ++(*band_count);
        if (pos > *peak_power) { *peak_power = pos; *signed_bin = (int32_t)k; }
        if (neg > *peak_power) { *peak_power = neg; *signed_bin = -(int32_t)k; }
    }
    return (*band_count >= 5u) && (*signed_bin != 0) && (*peak_power != 0u);
}

static uint32_t chirp_bin_frequency_millihz(int32_t signed_bin, uint32_t fft_rate_hz)
{
    const int64_t value = (int64_t)signed_bin * fft_rate_hz * 1000ll / CHIRP_DSP_FFT_SIZE;
    return (uint32_t)((value < 0) ? -value : value);
}

static uint32_t chirp_log_interpolated_frequency_millihz(int32_t signed_bin,
                                                          uint32_t fft_rate_hz)
{
    const uint32_t center = (signed_bin >= 0) ? (uint32_t)signed_bin :
                           CHIRP_DSP_FFT_SIZE - (uint32_t)(-signed_bin);
    const uint32_t left = (center + CHIRP_DSP_FFT_SIZE - 1u) & (CHIRP_DSP_FFT_SIZE - 1u);
    const uint32_t right = (center + 1u) & (CHIRP_DSP_FFT_SIZE - 1u);
    const uint32_t log_left = chirp_log2_power_q12(chirp_fft_power(left));
    const uint32_t log_center = chirp_log2_power_q12(chirp_fft_power(center));
    const uint32_t log_right = chirp_log2_power_q12(chirp_fft_power(right));
    const int32_t denominator = (int32_t)log_left - 2 * (int32_t)log_center + (int32_t)log_right;
    int32_t delta_q15 = 0;
    if (denominator != 0)
        delta_q15 = (int32_t)(((int64_t)((int32_t)log_left - (int32_t)log_right) * 16384ll) /
                              denominator);
    if (delta_q15 > 16384) delta_q15 = 16384;
    if (delta_q15 < -16384) delta_q15 = -16384;
    int64_t bin_position_q15 = ((int64_t)signed_bin * 32768ll) + delta_q15;
    int64_t frequency_millihz = bin_position_q15 * fft_rate_hz * 1000ll /
                                ((int64_t)CHIRP_DSP_FFT_SIZE << 15u);
    if (frequency_millihz < 0) frequency_millihz = -frequency_millihz;
    return (uint32_t)frequency_millihz;
}

/* Zoom-CZT-equivalent evaluation on the unit circle: direct DFT samples a
   dense 33-point frequency arc spanning the coarse peak's FFT-bin interval. */
static uint32_t chirp_czt_dense_frequency_millihz(int32_t signed_bin,
                                                  uint32_t fft_rate_hz,
                                                  uint32_t min_beat_millihz,
                                                  uint32_t max_beat_millihz,
                                                  bool kaiser)
{
    const uint32_t center_hz_milli = chirp_bin_frequency_millihz(signed_bin, fft_rate_hz);
    const uint32_t half_bin_milli = (fft_rate_hz * 1000u) /
                                    (2u * CHIRP_DSP_FFT_SIZE);
    uint32_t low = (center_hz_milli > half_bin_milli) ? center_hz_milli - half_bin_milli : 0u;
    uint32_t high = center_hz_milli + half_bin_milli;
    if (low < min_beat_millihz) low = min_beat_millihz;
    if (high > max_beat_millihz) high = max_beat_millihz;
    if (high <= low) return center_hz_milli;

    const int32_t sign = (signed_bin < 0) ? -1 : 1;
    uint64_t best_power = 0u;
    uint32_t best_frequency = center_hz_milli;
    for (uint32_t point = 0u; point < CHIRP_CZT_POINTS; ++point) {
        const uint32_t frequency_millihz = low + (uint32_t)(((uint64_t)(high - low) * point +
                                                              (CHIRP_CZT_POINTS - 1u) / 2u) /
                                                             (CHIRP_CZT_POINTS - 1u));
        const int64_t signed_frequency = (int64_t)sign * frequency_millihz;
        const uint32_t phase_step = (uint32_t)((signed_frequency * 4294967296ll) /
                                               ((int64_t)fft_rate_hz * 1000ll));
        uint32_t phase = 0u;
        int64_t acc_re = 0, acc_im = 0;
        for (uint32_t n = 0u; n < CHIRP_DSP_FFT_SIZE; ++n) {
            const int32_t window = chirp_window_q15(n, kaiser);
            const int32_t in_i = (int32_t)(((int64_t)chirp_baseband_iq[2u*n] * window) >> 11u);
            const int32_t in_q = (int32_t)(((int64_t)chirp_baseband_iq[2u*n+1u] * window) >> 11u);
            const int32_t wr = chirp_lookup_cos(phase);
            const int32_t wi = chirp_lookup_sin(phase);
            acc_re += (int64_t)in_i * wr + (int64_t)in_q * wi;
            acc_im += (int64_t)in_q * wr - (int64_t)in_i * wi;
            phase += phase_step;
        }
        const int64_t re = acc_re >> 15u, im = acc_im >> 15u;
        const uint64_t power = (uint64_t)(re * re) + (uint64_t)(im * im);
        if (power > best_power) { best_power = power; best_frequency = frequency_millihz; }
    }
    return best_frequency;
}

static void chirp_sort_power(uint64_t *values, uint32_t count)
{
    for (uint32_t i = 1u; i < count; ++i) {
        const uint64_t value = values[i];
        uint32_t j = i;
        while ((j > 0u) && (values[j - 1u] > value)) {
            values[j] = values[j - 1u];
            --j;
        }
        values[j] = value;
    }
}

static bool chirp_dsp_process_impl(const uint16_t *samples, uint32_t count,
                       uint32_t sample_rate_hz, uint32_t tx_start_sample,
                       uint32_t chirp_start_hz, uint32_t chirp_end_hz,
                       uint32_t chirp_duration_us, uint32_t sound_speed_mm_s,
                       uint32_t min_range_mm, uint32_t max_range_mm,
                       ChirpDspResult *result, uint32_t selected, bool compare_all)
{
    if ((samples == NULL) || (result == NULL)) return false;
    result->mean_code = result->beat_millihz = result->range_mm = 0u;
    result->peak_ratio_x100 = result->clipped_count = 0u;
    result->peak_bin = 0u;
    result->valid = false;
    for (uint32_t method = 0u; method < CHIRP_DSP_METHOD_COUNT; ++method) {
        result->method_beat_millihz[method] = 0u;
        result->method_range_mm[method] = 0u;
    }
    if ((count != CHIRP_DSP_FFT_SIZE * CHIRP_DSP_DECIMATION) ||
        (sample_rate_hz != 500000u) || (tx_start_sample >= count) ||
        (chirp_start_hz == 0u) || (chirp_end_hz <= chirp_start_hz) ||
        (chirp_end_hz >= sample_rate_hz / 2u) || (chirp_duration_us == 0u) ||
        (sound_speed_mm_s == 0u) || (min_range_mm == 0u) ||
        (max_range_mm <= min_range_mm) || (selected >= CHIRP_DSP_METHOD_COUNT)) return false;

    uint64_t sum = 0u;
    for (uint32_t i = 0u; i < count; ++i) {
        sum += samples[i];
        if ((samples[i] <= 15u) || (samples[i] >= 4080u)) ++result->clipped_count;
    }
    result->mean_code = (uint32_t)(sum / count);

    const uint64_t chirp_samples = ((uint64_t)sample_rate_hz * chirp_duration_us + 500000u) / 1000000u;
    if (chirp_samples < 2u) return false;
    /* At 500 kS/s, phase in turns at elapsed sample k is
       f0*k/fs + (f1-f0)*k*k/(fs*duration_us). Q46 keeps the scaled
       frequency numerators within uint64_t when rounding both coefficients. */
    const uint32_t phase_fraction_bits = 46u;
    const uint64_t phase_denominator = (uint64_t)sample_rate_hz * chirp_duration_us;
    const uint64_t linear_phase_q46 =
        (((uint64_t)chirp_start_hz << phase_fraction_bits) + sample_rate_hz / 2u) /
        sample_rate_hz;
    const uint64_t quadratic_phase_q46 =
        (((uint64_t)(chirp_end_hz - chirp_start_hz) << phase_fraction_bits) +
         phase_denominator / 2u) / phase_denominator;
    uint32_t history = 0u, output_index = 0u;
    /* Captures have gaps; the FIR must not inherit the tail of the previous
       frame. This also makes window comparisons independent of call order. */
    for (uint32_t i = 0u; i < CHIRP_DSP_FIR_TAPS; ++i) {
        chirp_i_history[i] = 0;
        chirp_q_history[i] = 0;
    }

    for (uint32_t n = 0u; n < count; ++n) {
        int16_t mix_i = 0, mix_q = 0;
        if (n >= tx_start_sample) {
            /* Evaluate the reference through the RX window for late echoes. */
            const uint32_t elapsed_samples = n - tx_start_sample;
            const uint64_t phase_q46 = elapsed_samples *
                (linear_phase_q46 + quadratic_phase_q46 * elapsed_samples);
            /* Keep the fractional turn in Q32 for the lookup table. */
            const uint32_t phase = (uint32_t)(phase_q46 >> (phase_fraction_bits - 32u));
            const int32_t centered = (int32_t)samples[n] - (int32_t)result->mean_code;
            mix_i = (int16_t)((centered * (int32_t)chirp_lookup_cos(phase)) >> 15u);
            mix_q = (int16_t)(-((centered * (int32_t)chirp_lookup_sin(phase)) >> 15u));
        }
        chirp_i_history[history] = mix_i;
        chirp_q_history[history] = mix_q;
        if (++history == CHIRP_DSP_FIR_TAPS) history = 0u;

        if ((n % CHIRP_DSP_DECIMATION) == (CHIRP_DSP_DECIMATION - 1u)) {
            int64_t acc_i = 0, acc_q = 0;
            uint32_t tap_index = history;
            for (uint32_t tap = 0u; tap < CHIRP_DSP_FIR_TAPS; ++tap) {
                acc_i += (int32_t)chirp_lpf_q15[tap] * chirp_i_history[tap_index];
                acc_q += (int32_t)chirp_lpf_q15[tap] * chirp_q_history[tap_index];
                if (++tap_index == CHIRP_DSP_FIR_TAPS) tap_index = 0u;
            }
            if (output_index >= CHIRP_DSP_FFT_SIZE) return false;
            const int32_t filtered_i = (int32_t)((acc_i + (1ll << 14u)) >> 15u);
            const int32_t filtered_q = (int32_t)((acc_q + (1ll << 14u)) >> 15u);
            chirp_baseband_iq[2u * output_index] = (int16_t)filtered_i;
            chirp_baseband_iq[2u * output_index + 1u] = (int16_t)filtered_q;
            ++output_index;
        }
    }
    if (output_index != CHIRP_DSP_FFT_SIZE) return false;

    const uint32_t slope_hz_s = (uint32_t)(((uint64_t)(chirp_end_hz - chirp_start_hz) * 1000000u) /
                                            chirp_duration_us);
    if (slope_hz_s == 0u) return false;
    const uint32_t min_beat_millihz = (uint32_t)(((uint64_t)2u * slope_hz_s * min_range_mm * 1000u) /
                                                  sound_speed_mm_s);
    const uint32_t max_beat_millihz = (uint32_t)(((uint64_t)2u * slope_hz_s * max_range_mm * 1000u) /
                                                  sound_speed_mm_s);
    const uint32_t fft_rate_hz = sample_rate_hz / CHIRP_DSP_DECIMATION;
    const uint32_t selected_window = selected / CHIRP_METHODS_PER_WINDOW;
    const uint32_t first_window = compare_all ? 0u : selected_window;
    const uint32_t past_last_window = compare_all ? 2u : selected_window + 1u;
    const bool run_log = compare_all || selected == CHIRP_METHOD_HANN_LOG ||
                         selected == CHIRP_METHOD_KAISER_LOG;
    const bool run_czt = compare_all || selected == CHIRP_METHOD_HANN_CZT ||
                         selected == CHIRP_METHOD_KAISER_CZT;
    for (uint32_t window_index = first_window; window_index < past_last_window; ++window_index) {
        const bool kaiser = window_index == 1u;
        chirp_load_fft_window(kaiser);
        chirp_fft_q15_1024(chirp_fft_interleaved);
        int32_t signed_bin;
        uint64_t peak_power;
        uint32_t band_count;
        if (!chirp_find_peak(fft_rate_hz, min_beat_millihz, max_beat_millihz,
                             &signed_bin, &peak_power, &band_count)) return false;
        const uint32_t base = window_index * CHIRP_METHODS_PER_WINDOW;
        const uint32_t bin_frequency = chirp_bin_frequency_millihz(signed_bin, fft_rate_hz);
        const uint32_t interpolated_frequency =
            run_log ? chirp_log_interpolated_frequency_millihz(signed_bin, fft_rate_hz) : 0u;
        const uint32_t dense_frequency = run_czt ?
            chirp_czt_dense_frequency_millihz(signed_bin, fft_rate_hz,
                min_beat_millihz, max_beat_millihz, kaiser) : 0u;
        result->method_beat_millihz[base] = bin_frequency;
        result->method_beat_millihz[base + 1u] = interpolated_frequency;
        result->method_beat_millihz[base + 2u] = dense_frequency;
        for (uint32_t method = 0u; method < CHIRP_METHODS_PER_WINDOW; ++method) {
            result->method_range_mm[base + method] = (uint32_t)(
                ((uint64_t)sound_speed_mm_s * result->method_beat_millihz[base + method]) /
                ((uint64_t)2u * slope_hz_s * 1000u));
        }

        if (window_index == selected_window) {
            uint32_t noise_count = 0u;
            const uint32_t abs_peak_bin = (signed_bin < 0) ? (uint32_t)(-signed_bin) :
                                          (uint32_t)signed_bin;
            for (uint32_t k = 1u; k < CHIRP_HALF_FFT; ++k) {
                if (!chirp_bin_in_band(k, fft_rate_hz, min_beat_millihz, max_beat_millihz)) continue;
                const uint32_t delta = (k > abs_peak_bin) ? k - abs_peak_bin : abs_peak_bin - k;
                if (delta <= 2u) continue;
                const uint64_t pos = chirp_fft_power(k);
                const uint64_t neg = chirp_fft_power(CHIRP_DSP_FFT_SIZE - k);
                chirp_noise_powers[noise_count++] = (pos > neg) ? pos : neg;
            }
            if (noise_count == 0u) return false;
            chirp_sort_power(chirp_noise_powers, noise_count);
            const uint64_t median_power = chirp_noise_powers[noise_count / 2u];
            if (median_power != 0u) {
                const uint64_t ratio = (peak_power * 100u) / median_power;
                result->peak_ratio_x100 = (ratio > UINT32_MAX) ? UINT32_MAX : (uint32_t)ratio;
            }
            result->peak_bin = (uint16_t)abs_peak_bin;
        }
        (void)band_count;
    }
    result->beat_millihz = result->method_beat_millihz[selected];
    result->range_mm = result->method_range_mm[selected];
    result->valid = (result->clipped_count == 0u) &&
                    (result->range_mm >= min_range_mm) && (result->range_mm <= max_range_mm);
    return true;
}

bool chirp_dsp_process_mm_s(const uint16_t *samples, uint32_t count,
    uint32_t sample_rate_hz, uint32_t tx_start_sample, uint32_t chirp_start_hz,
    uint32_t chirp_end_hz, uint32_t chirp_duration_us, uint32_t sound_speed_mm_s,
    uint32_t min_range_mm, uint32_t max_range_mm, ChirpDspResult *result)
{
    return chirp_dsp_process_impl(samples, count, sample_rate_hz, tx_start_sample,
        chirp_start_hz, chirp_end_hz, chirp_duration_us, sound_speed_mm_s,
        min_range_mm, max_range_mm, result, CHIRP_METHOD_HANN_BIN, true);
}

bool chirp_dsp_process_method_mm_s(const uint16_t *samples, uint32_t count,
    uint32_t sample_rate_hz, uint32_t tx_start_sample, uint32_t chirp_start_hz,
    uint32_t chirp_end_hz, uint32_t chirp_duration_us, uint32_t sound_speed_mm_s,
    uint32_t min_range_mm, uint32_t max_range_mm, uint32_t method, ChirpDspResult *result)
{
    return chirp_dsp_process_impl(samples, count, sample_rate_hz, tx_start_sample,
        chirp_start_hz, chirp_end_hz, chirp_duration_us, sound_speed_mm_s,
        min_range_mm, max_range_mm, result, method, false);
}

bool chirp_dsp_process_kaiser_fft_mm_s(const uint16_t *samples, uint32_t count,
    uint32_t sample_rate_hz, uint32_t tx_start_sample, uint32_t chirp_start_hz,
    uint32_t chirp_end_hz, uint32_t chirp_duration_us, uint32_t sound_speed_mm_s,
    uint32_t min_range_mm, uint32_t max_range_mm, ChirpDspResult *result)
{
    return chirp_dsp_process_method_mm_s(samples, count, sample_rate_hz, tx_start_sample,
        chirp_start_hz, chirp_end_hz, chirp_duration_us, sound_speed_mm_s,
        min_range_mm, max_range_mm, CHIRP_METHOD_KAISER_LOG, result);
}

const int16_t *chirp_dsp_baseband_iq(void)
{
    return chirp_baseband_iq;
}

bool chirp_dsp_synthesize_echo_mm_s(uint16_t *samples, uint32_t count,
                               uint32_t sample_rate_hz, uint32_t tx_start_sample,
                               uint32_t chirp_start_hz, uint32_t chirp_end_hz,
                               uint32_t chirp_duration_us, uint32_t target_range_mm,
                               uint32_t sound_speed_mm_s, int16_t echo_peak_codes,
                               uint32_t seed)
{
    if ((samples == NULL) || (count != CHIRP_DSP_FFT_SIZE * CHIRP_DSP_DECIMATION) ||
        (sample_rate_hz != 500000u) || (tx_start_sample >= count) ||
        (chirp_start_hz == 0u) || (chirp_end_hz <= chirp_start_hz) ||
        (chirp_duration_us == 0u) || (target_range_mm == 0u) ||
        (sound_speed_mm_s == 0u) || (echo_peak_codes <= 0)) return false;

    const uint64_t chirp_samples = ((uint64_t)sample_rate_hz * chirp_duration_us + 500000u) / 1000000u;
    const uint32_t delay_samples = (uint32_t)(((uint64_t)2u * target_range_mm * sample_rate_hz +
                                               sound_speed_mm_s / 2u) /
                                              sound_speed_mm_s);
    const uint64_t echo_end = (uint64_t)tx_start_sample + delay_samples + chirp_samples;
    if ((chirp_samples < 2u) || (echo_end > count)) return false;

    uint32_t rng = (seed == 0u) ? 0x4500C0DEu : seed;
    for (uint32_t i = 0u; i < count; ++i) {
        rng = rng * 1664525u + 1013904223u;
        const int32_t noise = (int32_t)(rng >> 29u) - 4;
        samples[i] = (uint16_t)(2039 + noise); /* 8-code peak-to-peak synthetic baseline noise. */
    }

    uint64_t step = ((uint64_t)chirp_start_hz << 32u) / sample_rate_hz;
    const uint64_t step_delta = ((uint64_t)(chirp_end_hz - chirp_start_hz) << 32u) /
        ((uint64_t)sample_rate_hz * (chirp_samples - 1u));
    uint32_t phase = 0u;
    const uint32_t echo_start = tx_start_sample + delay_samples;
    for (uint32_t i = 0u; i < (uint32_t)chirp_samples; ++i) {
        const int32_t echo = (echo_peak_codes * (int32_t)chirp_lookup_sin(phase)) >> 15u;
        const int32_t code = (int32_t)samples[echo_start+i] + echo;
        samples[echo_start+i] = (uint16_t)((code < 0) ? 0 : ((code > 4095) ? 4095 : code));
        phase += (uint32_t)step;
        step += step_delta;
    }
    return true;
}

bool chirp_dsp_process(const uint16_t *samples, uint32_t count,
                       uint32_t sample_rate_hz, uint32_t tx_start_sample,
                       uint32_t chirp_start_hz, uint32_t chirp_end_hz,
                       uint32_t chirp_duration_us, uint32_t sound_speed_m_s,
                       uint32_t min_range_mm, uint32_t max_range_mm,
                       ChirpDspResult *result)
{
    if (sound_speed_m_s > UINT32_MAX / 1000u) return false;
    return chirp_dsp_process_mm_s(samples, count, sample_rate_hz, tx_start_sample,
        chirp_start_hz, chirp_end_hz, chirp_duration_us, sound_speed_m_s * 1000u,
        min_range_mm, max_range_mm, result);
}

bool chirp_dsp_synthesize_echo(uint16_t *samples, uint32_t count,
                               uint32_t sample_rate_hz, uint32_t tx_start_sample,
                               uint32_t chirp_start_hz, uint32_t chirp_end_hz,
                               uint32_t chirp_duration_us, uint32_t target_range_mm,
                               uint32_t sound_speed_m_s, int16_t echo_peak_codes,
                               uint32_t seed)
{
    if (sound_speed_m_s > UINT32_MAX / 1000u) return false;
    return chirp_dsp_synthesize_echo_mm_s(samples, count, sample_rate_hz, tx_start_sample,
        chirp_start_hz, chirp_end_hz, chirp_duration_us, target_range_mm,
        sound_speed_m_s * 1000u, echo_peak_codes, seed);
}
