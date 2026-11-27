#ifndef RX_PROCESSING_H
#define RX_PROCESSING_H

#include <stdbool.h>
#include <stdint.h>

#define CHIRP_DSP_DECIMATION 16u
#define CHIRP_DSP_FIR_TAPS 129u
#define CHIRP_DSP_FFT_SIZE 1024u
#define CHIRP_DSP_OUTPUT_SAMPLES 1024u
/* These IDs index the result arrays and match the UART v4 algorithm field. */
#define CHIRP_METHOD_HANN_BIN 0u
#define CHIRP_METHOD_HANN_LOG 1u
#define CHIRP_METHOD_HANN_CZT 2u
#define CHIRP_METHOD_KAISER_BIN 3u
#define CHIRP_METHOD_KAISER_LOG 4u
#define CHIRP_METHOD_KAISER_CZT 5u
#define CHIRP_DSP_METHOD_COUNT 6u
/* The FFT search band is calculated from the configured range limits and
   chirp slope; 0.1..1.5 m is approximately 117..1749 Hz at 343 m/s. */

typedef struct {
    uint32_t mean_code;
    uint32_t beat_millihz;
    uint32_t range_mm;
    uint32_t peak_ratio_x100; /* Peak-bin power / in-band median power, x100. */
    uint32_t clipped_count;
    uint16_t peak_bin;
    bool valid;
    /* Order: Hann bin/log/CZT, Kaiser beta=4 bin/log/CZT. */
    uint32_t method_beat_millihz[CHIRP_DSP_METHOD_COUNT];
    uint32_t method_range_mm[CHIRP_DSP_METHOD_COUNT];
} ChirpDspResult;

typedef struct {
    uint16_t min_code;
    uint16_t max_code;
    uint32_t mean_code;
    uint32_t peak_to_peak;
    uint32_t rms_centered;
    bool clipped;
} RxStats;

RxStats rx_analyze(const uint16_t *samples, uint32_t count,
                   uint16_t clip_low, uint16_t clip_high);
void rx_remove_bias(const uint16_t *input, int32_t *output,
                    uint32_t count, uint16_t bias_code);

bool chirp_dsp_process(const uint16_t *samples, uint32_t count,
                       uint32_t sample_rate_hz, uint32_t tx_start_sample,
                       uint32_t chirp_start_hz, uint32_t chirp_end_hz,
                       uint32_t chirp_duration_us, uint32_t sound_speed_m_s,
                       uint32_t min_range_mm, uint32_t max_range_mm,
                       ChirpDspResult *result);

/* Temperature-compensated entry point. mm/s retains fractional m/s precision.
   The m/s entry point above remains available for existing host tests/tools. */
bool chirp_dsp_process_mm_s(const uint16_t *samples, uint32_t count,
                          uint32_t sample_rate_hz, uint32_t tx_start_sample,
                          uint32_t chirp_start_hz, uint32_t chirp_end_hz,
                          uint32_t chirp_duration_us, uint32_t sound_speed_mm_s,
                          uint32_t min_range_mm, uint32_t max_range_mm,
                          ChirpDspResult *result);

/* Live ranging with one window and one selected peak estimator.
   The coarse FFT bin is also populated for the selected window. */
bool chirp_dsp_process_method_mm_s(const uint16_t *samples, uint32_t count,
    uint32_t sample_rate_hz, uint32_t tx_start_sample, uint32_t chirp_start_hz,
    uint32_t chirp_end_hz, uint32_t chirp_duration_us, uint32_t sound_speed_mm_s,
    uint32_t min_range_mm, uint32_t max_range_mm, uint32_t method, ChirpDspResult *result);

/* Kaiser beta=4 -> FFT -> three-bin log-power interpolation.
   Only method slots 3 and 4 are populated; no dense DFT/CZT is evaluated.
   The legacy entry point above retains the full six-method comparison. */
bool chirp_dsp_process_kaiser_fft_mm_s(const uint16_t *samples, uint32_t count,
    uint32_t sample_rate_hz, uint32_t tx_start_sample, uint32_t chirp_start_hz,
    uint32_t chirp_end_hz, uint32_t chirp_duration_us, uint32_t sound_speed_mm_s,
    uint32_t min_range_mm, uint32_t max_range_mm, ChirpDspResult *result);

/* 1024 unwindowed complex samples, interleaved signed I,Q, 31.25 ksample/s.
   Valid after successful processing, until the next processing call. */
const int16_t *chirp_dsp_baseband_iq(void);

/* Software-only echo generator for MCU DSP/timing tests. It never starts HRTIM. */
bool chirp_dsp_synthesize_echo(uint16_t *samples, uint32_t count,
                               uint32_t sample_rate_hz, uint32_t tx_start_sample,
                               uint32_t chirp_start_hz, uint32_t chirp_end_hz,
                               uint32_t chirp_duration_us, uint32_t target_range_mm,
                               uint32_t sound_speed_m_s, int16_t echo_peak_codes,
                               uint32_t seed);

bool chirp_dsp_synthesize_echo_mm_s(uint16_t *samples, uint32_t count,
                                 uint32_t sample_rate_hz, uint32_t tx_start_sample,
                                 uint32_t chirp_start_hz, uint32_t chirp_end_hz,
                                 uint32_t chirp_duration_us, uint32_t target_range_mm,
                                 uint32_t sound_speed_mm_s, int16_t echo_peak_codes,
                                 uint32_t seed);

#endif
