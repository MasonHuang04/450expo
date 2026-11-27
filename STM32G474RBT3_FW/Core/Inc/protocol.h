#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RX_PROTOCOL_MAGIC 0xA55Au
#define RX_PROTOCOL_VERSION 3u
#define RX_PROTOCOL_HEADER_SIZE 50u
#define RX_PROTOCOL_CRC_SIZE 2u
#define RX_PROTOCOL_MAX_SAMPLES 16384u
#define RX_PROTOCOL_V4_VERSION 4u
#define RX_PROTOCOL_V4_HEADER_SIZE 66u
#define RX_PROTOCOL_PAYLOAD_ADC_U16 0u
#define RX_PROTOCOL_PAYLOAD_IQ_S16  1u
#define RX_PROTOCOL_ALGORITHM_KAISER4_FFT_LOG 4u

#define RX_PROTOCOL_FLAG_CLIPPED 0x0001u
#define RX_PROTOCOL_FLAG_TX_RX   0x0002u
#define RX_PROTOCOL_FLAG_CHIRP   0x0004u
#define RX_PROTOCOL_FLAG_RANGE_CANDIDATE 0x0008u
#define RX_PROTOCOL_FLAG_SIMULATED       0x0010u
#define RX_PROTOCOL_FLAG_TEMP_COMPENSATED 0x0020u
#define RX_PROTOCOL_FLAG_TEMP_UNAVAILABLE 0x0040u
#define RX_PROTOCOL_FLAG_NOMINAL_SPEED    0x0080u /* Explicit 343 m/s bring-up estimate. */

typedef struct {
    uint32_t sequence;
    uint32_t sample_count;
    uint32_t sample_rate_hz;
    uint16_t flags;
    uint32_t tx_start_sample;
    uint32_t chirp_start_hz;
    uint32_t chirp_end_hz;
    uint32_t chirp_duration_us;
    uint32_t range_candidate_mm;
    uint32_t beat_millihz;
    uint32_t peak_ratio_x100;
    uint32_t dsp_cycles;
} RxProtocolHeader;

/* v4 keeps v3's first 50 bytes, then describes the signal being transmitted.
   sample_count counts COMPLEX points for IQ; each point is signed I then Q.
   tx_start_sample is always measured on the original ADC time grid. */
typedef struct {
    RxProtocolHeader common;
    uint32_t adc_sample_rate_hz;
    uint32_t sound_speed_mm_s;
    int32_t temperature_centi_c; /* Meaningful only with TEMP_COMPENSATED. */
    uint16_t payload_format;
    uint16_t algorithm; /* CHIRP_METHOD_* ID, 0 through 5. */
} RxProtocolHeaderV4;

size_t protocol_encode_header_v4(uint8_t *output, size_t capacity,
                                const RxProtocolHeaderV4 *header);

uint16_t protocol_crc16(const uint8_t *data, size_t length);
uint16_t protocol_crc16_update(uint16_t crc, const uint8_t *data, size_t length);
size_t protocol_encode_header(uint8_t *output, size_t capacity,
                              const RxProtocolHeader *header);
bool protocol_decode_header(const uint8_t *input, size_t length,
                            RxProtocolHeader *header);
size_t protocol_encode_frame(uint8_t *output, size_t capacity,
                             const RxProtocolHeader *header,
                             const uint16_t *samples, uint32_t sample_count);

#endif
