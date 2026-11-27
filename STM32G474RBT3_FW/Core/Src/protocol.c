#include "protocol.h"

static void put_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void put_u32_le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

static uint16_t get_u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t get_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

uint16_t protocol_crc16(const uint8_t *data, size_t length)
{
    return protocol_crc16_update(0xFFFFu, data, length);
}

uint16_t protocol_crc16_update(uint16_t crc, const uint8_t *data, size_t length)
{
    if (data == 0) return crc;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0xA001u)
                             : (uint16_t)(crc >> 1);
    }
    return crc;
}

size_t protocol_encode_header(uint8_t *output, size_t capacity,
                              const RxProtocolHeader *header)
{
    if (output == 0 || header == 0 || capacity < RX_PROTOCOL_HEADER_SIZE)
        return 0;
    put_u16_le(output + 0, RX_PROTOCOL_MAGIC);
    output[2] = RX_PROTOCOL_VERSION;
    output[3] = 0;
    put_u32_le(output + 4, header->sequence);
    put_u32_le(output + 8, header->sample_count);
    put_u32_le(output + 12, header->sample_rate_hz);
    put_u16_le(output + 16, header->flags);
    put_u32_le(output + 18, header->tx_start_sample);
    put_u32_le(output + 22, header->chirp_start_hz);
    put_u32_le(output + 26, header->chirp_end_hz);
    put_u32_le(output + 30, header->chirp_duration_us);
    put_u32_le(output + 34, header->range_candidate_mm);
    put_u32_le(output + 38, header->beat_millihz);
    put_u32_le(output + 42, header->peak_ratio_x100);
    put_u32_le(output + 46, header->dsp_cycles);
    return RX_PROTOCOL_HEADER_SIZE;
}

size_t protocol_encode_header_v4(uint8_t *output, size_t capacity,
                                const RxProtocolHeaderV4 *header)
{
    if ((header == 0) || (capacity < RX_PROTOCOL_V4_HEADER_SIZE) ||
        (header->payload_format > RX_PROTOCOL_PAYLOAD_IQ_S16) ||
        (protocol_encode_header(output, capacity, &header->common) == 0u)) return 0u;
    output[2] = RX_PROTOCOL_V4_VERSION;
    put_u32_le(output + 50, header->adc_sample_rate_hz);
    put_u32_le(output + 54, header->sound_speed_mm_s);
    put_u32_le(output + 58, (uint32_t)header->temperature_centi_c);
    put_u16_le(output + 62, header->payload_format);
    put_u16_le(output + 64, header->algorithm);
    return RX_PROTOCOL_V4_HEADER_SIZE;
}

bool protocol_decode_header(const uint8_t *input, size_t length,
                            RxProtocolHeader *header)
{
    if (input == 0 || header == 0 || length < 18u)
        return false;
    if (get_u16_le(input) != RX_PROTOCOL_MAGIC || input[2] < 1u || input[2] > RX_PROTOCOL_V4_VERSION)
        return false;
    const uint8_t version = input[2];
    const uint32_t required = (version == 1u) ? 18u : ((version == 2u) ? 34u :
        ((version == 3u) ? RX_PROTOCOL_HEADER_SIZE : RX_PROTOCOL_V4_HEADER_SIZE));
    if (length < required)
        return false;
    header->sequence = get_u32_le(input + 4);
    header->sample_count = get_u32_le(input + 8);
    header->sample_rate_hz = get_u32_le(input + 12);
    header->flags = get_u16_le(input + 16);
    header->range_candidate_mm = 0u;
    header->beat_millihz = 0u;
    header->peak_ratio_x100 = 0u;
    header->dsp_cycles = 0u;
    if (version == 1u) {
        header->tx_start_sample = 0u;
        header->chirp_start_hz = 0u;
        header->chirp_end_hz = 0u;
        header->chirp_duration_us = 0u;
    } else {
        header->tx_start_sample = get_u32_le(input + 18);
        header->chirp_start_hz = get_u32_le(input + 22);
        header->chirp_end_hz = get_u32_le(input + 26);
        header->chirp_duration_us = get_u32_le(input + 30);
        if (version >= 3u) {
            header->range_candidate_mm = get_u32_le(input + 34);
            header->beat_millihz = get_u32_le(input + 38);
            header->peak_ratio_x100 = get_u32_le(input + 42);
            header->dsp_cycles = get_u32_le(input + 46);
        }
    }
    return true;
}

size_t protocol_encode_frame(uint8_t *output, size_t capacity,
                             const RxProtocolHeader *header,
                             const uint16_t *samples, uint32_t sample_count)
{
    if (output == 0 || header == 0 || samples == 0 ||
        header->sample_count != sample_count || sample_count > RX_PROTOCOL_MAX_SAMPLES)
        return 0;
    const size_t payload_bytes = (size_t)sample_count * 2u;
    const size_t total = RX_PROTOCOL_HEADER_SIZE + payload_bytes +
                         RX_PROTOCOL_CRC_SIZE;
    if (total < payload_bytes || capacity < total) return 0;
    if (protocol_encode_header(output, capacity, header) == 0) return 0;
    for (uint32_t i = 0; i < sample_count; ++i)
        put_u16_le(output + RX_PROTOCOL_HEADER_SIZE + (size_t)i * 2u,
                   samples[i]);
    const uint16_t crc = protocol_crc16(output,
                                        RX_PROTOCOL_HEADER_SIZE + payload_bytes);
    put_u16_le(output + RX_PROTOCOL_HEADER_SIZE + payload_bytes, crc);
    return total;
}
