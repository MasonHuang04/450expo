"""Decoder for Core/Inc/protocol.h v1/v2/v3/v4. No third-party dependencies.

Little endian: magic u16, version u8, reserved u8, sequence u32,
count u32, sample rate u32, flags u16, v2 chirp metadata, count*u16 samples,
CRC16/Modbus.
CRC covers the header and samples; its trailer is also little endian.
"""
from dataclasses import dataclass
import csv
import math
from pathlib import Path
import struct

MAGIC = b'\x5a\xa5'
HEADER_V1 = struct.Struct('<HBBIIIH')
HEADER_V2 = struct.Struct('<HBBIIIHIIII')
HEADER_V3 = struct.Struct('<HBBIIIHIIIIIIII')
HEADER_V4 = struct.Struct('<HBBIIIHIIIIIIIIIIiHH')
HEADER = HEADER_V3  # Current encoder fixture/API uses the v3 header.
MAX_SAMPLES = 16384
FLAG_TX_RX = 0x0002
FLAG_CHIRP = 0x0004
FLAG_RANGE_CANDIDATE = 0x0008
FLAG_SIMULATED = 0x0010
FLAG_TEMP_COMPENSATED = 0x0020
FLAG_TEMP_UNAVAILABLE = 0x0040
FLAG_NOMINAL_SPEED = 0x0080


def crc16(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc


@dataclass(frozen=True)
class Frame:
    sequence: int
    sample_rate_hz: int
    flags: int
    samples: tuple
    demo: bool = False
    tx_start_sample: int = 0
    chirp_start_hz: int = 0
    chirp_end_hz: int = 0
    chirp_duration_us: int = 0
    range_candidate_mm: int = 0
    beat_millihz: int = 0
    peak_ratio_x100: int = 0
    dsp_cycles: int = 0
    adc_sample_rate_hz: int = 0
    sound_speed_mm_s: int = 0
    temperature_centi_c: int = 0
    payload_format: int = 0  # 0: unsigned raw ADC; 1: signed interleaved I/Q.
    # IDs 0..5: Hann bin/log/CZT, then Kaiser beta=4 bin/log/CZT.
    algorithm: int = 65535
    samples_q: tuple = ()


def temperature_status(frame):
    """Describe received metadata without inventing a temperature value."""
    if frame.demo or frame.flags & FLAG_SIMULATED:
        return 'simulated'
    compensated = bool(frame.flags & FLAG_TEMP_COMPENSATED)
    unavailable = bool(frame.flags & FLAG_TEMP_UNAVAILABLE)
    nominal = bool(frame.flags & FLAG_NOMINAL_SPEED)
    if (compensated and (unavailable or nominal)) or (nominal and not unavailable):
        return 'inconsistent'
    if nominal:
        return 'nominal'
    if unavailable:
        return 'unavailable'
    if compensated:
        return 'compensated'
    return 'not_reported'


def range_precise_mm(frame):
    """Retain sub-mm digits using the MCU's beat frequency in milliHz and frame speed.

    v3's integer-mm field remains untouched. v4 already carries every input
    to R = c*f*T/(2*B); this restores its fractional part without redoing FFT.
    """
    if not (frame.flags & FLAG_RANGE_CANDIDATE) or temperature_status(frame) in ('unavailable', 'inconsistent'):
        return None
    bandwidth = frame.chirp_end_hz - frame.chirp_start_hz
    if (frame.adc_sample_rate_hz and frame.sound_speed_mm_s > 0 and
            bandwidth > 0 and frame.chirp_duration_us > 0):
        return (frame.sound_speed_mm_s * frame.beat_millihz * frame.chirp_duration_us /
                (2.0 * bandwidth * 1_000_000_000.0))
    return float(frame.range_candidate_mm)


class FrameParser:
    """Accept arbitrary serial chunks, discard garbage and reject bad CRCs."""
    def __init__(self):
        self.buffer = bytearray()
        self.bad_crc = 0
        self.bad_headers = 0
        self.discarded_bytes = 0
        self.last_header_error = ''
        self.last_header_hex = ''

    def clear_partial(self):
        self.discarded_bytes += len(self.buffer)
        self.buffer.clear()

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while True:
            start = self.buffer.find(MAGIC)
            if start < 0:
                keep = 1 if self.buffer.endswith(MAGIC[:1]) else 0
                drop = len(self.buffer) - keep
                del self.buffer[:drop]
                self.discarded_bytes += drop
                break
            if start:
                del self.buffer[:start]
                self.discarded_bytes += start
            if len(self.buffer) < HEADER_V1.size:
                break
            magic, version, reserved, seq, count, rate, flags = HEADER_V1.unpack_from(self.buffer)
            header_struct = {1: HEADER_V1, 2: HEADER_V2, 3: HEADER_V3, 4: HEADER_V4}.get(version, HEADER_V3)
            header_size = header_struct.size
            if version not in (1, 2, 3, 4) or reserved != 0 or not 1 <= count <= MAX_SAMPLES or not 1 <= rate <= 10_000_000:
                self.last_header_error = (
                    f'fixed header: magic=0x{magic:04X}, version={version}, reserved={reserved}, '
                    f'sequence={seq}, count={count}, rate={rate}, flags=0x{flags:04X}')
                self.last_header_hex = self.buffer[:HEADER_V3.size].hex(' ')
                del self.buffer[0]
                self.bad_headers += 1
                continue
            if len(self.buffer) < header_size:
                break
            range_mm = beat_millihz = peak_ratio_x100 = dsp_cycles = 0
            adc_rate = speed = temperature = payload_format = 0
            algorithm = 65535
            if version >= 2:
                fields = header_struct.unpack_from(self.buffer)
                tx_start, f_start, f_end, duration_us = fields[7:11]
                if version >= 3:
                    range_mm, beat_millihz, peak_ratio_x100, dsp_cycles = fields[11:15]
                if version == 4:
                    adc_rate, speed, temperature, payload_format, algorithm = fields[15:20]
                    if not (payload_format in (0, 1) and 1 <= adc_rate <= 10_000_000 and
                            rate <= adc_rate and adc_rate % rate == 0 and
                            (payload_format != 1 or (flags & FLAG_CHIRP and count == 1024))):
                        self.last_header_error = 'Invalid v4 signal format or ADC/sample-rate relationship'
                        self.last_header_hex = self.buffer[:header_size].hex(' ')
                        del self.buffer[0]
                        self.bad_headers += 1
                        continue
                if flags & FLAG_CHIRP:
                    source_rate = adc_rate or rate
                    source_count = count * source_rate // rate
                    if not (0 < f_start < f_end < source_rate // 2 and 0 < duration_us <= 1_000_000 and tx_start < source_count):
                        self.last_header_error = (
                            f'chirp metadata: rate={rate}, flags=0x{flags:04X}, tx={tx_start}, '
                            f'f_start={f_start}, f_end={f_end}, duration_us={duration_us}, count={count}')
                        self.last_header_hex = self.buffer[:header_size].hex(' ')
                        del self.buffer[0]
                        self.bad_headers += 1
                        continue
                else:
                    f_start = f_end = duration_us = 0
                    if not flags & FLAG_TX_RX:
                        tx_start = 0
            else:
                tx_start = f_start = f_end = duration_us = 0
            total = header_size + count * (4 if payload_format == 1 else 2) + 2
            if len(self.buffer) < total:
                break
            expected = struct.unpack_from('<H', self.buffer, total - 2)[0]
            if crc16(self.buffer[:total - 2]) != expected:
                del self.buffer[0]
                self.bad_crc += 1
                continue
            quadrature = ()
            if payload_format == 1:
                iq = struct.unpack_from(f'<{count * 2}h', self.buffer, header_size)
                samples, quadrature = iq[0::2], iq[1::2]
            else:
                samples = struct.unpack_from(f'<{count}H', self.buffer, header_size)
            if payload_format == 0 and any(sample > 4095 for sample in samples):
                first_bad = next((i for i, sample in enumerate(samples) if sample > 4095), -1)
                self.last_header_error = f'payload ADC code out of range: sample[{first_bad}]={samples[first_bad]}'
                self.last_header_hex = self.buffer[:header_size].hex(' ')
                self.bad_headers += 1
                del self.buffer[:total]
                continue
            frames.append(Frame(seq, rate, flags, samples, False,
                                tx_start, f_start, f_end, duration_us,
                                range_mm, beat_millihz, peak_ratio_x100, dsp_cycles,
                                adc_rate, speed, temperature, payload_format, algorithm, quadrature))
            del self.buffer[:total]
        return frames


def encode_frame(frame):
    """For offline fixtures/demo. Firmware uses the independent C encoder."""
    if not 1 <= len(frame.samples) <= MAX_SAMPLES:
        raise ValueError('Invalid sample count')
    if not 1 <= frame.sample_rate_hz <= 10_000_000:
        raise ValueError('Invalid sample rate')
    if frame.payload_format == 1:
        if (len(frame.samples_q) != len(frame.samples) or len(frame.samples) != 1024 or
                any(not -32768 <= code <= 32767 for code in (*frame.samples, *frame.samples_q))):
            raise ValueError('Expected 1024 signed I/Q pairs')
    elif frame.payload_format != 0 or any(not 0 <= code <= 4095 for code in frame.samples):
        raise ValueError('Expected 12-bit ADC codes')
    source_rate = frame.adc_sample_rate_hz or frame.sample_rate_hz
    if frame.flags & FLAG_CHIRP:
        if not (0 < frame.chirp_start_hz < frame.chirp_end_hz < source_rate // 2 and
                0 < frame.chirp_duration_us <= 1_000_000 and
                frame.tx_start_sample < len(frame.samples) * source_rate // frame.sample_rate_hz):
            raise ValueError('Chirp frame metadata is invalid')
        packet = HEADER_V3.pack(0xA55A, 3, 0, frame.sequence, len(frame.samples),
                                frame.sample_rate_hz, frame.flags, frame.tx_start_sample,
                                frame.chirp_start_hz, frame.chirp_end_hz, frame.chirp_duration_us,
                                frame.range_candidate_mm, frame.beat_millihz,
                                frame.peak_ratio_x100, frame.dsp_cycles)
    else:
        packet = HEADER_V3.pack(0xA55A, 3, 0, frame.sequence, len(frame.samples),
                                frame.sample_rate_hz, frame.flags,
                                frame.tx_start_sample if frame.flags & FLAG_TX_RX else 0, 0, 0, 0,
                                frame.range_candidate_mm, frame.beat_millihz,
                                frame.peak_ratio_x100, frame.dsp_cycles)
    if frame.adc_sample_rate_hz:
        fields = list(HEADER_V3.unpack(packet))
        fields[2] = 0
        fields[1] = 4
        packet = HEADER_V4.pack(*fields, frame.adc_sample_rate_hz, frame.sound_speed_mm_s,
                               frame.temperature_centi_c, frame.payload_format, frame.algorithm)
    elif frame.payload_format:
        raise ValueError('I/Q requires v4 ADC sample-rate metadata')
    if frame.payload_format == 1:
        iq = [v for pair in zip(frame.samples, frame.samples_q) for v in pair]
        packet += struct.pack(f'<{len(iq)}h', *iq)
    else:
        packet += struct.pack(f'<{len(frame.samples)}H', *frame.samples)
    return packet + struct.pack('<H', crc16(packet))


def demo_frame():
    # Clearly labelled synthetic data; not a measurement from the user's board.
    samples = tuple(round(2039 + 14 * math.sin(2 * math.pi * 40000 * i / 500000))
                    for i in range(MAX_SAMPLES))
    return Frame(0, 500000, 0, samples, True)


def save_csv(path, frame, vref):
    """Time is relative to RX frame start; voltage is an uncalibrated estimate."""
    if not math.isfinite(vref) or vref <= 0:
        raise ValueError('VREF must be a positive finite voltage')
    with Path(path).open('w', newline='', encoding='utf-8-sig') as stream:
        writer = csv.writer(stream)
        if frame.payload_format == 1:
            writer.writerow(['source', 'sequence', 'sample_rate_hz', 'adc_sample_rate_hz',
                'payload_format', 'algorithm', 'sound_speed_mm_s', 'temperature_status',
                'range_mm', 'beat_millihz', 'sample_index', 'time_us', 'i_code', 'q_code'])
            for i, (iv, qv) in enumerate(zip(frame.samples, frame.samples_q)):
                writer.writerow(['SIMULATED' if frame.demo or frame.flags & FLAG_SIMULATED else 'UART4',
                    frame.sequence, frame.sample_rate_hz, frame.adc_sample_rate_hz, 'IQ_S16',
                    frame.algorithm, frame.sound_speed_mm_s, temperature_status(frame),
                    '' if range_precise_mm(frame) is None else f'{range_precise_mm(frame):.2f}', frame.beat_millihz, i,
                    f'{(frame.adc_sample_rate_hz // frame.sample_rate_hz - 1) * 1e6 / frame.adc_sample_rate_hz + i * 1e6 / frame.sample_rate_hz:.6f}', iv, qv])
            return
        writer.writerow(['source', 'sequence', 'sample_rate_hz', 'flags',
                         'tx_start_sample', 'chirp_start_hz', 'chirp_end_hz', 'chirp_duration_us',
                         'range_candidate_mm', 'beat_millihz', 'peak_ratio_x100', 'dsp_cycles',
                         'sample_index', 'time_us', 'adc_code', 'vref_V',
                         'estimated_adc_voltage_V', 'temperature_status', 'range_precise_mm'])
        for i, code in enumerate(frame.samples):
            source = 'DEMO' if frame.demo else ('SIMULATED' if frame.flags & FLAG_SIMULATED else 'UART4')
            writer.writerow([source, frame.sequence,
                             frame.sample_rate_hz, frame.flags, frame.tx_start_sample,
                             frame.chirp_start_hz, frame.chirp_end_hz, frame.chirp_duration_us,
                             frame.range_candidate_mm, frame.beat_millihz, frame.peak_ratio_x100,
                             frame.dsp_cycles, i,
                             f'{i * 1e6 / frame.sample_rate_hz:.6f}', code,
                             vref, f'{code * vref / 4095:.9f}', temperature_status(frame),
                             '' if range_precise_mm(frame) is None else f'{range_precise_mm(frame):.2f}'])
