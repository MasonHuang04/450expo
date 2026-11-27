"""Local browser waveform viewer: python rx_viewer.py [--demo].

ASCII r=RX, b=single burst, c=chirp with I/Q, C=chirp with raw samples.
No network service is used: the HTTP server binds only to 127.0.0.1.
"""
import argparse
from collections import deque
from dataclasses import asdict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
import time
import webbrowser
import sys
try:
    import numpy as np
except ImportError:
    np = None

# Optional project-local dependencies installed by install_dependencies.cmd.
local_deps = Path(__file__).with_name('_deps')
if local_deps.is_dir():
    sys.path.insert(0, str(local_deps))

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

from rx_protocol import FrameParser, demo_frame, temperature_status, range_precise_mm


class Session:
    """Serialized access from the HTTP handlers and the serial polling thread."""
    def __init__(self):
        self.lock = threading.RLock()
        self.link = None
        self.port = ''
        self.parser = FrameParser()
        self.frame = None
        self.revision = 0
        self.auto = False
        self.auto_mode = 'rx'
        self.history = deque(maxlen=300)
        self.history_start = None
        self.outstanding = False
        self.deadline = 0.0
        self.next_request = 0.0
        self.last_byte_time = 0.0
        self.bytes_received = 0
        self.frames_received = 0
        self.status = '未连接。选择 COM 口后连接，点击“采集一帧”。'

    def disconnect(self, reason='已断开；保留最后一帧。'):
        with self.lock:
            link, self.link = self.link, None
            if link:
                try:
                    link.close()
                except OSError:
                    pass
            self.auto = self.outstanding = False
            self.status = reason

    def connect(self, port):
        with self.lock:
            if serial is None:
                raise ValueError('缺少 pyserial。运行：python -m pip install -r requirements.txt')
            available = {p.device for p in list_ports.comports()}
            if port not in available:
                raise ValueError('端口不存在，请插入 USB-UART 并刷新端口列表。')
            self.disconnect()
            link = serial.Serial(port=None, baudrate=115200, timeout=0, write_timeout=0.2,
                                 xonxoff=False, rtscts=False, dsrdtr=False)
            try:
                link.dtr = link.rts = False
                link.port = port
                link.open()
                link.reset_input_buffer()
            except OSError:
                link.close()
                raise
            self.link, self.port = link, port
            self.parser = FrameParser()
            self.bytes_received = self.frames_received = 0
            self.history.clear()
            self.history_start = None
            self.status = '已连接。请确认 MCU 正在运行，然后点击“采集一帧”。'

    def request(self, mode='rx', automatic=False):
        with self.lock:
            commands = {'rx': b'r', 'burst': b'b', 'chirp': b'c', 'chirp_raw': b'C'}
            if mode not in commands:
                raise ValueError('未知采集模式。')
            if not self.link:
                raise ValueError('请先连接串口。')
            if self.outstanding:
                return
            if not automatic:
                self.auto = False  # Manual requests stop the repetition scheduler.
            try:
                if self.link.write(commands[mode]) != 1:
                    raise OSError('命令未完整发送')
            except OSError as exc:
                self.disconnect(f'发送失败：{exc}')
                raise
            self.outstanding = True
            # 16384 samples plus v3 header take about 2.85 s at 115200/8N1.
            self.deadline = time.monotonic() + 8.0
            self.status = {'rx': '等待 RX 数据……正常约 3 秒。本次不发射。',
                           'burst': '已请求单次 40 kHz 短脉冲 TX+RX；等待波形约 3 秒。',
                           'chirp': '已请求单次 Chirp，等待 MCU 测距结果和 1024 个 I/Q 点。',
                           'chirp_raw': '已请求单次 Chirp 原始 ADC 波形；16384 点传输约 3 秒。'}[mode]
            if automatic:
                self.status = self.status.replace('已请求单次', '连续测量中，已请求本次')

    def set_auto(self, enabled, mode='rx'):
        with self.lock:
            if mode not in ('rx', 'burst', 'chirp', 'chirp_raw'):
                raise ValueError('未知连续采集模式。')
            if enabled and not self.link:
                raise ValueError('请先连接串口。')
            self.auto = enabled
            if enabled:
                self.auto_mode = mode
                self.next_request = time.monotonic()
                self.request(self.auto_mode, automatic=True)
            else:
                self.status = '连续测量已停止；已请求的一帧仍会接收。'

    def show_demo(self):
        with self.lock:
            if self.link:
                raise ValueError('请先断开串口，再查看演示数据，避免与实测混淆。')
            self.frame = demo_frame()
            self.revision += 1
            self.status = '离线演示：合成 40 kHz 数据，不是板上采集结果。'

    def tick(self):
        with self.lock:
            if not self.link:
                return
            now = time.monotonic()
            try:
                data = self.link.read(min(self.link.in_waiting, 65536))
                if data:
                    self.last_byte_time = now
                    self.bytes_received += len(data)
                    frames = self.parser.feed(data)
                    if frames:
                        for f in frames:
                            if f.flags & 4 and not (f.demo or f.flags & 16):
                                if self.history_start is None:
                                    self.history_start = now
                                temp = temperature_status(f)
                                valid = bool(f.flags & 8) and temp not in ('unavailable', 'inconsistent')
                                self.history.append({'elapsed_s': round(now-self.history_start, 3),
                                    'timestamp_s': time.time(), 'sequence': f.sequence,
                                    'range_mm': range_precise_mm(f) if valid else None,
                                    'valid': valid, 'beat_hz': f.beat_millihz/1000,
                                    'peak_ratio': f.peak_ratio_x100/100,
                                    'dsp_ms': f.dsp_cycles/170000,
                                    'temperature_status': temp,
                                    'sound_speed_mm_s': f.sound_speed_mm_s,
                                    'algorithm': f.algorithm})
                        self.frame = frames[-1]
                        self.revision += len(frames)
                        self.frames_received += len(frames)
                        self.outstanding = False
                        self.next_request = now + 0.05
                        self.status = '已收到 CRC 校验通过的 ADC 帧。可放大、去均值或保存 CSV。'
                if self.outstanding and now >= self.deadline:
                    self.outstanding = self.auto = False
                    self.parser.clear_partial()
                    self.status = '8 秒未收到完整有效帧：检查 MCU 是否运行、断点、固件、TX/RX 接线和共地，再点采集。'
                if self.parser.buffer and now - self.last_byte_time > 3.0:
                    self.parser.clear_partial()
                if self.auto and not self.outstanding and now >= self.next_request:
                    self.request(self.auto_mode, automatic=True)
            except OSError as exc:
                self.disconnect(f'串口断开或读取失败：{exc}')

    def snapshot(self, revision=-1):
        with self.lock:
            frame = asdict(self.frame) if self.frame and self.revision != revision else None
            if frame is not None:
                frame['temperature_status'] = temperature_status(self.frame)
                frame['range_precise_mm'] = range_precise_mm(self.frame)
                if self.frame.payload_format == 1 and np is not None:
                    # Display only. The reported range remains the MCU result.
                    x = np.asarray(self.frame.samples) + 1j*np.asarray(self.frame.samples_q)
                    use_hann = 0 <= self.frame.algorithm < 3
                    window = np.hanning(len(x)) if use_hann else np.kaiser(len(x), 4)
                    power = np.abs(np.fft.fft(x * window))**2
                    frame['spectrum_window'] = 'Hann' if use_hann else 'Kaiser β=4'
                    # Show the stronger signed side at each absolute beat frequency.
                    bins = np.arange(len(x)//2+1)
                    power = np.maximum(power[bins],power[(-bins)%len(x)])
                    db = 10*np.log10(np.maximum(power,1e-12)/max(float(np.max(power)),1e-12))
                    frame['spectrum_db'] = db.tolist()
            return {
                'connected': self.link is not None, 'port': self.port,
                'auto': self.auto, 'waiting': self.outstanding, 'status': self.status,
                'auto_mode': self.auto_mode, 'history': list(self.history),
                'bytes_received': self.bytes_received, 'frames_received': self.frames_received,
                'bad_crc': self.parser.bad_crc, 'bad_headers': self.parser.bad_headers,
                'last_header_error': self.parser.last_header_error,
                'last_header_hex': self.parser.last_header_hex,
                'revision': self.revision,
                'frame': frame,
            }


def make_handler(session):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send(self, status, body, content_type='application/json; charset=utf-8'):
            if not isinstance(body, bytes):
                body = json.dumps(body, ensure_ascii=False).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            self.wfile.write(body)

        def local_request(self):
            host = self.headers.get('Host', '')
            allowed = {f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}'}
            origin = self.headers.get('Origin')
            return host in allowed and (not origin or origin == 'http://' + host)

        def do_GET(self):
            if not self.local_request():
                self.send(403, {'error': 'Local access only'})
                return
            if self.path == '/':
                self.send(200, Path(__file__).with_name('viewer.html').read_bytes(), 'text/html; charset=utf-8')
            elif self.path == '/api/ports':
                ports = [] if list_ports is None else [
                    {'port': p.device, 'description': p.description} for p in list_ports.comports()]
                self.send(200, {'ports': ports, 'serial_available': serial is not None})
            elif self.path.startswith('/api/state?revision='):
                try:
                    revision = int(self.path.split('=', 1)[1])
                except ValueError:
                    revision = -1
                self.send(200, session.snapshot(revision))
            else:
                self.send(404, {'error': 'Not found'})

        def do_POST(self):
            if not self.local_request() or self.headers.get('X-RX-Viewer') != '1':
                self.send(403, {'error': 'Local viewer requests only'})
                return
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 <= length <= 1024:
                    raise ValueError('Invalid request length')
                data = json.loads(self.rfile.read(length) or b'{}')
                if not isinstance(data, dict):
                    raise ValueError('Expected a JSON object')
                if self.path == '/api/connect':
                    session.connect(str(data.get('port', '')))
                elif self.path == '/api/disconnect':
                    session.disconnect()
                elif self.path == '/api/request':
                    session.request(data.get('mode', 'rx'))
                elif self.path == '/api/auto':
                    session.set_auto(bool(data.get('enabled', False)), data.get('mode', 'rx'))
                elif self.path == '/api/demo':
                    session.show_demo()
                else:
                    self.send(404, {'error': 'Not found'})
                    return
                self.send(200, {'ok': True})
            except (ValueError, OSError) as exc:
                self.send(400, {'error': str(exc)})
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo', action='store_true', help='Display labelled synthetic data')
    parser.add_argument('--no-browser', action='store_true', help='Print URL without opening a browser')
    parser.add_argument('--port', type=int, default=8765, help='Local HTTP port; 0 chooses a free port')
    args = parser.parse_args()
    session = Session()
    if args.demo:
        session.show_demo()
    try:
        server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(session))
    except OSError as exc:
        raise SystemExit(f'Cannot start local viewer: {exc}\nTry: python rx_viewer.py --port 0') from exc
    server.daemon_threads = True
    stop = threading.Event()

    def worker():
        while not stop.wait(0.04):
            session.tick()

    thread = threading.Thread(target=worker, daemon=True)
    thread.start()
    url = f'http://127.0.0.1:{server.server_port}/'
    print(f'RX waveform viewer: {url}\nKeep this window open. Press Ctrl+C to stop.', flush=True)
    if serial is None:
        print('For physical serial ports install: python -m pip install -r requirements.txt', flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        thread.join(timeout=1)
        session.disconnect()
        server.server_close()


if __name__ == '__main__':
    main()
