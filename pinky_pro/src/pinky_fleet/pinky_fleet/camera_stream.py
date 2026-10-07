"""Receive Pinky Pro camera frames from MJPEG or its HTML viewer snapshot endpoint."""
from http.client import HTTPConnection, HTTPException
import re
import threading
import time
from collections import deque
from urllib.parse import urlsplit


class MJPEGCamera:
    """Reconnect to the robot HTTP camera endpoint without queuing old frames."""

    MAX_BUFFER = 8 * 1024 * 1024

    def __init__(self, host, port=5000, name='camera'):
        self.host = host.strip()
        self.port = int(port)
        self.path = '/'
        self._connection = None
        self.name = name
        self._lock = threading.Condition()
        self._stop = threading.Event()
        self._active = threading.Event()
        self._frame = None
        self._overlay = None
        self._sequence = 0
        self._frame_stamps = deque(maxlen=100)
        self._display_sequence = 0
        self._connected = False
        self._content_type = ''
        self._last_frame_at = None
        self._error = ''
        self._thread = None
        self._inference = dict(state='disabled', model=None, latency_ms=None, labels=[], sequence=0)
        self._inference_times = deque(maxlen=20)
        self._last_inference_at = None

    def set_stream_url(self, url):
        """Use the stream URL reported by Pinky's BLE camera response when available."""
        value = str(url or '').strip()
        if not value:
            return False
        if value.startswith(':'):
            parsed = urlsplit('http://pinky' + value)
            host = self.host
        elif value.startswith('/'):
            parsed = urlsplit(value)
            host = self.host
        else:
            parsed = urlsplit(value if '://' in value else 'http://' + value)
            host = parsed.hostname or self.host
        if parsed.scheme not in ('', 'http') or not host:
            raise ValueError(f'지원하지 않는 카메라 스트림 주소: {value}')
        port = parsed.port or (self.port if value.startswith('/') else 80)
        path = parsed.path or '/'
        if parsed.query:
            path += '?' + parsed.query
        with self._lock:
            changed = (self.host, self.port, self.path) != (host, port, path)
            self.host, self.port, self.path = host, port, path
            active = self._connection if changed else None
        if active:
            active.close()
        return changed

    @property
    def enabled(self):
        return bool(self.host)

    def start(self):
        if self.enabled and self._thread is None:
            self._thread = threading.Thread(target=self._run, name=f'{self.name}-mjpeg', daemon=True)
            self._thread.start()

    @property
    def active(self):
        return self._active.is_set()

    def activate(self):
        if self.enabled:
            self._active.set()
            with self._lock:
                self._error = ''
                self._lock.notify_all()

    def deactivate(self):
        self._active.clear()
        with self._lock:
            connection = self._connection
            self._lock.notify_all()
        if connection:
            connection.close()

    def close(self):
        self._stop.set()
        with self._lock:
            self._lock.notify_all()
        if self._thread:
            self._thread.join(timeout=1.0)

    def latest(self):
        with self._lock:
            return self._frame, self._sequence

    def display_frame(self):
        with self._lock:
            if self._overlay and self._inference['state'] == 'online':
                return self._overlay[1], self._display_sequence
            return self._frame, self._display_sequence

    def set_overlay(self, sequence, frame, latency_ms, detections):
        with self._lock:
            self._overlay = (sequence, frame)
            self._display_sequence += 1
            now = time.monotonic()
            self._last_inference_at = now
            self._inference_times.append(self._last_inference_at)
            source_at = next((stamp for frame_sequence, stamp in reversed(self._frame_stamps)
                              if frame_sequence == sequence), now)
            labels = sorted({item['label'] for item in detections})
            self._inference = dict(state='online', model=self._inference['model'],
                                   latency_ms=round(latency_ms, 1), labels=labels[:8], sequence=sequence,
                                   detections=detections[:20], source_age_s=round(max(0.0, now - source_at), 2),
                                   fps=self._inference_fps())
            self._lock.notify_all()

    def _inference_fps(self):
        if len(self._inference_times) < 2:
            return 0.0
        elapsed = self._inference_times[-1] - self._inference_times[0]
        return round((len(self._inference_times) - 1) / elapsed, 1) if elapsed > 0 else 0.0

    def set_inference_state(self, state, model=None, error=''):
        with self._lock:
            self._inference = dict(state=state, model=model, latency_ms=None, labels=[], error=error[:160])
            if state == 'error':
                self._display_sequence += 1
                self._lock.notify_all()

    def wait_next(self, sequence, timeout=1.0):
        deadline = time.monotonic() + timeout
        with self._lock:
            while not self._stop.is_set() and self._active.is_set() and self._display_sequence <= sequence:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._lock.wait(remaining)
            if self._overlay and self._inference['state'] == 'online':
                return self._overlay[1], self._display_sequence
            return self._frame, self._display_sequence

    def status(self):
        now = time.monotonic()
        with self._lock:
            age = None if self._last_frame_at is None else max(0.0, now - self._last_frame_at)
            if not self.enabled:
                state = 'disabled'
            elif not self._active.is_set():
                state = 'idle'
            elif self._sequence and age is not None and age <= 2.5:
                state = 'online'
            elif self._connected and self._sequence == 0:
                state = 'waiting'
            elif self._connected:
                state = 'stale'
            elif self._error:
                state = 'offline'
            else:
                state = 'connecting'
            inference = {**self._inference, 'fps': self._inference_fps(),
                         'age_s': round(max(0.0, now - self._last_inference_at), 1)
                         if self._last_inference_at is not None else None}
            return dict(enabled=self.enabled, active=self._active.is_set(), state=state, frames=self._sequence,
                        age_s=round(age, 1) if age is not None else None,
                        source_url=f'http://{self.host}:{self.port}{self.path}',
                        content_type=self._content_type, error=self._error, inference=inference,
                        url=f'/camera/{self.name}.jpg' if self.enabled else None)

    def _set_connection(self, connected, error=None):
        with self._lock:
            self._connected = connected
            if error is not None:
                self._error = error[:160]

    def _publish(self, frame):
        with self._lock:
            self._frame = frame
            self._sequence += 1
            self._frame_stamps.append((self._sequence, time.monotonic()))
            if not (self._overlay and self._inference['state'] == 'online'):
                self._display_sequence += 1
            self._last_frame_at = time.monotonic()
            self._error = ''
            if not (self._overlay and self._inference['state'] == 'online'):
                self._lock.notify_all()

    @staticmethod
    def _take_frames(buffer):
        frames = []
        while True:
            start = buffer.find(b'\xff\xd8')
            if start < 0:
                return frames, buffer[-1:] if buffer.endswith(b'\xff') else b''
            end = buffer.find(b'\xff\xd9', start + 2)
            if end < 0:
                return frames, buffer[start:]
            frames.append(buffer[start:end + 2])
            buffer = buffer[end + 2:]

    @staticmethod
    def _snapshot_path(page):
        # Pinky camera viewer sets img.src='/snapshot?t='+Date.now() on every frame.
        match = re.search(r"img\.src\s*=\s*['\"]([^'\"]*snapshot[^'\"]*)['\"]", page, re.IGNORECASE)
        if not match:
            return None
        return match.group(1).split('?', 1)[0] or '/snapshot'

    def _read_snapshots(self, host, port, path):
        while not self._stop.is_set() and self._active.is_set():
            connection = HTTPConnection(host, port, timeout=4)
            with self._lock:
                self._connection = connection
            try:
                request_path = f'{path}?t={int(time.time() * 1000)}'
                connection.request('GET', request_path, headers={
                    'Host': host,
                    'Accept': 'image/jpeg, */*',
                    'Cache-Control': 'no-cache',
                    'Connection': 'close',
                })
                response = connection.getresponse()
                if response.status != 200:
                    raise ConnectionError(f'카메라 snapshot HTTP 응답: {response.status} {response.reason}')
                content_type = response.getheader('Content-Type', '').lower()
                if content_type and 'image/jpeg' not in content_type and 'application/octet-stream' not in content_type:
                    raise ValueError(f'snapshot 응답이 JPEG가 아닙니다: {content_type[:80]}')
                frame = response.read(self.MAX_BUFFER + 1)
                if len(frame) > self.MAX_BUFFER:
                    raise ValueError('카메라 JPEG 프레임 크기가 제한을 넘었습니다')
                if not frame.startswith(b'\xff\xd8') or not frame.endswith(b'\xff\xd9'):
                    raise ValueError('snapshot 응답에 올바른 JPEG 프레임이 없습니다')
                self._publish(frame)
            finally:
                connection.close()
                with self._lock:
                    if self._connection is connection:
                        self._connection = None
            self._stop.wait(1.0 / 15.0)

    def _run(self):
        while not self._stop.is_set():
            if not self._active.wait(0.1):
                continue
            connection = None
            try:
                with self._lock:
                    host, port, path = self.host, self.port, self.path
                connection = HTTPConnection(host, port, timeout=4)
                with self._lock:
                    self._connection = connection
                connection.request('GET', path, headers={
                    'Host': host,
                    'Accept': 'multipart/x-mixed-replace, image/jpeg, */*',
                    'Connection': 'keep-alive',
                })
                response = connection.getresponse()
                if response.status != 200:
                    raise ConnectionError(f'카메라 HTTP 응답: {response.status} {response.reason}')
                content_type = response.getheader('Content-Type', '').lower()
                if content_type.startswith('text/html'):
                    page = response.read1(65536).decode('utf-8', errors='replace')
                    snapshot_path = self._snapshot_path(page)
                    if not snapshot_path:
                        raise ValueError(f'{host}:{port}{path}에서 카메라 snapshot 경로가 없는 HTML을 받았습니다')
                    self._content_type = 'HTML viewer + JPEG snapshot polling'
                    self._set_connection(True)
                    self._read_snapshots(host, port, snapshot_path)
                    continue
                if content_type and not any(kind in content_type for kind in
                                            ('multipart/x-mixed-replace', 'multipart/mixed', 'image/jpeg')):
                    raise ValueError(f'카메라 응답 형식이 MJPEG가 아닙니다: {content_type[:80]}')
                self._content_type = content_type
                self._set_connection(True)
                buffer = b''
                while not self._stop.is_set() and self._active.is_set():
                    chunk = response.read1(65536)
                    if not chunk:
                        raise ConnectionError('카메라가 HTTP 스트림을 종료했습니다')
                    buffer += chunk
                    frames, buffer = self._take_frames(buffer)
                    for frame in frames:
                        self._publish(frame)
                    if len(buffer) > self.MAX_BUFFER:
                        raise ValueError('JPEG 프레임 버퍼가 제한을 넘었습니다')
            except (OSError, HTTPException, ValueError, ConnectionError) as exc:
                if self._active.is_set():
                    self._set_connection(False, str(exc))
                    self._stop.wait(2.0)
                else:
                    self._set_connection(False)
            finally:
                if connection:
                    connection.close()
                with self._lock:
                    if self._connection is connection:
                        self._connection = None
        self._set_connection(False)
