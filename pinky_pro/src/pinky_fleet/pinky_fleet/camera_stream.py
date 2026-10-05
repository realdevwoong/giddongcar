"""Receive Pinky Pro's HTTP MJPEG stream and keep only its newest frame."""
from http.client import HTTPConnection, HTTPException
import threading
import time
from collections import deque


class MJPEGCamera:
    """Reconnect to the robot HTTP camera endpoint without queuing old frames."""

    MAX_BUFFER = 8 * 1024 * 1024

    def __init__(self, host, port=5000, name='camera'):
        self.host = host.strip()
        self.port = int(port)
        self.name = name
        self._lock = threading.Condition()
        self._stop = threading.Event()
        self._frame = None
        self._overlay = None
        self._sequence = 0
        self._frame_stamps = deque(maxlen=100)
        self._display_sequence = 0
        self._connected = False
        self._last_frame_at = None
        self._error = ''
        self._thread = None
        self._inference = dict(state='disabled', model=None, latency_ms=None, labels=[])
        self._inference_times = deque(maxlen=20)
        self._last_inference_at = None

    @property
    def enabled(self):
        return bool(self.host)

    def start(self):
        if self.enabled and self._thread is None:
            self._thread = threading.Thread(target=self._run, name=f'{self.name}-mjpeg', daemon=True)
            self._thread.start()

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
                                   latency_ms=round(latency_ms, 1), labels=labels[:8],
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
            while not self._stop.is_set() and self._display_sequence <= sequence:
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
            elif age is not None and age <= 2.5:
                state = 'online'
            elif self._connected:
                state = 'stale'
            elif self._sequence:
                state = 'offline'
            else:
                state = 'connecting'
            inference = {**self._inference, 'fps': self._inference_fps(),
                         'age_s': round(max(0.0, now - self._last_inference_at), 1)
                         if self._last_inference_at is not None else None}
            return dict(enabled=self.enabled, state=state, frames=self._sequence,
                        age_s=round(age, 1) if age is not None else None,
                        error=self._error, inference=inference,
                        url=f'/camera/{self.name}.mjpg' if self.enabled else None)

    def _set_connection(self, connected, error=''):
        with self._lock:
            self._connected = connected
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

    def _run(self):
        while not self._stop.is_set():
            connection = None
            try:
                connection = HTTPConnection(self.host, self.port, timeout=4)
                connection.request('GET', '/', headers={
                    'Host': self.host,
                    'Accept': 'multipart/x-mixed-replace, image/jpeg, */*',
                    'Connection': 'keep-alive',
                })
                response = connection.getresponse()
                if response.status != 200:
                    raise ConnectionError(f'카메라 HTTP 응답: {response.status} {response.reason}')
                content_type = response.getheader('Content-Type', '').lower()
                if content_type.startswith('text/html'):
                    raise ValueError('카메라 주소가 MJPEG가 아닌 HTML 페이지를 반환했습니다')

                self._set_connection(True)
                buffer = b''
                while not self._stop.is_set():
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
                self._set_connection(False, str(exc))
                self._stop.wait(2.0)
            finally:
                if connection:
                    connection.close()
        self._set_connection(False)
