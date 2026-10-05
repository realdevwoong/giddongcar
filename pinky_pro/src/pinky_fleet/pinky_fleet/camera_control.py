"""Start and stop a Pinky camera through the robot's BLE control service."""
import asyncio
import json
import logging
import threading


SERVICE_UUID = '0000a000-0000-1000-8000-00805f9b34fb'
RX_CHAR_UUID = '0000a001-0000-1000-8000-00805f9b34fb'
TX_CHAR_UUID = '0000a002-0000-1000-8000-00805f9b34fb'
CHUNK_SIZE = 20
LOGGER = logging.getLogger(__name__)


class _OtherPinky(RuntimeError):
    """A discovered Pinky whose Wi-Fi IP does not match this controller."""


class _CameraCommandError(RuntimeError):
    """The matching robot rejected or failed a camera command."""


class PinkyCameraControl:
    """Find the Pinky by its reported WiFi IP, then control its camera over BLE."""

    def __init__(self, host, name='camera', camera=None):
        self.host = host.strip()
        self.name = name
        self.camera = camera
        self._stream_url = ''
        self.address = None
        self._lock = threading.Lock()
        self._enabled = False
        self._running = False
        self._state = 'unconfigured' if not self.host else 'off'
        self._error = ''

    def status(self):
        with self._lock:
            return dict(configured=bool(self.host), state=self._state,
                        enabled=self._enabled, running=self._running, error=self._error, stream_url=self._stream_url)

    def request(self, enabled):
        if not self.host:
            raise ValueError('카메라 IP가 설정되지 않았습니다.')
        LOGGER.info('%s 카메라 BLE %s 요청: %s', self.name,
                    '시작' if enabled else '중지', self.host)
        with self._lock:
            self._state = 'starting' if enabled else 'stopping'
            self._error = ''
        try:
            result = asyncio.run(self._request(enabled))
        except Exception as exc:
            message = self._error_message(exc)
            with self._lock:
                self._state = 'error'
                self._error = message[:160]
            LOGGER.error('%s 카메라 BLE 제어 실패 (%s): %s', self.name, self.host, message)
            raise ValueError(self._error) from exc
        stream_url = str(result.get('url') or '').strip()
        if enabled and stream_url and self.camera:
            try:
                self.camera.set_stream_url(stream_url)
            except ValueError as exc:
                LOGGER.warning('%s 로봇이 반환한 카메라 주소를 적용하지 못함: %s', self.name, exc)
                stream_url = ''
        with self._lock:
            self._enabled = bool(result.get('enabled', enabled))
            self._running = bool(result.get('running', False))
            self._stream_url = stream_url or self._stream_url
            self._state = 'on' if self._enabled else 'off'
            self._error = ''
        LOGGER.info('%s 카메라 BLE %s 완료: %s', self.name,
                    '시작' if enabled else '중지', self.host)
        if enabled:
            endpoint = self.camera.status()['source_url'] if self.camera else (self._stream_url or self.host)
            source = '로봇 제공 주소' if stream_url else '기본 주소'
            return f'카메라 시작 응답 완료 · 스트림 확인 주소({source}): {endpoint}'
        return '카메라 스트리밍 중지 응답 완료'

    async def _request(self, enabled):
        try:
            from bleak import BleakClient, BleakScanner
        except ImportError as exc:
            raise RuntimeError('BLE 제어 라이브러리가 없습니다. python3-bleak 설치가 필요합니다.') from exc

        address = self.address
        devices = []
        if address:
            devices.append((address, self.name))
        else:
            discovered = await BleakScanner.discover(timeout=6.0, return_adv=True)
            for device, advertisement in discovered.values():
                device_name = advertisement.local_name or device.name or ''
                manufacturer_names = [bytes(value).decode('ascii', errors='ignore')
                                      for value in advertisement.manufacturer_data.values()]
                if (device_name.lower().startswith('pinky_')
                        or any(value.lower().startswith('pinky_') for value in manufacturer_names)):
                    devices.append((device.address, device_name))
        if not devices:
            raise RuntimeError('BLE에서 Pinky를 찾지 못했습니다. 로봇 전원과 BLE 연결을 확인하세요.')
        LOGGER.info('%s BLE Pinky 검색 완료: %d대', self.name, len(devices))

        last_error = None
        for candidate, device_name in devices:
            LOGGER.info('%s BLE 연결 시도: %s (%s)', self.name, device_name or 'Pinky', candidate)
            try:
                result = await self._request_from_device(BleakClient, candidate, enabled)
                self.address = candidate
                LOGGER.info('%s BLE 카메라 응답 수신: enabled=%s running=%s url=%s', self.name,
                            result.get('enabled'), result.get('running'), result.get('url') or '(URL 없음)')
                return result
            except _CameraCommandError:
                raise
            except _OtherPinky as exc:
                LOGGER.info('%s BLE 장치 제외: %s', self.name, exc)
                last_error = exc
                if candidate == self.address:
                    self.address = None
            except Exception as exc:
                LOGGER.warning('%s BLE 후보 실패 (%s): %s', self.name, candidate, self._error_message(exc))
                last_error = RuntimeError(
                    f'{device_name or candidate} BLE 연결 실패: {self._error_message(exc)}')
                if candidate == self.address:
                    self.address = None
        raise RuntimeError(str(last_error or '로봇 BLE 연결에 실패했습니다.'))

    async def _request_from_device(self, client_type, address, enabled):
        async with client_type(address, timeout=12.0) as client:
            messages = asyncio.Queue()
            pending = bytearray()

            def on_notify(_characteristic, data):
                pending.extend(data)
                while b'\n' in pending:
                    raw, _, remaining = pending.partition(b'\n')
                    pending[:] = remaining
                    try:
                        event = json.loads(raw.decode('utf-8').strip())
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        continue
                    messages.put_nowait(event)

            await client.start_notify(TX_CHAR_UUID, on_notify)
            status = await self._send(client, messages, {'cmd': 'status'}, 'status')
            LOGGER.info('%s BLE 상태 응답: 주소=%s 로봇 IP=%s', self.name, address, status.get('ip'))
            if str(status.get('ip', '')).strip() != self.host:
                raise _OtherPinky(f'{address}는 {self.host} 로봇이 아닙니다.')

            try:
                if enabled:
                    result = await self._send(client, messages, {
                        'cmd': 'set_camera', 'enabled': True, 'width': 320, 'height': 240, 'fps': 15,
                    }, 'camera_result')
                else:
                    result = await self._send(client, messages, {'cmd': 'set_camera', 'enabled': False}, 'camera_result')
                if not result.get('ok'):
                    raise RuntimeError(str(result.get('message') or '로봇이 카메라 명령을 거부했습니다.'))
                LOGGER.info('%s 로봇 set_camera 응답: %s', self.name, result)
            except Exception as exc:
                raise _CameraCommandError(f'{self.host} 카메라 명령 실패: {self._error_message(exc)}') from exc
            return result

    @staticmethod
    async def _send(client, messages, command, expected_event):
        payload = json.dumps(command, separators=(',', ':')).encode('utf-8') + b'\n'
        for start in range(0, len(payload), CHUNK_SIZE):
            await client.write_gatt_char(RX_CHAR_UUID, payload[start:start + CHUNK_SIZE], response=True)
        while True:
            try:
                event = await asyncio.wait_for(messages.get(), timeout=20.0)
            except asyncio.TimeoutError as exc:
                raise RuntimeError(f'로봇 {expected_event} 응답 시간 초과') from exc
            if event.get('event') == 'error':
                raise RuntimeError(str(event.get('message') or '로봇 BLE 명령 오류'))
            if event.get('event') == expected_event:
                return event

    @staticmethod
    def _error_message(exc):
        detail = str(exc).strip()
        if not detail:
            detail = '오류 메시지가 없는 BLE 예외'
        return f'{type(exc).__name__}: {detail}'
