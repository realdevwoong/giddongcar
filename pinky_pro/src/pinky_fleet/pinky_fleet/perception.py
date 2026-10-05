"""Optional YOLO overlay generation for the fleet camera previews."""
import threading
import time


class YOLOPerception:
    """Run one model over the newest frame from each configured robot camera."""

    def __init__(self, cameras, model_path):
        self.cameras = {name: camera for name, camera in cameras.items() if camera.enabled}
        self.model_path = model_path
        self._stop = threading.Event()
        self._thread = None
        if not model_path or not self.cameras:
            self.model = None
            return
        try:
            import cv2
            import numpy as np
            from ultralytics import YOLO
            self.cv2 = cv2
            self.np = np
            # Ultralytics downloads this fixed model by name when it is not local.
            self.model = YOLO(model_path)
        except Exception as exc:
            message = f'YOLO 초기화 실패: {exc}'
            for camera in self.cameras.values():
                camera.set_inference_state('error', model_path, message)
            self.model = None
            return
        for camera in self.cameras.values():
            camera.set_inference_state('waiting', model_path)

    def start(self):
        if self.model and self.cameras and self._thread is None:
            self._thread = threading.Thread(target=self._run, name='fleet-yolo', daemon=True)
            self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _run(self):
        seen = {name: 0 for name in self.cameras}
        while not self._stop.is_set():
            worked = False
            for name, camera in self.cameras.items():
                if self._stop.is_set():
                    break
                frame, sequence = camera.latest()
                if frame is None or sequence == seen[name]:
                    continue
                seen[name] = sequence
                worked = True
                started = time.monotonic()
                try:
                    image = self.cv2.imdecode(self.np.frombuffer(frame, self.np.uint8), self.cv2.IMREAD_COLOR)
                    if image is None:
                        raise ValueError('MJPEG 프레임을 JPEG로 디코딩할 수 없습니다')
                    result = self.model.predict(image, verbose=False)[0]
                    ok, encoded = self.cv2.imencode('.jpg', result.plot())
                    if not ok:
                        raise ValueError('YOLO 오버레이 JPEG 인코딩에 실패했습니다')
                    detections = []
                    for box in result.boxes:
                        detections.append(dict(
                            label=result.names[int(box.cls.item())],
                            confidence=round(float(box.conf.item()), 3),
                            xyxy=[round(float(value), 1) for value in box.xyxy[0].tolist()]))
                    camera.set_overlay(sequence, encoded.tobytes(), (time.monotonic() - started) * 1000,
                                       detections)
                except Exception as exc:
                    camera.set_inference_state('error', self.model_path, str(exc))
            if not worked:
                self._stop.wait(0.01)
