"""Standalone Pinky camera perception and guarded lane-following prototype.

This tool is intentionally separate from the fleet dashboard. Its default
``observe`` mode never publishes velocity commands. ``drive`` is an attended,
low-speed experiment that requires an explicit confirmation and a verified
robot-side cmd_vel watchdog.
"""
import argparse
from datetime import datetime
import json
import logging
import math
import os
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from pinky_fleet.camera_control import PinkyCameraControl
from pinky_fleet.camera_stream import MJPEGCamera


LOGGER = logging.getLogger('vision_drive')


def parse_args():
    parser = argparse.ArgumentParser(
        description='Pinky 카메라/YOLO 인식과 감독형 차선 주행 실험')
    parser.add_argument('--robot-ip', required=True, help='로봇 Wi-Fi IP (예: 192.168.0.6)')
    parser.add_argument('--camera-port', type=int, default=5000)
    parser.add_argument('--model', required=True, help='학습한 Ultralytics segmentation .pt 파일 경로')
    parser.add_argument('--device', default='auto', help='Ultralytics 장치: auto, cpu, 0 등')
    parser.add_argument('--driveable-class', default='driveable_area',
                        help='분할 모델의 주행 가능 영역 클래스명')
    parser.add_argument('--obstacle-classes',
                        default='person,bicycle,car,motorcycle,bus,truck,bench,backpack,suitcase,chair',
                        help='보이면 정지할 탐지 클래스명, 쉼표 구분')
    parser.add_argument('--crosswalk-class', default='crosswalk')
    parser.add_argument('--crosswalk-action', choices=('slow', 'stop', 'ignore'), default='slow')
    parser.add_argument('--mode', choices=('observe', 'drive'), default='observe')
    parser.add_argument('--enable-motion', action='store_true',
                        help='실제 주행 명령을 허용 (drive 모드에서만 적용)')
    parser.add_argument('--confirm-supervised-test', action='store_true',
                        help='장애물 없는 통제 구역에서 직접 감독함을 확인')
    parser.add_argument('--watchdog-verified', action='store_true',
                        help='로봇 측 cmd_vel 정지 watchdog과 비상정지를 확인')
    parser.add_argument('--max-linear', type=float, default=0.04, help='최대 전진 속도 m/s')
    parser.add_argument('--max-angular', type=float, default=0.25, help='최대 회전 속도 rad/s')
    parser.add_argument('--stop-distance', type=float, default=0.35,
                        help='라이다 전방 정지 거리 m (제동 시험 전 보수적 초기값)')
    parser.add_argument('--headless', action='store_true', help='OpenCV 영상 창을 띄우지 않음')
    parser.add_argument('--output-dir', default='~/vision_drive_observations',
                        help='관찰 모드에서 프레임/결과를 저장할 디렉터리')
    return parser.parse_args()


class VisionDriveNode(Node):
    def __init__(self, mode):
        super().__init__('vision_drive')
        self.mode = mode
        self.scan = None
        self.scan_at = None
        self.subscription = self.create_subscription(LaserScan, 'scan', self._on_scan, 10)
        self.command_pub = self.create_publisher(Twist, 'cmd_vel', 10) if mode == 'drive' else None

    def _on_scan(self, message):
        self.scan = message
        self.scan_at = time.monotonic()

    def front_range(self, max_age=0.5, half_angle=math.radians(22.5)):
        if self.scan is None or self.scan_at is None or time.monotonic() - self.scan_at > max_age:
            return None
        values = []
        scan = self.scan
        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= half_angle and math.isfinite(distance):
                if scan.range_min <= distance <= scan.range_max:
                    values.append(distance)
        return min(values) if values else None

    def external_cmd_vel_publishers(self):
        return [info for info in self.get_publishers_info_by_topic('cmd_vel')
                if info.node_name != self.get_name()]

    def publish_stop(self, duration=0.0):
        if self.command_pub is None:
            return
        zero = Twist()
        deadline = time.monotonic() + duration
        while rclpy.ok():
            self.command_pub.publish(zero)
            if time.monotonic() >= deadline:
                break
            rclpy.spin_once(self, timeout_sec=0.02)


def _mask_for_class(result, class_name, image_shape):
    if result.masks is None or result.boxes is None:
        return None
    height, width = image_shape[:2]
    mask_data = result.masks.data
    names = result.names
    combined = np.zeros((height, width), dtype=np.uint8)
    found = False
    for index, box in enumerate(result.boxes):
        class_id = int(box.cls.item())
        label = names[class_id] if isinstance(names, (list, tuple)) else names.get(class_id, str(class_id))
        if label != class_name:
            continue
        found = True
        mask = mask_data[index].detach().cpu().numpy()
        mask = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)
        combined |= (mask > 0.5).astype(np.uint8)
    return combined if found else None


def _lane_error(mask):
    """Return normalized visual steering error; reject masks outside the camera center."""
    if mask is None:
        return None
    height, width = mask.shape
    count, labels, _, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if count <= 1:
        return None
    base_y = min(height - 1, int(height * 0.92))
    x_center = width // 2
    component = int(labels[base_y, x_center])
    if component == 0:
        return None
    look_y = min(height - 1, int(height * 0.68))
    row = labels[look_y]
    xs = np.flatnonzero(row == component)
    if xs.size < max(3, int(width * 0.025)):
        return None
    lane_center = (float(xs[0]) + float(xs[-1])) / 2.0
    return (lane_center - x_center) / max(1.0, width / 2.0)


def _detections(result):
    output = []
    if result.boxes is None:
        return output
    names = result.names
    for box in result.boxes:
        class_id = int(box.cls.item())
        label = names[class_id] if isinstance(names, (list, tuple)) else names.get(class_id, str(class_id))
        confidence = float(box.conf.item())
        x1, y1, x2, y2 = (float(value) for value in box.xyxy[0].tolist())
        output.append((label, confidence, x1, y1, x2, y2))
    return output


def _policy(node, mask, detections, args, inference_at, camera_at):
    """Fail closed when perception or robot sensor data is unavailable/stale."""
    if time.monotonic() - inference_at > 0.5 or camera_at is None or camera_at > 0.5:
        return 0.0, 0.0, '카메라/인식 지연: 정지'
    error = _lane_error(mask)
    if error is None:
        return 0.0, 0.0, '주행 영역 불명확: 정지'
    front = node.front_range()
    if args.mode == 'drive' and front is None:
        return 0.0, 0.0, '라이다 입력 없음/지연: 정지'
    if front is not None and front <= args.stop_distance:
        return 0.0, 0.0, f'전방 장애물 {front:.2f} m: 정지'

    height, width = mask.shape
    stop_classes = {name.strip() for name in args.obstacle_classes.split(',') if name.strip()}
    slow_crosswalk = False
    for label, confidence, x1, y1, x2, y2 in detections:
        if confidence < 0.45:
            continue
        in_path = (y2 >= height * 0.48 and x2 >= width * 0.2 and x1 <= width * 0.8)
        if in_path and label in stop_classes:
            return 0.0, 0.0, f'{label} 탐지: 정지'
        if in_path and label == args.crosswalk_class:
            if args.crosswalk_action == 'stop':
                return 0.0, 0.0, '횡단보도: 정지 정책'
            slow_crosswalk = args.crosswalk_action == 'slow'

    linear = min(args.max_linear, 0.02 if slow_crosswalk else args.max_linear)
    angular = max(-args.max_angular, min(args.max_angular, -0.35 * error))
    if front is not None and front < args.stop_distance + 0.25:
        linear *= max(0.0, (front - args.stop_distance) / 0.25)
    return linear, angular, '횡단보도 감속' if slow_crosswalk else '차선 영역 추종'


def main():
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
    args = parse_args()
    model_path = Path(args.model).expanduser()
    if not model_path.is_file():
        raise SystemExit(f'학습 모델 파일을 찾을 수 없습니다: {model_path}')
    if args.mode == 'drive':
        if not (args.enable_motion and args.confirm_supervised_test and args.watchdog_verified):
            raise SystemExit('주행 모드에는 --enable-motion, --confirm-supervised-test, '
                             '--watchdog-verified가 모두 필요합니다.')
        if (args.max_linear <= 0 or args.max_linear > 0.04
                or args.max_angular <= 0 or args.max_angular > 0.25
                or args.stop_distance < 0.35):
            raise SystemExit('초기 주행 한도는 max-linear <= 0.04 m/s, max-angular <= 0.25 rad/s, '
                             'stop-distance >= 0.35 m 입니다.')

    try:
        # Ultralytics probes public DNS during import unless offline mode is set.
        # A disconnected network can block before this node has a chance to log.
        os.environ.setdefault('YOLO_OFFLINE', 'true')
        LOGGER.info('Ultralytics 오프라인 모드로 초기화합니다')
        from ultralytics import YOLO
    except ImportError as exc:
        raise SystemExit('Ultralytics가 설치된 Python 환경에서 실행하세요.') from exc
    LOGGER.info('YOLO 모델 로드 중: %s', model_path)
    model = YOLO(str(model_path))
    LOGGER.info('YOLO 모델 로드 완료: %s', model.names)
    camera = MJPEGCamera(args.robot_ip, args.camera_port, name='vision_drive')
    control = PinkyCameraControl(args.robot_ip, name='vision_drive', camera=camera)
    node = None
    camera_started = False
    # Defined before try: the finally block uses them even if startup checks fail.
    video_writers = None
    recording_stem = None
    try:
        rclpy.init()
        node = VisionDriveNode(args.mode)
        camera.start()
        LOGGER.info('로봇 카메라 시작 요청: %s', args.robot_ip)
        print(control.request(True))
        camera_started = True
        if args.mode == 'drive':
            time.sleep(2.0)
            others = node.external_cmd_vel_publishers()
            if others:
                raise RuntimeError('다른 /cmd_vel 발행자가 있습니다. Nav2/대시보드를 중지한 뒤 다시 실행하세요: '
                                   + ', '.join(info.node_name for info in others))
            if node.front_range() is None:
                raise RuntimeError('라이다 /scan이 아직 유효하지 않습니다. 로봇 ROS 도메인과 토픽을 확인하세요.')
            LOGGER.warning('감독형 저속 주행 실험 시작. 즉시 정지하려면 창에서 q를 누르세요.')

        inference_at = 0.0
        last_frame_sequence = -1
        camera_age = None
        overlay = None
        last_frame = None
        last_detections = []
        last_latency_ms = None
        current_command = (0.0, 0.0, '초기화')
        last_control_at = 0.0
        device = None if args.device == 'auto' else args.device
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            frame_bytes, sequence = camera.latest()
            status = camera.status()
            camera_age = status.get('age_s')
            if frame_bytes is not None and sequence != last_frame_sequence:
                last_frame_sequence = sequence
                frame = cv2.imdecode(np.frombuffer(frame_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    current_command = (0.0, 0.0, 'JPEG 디코딩 실패: 정지')
                    if args.mode == 'drive':
                        node.publish_stop()
                    continue
                inference_started = time.monotonic()
                results = model.predict(frame, device=device, verbose=False, conf=0.35, imgsz=320)
                result = results[0]
                mask = _mask_for_class(result, args.driveable_class, frame.shape)
                detections = _detections(result)
                inference_at = time.monotonic()
                last_latency_ms = (inference_at - inference_started) * 1000.0
                current_command = _policy(node, mask, detections, args, inference_at, camera_age)
                overlay = result.plot()
                color = (0, 220, 0) if current_command[0] > 0 else (0, 0, 255)
                cv2.putText(overlay, current_command[2], (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                            0.52, color, 2, cv2.LINE_AA)
                cv2.putText(overlay, 'OBSERVE ONLY' if args.mode == 'observe' else
                            f'VERIFIED DRIVE  v={current_command[0]:.2f}',
                            (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1, cv2.LINE_AA)
                names = ', '.join(sorted({item[0] for item in detections})) or '탐지 없음'
                LOGGER.info('mode=%s policy=%s detections=%s inference=%.1fms',
                            args.mode, current_command[2], names, last_latency_ms)
                last_frame = frame.copy()
                last_detections = detections
                if video_writers:
                    video_writers[0].write(last_frame)
                    video_writers[1].write(overlay)

            if args.mode == 'drive' and time.monotonic() - last_control_at >= 0.1:
                linear, angular, reason = current_command
                if time.monotonic() - inference_at > 0.5 or camera_age is None or camera_age > 0.5:
                    linear, angular, reason = 0.0, 0.0, '카메라/인식 지연: 정지'
                front = node.front_range()
                if front is None or (front is not None and front <= args.stop_distance):
                    linear, angular, reason = 0.0, 0.0, '라이다 입력/전방 장애물: 정지'
                if node.external_cmd_vel_publishers():
                    linear, angular, reason = 0.0, 0.0, '다른 cmd_vel 발행자: 정지'
                command = Twist()
                command.linear.x = linear
                command.angular.z = angular
                node.command_pub.publish(command)
                last_control_at = time.monotonic()
                current_command = (linear, angular, reason)

            if not args.headless and overlay is not None:
                cv2.imshow('Pinky vision drive (q = stop)', overlay)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                if args.mode == 'observe' and key == ord('s') and last_frame is not None:
                    output_dir = Path(args.output_dir).expanduser()
                    output_dir.mkdir(parents=True, exist_ok=True)
                    stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                    stem = output_dir / f'frame_{stamp}'
                    cv2.imwrite(str(stem.with_name(stem.name + '_raw.jpg')), last_frame)
                    cv2.imwrite(str(stem.with_name(stem.name + '_overlay.jpg')), overlay)
                    metadata = {
                        'timestamp': datetime.now().astimezone().isoformat(),
                        'model': str(model_path),
                        'class_names': model.names,
                        'detections': [
                            {'label': label, 'confidence': confidence,
                             'xyxy': [x1, y1, x2, y2]}
                            for label, confidence, x1, y1, x2, y2 in last_detections
                        ],
                        'policy': current_command[2],
                        'inference_ms': last_latency_ms,
                        'camera_age_s': camera_age,
                    }
                    stem.with_name(stem.name + '.json').write_text(
                        json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
                    LOGGER.info('관찰 프레임 저장: %s', stem)
                elif args.mode == 'observe' and key == ord('r'):
                    if video_writers:
                        for writer in video_writers:
                            writer.release()
                        video_writers = None
                        LOGGER.info('관찰 영상 녹화 완료: %s', recording_stem)
                    elif last_frame is not None:
                        output_dir = Path(args.output_dir).expanduser()
                        output_dir.mkdir(parents=True, exist_ok=True)
                        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                        recording_stem = output_dir / f'observe_{stamp}'
                        height, width = last_frame.shape[:2]
                        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                        writers = [
                            cv2.VideoWriter(str(recording_stem.with_name(recording_stem.name + '_raw.mp4')),
                                            fourcc, 15.0, (width, height)),
                            cv2.VideoWriter(str(recording_stem.with_name(recording_stem.name + '_overlay.mp4')),
                                            fourcc, 15.0, (width, height)),
                        ]
                        if not all(writer.isOpened() for writer in writers):
                            for writer in writers:
                                writer.release()
                            LOGGER.error('관찰 영상 파일을 열 수 없습니다: %s', recording_stem)
                        else:
                            video_writers = writers
                            LOGGER.info('관찰 영상 녹화 시작: %s (r 키로 종료)', recording_stem)
            elif frame_bytes is None:
                time.sleep(0.02)
            else:
                time.sleep(0.005)
    except KeyboardInterrupt:
        LOGGER.info('사용자 종료 요청')
    finally:
        if video_writers:
            for writer in video_writers:
                writer.release()
            LOGGER.info('관찰 영상 녹화 저장: %s', recording_stem)
        if node is not None and args.mode == 'drive':
            node.publish_stop(duration=0.5)
        if camera_started:
            try:
                print(control.request(False))
            except Exception:
                LOGGER.exception('카메라 중지 요청 실패')
        camera.close()
        if not args.headless:
            cv2.destroyAllWindows()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
