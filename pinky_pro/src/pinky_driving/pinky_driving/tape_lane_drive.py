"""White-tape lane observation and supervised low-speed following prototype."""
import argparse
from datetime import datetime
import json
import logging
import math
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


LOGGER = logging.getLogger('tape_lane_drive')


def parse_args():
    parser = argparse.ArgumentParser(
        description='Pinky 카메라의 흰 테이프 차선 관찰 및 감독형 추종 실험')
    parser.add_argument('--robot-ip', required=True, help='로봇 Wi-Fi IP (예: 192.168.0.6)')
    parser.add_argument('--camera-port', type=int, default=5000)
    parser.add_argument('--ble-name', default='',
                        help='카메라를 켤 로봇의 BLE 이름 (예: pinky_6422). 같은 IP 로봇이 여럿일 때 지정')
    parser.add_argument('--mode', choices=('observe', 'drive'), default='observe')
    parser.add_argument('--white-value', type=int, default=165,
                        help='흰 테이프 최소 HSV 밝기 (0-255)')
    parser.add_argument('--max-saturation', type=int, default=115,
                        help='흰 테이프 최대 HSV 채도 (0-255)')
    parser.add_argument('--roi-top', type=float, default=0.42,
                        help='차선 검출 ROI의 위쪽 높이 비율 (0-1)')
    parser.add_argument('--lane-width-min', type=float, default=0.16,
                        help='영상 폭 대비 차선 최소 폭 비율')
    parser.add_argument('--lane-width-max', type=float, default=0.92,
                        help='영상 폭 대비 차선 최대 폭 비율')
    parser.add_argument('--max-linear', type=float, default=0.02,
                        help='주행 모드 최대 전진 속도 m/s (최대 0.04)')
    parser.add_argument('--max-angular', type=float, default=0.15,
                        help='주행 모드 최대 회전 속도 rad/s (최대 0.25)')
    parser.add_argument('--steering-gain', type=float, default=0.30)
    parser.add_argument('--stop-distance', type=float, default=0.40,
                        help='LiDAR 전방 정지 거리 m (최소 0.35)')
    parser.add_argument('--enable-motion', action='store_true')
    parser.add_argument('--confirm-supervised-test', action='store_true')
    parser.add_argument('--watchdog-verified', action='store_true')
    parser.add_argument('--output-dir', default='~/vision_drive_observations')
    parser.add_argument('--capture-interval', type=float, default=0.5,
                        help='r 키 학습용 연속 저장 간격 초')
    parser.add_argument('--headless', action='store_true')
    return parser.parse_args()


class TapeLaneModel:
    """Detect nearly straight white-tape boundaries and fit a line to each."""

    def __init__(self, white_value, max_saturation, roi_top, lane_width_min,
                 lane_width_max):
        self.white_value = white_value
        self.max_saturation = max_saturation
        self.roi_top = roi_top
        self.lane_width_min = lane_width_min
        self.lane_width_max = lane_width_max
        self.lines = None  # each line is x = slope*y + intercept
        self.valid = False
        self.reason = '초기화'
        self.confidence = 0.0
        self.left_points = []
        self.right_points = []
        self.segment_counts = (0, 0)
        self.fit_error_px = None

    @staticmethod
    def _fit_cluster(lines, selected, width):
        x_ref, slope, _, _ = selected
        cluster = [line for line in lines
                   if abs(line[0] - x_ref) <= width * 0.075
                   and abs(line[1] - slope) <= 0.45]
        points, weights = [], []
        for _, _, length, endpoints in cluster:
            for x, y in endpoints:
                points.append((float(y), float(x)))
                weights.append(max(1.0, length ** 0.5))
        if len(points) < 2:
            return None
        ys = np.asarray([point[0] for point in points])
        xs = np.asarray([point[1] for point in points])
        fitted_slope, intercept = np.polyfit(ys, xs, 1, w=np.asarray(weights))
        residuals = np.abs(xs - (fitted_slope * ys + intercept))
        fit_error = float(np.median(residuals))
        return float(fitted_slope), float(intercept), cluster, fit_error

    def update(self, frame):
        height, width = frame.shape[:2]
        self.valid = False
        self.left_points, self.right_points = [], []
        self.segment_counts = (0, 0)
        self.fit_error_px = None
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        white = cv2.inRange(
            hsv,
            np.array([0, 0, self.white_value], dtype=np.uint8),
            np.array([180, self.max_saturation, 255], dtype=np.uint8),
        )
        roi_y = min(height - 2, max(1, int(height * self.roi_top)))
        white[:roi_y, :] = 0
        white = cv2.morphologyEx(white, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        edges = cv2.Canny(white, 30, 90)
        segments = cv2.HoughLinesP(
            edges, 1, np.pi / 180, threshold=10,
            minLineLength=max(14, int(height * 0.07)),
            maxLineGap=max(18, int(height * 0.09)))

        reference_y = height * 0.60
        left_candidates, right_candidates = [], []
        if segments is not None:
            for segment in segments[:, 0]:
                x1, y1, x2, y2 = map(int, segment)
                dx, dy = x2 - x1, y2 - y1
                if abs(dy) < height * 0.05:
                    continue  # crosswalk bars are mostly horizontal
                slope = dx / dy
                if not 0.25 <= abs(slope) <= 3.5:
                    continue
                x_ref = x1 + (reference_y - y1) * slope
                length = math.hypot(dx, dy)
                entry = (x_ref, slope, length, ((x1, y1), (x2, y2)))
                if slope < 0 and -0.45 * width <= x_ref <= 0.52 * width:
                    left_candidates.append(entry)
                elif slope > 0 and 0.48 * width <= x_ref <= 1.45 * width:
                    right_candidates.append(entry)

        best_pair, best_score = None, float('inf')
        expected_width = width * (self.lane_width_min + self.lane_width_max) * 0.5
        for left in left_candidates:
            for right in right_candidates:
                lane_width = right[0] - left[0]
                lane_center = (right[0] + left[0]) * 0.5
                if not self.lane_width_min * width <= lane_width <= self.lane_width_max * width:
                    continue
                if abs(abs(left[1]) - abs(right[1])) > 1.2:
                    continue
                if abs(lane_center - width * 0.5) > width * 0.25:
                    continue
                score = (abs(lane_center - width * 0.5) * 0.8
                         + abs(lane_width - expected_width) * 0.15
                         + abs(abs(left[1]) - abs(right[1])) * 16.0
                         - (left[2] + right[2]) * 0.12)
                if self.lines is not None:
                    old_left, old_right = self.lines
                    old_left_ref = old_left[0] * reference_y + old_left[1]
                    old_right_ref = old_right[0] * reference_y + old_right[1]
                    score += (abs(left[0] - old_left_ref) + abs(right[0] - old_right_ref)) * 0.45
                    score += (abs(left[1] - old_left[0]) + abs(right[1] - old_right[0])) * 12.0
                if score < best_score:
                    best_pair, best_score = (left, right), score

        if best_pair is None:
            self.valid = False
            self.reason = f'좌우 테이프 직선 부족 (좌 {len(left_candidates)}, 우 {len(right_candidates)})'
            self.confidence = 0.0
            self.left_points, self.right_points = [], []
            return white

        fitted = [self._fit_cluster(candidates, selected, width)
                  for candidates, selected in zip((left_candidates, right_candidates), best_pair)]
        if any(line is None for line in fitted):
            self.valid = False
            self.reason = '테이프 직선 적합 실패'
            self.confidence = 0.0
            return white

        candidate = [(line[0], line[1]) for line in fitted]
        target_y = height * 0.72
        for y in (height * 0.60, height * 0.75, height * 0.92):
            left_x = candidate[0][0] * y + candidate[0][1]
            right_x = candidate[1][0] * y + candidate[1][1]
            lane_width = right_x - left_x
            lane_center = (left_x + right_x) * 0.5
            if (lane_width < width * 0.12 or lane_width > width * 2.20
                    or not -0.35 * width <= lane_center <= 1.35 * width):
                self.valid = False
                self.reason = '좌우 경계 기하 조건 불일치'
                self.confidence = 0.0
                return white
        target_center = sum(line[0] * target_y + line[1] for line in candidate) * 0.5
        if abs(target_center - width * 0.5) > width * 0.17:
            self.valid = False
            self.reason = '중앙 경로 위치 불확실'
            self.confidence = 0.0
            return white
        fit_tolerance = width * 0.075
        if max(line[3] for line in fitted) > fit_tolerance:
            self.valid = False
            self.reason = '직선 적합 오차가 큼'
            self.confidence = 0.0
            return white

        if self.lines is None:
            self.lines = candidate
        else:
            self.lines = [(0.45 * new[0] + 0.55 * old[0],
                           0.45 * new[1] + 0.55 * old[1])
                          for old, new in zip(self.lines, candidate)]
        self.left_points = [point for _, _, _, ends in fitted[0][2] for point in ends]
        self.right_points = [point for _, _, _, ends in fitted[1][2] for point in ends]
        self.segment_counts = (len(fitted[0][2]), len(fitted[1][2]))
        support = sum(line[2] for fit in fitted for line in fit[2])
        self.fit_error_px = max(line[3] for line in fitted)
        support_score = min(1.0, support / (height * 0.75))
        fit_score = max(0.0, 1.0 - self.fit_error_px / fit_tolerance)
        all_segments = [line[2] for fit in fitted for line in fit[2]]
        balance_score = min(all_segments) / max(1.0, max(all_segments))
        self.confidence = support_score * fit_score * (0.65 + 0.35 * balance_score)
        if self.confidence < 0.20:
            self.valid = False
            self.reason = '차선 신뢰도 낮음'
            return white
        self.valid = True
        self.reason = '직선 흰 테이프 좌우 경계'
        return white

    def center_x(self, y, width, height):
        if self.lines is None:
            return None
        left, right = self.lines
        left_x = left[0] * y + left[1]
        right_x = right[0] * y + right[1]
        return (left_x + right_x) * 0.5

    def draw(self, frame, white_mask):
        height, width = frame.shape[:2]
        overlay = frame.copy()
        overlay[white_mask > 0] = (40, 110, 230)
        output = cv2.addWeighted(frame, 0.68, overlay, 0.32, 0)
        roi_y = int(height * self.roi_top)
        cv2.line(output, (0, roi_y), (width - 1, roi_y), (180, 180, 0), 1)
        for points, color in ((self.left_points, (255, 80, 20)),
                              (self.right_points, (30, 30, 255))):
            for x, y in points:
                cv2.circle(output, (int(x), int(y)), 2, color, -1)
        if self.lines is not None and self.valid:
            left, right = self.lines
            ys = np.asarray([height * 0.42, height - 1])
            left_x = left[0] * ys + left[1]
            right_x = right[0] * ys + right[1]
            center_x = (left_x + right_x) * 0.5
            for coords, color, thickness in ((left_x, (255, 80, 20), 2),
                                              (right_x, (30, 30, 255), 2),
                                              (center_x, (40, 255, 50), 2)):
                points = np.asarray([(int(np.clip(x, 0, width - 1)), int(y))
                                     for x, y in zip(coords, ys)], dtype=np.int32)
                cv2.line(output, tuple(points[0]), tuple(points[1]), color,
                         thickness, cv2.LINE_AA)
        return output


class TapeLaneNode(Node):
    def __init__(self, mode):
        super().__init__('tape_lane_drive')
        self.scan = None
        self.scan_at = None
        self.command_pub = self.create_publisher(Twist, 'cmd_vel', 10) if mode == 'drive' else None
        self.create_subscription(LaserScan, 'scan', self._on_scan, 10)

    def _on_scan(self, message):
        self.scan = message
        self.scan_at = time.monotonic()

    def front_range(self, max_age=0.5, half_angle=math.radians(22.5)):
        if self.scan is None or self.scan_at is None or time.monotonic() - self.scan_at > max_age:
            return None
        readings = []
        for index, distance in enumerate(self.scan.ranges):
            angle = math.atan2(math.sin(self.scan.angle_min + index * self.scan.angle_increment),
                               math.cos(self.scan.angle_min + index * self.scan.angle_increment))
            if abs(angle) <= half_angle and math.isfinite(distance):
                if self.scan.range_min <= distance <= self.scan.range_max:
                    readings.append(distance)
        return min(readings) if readings else None

    def has_other_cmd_vel_publishers(self):
        return any(info.node_name != self.get_name()
                   for info in self.get_publishers_info_by_topic('cmd_vel'))

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


def _overlay_status(overlay, text, safe):
    color = (0, 210, 0) if safe else (0, 0, 255)
    cv2.putText(overlay, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                color, 2, cv2.LINE_AA)


def main():
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
    args = parse_args()
    if not 0 <= args.white_value <= 255 or not 0 <= args.max_saturation <= 255:
        raise SystemExit('white-value와 max-saturation은 0~255 범위여야 합니다.')
    if not 0.05 <= args.roi_top < 0.9:
        raise SystemExit('roi-top은 0.05 이상 0.9 미만이어야 합니다.')
    if args.capture_interval <= 0:
        raise SystemExit('capture-interval은 0보다 커야 합니다.')
    if not 0.05 <= args.lane_width_min < args.lane_width_max <= 1.0:
        raise SystemExit('lane-width-min/max 비율을 확인하세요.')
    if args.mode == 'drive':
        if not (args.enable_motion and args.confirm_supervised_test and args.watchdog_verified):
            raise SystemExit('주행에는 --enable-motion, --confirm-supervised-test, '
                             '--watchdog-verified가 모두 필요합니다.')
        if (not 0 < args.max_linear <= 0.04 or not 0 < args.max_angular <= 0.25
                or args.stop_distance < 0.35):
            raise SystemExit('초기 한도는 max-linear <= 0.04, max-angular <= 0.25, '
                             'stop-distance >= 0.35 입니다.')

    lane = TapeLaneModel(args.white_value, args.max_saturation, args.roi_top,
                         args.lane_width_min, args.lane_width_max)
    camera = MJPEGCamera(args.robot_ip, args.camera_port, name='tape_lane_drive')
    control = PinkyCameraControl(args.robot_ip, name='tape_lane_drive', camera=camera,
                                 ble_name=args.ble_name)
    node = None
    camera_started = False
    try:
        rclpy.init()
        node = TapeLaneNode(args.mode)
        camera.start()
        camera_started = True
        LOGGER.info('흰 테이프 차선 관찰 시작: %s (mode=%s)', args.robot_ip, args.mode)
        print(control.request(True))
        if args.mode == 'drive':
            # Spin while waiting so /scan callbacks run; at least 2 s for discovery.
            started_wait = time.monotonic()
            while time.monotonic() - started_wait < 5.0:
                rclpy.spin_once(node, timeout_sec=0.1)
                if time.monotonic() - started_wait >= 2.0 and node.front_range() is not None:
                    break
            if node.has_other_cmd_vel_publishers():
                raise RuntimeError('다른 /cmd_vel 발행자가 있습니다. Nav2/대시보드를 중지하세요.')
            if node.front_range() is None:
                raise RuntimeError('유효한 LiDAR /scan이 없습니다. 도메인/토픽을 확인하세요.')
            LOGGER.warning('감독형 흰 테이프 차선 추종을 시작합니다. 즉시 정지: q 또는 Ctrl+C')

        previous_sequence = -1
        current_frame = None
        current_white_mask = None
        current_overlay = None
        last_frame_at = None
        last_lane_at = None
        lane_error = None
        command = (0.0, 0.0)
        policy = '초기화'
        last_publish_at = 0.0
        last_logged_policy = None
        last_detection_log_at = 0.0
        last_detection_log_state = None
        capture_dir = None
        capture_count = 0
        last_capture_at = 0.0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            frame_bytes, sequence = camera.latest()
            if frame_bytes is not None and sequence != previous_sequence:
                previous_sequence = sequence
                frame = cv2.imdecode(np.frombuffer(frame_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    current_frame = frame
                    last_frame_at = time.monotonic()
                    white_mask = lane.update(frame)
                    current_white_mask = white_mask
                    last_lane_at = time.monotonic()
                    current_overlay = lane.draw(frame, white_mask)
                    look_y = int(frame.shape[0] * 0.72)
                    center_x = lane.center_x(look_y, frame.shape[1], frame.shape[0])
                    lane_error = None if center_x is None else (
                        (center_x - frame.shape[1] / 2) / (frame.shape[1] / 2))
                    policy = lane.reason if lane.valid else f'{lane.reason} · 정지'
                    if lane.valid and lane_error is not None:
                        display_status = f'OK c={lane.confidence:.2f} dx={lane_error:+.2f}'
                    elif '기하' in lane.reason:
                        display_status = 'STOP: GEOMETRY CHECK'
                    elif '오차' in lane.reason:
                        display_status = 'STOP: POOR LINE FIT'
                    elif '신뢰도' in lane.reason:
                        display_status = 'STOP: LOW CONFIDENCE'
                    elif '중앙 경로' in lane.reason:
                        display_status = 'STOP: CENTER OFFSET'
                    else:
                        display_status = 'STOP: NO LANE PAIR'
                    _overlay_status(current_overlay, display_status, lane.valid)
                    # Raw frames only: labels for training are drawn later in a labeling tool.
                    if (capture_dir is not None
                            and last_frame_at - last_capture_at >= args.capture_interval):
                        capture_count += 1
                        cv2.imwrite(str(capture_dir / f'frame_{capture_count:05d}.jpg'), frame)
                        last_capture_at = last_frame_at
                    if capture_dir is not None:
                        cv2.putText(current_overlay, f'REC {capture_count}', (8, 44),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 255), 2, cv2.LINE_AA)
                    detection_state = (lane.valid, lane.reason)
                    if (detection_state != last_detection_log_state
                            or time.monotonic() - last_detection_log_at >= 1.0):
                        LOGGER.info('lane=%s segments=%s confidence=%.2f fit_error=%s offset=%s',
                                    lane.reason, lane.segment_counts, lane.confidence,
                                    'n/a' if lane.fit_error_px is None else f'{lane.fit_error_px:.1f}px',
                                    'n/a' if lane_error is None else f'{lane_error:+.3f}')
                        last_detection_log_at = time.monotonic()
                        last_detection_log_state = detection_state

            # Keep a live zero command flowing when camera, lane, or LiDAR data
            # goes stale. Pinky bringup otherwise retains its last wheel speed.
            now = time.monotonic()
            if args.mode == 'drive' and now - last_publish_at >= 0.1:
                front = node.front_range()
                stale_frame = last_frame_at is None or now - last_frame_at > 0.5
                stale_lane = last_lane_at is None or now - last_lane_at > 0.5
                if stale_frame or stale_lane or not lane.valid or lane_error is None:
                    command = (0.0, 0.0)
                    policy = '영상/차선 불확실: 정지'
                elif front is None:
                    command = (0.0, 0.0)
                    policy = 'LiDAR 입력 없음/지연: 정지'
                elif front <= args.stop_distance:
                    command = (0.0, 0.0)
                    policy = f'전방 장애물 {front:.2f}m: 정지'
                elif node.has_other_cmd_vel_publishers():
                    command = (0.0, 0.0)
                    policy = '다른 cmd_vel 발행자: 정지'
                else:
                    angular = max(-args.max_angular,
                                  min(args.max_angular, -args.steering_gain * lane_error))
                    command = (args.max_linear, angular)
                    policy = '차선 추종 실험'
                twist = Twist()
                twist.linear.x, twist.angular.z = command
                node.command_pub.publish(twist)
                last_publish_at = now
                if policy != last_logged_policy:
                    LOGGER.warning('%s (v=%.3f m/s, w=%.3f rad/s)',
                                   policy, command[0], command[1])
                    last_logged_policy = policy
                if current_overlay is not None:
                    _overlay_status(current_overlay,
                                    f'{"DRIVE" if command[0] > 0 else "STOP"} '
                                    f'v={command[0]:.2f} w={command[1]:.2f}',
                                    command[0] > 0)

            if not args.headless and current_overlay is not None:
                if last_frame_at is None or time.monotonic() - last_frame_at > 0.5:
                    _overlay_status(current_overlay, 'STOP: CAMERA STALE', False)
                cv2.imshow('Pinky white-tape lane (q quit, s save, r record)', current_overlay)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                if key == ord('r'):
                    if capture_dir is None:
                        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                        capture_dir = Path(args.output_dir).expanduser() / f'dataset_{stamp}'
                        capture_dir.mkdir(parents=True, exist_ok=True)
                        capture_count = 0
                        last_capture_at = 0.0
                        LOGGER.info('학습용 연속 저장 시작: %s (%.1f초 간격, r로 종료)',
                                    capture_dir, args.capture_interval)
                    else:
                        LOGGER.info('학습용 연속 저장 종료: %s (%d장)', capture_dir, capture_count)
                        capture_dir = None
                if key == ord('s') and current_frame is not None:
                    output_dir = Path(args.output_dir).expanduser()
                    output_dir.mkdir(parents=True, exist_ok=True)
                    stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                    raw_path = output_dir / f'tape_lane_{stamp}_raw.jpg'
                    overlay_path = output_dir / f'tape_lane_{stamp}_overlay.jpg'
                    mask_path = output_dir / f'tape_lane_{stamp}_white_mask.png'
                    cv2.imwrite(str(raw_path), current_frame)
                    cv2.imwrite(str(overlay_path), current_overlay)
                    cv2.imwrite(str(mask_path), current_white_mask)
                    (output_dir / f'tape_lane_{stamp}.json').write_text(
                        json.dumps({'timestamp': datetime.now().astimezone().isoformat(),
                                    'white_value': args.white_value,
                                    'max_saturation': args.max_saturation,
                                    'roi_top': args.roi_top,
                                    'lane_valid': lane.valid,
                                    'lane_reason': lane.reason,
                                    'lane_confidence': lane.confidence,
                                    'lane_fit_error_px': lane.fit_error_px,
                                    'lane_segments': lane.segment_counts,
                                    'lane_center_x': lane.center_x(
                                        int(current_frame.shape[0] * 0.72),
                                        current_frame.shape[1], current_frame.shape[0])},
                                   ensure_ascii=False, indent=2), encoding='utf-8')
                    LOGGER.info('관찰 자료 저장: %s', raw_path.parent / f'tape_lane_{stamp}_*')
            elif frame_bytes is None:
                time.sleep(0.02)
    except KeyboardInterrupt:
        LOGGER.info('사용자 종료 요청')
    finally:
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
