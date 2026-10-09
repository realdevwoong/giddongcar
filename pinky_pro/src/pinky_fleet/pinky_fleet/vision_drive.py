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

import yaml

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from pinky_fleet.camera_control import PinkyCameraControl
from pinky_fleet.camera_stream import MJPEGCamera
from pinky_fleet.fleet_common import yaw


LOGGER = logging.getLogger('vision_drive')


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Pinky 카메라/YOLO 인식과 감독형 차선 주행 실험')
    parser.add_argument('--robot-ip', required=True, help='로봇 Wi-Fi IP (예: 192.168.0.6)')
    parser.add_argument('--preset', help='실행 설정 YAML 파일 경로')
    parser.add_argument('--camera-port', type=int, default=5000)
    parser.add_argument('--ble-name', default='',
                        help='카메라를 켤 로봇의 BLE 이름 (예: pinky_6422). 같은 IP 로봇이 여럿일 때 지정')
    parser.add_argument('--model', help='학습한 Ultralytics segmentation .pt 파일 경로 (preset 값 덮어쓰기 가능)')
    parser.add_argument('--device', default='auto', help='Ultralytics 장치: auto, cpu, 0 등')
    parser.add_argument('--driveable-class', default='driveable_area',
                        help='분할 모델의 주행 가능 영역 클래스명')
    parser.add_argument('--obstacle-classes',
                        default='person,bicycle,car,motorcycle,bus,truck,bench,backpack,suitcase,chair',
                        help='보이면 정지할 탐지 클래스명, 쉼표 구분')
    parser.add_argument('--crosswalk-class', default='crosswalk')
    parser.add_argument('--crosswalk-action', choices=('slow', 'stop', 'stop-then-go', 'ignore'),
                        default='stop-then-go')
    parser.add_argument('--crosswalk-stop-seconds', type=float, default=10.0,
                        help='stop-then-go 모드에서 횡단보도 감지 후 정지할 시간')
    parser.add_argument('--mode', choices=('observe', 'drive'), default='observe')
    parser.add_argument('--enable-motion', action='store_true',
                        help='실제 주행 명령을 허용 (drive 모드에서만 적용)')
    parser.add_argument('--confirm-supervised-test', action='store_true',
                        help='장애물 없는 통제 구역에서 직접 감독함을 확인')
    parser.add_argument('--watchdog-verified', action='store_true',
                        help='로봇 측 cmd_vel 정지 watchdog과 비상정지를 확인')
    parser.add_argument('--confirm-attended-test-without-watchdog', action='store_true',
                        help='로봇 watchdog 없이 시험함을 확인; 사람이 로봇 옆에서 물리 비상정지를 잡고 감독')
    parser.add_argument('--max-linear', type=float, default=0.08, help='최대 전진 속도 m/s (한도 0.10)')
    parser.add_argument('--max-angular', type=float, default=0.40, help='최대 회전 속도 rad/s (한도 0.60)')
    parser.add_argument('--steering-gain', type=float, default=1.2,
                        help='먼 쪽 주행 영역 중심 오차에 적용할 조향 gain')
    parser.add_argument('--turn-radius-limit', type=float, default=0.08,
                        help='급회전 때 전진 속도를 제한할 최대 곡률 반경 m')
    parser.add_argument('--stop-distance', type=float, default=0.35,
                        help='라이다 전방 정지 거리 m (제동 시험 전 보수적 초기값)')
    parser.add_argument('--obstacle-min-width', type=float, default=0.12,
                        help='정지 장애물로 볼 LiDAR 물체의 최소 가로 폭 m')
    parser.add_argument('--corner-wall-distance', type=float, default=0.45,
                        help='차선이 정면의 코스 벽을 향하고 벽이 이 거리(m) 안이면 바로 코너로 보고 제자리 회전. '
                             '0이면 차선이 사라질 때까지 기다림')
    parser.add_argument('--corner-max-turn-deg', type=float, default=100.0,
                        help='코너에서 차선을 잃었을 때 한쪽으로 제자리 회전할 최대 각도(도). '
                             '예상 방향으로 먼저 돌고, 못 찾으면 반대쪽으로 같은 각도까지 돈다. 0이면 회전 안 함')
    parser.add_argument('--headless', action='store_true', help='OpenCV 영상 창을 띄우지 않음')
    parser.add_argument('--output-dir', default='~/vision_drive_observations',
                        help='프레임·녹화 영상(r)·실행 로그를 저장할 디렉터리')
    raw_args = list(os.sys.argv[1:] if argv is None else argv)
    args = parser.parse_args(raw_args)
    supplied = {
        action.dest
        for action in parser._actions
        if action.option_strings and any(
            token == option or token.startswith(option + '=')
            for option in action.option_strings for token in raw_args)
    }
    if args.preset:
        preset_path = Path(args.preset).expanduser()
        try:
            preset = yaml.safe_load(preset_path.read_text(encoding='utf-8'))
        except (OSError, yaml.YAMLError) as exc:
            parser.error(f'preset YAML을 읽을 수 없습니다 ({preset_path}): {exc}')
        if not isinstance(preset, dict):
            parser.error('preset은 key-value 형식의 YAML mapping이어야 합니다.')
        # Motion permission and supervision confirmations must be typed by the
        # operator each run; a preset file alone must never make the robot move.
        protected = {
            'robot_ip', 'preset',
            'enable_motion', 'confirm_supervised_test',
            'watchdog_verified', 'confirm_attended_test_without_watchdog',
        }
        actions = {action.dest: action for action in parser._actions}
        for key, value in preset.items():
            if key in protected:
                parser.error(f'preset에서 {key} 설정은 허용하지 않습니다. 명령행에서 직접 지정하세요.')
            action = actions.get(key)
            if action is None or key == 'help':
                parser.error(f'preset에 알 수 없는 설정이 있습니다: {key}')
            if key in supplied:
                continue
            try:
                if isinstance(action, argparse._StoreTrueAction):
                    if not isinstance(value, bool):
                        raise ValueError('boolean 값이어야 합니다')
                elif action.type is not None:
                    value = action.type(value)
                if action.choices is not None and value not in action.choices:
                    raise ValueError(f'허용 값: {", ".join(map(str, action.choices))}')
            except (TypeError, ValueError) as exc:
                parser.error(f'preset 설정 {key} 값이 잘못되었습니다: {exc}')
            setattr(args, key, value)
        LOGGER.info('주행 preset 로드: %s', preset_path)
    if not args.model:
        parser.error('--model 또는 preset의 model 설정이 필요합니다.')
    if args.mode == 'drive':
        LOGGER.warning('최종 설정: mode=drive, enable_motion=%s, supervised=%s, watchdog=%s, attended_without_watchdog=%s',
                       args.enable_motion, args.confirm_supervised_test,
                       args.watchdog_verified, args.confirm_attended_test_without_watchdog)
    return args


class VisionDriveNode(Node):
    def __init__(self, mode):
        super().__init__('vision_drive')
        self.mode = mode
        self.scan = None
        self.scan_at = None
        # Pinky mounts the lidar rotated by pi (URDF rplidar_link), so raw scan
        # angle 0 looks backwards. The yaw comes from the robot's static TF.
        self.scan_yaw = None
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.subscription = self.create_subscription(LaserScan, 'scan', self._on_scan, 10)
        self.command_pub = self.create_publisher(Twist, 'cmd_vel', 10) if mode == 'drive' else None

    def _on_scan(self, message):
        self.scan = message
        self.scan_at = time.monotonic()
        if self.scan_yaw is None:
            try:
                transform = self.tf_buffer.lookup_transform('base_link', message.header.frame_id, Time())
            except TransformException:
                return
            self.scan_yaw = yaw(transform.transform.rotation)
            LOGGER.info('라이다 방향: %s 은(는) base_link 기준 %.0f도 돌아가 있음',
                        message.header.frame_id, math.degrees(self.scan_yaw))

    def _beams(self, half_angle, center=0.0, max_age=0.5):
        """Ranges in a sector of the robot frame; no return counts as range_max (open).

        None when there is no fresh scan or the lidar orientation is unknown.
        """
        if self.scan is None or self.scan_at is None or time.monotonic() - self.scan_at > max_age:
            return None
        if self.scan_yaw is None:
            return None
        scan = self.scan
        beams = []
        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment + self.scan_yaw - center
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) > half_angle:
                continue
            if not math.isfinite(distance) or distance > scan.range_max:
                beams.append(scan.range_max)
            elif distance >= scan.range_min:
                beams.append(distance)
        return beams

    def wall_ahead(self, distance, half_angle=math.radians(30.0)):
        """True when something spans the whole front sector within `distance`.

        A course wall at a corner fills the sector; a box in the lane does not.
        """
        beams = self._beams(half_angle)
        if not beams or len(beams) < 10:
            return False
        return sum(beam <= distance for beam in beams) / len(beams) >= 0.75

    def side_clearance(self, half_angle=math.radians(40.0)):
        """(left, right) median free range around +/-90 degrees, or None."""
        left = self._beams(half_angle, math.radians(90.0))
        right = self._beams(half_angle, -math.radians(90.0))
        if not left or not right:
            return None
        return float(np.median(left)), float(np.median(right))

    def front_range(self, max_age=0.5, half_angle=math.radians(22.5), min_width=0.12):
        """Nearest substantial obstacle ahead in metres.

        None means no fresh scan or unknown lidar orientation (stop). math.inf
        means the scan is fresh and the front sector is clear, which must not be
        confused with a lost lidar.
        """
        if self.scan is None or self.scan_at is None or time.monotonic() - self.scan_at > max_age:
            return None
        if self.scan_yaw is None:
            return None
        points = []
        scan = self.scan
        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment + self.scan_yaw
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= half_angle and math.isfinite(distance):
                if scan.range_min <= distance <= scan.range_max:
                    points.append((index, angle, distance))

        # Ignore isolated returns such as floor specks/tape edges. Keep only
        # connected front-sector clusters whose measured lateral span is wide
        # enough to represent a substantial obstacle.
        clusters = []
        current = []
        previous_index = None
        previous_xy = None
        for index, angle, distance in points:
            xy = (distance * math.cos(angle), distance * math.sin(angle))
            adjacent = (previous_index is not None and index == previous_index + 1
                        and math.dist(xy, previous_xy) <= 0.06)
            if not adjacent and current:
                clusters.append(current)
                current = []
            current.append((xy, distance))
            previous_index, previous_xy = index, xy
        if current:
            clusters.append(current)
        # When the robot front falls on the array boundary (Pinky: scan -pi..pi
        # with yaw pi), the straight-ahead object is split across the end and the
        # start of the array; join it so its width is not halved.
        if (len(clusters) > 1 and points[0][0] == 0 and points[-1][0] == len(scan.ranges) - 1
                and math.dist(clusters[0][0][0], clusters[-1][-1][0]) <= 0.06):
            clusters[0] = clusters.pop() + clusters[0]

        obstacle_ranges = []
        for cluster in clusters:
            if len(cluster) < 2:
                continue
            lateral_span = max(point[0][1] for point in cluster) - min(point[0][1] for point in cluster)
            if lateral_span >= min_width:
                obstacle_ranges.extend(distance for _, distance in cluster)
        return min(obstacle_ranges) if obstacle_ranges else math.inf

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


def _lane_mask(result, args, image_shape):
    """Road surface used for steering: driveable area plus crosswalk.

    Segmentation training assigns overlapping pixels to the smaller crosswalk
    instance, so the driveable mask has a gap exactly where the robot crosses.
    """
    lane = _mask_for_class(result, args.driveable_class, image_shape)
    crosswalk = _mask_for_class(result, args.crosswalk_class, image_shape)
    if crosswalk is None:
        return lane
    return crosswalk if lane is None else lane | crosswalk


def _lane_error(mask):
    """Estimate steering from several lookahead rows to anticipate bends."""
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
        # A sharp bend can move the visible driveable patch slightly off the
        # exact image center. Reacquire only a component that still intersects
        # a narrow corridor near the bottom of the image.
        half_corridor = int(width * 0.28)
        for fraction in (0.95, 0.88, 0.84):
            row_y = min(height - 1, int(height * fraction))
            row_labels = labels[row_y, max(0, x_center - half_corridor):
                                min(width, x_center + half_corridor + 1)]
            candidates = row_labels[row_labels > 0]
            if candidates.size:
                ids, counts = np.unique(candidates, return_counts=True)
                component = int(ids[np.argmax(counts)])
                break
        if component == 0:
            return None
    # The lower row reacts late at corners. Blend centers from farther lookahead
    # rows so the steering starts following a bend before the near mask disappears.
    centers = []
    weights = []
    # Far rows predict the upcoming bend; near rows keep the robot centered.
    for fraction, weight in ((0.52, 0.65), (0.62, 0.18), (0.72, 0.11), (0.82, 0.06)):
        row = labels[min(height - 1, int(height * fraction))]
        xs = np.flatnonzero(row == component)
        if xs.size >= max(3, int(width * 0.025)):
            centers.append((float(xs[0]) + float(xs[-1])) / 2.0)
            weights.append(weight)
    # A single thin row is too fragile to steer from; fail closed until a path
    # direction is supported by multiple parts of the visible mask.
    if len(centers) < 3:
        return None
    lane_center = float(np.average(centers, weights=weights))
    return (lane_center - x_center) / max(1.0, width / 2.0)


def _overlay_status(command):
    """ASCII status line; OpenCV Hershey fonts draw Korean reasons as '???'."""
    linear, angular, _ = command
    if linear > 0:
        return f'GO v={linear:.2f} w={angular:+.2f}'
    if angular:
        return f'TURN w={angular:+.2f}'
    return 'STOP (reason: terminal policy=)'


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


CORNER_REASON = '코너 제자리 회전'


def _corner_direction(history, window=1.0):
    """Guess the turn from lane errors just before the lane disappeared.

    +1 turns left (counter-clockwise), -1 right. A positive error means the
    path ahead leaned right. In recorded 90-degree corners this matched the
    real turn 6 times out of 8, so _corner_pivot also searches the other side.
    """
    if not history:
        return 1
    last_at = history[-1][0]
    recent = [error for at, error in history if last_at - at <= window]
    return -1 if sum(recent) / len(recent) > 0 else 1


def _corner_pick(node):
    """Choose the pivot side: lidar clearance first, camera lean as the fallback.

    At a corner the lane continues on the open side while the course wall closes
    the other one, so the side with more free range is the turn. The camera guess
    was wrong on 1 of 2 real corners (2026-10-09), which cost a 300-degree sweep.
    """
    clearance = node.side_clearance()
    if clearance:
        left, right = clearance
        if left >= right * 1.3 and left - right >= 0.15:
            return 1, f'라이다 좌 {left:.2f} m > 우 {right:.2f} m'
        if right >= left * 1.3 and right - left >= 0.15:
            return -1, f'라이다 우 {right:.2f} m > 좌 {left:.2f} m'
    direction = _corner_direction(getattr(node, 'lane_error_history', []))
    detail = f' (라이다 좌 {clearance[0]:.2f} 우 {clearance[1]:.2f} 비슷)' if clearance else ' (라이다 없음)'
    return direction, '카메라 직전 조향' + detail


def _corner_pivot(node, args, now):
    """Bounded in-place search at a corner.

    Turn toward the chosen side up to corner_max_turn_deg, then sweep back to the
    same angle on the other side, then stop for good. Never moves forward, and the
    heading stays within +/- corner_max_turn_deg of where the corner began.
    """
    corner = getattr(node, 'corner', None)
    if corner is None:
        first, basis = _corner_pick(node)
        corner = node.corner = dict(first=first, direction=first, started_at=now, failed=False)
        LOGGER.info('코너 진입: %s으로 회전 (%s)', '왼쪽' if first > 0 else '오른쪽', basis)
    rate = args.max_angular
    if rate <= 0.0 or args.corner_max_turn_deg <= 0.0:
        return 0.0, 0.0, '주행 영역 불명확: 정지'
    sweep = math.radians(args.corner_max_turn_deg) / rate
    elapsed = now - corner['started_at']
    if corner['failed'] or elapsed >= 3.0 * sweep:
        if not corner['failed']:
            corner['failed'] = True
            # Do not spin again at this wall; the obstacle stop-and-wait applies instead.
            node.corner_blocked_until = now + 15.0
            LOGGER.warning('코너에서 차선을 못 찾음: 정지 (양쪽 %.0f도 탐색)', args.corner_max_turn_deg)
        return 0.0, 0.0, '코너에서 차선을 못 찾음: 정지'
    if elapsed < sweep:
        direction, phase = corner['first'], '예상 방향'
    else:
        direction, phase = -corner['first'], '반대쪽'
    corner['direction'] = direction
    side = '왼쪽' if direction > 0 else '오른쪽'
    return 0.0, direction * rate, f'{CORNER_REASON}: {side} ({phase})'


def _corner_keeps_turning(node, args, corner, error):
    """While pivoting, keep turning until the lane sits ahead and the wall is gone.

    Resuming early (lane at the side, wall still close) drove a forward arc that
    cut the inner tape on 2026-10-09.
    """
    if error is None:
        return True
    if corner['failed']:
        return abs(error) > 0.25            # only a lane ahead ends a failed search
    if abs(error) > 0.25:
        return error * corner['direction'] < 0   # still entering from the turning side
    return node.wall_ahead(args.corner_wall_distance)


def _drive_guard(command, front, stop_distance):
    """Last check before publishing a drive command.

    No lidar stops everything. An obstacle within stop_distance blocks forward
    motion, but a corner pivot in place may continue: at a corner the course wall
    is often that close while the robot only needs to turn.
    """
    linear, angular, reason = command
    if front is None:
        return 0.0, 0.0, '라이다 입력 없음/지연: 정지'
    if front <= stop_distance and not (linear == 0.0 and reason.startswith(CORNER_REASON)):
        return 0.0, 0.0, f'전방 장애물 {front:.2f} m: 정지'
    return command


def _policy(node, mask, detections, args, inference_at, camera_at):
    """Fail closed when perception or robot sensor data is unavailable/stale."""
    height, width = mask.shape if mask is not None else (0, 0)
    stop_classes = {name.strip() for name in args.obstacle_classes.split(',') if name.strip()}
    crosswalk_seen = False
    for label, confidence, x1, y1, x2, y2 in detections:
        if confidence < 0.45:
            continue
        in_path = (y2 >= height * 0.48 and x2 >= width * 0.2 and x1 <= width * 0.8)
        if in_path and label == args.crosswalk_class:
            crosswalk_seen = True

    now = time.monotonic()
    if crosswalk_seen:
        if getattr(args, '_crosswalk_started_at', None) is None:
            args._crosswalk_started_at = now
        args._crosswalk_last_seen_at = now
    elif (getattr(args, '_crosswalk_last_seen_at', None) is not None
          and now - args._crosswalk_last_seen_at > 0.75):
        args._crosswalk_started_at = None
        args._crosswalk_last_seen_at = None

    if time.monotonic() - inference_at > 0.5 or camera_at is None or camera_at > 0.5:
        return 0.0, 0.0, '카메라/인식 지연: 정지'
    error = _lane_error(mask)
    corner = getattr(node, 'corner', None)
    if corner is not None:
        if _corner_keeps_turning(node, args, corner, error):
            return _corner_pivot(node, args, now)
        node.corner = None
    elif error is None:
        return _corner_pivot(node, args, now)
    history = getattr(node, 'lane_error_history', [])
    history.append((now, error))
    node.lane_error_history = [item for item in history if now - item[0] <= 2.0]
    front = node.front_range(min_width=args.obstacle_min_width)
    if args.mode == 'drive' and front is None:
        return 0.0, 0.0, '라이다 입력 없음/지연: 정지'
    # The lane points straight at a course wall that fills the front: a corner.
    # Turn now instead of crawling up to the wall until the lane disappears.
    if (args.corner_wall_distance > 0.0 and abs(error) <= 0.25
            and now >= getattr(node, 'corner_blocked_until', 0.0)
            and node.wall_ahead(args.corner_wall_distance)):
        return _corner_pivot(node, args, now)
    if front is not None and front <= args.stop_distance:
        return 0.0, 0.0, f'전방 장애물 {front:.2f} m: 정지'

    slow_crosswalk = False
    crosswalk_released = False
    for label, confidence, x1, y1, x2, y2 in detections:
        if confidence < 0.45:
            continue
        in_path = (y2 >= height * 0.48 and x2 >= width * 0.2 and x1 <= width * 0.8)
        if in_path and label in stop_classes:
            return 0.0, 0.0, f'{label} 탐지: 정지'
        if in_path and label == args.crosswalk_class:
            if args.crosswalk_action == 'stop':
                return 0.0, 0.0, '횡단보도: 정지 정책'
            if args.crosswalk_action == 'stop-then-go':
                elapsed = now - args._crosswalk_started_at
                if elapsed < args.crosswalk_stop_seconds:
                    remaining = max(0.0, args.crosswalk_stop_seconds - elapsed)
                    return 0.0, 0.0, f'횡단보도 대기: {remaining:.1f}초'
                crosswalk_released = True
            slow_crosswalk = args.crosswalk_action == 'slow'

    linear = min(args.max_linear, 0.02 if slow_crosswalk else args.max_linear)
    angular = max(-args.max_angular, min(args.max_angular, -args.steering_gain * error))
    turn_limited = False
    # Blend into a curvature speed cap as the predicted turn gets sharper.
    # This avoids carrying straight-line speed into a corner while keeping the
    # speed transition smooth. At the angular limit, v/|w| is capped by radius.
    angular_demand = abs(angular)
    turn_start = min(0.03, args.max_angular * 0.5)
    turn_blend = min(1.0, max(0.0, (angular_demand - turn_start)
                              / max(1e-6, args.max_angular - turn_start)))
    curve_speed_limit = min(linear, angular_demand * args.turn_radius_limit)
    if turn_blend > 0.0 and curve_speed_limit < linear:
        linear = linear * (1.0 - turn_blend) + curve_speed_limit * turn_blend
        turn_limited = True
    obstacle_limited = False
    if front is not None and front < args.stop_distance + 0.25:
        linear *= max(0.0, (front - args.stop_distance) / 0.25)
        obstacle_limited = True
    if slow_crosswalk:
        reason = '횡단보도 감속'
    elif obstacle_limited:
        reason = f'전방 근접 감속 {front:.2f} m'
    elif turn_limited:
        reason = '코너 감속·조향'
    elif crosswalk_released:
        reason = '횡단보도 대기 완료'
    else:
        reason = '차선 영역 추종'
    return linear, angular, reason


def main():
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
    args = parse_args()
    # Keep every run's log next to the recordings so a field run can be reviewed later.
    log_dir = Path(args.output_dir).expanduser()
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = logging.FileHandler(log_dir / f'vision_drive_{datetime.now():%Y%m%d_%H%M%S}_{args.mode}.log',
                                   encoding='utf-8')
    log_file.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(message)s'))
    logging.getLogger().addHandler(log_file)
    LOGGER.info('실행 로그: %s', log_file.baseFilename)
    LOGGER.info('설정: %s', {key: value for key, value in vars(args).items() if not key.startswith('_')})
    model_path = Path(args.model).expanduser()
    if not model_path.is_file():
        raise SystemExit(f'학습 모델 파일을 찾을 수 없습니다: {model_path}')
    if args.mode == 'drive':
        stop_assurance = (args.watchdog_verified
                          or args.confirm_attended_test_without_watchdog)
        if not (args.enable_motion and args.confirm_supervised_test and stop_assurance):
            raise SystemExit('주행 모드에는 --enable-motion, --confirm-supervised-test, '
                             '그리고 --watchdog-verified 또는 '
                             '--confirm-attended-test-without-watchdog가 필요합니다.')
        wall_distance_ok = (args.corner_wall_distance == 0.0
                            or args.stop_distance + 0.05 <= args.corner_wall_distance <= 0.80)
        if (args.max_linear <= 0 or args.max_linear > 0.10
                or args.max_angular <= 0 or args.max_angular > 0.60
                or not math.isfinite(args.corner_wall_distance) or not wall_distance_ok
                or not math.isfinite(args.steering_gain)
                or args.steering_gain <= 0.0 or args.steering_gain > 2.0
                or not math.isfinite(args.turn_radius_limit)
                or args.turn_radius_limit < 0.03 or args.turn_radius_limit > 0.30
                or args.stop_distance < 0.20
                or not math.isfinite(args.obstacle_min_width)
                or args.obstacle_min_width < 0.03 or args.obstacle_min_width > 0.50
                or not math.isfinite(args.corner_max_turn_deg)
                or args.corner_max_turn_deg < 0.0
                or args.corner_max_turn_deg > 120.0
                or not math.isfinite(args.crosswalk_stop_seconds)
                or args.crosswalk_stop_seconds < 0.0):
            raise SystemExit('초기 주행 한도는 max-linear <= 0.10 m/s, max-angular <= 0.60 rad/s, '
                             'steering-gain <= 2.0, '
                             'turn-radius-limit between 0.03 and 0.30 m, '
                             'stop-distance >= 0.20 m, obstacle-min-width between 0.03 and 0.50 m, '
                             'corner-wall-distance 0 or between stop-distance+0.05 and 0.80 m, '
                             'corner-max-turn-deg between 0 and 120, '
                             'crosswalk-stop-seconds >= 0 입니다.')

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
    control = PinkyCameraControl(args.robot_ip, name='vision_drive', camera=camera,
                                 ble_name=args.ble_name)
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
            # Spin while waiting so /scan callbacks run; at least 2 s for discovery.
            started_wait = time.monotonic()
            while time.monotonic() - started_wait < 5.0:
                rclpy.spin_once(node, timeout_sec=0.1)
                if (time.monotonic() - started_wait >= 2.0
                        and node.front_range(min_width=args.obstacle_min_width) is not None):
                    break
            others = node.external_cmd_vel_publishers()
            if others:
                raise RuntimeError('다른 /cmd_vel 발행자가 있습니다. Nav2/대시보드를 중지한 뒤 다시 실행하세요: '
                                   + ', '.join(info.node_name for info in others))
            if node.scan is not None and node.scan_yaw is None:
                raise RuntimeError(f'라이다 방향 TF(base_link → {node.scan.header.frame_id})를 받지 못했습니다. '
                                   '로봇 bringup의 robot_state_publisher가 켜져 있는지 확인하세요.')
            if node.front_range(min_width=args.obstacle_min_width) is None:
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
                mask = _lane_mask(result, args, frame.shape)
                detections = _detections(result)
                inference_at = time.monotonic()
                last_latency_ms = (inference_at - inference_started) * 1000.0
                current_command = _policy(node, mask, detections, args, inference_at, camera_age)
                overlay = result.plot()
                lane_error = _lane_error(mask)
                if lane_error is not None:
                    height, width = overlay.shape[:2]
                    preview_x = int(np.clip(width / 2 + lane_error * width / 2, 0, width - 1))
                    preview_y = int(height * 0.62)
                    cv2.circle(overlay, (preview_x, preview_y), 6, (255, 0, 255), -1)
                    cv2.line(overlay, (width // 2, preview_y), (preview_x, preview_y),
                             (255, 0, 255), 2)
                color = (0, 220, 0) if current_command[0] > 0 else (0, 0, 255)
                cv2.putText(overlay, _overlay_status(current_command), (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                            0.52, color, 2, cv2.LINE_AA)
                cv2.putText(overlay, 'OBSERVE ONLY' if args.mode == 'observe' else
                            f'VERIFIED DRIVE  v={current_command[0]:.2f}',
                            (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1, cv2.LINE_AA)
                if video_writers:
                    cv2.putText(overlay, 'REC', (overlay.shape[1] - 52, 22), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (0, 0, 255), 2, cv2.LINE_AA)
                names = ', '.join(sorted({item[0] for item in detections})) or '탐지 없음'
                LOGGER.info('mode=%s policy=%s command=(v=%.3f,w=%.3f) detections=%s inference=%.1fms',
                            args.mode, current_command[2], current_command[0], current_command[1],
                            names, last_latency_ms)
                last_frame = frame.copy()
                last_detections = detections
                if video_writers:
                    video_writers[0].write(last_frame)
                    video_writers[1].write(overlay)

            if args.mode == 'drive' and time.monotonic() - last_control_at >= 0.1:
                linear, angular, reason = current_command
                if time.monotonic() - inference_at > 0.5 or camera_age is None or camera_age > 0.5:
                    linear, angular, reason = 0.0, 0.0, '카메라/인식 지연: 정지'
                linear, angular, reason = _drive_guard(
                    (linear, angular, reason), node.front_range(min_width=args.obstacle_min_width),
                    args.stop_distance)
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
                elif key == ord('r'):
                    if video_writers:
                        for writer in video_writers:
                            writer.release()
                        video_writers = None
                        LOGGER.info('영상 녹화 완료: %s', recording_stem)
                    elif last_frame is not None:
                        output_dir = Path(args.output_dir).expanduser()
                        output_dir.mkdir(parents=True, exist_ok=True)
                        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                        recording_stem = output_dir / f'{args.mode}_{stamp}'
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
                            LOGGER.error('영상 파일을 열 수 없습니다: %s', recording_stem)
                        else:
                            video_writers = writers
                            LOGGER.info('영상 녹화 시작: %s (r 키로 종료)', recording_stem)
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
            LOGGER.info('영상 녹화 저장: %s', recording_stem)
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
