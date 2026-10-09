"""Standalone Pinky camera perception and guarded lane-following prototype.

This tool is intentionally separate from the fleet dashboard. Its default
``observe`` mode never publishes velocity commands. ``drive`` is an attended,
low-speed experiment that requires an explicit confirmation and a verified
robot-side cmd_vel watchdog.

The dashboard watches it over relative ROS topics in the robot's domain:
``vision_drive/overlay/compressed`` (the window image), ``vision_drive/state``
(JSON policy and hold) and ``vision_drive/command`` (JSON go/stop from the web).
A drive run starts held, and a crosswalk holds again until a go signal arrives.
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
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import String
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
    parser.add_argument('--crosswalk-action', choices=('wait-signal', 'slow', 'stop', 'stop-then-go', 'ignore'),
                        default='wait-signal',
                        help='wait-signal: 횡단보도에서 멈추고 관제 웹 ▶ 출발(또는 영상 창 g)을 받아야 다시 간다. '
                             'stop-then-go: --crosswalk-stop-seconds 동안 멈춘 뒤 스스로 간다')
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
    parser.add_argument('--corner-min-turn-deg', type=float, default=60.0,
                        help='정면 벽 앞에서 시작한 코너 회전은 odom으로 이 각도(도)를 돌기 전에는 끝내지 않는다. '
                             '20~30°만 돌고 전진해 코너 안쪽 테이프를 넘던 문제를 막는다. 0이면 끔')
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
    def __init__(self, mode, context=None):
        super().__init__('vision_drive', context=context)
        self.mode = mode
        self.scan = None
        self.scan_at = None
        # Pinky mounts the lidar rotated by pi (URDF rplidar_link), so raw scan
        # angle 0 looks backwards. The yaw comes from the robot's static TF.
        self.scan_yaw = None
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.subscription = self.create_subscription(LaserScan, 'scan', self._on_scan, 10)
        # Wheel odometry heading measures how far a corner pivot really turned;
        # the commanded rate alone overestimates it on carpet.
        self.odom_yaw = None
        self.odom_at = None
        self._odom_raw = None
        self.create_subscription(Odometry, 'odom', self._on_odom, qos_profile_sensor_data)
        self.command_pub = self.create_publisher(Twist, 'cmd_vel', 10) if mode == 'drive' else None
        # A drive run never moves on its own: it starts held until a go signal.
        self.hold = None
        self.hold_seq = 0
        if mode == 'drive':
            _hold(self, 'start', time.monotonic(), '시작')
        self.crosswalk = dict(started_at=None, seen_at=None, frames=0, released=False)
        # Dashboard link (same ROS domain): what the window shows, the policy, and go/stop.
        self.overlay_pub = self.create_publisher(CompressedImage, 'vision_drive/overlay/compressed',
                                                 qos_profile_sensor_data)
        self.state_pub = self.create_publisher(String, 'vision_drive/state', qos_profile_sensor_data)
        self.create_subscription(String, 'vision_drive/command', self._on_command, 10)

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

    def _on_command(self, message):
        _command(self, message.data, time.monotonic())

    def publish_overlay(self, overlay):
        if self.overlay_pub.get_subscription_count() == 0:
            return
        ok, encoded = cv2.imencode('.jpg', overlay, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if not ok:
            return
        message = CompressedImage(format='jpeg', data=encoded.tobytes())
        message.header.stamp = self.get_clock().now().to_msg()
        self.overlay_pub.publish(message)

    def publish_state(self, state):
        try:
            data = json.dumps(state, ensure_ascii=False, allow_nan=False)
        except (TypeError, ValueError) as exc:    # a NaN must not stop the drive loop
            LOGGER.warning('관제 상태 발행 생략: %s', exc)
            return
        self.state_pub.publish(String(data=data))

    def _on_odom(self, message):
        heading = yaw(message.pose.pose.orientation)
        if self._odom_raw is None:
            self.odom_yaw = heading
        else:
            # Unwrapped, so a pivot through +-180 degrees keeps counting.
            self.odom_yaw += math.atan2(math.sin(heading - self._odom_raw), math.cos(heading - self._odom_raw))
        self._odom_raw = heading
        self.odom_at = time.monotonic()

    def heading(self, max_age=0.5):
        """Unwrapped odometry yaw in radians (+ = left), or None without fresh odom."""
        if self.odom_at is None or time.monotonic() - self.odom_at > max_age:
            return None
        return self.odom_yaw

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

    def front_clear(self, half_angle=math.radians(10.0)):
        """Median free range in a narrow cone straight ahead, or None without a fresh scan.

        Used to tell "facing down the next corridor" from "still facing the corner".
        """
        beams = self._beams(half_angle)
        if not beams:
            return None
        return float(np.median(beams))

    def open_heading(self, side, reach):
        """Bearing (rad, + = left) of the next corridor on one side, or None if closed.

        Looks 20..150 degrees to `side` in 5-degree steps (median over +-6 degrees)
        and returns the middle of the widest run of directions free beyond `reach`.
        The single farthest ray would point at a far corner instead of along the corridor.
        """
        runs, run = [], []
        for degrees in range(20, 151, 5):
            beams = self._beams(math.radians(6.0), side * math.radians(degrees))
            if beams is None:
                return None
            if beams and float(np.median(beams)) >= reach:
                run.append(degrees)
                continue
            if run:
                runs.append(run)
                run = []
        if run:
            runs.append(run)
        if not runs:
            return None
        widest = max(runs, key=len)
        return side * math.radians((widest[0] + widest[-1]) / 2.0)

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


def _overlay_status(command, hold=None):
    """ASCII status line; OpenCV Hershey fonts draw Korean reasons as '???'."""
    if hold is not None:
        return f"WAIT GO #{hold['id']} {HOLD_ASCII[hold['reason']]} (g / web)"
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


# Why the robot waits for a go signal. Korean for logs and the web, ASCII for the OpenCV window.
HOLD_TEXT = {'start': '출발 신호 대기', 'crosswalk': '횡단보도: 출발 신호 대기', 'operator': '정지: 출발 신호 대기'}
HOLD_ASCII = {'start': 'START', 'crosswalk': 'CROSSWALK', 'operator': 'STOPPED'}
CROSSWALK_GAP = 2.0       # s without the crosswalk in path before it counts as passed
CROSSWALK_FRAMES = 2      # consecutive in-path frames before a crosswalk holds (one-frame false hits)
CROSSING_SEEN_S = 0.5     # keep crossing straight while the crosswalk was in path this recently


def _hold_reason(hold):
    return f"{HOLD_TEXT[hold['reason']]} #{hold['id']}"


def _hold(node, reason, now, source=''):
    """Stop and wait for a go signal. An existing hold stays as it is."""
    if getattr(node, 'hold', None) is None:
        node.hold_seq = getattr(node, 'hold_seq', 0) + 1
        node.hold = dict(id=node.hold_seq, reason=reason, since=now)
        LOGGER.warning('%s%s', _hold_reason(node.hold), f' ({source})' if source else '')
    return node.hold


def _release(node, hold_id, now, source):
    """Go signal for hold `hold_id`.

    The id is the hold the operator saw on screen. A different id means the robot
    has stopped again since (another crosswalk), so the signal is ignored.
    """
    hold = getattr(node, 'hold', None)
    if hold is None or hold['id'] != hold_id:
        LOGGER.warning('출발 신호 무시(%s): 대기 #%s 아님 (지금 %s)', source, hold_id,
                       _hold_reason(hold) if hold else '대기 없음')
        return False
    node.hold = None
    crosswalk = _crosswalk_state(node)
    if crosswalk['seen_at'] is not None:
        crosswalk['released'] = True        # this crosswalk is cleared: no new stop while crossing it
    corner = getattr(node, 'corner', None)
    if corner is not None:
        # The pivot was paused: do not count the wait against its time limits.
        held = now - hold['since']
        corner['started_at'] += held
        corner['phase_at'] += held
    LOGGER.info('출발 신호(%s): %s 해제, %.1f초 대기', source, _hold_reason(hold), now - hold['since'])
    return True


def _command(node, text, now):
    """JSON command from the dashboard: {"action": "go", "hold": <id>} or {"action": "stop"}."""
    try:
        command = json.loads(text)
        action = command['action']
    except (ValueError, TypeError, KeyError):
        LOGGER.warning('알 수 없는 관제 명령: %r', str(text)[:80])
        return False
    if action == 'go':
        hold_id = command.get('hold')
        if isinstance(hold_id, bool) or not isinstance(hold_id, int):
            LOGGER.warning('관제 출발 신호에 대기 번호가 없음: %r', str(text)[:80])
            return False
        return _release(node, hold_id, now, '관제')
    if action == 'stop':
        _hold(node, 'operator', now, '관제')
        return True
    LOGGER.warning('알 수 없는 관제 명령: %r', str(text)[:80])
    return False


def _crosswalk_state(node):
    state = getattr(node, 'crosswalk', None)
    if state is None:
        state = node.crosswalk = dict(started_at=None, seen_at=None, frames=0, released=False)
    return state


def _track_crosswalk(node, seen, now):
    """One crosswalk episode: from the first in-path frame until it is gone for CROSSWALK_GAP."""
    state = _crosswalk_state(node)
    if seen:
        if state['started_at'] is None:
            state['started_at'] = now
        state['seen_at'] = now
        state['frames'] += 1
    else:
        state['frames'] = 0
        if state['seen_at'] is not None and now - state['seen_at'] > CROSSWALK_GAP:
            state.update(started_at=None, seen_at=None, released=False)
    return state


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


def _corner_reach(args):
    """Free range that counts as an open corridor (and as 'not facing the corner wall')."""
    return 2.0 * max(args.corner_wall_distance, args.stop_distance)


def _corner_pick(node, reach=None):
    """Choose the pivot side: lidar first, camera lean as the fallback.

    At a corner the lane continues on the open side while the course wall closes
    the other one. With the front closed (`reach` given), a side that opens into a
    corridor wins outright; 18:39 on 2026-10-09 had side clearances of 0.42 and
    0.35 m, too close to call. Then the side with more free range. The camera
    guess was wrong on 1 of 2 real corners, which cost a 300-degree sweep.
    """
    if reach is not None and hasattr(node, 'open_heading'):
        left, right = node.open_heading(1, reach), node.open_heading(-1, reach)
        if (left is None) != (right is None):
            side = 1 if left is not None else -1
            return side, f"라이다 {'왼쪽' if side > 0 else '오른쪽'}만 {reach:.1f} m 넘게 트임"
    clearance = node.side_clearance()
    if clearance:
        left, right = clearance
        if left >= right * 1.25 and left - right >= 0.05:
            return 1, f'라이다 좌 {left:.2f} m > 우 {right:.2f} m'
        if right >= left * 1.25 and right - left >= 0.05:
            return -1, f'라이다 우 {right:.2f} m > 좌 {left:.2f} m'
    direction = _corner_direction(getattr(node, 'lane_error_history', []))
    detail = f' (라이다 좌 {clearance[0]:.2f} 우 {clearance[1]:.2f} 비슷)' if clearance else ' (라이다 없음)'
    return direction, '카메라 직전 조향' + detail


def _corner_heading(node):
    heading = getattr(node, 'heading', None)
    return heading() if heading else None


def _corner_turned(node, corner, now, args):
    """Signed rotation since the corner began in radians (+ = left).

    Odometry when the corner started with a fresh heading. Without odometry the
    commanded rate times time, which overestimates a pivot on carpet.
    """
    if corner.get('yaw0') is not None:
        heading = _corner_heading(node)
        if heading is not None:
            corner['turned'] = heading - corner['yaw0']
        return corner['turned']
    rate = max(args.max_angular, 1e-6)
    sweep = math.radians(args.corner_max_turn_deg) / rate
    elapsed = now - corner['started_at']
    if elapsed < sweep:
        return corner['first'] * rate * elapsed
    return corner['first'] * rate * (2.0 * sweep - elapsed)


def _corner_pivot(node, args, now, trigger='차선 소실'):
    """Bounded in-place search at a corner.

    Turn toward the chosen side up to corner_max_turn_deg, then sweep back to the
    same angle on the other side, then stop for good. Never moves forward, and the
    heading stays within +/- corner_max_turn_deg of where the corner began. With
    odometry the angles are measured; time limits remain as a backstop.
    """
    corner = getattr(node, 'corner', None)
    if corner is None:
        # Front closed (wall within reach) means a real corner, however it was noticed:
        # on 2026-10-09 18:39 every corner started with the lane lost, not the wall rule.
        reach = _corner_reach(args)
        clear = node.front_clear()
        closed = (trigger.startswith('정면 벽') or bool(node.wall_ahead(args.corner_wall_distance))
                  or (clear is not None and clear < reach))
        first, basis = _corner_pick(node, reach if closed else None)
        target = node.open_heading(first, reach) if closed and hasattr(node, 'open_heading') else None
        if target is not None:
            # Reach the corridor heading before the side limit ends the first sweep.
            target = first * min(abs(target), math.radians(args.corner_max_turn_deg - 5.0))
        corner = node.corner = dict(
            first=first, direction=first, started_at=now, failed=False, overshoot=0,
            yaw0=_corner_heading(node), turned=0.0, phase='first', phase_at=now, target=target,
            min_turn=math.radians(max(0.0, args.corner_min_turn_deg)) if closed and target is None else 0.0)
        if target is not None:
            plan = f'라이다 트인 쪽 {abs(math.degrees(target)):.0f}도까지'
        elif closed:
            plan = f'최소 {args.corner_min_turn_deg:.0f}도'
        else:
            plan = '정면 트임: 차선만 다시 찾음'
        LOGGER.info('코너 진입(%s): %s으로 회전 (%s, %s, 회전량 %s)', trigger, '왼쪽' if first > 0 else '오른쪽',
                    basis, plan, 'odom' if corner['yaw0'] is not None else 'odom 없음: 시간으로 추정')
    rate = args.max_angular
    if rate <= 0.0 or args.corner_max_turn_deg <= 0.0:
        return 0.0, 0.0, '주행 영역 불명확: 정지'
    limit = math.radians(args.corner_max_turn_deg)
    sweep = limit / rate
    if corner['yaw0'] is not None:
        turned = _corner_turned(node, corner, now, args)
        # Allow for a robot that turns slower than commanded before giving up on a side.
        if corner['phase'] == 'first' and (corner['first'] * turned >= limit - 1e-6
                                            or now - corner['started_at'] >= 1.5 * sweep):
            corner['phase'], corner['phase_at'] = 'back', now
        give_up = corner['phase'] == 'back' and (-corner['first'] * turned >= limit - 1e-6
                                                  or now - corner['phase_at'] >= 3.0 * sweep)
    else:
        elapsed = now - corner['started_at']
        corner['phase'] = 'first' if elapsed < sweep else 'back'
        give_up = elapsed >= 3.0 * sweep
    if corner['failed'] or give_up:
        if not corner['failed']:
            corner['failed'] = True
            # Do not spin again at this wall; the obstacle stop-and-wait applies instead.
            node.corner_blocked_until = now + 15.0
            LOGGER.warning('코너에서 차선을 못 찾음: 정지 (양쪽 %.0f도 탐색)', args.corner_max_turn_deg)
        return 0.0, 0.0, '코너에서 차선을 못 찾음: 정지'
    if corner['phase'] == 'first':
        direction, phase = corner['first'], '예상 방향'
    else:
        direction, phase = -corner['first'], '반대쪽'
    corner['direction'] = direction
    side = '왼쪽' if direction > 0 else '오른쪽'
    return 0.0, direction * rate, f'{CORNER_REASON}: {side} ({phase})'


def _corner_keeps_turning(node, args, corner, error, now):
    """While pivoting, keep turning until the robot faces the next corridor.

    With the front closed the robot first turns to the lidar's open-corridor heading
    (or corner_min_turn_deg without one); only then may the lane end the turn. On
    2026-10-09 the floor at a corner looked wide and centred after 20-40 degrees, and
    driving on from there went diagonally into the corner.
    """
    if error is None:
        return True
    if corner['failed']:
        return abs(error) > 0.25            # only a lane ahead ends a failed search
    if corner['phase'] == 'first':
        target = corner.get('target')
        need = abs(target) - math.radians(10.0) if target is not None else corner.get('min_turn', 0.0)
        if abs(_corner_turned(node, corner, now, args)) < need:
            corner['overshoot'] = 0
            return True
        if target is not None and abs(error) <= 0.5:
            return False                    # facing the open corridor and the lane is roughly ahead
    if abs(error) > 0.25 and error * corner['direction'] > 0:
        # Lane firmly on the far side for several frames: we overshot, steering takes over.
        corner['overshoot'] += 1
        return corner['overshoot'] < 3
    corner['overshoot'] = 0
    if abs(error) > 0.3:
        return True                         # lane still off to the turning side
    # Only a clear narrow cone ahead proves the turn is done.
    clear = node.front_clear()
    return clear is None or clear < _corner_reach(args)


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
    crosswalk = _track_crosswalk(node, crosswalk_seen, now)
    if (args.crosswalk_action == 'wait-signal' and crosswalk['frames'] >= CROSSWALK_FRAMES
            and not crosswalk['released']):
        _hold(node, 'crosswalk', now)

    if time.monotonic() - inference_at > 0.5 or camera_at is None or camera_at > 0.5:
        return 0.0, 0.0, '카메라/인식 지연: 정지'
    hold = getattr(node, 'hold', None)
    if hold is not None:
        return 0.0, 0.0, _hold_reason(hold)
    # Released crosswalk: cross it straight. The crosswalk is wider than the lane, so
    # steering on lane+crosswalk pulled the robot ~55 degrees right on 2026-10-09
    # 18:59 and left it 0.15 m from the right wall. Lane steering resumes once past.
    crossing = (args.crosswalk_action in ('wait-signal', 'stop-then-go') and crosswalk['released']
                and crosswalk['seen_at'] is not None and now - crosswalk['seen_at'] <= CROSSING_SEEN_S)
    error = 0.0 if crossing else _lane_error(mask)
    corner = getattr(node, 'corner', None)
    if crossing:
        pass
    elif corner is not None:
        if _corner_keeps_turning(node, args, corner, error, now):
            return _corner_pivot(node, args, now)
        turned = math.degrees(_corner_turned(node, corner, now, args))
        clear = node.front_clear()
        LOGGER.info('코너 회전 끝: %s %.0f도 (%s), 조향 %.2f, 정면 %s', '왼쪽' if turned >= 0 else '오른쪽',
                    abs(turned), 'odom' if corner['yaw0'] is not None else '시간 추정', error,
                    '없음' if clear is None else f'{clear:.2f} m')
        node.corner = None
    elif error is None:
        return _corner_pivot(node, args, now)
    if not crossing:
        history = getattr(node, 'lane_error_history', [])
        history.append((now, error))
        node.lane_error_history = [item for item in history if now - item[0] <= 2.0]
    front = node.front_range(min_width=args.obstacle_min_width)
    if args.mode == 'drive' and front is None:
        return 0.0, 0.0, '라이다 입력 없음/지연: 정지'
    # The lane points straight at a course wall that fills the front: a corner.
    # Turn now instead of crawling up to the wall until the lane disappears.
    if (not crossing and args.corner_wall_distance > 0.0 and abs(error) <= 0.25
            and now >= getattr(node, 'corner_blocked_until', 0.0)
            and node.wall_ahead(args.corner_wall_distance)):
        return _corner_pivot(node, args, now, trigger=f'정면 벽 {front:.2f} m')
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
                elapsed = now - crosswalk['started_at']
                if elapsed < args.crosswalk_stop_seconds:
                    remaining = max(0.0, args.crosswalk_stop_seconds - elapsed)
                    return 0.0, 0.0, f'횡단보도 대기: {remaining:.1f}초'
                crosswalk_released = True
                crosswalk['released'] = True    # from the next frame: cross straight
            if args.crosswalk_action == 'wait-signal' and crosswalk['released']:
                crosswalk_released = True   # held above until the go signal; now crossing
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
    # Slow only between the corner-trigger distance and the stop distance. A 0.25 m
    # band made the robot crawl for ~10 s in front of every corner wall.
    slow_band = max(0.10, args.corner_wall_distance - args.stop_distance)
    if front is not None and front < args.stop_distance + slow_band:
        linear *= max(0.0, (front - args.stop_distance) / slow_band)
        obstacle_limited = True
    if slow_crosswalk:
        reason = '횡단보도 감속'
    elif crossing:
        reason = f'횡단보도 직진 통과{f" (전방 {front:.2f} m 감속)" if obstacle_limited else ""}'
    elif obstacle_limited:
        reason = f'전방 근접 감속 {front:.2f} m'
    elif turn_limited:
        reason = '코너 감속·조향'
    elif crosswalk_released:
        reason = '횡단보도 통과' if args.crosswalk_action == 'wait-signal' else '횡단보도 대기 완료'
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
                or not math.isfinite(args.corner_min_turn_deg)
                or args.corner_min_turn_deg < 0.0
                or (args.corner_max_turn_deg > 0.0 and args.corner_min_turn_deg > args.corner_max_turn_deg)
                or not math.isfinite(args.crosswalk_stop_seconds)
                or args.crosswalk_stop_seconds < 0.0):
            raise SystemExit('초기 주행 한도는 max-linear <= 0.10 m/s, max-angular <= 0.60 rad/s, '
                             'steering-gain <= 2.0, '
                             'turn-radius-limit between 0.03 and 0.30 m, '
                             'stop-distance >= 0.20 m, obstacle-min-width between 0.03 and 0.50 m, '
                             'corner-wall-distance 0 or between stop-distance+0.05 and 0.80 m, '
                             'corner-max-turn-deg between 0 and 120, '
                             'corner-min-turn-deg between 0 and corner-max-turn-deg, '
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
            LOGGER.warning('출발 신호를 기다립니다: 영상 창에서 g, 또는 관제 웹 카드의 ▶ 출발 (space = 정지)')

        inference_at = 0.0
        last_frame_sequence = -1
        camera_age = None
        overlay = None
        last_frame = None
        last_detections = []
        last_latency_ms = None
        current_command = (0.0, 0.0, '초기화')
        last_control_at = 0.0
        last_state_at = 0.0
        lane_error = None
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
                cv2.putText(overlay, _overlay_status(current_command, node.hold), (8, 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 2, cv2.LINE_AA)
                cv2.putText(overlay, 'OBSERVE ONLY' if args.mode == 'observe' else
                            f'VERIFIED DRIVE  v={current_command[0]:.2f}',
                            (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1, cv2.LINE_AA)
                if video_writers:
                    cv2.putText(overlay, 'REC', (overlay.shape[1] - 52, 22), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (0, 0, 255), 2, cv2.LINE_AA)
                node.publish_overlay(overlay)
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
                if node.hold is not None:
                    # A stop from the web takes effect now, not at the next camera frame.
                    linear, angular, reason = 0.0, 0.0, _hold_reason(node.hold)
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

            now = time.monotonic()
            if now - last_state_at >= 0.1:
                last_state_at = now
                hold = node.hold
                node.publish_state(dict(
                    mode=args.mode, policy=current_command[2],
                    linear=round(current_command[0], 3), angular=round(current_command[1], 3),
                    hold=None if hold is None else dict(
                        id=hold['id'], reason=hold['reason'], text=HOLD_TEXT[hold['reason']],
                        age_s=round(now - hold['since'], 1)),
                    crosswalk_action=args.crosswalk_action,
                    detections=sorted({item[0] for item in last_detections}),
                    lane_error=None if lane_error is None else round(lane_error, 3),
                    inference_ms=None if last_latency_ms is None else round(last_latency_ms, 1),
                    camera_age_s=None if camera_age is None else round(camera_age, 2),
                    recording=bool(video_writers)))

            if not args.headless and overlay is not None:
                cv2.imshow('Pinky vision drive (q = quit, g = go, space = stop)', overlay)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                if key == ord('g') and node.hold is not None:
                    _release(node, node.hold['id'], time.monotonic(), '영상 창 g')
                elif key == ord(' '):
                    _hold(node, 'operator', time.monotonic(), '영상 창 space')
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
