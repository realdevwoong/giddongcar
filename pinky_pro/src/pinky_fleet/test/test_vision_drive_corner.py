"""vision_drive 코너: 차선을 잃으면 제자리에서 예상 방향 → 반대쪽으로 찾고, 못 찾으면 정지."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

import pinky_fleet.vision_drive as vd

MASK = np.zeros((240, 320), np.uint8)
SWEEP = math.radians(100.0) / 0.25     # 기본 100도, 0.25 rad/s → 한쪽 약 7초


@pytest.fixture
def drive(monkeypatch):
    """가짜 시계와 가짜 조향값으로 _policy를 한 프레임씩 돌린다."""
    state = SimpleNamespace(now=100.0, error=None, wall=False, sides=None, clear=math.inf, front=math.inf,
                            heading=None)
    monkeypatch.setattr(vd, 'time', SimpleNamespace(monotonic=lambda: state.now))
    monkeypatch.setattr(vd, '_lane_error', lambda mask: state.error)
    args = vd.parse_args(['--robot-ip', '192.0.2.1', '--model', 'm.pt', '--mode', 'drive',
                          '--max-linear', '0.05', '--max-angular', '0.25'])   # SWEEP 계산과 맞춤
    node = SimpleNamespace(front_range=lambda min_width: state.front,   # 정지 판단용 전방 장애물 거리
                           front_clear=lambda: state.clear,             # 정면 ±10° 빈 거리
                           wall_ahead=lambda distance: state.wall,      # 정면 벽(코너) 여부
                           side_clearance=lambda: state.sides,          # (왼쪽, 오른쪽) 여유 거리
                           heading=lambda: state.heading)               # odom 방향(rad). None이면 시간으로 추정

    def step(error, dt=0.1, **lidar):
        state.now += dt
        state.error = error
        for key, value in lidar.items():
            setattr(state, key, value)
        return vd._policy(node, MASK, [], args, state.now, 0.0)
    step.node, step.args = node, args
    return step


def test_lane_lost_after_right_lean_pivots_right_in_place(drive):
    for _ in range(5):
        drive(0.12)
    linear, angular, reason = drive(None)
    assert linear == 0.0 and angular == pytest.approx(-0.25)
    assert reason.startswith(vd.CORNER_REASON) and '오른쪽' in reason


def test_corner_search_tries_other_side_then_stops(drive):
    drive(-0.1)
    assert drive(None)[1] == pytest.approx(0.25)               # 왼쪽 먼저
    assert drive(None, dt=SWEEP + 0.1)[1] == pytest.approx(-0.25)    # 못 찾으면 반대쪽
    linear, angular, reason = drive(None, dt=2 * SWEEP + 0.1)
    assert (linear, angular) == (0.0, 0.0) and '못 찾음' in reason


def test_pivot_continues_until_lane_is_ahead(drive):
    drive(0.1)
    drive(None)                                   # 오른쪽으로 회전 시작
    _, angular, reason = drive(0.8)               # 차선이 오른쪽에서 들어오는 중
    assert angular == pytest.approx(-0.25) and reason.startswith(vd.CORNER_REASON)
    assert drive(0.8, clear=0.5)[2].startswith(vd.CORNER_REASON)   # 정면이 아직 막혀 있으면 계속 회전
    linear, _, reason = drive(0.3, clear=2.0)     # 정면이 트이고 차선이 대략 앞 → 정상 주행
    assert linear > 0.0 and not reason.startswith(vd.CORNER_REASON)
    assert drive.node.corner is None


def test_zero_turn_angle_stops_instead_of_pivoting(drive):
    drive.args.corner_max_turn_deg = 0.0
    drive(0.1)
    assert drive(None)[:2] == (0.0, 0.0)


def test_corner_direction_uses_only_recent_errors():
    assert vd._corner_direction([]) == 1
    assert vd._corner_direction([(0.0, -0.5), (2.0, 0.1), (2.5, 0.05)]) == -1   # 1초 넘은 값은 무시
    assert vd._corner_direction([(0.0, 0.2), (0.5, -0.4)]) == 1


@pytest.mark.parametrize('command, front, expected_angular', [
    ((0.05, 0.0, '차선 영역 추종'), math.inf, 0.0),
    ((0.05, 0.0, '차선 영역 추종'), 0.30, 0.0),                         # 앞에 막힘 → 정지
    ((0.0, -0.25, f'{vd.CORNER_REASON}: 오른쪽 (예상 방향)'), 0.30, -0.25),  # 코너 벽 앞 회전은 허용
    ((0.0, -0.25, f'{vd.CORNER_REASON}: 오른쪽 (예상 방향)'), None, 0.0),    # 라이다 없으면 정지
])
def test_drive_guard(command, front, expected_angular):
    linear, angular, _ = vd._drive_guard(command, front, 0.35)
    assert angular == pytest.approx(expected_angular)
    assert linear == (command[0] if front == math.inf else 0.0)


def test_wall_ahead_with_straight_lane_turns_toward_lidar_open_side(drive):
    # 실물 16:53: 차선은 정면 벽을 향해 곧고(e≈0) 라이다가 벽을 본다 → 기어가지 않고 바로 회전
    for _ in range(5):
        drive(0.0)
    linear, angular, reason = drive(0.0, wall=True, sides=(1.6, 0.3), clear=0.4)
    assert linear == 0.0 and angular == pytest.approx(0.25)
    assert reason.startswith(vd.CORNER_REASON) and '왼쪽' in reason


def test_lidar_tie_falls_back_to_camera_lean(drive):
    for _ in range(5):
        drive(0.12)
    _, angular, reason = drive(0.12, wall=True, sides=(0.8, 0.8), clear=0.4)
    assert angular == pytest.approx(-0.25) and '오른쪽' in reason


def test_pivot_holds_until_front_cone_is_clear(drive):
    drive(0.0, wall=True, sides=(1.6, 0.3), clear=0.4)    # 왼쪽 회전 시작
    for _ in range(12):                                   # 실물: 20°쯤 돌면 벽 판정이 풀리고 바닥이 넓게 보인다
        _, angular, reason = drive(0.0, wall=False, clear=0.6)
        assert angular == pytest.approx(0.25) and reason.startswith(vd.CORNER_REASON)
    assert drive(0.0, clear=2.0)[2].startswith(vd.CORNER_REASON)   # 정면이 트여도 아직 약 19°: 계속 회전
    linear, _, reason = drive(0.0, dt=3.0, clear=2.0)     # 60° 넘게 돌았고 새 복도 정면 → 전진
    assert linear > 0.0 and not reason.startswith(vd.CORNER_REASON)
    assert drive.node.corner is None


def test_wall_corner_turns_at_least_min_angle_by_odom(drive):
    # 실물 17:15: 20~30°만 돌고 바닥이 넓게 보이자 전진해 코너 안쪽으로 파고들었다
    drive(0.0, wall=True, sides=(1.6, 0.3), clear=0.4, heading=0.0)
    for degrees in (10, 25, 40, 55):                       # 차선이 앞에 있고 정면도 트였지만 아직 60° 전
        _, angular, reason = drive(0.0, wall=False, clear=2.0, heading=math.radians(degrees))
        assert angular == pytest.approx(0.25) and reason.startswith(vd.CORNER_REASON)
    linear, _, _ = drive(0.0, clear=2.0, heading=math.radians(65))
    assert linear > 0.0 and drive.node.corner is None


def test_lane_lost_away_from_wall_needs_no_min_angle(drive):
    drive(-0.1, heading=0.0)
    drive(None, heading=0.0)                               # 벽 없이 차선만 잃음: 다시 찾으면 바로 출발
    linear, _, _ = drive(0.0, clear=2.0, heading=math.radians(15))
    assert linear > 0.0


def test_odom_limits_each_side_by_measured_angle(drive):
    drive(-0.1, heading=0.0)
    assert drive(None, heading=0.0)[1] == pytest.approx(0.25)                 # 왼쪽 먼저
    # 명령 속도로는 100°를 넘을 시간이지만 실제로는 70°만 돌았다 → 아직 왼쪽
    assert drive(None, dt=SWEEP + 0.1, heading=math.radians(70))[1] == pytest.approx(0.25)
    assert drive(None, heading=math.radians(101))[1] == pytest.approx(-0.25)  # 100° 도달 → 반대쪽
    assert drive(None, dt=2.0, heading=math.radians(-20))[1] == pytest.approx(-0.25)
    linear, angular, reason = drive(None, heading=math.radians(-101))         # 반대쪽 100°까지 없음 → 정지
    assert (linear, angular) == (0.0, 0.0) and '못 찾음' in reason


def test_odom_stuck_robot_still_gives_up_by_time(drive):
    drive(-0.1, heading=0.0)
    drive(None, heading=0.0)
    assert drive(None, dt=1.5 * SWEEP + 0.1, heading=0.0)[1] == pytest.approx(-0.25)   # 안 돌아도 시간이 지나면 반대쪽
    linear, angular, reason = drive(None, dt=3.0 * SWEEP + 0.1, heading=0.0)
    assert (linear, angular) == (0.0, 0.0) and '못 찾음' in reason


def test_pivot_exits_after_persistent_overshoot(drive):
    drive(-0.1)
    drive(None)                                           # 정면이 트인 채 차선만 잃음: 왼쪽 회전
    assert drive(0.4)[2].startswith(vd.CORNER_REASON)     # 한두 프레임 튄 값은 무시
    assert drive(0.4)[2].startswith(vd.CORNER_REASON)
    linear, angular, reason = drive(0.4)                  # 3프레임 연속 반대쪽 → 조향에 맡김
    assert not reason.startswith(vd.CORNER_REASON) and angular < 0.0


def test_closed_front_lane_lost_corner_also_turns_min_angle(drive):
    # 실물 18:39: 코너는 모두 '차선 소실'로 시작했다. 정면이 막혔으면 벽 코너와 같이 최소 60°
    drive(-0.1, heading=0.0)
    drive(None, clear=0.4, heading=0.0)
    for degrees in (15, 25, 40):                          # 반대쪽에 차선이 보여도(25° 종료 사례) 아직 돈다
        assert drive(0.4, clear=2.0, heading=math.radians(degrees))[2].startswith(vd.CORNER_REASON)
    assert drive(-0.4, clear=2.0, heading=math.radians(50))[2].startswith(vd.CORNER_REASON)
    linear, _, _ = drive(-0.1, clear=2.0, heading=math.radians(65))
    assert linear > 0.0 and drive.node.corner is None


def test_closed_corner_turns_to_lidar_corridor_heading(drive):
    # 실물 18:39:13: 40°에서 차선이 왼쪽(-0.40)에 보이고 정면 1.10 m라 끝내고 대각선으로 갔다
    drive.node.open_heading = lambda side, reach: side * math.radians(85.0)
    drive(0.0, heading=0.0)
    drive(None, clear=0.4, heading=0.0)                   # 정면 막힘 + 차선 소실 → 목표 85°
    assert drive.node.corner['target'] == pytest.approx(math.radians(85.0))
    for degrees in (40, 60, 70):
        assert drive(-0.4, clear=1.1, heading=math.radians(degrees))[2].startswith(vd.CORNER_REASON)
    linear, angular, _ = drive(-0.4, clear=1.1, heading=math.radians(77))   # 목표 -10° 안: 차선이 대략 앞이면 출발
    assert linear > 0.0 and angular > 0.0 and drive.node.corner is None


def test_corridor_side_wins_when_side_clearances_are_close(drive):
    drive.node.open_heading = lambda side, reach: None if side > 0 else -math.radians(80.0)
    for _ in range(3):
        drive(-0.05)                                       # 카메라는 왼쪽으로 기울었지만
    _, angular, reason = drive(None, clear=0.4, sides=(0.42, 0.35))
    assert angular < 0.0 and '오른쪽' in reason            # 오른쪽만 복도로 트임


def test_slowdown_band_starts_at_corner_distance(drive):
    assert drive(0.0, front=0.50)[0] == pytest.approx(drive.args.max_linear)     # 0.45 m 밖: 감속 없음
    linear, _, reason = drive(0.0, front=0.40)                                   # 0.35~0.45 m: 비례 감속
    assert 0.0 < linear < drive.args.max_linear and reason.startswith('전방 근접 감속')


def test_failed_search_stays_stopped_and_does_not_spin_again(drive):
    drive(-0.1)
    drive(None)
    linear, angular, reason = drive(None, dt=3 * SWEEP + 0.1)
    assert (linear, angular) == (0.0, 0.0) and '못 찾음' in reason
    for _ in range(5):                                # 차선이 계속 없어도 다시 돌지 않는다
        assert drive(None)[:2] == (0.0, 0.0)
    assert drive(0.8)[:2] == (0.0, 0.0)               # 옆에 보이는 것으로는 부족
    linear, _, reason = drive(0.0, wall=True)         # 앞에 차선: 출발. 같은 벽으로 코너 재진입은 15초 금지
    assert linear > 0.0 and not reason.startswith(vd.CORNER_REASON)


def test_odom_heading_unwraps_and_expires(monkeypatch):
    from nav_msgs.msg import Odometry
    clock = SimpleNamespace(now=50.0)
    monkeypatch.setattr(vd, 'time', SimpleNamespace(monotonic=lambda: clock.now))
    node = SimpleNamespace(odom_yaw=None, odom_at=None, _odom_raw=None)

    def odom(degrees):
        message = Odometry()
        message.pose.pose.orientation.z = math.sin(math.radians(degrees) / 2)
        message.pose.pose.orientation.w = math.cos(math.radians(degrees) / 2)
        vd.VisionDriveNode._on_odom(node, message)

    assert vd.VisionDriveNode.heading(node) is None                 # odom을 아직 못 받음
    odom(170.0)
    odom(-170.0)                                                    # +-180°를 지나도 이어서 센다
    assert math.degrees(vd.VisionDriveNode.heading(node)) == pytest.approx(190.0)
    clock.now += 0.6
    assert vd.VisionDriveNode.heading(node) is None                 # 0.5초 넘게 끊기면 쓰지 않는다


# ---- 라이다 스캔 합성: 코너 모양 벽에서 360° 레이저 ----

def _ray_scan(walls, x, y, step=math.radians(1.0)):
    """로봇 (x, y), 정면 +x에서 벽 선분들까지의 거리. Pinky와 같은 -pi..pi 배열."""
    ranges = []
    for index in range(int(round(2 * math.pi / step))):
        angle = -math.pi + index * step
        dx, dy = math.cos(angle), math.sin(angle)
        best = math.inf
        for (ax, ay), (bx, by) in walls:
            ex, ey = bx - ax, by - ay
            denom = dx * ey - dy * ex
            if abs(denom) < 1e-12:
                continue
            t = ((ax - x) * ey - (ay - y) * ex) / denom
            u = ((ax - x) * dy - (ay - y) * dx) / denom
            if t > 0 and 0 <= u <= 1:
                best = min(best, t)
        ranges.append(best)
    scan = SimpleNamespace(ranges=ranges, angle_min=-math.pi, angle_increment=step, range_min=0.05, range_max=8.0)
    node = SimpleNamespace(scan=scan, scan_at=vd.time.monotonic(), scan_yaw=0.0)
    node._beams = lambda *a, **k: vd.VisionDriveNode._beams(node, *a, **k)
    return node


# 폭 0.8 m 복도가 +x로 오다가 왼쪽(+y)으로 꺾인다. 정면 벽 x=2.0
LEFT_TURN = [((-3.0, -0.4), (2.0, -0.4)), ((2.0, -0.4), (2.0, 3.0)), ((-3.0, 0.4), (1.2, 0.4)),
             ((1.2, 0.4), (1.2, 3.0)), ((1.2, 3.0), (2.0, 3.0))]


@pytest.mark.parametrize('x, y', [(1.4, 0.0), (1.6, 0.0), (1.3, -0.15)])
def test_open_heading_points_into_the_next_corridor(x, y):
    node = _ray_scan(LEFT_TURN, x, y)
    left = vd.VisionDriveNode.open_heading(node, 1, 0.9)
    assert left is not None and 60.0 <= math.degrees(left) <= 100.0
    assert vd.VisionDriveNode.open_heading(node, -1, 0.9) is None      # 오른쪽은 벽


def test_open_heading_needs_a_fresh_scan():
    node = _ray_scan(LEFT_TURN, 1.4, 0.0)
    node.scan_at -= 1.0
    assert vd.VisionDriveNode.open_heading(node, 1, 0.9) is None


# ---- 코너: 복도 가운데쯤까지 직진한 뒤 회전 ----

def test_closed_corner_drives_up_to_pivot_point_before_turning(drive):
    # 실물 19:16: 벽이 0.45 m보다 멀 때 차선이 사라졌고, 그 자리에서 돌아 안쪽 선을 밟았다
    drive.node.open_heading = lambda side, reach: side * math.radians(85.0)
    drive(0.0, heading=0.0)
    linear, angular, reason = drive(None, clear=0.7, heading=0.0)        # 정면 0.7 m에서 차선 소실
    assert linear == pytest.approx(drive.args.max_linear) and angular == 0.0
    assert reason.startswith(vd.CORNER_APPROACH_REASON)
    assert drive(0.0, clear=0.6, heading=0.0)[1] == 0.0                 # 바닥이 넓게 보여도 차선은 안 본다
    slow, _, _ = drive(None, clear=0.46, heading=0.0)
    assert 0.0 < slow < drive.args.max_linear                           # 마지막 15 cm는 줄여서
    linear, angular, reason = drive(None, clear=0.40, heading=0.0)       # 0.40 m: 제자리 회전 시작
    assert linear == 0.0 and angular > 0.0 and reason.startswith(vd.CORNER_REASON)
    assert drive.node.corner['target'] == pytest.approx(math.radians(85.0))


def test_approach_stops_for_a_near_obstacle_or_after_time_cap(drive):
    drive(0.0)
    drive(None, clear=0.7)
    _, angular, reason = drive(None, clear=0.7, front=0.37)             # 넓은 장애물이 0.37 m: 바로 회전
    assert angular != 0.0 and reason.startswith(vd.CORNER_REASON)
    drive.node.corner = None
    drive(0.0, front=math.inf)
    drive(None, clear=0.7)
    _, angular, reason = drive(None, clear=0.7, dt=vd.CORNER_APPROACH_S + 0.1)   # 벽이 안 가까워져도 10초면 회전
    assert angular != 0.0 and reason.startswith(vd.CORNER_REASON)


def test_open_front_or_zero_pivot_distance_turns_at_once(drive):
    drive(0.0)
    assert drive(None)[2].startswith(vd.CORNER_REASON)                  # 정면이 트임(인식 끊김): 바로 다시 찾기
    drive.node.corner = None
    drive.args.corner_pivot_distance = 0.0
    drive(0.0)
    assert drive(None, clear=0.7)[2].startswith(vd.CORNER_REASON)       # 0이면 판단한 자리에서 회전
