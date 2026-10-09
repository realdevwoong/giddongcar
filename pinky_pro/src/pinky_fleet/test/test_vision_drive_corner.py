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
    state = SimpleNamespace(now=100.0, error=None, wall=False, sides=None)
    monkeypatch.setattr(vd, 'time', SimpleNamespace(monotonic=lambda: state.now))
    monkeypatch.setattr(vd, '_lane_error', lambda mask: state.error)
    args = vd.parse_args(['--robot-ip', '192.0.2.1', '--model', 'm.pt', '--mode', 'drive',
                          '--max-linear', '0.05', '--max-angular', '0.25'])   # SWEEP 계산과 맞춤
    node = SimpleNamespace(front_range=lambda min_width: math.inf,
                           wall_ahead=lambda distance: state.wall,      # 정면 벽(코너) 여부
                           side_clearance=lambda: state.sides)          # (왼쪽, 오른쪽) 여유 거리

    def step(error, dt=0.1, wall=None, sides=None):
        state.now += dt
        state.error = error
        if wall is not None:
            state.wall = wall
        if sides is not None:
            state.sides = sides
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
    assert drive(0.3)[2].startswith(vd.CORNER_REASON)   # 0.25보다 밖이면 아직 회전(안쪽 테이프 침범 방지)
    linear, _, reason = drive(0.2)                # 앞쪽에 들어옴 → 정상 주행
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
    linear, angular, reason = drive(0.0, wall=True, sides=(1.6, 0.3))
    assert linear == 0.0 and angular == pytest.approx(0.25)
    assert reason.startswith(vd.CORNER_REASON) and '왼쪽' in reason


def test_lidar_tie_falls_back_to_camera_lean(drive):
    for _ in range(5):
        drive(0.12)
    _, angular, reason = drive(0.12, wall=True, sides=(0.8, 0.8))
    assert angular == pytest.approx(-0.25) and '오른쪽' in reason


def test_pivot_holds_until_wall_is_gone(drive):
    drive(0.0, wall=True, sides=(1.6, 0.3))          # 왼쪽 회전 시작
    _, angular, reason = drive(0.0)                  # 가운데 보이는 바닥 조각, 벽은 아직 정면
    assert angular == pytest.approx(0.25) and reason.startswith(vd.CORNER_REASON)
    linear, _, reason = drive(0.0, wall=False)       # 새 복도 정렬 → 전진
    assert linear > 0.0 and not reason.startswith(vd.CORNER_REASON)
    assert drive.node.corner is None


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
