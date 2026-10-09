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
    state = SimpleNamespace(now=100.0, error=None)
    monkeypatch.setattr(vd, 'time', SimpleNamespace(monotonic=lambda: state.now))
    monkeypatch.setattr(vd, '_lane_error', lambda mask: state.error)
    args = vd.parse_args(['--robot-ip', '192.0.2.1', '--model', 'm.pt', '--mode', 'drive'])
    node = SimpleNamespace(front_range=lambda min_width: math.inf)

    def step(error, dt=0.1):
        state.now += dt
        state.error = error
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
    linear, _, reason = drive(0.3)                # 앞쪽에 들어옴 → 정상 주행
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
