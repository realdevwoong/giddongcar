"""달리면서 양보(YieldRule): robot1(앞 순위)은 그대로, robot2(뒤 순위)가 go / stop / back."""
from pathlib import Path

import pytest

from pinky_fleet.traffic import YieldRule, load_zones, path_ahead, polyline_distance, segment_distance

ZONES = Path(__file__).resolve().parents[1] / 'params' / 'traffic_good3.yaml'


def line(x0, y0, x1, y1, n=20):
    """(x0, y0)에서 (x1, y1)까지 Nav2 /plan처럼 촘촘한 점 목록."""
    return [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n + 1)]


def moving(pose, path):
    return dict(pose=pose, active=True, path=path)


def rule():
    return YieldRule(dict(yield_distance=0.30, yield_release=0.10, yield_lookahead=1.0))


# ───── 기하 ─────
def test_segment_distance():
    assert segment_distance((0, 0), (2, 0), (1, -1), (1, 1)) == 0.0                 # 교차
    assert segment_distance((0, 0), (1, 0), (0, 0.5), (1, 0.5)) == pytest.approx(0.5)  # 나란히
    assert segment_distance((0, 0), (1, 0), (2, 0), (3, 0)) == pytest.approx(1.0)      # 한 줄 위, 떨어짐


def test_polyline_distance_handles_single_points_and_empty():
    assert polyline_distance([(0, 0)], line(1, -1, 1, 1)) == pytest.approx(1.0)
    assert polyline_distance([], line(0, 0, 1, 0)) == float('inf')


def test_path_ahead_cuts_at_length():
    ahead = path_ahead((0, 0), line(0, 0, 3, 0), 1.0)
    assert ahead[-1] == pytest.approx((1.0, 0.0))
    assert path_ahead((0, 0), [], 1.0) == [(0, 0)]                                # 경로가 아직 없으면 자기 자리만


# ───── 판단 ─────
def test_go_when_robot1_is_not_moving():
    robot1 = dict(pose=(1.0, 0.0), active=False, path=[])
    robot2 = moving((0.0, 0.0), line(0, 0, 2, 0))                                 # robot1 자리를 지나가도
    assert rule().decide(robot1, robot2)[0] == 'go'                               # 서 있는 로봇은 Nav2가 피한다


def test_go_when_paths_are_far_apart():
    robot1 = moving((0, 0), line(0, 0, 2, 0))
    robot2 = moving((0, 1), line(0, 1, 2, 1))                                     # 1 m 떨어진 나란한 길
    assert rule().decide(robot1, robot2)[0] == 'go'


def test_stop_when_paths_cross_just_ahead():
    robot1 = moving((1.0, -1.0), line(1.0, -1.0, 1.0, 1.0))                       # 세로로 지나간다
    robot2 = moving((0.5, 0.0), line(0.5, 0.0, 2.0, 0.0))                         # 0.5 m 앞에서 가로지른다
    assert rule().decide(robot1, robot2)[0] == 'stop'


def test_go_when_crossing_is_beyond_lookahead():
    robot1 = moving((3.0, -1.0), line(3.0, -1.0, 3.0, 1.0))
    robot2 = moving((0.0, 0.0), line(0.0, 0.0, 4.0, 0.0))                         # 겹치는 곳이 3 m 앞
    assert rule().decide(robot1, robot2)[0] == 'go'                               # 가까워지면 그때 멈춘다


def test_go_when_robot1_is_still_far_away():
    r = YieldRule(dict(yield_distance=0.25, yield_lookahead=0.5, yield_leader_ahead=0.8))
    robot1 = moving((-3.0, 0.0), line(-3.0, 0.0, 3.0, 0.0))                       # 같은 길이지만 3 m 뒤
    robot2 = moving((0.0, 0.5), line(0.0, 0.5, 0.0, -1.0))                        # 그 길을 가로지른다
    assert r.decide(robot1, robot2)[0] == 'go'                                    # 멀리서부터 비키지 않는다


def test_back_when_robot2_stands_on_robot1_path():
    robot1 = moving((0.0, 0.0), line(0.0, 0.0, 2.0, 0.0))
    robot2 = moving((1.0, 0.1), line(1.0, 0.1, 1.0, 1.0))                         # robot1 길 한가운데
    assert rule().decide(robot1, robot2)[0] == 'back'


def test_go_again_after_robot1_has_passed():
    # robot1이 교차점을 지나 남은 경로가 robot2 앞길에서 멀어졌다
    robot1 = moving((1.0, 0.8), line(1.0, 0.8, 1.0, 1.5))
    robot2 = moving((0.5, 0.0), line(0.5, 0.0, 2.0, 0.0))
    assert rule().decide(robot1, robot2, prev='stop')[0] == 'go'


def test_release_needs_extra_gap():
    robot1 = moving((0.0, 0.35), line(0.0, 0.35, 2.0, 0.35))                      # 0.35 m: distance(0.30)보다 멀다
    robot2 = moving((0.0, 0.0), line(0.0, 0.0, 2.0, 0.0))
    assert rule().decide(robot1, robot2, prev='go')[0] == 'go'                    # 달리던 중이면 계속 간다
    assert rule().decide(robot1, robot2, prev='stop')[0] == 'stop'                # 멈춘 중이면 0.40 m까지 더 기다린다


def test_unknown_pose_does_nothing():
    robot1 = moving((0, 0), line(0, 0, 2, 0))
    assert rule().decide(robot1, dict(pose=None, active=True, path=[]))[0] == 'go'


def test_good3_door_head_on():
    # good3 문(x≈1.585, y 0.88~1.23)에서 정면으로 만난다: robot1 왼→오, robot2 오→왼, robot2가 문 바로 앞
    r = YieldRule(load_zones(ZONES))
    robot1 = moving((1.0, 1.05), line(1.0, 1.05, 2.1, 1.05))
    robot2 = moving((2.3, 1.05), line(2.3, 1.05, 0.4, 1.05))
    assert r.decide(robot1, robot2)[0] == 'stop'                                  # 1.3 m 앞: 물러나진 않고 멈춘다
    robot2_in_door = moving((1.7, 1.05), line(1.7, 1.05, 0.4, 1.05))
    assert r.decide(robot1, robot2_in_door)[0] == 'back'                          # 0.7 m 앞 robot1 길 위: 물러난다
    robot2_side = moving((2.0, 0.4), line(2.0, 0.4, 1.9, 1.05) + line(1.9, 1.05, 0.4, 1.05))
    assert r.decide(robot1, robot2_side)[0] == 'stop'                             # 옆에서 문으로 오던 중: 멈춰 기다린다

