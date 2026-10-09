"""vision_drive 전방 라이다 판단: 앞이 비어 있으면 달리고, scan이 끊기면 멈춘다."""
import math
import time
from types import SimpleNamespace

import pytest

from pinky_fleet.vision_drive import VisionDriveNode

STEP = math.radians(1.0)


def _node(ranges, angle_min=-math.pi, age=0.0):
    scan = SimpleNamespace(ranges=ranges, angle_min=angle_min, angle_increment=STEP,
                           range_min=0.05, range_max=8.0)
    return SimpleNamespace(scan=scan, scan_at=time.monotonic() - age)


def _front(node):
    return VisionDriveNode.front_range(node, min_width=0.12)


def _ranges_with(indices, distance, count=360):
    ranges = [math.inf] * count
    for i in indices:
        ranges[i % count] = distance
    return ranges


def test_no_scan_or_stale_scan_is_none():
    assert _front(SimpleNamespace(scan=None, scan_at=None)) is None
    assert _front(_node([math.inf] * 360, age=1.0)) is None


def test_clear_front_is_infinite_not_none():
    assert _front(_node([math.inf] * 360)) == math.inf


def test_distant_sparse_returns_are_clear():
    # 5 m 앞 점들은 1도 간격이 8.7 cm라 서로 이어지지 않는다.
    assert _front(_node([5.0] * 360)) == math.inf


def test_wide_obstacle_ahead_is_measured():
    # angle_min=-pi이면 정면은 180번. 0.3 m에서 ±12도 = 폭 약 0.12 m 이상
    assert _front(_node(_ranges_with(range(168, 193), 0.30))) == pytest.approx(0.30)


def test_single_speck_is_ignored():
    assert _front(_node(_ranges_with([180], 0.20))) == math.inf


def test_obstacle_split_across_scan_start_is_joined():
    # angle_min=0이면 정면 물체(±12도)가 배열 끝(348~359)과 처음(0~12)으로 나뉜다.
    ranges = _ranges_with(list(range(-12, 13)), 0.30)
    assert _front(_node(ranges, angle_min=0.0)) == pytest.approx(0.30)
