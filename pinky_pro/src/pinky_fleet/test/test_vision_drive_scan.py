"""vision_drive 전방 라이다 판단: 앞이 비어 있으면 달리고, scan이 끊기면 멈춘다."""
import math
import time
from types import SimpleNamespace

import pytest
from tf2_ros import TransformException

from pinky_fleet.vision_drive import VisionDriveNode

STEP = math.radians(1.0)
PINKY = dict(angle_min=-math.pi, scan_yaw=math.pi)   # sllidar -pi..pi, URDF rplidar_link yaw pi


def _node(ranges, angle_min=-math.pi, age=0.0, scan_yaw=0.0):
    scan = SimpleNamespace(ranges=ranges, angle_min=angle_min, angle_increment=STEP,
                           range_min=0.05, range_max=8.0)
    return SimpleNamespace(scan=scan, scan_at=time.monotonic() - age, scan_yaw=scan_yaw)


def _front(node):
    return VisionDriveNode.front_range(node, min_width=0.12)


def _ranges_with(indices, distance, count=360):
    ranges = [math.inf] * count
    for i in indices:
        ranges[i % count] = distance
    return ranges


def test_no_scan_or_stale_scan_is_none():
    assert _front(SimpleNamespace(scan=None, scan_at=None, scan_yaw=0.0)) is None
    assert _front(_node([math.inf] * 360, age=1.0)) is None


def test_unknown_lidar_orientation_is_none():
    assert _front(_node([math.inf] * 360, scan_yaw=None)) is None


def test_clear_front_is_infinite_not_none():
    assert _front(_node([math.inf] * 360)) == math.inf


def test_distant_sparse_returns_are_clear():
    # 5 m 앞 점들은 1도 간격이 8.7 cm라 서로 이어지지 않는다.
    assert _front(_node([5.0] * 360)) == math.inf


def test_wide_obstacle_ahead_is_measured():
    # angle_min=-pi, yaw 0이면 정면은 180번. 0.3 m에서 ±12도 = 폭 약 0.12 m 이상
    assert _front(_node(_ranges_with(range(168, 193), 0.30))) == pytest.approx(0.30)


def test_single_speck_is_ignored():
    assert _front(_node(_ranges_with([180], 0.20))) == math.inf


def test_obstacle_split_across_scan_start_is_joined():
    # angle_min=0이면 정면 물체(±12도)가 배열 끝(348~359)과 처음(0~12)으로 나뉜다.
    ranges = _ranges_with(list(range(-12, 13)), 0.30)
    assert _front(_node(ranges, angle_min=0.0)) == pytest.approx(0.30)


def test_pinky_ignores_person_behind_robot():
    # 실물 증상: 로봇 뒤 0.22 m(scan 각도 0 = 180번, ±20도 = 폭 0.15 m)에 사람이 있으면
    # "전방 장애물 0.22 m"로 멈췄다.
    assert _front(_node(_ranges_with(range(160, 201), 0.22), **PINKY)) == math.inf


def test_pinky_sees_obstacle_in_front():
    # 로봇 정면은 scan 각도 ±pi = 배열 처음(0~12)과 끝(348~359)
    assert _front(_node(_ranges_with(range(-12, 13), 0.30), **PINKY)) == pytest.approx(0.30)


class _Buffer:
    def __init__(self, rotation=None):
        self.rotation = rotation

    def lookup_transform(self, target, source, when):
        if self.rotation is None:
            raise TransformException('no tf yet')
        assert (target, source) == ('base_link', 'rplidar_link')
        return SimpleNamespace(transform=SimpleNamespace(rotation=self.rotation))


def _scan_message():
    return SimpleNamespace(header=SimpleNamespace(frame_id='rplidar_link'))


def test_scan_yaw_read_from_tf_once_available():
    node = SimpleNamespace(scan=None, scan_at=None, scan_yaw=None, tf_buffer=_Buffer())
    VisionDriveNode._on_scan(node, _scan_message())
    assert node.scan is not None and node.scan_yaw is None   # TF가 아직 없으면 방향 모름
    node.tf_buffer = _Buffer(SimpleNamespace(x=0.0, y=0.0, z=1.0, w=0.0))   # z축 180도
    VisionDriveNode._on_scan(node, _scan_message())
    assert abs(node.scan_yaw) == pytest.approx(math.pi)
