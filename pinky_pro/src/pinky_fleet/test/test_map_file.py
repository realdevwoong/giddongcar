"""지도 YAML을 ROS 없이 읽기: map_server와 같은 칸 값(100 벽, 0 바닥, -1 미탐색)."""
import threading
from pathlib import Path

import pytest

from pinky_fleet.fleet import Fleet
from pinky_fleet.map_file import _read_pgm, load_map

GOOD3 = Path(__file__).resolve().parents[1] / 'maps' / 'good3.yaml'


def test_good3_matches_map_server_trinary():
    map_id, grid = load_map(GOOD3)
    assert (grid['width'], grid['height'], grid['resolution']) == (84, 56, 0.05)
    assert grid['origin'] == dict(x=-1.067, y=-0.171, yaw=0.0)
    counts = {value: grid['data'].count(value) for value in (100, 0, -1)}
    assert counts == {100: 358, 0: 1501, -1: 2845}         # 픽셀 0 / 254 / 205와 같은 수
    assert len(map_id) == 64 and load_map(GOOD3)[0] == map_id


def test_image_top_row_is_the_far_y_edge(tmp_path):
    # 3x2 이미지: 위 줄 [벽 바닥 미탐색], 아래 줄 [바닥 바닥 벽] → data는 아래 줄부터
    (tmp_path / 'm.pgm').write_bytes(b'P5\n# comment\n3 2\n255\n' + bytes([0, 254, 205, 254, 254, 0]))
    (tmp_path / 'm.yaml').write_text('image: m.pgm\nresolution: 0.1\norigin: [1.0, 2.0, 0.5]\n'
                                     'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    _, grid = load_map(tmp_path / 'm.yaml')
    assert grid['data'] == [0, 0, 100, 100, 0, -1]
    assert grid['origin'] == dict(x=1.0, y=2.0, yaw=0.5)


def test_ascii_pgm_and_bad_header():
    assert _read_pgm(b'P2\n2 1\n255\n0 254\n') == (2, 1, 255, [0, 254])
    with pytest.raises(ValueError):
        _read_pgm(b'P6\n1 1\n255\n\x00\x00\x00')


class _Robot:
    def __init__(self, map_data=None):
        self.lock = threading.RLock()
        self.map_data, self.map_id = map_data, 'robot-map' if map_data else None


def test_fleet_shows_file_map_until_map_server_sends_one():
    file_map = load_map(GOOD3)
    one, two = _Robot(), _Robot()
    fleet = Fleet(dict(robot1=one, robot2=two), file_map=file_map)
    assert fleet.map() == file_map and fleet.map_source() == 'file'
    two.map_data, two.map_id = {'width': 1}, 'robot-map'
    assert fleet.map() == ('robot-map', {'width': 1}) and fleet.map_source() == 'robot'
    assert Fleet(dict(robot1=_Robot())).map() == (None, None)
