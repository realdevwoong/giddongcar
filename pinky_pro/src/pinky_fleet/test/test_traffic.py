from pathlib import Path

import pytest

from pinky_fleet.traffic import (TrafficGate, distance_to_polygon, load_zones, map_mismatch, point_in_polygon)

PKG = Path(__file__).resolve().parents[1]
ZONES = PKG / 'params' / 'traffic_good3.yaml'
SQUARE = [[0, 0], [1, 0], [1, 1], [0, 1]]


# ───── 기하 ─────
def test_point_in_polygon():
    assert point_in_polygon(0.5, 0.5, SQUARE)
    assert not point_in_polygon(1.5, 0.5, SQUARE)
    assert not point_in_polygon(0.5, -0.1, SQUARE)


def test_distance_to_polygon():
    assert distance_to_polygon(0.5, 0.5, SQUARE) == 0.0                     # 안
    assert distance_to_polygon(1.3, 0.5, SQUARE) == pytest.approx(0.3)      # 오른쪽 변까지
    assert distance_to_polygon(2.0, 2.0, SQUARE) == pytest.approx(2 ** 0.5)  # 모서리까지


def test_bad_zone_file(tmp_path):
    bad = tmp_path / 'bad.yaml'
    bad.write_text('cells: {a: {polygon: [[0, 0], [1, 0], [1, 1]]}}\n'
                   'links: [{between: [a, nowhere]}]\n')
    with pytest.raises(ValueError):
        load_zones(bad)


def test_map_check():
    config = load_zones(ZONES)
    good = dict(width=84, height=56, resolution=0.05, origin=dict(x=-1.067, y=-0.171, yaw=0.0))
    assert map_mismatch(config, good) is None
    assert map_mismatch(config, dict(good, width=90))
    assert map_mismatch(config, dict(good, origin=dict(x=-1.75, y=-0.65, yaw=0.0)))


# ───── 칸 지도가 실제 지도(good3.pgm)와 맞는지 ─────
def free_cells():
    """good3.pgm의 빈 칸 좌표 (x, y) 목록과 격자 연결용 표. PIL 없이 P5 PGM을 직접 읽는다."""
    raw = (PKG / 'maps' / 'good3.pgm').read_bytes()
    fields, pos = [], 0
    while len(fields) < 4:                      # P5, 너비, 높이, 최댓값 (주석 줄은 건너뛴다)
        end = raw.index(b'\n', pos)
        line = raw[pos:end].split(b'#')[0]
        fields += line.split()
        pos = end + 1
    width, height = int(fields[1]), int(fields[2])
    pixels = raw[pos:pos + width * height]
    res, ox, oy = 0.05, -1.067, -0.171          # maps/good3.yaml
    free = {}
    for row in range(height):
        for col in range(width):
            if (255 - pixels[row * width + col]) / 255 < 0.196:   # free_thresh
                free[(row, col)] = (ox + (col + 0.5) * res, oy + (height - row - 0.5) * res)
    return free


def test_every_free_cell_is_in_exactly_one_traffic_cell():
    gate = TrafficGate(load_zones(ZONES))
    for x, y in free_cells().values():
        inside = [name for name, cell in gate.cells.items() if point_in_polygon(x, y, cell['polygon'])]
        assert len(inside) == 1, (x, y, inside)


def test_cells_touch_only_through_the_declared_links():
    # 통로를 막으면 via 없는 링크(①–②)만 남고, 통로가 있으면 선언한 링크가 모두 이어진다
    config = load_zones(ZONES)
    gate = TrafficGate(config)
    free = free_cells()

    def touching(blocked):
        pairs = set()
        for (row, col), xy in free.items():
            if blocked(xy):
                continue
            for n in ((row + 1, col), (row, col + 1)):
                if n in free and not blocked(free[n]):
                    a, b = gate.cell_of(*xy), gate.cell_of(*free[n])
                    if a != b:
                        pairs.add(frozenset((a, b)))
        return pairs

    in_passage = lambda xy: any(point_in_polygon(*xy, p['polygon']) for p in config['passages'].values())
    links = {frozenset(link['between']): link.get('via') for link in config['links']}
    assert touching(in_passage) == {pair for pair, via in links.items() if via is None}
    assert touching(lambda xy: False) == set(links)


# ───── 칸 열쇠 ─────
LEFT_TOP, LEFT_BOTTOM = dict(x=0.30, y=1.00, yaw=3.14), dict(x=0.50, y=0.30, yaw=0.0)
RIGHT_FRONT, RIGHT_BACK = dict(x=2.10, y=1.05, yaw=0.0), dict(x=2.75, y=1.00, yaw=0.0)


def gate():
    return TrafficGate(load_zones(ZONES))


def state(pose, active=False, target=None):
    return dict(pose=pose, active=active, target=target)


def parked(r1=(0.5, 0.5), r2=(2.0, 0.5)):
    """두 로봇이 생성 위치(①② 쪽과 ③)에 서 있다."""
    return {'robot1': state(r1), 'robot2': state(r2)}


def test_route_goes_through_the_cell_graph():
    g = gate()
    assert g.route((0.5, 0.3), (2.1, 1.05)) == (['left_bottom', 'left_top', 'right_front'], ['door'])
    assert g.route((2.75, 1.0), (0.5, 0.3)) == (['right_back', 'right_front', 'left_top', 'left_bottom'],
                                                 ['connector', 'door'])
    assert g.route((0.5, 0.3), (0.6, 0.35)) == (['left_bottom'], [])
    assert g.route((5.0, 5.0), (0.5, 0.3)) == (None, None)      # 칸 밖


def test_head_on_through_the_door_second_robot_waits_then_goes():
    g = gate()
    robots = parked()
    # robot1 ②→③ 받음: ②①③. robot2 ③→① 은 ③·①이 막혀 기다린다
    assert g.request('robot1', (0.5, 0.5), RIGHT_FRONT, 0, robots) == ('go', ['left_bottom', 'left_top', 'right_front'])
    assert g.request('robot2', (2.0, 0.5), LEFT_TOP, 1, robots) == ('hold', ('robot1', '③ 오른쪽 앞칸 사용 중'))
    moving = lambda pose: {'robot1': state(pose, True, (2.10, 1.05)), 'robot2': state((2.0, 0.5))}
    assert g.update(moving((1.20, 1.00)), 5) == ([], [])          # robot1 ①: ② 반납, ①③ 쥠
    assert g.owner['left_bottom'] is None and g.owner['left_top'] == 'robot1'
    assert g.update(moving((1.70, 1.05)), 8) == ([], [])          # 문 통과 중: ①에서 0.15 m 안이라 아직 쥠
    assert g.owner['left_top'] == 'robot1'
    arrived = {'robot1': state((2.10, 1.05)), 'robot2': state((2.0, 0.5))}
    ready, dropped = g.update(arrived, 12)                        # 도착: 전부 반납 → robot2 출발
    assert ready == [('robot2', LEFT_TOP)] and dropped == []
    assert g.owner == dict(left_top='robot2', left_bottom=None, right_front='robot2', right_back=None)


def test_disjoint_routes_move_at_the_same_time():
    g = gate()
    robots = parked()
    assert g.request('robot1', (0.5, 0.5), LEFT_BOTTOM, 0, robots)[0] == 'go'    # ② 안
    assert g.request('robot2', (2.0, 0.5), RIGHT_BACK, 1, robots)[0] == 'go'     # ③→④


def test_passed_cells_are_released_while_moving():
    g = gate()
    g.request('robot1', (0.5, 0.5), RIGHT_BACK, 0, parked())    # ②①③④
    g.request('robot2', (2.0, 0.5), dict(x=0.30, y=0.20, yaw=0.0), 1, parked())   # ③①② 필요 → 기다림
    # robot1이 ④에 들어가 ③에서 충분히 벗어나면 ②①③을 반납 → robot2 출발
    ready, _ = g.update({'robot1': state((2.80, 1.00), True, (2.75, 1.00)), 'robot2': state((2.0, 0.5))}, 9)
    assert ready == [('robot2', dict(x=0.30, y=0.20, yaw=0.0))]
    assert g.owner['right_back'] == 'robot1'


def test_parked_robots_hold_no_keys_but_their_spot_and_goal_are_protected():
    g = gate()
    robots = parked(r2=(0.27, 0.85))                           # 2026-09-28: robot2가 (0.27, 0.85)에 섰다
    code, reason = g.check_target('robot1', dict(x=0.39, y=0.64, yaw=0.2), robots)   # 0.24 m → 그때 실패한 목적지
    assert code == 'near_robot' and '서 있는 자리' in reason
    moving = {'robot1': state((0.5, 0.5)), 'robot2': state((2.0, 0.5), True, (0.24, 0.85))}
    code, reason = g.check_target('robot1', dict(x=0.39, y=0.64, yaw=0.2), moving)   # 목적지끼리 0.26 m
    assert code == 'near_robot' and '목적지' in reason
    assert g.check_target('robot1', LEFT_BOTTOM, robots) is None


def test_goal_in_or_next_to_a_passage_or_off_the_cells_is_rejected():
    g = gate()
    for target, code in ((dict(x=1.60, y=1.05, yaw=0), 'near_zone'),   # 문 안
                         (dict(x=2.00, y=0.95, yaw=0), 'near_zone'),   # 문 옆
                         (dict(x=2.30, y=0.40, yaw=0), 'near_zone'),   # 칸막이 밑 통로
                         (dict(x=5.00, y=5.00, yaw=0), 'off_map')):
        assert g.check_target('robot1', target, parked())[0] == code


def test_robot_standing_in_a_passage_blocks_routes_through_it():
    g = gate()
    robots = parked(r2=(1.60, 1.05))                            # robot2가 문 안에 멈춰 있다(열쇠는 없음)
    assert g.request('robot1', (0.5, 0.5), RIGHT_FRONT, 0, robots) == ('hold', ('robot2', '문에 서 있음'))


def test_unknown_position_keeps_the_keys():
    g = gate()
    g.request('robot1', (0.5, 0.5), RIGHT_FRONT, 0, parked())
    g.update({'robot1': state(None), 'robot2': state((2.0, 0.5))}, 30)
    assert g.owner['right_front'] == 'robot1'


def test_keys_are_kept_while_the_goal_is_being_sent():
    # 열쇠를 받고 Nav2가 목표를 알려 오기 전(active 아님)이라도 start_grace 안에는 반납하지 않는다
    g = gate()
    g.request('robot1', (0.5, 0.5), RIGHT_FRONT, 0, parked())
    g.update(parked(), 2)
    assert g.owner['right_front'] == 'robot1'
    g.update(parked(), 6)                                       # 5초가 지나도 목표가 없으면 반납
    assert g.owner['right_front'] is None


def test_waiting_goal_is_dropped_if_someone_parks_next_to_it():
    g = gate()
    g.request('robot1', (0.5, 0.5), RIGHT_FRONT, 0, parked())
    g.request('robot2', (2.0, 0.5), LEFT_TOP, 1, parked())
    # robot1이 목적지를 바꿔 robot2가 기다리던 목적지 옆에 섰다
    ready, dropped = g.update({'robot1': state((0.40, 1.05)), 'robot2': state((2.0, 0.5))}, 20)
    assert ready == [] and dropped[0][0] == 'robot2' and g.pending == []


def test_cancel_and_new_goal_replace_the_waiting_goal():
    g = gate()
    g.request('robot1', (0.5, 0.5), RIGHT_FRONT, 0, parked())
    g.request('robot2', (2.0, 0.5), LEFT_TOP, 1, parked())
    g.cancel('robot2')
    assert g.pending == []
    g.request('robot2', (2.0, 0.5), LEFT_TOP, 2, parked())
    g.request('robot2', (2.0, 0.5), dict(x=0.40, y=0.30, yaw=0.0), 3, parked())   # 새 목표가 이전 보류를 바꾼다
    assert [p['target']['x'] for p in g.pending] == [0.40]


def test_waiting_too_long_drops_the_goal():
    g = gate()
    g.request('robot1', (0.5, 0.5), RIGHT_FRONT, 0, parked())
    g.request('robot2', (2.0, 0.5), LEFT_TOP, 1, parked())
    moving = {'robot1': state((1.2, 1.0), True, (2.10, 1.05)), 'robot2': state((2.0, 0.5))}
    assert g.update(moving, 100) == ([], [])                      # 아직 기다린다
    ready, dropped = g.update(moving, 122)                        # 120초 넘음: 버린다
    assert ready == [] and dropped[0][0] == 'robot2' and g.pending == []
