"""교통 정리 판단. ROS 없는 순수 파이썬이라 DDS 없이 pytest로 시험한다.

칸 열쇠 규칙 (TrafficGate)
- 지도를 칸(cell)으로 나누고 칸마다 열쇠가 하나다. 칸 사이 좁은 곳은 통로(passage)다.
- 움직이려는 로봇은 출발 칸부터 도착 칸까지 지나갈 칸의 열쇠를 **모두 한꺼번에** 받아야 출발한다.
  하나라도 다른 로봇이 쥐고 있으면 줄을 선다(먼저 온 순서). 반만 쥐고 기다리는 일이 없어서 교착이 생기지 않는다.
- 지나온 칸은 벗어나는 대로 반납하고, 도착·실패·취소하면 전부 반납한다. 멈춘 로봇은 열쇠가 없다(Nav2가 장애물로 피한다).
- 대신 통로 근처, 멈춘 로봇 자리, 다른 로봇의 목적지 근처는 목적지로 고를 수 없다.
- 위치를 모르는 로봇의 열쇠는 반납하지 않는다(모를 때는 잠가 두는 쪽이 안전하다).
보내는 일은 fleet_dashboard가 한다. 설명은 docs/learn/traffic.md.
"""
import math
from collections import deque
from pathlib import Path

import yaml


def edges(polygon):
    """다각형의 변 (앞 점, 뒤 점). 마지막 점과 첫 점도 잇는다."""
    return zip(polygon, polygon[1:] + polygon[:1])


def point_in_polygon(x, y, polygon):
    """점 (x, y)가 다각형 안이면 True. 오른쪽으로 쏜 선이 변을 홀수 번 넘으면 안이다(광선 투사)."""
    inside = False
    for (x1, y1), (x2, y2) in edges(polygon):
        if (y1 > y) != (y2 > y):                               # 이 변이 점의 높이를 가로지른다
            cross_x = x1 + (y - y1) * (x2 - x1) / (y2 - y1)    # 그 높이에서 변의 x
            if x < cross_x:
                inside = not inside
    return inside


def distance_to_segment(x, y, a, b):
    """점 (x, y)에서 선분 a-b까지 가장 가까운 거리 [m]."""
    (ax, ay), (bx, by) = a, b
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    # 점을 선분 위에 수직으로 내린 위치 t (0=a, 1=b). 선분 밖이면 끝점으로 자른다
    t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / length2))
    return math.hypot(x - (ax + t * dx), y - (ay + t * dy))


def distance_to_polygon(x, y, polygon):
    """다각형까지 거리 [m]. 안에 있으면 0.0."""
    if point_in_polygon(x, y, polygon):
        return 0.0
    return min(distance_to_segment(x, y, a, b) for a, b in edges(polygon))


def load_zones(path):
    """교통 정리 YAML(cells, links, passages)을 읽고 검사한다. 틀리면 ValueError."""
    config = yaml.safe_load(Path(path).read_text())
    cells, passages = config.get('cells') or {}, config.get('passages') or {}
    for name, shape in {**cells, **passages}.items():
        if len(shape['polygon']) < 3:
            raise ValueError(f'{name}: polygon은 점이 3개 이상이어야 합니다')
    for link in config.get('links') or []:
        unknown = set(link['between']) - set(cells)
        if unknown or len(link['between']) != 2:
            raise ValueError(f"링크 {link['between']}: 모르는 칸 {sorted(unknown)}")
        if link.get('via') and link['via'] not in passages:
            raise ValueError(f"링크 {link['between']}: 모르는 통로 {link['via']}")
    return config


def map_mismatch(config, map_info):
    """구역 파일의 map_check와 받은 지도가 다르면 이유 글자, 같으면 None. map_check가 없으면 검사하지 않는다."""
    check = config.get('map_check')
    if not check:
        return None
    got = (map_info['width'], map_info['height'], map_info['resolution'],
           map_info['origin']['x'], map_info['origin']['y'])
    want = (check['width'], check['height'], check['resolution'], *check['origin'][:2])
    if got[:2] != want[:2] or any(abs(g - w) > 1e-3 for g, w in zip(got[2:], want[2:])):
        return f'구역 파일이 지도와 달라요 (지도 {got[0]}x{got[1]}, 구역 파일 {want[0]}x{want[1]})'
    return None


class TrafficGate:
    """칸 열쇠. robots는 {id: {'pose': (x, y) 또는 None, 'active': 목표 진행 중인가, 'target': (x, y) 또는 None}}.
    target(목적지)은 {'x', 'y', 'yaw'} dict."""

    def __init__(self, config):
        self.config = config
        self.cells = config['cells']
        self.passages = config.get('passages') or {}
        self.links = {}   # 칸 → [(이웃 칸, 통로 또는 None)]
        for link in config.get('links') or []:
            a, b = link['between']
            self.links.setdefault(a, []).append((b, link.get('via')))
            self.links.setdefault(b, []).append((a, link.get('via')))
        self.occupy_margin = config.get('occupy_margin', 0.10)
        self.release_margin = config.get('release_margin', 0.15)
        self.goal_clearance = config.get('goal_clearance', 0.20)
        self.robot_clearance = config.get('robot_clearance', 0.35)
        # 열쇠를 받고 Nav2가 목표를 받았다고 알려 오기까지(최대 4초 요청 + 상태 전달) '출발 중'으로 치는 시간
        self.start_grace = config.get('start_grace', 5.0)
        self.pending_ttl = config.get('pending_ttl', 120.0)  # 이보다 오래 기다린 목표는 버린다(뜻밖의 늦은 출발 방지)
        self.owner = {cell: None for cell in self.cells}
        self.routes = {}    # 움직이는 로봇의 경로 {robot: [칸, ...]}
        self.granted = {}   # 로봇이 마지막으로 열쇠를 받은 시각
        self.pending = []   # 기다리는 목표 [{robot, target, since}], 먼저 온 순서

    # ───── 칸과 경로 ─────
    def cell_of(self, x, y):
        """점이 속한 칸 이름. 어느 칸에도 없으면 None."""
        for name, cell in self.cells.items():
            if point_in_polygon(x, y, cell['polygon']):
                return name
        return None

    def route(self, start, goal):
        """start (x, y)에서 goal (x, y)까지 지나갈 칸 목록과 통로 목록(칸 지도에서 BFS). 못 가면 (None, None)."""
        a, b = self.cell_of(*start), self.cell_of(*goal)
        if a is None or b is None:
            return None, None
        came = {a: (None, None)}   # 칸 → (이전 칸, 지나온 통로)
        queue = deque([a])
        while queue:
            cell = queue.popleft()
            if cell == b:
                break
            for nxt, via in self.links.get(cell, []):
                if nxt not in came:
                    came[nxt] = (cell, via)
                    queue.append(nxt)
        if b not in came:
            return None, None
        cells, vias, cell = [], [], b
        while cell is not None:
            cells.append(cell)
            prev, via = came[cell]
            if via:
                vias.append(via)
            cell = prev
        return cells[::-1], vias[::-1]

    # ───── 목적지 검사 ─────
    def check_target(self, robot, target, robots):
        """고를 수 없는 목적지면 (코드, 이유), 괜찮으면 None."""
        x, y = target['x'], target['y']
        if self.cell_of(x, y) is None:
            return 'off_map', '목적지가 교통 정리 칸 밖이에요'
        for passage in self.passages.values():
            if distance_to_polygon(x, y, passage['polygon']) < self.goal_clearance:
                return 'near_zone', (f"목적지가 {passage.get('name', '통로')}에 너무 가까워요 — "
                                     f'{self.goal_clearance:.2f} m 넘게 떨어진 곳을 고르세요')
        waiting = {p['robot']: p['target'] for p in self.pending}
        for other, s in robots.items():
            if other == robot:
                continue
            spots = []   # 다른 로봇이 앞으로 서 있을 곳: 멈춘 자리, 가고 있는 목적지, 기다리는 목적지
            if s['pose'] and not s['active']:
                spots.append((s['pose'], '서 있는 자리'))
            if s.get('target') and s['active']:
                spots.append((s['target'], '목적지'))
            if other in waiting:
                spots.append(((waiting[other]['x'], waiting[other]['y']), '목적지'))
            for (ox, oy), what in spots:
                if math.hypot(ox - x, oy - y) < self.robot_clearance:
                    return 'near_robot', (f'{other}의 {what}에 너무 가까워요 — '
                                          f'{self.robot_clearance:.2f} m 넘게 떨어진 곳을 고르세요')
        return None

    # ───── 열쇠 주고받기 ─────
    def blocker(self, robot, cells, vias, robots):
        """경로를 막는 다른 로봇과 이유. 없으면 None."""
        for cell in cells:
            if self.owner[cell] not in (None, robot):
                return self.owner[cell], f"{self.cells[cell].get('name', cell)} 사용 중"
        for via in vias:   # 통로 안에 서 있는 로봇(멈췄거나 실패)도 막는다
            for other, s in robots.items():
                if other != robot and s['pose'] \
                        and distance_to_polygon(*s['pose'], self.passages[via]['polygon']) <= self.occupy_margin:
                    return other, f"{self.passages[via].get('name', via)}에 서 있음"
        return None

    def take(self, robot, cells, now):
        """경로의 칸 열쇠를 모두 받고, 경로 밖에서 쥐고 있던 열쇠는 돌려놓는다."""
        for cell in self.cells:
            if cell in cells:
                self.owner[cell] = robot
            elif self.owner[cell] == robot:
                self.owner[cell] = None
        self.routes[robot] = list(cells)
        self.granted[robot] = now

    def release_all(self, robot):
        for cell, owner in self.owner.items():
            if owner == robot:
                self.owner[cell] = None
        self.routes.pop(robot, None)
        self.granted.pop(robot, None)

    def request(self, robot, pose, target, now, robots):
        """사람이 준 새 목표. ('go', 경로 칸들) 또는 ('hold', (막은 로봇, 이유)). 이전 보류는 새 목표로 바뀐다.
        갈 수 없는 칸이면 ValueError."""
        self.cancel(robot)
        cells, vias = self.route(pose, (target['x'], target['y']))
        if cells is None:
            raise ValueError('출발 칸에서 도착 칸으로 가는 길이 칸 지도에 없어요')
        blocked = self.blocker(robot, cells, vias, robots)
        if blocked:
            self.pending.append(dict(robot=robot, target=target, since=now))
            return 'hold', blocked
        self.take(robot, cells, now)
        return 'go', cells

    def cancel(self, robot):
        """보류 중인 목표를 지운다(사람이 취소했거나 새 목표를 줬을 때)."""
        self.pending = [p for p in self.pending if p['robot'] != robot]

    def update(self, robots, now):
        """주기적으로 부른다. (지금 보낼 보류 목표 [(robot, target)], 버린 보류 [(robot, 이유)])를 돌려준다."""
        for robot, s in robots.items():
            if s['pose'] is None:
                continue                          # 위치를 모르면 쥐고 있는다
            starting = now - self.granted.get(robot, -math.inf) < self.start_grace
            if not s['active'] and not starting:
                if not any(p['robot'] == robot for p in self.pending):
                    self.release_all(robot)       # 멈춘 로봇은 열쇠가 없다
                continue
            # 이동 중: 경로에서 지나온 칸을 벗어났으면 반납
            route = self.routes.get(robot, [])
            here = self.cell_of(*s['pose'])
            if here in route:
                for cell in route[:route.index(here)]:
                    if self.owner[cell] == robot and \
                            distance_to_polygon(*s['pose'], self.cells[cell]['polygon']) > self.release_margin:
                        self.owner[cell] = None
        ready, dropped = [], []
        for p in list(self.pending):              # 먼저 기다린 로봇부터
            robot, s = p['robot'], robots.get(p['robot'])
            if now - p['since'] > self.pending_ttl:
                self.pending.remove(p)
                dropped.append((robot, f'{self.pending_ttl:.0f}초 넘게 기다려서 버렸어요'))
                continue
            if not s or s['pose'] is None:
                continue                          # 위치를 알 때까지 기다린다
            others = {r: v for r, v in robots.items() if r != robot}
            problem = self.check_target(robot, p['target'], {robot: s, **others})
            if problem:                           # 기다리는 사이 목적지 근처에 다른 로봇이 섰다
                self.pending.remove(p)
                dropped.append((robot, problem[1]))
                continue
            cells, vias = self.route(s['pose'], (p['target']['x'], p['target']['y']))
            if cells is None:
                self.pending.remove(p)
                dropped.append((robot, '갈 수 있는 칸 경로가 없어요'))
                continue
            if not self.blocker(robot, cells, vias, robots):
                self.take(robot, cells, now)
                self.pending.remove(p)
                ready.append((robot, p['target']))
        return ready, dropped

    def snapshot(self, now):
        return dict(owners=dict(self.owner), routes={r: list(c) for r, c in self.routes.items()},
                    pending=[dict(robot=p['robot'], target=p['target'], wait=round(now - p['since'], 1))
                             for p in self.pending])
