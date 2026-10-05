"""Fleet-wide command routing, traffic handling, and status aggregation."""
import threading

from pinky_fleet.robot import Robot
from pinky_fleet.camera_control import PinkyCameraControl
from pinky_fleet.traffic import TrafficGate, YieldRule, map_mismatch
from pinky_fleet.fleet_common import (
    BACK_LIMIT, BACK_SPEED, BACK_STEP, FOLLOWER, LEADER, RETRY_CODES, RETRY_LIMIT,
    CommandError, pose_input, yield_view,
)

class Fleet:
    def __init__(self, robots, zones=None, cameras=None, camera_controls=None):
        self.robots = robots
        self.cameras = cameras or {}
        self.camera_controls = camera_controls or {}
        # 교통 정리. zones가 없으면 끈다. 지도가 구역 파일과 다르면 첫 확인 때 스스로 끈다.
        # 칸 정보(gate)는 목적지 검사에만 쓰고, 움직이는 중 판단은 양보 규칙(rule)이 한다
        self.gate = TrafficGate(zones) if zones else None
        self.rule = YieldRule(zones) if zones else None
        # 양보 상태와 FOLLOWER 명령을 한 번에 하나씩: 교통 스레드가 멈추는 사이 사람이 새 목표를 주는 경우 등
        self.traffic_lock = threading.RLock()
        self.traffic_off = None if zones else '구역 파일 없음'
        self.traffic_note = None               # 마지막 교통 정리 오류·알림
        self.traffic_stop = threading.Event()
        self.yielding = self.no_yield()
        self.retries = {}   # 로봇별 다시 보내기 {target, count, seen: 마지막으로 처리한 실패 목표 id}

    @staticmethod
    def no_yield():
        # action: go | stop | back, target: 멈추기 전 목적지(다시 보낼 것), backs: 연달아 물러난 횟수, backing: 후진 결과 future
        return dict(action='go', target=None, reason='', backs=0, backing=None)

    def map(self):
        for robot in self.robots.values():
            with robot.lock:
                if robot.map_data is not None:
                    return robot.map_id, robot.map_data
        return None, None

    def state(self):
        map_id, _ = self.map()
        states = [robot.snapshot() for robot in self.robots.values()]
        for state in states:
            camera = self.cameras.get(state['id'])
            state['camera'] = camera.status() if camera else dict(
                enabled=False, state='disabled', frames=0, age_s=None, error='', url=None)
            control = self.camera_controls.get(state['id'])
            state['camera']['control'] = control.status() if control else dict(
                configured=False, state='unconfigured', enabled=False, running=False, error='')
        if self.yielding['action'] != 'go' and self.yielding['target']:
            # Nav2 목표 취소는 양보를 위해 자동으로 한 것이므로 사용자 취소처럼 보이지 않게 한다.
            for state in states:
                if state['id'] == FOLLOWER:
                    state['nav'].update(state='yielding', label='양보 대기', active=False,
                                        target=self.yielding['target'])
                    state['status'] = '양보 대기'
        for state in states:
            state['map_matches'] = bool(map_id and state['map_id'] == map_id)
        return dict(map_id=map_id, robots=states, traffic=self.traffic_state())

    def traffic_state(self):
        if not self.gate:
            return dict(enabled=False, off=self.traffic_off)
        # 잠그지 않고 읽는다: 교통 스레드가 멈추기·물러나기 명령(최대 4초)을 하는 동안에도 화면 갱신이 막히지 않게
        y = self.yielding
        shown = dict(robot=FOLLOWER, leader=LEADER, action=y['action'], reason=y['reason'], target=y['target'])
        return dict(enabled=self.traffic_off is None, off=self.traffic_off, note=self.traffic_note, yielding=shown)

    def command(self, robot_id, action, body):
        if robot_id not in self.robots:
            raise CommandError('Unknown robot or action', 'unknown_route')
        if action in ('camera_start', 'camera_stop'):
            controller = self.camera_controls.get(robot_id)
            if not controller:
                raise CommandError('카메라 제어가 준비되지 않았습니다.', 'camera_control_unavailable')
            try:
                return controller.request(action == 'camera_start')
            except ValueError as exc:
                raise CommandError(str(exc), 'camera_control_failed') from exc
        if action not in ('goal', 'initialpose', 'stop', 'spin'):
            raise CommandError('Unknown robot or action', 'unknown_route')
        robot = self.robots[robot_id]
        if action != 'stop':
            map_id, _ = self.map()
            if not map_id or robot.snapshot()['map_id'] != map_id:
                raise CommandError('공통 지도 수신을 기다리세요. 두 로봇의 지도가 같아야 합니다.', 'map_mismatch')
        with self.traffic_lock:
            self.retries.pop(robot_id, None)      # 사람이 명령하면 다시 보내기 횟수는 새로 센다
        if robot_id != FOLLOWER or not self.gate:
            return self.send_command(robot_id, robot, action, body)
        with self.traffic_lock:   # 사람이 명령하면(거절되더라도) 양보하며 쥐고 있던 목적지는 버린다
            paused = self.yielding['action'] != 'go'
            self.yielding = self.no_yield()
            try:
                return self.send_command(robot_id, robot, action, body)
            except CommandError as exc:
                if paused and exc.code == 'cancel_rejected':   # 양보로 멈춰 있어 Nav2 목표가 없었다
                    return '양보 중이던 목표 취소 — 다시 출발하지 않아요'
                raise

    def send_command(self, robot_id, robot, action, body):
        """목적지는 칸 검사(통로 앞·다른 로봇의 목적지)만 하고 바로 보낸다. 겹치면 달리는 중에 traffic_tick이 양보시킨다.
        다른 로봇이 지금 서 있는 자리는 막지 않는다: 자리 바꾸기처럼 곧 떠날 수 있고, 안 떠나면 Nav2가 장애물로 본다."""
        if action == 'goal' and self.gate and self.traffic_off is None:
            x, y, heading = pose_input(body)
            problem = self.gate.check_target(robot_id, dict(x=x, y=y, yaw=heading), self.traffic_robots(),
                                             parked=False)
            if problem:
                raise CommandError(problem[1], problem[0])
        return robot.command(action, body)

    def traffic_robots(self):
        """칸 검사가 보는 로봇 상태 {id: {pose, active, target}}. pose·target은 (x, y), 연결이 끊기면 pose는 None."""
        robots = {}
        for robot_id, robot in self.robots.items():
            s = robot.snapshot()
            target = s['nav'].get('target')
            robots[robot_id] = dict(pose=(s['pose']['x'], s['pose']['y']) if s['pose'] else None,
                                    active=s['nav']['active'],
                                    target=(target['x'], target['y']) if target else None)
        return robots

    def retry_tick(self):
        """막혀서 실패한 목표(RETRY_CODES)를 같은 목적지로 RETRY_LIMIT번까지 다시 보낸다. 사람이 새 목적지를 주면 횟수를 새로 센다."""
        for robot_id, robot in self.robots.items():
            nav = robot.snapshot()['nav']
            target, code = nav.get('target'), nav.get('error_code')
            if nav.get('state') != 'aborted' or not target or code not in RETRY_CODES:
                continue                          # 결과(error_code)가 아직 안 왔으면 다음 번에 본다
            if robot_id == FOLLOWER and self.yielding['action'] != 'go':
                continue                          # 양보 중: 양보가 끝나면 다시 보낸다
            r = self.retries.get(robot_id)
            if r and r['seen'] == nav['id']:
                continue                          # 이 실패는 이미 처리했다
            if not r or r['target'] != target:
                r = self.retries[robot_id] = dict(target=target, count=0, seen=None)
            r['seen'] = nav['id']
            if r['count'] >= RETRY_LIMIT:
                self.traffic_note = f'{robot_id} {RETRY_LIMIT}번 다시 보내도 실패 — 길을 확인하고 직접 보내 주세요'
                continue
            r['count'] += 1
            try:
                robot.command('goal', target)
                self.traffic_note = f'{robot_id} 막혀서 실패 → 다시 보냄 ({r["count"]}/{RETRY_LIMIT})'
            except Exception as exc:
                self.traffic_note = f'{robot_id} 다시 보내기 실패: {exc}'

    def traffic_tick(self):
        """0.2초마다: 지도 확인 후 FOLLOWER 양보 판단."""
        if not self.gate or self.traffic_off:
            return
        _, map_data = self.map()
        if map_data is None:
            return   # 지도를 받을 때까지는 판단하지 않는다
        mismatch = map_mismatch(self.gate.config, map_data)
        if mismatch:
            with self.traffic_lock:
                self.traffic_off = f'교통 정리 꺼짐: {mismatch}'
                self.yielding = self.no_yield()
            return
        with self.traffic_lock:
            self.yield_tick()

    def yield_tick(self):
        """FOLLOWER가 할 일을 정하고 바꿀 때만 명령한다. traffic_lock 안에서 부른다."""
        y, follower = self.yielding, self.robots[FOLLOWER]
        if y['backing'] is not None and not y['backing'].done():
            return                                # 물러나는 중: 끝나면 다시 본다
        y['backing'] = None
        lead, me = self.robots[LEADER].snapshot(), follower.snapshot()
        if y['action'] == 'go' and not me['nav']['active']:
            return                                # 목표 없이 서 있는 로봇은 건드리지 않는다
        action, reason = self.rule.decide(yield_view(lead), yield_view(me), y['action'])
        if action == 'back' and y['backs'] >= BACK_LIMIT:
            action, reason = 'stop', f'{BACK_LIMIT}번 물러나도 {LEADER} 길 위예요 — 멈춰서 기다려요'
        elif action == 'stop':
            y['backs'] = 0                        # 길에서 벗어났다: 다음에 다시 길 위가 되면 또 물러날 수 있다
        if action == y['action'] == 'go':
            return
        if action == 'go':
            self.resume(y, follower)
            return
        if y['action'] == 'go':                   # 달리던 중: 목적지를 기억하고 Nav2 목표를 멈춘다
            target = me['nav'].get('target')
            if not target:
                self.traffic_note = f'{FOLLOWER} 목적지를 몰라(RViz로 보낸 목표?) 양보할 수 없어요'
                return
            follower.command('stop', {})
            y['target'] = target
            self.traffic_note = f'{FOLLOWER} 양보 — {LEADER} 먼저'
        y['action'], y['reason'] = action, reason
        if action == 'back':
            y['backs'] += 1
            y['backing'] = follower.back_up(BACK_STEP, BACK_SPEED)

    def resume(self, y, follower):
        target = y['target']
        self.yielding = self.no_yield()
        if not target:
            return
        try:
            follower.command('goal', target)
            self.traffic_note = f'{FOLLOWER} {LEADER}이 지나가서 다시 출발'
        except Exception as exc:                  # 못 보냈으면 사람이 다시 보낸다(멈춰 있으니 안전하다)
            self.traffic_note = f'{FOLLOWER} 다시 출발 실패: {exc} — 목적지를 다시 보내 주세요'

    def run_traffic(self, period=0.2):
        """교통 스레드 본체. 오류가 나도 멈추지 않는다(대시보드가 죽으면 launch 전체가 꺼진다)."""
        while not self.traffic_stop.wait(period):
            try:
                self.traffic_tick()
            except Exception as exc:
                self.traffic_note = f'교통 정리 오류: {exc}'
            try:
                with self.traffic_lock:
                    self.retry_tick()
            except Exception as exc:
                self.traffic_note = f'다시 보내기 오류: {exc}'
