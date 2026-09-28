#!/usr/bin/env python3
"""One HTTP UI, separate ROS contexts and TF buffers for each robot. No Flask needed."""
import argparse
import hashlib
import json
import math
import signal
from pathlib import Path
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Path as NavPath, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from pinky_interfaces.srv import SetLamp
from pinky_fleet.traffic import TrafficGate, load_zones, map_mismatch
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


# action_msgs/GoalStatus 번호 → (상태 키, 화면 글자). 0(UNKNOWN)이거나 목표가 없으면 대기.
NAV_STATES = {1: ('accepted', '목표 수락'), 2: ('executing', '이동 중'), 3: ('canceling', '취소 중'),
              4: ('succeeded', '도착'), 5: ('canceled', '취소됨'), 6: ('aborted', '이동 실패')}
# NavigateToPose 결과 error_code (nav2_msgs FollowPath 1xx, ComputePathToPose 2xx)
# 104: RPP "collision ahead" 같은 제어 실패가 controller_server의 failure_tolerance(0.3 s)보다 오래 이어질 때
NAV_ERRORS = {100: '경로 추종 중 알 수 없는 오류', 101: '경로 추종기 설정 오류', 102: '위치 변환(TF) 실패',
              103: '경로가 잘못됨', 104: '제어 실패가 계속됨 — 앞이 막혔을 수 있음(collision ahead 등)',
              105: '진전 없음 — 막혀서 못 움직임', 106: '앞에 장애물 — 안전한 속도를 못 찾음',
              107: '경로 추종 시간 초과', 200: '경로 계획 중 알 수 없는 오류', 201: '경로 계획기 설정 오류', 202: '위치 변환(TF) 실패',
              203: '출발 위치가 지도 밖', 204: '목적지가 지도 밖', 205: '출발 위치가 장애물 위',
              206: '목적지가 장애물 위', 207: '경로 계획 시간 초과', 208: '갈 수 있는 경로 없음'}
# 복구 동작을 다 쓰고 실패하면 error_code 0으로 끝나기도 한다. 그래도 이유 칸은 비우지 않는다.
NO_REASON = '이유 코드 없음 (Nav2 복구를 다 써도 실패)'


# 목표 상태 → 로봇 램프 (mode, (r, g, b), time ms, 화면 글자). 실물 pinky_lamp_control과 시뮬 sim_lamp가 같은 set_lamp로 받는다.
# mode: 1 켜기, 2 깜빡임, 3 숨쉬기
LAMP = {'idle': (3, (1.0, 1.0, 1.0), 1000, '흰색 숨쉬기'),
        'accepted': (2, (0.0, 0.4, 1.0), 500, '파랑 깜빡임'),
        'executing': (2, (0.0, 0.4, 1.0), 500, '파랑 깜빡임'),
        'canceling': (1, (1.0, 0.7, 0.0), 0, '노랑'),
        'canceled': (1, (1.0, 0.7, 0.0), 0, '노랑'),
        'succeeded': (1, (0.0, 1.0, 0.2), 0, '초록'),
        'aborted': (2, (1.0, 0.0, 0.0), 250, '빨강 빠른 깜빡임')}
LAMP_HOLD = 5.0     # 도착·취소 색을 보여 주는 시간(초). 그다음 대기로. 실패 빨강은 다음 목표까지 둔다(경보)
LAMP_TIMEOUT = 3.0  # 램프 서비스가 이 시간(초) 안에 답이 없으면 실패로 보고 다시 보낸다


class CommandError(ValueError):
    """명령 실패. code는 화면이 글자 대신 보고 판단하는 이름이다(예: 'map_mismatch')."""
    def __init__(self, message, code=None):
        super().__init__(message)
        self.code = code


def seconds(duration):
    return round(duration.sec + duration.nanosec / 1e9, 1)


def remember(table, key, value, keep=20):
    table[key] = value
    while len(table) > keep:
        table.pop(next(iter(table)))


def pose_input(body):
    if not isinstance(body, dict):
        raise CommandError('Expected a JSON object', 'bad_pose')
    try:
        values = [float(body[k]) for k in ('x', 'y', 'yaw')]
    except (KeyError, TypeError, ValueError):
        raise CommandError('x, y, yaw 숫자가 필요합니다.', 'bad_pose') from None
    if not all(math.isfinite(v) for v in values):
        raise CommandError('Coordinates must be finite', 'bad_pose')
    return values


def await_future(future, timeout=4):
    ready = threading.Event()
    future.add_done_callback(lambda _: ready.set())
    if not ready.wait(timeout):
        # Do not retry automatically: a timed-out request may still reach Nav2.
        raise TimeoutError('응답 시간 초과: 요청이 처리됐을 수 있습니다. 상태를 확인하세요.')
    return future.result()


class Robot(Node):
    def __init__(self, name, domain, use_sim_time=False):
        self.ros_context = Context()
        rclpy.init(context=self.ros_context, domain_id=domain)
        # 시뮬 시간이면 now()가 /clock을 따라가서 Gazebo TF 스탬프가 2초 신선도 검사를 통과한다.
        super().__init__(f'fleet_dashboard_{name}', context=self.ros_context,
                         parameter_overrides=[Parameter('use_sim_time', value=use_sim_time)])
        self.robot_name, self.domain = name, domain
        self.lock = threading.RLock()
        self.command_lock = threading.Lock()
        self.map_data, self.map_id = None, None
        self.path, self.pose = [], None
        self.last_odom = 0
        # 가장 최근 Nav2 목표. 누가 보냈든(대시보드, RViz) 상태와 진행은 보이고,
        # 목적지 좌표와 결과(error_code, 실패 이유)는 대시보드가 보낸 목표만 안다.
        self.goal = dict(id=None, code=0, feedback=None)
        self.targets, self.results = {}, {}
        self.sent = None   # 대시보드가 마지막으로 보낸 목표 {id, at}. Nav2 상태에 나타나기 전에도 '진행 중'으로 친다
        # 로봇 램프: 마지막으로 정한 상태와 결과(ok None=보내는 중, False=램프 노드 없음/실패)
        self.lamp_state, self.lamp = None, None
        # 램프를 바꾼 (목표, 상태), 그 상태로 바꾼 시각, 마지막 요청을 보낸 시각
        self.lamp_goal, self.lamp_since, self.lamp_sent = None, 0.0, 0.0
        self.lamp_future = None   # 아직 답을 기다리는 램프 요청
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, 'map', self.on_map, transient)
        self.create_subscription(NavPath, 'plan', self.on_path, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(GoalStatusArray, 'navigate_to_pose/_action/status', self.on_status, transient)
        self.create_subscription(NavigateToPose.Impl.FeedbackMessage, 'navigate_to_pose/_action/feedback',
                                 self.on_feedback, 10)
        self.initial = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.navigator = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cancel = self.create_client(CancelGoal, 'navigate_to_pose/_action/cancel_goal')
        self.lamp_client = self.create_client(SetLamp, 'set_lamp')
        self.create_timer(0.1, self.update_pose)
        self.create_timer(1.0, self.check_lamp)
        self.ros_executor = SingleThreadedExecutor(context=self.ros_context)
        self.ros_executor.add_node(self)
        self.thread = threading.Thread(target=self.spin, daemon=True)
        self.thread.start()

    def spin(self):
        from rclpy.executors import ExternalShutdownException
        try:
            self.ros_executor.spin()
        except ExternalShutdownException:
            pass

    def on_map(self, msg):
        if msg.header.frame_id != 'map':
            return
        info = msg.info
        data = dict(width=info.width, height=info.height, resolution=info.resolution,
                    origin=dict(x=info.origin.position.x, y=info.origin.position.y,
                                yaw=yaw(info.origin.orientation)), data=list(msg.data))
        digest = hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()
        with self.lock:
            self.map_data, self.map_id = data, digest

    def on_path(self, msg):
        with self.lock:
            self.path = ([dict(x=p.pose.position.x, y=p.pose.position.y) for p in msg.poses]
                         if msg.header.frame_id == 'map' else [])

    def on_odom(self, msg):
        # 로봇 시계 차이 = 이 노드 시각 - 로봇이 찍은 시각. 인터넷 없는 공유기에서는 로봇 시계가 틀어져
        # Nav2가 변환에 실패한다(예: 멀리 있는 목적지인데 0초 만에 '도착'). 화면에 경고로 띄운다.
        skew = (self.get_clock().now().nanoseconds - Time.from_msg(msg.header.stamp).nanoseconds) / 1e9
        with self.lock:
            self.last_odom = time.monotonic()
            self.clock_skew = skew

    def on_status(self, msg):
        seen = (None, 0)  # 목표가 하나도 없으면 대기
        if msg.status_list:
            latest = max(msg.status_list, key=lambda s: (s.goal_info.stamp.sec, s.goal_info.stamp.nanosec))
            goal_id = bytes(latest.goal_info.goal_id.uuid).hex()
            with self.lock:
                if goal_id != self.goal['id']:
                    self.goal = dict(id=goal_id, code=latest.status, feedback=None)
                else:
                    self.goal['code'] = latest.status
            seen = (goal_id, latest.status)
        # 램프는 (목표, 상태)가 바뀔 때만 바꾼다. 같은 메시지가 또 와도 대기로 돌아간 램프를 되돌리지 않는다.
        if seen != self.lamp_goal:
            self.lamp_goal = seen
            self.set_lamp(NAV_STATES.get(seen[1], ('idle',))[0])

    def set_lamp(self, state):
        """로봇 램프를 상태 색으로 바꾼다."""
        with self.lock:
            self.lamp_state, self.lamp_since = state, time.monotonic()
        self.send_lamp(state)

    def send_lamp(self, state):
        """램프 요청을 보낸다. 기다리지 않는다(램프 때문에 관제가 멈추면 안 된다)."""
        mode, (r, g, b), period, label = LAMP[state]
        shown = lambda ok: dict(label=label, ok=ok, rgb=[r, g, b])
        with self.lock:
            if not self.lamp_client.service_is_ready():
                self.lamp = shown(False)  # 램프 노드가 없다. check_lamp가 다시 시도한다
                return
            self.lamp, self.lamp_sent = shown(None), time.monotonic()
        request = SetLamp.Request(mode=mode, time=period)
        request.color.r, request.color.g, request.color.b, request.color.a = r, g, b, 1.0

        def done(future):
            with self.lock:
                if self.lamp_state == state:
                    self.lamp = shown(bool(future.exception() is None and future.result().result))
        future = self.lamp_client.call_async(request)
        with self.lock:
            self.lamp_future = future
        future.add_done_callback(done)

    def check_lamp(self):
        """1초마다: 도착·취소 색이 지나면 대기로, 실패했거나 답이 없는 램프 요청은 다시 보낸다."""
        now = time.monotonic()
        with self.lock:
            state, since, lamp = self.lamp_state, self.lamp_since, self.lamp
            if lamp and lamp['ok'] is None and now - self.lamp_sent > LAMP_TIMEOUT:
                self.lamp = lamp = dict(lamp, ok=False)  # 서비스 응답이 끝내 안 왔다
                if self.lamp_future is not None:  # 답 없는 요청이 클라이언트에 쌓이지 않게 버린다
                    self.lamp_client.remove_pending_request(self.lamp_future)
                    self.lamp_future = None
            retry = lamp and lamp['ok'] is False and now - self.lamp_sent > LAMP_TIMEOUT
        if state in ('succeeded', 'canceled') and now - since > LAMP_HOLD:
            self.set_lamp('idle')
        # 램프 노드가 늦게 켜졌거나, 답이 없었거나, 거부했을 때. 마지막 요청에서 LAMP_TIMEOUT 뒤에만 다시 보낸다
        elif retry and self.lamp_client.service_is_ready():
            self.send_lamp(state)

    def on_feedback(self, msg):
        fb = msg.feedback
        with self.lock:
            if bytes(msg.goal_id.uuid).hex() == self.goal['id']:
                self.goal['feedback'] = dict(
                    distance=round(fb.distance_remaining, 2), remaining=seconds(fb.estimated_time_remaining),
                    elapsed=seconds(fb.navigation_time), recoveries=fb.number_of_recoveries)

    def on_result(self, goal_id, future):
        try:
            response = future.result()
        except Exception:
            return
        code, reason = response.result.error_code, None
        # 실패 여부는 error_code가 아니라 결과 상태로 본다
        if response.status == GoalStatus.STATUS_ABORTED:
            reason = NAV_ERRORS.get(code) or response.result.error_msg or (NO_REASON if code == 0 else '알 수 없는 오류')
            if code:
                reason = f'{reason} (코드 {code})'
        with self.lock:
            remember(self.results, goal_id, dict(code=code, reason=reason))

    def update_pose(self):
        """map → base_link 위치. 신선도는 로봇이 찍은 시각이 아니라 이 PC가 '새 값을 받은 시각'으로 본다.
        로봇 시계가 PC와 달라도(인터넷 없는 공유기) 그려진다. 2초 동안 새 값이 없으면 숨긴다."""
        pose = None
        try:
            tf = self.buffer.lookup_transform('map', 'base_link', Time())
            stamp, now = Time.from_msg(tf.header.stamp).nanoseconds, time.monotonic()
            if stamp != getattr(self, 'tf_stamp', None):   # 새 위치가 들어왔다(들어온 순서로 본다)
                self.tf_stamp, self.tf_arrival = stamp, now
            if now - self.tf_arrival < 2:
                t = tf.transform
                pose = dict(x=t.translation.x, y=t.translation.y, yaw=yaw(t.rotation))
        except Exception:
            pass
        with self.lock:
            self.pose = pose

    def snapshot(self):
        with self.lock:
            online = time.monotonic() - self.last_odom < 2
            goal_id = self.goal['id']
            state, label = NAV_STATES.get(self.goal['code'], ('idle', '대기'))
            result = self.results.get(goal_id, {})
            # 목표 진행 중: Nav2가 수락·이동·취소 중이거나, 방금 보낸 목표가 아직 상태에 안 나타났다(10초까지)
            unseen = self.sent and self.sent['id'] != goal_id and time.monotonic() - self.sent['at'] < 10
            nav = dict(id=goal_id and goal_id[:8], state=state, label=label,
                       active=bool(state in ('accepted', 'executing', 'canceling') or unseen),
                       target=self.targets.get(goal_id), feedback=self.goal['feedback'],
                       error=result.get('reason') if state == 'aborted' else None,
                       error_code=result.get('code'))
            return dict(id=self.robot_name, domain=self.domain, online=online,
                        pose=self.pose if online else None, map_id=self.map_id,
                        path=self.path if online else [], status=label, nav=nav, lamp=self.lamp,
                        clock_skew=round(getattr(self, 'clock_skew', 0.0), 2) if online else None,
                        nav_ready=self.navigator.server_is_ready())

    def command(self, action, body):
        # Serialize HTTP requests per robot; the ROS executor remains independent.
        with self.command_lock:
            if action == 'stop':
                if not self.cancel.wait_for_service(timeout_sec=1):
                    raise CommandError('Nav2 취소 서비스가 준비되지 않았습니다.', 'cancel_unavailable')
                response = await_future(self.cancel.call_async(CancelGoal.Request()))
                if response.return_code != CancelGoal.Response.ERROR_NONE:
                    raise CommandError('Nav2가 취소 요청을 수락하지 않았습니다.', 'cancel_rejected')
                return '이동 취소 요청 수락'
            x, y, heading = pose_input(body)
            if action == 'initialpose':
                if self.initial.get_subscription_count() == 0:
                    raise CommandError('AMCL이 아직 준비되지 않았습니다.', 'amcl_not_ready')
                msg = PoseWithCovarianceStamped()
                msg.header.frame_id = 'map'
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
                msg.pose.pose.orientation.z = math.sin(heading / 2)
                msg.pose.pose.orientation.w = math.cos(heading / 2)
                msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
                msg.pose.covariance[35] = 0.06854
                self.initial.publish(msg)
                return '초기 위치 전송 완료 — 지도에서 위치를 확인하세요.'
            state = self.snapshot()
            if not state['online'] or state['pose'] is None:
                raise CommandError('로봇 연결과 지도상의 초기 위치를 먼저 확인하세요.', 'no_pose')
            if not self.navigator.wait_for_server(timeout_sec=1):
                raise CommandError('Nav2가 아직 준비되지 않았습니다.', 'nav2_not_ready')
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
            goal.pose.pose.orientation.z = math.sin(heading / 2)
            goal.pose.pose.orientation.w = math.cos(heading / 2)
            handle = await_future(self.navigator.send_goal_async(goal))
            if not handle.accepted:
                raise CommandError('Nav2가 목적지를 거부했습니다.', 'goal_rejected')
            goal_id = bytes(handle.goal_id.uuid).hex()
            with self.lock:
                remember(self.targets, goal_id, dict(x=x, y=y, yaw=heading))
                self.sent = dict(id=goal_id, at=time.monotonic())
            handle.get_result_async().add_done_callback(lambda future: self.on_result(goal_id, future))
            return '목적지 수락 — 이동 상태를 확인하세요.'

    def close(self):
        # launch는 SIGINT 후 5초면 SIGTERM으로 올리므로 로봇 2대 합쳐 그 안에 끝나야 한다.
        self.ros_executor.shutdown(timeout_sec=1)
        self.thread.join(timeout=1)
        self.destroy_node()
        self.ros_context.try_shutdown()


class Fleet:
    def __init__(self, robots, zones=None):
        self.robots = robots
        # 교통 정리(칸 열쇠). zones가 없으면 끈다. 지도가 구역 파일과 다르면 첫 확인 때 스스로 끈다
        self.gate = TrafficGate(zones) if zones else None
        self.traffic_lock = threading.Lock()   # HTTP 스레드와 교통 스레드가 열쇠를 동시에 만지지 않게
        self.traffic_off = None if zones else '구역 파일 없음'
        self.traffic_note = None               # 마지막 교통 정리 오류·알림
        self.traffic_stop = threading.Event()

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
            state['map_matches'] = bool(map_id and state['map_id'] == map_id)
        return dict(map_id=map_id, robots=states, traffic=self.traffic_state())

    def traffic_state(self):
        if not self.gate:
            return dict(enabled=False, off=self.traffic_off)
        with self.traffic_lock:
            snap = self.gate.snapshot(time.monotonic())
        return dict(enabled=self.traffic_off is None, off=self.traffic_off, note=self.traffic_note, **snap)

    def command(self, robot_id, action, body):
        if robot_id not in self.robots or action not in ('goal', 'initialpose', 'stop'):
            raise CommandError('Unknown robot or action', 'unknown_route')
        robot = self.robots[robot_id]
        if action != 'stop':
            map_id, _ = self.map()
            if not map_id or robot.snapshot()['map_id'] != map_id:
                raise CommandError('공통 지도 수신을 기다리세요. 두 로봇의 지도가 같아야 합니다.', 'map_mismatch')
        waiting = False
        if action in ('stop', 'goal') and self.gate:
            with self.traffic_lock:   # 사람이 취소하거나 새 목표를 주면(거절되더라도) 기다리던 목표는 지운다
                waiting = any(p['robot'] == robot_id for p in self.gate.pending)
                self.gate.cancel(robot_id)
        if action == 'goal' and self.gate and self.traffic_off is None:
            return self.command_goal(robot_id, robot, body)
        try:
            return robot.command(action, body)
        except CommandError as exc:
            if waiting and exc.code == 'cancel_rejected':   # 대기 중이라 Nav2 목표가 없었다
                return '대기 목표 취소 — 자동 출발하지 않아요'
            raise

    def command_goal(self, robot_id, robot, body):
        """지나갈 칸 열쇠를 모두 받을 수 있을 때만 보낸다. 아니면 보류했다가 traffic_tick이 보낸다."""
        x, y, heading = pose_input(body)
        target = dict(x=x, y=y, yaw=heading)
        me = robot.snapshot()
        if not me['online'] or me['pose'] is None:
            raise CommandError('로봇 연결과 지도상의 초기 위치를 먼저 확인하세요.', 'no_pose')
        robots = self.traffic_robots()
        with self.traffic_lock:
            problem = self.gate.check_target(robot_id, target, robots)
            if problem:
                raise CommandError(problem[1], problem[0])
            try:
                decision, detail = self.gate.request(robot_id, (me['pose']['x'], me['pose']['y']), target,
                                                     time.monotonic(), robots)
            except ValueError as exc:
                raise CommandError(str(exc), 'no_route') from None
        if decision == 'hold':
            if me['nav']['active']:   # 다른 곳으로 가던 중이면 멈춘다. 새 목표를 줬으니 옛 목표로 가면 안 된다
                try:
                    robot.command('stop', {})
                except Exception:     # 못 멈췄으면 기다리게 두면 안 된다(옛 목표로 달리는 중)
                    with self.traffic_lock:
                        self.gate.cancel(robot_id)
                    raise CommandError('가던 목표를 멈추지 못해 새 목표를 대기시키지 않았어요 — '
                                       '■ 이동 취소 후 다시 보내세요', 'hold_stop_failed') from None
            blocker, why = detail
            return f'칸 대기 — {blocker}: {why}. 비면 자동으로 출발해요.'
        try:
            return robot.command('goal', body)
        except CommandError:
            with self.traffic_lock:
                self.gate.release_all(robot_id)   # 확실히 못 보냈으니 방금 받은 열쇠를 돌려놓는다
            raise
        # 시간 초과(TimeoutError)는 목표가 Nav2에 닿았을 수 있어 열쇠를 쥔 채 둔다.
        # 로봇이 실제로 안 움직였다면 start_grace 뒤 update가 반납한다.

    def traffic_robots(self):
        """열쇠 규칙이 보는 로봇 상태 {id: {pose, active, target}}. pose·target은 (x, y), 연결이 끊기면 pose는 None."""
        robots = {}
        for robot_id, robot in self.robots.items():
            s = robot.snapshot()
            target = s['nav'].get('target')
            robots[robot_id] = dict(pose=(s['pose']['x'], s['pose']['y']) if s['pose'] else None,
                                    active=s['nav']['active'],
                                    target=(target['x'], target['y']) if target else None)
        return robots

    def traffic_tick(self):
        """0.2초마다: 지도 확인, 열쇠 반납, 기다리던 목표 자동 출발."""
        if not self.gate or self.traffic_off:
            return
        _, map_data = self.map()
        if map_data is None:
            return   # 지도를 받을 때까지는 판단하지 않는다
        mismatch = map_mismatch(self.gate.config, map_data)
        if mismatch:
            with self.traffic_lock:
                self.traffic_off = f'교통 정리 꺼짐: {mismatch}'
                self.gate.pending.clear()
            return
        robots = self.traffic_robots()
        with self.traffic_lock:
            ready, dropped = self.gate.update(robots, time.monotonic())
        for robot_id, reason in dropped:
            self.traffic_note = f'{robot_id} 대기 목표를 버렸어요: {reason} — 다시 보내 주세요'
        for robot_id, target in ready:
            try:
                self.robots[robot_id].command('goal', target)
                self.traffic_note = f'{robot_id} 칸이 비어 자동 출발'
            except CommandError as exc:          # 확실히 못 보냄: 열쇠 반납
                with self.traffic_lock:
                    self.gate.release_all(robot_id)
                self.traffic_note = f'{robot_id} 자동 출발 실패: {exc} — 다시 보내 주세요'
            except Exception as exc:             # 시간 초과 등: 닿았을 수 있어 열쇠는 쥔 채 둔다
                self.traffic_note = f'{robot_id} 자동 출발 응답 없음: {exc} — 로봇 상태를 확인하세요'

    def run_traffic(self, period=0.2):
        """교통 스레드 본체. 오류가 나도 멈추지 않는다(대시보드가 죽으면 launch 전체가 꺼진다)."""
        while not self.traffic_stop.wait(period):
            try:
                self.traffic_tick()
            except Exception as exc:
                self.traffic_note = f'교통 정리 오류: {exc}'


def handler_for(fleet):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send(self, status, body, content_type='application/json; charset=utf-8'):
            payload = body if isinstance(body, bytes) else json.dumps(body, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self):
            if self.path == '/':
                page = Path(get_package_share_directory('pinky_fleet')) / 'web' / 'fleet.html'
                self.send(200, page.read_bytes(), 'text/html; charset=utf-8')
            elif self.path == '/api/state':
                self.send(200, fleet.state())
            elif self.path == '/api/map':
                map_id, data = fleet.map()
                self.send(200, dict(id=map_id, map=data))
            else:
                self.send(404, dict(error='Not found'))

        def do_POST(self):
            try:
                if self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                    raise ValueError('Expected application/json')
                parts = self.path.strip('/').split('/')
                if len(parts) != 4 or parts[:2] != ['api', 'robots']:
                    raise CommandError('Unknown route', 'unknown_route')
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 4096:
                    raise ValueError('Invalid request size')
                body = json.loads(self.rfile.read(length))
                message = fleet.command(parts[2], parts[3], body)
                self.send(200, dict(success=True, message=message))
            except (ValueError, KeyError, TypeError) as exc:
                code = exc.code if isinstance(exc, CommandError) else None
                self.send(400, dict(success=False, error=str(exc), code=code))
            except TimeoutError as exc:
                self.send(504, dict(success=False, error=str(exc)))
            except Exception as exc:
                self.send(500, dict(success=False, error=str(exc)))
    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot1-domain', type=int, default=15)
    parser.add_argument('--robot2-domain', type=int, default=17)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--use-sim-time', action='store_true', help='Gazebo: /clock 시간을 쓴다')
    parser.add_argument('--traffic-zones', default='', help='교통 정리 구역 YAML. 비우면 교통 정리를 끈다')
    args = parser.parse_args()
    zones = load_zones(args.traffic_zones) if args.traffic_zones else None
    # nohup이나 스크립트 백그라운드로 띄우면 SIGINT 무시가 상속돼 Ctrl+C로 안 꺼진다. 항상 KeyboardInterrupt를 받게 한다.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    robots = {}
    server = fleet = None
    try:
        for i, domain in enumerate((args.robot1_domain, args.robot2_domain), 1):
            robots[f'robot{i}'] = Robot(f'robot{i}', domain, args.use_sim_time)
        fleet = Fleet(robots, zones)
        if fleet.gate:
            threading.Thread(target=fleet.run_traffic, daemon=True).start()
        server = ThreadingHTTPServer((args.host, args.port), handler_for(fleet))
        print(f'Fleet dashboard: http://{args.host}:{args.port}'
              + (' · 교통 정리 켬' if fleet.gate else ' · 교통 정리 끔'), flush=True)
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        # Terminal and parent launch can both send SIGINT during shutdown.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if fleet:
            fleet.traffic_stop.set()
        if server:
            server.server_close()
        for robot in robots.values():
            robot.close()


if __name__ == '__main__':
    main()
