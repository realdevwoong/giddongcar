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
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


# action_msgs/GoalStatus 번호 → (상태 키, 화면 글자). 0(UNKNOWN)이거나 목표가 없으면 대기.
NAV_STATES = {1: ('accepted', '목표 수락'), 2: ('executing', '이동 중'), 3: ('canceling', '취소 중'),
              4: ('succeeded', '도착'), 5: ('canceled', '취소됨'), 6: ('aborted', '이동 실패')}
# NavigateToPose 결과 error_code (nav2_msgs FollowPath 1xx, ComputePathToPose 2xx)
NAV_ERRORS = {101: '경로 추종기 설정 오류', 102: '위치 변환(TF) 실패', 103: '경로가 잘못됨',
              104: '시간 한도 초과', 105: '진전 없음 — 막혀서 못 움직임', 106: '앞에 장애물 — 안전한 속도를 못 찾음',
              107: '경로 추종 시간 초과', 201: '경로 계획기 설정 오류', 202: '위치 변환(TF) 실패',
              203: '출발 위치가 지도 밖', 204: '목적지가 지도 밖', 205: '출발 위치가 장애물 위',
              206: '목적지가 장애물 위', 207: '경로 계획 시간 초과', 208: '갈 수 있는 경로 없음'}


# 목표 상태 → 로봇 램프 (mode, (r, g, b), time ms, 화면 글자). 실물 pinky_lamp_control과 시뮬 sim_lamp가 같은 set_lamp로 받는다.
# mode: 1 켜기, 2 깜빡임, 3 숨쉬기
LAMP = {'idle': (3, (1.0, 1.0, 1.0), 1000, '흰색 숨쉬기'),
        'accepted': (2, (0.0, 0.4, 1.0), 500, '파랑 깜빡임'),
        'executing': (2, (0.0, 0.4, 1.0), 500, '파랑 깜빡임'),
        'canceling': (1, (1.0, 0.7, 0.0), 0, '노랑'),
        'canceled': (1, (1.0, 0.7, 0.0), 0, '노랑'),
        'succeeded': (1, (0.0, 1.0, 0.2), 0, '초록'),
        'aborted': (2, (1.0, 0.0, 0.0), 250, '빨강 빠른 깜빡임')}


def seconds(duration):
    return round(duration.sec + duration.nanosec / 1e9, 1)


def remember(table, key, value, keep=20):
    table[key] = value
    while len(table) > keep:
        table.pop(next(iter(table)))


def pose_input(body):
    if not isinstance(body, dict):
        raise ValueError('Expected a JSON object')
    values = [float(body[k]) for k in ('x', 'y', 'yaw')]
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Coordinates must be finite')
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
        # 목적지 좌표와 실패 이유는 대시보드가 보낸 목표만 안다.
        self.goal = dict(id=None, code=0, feedback=None)
        self.targets, self.errors = {}, {}
        # 로봇 램프: 마지막으로 보낸 상태와 결과(ok None=보내는 중, False=램프 노드 없음/실패)
        self.lamp_state, self.lamp = None, None
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
        self.create_timer(2.0, self.retry_lamp)
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
        with self.lock:
            self.last_odom = time.monotonic()

    def on_status(self, msg):
        if not msg.status_list:
            return
        latest = max(msg.status_list, key=lambda s: (s.goal_info.stamp.sec, s.goal_info.stamp.nanosec))
        goal_id = bytes(latest.goal_info.goal_id.uuid).hex()
        with self.lock:
            if goal_id != self.goal['id']:
                self.goal = dict(id=goal_id, code=latest.status, feedback=None)
            else:
                self.goal['code'] = latest.status
        state = NAV_STATES.get(latest.status, ('idle',))[0]
        if state != self.lamp_state:
            self.set_lamp(state)

    def set_lamp(self, state):
        """로봇 램프를 상태 색으로. 기다리지 않는다(램프 때문에 관제가 멈추면 안 된다)."""
        mode, (r, g, b), period, label = LAMP[state]
        with self.lock:
            self.lamp_state = state
            if not self.lamp_client.service_is_ready():
                self.lamp = dict(label=label, ok=False)  # 램프 노드가 없다. retry_lamp가 다시 시도한다
                return
            self.lamp = dict(label=label, ok=None)
        request = SetLamp.Request(mode=mode, time=period)
        request.color.r, request.color.g, request.color.b, request.color.a = r, g, b, 1.0

        def done(future):
            with self.lock:
                if self.lamp_state == state:
                    self.lamp = dict(label=label, ok=bool(future.exception() is None and future.result().result))
        self.lamp_client.call_async(request).add_done_callback(done)

    def retry_lamp(self):
        # 로봇 램프 노드가 대시보드보다 늦게 켜졌거나 다시 켜졌을 때
        if self.lamp_state and self.lamp and self.lamp['ok'] is False and self.lamp_client.service_is_ready():
            self.set_lamp(self.lamp_state)

    def on_feedback(self, msg):
        fb = msg.feedback
        with self.lock:
            if bytes(msg.goal_id.uuid).hex() == self.goal['id']:
                self.goal['feedback'] = dict(
                    distance=round(fb.distance_remaining, 2), remaining=seconds(fb.estimated_time_remaining),
                    elapsed=seconds(fb.navigation_time), recoveries=fb.number_of_recoveries)

    def on_result(self, goal_id, future):
        try:
            result = future.result().result
        except Exception:
            return
        if result.error_code:
            reason = NAV_ERRORS.get(result.error_code) or result.error_msg or '알 수 없는 오류'
            with self.lock:
                remember(self.errors, goal_id, f'{reason} (코드 {result.error_code})')

    def update_pose(self):
        pose = None
        try:
            tf = self.buffer.lookup_transform('map', 'base_link', Time())
            age = (self.get_clock().now().nanoseconds - Time.from_msg(tf.header.stamp).nanoseconds) / 1e9
            if -2 < age < 2:
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
            nav = dict(id=goal_id and goal_id[:8], state=state, label=label,
                       target=self.targets.get(goal_id), feedback=self.goal['feedback'],
                       error=self.errors.get(goal_id) if state == 'aborted' else None)
            return dict(id=self.robot_name, domain=self.domain, online=online,
                        pose=self.pose if online else None, map_id=self.map_id,
                        path=self.path if online else [], status=label, nav=nav, lamp=self.lamp,
                        nav_ready=self.navigator.server_is_ready())

    def command(self, action, body):
        # Serialize HTTP requests per robot; the ROS executor remains independent.
        with self.command_lock:
            if action == 'stop':
                if not self.cancel.wait_for_service(timeout_sec=1):
                    raise ValueError('Nav2 취소 서비스가 준비되지 않았습니다.')
                response = await_future(self.cancel.call_async(CancelGoal.Request()))
                if response.return_code != CancelGoal.Response.ERROR_NONE:
                    raise ValueError('Nav2가 취소 요청을 수락하지 않았습니다.')
                return '이동 취소 요청 수락'
            x, y, heading = pose_input(body)
            if action == 'initialpose':
                if self.initial.get_subscription_count() == 0:
                    raise ValueError('AMCL이 아직 준비되지 않았습니다.')
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
                raise ValueError('로봇 연결과 지도상의 초기 위치를 먼저 확인하세요.')
            if not self.navigator.wait_for_server(timeout_sec=1):
                raise ValueError('Nav2가 아직 준비되지 않았습니다.')
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
            goal.pose.pose.orientation.z = math.sin(heading / 2)
            goal.pose.pose.orientation.w = math.cos(heading / 2)
            handle = await_future(self.navigator.send_goal_async(goal))
            if not handle.accepted:
                raise ValueError('Nav2가 목적지를 거부했습니다.')
            goal_id = bytes(handle.goal_id.uuid).hex()
            with self.lock:
                remember(self.targets, goal_id, dict(x=x, y=y, yaw=heading))
            handle.get_result_async().add_done_callback(lambda future: self.on_result(goal_id, future))
            return '목적지 수락 — 이동 상태를 확인하세요.'

    def close(self):
        # launch는 SIGINT 후 5초면 SIGTERM으로 올리므로 로봇 2대 합쳐 그 안에 끝나야 한다.
        self.ros_executor.shutdown(timeout_sec=1)
        self.thread.join(timeout=1)
        self.destroy_node()
        self.ros_context.try_shutdown()


class Fleet:
    def __init__(self, robots):
        self.robots = robots

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
        return dict(map_id=map_id, robots=states)

    def command(self, robot_id, action, body):
        if robot_id not in self.robots or action not in ('goal', 'initialpose', 'stop'):
            raise ValueError('Unknown robot or action')
        robot = self.robots[robot_id]
        if action != 'stop':
            map_id, _ = self.map()
            if not map_id or robot.snapshot()['map_id'] != map_id:
                raise ValueError('공통 지도 수신을 기다리세요. 두 로봇의 지도가 같아야 합니다.')
        return robot.command(action, body)


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
                    raise ValueError('Unknown route')
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 4096:
                    raise ValueError('Invalid request size')
                body = json.loads(self.rfile.read(length))
                message = fleet.command(parts[2], parts[3], body)
                self.send(200, dict(success=True, message=message))
            except (ValueError, KeyError, TypeError) as exc:
                self.send(400, dict(success=False, error=str(exc)))
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
    args = parser.parse_args()
    # nohup이나 스크립트 백그라운드로 띄우면 SIGINT 무시가 상속돼 Ctrl+C로 안 꺼진다. 항상 KeyboardInterrupt를 받게 한다.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    robots = {}
    server = None
    try:
        for i, domain in enumerate((args.robot1_domain, args.robot2_domain), 1):
            robots[f'robot{i}'] = Robot(f'robot{i}', domain, args.use_sim_time)
        server = ThreadingHTTPServer((args.host, args.port), handler_for(Fleet(robots)))
        print(f'Fleet dashboard: http://{args.host}:{args.port}', flush=True)
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        # Terminal and parent launch can both send SIGINT during shutdown.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if server:
            server.server_close()
        for robot in robots.values():
            robot.close()


if __name__ == '__main__':
    main()
