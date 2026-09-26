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
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Path as NavPath, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from tf2_ros import Buffer, TransformListener

ROOT = Path(__file__).resolve().parent


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


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
    def __init__(self, name, domain):
        self.ros_context = Context()
        rclpy.init(context=self.ros_context, domain_id=domain)
        super().__init__(f'fleet_dashboard_{name}', context=self.ros_context)
        self.robot_name, self.domain = name, domain
        self.lock = threading.RLock()
        self.command_lock = threading.Lock()
        self.map_data, self.map_id = None, None
        self.path, self.pose = [], None
        self.last_odom = 0
        self.status = '대기'
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, 'map', self.on_map, transient)
        self.create_subscription(NavPath, 'plan', self.on_path, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(GoalStatusArray, 'navigate_to_pose/_action/status', self.on_status, transient)
        self.initial = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.navigator = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cancel = self.create_client(CancelGoal, 'navigate_to_pose/_action/cancel_goal')
        self.create_timer(0.1, self.update_pose)
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
        labels = {1: '목표 수락', 2: '이동 중', 3: '취소 중', 4: '도착', 5: '취소됨', 6: '이동 실패'}
        with self.lock:
            self.status = labels.get(latest.status, '대기')

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
            return dict(id=self.robot_name, domain=self.domain, online=online,
                        pose=self.pose if online else None, map_id=self.map_id,
                        path=self.path if online else [], status=self.status,
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
            return '목적지 수락 — 이동 상태를 확인하세요.'

    def close(self):
        self.ros_executor.shutdown(timeout_sec=2)
        self.thread.join(timeout=2)
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
                self.send(200, (ROOT / 'fleet.html').read_bytes(), 'text/html; charset=utf-8')
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
    args = parser.parse_args()
    robots = {}
    server = None
    try:
        for i, domain in enumerate((args.robot1_domain, args.robot2_domain), 1):
            robots[f'robot{i}'] = Robot(f'robot{i}', domain)
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
