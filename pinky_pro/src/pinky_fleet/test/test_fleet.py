import importlib.util
import io
import json
import math
import os
import tempfile
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, call, patch

import yaml
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose
from launch import LaunchContext
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.utilities import perform_substitutions
from rclpy.time import Time

import pinky_fleet.fleet_dashboard as dashboard
from pinky_fleet.fleet_dashboard import CommandError, Fleet, Robot, handler_for, pose_input, await_future
from pinky_fleet.localize import Localizer
from pinky_fleet.camera_stream import MJPEGCamera
from pinky_fleet.perception import YOLOPerception


class CameraStreamTests(unittest.TestCase):
    def test_receives_http_mjpeg_from_camera_root(self):
        frame = b'\xff\xd8test-frame\xff\xd9'
        requested = threading.Event()
        paths = []

        class MJPEGHandler(BaseHTTPRequestHandler):
            protocol_version = 'HTTP/1.1'

            def do_GET(self):
                paths.append(self.path)
                requested.set()
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.end_headers()
                try:
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                    self.wfile.flush()
                    while True:
                        time.sleep(0.02)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def log_message(self, *_args):
                pass

        server = ThreadingHTTPServer(('127.0.0.1', 0), MJPEGHandler)
        server_thread = threading.Thread(target=server.serve_forever, daemon=True)
        server_thread.start()
        camera = MJPEGCamera('127.0.0.1', server.server_port, 'test')
        try:
            camera.start()
            deadline = time.monotonic() + 2
            received = None
            while time.monotonic() < deadline:
                received, sequence = camera.latest()
                if sequence:
                    break
                time.sleep(0.01)
            self.assertTrue(requested.is_set())
            self.assertEqual(paths, ['/'])
            self.assertEqual(received, frame)
        finally:
            camera.close()
            server.shutdown()
            server.server_close()

    def test_reads_pinky_html_viewer_snapshot_endpoint(self):
        frame = b'\xff\xd8camera-snapshot\xff\xd9'
        requested = []
        page = b"<!doctype html><img id=v><script>img.src='/snapshot?t='+Date.now();</script>"

        class SnapshotHandler(BaseHTTPRequestHandler):
            def do_GET(self):
                requested.append(self.path)
                if self.path == '/':
                    body, content_type = page, 'text/html; charset=utf-8'
                elif self.path.startswith('/snapshot?t='):
                    body, content_type = frame, 'image/jpeg'
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *_args):
                pass

        server = ThreadingHTTPServer(('127.0.0.1', 0), SnapshotHandler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        camera = MJPEGCamera('127.0.0.1', server.server_port, 'snapshot-test')
        try:
            camera.start()
            deadline = time.monotonic() + 2
            received, sequence = None, 0
            while time.monotonic() < deadline:
                received, sequence = camera.latest()
                if sequence:
                    break
                time.sleep(0.01)
            self.assertEqual(received, frame)
            self.assertGreater(sequence, 0)
            self.assertTrue(any(path.startswith('/snapshot?t=') for path in requested))
            self.assertEqual(camera.status()['state'], 'online')
        finally:
            camera.close()
            server.shutdown()
            server.server_close()

    def test_extracts_multiple_jpegs_and_keeps_partial_marker(self):
        first, second = b'\xff\xd8one\xff\xd9', b'\xff\xd8two\xff\xd9'
        frames, remaining = MJPEGCamera._take_frames(b'noise' + first + second + b'\xff')
        self.assertEqual(frames, [first, second])
        self.assertEqual(remaining, b'\xff')

    def test_wait_next_returns_newest_frame(self):
        camera = MJPEGCamera('', name='robot1')
        camera._publish(b'jpeg-frame')
        frame, sequence = camera.wait_next(0, timeout=0.01)
        self.assertEqual((frame, sequence), (b'jpeg-frame', 1))
        self.assertEqual(camera.status()['state'], 'disabled')

    def test_ble_reported_camera_url_can_override_port_and_path(self):
        camera = MJPEGCamera('192.168.0.6', 5000, 'robot1')
        self.assertTrue(camera.set_stream_url('http://192.168.0.6:8765/camera/stream'))
        self.assertEqual((camera.host, camera.port, camera.path),
                         ('192.168.0.6', 8765, '/camera/stream'))
        self.assertTrue(camera.set_stream_url(':5001'))
        self.assertEqual((camera.host, camera.port, camera.path), ('192.168.0.6', 5001, '/'))

    def test_camera_status_distinguishes_waiting_from_received_video(self):
        camera = MJPEGCamera('camera-host', name='robot1')
        camera._set_connection(True)
        status = camera.status()
        self.assertEqual(status['state'], 'waiting')
        self.assertEqual(status['frames'], 0)
        camera._publish(b'jpeg-frame')
        self.assertEqual(camera.status()['state'], 'online')

    def test_camera_connection_error_is_exposed_in_status(self):
        camera = MJPEGCamera('camera-host', name='robot1')
        camera._set_connection(False, 'MJPEG 응답 형식 오류')
        status = camera.status()
        self.assertEqual(status['state'], 'offline')
        self.assertEqual(status['error'], 'MJPEG 응답 형식 오류')

    def test_overlay_status_keeps_detection_confidence(self):
        camera = MJPEGCamera('camera-host', name='robot1')
        camera.set_inference_state('waiting', 'model.pt')
        camera._publish(b'jpeg-frame')
        detection = dict(label='person', confidence=0.91, xyxy=[1.0, 2.0, 3.0, 4.0])
        camera.set_overlay(1, b'annotated-frame', 42.0, [detection])
        frame, _ = camera.display_frame()
        self.assertEqual(frame, b'annotated-frame')
        self.assertEqual(camera.status()['inference']['detections'], [detection])

    def test_disabled_perception_does_not_load_yolo(self):
        perception = YOLOPerception({'robot1': MJPEGCamera('')}, 'unused.pt')
        self.assertIsNone(perception.model)


class FakeRobot:
    def __init__(self, name, map_id='same'):
        self.name = name
        self.map_id = map_id
        self.map_data = {'width': 1}
        self.lock = threading.RLock()
        self.command = Mock(return_value='accepted')

    def snapshot(self):
        return dict(id=self.name, map_id=self.map_id)


class FleetTests(unittest.TestCase):
    def setUp(self):
        self.one, self.two = FakeRobot('robot1'), FakeRobot('robot2')
        self.fleet = Fleet(dict(robot1=self.one, robot2=self.two))

    def test_routes_goal_only_to_selected_robot(self):
        payload = dict(x=1, y=2, yaw=0)
        self.fleet.command('robot2', 'goal', payload)
        self.two.command.assert_called_once_with('goal', payload)
        self.one.command.assert_not_called()

    def test_mismatched_map_blocks_goals_but_allows_cancel(self):
        self.two.map_id = 'different'
        self.assertFalse(self.fleet.state()['robots'][1]['map_matches'])
        with self.assertRaises(CommandError) as caught:
            self.fleet.command('robot2', 'goal', dict(x=1, y=2, yaw=0))
        self.assertEqual(caught.exception.code, 'map_mismatch')
        self.two.command.assert_not_called()
        self.fleet.command('robot2', 'stop', {})
        self.two.command.assert_called_once_with('stop', {})

    def test_unknown_route_does_not_dispatch(self):
        for robot, action in [('robot3', 'goal'), ('robot1', 'reset')]:
            with self.assertRaises(CommandError) as caught:
                self.fleet.command(robot, action, {})
            self.assertEqual(caught.exception.code, 'unknown_route')
        self.one.command.assert_not_called()

    def test_rejects_bad_coordinates(self):
        for body in (dict(x=math.nan, y=0, yaw=0), dict(x=math.inf, y=0, yaw=0), dict(x=-math.inf, y=0, yaw=0),
                     dict(x=1, y=2), dict(x='a', y=0, yaw=0), [1, 2, 3]):
            with self.subTest(body=body):
                with self.assertRaises(CommandError) as caught:
                    pose_input(body)
                self.assertEqual(caught.exception.code, 'bad_pose')

    def test_command_errors_have_codes(self):
        # DDS 없이 Robot.command만 돌린다. 인자로 준 것만 준비가 안 된 가짜 로봇
        def answered(value):
            return Mock(add_done_callback=lambda callback: callback(None), result=Mock(return_value=value))

        def robot(online=True, nav2=True, accepted=True, amcl=1, cancel=True, cancel_code=0):
            return SimpleNamespace(
                command_lock=threading.Lock(), lock=threading.RLock(), targets={}, backup_handle=None,
                snapshot=lambda: dict(online=online, pose={'x': 0} if online else None),
                get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=1)),
                localizer=Localizer(known=True), stop_spin=Mock(return_value=False),
                initial=Mock(get_subscription_count=Mock(return_value=amcl)),
                navigator=Mock(wait_for_server=Mock(return_value=nav2),
                               send_goal_async=Mock(return_value=answered(SimpleNamespace(accepted=accepted)))),
                cancel=Mock(wait_for_service=Mock(return_value=cancel),
                            call_async=Mock(return_value=answered(SimpleNamespace(return_code=cancel_code)))))
        cases = [(robot(cancel=False), 'stop', 'cancel_unavailable'),
                 (robot(cancel_code=CancelGoal.Response.ERROR_REJECTED), 'stop', 'cancel_rejected'),
                 (robot(amcl=0), 'initialpose', 'amcl_not_ready'),
                 (robot(online=False), 'goal', 'no_pose'),
                 (robot(nav2=False), 'goal', 'nav2_not_ready'),
                 (robot(accepted=False), 'goal', 'goal_rejected')]
        for fake, action, code in cases:
            with self.subTest(code):
                with self.assertRaises(CommandError) as caught:
                    Robot.command(fake, action, dict(x=1, y=2, yaw=0))
                self.assertEqual(caught.exception.code, code)

    def test_timeout_does_not_report_success(self):
        future = Mock()
        with self.assertRaises(TimeoutError):
            await_future(future, timeout=0.001)
        future.result.assert_not_called()

    def test_disconnect_hides_old_pose(self):
        # Exercise the real snapshot logic without initializing DDS.
        robot = self.nav_robot()
        robot.last_odom, robot.pose, robot.path = time.monotonic() - 5, {'x': 4}, [{'x': 3}]
        result = Robot.snapshot(robot)
        self.assertFalse(result['online'])
        self.assertIsNone(result['pose'])
        self.assertEqual(result['path'], [])

    def nav_robot(self, lamp_ready=False):
        # DDS 없이 상태 처리 코드만 돌리는 가짜 로봇
        robot = SimpleNamespace(lock=threading.RLock(), goal=dict(id=None, code=0, feedback=None),
                                targets={}, results={}, sent=None, last_odom=time.monotonic(), robot_name='robot1',
                                domain=25, pose={'x': 0}, map_id='same', path=[],
                                navigator=SimpleNamespace(server_is_ready=lambda: True),
                                localizer=Localizer(known=True), lamp_state=None, lamp=None, lamp_goal=None, lamp_since=0.0, lamp_sent=0.0,
                                lamp_future=None, lamp_client=Mock())
        robot.lamp_client.service_is_ready.return_value = lamp_ready
        robot.lamp_client.call_async.side_effect = lambda request: self.lamp_reply()
        robot.set_lamp = lambda state: Robot.set_lamp(robot, state)
        robot.send_lamp = lambda state: Robot.send_lamp(robot, state)
        return robot

    @staticmethod
    def lamp_reply(answered=True):
        # 램프 서비스 응답(성공). answered=False면 응답이 끝내 안 온다
        future = Mock()
        future.exception.return_value = None
        future.result.return_value = SimpleNamespace(result=True)
        if answered:
            future.add_done_callback.side_effect = lambda callback: callback(future)
        return future

    @staticmethod
    def lamp_requests(robot):
        return [(c.args[0].mode, c.args[0].time) for c in robot.lamp_client.call_async.call_args_list]

    def test_lamp_follows_goal_state_once_per_change(self):
        robot = self.nav_robot(lamp_ready=True)
        Robot.on_status(robot, self.status((1, GoalStatus.STATUS_EXECUTING, 1)))
        Robot.on_status(robot, self.status((1, GoalStatus.STATUS_EXECUTING, 1)))  # 같은 상태면 다시 안 보낸다
        Robot.on_status(robot, self.status((1, GoalStatus.STATUS_SUCCEEDED, 1)))
        sent = [c.args[0] for c in robot.lamp_client.call_async.call_args_list]
        self.assertEqual([(r.mode, r.time) for r in sent], [(2, 500), (1, 0)])
        self.assertEqual((sent[0].color.b, sent[1].color.g), (1.0, 1.0))  # 이동 중 파랑, 도착 초록
        self.assertEqual(Robot.snapshot(robot)['lamp'], dict(label='초록', ok=True, rgb=[0.0, 1.0, 0.2]))

    def test_missing_lamp_node_is_shown_and_retried(self):
        robot = self.nav_robot(lamp_ready=False)
        Robot.on_status(robot, self.status((1, GoalStatus.STATUS_ABORTED, 1)))
        self.assertEqual(Robot.snapshot(robot)['lamp'], dict(label='빨강 빠른 깜빡임', ok=False, rgb=[1.0, 0.0, 0.0]))
        robot.lamp_client.call_async.assert_not_called()
        robot.lamp_client.service_is_ready.return_value = True  # 램프 노드가 늦게 켜짐
        Robot.check_lamp(robot)
        self.assertEqual(self.lamp_requests(robot), [(2, 250)])
        self.assertTrue(Robot.snapshot(robot)['lamp']['ok'])

    def test_unanswered_lamp_request_is_retried(self):
        robot = self.nav_robot(lamp_ready=True)
        robot.lamp_client.call_async.side_effect = [self.lamp_reply(answered=False), self.lamp_reply()]
        Robot.on_status(robot, self.status((1, GoalStatus.STATUS_EXECUTING, 1)))
        Robot.check_lamp(robot)  # 아직 기다리는 중
        self.assertIsNone(Robot.snapshot(robot)['lamp']['ok'])
        robot.lamp_sent -= dashboard.LAMP_TIMEOUT + 1  # 응답이 끝내 안 온다
        robot.lamp_client.service_is_ready.return_value = False
        Robot.check_lamp(robot)
        self.assertIs(Robot.snapshot(robot)['lamp']['ok'], False)  # 보내는 중에 멈추지 않고 실패로 보인다
        robot.lamp_client.remove_pending_request.assert_called_once()  # 답 없는 요청은 버려서 쌓이지 않는다
        robot.lamp_client.service_is_ready.return_value = True
        Robot.check_lamp(robot)
        self.assertEqual(self.lamp_requests(robot), [(2, 500), (2, 500)])
        self.assertTrue(Robot.snapshot(robot)['lamp']['ok'])

    def test_rejected_lamp_is_retried_only_after_timeout(self):
        robot = self.nav_robot(lamp_ready=True)
        rejected = self.lamp_reply()
        rejected.result.return_value = SimpleNamespace(result=False)
        robot.lamp_client.call_async.side_effect = [rejected, self.lamp_reply()]
        Robot.on_status(robot, self.status((1, GoalStatus.STATUS_EXECUTING, 1)))
        Robot.check_lamp(robot)  # 방금 거부당했다: 바로 다시 보내지 않는다(1초마다 깜빡이지 않게)
        self.assertEqual(len(self.lamp_requests(robot)), 1)
        robot.lamp_sent -= dashboard.LAMP_TIMEOUT + 1
        Robot.check_lamp(robot)
        self.assertEqual(len(self.lamp_requests(robot)), 2)
        self.assertTrue(Robot.snapshot(robot)['lamp']['ok'])

    def test_finished_goal_lamp_returns_to_idle_after_hold(self):
        for code, label in ((GoalStatus.STATUS_SUCCEEDED, '초록'), (GoalStatus.STATUS_CANCELED, '노랑')):
            with self.subTest(label):
                robot = self.nav_robot(lamp_ready=True)
                Robot.on_status(robot, self.status((1, code, 1)))
                Robot.check_lamp(robot)  # 잠깐은 결과 색을 보여 준다
                self.assertEqual(Robot.snapshot(robot)['lamp']['label'], label)
                robot.lamp_since -= dashboard.LAMP_HOLD + 1
                Robot.check_lamp(robot)
                Robot.on_status(robot, self.status((1, code, 1)))  # 같은 결과가 또 와도 대기로 둔다
                Robot.check_lamp(robot)
                self.assertEqual(self.lamp_requests(robot), [(1, 0), (3, 1000)])
                self.assertEqual(Robot.snapshot(robot)['lamp'], dict(label='흰색 숨쉬기', ok=True, rgb=[1.0, 1.0, 1.0]))

    def test_aborted_lamp_stays_red_until_next_goal(self):
        robot = self.nav_robot(lamp_ready=True)
        Robot.on_status(robot, self.status((1, GoalStatus.STATUS_ABORTED, 1)))
        robot.lamp_since -= dashboard.LAMP_HOLD + 60
        Robot.check_lamp(robot)  # 실패는 경보라서 시간이 지나도 빨강
        self.assertEqual(Robot.snapshot(robot)['lamp']['label'], '빨강 빠른 깜빡임')
        Robot.on_status(robot, self.status((1, GoalStatus.STATUS_ABORTED, 1), (2, GoalStatus.STATUS_ACCEPTED, 2)))
        self.assertEqual(self.lamp_requests(robot), [(2, 250), (2, 500)])

    def test_no_goals_means_idle_lamp(self):
        robot = self.nav_robot(lamp_ready=True)
        Robot.on_status(robot, GoalStatusArray())
        Robot.on_status(robot, GoalStatusArray())
        self.assertEqual(self.lamp_requests(robot), [(3, 1000)])
        self.assertEqual(Robot.snapshot(robot)['nav']['state'], 'idle')

    @staticmethod
    def status(*goals):
        msg = GoalStatusArray()
        for uid, code, sec in goals:
            s = GoalStatus()
            s.goal_info.goal_id.uuid, s.goal_info.stamp.sec, s.status = [uid] * 16, sec, code
            msg.status_list.append(s)
        return msg

    @staticmethod
    def feedback(uid, distance):
        msg = NavigateToPose.Impl.FeedbackMessage()
        msg.goal_id.uuid = [uid] * 16
        msg.feedback.distance_remaining, msg.feedback.number_of_recoveries = distance, 1
        msg.feedback.navigation_time.sec = 12
        return msg

    def test_card_follows_latest_goal_and_its_progress(self):
        robot = self.nav_robot()
        Robot.on_status(robot, self.status((1, GoalStatus.STATUS_EXECUTING, 10),
                                           (2, GoalStatus.STATUS_SUCCEEDED, 5)))
        Robot.on_feedback(robot, self.feedback(1, 1.234))
        Robot.on_feedback(robot, self.feedback(2, 9.0))  # 지난 목표의 진행은 무시
        nav = Robot.snapshot(robot)['nav']
        self.assertEqual((nav['state'], nav['label']), ('executing', '이동 중'))
        self.assertEqual(nav['feedback'], dict(distance=1.23, remaining=0.0, elapsed=12.0, recoveries=1))
        Robot.on_status(robot, self.status((3, GoalStatus.STATUS_ACCEPTED, 20)))
        nav = Robot.snapshot(robot)['nav']
        self.assertEqual(nav['state'], 'accepted')
        self.assertIsNone(nav['feedback'])  # 새 목표면 진행 정보를 비운다

    @staticmethod
    def result(status, code, msg=''):
        # get_result_async가 돌려주는 응답: 결과 상태 + NavigateToPose 결과
        response = NavigateToPose.Impl.GetResultService.Response(status=status)
        response.result.error_code, response.result.error_msg = code, msg
        return Mock(result=Mock(return_value=response))

    def test_abort_reason_comes_from_nav2_result(self):
        robot = self.nav_robot()
        Robot.on_status(robot, self.status((7, GoalStatus.STATUS_ABORTED, 1)))
        goal_id = robot.goal['id']
        cases = [(0, '', '이유 코드 없음 (Nav2 복구를 다 써도 실패)'),  # 복구를 다 쓰고 실패하면 코드가 0이다
                 (208, '', '갈 수 있는 경로 없음 (코드 208)'),
                 (104, '', '제어 실패가 계속됨 — 앞이 막혔을 수 있음(collision ahead 등) (코드 104)'),
                 (150, 'custom', 'custom (코드 150)'),
                 (9000, '', '알 수 없는 오류 (코드 9000)')]  # 표에 없고 설명도 없는 코드
        for code, msg, reason in cases:
            Robot.on_result(robot, goal_id, self.result(GoalStatus.STATUS_ABORTED, code, msg))
            nav = Robot.snapshot(robot)['nav']
            self.assertEqual((nav['error'], nav['error_code']), (reason, code))

    def test_success_has_code_but_no_reason(self):
        robot = self.nav_robot()
        Robot.on_status(robot, self.status((7, GoalStatus.STATUS_SUCCEEDED, 1)))
        self.assertIsNone(Robot.snapshot(robot)['nav']['error_code'])  # 결과를 아직 모른다
        Robot.on_result(robot, robot.goal['id'], self.result(GoalStatus.STATUS_SUCCEEDED, 0))
        nav = Robot.snapshot(robot)['nav']
        self.assertEqual((nav['error'], nav['error_code']), (None, 0))

    def test_remember_keeps_only_recent_goals(self):
        table = {}
        for i in range(25):
            dashboard.remember(table, i, i)
        self.assertEqual(list(table), list(range(5, 25)))

    def test_tf_failure_clears_previous_pose(self):
        robot = SimpleNamespace(lock=threading.RLock(), pose={'x': 3}, buffer=Mock())
        robot.buffer.lookup_transform.side_effect = RuntimeError('No transform')
        Robot.update_pose(robot)
        self.assertIsNone(robot.pose)

    def post(self, path, payload, content_type='application/json'):
        # 서버 소켓 없이 POST 처리만 돌린다. (HTTP 상태, 응답 본문)
        handler = object.__new__(handler_for(self.fleet))
        body = json.dumps(payload).encode()
        handler.path = path
        handler.headers = {'Content-Type': content_type, 'Content-Length': str(len(body))}
        handler.rfile = io.BytesIO(body)
        handler.send = Mock()
        handler.do_POST()
        return handler.send.call_args.args

    def test_http_invalid_robot_returns_error(self):
        status, body = self.post('/api/robots/robot3/goal', {})
        self.assertEqual((status, body['success'], body['code']), (400, False, 'unknown_route'))
        self.one.command.assert_not_called()

    def test_http_errors_carry_code(self):
        self.two.map_id = 'different'
        self.one.command.side_effect = CommandError('Nav2가 아직 준비되지 않았습니다.', 'nav2_not_ready')
        goal = dict(x=1, y=2, yaw=0)
        cases = [('/api/robots/robot2/goal', 'application/json', 'map_mismatch'),
                 ('/api/robots/robot1/goal', 'application/json', 'nav2_not_ready'),  # 로봇이 낸 코드도 그대로
                 ('/api/robot1/goal', 'application/json', 'unknown_route'),
                 ('/api/robots/robot1/goal', 'text/plain', None)]
        for path, content_type, code in cases:
            with self.subTest(path=path, code=code):
                status, body = self.post(path, goal, content_type)
                self.assertEqual((status, body['success'], body['code']), (400, False, code))
                self.assertTrue(body['error'])

    def test_http_goal_routing(self):
        status, _ = self.post('/api/robots/robot2/goal', dict(x=1, y=2, yaw=0))
        self.assertEqual(status, 200)
        self.two.command.assert_called_once()
        self.one.command.assert_not_called()

    def test_pose_freshness_uses_arrival_time_not_the_robot_clock(self):
        # 로봇 시계가 PC와 한참 달라도(인터넷 없는 공유기, 시뮬 시간) 새 값이 계속 들어오면 그린다
        tf = TransformStamped()
        tf.header.stamp.sec = 100                       # PC 시계와 전혀 다른 시각
        tf.transform.rotation.w = 1.0
        robot = SimpleNamespace(lock=threading.RLock(), pose=None, buffer=Mock())
        robot.buffer.lookup_transform.return_value = tf
        with patch.object(dashboard.time, 'monotonic', return_value=1000.0):
            Robot.update_pose(robot)
        self.assertIsNotNone(robot.pose)
        with patch.object(dashboard.time, 'monotonic', return_value=1001.5):
            Robot.update_pose(robot)                    # 같은 값이 1.5초째: 아직 보인다
        self.assertIsNotNone(robot.pose)
        with patch.object(dashboard.time, 'monotonic', return_value=1002.5):
            Robot.update_pose(robot)                    # 2초 넘게 새 값이 없다: 숨긴다
        self.assertIsNone(robot.pose)
        tf.header.stamp.sec = 101                       # 새 값이 들어오면 다시 보인다
        with patch.object(dashboard.time, 'monotonic', return_value=1003.0):
            Robot.update_pose(robot)
        self.assertIsNotNone(robot.pose)

    def run_dashboard(self, *argv):
        with patch.object(dashboard, 'Robot') as robot, \
                patch.object(dashboard, 'ThreadingHTTPServer') as server, \
                patch.object(dashboard.signal, 'signal'), \
                patch('sys.argv', ['fleet_dashboard', *argv]):
            server.return_value.serve_forever.side_effect = KeyboardInterrupt
            dashboard.main()
        return robot

    def test_dashboard_sim_time_flag_reaches_both_robots(self):
        self.assertEqual(self.run_dashboard('--use-sim-time').call_args_list,
                         [call('robot1', 15, True, False, False), call('robot2', 17, True, False, False)])
        self.assertEqual(self.run_dashboard().call_args_list,
                         [call('robot1', 15, False, False, False), call('robot2', 17, False, False, False)])
        self.assertEqual(self.run_dashboard('--auto-spin').call_args_list,
                         [call('robot1', 15, False, False, True), call('robot2', 17, False, False, True)])

    def test_known_pose_flag_skips_global_localization_for_that_robot(self):
        self.assertEqual(self.run_dashboard('--known-pose', 'robot2').call_args_list,
                         [call('robot1', 15, False, False, False), call('robot2', 17, False, True, False)])

    def test_robot_node_follows_sim_clock_when_asked(self):
        # 진짜 노드를 하나 만든다(이 PC 안에서만, 안 쓰는 도메인). /clock을 따라가는 ROS 시간이 켜져야 한다.
        with patch.dict(os.environ, ISOLATED):
            robot = Robot('probe', 77, use_sim_time=True)
        try:
            self.assertTrue(robot.get_parameter('use_sim_time').value)
            self.assertTrue(robot.get_clock().ros_time_is_active)
        finally:
            robot.close()


class TrafficRobot:
    """교통 정리 시험용 가짜 로봇: 위치·목표 진행·계획 경로만 있고, 받은 명령을 기록한다."""
    def __init__(self, name, x, y, active=False):
        self.name, self.pose, self.active = name, dict(x=x, y=y, yaw=0.0), active
        self.path, self.target = [], None
        self.lock = threading.RLock()
        self.map_id, self.map_data = 'same', GOOD3
        self.command = Mock(return_value='목적지 수락')
        self.backing = SimpleNamespace(done=lambda: True)    # 후진은 바로 끝난 것으로
        self.back_up = Mock(return_value=self.backing)

    def drive(self, x0, y0, x1, y1):
        """(x0, y0)에서 (x1, y1)로 가는 중: 위치·경로·목적지를 그렇게 둔다."""
        self.pose, self.active = dict(x=x0, y=y0, yaw=0.0), True
        self.path = [dict(x=x0 + (x1 - x0) * i / 20, y=y0 + (y1 - y0) * i / 20) for i in range(21)]
        self.target = dict(x=x1, y=y1, yaw=0.0)

    def snapshot(self):
        return dict(id=self.name, map_id=self.map_id, online=True, pose=self.pose, path=self.path,
                    nav=dict(active=self.active, target=self.target))


GOOD3 = dict(width=84, height=56, resolution=0.05, origin=dict(x=-1.067, y=-0.171, yaw=0.0))
ZONES = Path(__file__).resolve().parents[1] / 'params' / 'traffic_good3.yaml'


class TrafficFleetTests(unittest.TestCase):
    def setUp(self):
        self.r1, self.r2 = TrafficRobot('robot1', 0.5, 0.5), TrafficRobot('robot2', 2.0, 0.5)
        self.fleet = Fleet(dict(robot1=self.r1, robot2=self.r2), dashboard.load_zones(ZONES))

    def goal(self, robot, x, y):
        return self.fleet.command(robot, 'goal', dict(x=x, y=y, yaw=0.0))

    def yielding(self):
        return self.fleet.state()['traffic']['yielding']

    def test_both_goals_leave_at_once_even_if_routes_cross(self):
        self.goal('robot1', 2.10, 1.05)
        self.r1.active = True
        self.goal('robot2', 0.30, 1.00)                                     # 예약하지 않고 바로 보낸다
        self.r2.command.assert_called_once_with('goal', dict(x=0.30, y=1.00, yaw=0.0))

    def test_robot2_stops_for_robot1_and_resumes_after_it_passes(self):
        self.r1.drive(1.0, 1.05, 2.1, 1.05)                                 # 문을 왼→오로 건너는 중
        self.r2.drive(2.0, 0.40, 1.9, 1.05)                                 # 옆에서 문 쪽으로 오는 중
        self.fleet.traffic_tick()
        self.r2.command.assert_called_once_with('stop', {})                 # Nav2 목표를 멈췄다
        self.assertEqual(self.yielding()['action'], 'stop')
        robot2_state = next(r for r in self.fleet.state()['robots'] if r['id'] == 'robot2')
        self.assertEqual(robot2_state['nav']['state'], 'yielding')           # 자동 취소를 사용자 취소로 보이지 않는다
        self.assertEqual(robot2_state['nav']['label'], '양보 대기')
        self.r2.active = False                                              # 취소됨
        self.fleet.traffic_tick()
        self.assertEqual(self.r2.command.call_count, 1)                     # robot1이 아직 지나는 중
        self.r1.active = False                                              # robot1 도착
        self.fleet.traffic_tick()
        self.r2.command.assert_called_with('goal', dict(x=1.9, y=1.05, yaw=0.0))   # 원래 목적지로 다시
        self.assertEqual(self.yielding()['action'], 'go')
        self.r1.command.assert_not_called()                                 # robot1은 건드리지 않는다

    def test_robot2_backs_off_robot1_path(self):
        self.r1.drive(1.0, 1.05, 2.1, 1.05)
        self.r2.drive(1.8, 1.05, 0.4, 1.05)                                 # 문 안에서 정면으로 만남
        self.fleet.traffic_tick()
        self.r2.command.assert_called_once_with('stop', {})                 # 먼저 Nav2 목표를 멈추고
        self.r2.back_up.assert_called_once_with(dashboard.BACK_STEP, dashboard.BACK_SPEED)   # 물러난다
        self.assertEqual(self.yielding()['action'], 'back')

    def test_backing_is_limited_then_waits(self):
        self.r1.drive(1.0, 1.05, 2.1, 1.05)
        self.r2.drive(1.8, 1.05, 0.4, 1.05)
        for _ in range(dashboard.BACK_LIMIT + 2):                           # 물러나도 위치가 그대로(벽에 막힘)
            self.fleet.traffic_tick()
        self.assertEqual(self.r2.back_up.call_count, dashboard.BACK_LIMIT)
        self.assertEqual(self.yielding()['action'], 'stop')

    def test_waits_while_backing_is_in_progress(self):
        self.r1.drive(1.0, 1.05, 2.1, 1.05)
        self.r2.drive(1.8, 1.05, 0.4, 1.05)
        self.r2.backing.done = lambda: False
        for _ in range(3):
            self.fleet.traffic_tick()
        self.r2.back_up.assert_called_once()

    def test_robot2_without_a_goal_is_left_alone(self):
        self.r1.drive(1.0, 1.05, 2.1, 1.05)
        self.r2.pose = dict(x=1.8, y=1.05, yaw=0.0)                         # 길 위에 서 있지만 목표 없음
        self.fleet.traffic_tick()
        self.r2.command.assert_not_called()
        self.r2.back_up.assert_not_called()

    def test_human_stop_while_yielding_is_a_success_and_no_resume(self):
        self.r1.drive(1.0, 1.05, 2.1, 1.05)
        self.r2.drive(2.0, 0.40, 1.9, 1.05)
        self.fleet.traffic_tick()                                           # 양보로 멈춤
        self.r2.active = False
        self.r2.command.side_effect = CommandError('Nav2가 취소 요청을 수락하지 않았습니다.', 'cancel_rejected')
        self.assertIn('양보 중이던 목표 취소', self.fleet.command('robot2', 'stop', {}))
        self.r2.command.side_effect = None
        self.r1.active = False
        self.fleet.traffic_tick()
        self.assertNotIn('goal', [c.args[0] for c in self.r2.command.call_args_list])   # 다시 출발하지 않는다

    def test_goal_next_to_the_door_or_on_another_goal_is_rejected(self):
        self.r2.drive(2.0, 0.5, 2.7, 1.1)                                   # robot2가 (2.7, 1.1)로 가는 중
        for (x, y), code in (((1.60, 1.05), 'near_zone'), ((2.75, 1.05), 'near_robot')):
            with self.subTest(code):
                with self.assertRaises(CommandError) as caught:
                    self.goal('robot1', x, y)
                self.assertEqual(caught.exception.code, code)
        self.r1.command.assert_not_called()

    def test_robots_can_swap_places(self):
        # 서 있는 다른 로봇 자리는 곧 빌 수 있어 막지 않는다
        self.goal('robot1', 2.0, 0.5)
        self.goal('robot2', 0.5, 0.5)
        self.r1.command.assert_called_once_with('goal', dict(x=2.0, y=0.5, yaw=0.0))
        self.r2.command.assert_called_once_with('goal', dict(x=0.5, y=0.5, yaw=0.0))

    def fail(self, robot, goal_id, code=104):
        robot.active = False
        robot.snapshot = lambda: dict(id=robot.name, map_id='same', online=True, pose=robot.pose, path=[],
                                      nav=dict(active=False, state='aborted', id=goal_id, error_code=code,
                                               target=dict(x=0.1, y=0.98, yaw=0.0)))

    def test_blocked_goal_is_sent_again_up_to_the_limit(self):
        for i in range(dashboard.RETRY_LIMIT + 1):
            self.fail(self.r2, f'goal{i}')
            self.fleet.retry_tick()
            self.fleet.retry_tick()                                         # 같은 실패를 두 번 보내지 않는다
        self.assertEqual(self.r2.command.call_count, dashboard.RETRY_LIMIT)
        self.r2.command.assert_called_with('goal', dict(x=0.1, y=0.98, yaw=0.0))
        self.assertIn('다시 보내도 실패', self.fleet.state()['traffic']['note'])

    def test_other_failures_are_not_retried(self):
        self.fail(self.r2, 'goal0', code=206)                               # 목적지가 장애물 위: 다시 해도 같다
        self.fleet.retry_tick()
        self.r2.command.assert_not_called()

    def test_human_goal_resets_the_retry_count(self):
        for i in range(dashboard.RETRY_LIMIT):
            self.fail(self.r2, f'goal{i}')
            self.fleet.retry_tick()
        self.r2.snapshot = TrafficRobot.snapshot.__get__(self.r2)
        self.goal('robot2', 0.30, 1.00)                                     # 사람이 다시 보냄
        self.fail(self.r2, 'goal9')
        self.fleet.retry_tick()
        self.r2.command.assert_called_with('goal', dict(x=0.1, y=0.98, yaw=0.0))   # 다시 3번까지

    def test_rejected_new_goal_still_forgets_the_yielded_goal(self):
        self.r1.drive(1.0, 1.05, 2.1, 1.05)
        self.r2.drive(2.0, 0.40, 1.9, 1.05)
        self.fleet.traffic_tick()                                           # 양보로 멈춤
        with self.assertRaises(CommandError):
            self.goal('robot2', 1.60, 1.05)                                 # 문 안: 거절
        self.assertIsNone(self.yielding()['target'])                        # 옛 목적지로 나중에 출발하지 않는다

    def test_wrong_map_turns_traffic_control_off(self):
        self.r1.map_data = self.r2.map_data = dict(GOOD3, width=90)
        self.fleet.traffic_tick()
        self.assertIn('교통 정리 꺼짐', self.fleet.state()['traffic']['off'])
        self.goal('robot1', 2.10, 1.05)
        self.goal('robot2', 0.30, 1.00)
        self.r2.command.assert_called_once()                                # 꺼지면 그냥 보낸다

    def test_just_sent_goal_counts_as_active_until_nav2_reports_it(self):
        robot = FleetTests.nav_robot(FleetTests())
        robot.sent = dict(id='ab' * 16, at=time.monotonic())
        self.assertTrue(Robot.snapshot(robot)['nav']['active'])            # 상태가 아직 안 옴
        robot.sent['at'] -= 11
        self.assertFalse(Robot.snapshot(robot)['nav']['active'])           # 10초 넘게 안 오면 진행 중 아님


class SimLampTests(unittest.TestCase):
    def test_modes_match_real_lamp_rules(self):
        from pinky_fleet.sim_lamp import OFF, lamp_color
        blue = (0.0, 0.4, 1.0)
        self.assertEqual(lamp_color(0, blue, 500, 1.0), OFF)
        self.assertEqual(lamp_color(1, blue, 0, 7.0), blue)
        self.assertEqual([lamp_color(2, blue, 500, t) for t in (0.1, 0.6, 1.1)], [blue, OFF, blue])
        self.assertEqual(lamp_color(3, (1.0, 1.0, 1.0), 1000, 0.5), (0.5, 0.5, 0.5))  # 숨쉬기 중간
        self.assertEqual(lamp_color(3, (1.0, 1.0, 1.0), 1000, 1.0), (1.0, 1.0, 1.0))


class SimLaunchTests(unittest.TestCase):
    def load(self):
        path = Path(__file__).resolve().parents[1] / 'launch' / 'sim.launch.py'
        spec = importlib.util.spec_from_file_location('sim_launch', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_vendor_lamp_plugin_removed_and_topics_moved(self):
        urdf = self.load().gazebo_urdf('robot2')
        self.assertNotIn('gz-sim-lamp-control-system', urdf)
        self.assertIn('<topic>/robot2/', urdf)

    def test_sim_launch_pins_fastdds(self):
        description = self.load().generate_launch_description()
        context = LaunchContext()
        env = [(perform_substitutions(context, action.name), perform_substitutions(context, action.value))
               for action in description.entities
               if isinstance(action, SetEnvironmentVariable)]
        self.assertIn(('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'), env)
        self.assertIn(('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'), env)

    def test_lamp_topic_uses_world_name_from_file(self):
        module = self.load()
        with tempfile.NamedTemporaryFile('w', suffix='.world') as world:
            world.write('<sdf version="1.8">\n  <world name="good_map">\n  </world>\n</sdf>')
            world.flush()
            self.assertEqual(module.material_color_topic(world.name), '/world/good_map/material_color')
        # 기본 월드 두 개 모두 이름을 읽을 수 있어야 한다
        worlds = Path(__file__).resolve().parents[1] / 'worlds'
        self.assertEqual(module.material_color_topic(worlds / 'pinky_factory.world'),
                         '/world/pinky_factory/material_color')


LAUNCH_FILE = Path(__file__).resolve().parents[1] / 'launch' / 'multi_robot.launch.py'
ISOLATED = {'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST', 'ROS_STATIC_PEERS': '',
            'FASTRTPS_DEFAULT_PROFILES_FILE': '', 'ROS_DISCOVERY_SERVER': '',
            'ROS_SUPER_CLIENT': '', 'CYCLONEDDS_URI': ''}
REAL, SIM = ('15', '17'), ('25', '27')  # 도메인: 실물, 시뮬


class LaunchTests(unittest.TestCase):
    def start(self, use_sim_time, domains, env=ISOLATED, poses=('', ''), traffic_zones='', auto_spin='false'):
        spec = importlib.util.spec_from_file_location('multi_robot_launch', LAUNCH_FILE)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        module.get_package_prefix = lambda _: '/fake/prefix'
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        params = Path(temp.name) / 'nav2_params.yaml'
        params.write_text('amcl:\n  ros__parameters:\n    set_initial_pose: true\n')
        context = LaunchContext()
        context.launch_configurations.update(
            map=str(LAUNCH_FILE.parents[1] / 'maps' / 'good3.yaml'), params_file=str(params),
            robot1_domain=domains[0], robot2_domain=domains[1], host='127.0.0.1', port='8080',
            use_sim_time=use_sim_time, robot1_initial_pose=poses[0], robot2_initial_pose=poses[1],
            traffic_zones=traffic_zones, auto_spin=auto_spin)
        with patch.dict(os.environ, env):
            actions = module.start(context)
        self.actions = actions  # 살려 둬야 임시 params 파일이 지워지지 않는다
        processes = [a for a in actions if isinstance(a, ExecuteProcess)]
        cmds = [[perform_substitutions(context, part) for part in p.cmd] for p in processes]
        domains = [dict((perform_substitutions(context, k), perform_substitutions(context, v))
                        for k, v in p.additional_env or []).get('ROS_DOMAIN_ID') for p in processes]
        return cmds, domains

    def test_traffic_zones_reach_the_dashboard(self):
        cmds, _ = self.start('false', ('15', '17'), env={}, traffic_zones='/zones/traffic_good3.yaml')
        self.assertEqual(cmds[2][-2:], ['--traffic-zones', '/zones/traffic_good3.yaml'])
        self.assertTrue(LAUNCH_FILE.with_name('multi_robot.launch.py').read_text().count("'traffic_good3.yaml'"))  # 기본값

    def test_real_mode_is_unchanged(self):
        cmds, domains = self.start('false', REAL, env={})
        self.assertEqual(domains, ['15', '17', None])
        for cmd in cmds[:2]:
            self.assertEqual(cmd[:4], ['ros2', 'launch', 'pinky_navigation', 'bringup_launch.xml'])
            self.assertTrue(cmd[4].startswith('map:=') and cmd[5].startswith('params_file:='))
            self.assertEqual(len(cmd), 6)
        self.assertEqual(cmds[2], ['/fake/prefix/lib/pinky_fleet/fleet_dashboard',
                                   '--robot1-domain', '15', '--robot2-domain', '17',
                                   '--robot1-camera-host', '', '--robot2-camera-host', '',
                                   '--camera-port', '5000',
                                   '--host', '127.0.0.1', '--port', '8080'])

    def test_generated_params_wait_for_initial_pose(self):
        cmds, _ = self.start('false', REAL, env={})
        config = yaml.safe_load(Path(cmds[0][5].removeprefix('params_file:=')).read_text())
        self.assertFalse(config['amcl']['ros__parameters']['set_initial_pose'])
        for costmap in ('global_costmap', 'local_costmap'):
            self.assertEqual(config[costmap][costmap]['ros__parameters']['initial_transform_timeout'], 600.0)

    def test_gazebo_mode_runs_both_nav2_on_sim_time(self):
        cmds, domains = self.start('True', SIM)
        self.assertEqual(domains, ['25', '27', None])
        self.assertTrue(all('use_sim_time:=True' in c for c in cmds[:2]))
        self.assertIn('--use-sim-time', cmds[2])

    def test_gazebo_mode_requires_local_only_discovery(self):
        for key, value in (('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET'),
                           ('ROS_STATIC_PEERS', '192.168.0.2'),
                           ('FASTRTPS_DEFAULT_PROFILES_FILE', '/home/x/.ros/wifi.xml'),
                           ('ROS_DISCOVERY_SERVER', '192.168.0.5:11811'),
                           ('ROS_SUPER_CLIENT', 'true'),
                           ('CYCLONEDDS_URI', 'file:///home/x/.ros/cyclone.xml')):
            with self.subTest(key), self.assertRaisesRegex(RuntimeError, key):  # 무엇을 고칠지 메시지에 나온다
                self.start('true', SIM, env={**ISOLATED, key: value})

    def test_gazebo_mode_refuses_real_robot_domains(self):
        # 시뮬 Nav2가 실물 로봇 도메인에 붙으면 실물에 cmd_vel을 보낼 수 있다
        for domains in (REAL, ('25', '17'), ('15', '27')):
            with self.subTest(domains), self.assertRaisesRegex(RuntimeError, 'robot1_domain:=25 robot2_domain:=27'):
                self.start('true', domains)

    def test_rejects_unknown_use_sim_time(self):
        with self.assertRaises(RuntimeError):
            self.start('yes', SIM)

    def test_auto_spin_reaches_the_dashboard(self):
        cmds, _ = self.start('false', REAL, env={}, auto_spin='true')
        self.assertIn('--auto-spin', cmds[2])
        self.assertIn("DeclareLaunchArgument('auto_spin', default_value='true'", LAUNCH_FILE.read_text())  # 기본은 켬
        with self.assertRaises(RuntimeError):
            self.start('false', REAL, env={}, auto_spin='yes')

    def test_robots_with_known_pose_skip_global_localization(self):
        cmds, _ = self.start('true', SIM, poses=('', '2.0,0.5,0'))
        self.assertIn('--known-pose', cmds[2])
        self.assertEqual(cmds[2][cmds[2].index('--known-pose') + 1], 'robot2')
        self.assertEqual(cmds[2].count('--known-pose'), 1)

    def test_real_mode_shares_one_params_waiting_for_pose(self):
        cmds, _ = self.start('false', REAL, env={})
        self.assertEqual(cmds[0][5], cmds[1][5])

    def test_known_start_pose_is_set_when_amcl_starts(self):
        cmds, _ = self.start('true', SIM, poses=('0.5,0.5,0', '2.0, 0.5, 1.57'))
        amcl = [yaml.safe_load(Path(c[5].removeprefix('params_file:=')).read_text())['amcl']['ros__parameters']
                for c in cmds[:2]]
        self.assertTrue(amcl[0]['set_initial_pose'] and amcl[1]['set_initial_pose'])
        self.assertEqual(amcl[0]['initial_pose'], dict(x=0.5, y=0.5, z=0.0, yaw=0.0))
        self.assertEqual(amcl[1]['initial_pose'], dict(x=2.0, y=0.5, z=0.0, yaw=1.57))

    def test_pose_only_for_one_robot_keeps_the_other_waiting(self):
        cmds, _ = self.start('true', SIM, poses=('0.5,0.5,0', ''))
        second = yaml.safe_load(Path(cmds[1][5].removeprefix('params_file:=')).read_text())
        self.assertFalse(second['amcl']['ros__parameters']['set_initial_pose'])

    def test_rejects_malformed_initial_pose(self):
        for text in ('0.5,0.5', 'a,b,c', 'nan,0,0', '1,2,3,4'):
            with self.assertRaises(RuntimeError):
                self.start('true', SIM, poses=(text, ''))


if __name__ == '__main__':
    unittest.main()
