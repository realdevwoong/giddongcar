import importlib.util
import io
import json
import math
import os
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, call, patch

import yaml
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose
from launch import LaunchContext
from launch.actions import ExecuteProcess
from launch.utilities import perform_substitutions
from rclpy.time import Time

import pinky_fleet.fleet_dashboard as dashboard
from pinky_fleet.fleet_dashboard import Fleet, Robot, handler_for, pose_input, await_future


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
        with self.assertRaises(ValueError):
            self.fleet.command('robot2', 'goal', dict(x=1, y=2, yaw=0))
        self.two.command.assert_not_called()
        self.fleet.command('robot2', 'stop', {})
        self.two.command.assert_called_once_with('stop', {})

    def test_unknown_route_does_not_dispatch(self):
        for robot, action in [('robot3', 'goal'), ('robot1', 'reset')]:
            with self.assertRaises(ValueError):
                self.fleet.command(robot, action, {})
        self.one.command.assert_not_called()

    def test_rejects_nonfinite_coordinates(self):
        for value in (math.nan, math.inf, -math.inf):
            with self.assertRaises(ValueError):
                pose_input(dict(x=value, y=0, yaw=0))

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

    def nav_robot(self):
        # DDS 없이 상태 처리 코드만 돌리는 가짜 로봇
        return SimpleNamespace(lock=threading.RLock(), goal=dict(id=None, code=0, feedback=None),
                               targets={}, errors={}, last_odom=time.monotonic(), robot_name='robot1',
                               domain=25, pose={'x': 0}, map_id='same', path=[],
                               navigator=SimpleNamespace(server_is_ready=lambda: True))

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

    def test_abort_reason_comes_from_nav2_error_code(self):
        robot = self.nav_robot()
        Robot.on_status(robot, self.status((7, GoalStatus.STATUS_ABORTED, 1)))
        goal_id = robot.goal['id']
        done = lambda code, msg='': Mock(result=Mock(return_value=SimpleNamespace(
            result=SimpleNamespace(error_code=code, error_msg=msg))))
        Robot.on_result(robot, goal_id, done(0))
        self.assertIsNone(Robot.snapshot(robot)['nav']['error'])
        Robot.on_result(robot, goal_id, done(208))
        self.assertEqual(Robot.snapshot(robot)['nav']['error'], '갈 수 있는 경로 없음 (코드 208)')
        Robot.on_result(robot, goal_id, done(150, 'custom'))
        self.assertEqual(Robot.snapshot(robot)['nav']['error'], 'custom (코드 150)')

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

    def test_http_invalid_robot_returns_error(self):
        handler_class = handler_for(self.fleet)
        handler = object.__new__(handler_class)
        handler.path = '/api/robots/robot3/goal'
        handler.headers = {'Content-Type': 'application/json', 'Content-Length': '2'}
        handler.rfile = io.BytesIO(b'{}')
        handler.send = Mock()
        handler.do_POST()
        self.assertEqual(handler.send.call_args.args[0], 400)
        self.one.command.assert_not_called()

    def test_http_goal_routing(self):
        handler = object.__new__(handler_for(self.fleet))
        handler.path = '/api/robots/robot2/goal'
        payload = json.dumps(dict(x=1, y=2, yaw=0)).encode()
        handler.headers = {'Content-Type': 'application/json', 'Content-Length': str(len(payload))}
        handler.rfile = io.BytesIO(payload)
        handler.send = Mock()
        handler.do_POST()
        self.assertEqual(handler.send.call_args.args[0], 200)
        self.two.command.assert_called_once()
        self.one.command.assert_not_called()

    def test_pose_age_uses_node_clock(self):
        # Gazebo TF 스탬프는 시뮬 시간이라 노드 시계도 시뮬 시간일 때만 신선하다.
        tf = TransformStamped()
        tf.header.stamp.sec = 100
        tf.transform.rotation.w = 1.0
        for now, visible in ((100.5, True), (1.79e9, False)):
            robot = SimpleNamespace(lock=threading.RLock(), pose=None, buffer=Mock(),
                                    get_clock=lambda now=now: SimpleNamespace(now=lambda: Time(seconds=now)))
            robot.buffer.lookup_transform.return_value = tf
            Robot.update_pose(robot)
            self.assertEqual(robot.pose is not None, visible)

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
                         [call('robot1', 15, True), call('robot2', 17, True)])
        self.assertEqual(self.run_dashboard().call_args_list,
                         [call('robot1', 15, False), call('robot2', 17, False)])

    def test_robot_node_follows_sim_clock_when_asked(self):
        # 진짜 노드를 하나 만든다(이 PC 안에서만, 안 쓰는 도메인). /clock을 따라가는 ROS 시간이 켜져야 한다.
        with patch.dict(os.environ, ISOLATED):
            robot = Robot('probe', 77, use_sim_time=True)
        try:
            self.assertTrue(robot.get_parameter('use_sim_time').value)
            self.assertTrue(robot.get_clock().ros_time_is_active)
        finally:
            robot.close()


LAUNCH_FILE = Path(__file__).resolve().parents[1] / 'launch' / 'multi_robot.launch.py'
ISOLATED = {'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST', 'ROS_STATIC_PEERS': '',
            'FASTRTPS_DEFAULT_PROFILES_FILE': '', 'ROS_DISCOVERY_SERVER': ''}


class LaunchTests(unittest.TestCase):
    def start(self, use_sim_time, env=ISOLATED, poses=('', '')):
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
            robot1_domain='15', robot2_domain='17', host='127.0.0.1', port='8080',
            use_sim_time=use_sim_time, robot1_initial_pose=poses[0], robot2_initial_pose=poses[1])
        with patch.dict(os.environ, env):
            actions = module.start(context)
        self.actions = actions  # 살려 둬야 임시 params 파일이 지워지지 않는다
        processes = [a for a in actions if isinstance(a, ExecuteProcess)]
        cmds = [[perform_substitutions(context, part) for part in p.cmd] for p in processes]
        domains = [dict((perform_substitutions(context, k), perform_substitutions(context, v))
                        for k, v in p.additional_env or []).get('ROS_DOMAIN_ID') for p in processes]
        return cmds, domains

    def test_real_mode_is_unchanged(self):
        cmds, domains = self.start('false', env={})
        self.assertEqual(domains, ['15', '17', None])
        for cmd in cmds[:2]:
            self.assertEqual(cmd[:4], ['ros2', 'launch', 'pinky_navigation', 'bringup_launch.xml'])
            self.assertTrue(cmd[4].startswith('map:=') and cmd[5].startswith('params_file:='))
            self.assertEqual(len(cmd), 6)
        self.assertEqual(cmds[2], ['/fake/prefix/lib/pinky_fleet/fleet_dashboard',
                                   '--robot1-domain', '15', '--robot2-domain', '17',
                                   '--host', '127.0.0.1', '--port', '8080'])

    def test_generated_params_wait_for_initial_pose(self):
        cmds, _ = self.start('false', env={})
        config = yaml.safe_load(Path(cmds[0][5].removeprefix('params_file:=')).read_text())
        self.assertFalse(config['amcl']['ros__parameters']['set_initial_pose'])
        for costmap in ('global_costmap', 'local_costmap'):
            self.assertEqual(config[costmap][costmap]['ros__parameters']['initial_transform_timeout'], 600.0)

    def test_gazebo_mode_runs_both_nav2_on_sim_time(self):
        cmds, domains = self.start('True')
        self.assertEqual(domains, ['15', '17', None])
        self.assertTrue(all('use_sim_time:=True' in c for c in cmds[:2]))
        self.assertIn('--use-sim-time', cmds[2])

    def test_gazebo_mode_requires_local_only_discovery(self):
        for key, value in (('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET'),
                           ('ROS_STATIC_PEERS', '192.168.0.2'),
                           ('FASTRTPS_DEFAULT_PROFILES_FILE', '/home/x/.ros/wifi.xml'),
                           ('ROS_DISCOVERY_SERVER', '192.168.0.5:11811')):
            with self.assertRaises(RuntimeError):
                self.start('true', env={**ISOLATED, key: value})

    def test_rejects_unknown_use_sim_time(self):
        with self.assertRaises(RuntimeError):
            self.start('yes')

    def test_real_mode_shares_one_params_waiting_for_pose(self):
        cmds, _ = self.start('false', env={})
        self.assertEqual(cmds[0][5], cmds[1][5])

    def test_known_start_pose_is_set_when_amcl_starts(self):
        cmds, _ = self.start('true', poses=('0.5,0.5,0', '2.0, 0.5, 1.57'))
        amcl = [yaml.safe_load(Path(c[5].removeprefix('params_file:=')).read_text())['amcl']['ros__parameters']
                for c in cmds[:2]]
        self.assertTrue(amcl[0]['set_initial_pose'] and amcl[1]['set_initial_pose'])
        self.assertEqual(amcl[0]['initial_pose'], dict(x=0.5, y=0.5, z=0.0, yaw=0.0))
        self.assertEqual(amcl[1]['initial_pose'], dict(x=2.0, y=0.5, z=0.0, yaw=1.57))

    def test_pose_only_for_one_robot_keeps_the_other_waiting(self):
        cmds, _ = self.start('true', poses=('0.5,0.5,0', ''))
        second = yaml.safe_load(Path(cmds[1][5].removeprefix('params_file:=')).read_text())
        self.assertFalse(second['amcl']['ros__parameters']['set_initial_pose'])

    def test_rejects_malformed_initial_pose(self):
        for text in ('0.5,0.5', 'a,b,c', 'nan,0,0', '1,2,3,4'):
            with self.assertRaises(RuntimeError):
                self.start('true', poses=(text, ''))


if __name__ == '__main__':
    unittest.main()
