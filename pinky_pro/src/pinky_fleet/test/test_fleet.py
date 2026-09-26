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
from geometry_msgs.msg import TransformStamped
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
        robot = SimpleNamespace(lock=threading.RLock(), last_odom=time.monotonic()-5,
                                robot_name='robot1', domain=15, pose={'x': 4},
                                map_id='same', path=[{'x': 3}], status='이동 중',
                                navigator=SimpleNamespace(server_is_ready=lambda: True))
        result = Robot.snapshot(robot)
        self.assertFalse(result['online'])
        self.assertIsNone(result['pose'])
        self.assertEqual(result['path'], [])

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
    def start(self, use_sim_time, env=ISOLATED):
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
            use_sim_time=use_sim_time)
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


if __name__ == '__main__':
    unittest.main()
