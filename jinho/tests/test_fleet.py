import io
import json
import math
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from fleet_dashboard import Fleet, Robot, handler_for, pose_input, await_future


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


if __name__ == '__main__':
    unittest.main()
