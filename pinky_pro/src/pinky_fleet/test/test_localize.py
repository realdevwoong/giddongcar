import math
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from action_msgs.srv import CancelGoal

from pinky_fleet import localize
from pinky_fleet.fleet_dashboard import CommandError, Robot
from pinky_fleet.localize import Localizer

WIDE = [0.0] * 36     # 전역 찾기 직후: 후보가 지도 전체에 퍼져 있다
WIDE[0] = WIDE[7] = 4.0
WIDE[35] = 3.0
TIGHT = [0.0] * 36    # 한 곳으로 모였다
TIGHT[0] = TIGHT[7] = 0.01
TIGHT[35] = 0.01


def searching():
    loc = Localizer()
    loc.started()
    return loc


class LocalizerTests(unittest.TestCase):
    def test_known_pose_is_shown_and_never_searches(self):
        loc = Localizer(known=True)
        self.assertTrue(loc.shows_pose)
        self.assertFalse(loc.amcl_ready(100.0))
        self.assertFalse(loc.quiet_tick())

    def test_waits_for_amcl_to_settle_before_searching(self):
        loc = Localizer()
        self.assertFalse(loc.shows_pose)
        self.assertFalse(loc.amcl_ready(10.0))
        self.assertFalse(loc.amcl_ready(10.0 + localize.SETTLE_S - 0.1))
        self.assertTrue(loc.amcl_ready(10.0 + localize.SETTLE_S))

    def test_found_only_after_steady_tight_poses(self):
        loc = searching()
        loc.on_pose(WIDE)
        for _ in range(localize.STEADY - 1):
            self.assertFalse(loc.on_pose(TIGHT))
        loc.on_pose(WIDE)                      # 한 번 흩어지면 처음부터 다시 센다
        for _ in range(localize.STEADY - 1):
            self.assertFalse(loc.on_pose(TIGHT))
        self.assertTrue(loc.on_pose(TIGHT))
        self.assertEqual(loc.phase, 'found')
        self.assertTrue(loc.shows_pose)

    def test_gives_up_quietly_after_quiet_updates(self):
        loc = searching()
        sent = 0
        while loc.quiet_tick():
            sent += 1
        self.assertEqual((sent, loc.phase), (localize.QUIET_UPDATES, 'unsure'))
        self.assertFalse(loc.shows_pose)

    def test_poses_before_search_do_not_count(self):
        loc = Localizer()
        for _ in range(localize.STEADY):
            loc.on_pose(TIGHT)
        self.assertEqual(loc.phase, 'waiting')

    def test_spin_stops_after_one_turn(self):
        loc = searching()
        loc.spin_start(0.0, 0.0)
        heading, t = 0.0, 0.0
        while loc.spin_update(t, math.atan2(math.sin(heading), math.cos(heading))):
            heading += 0.05                    # 0.1초마다 0.05 rad (0.5 rad/s)
            t += 0.1
            self.assertLess(t, localize.SPIN_TIMEOUT)
        self.assertGreaterEqual(heading, localize.SPIN_TURN - 0.1)
        self.assertEqual(loc.phase, 'unsure')

    def test_spin_stops_on_timeout_without_odom(self):
        loc = searching()
        loc.spin_start(0.0, None)
        self.assertTrue(loc.spin_update(1.0, None))
        self.assertFalse(loc.spin_update(localize.SPIN_TIMEOUT, None))

    def test_spin_ends_as_soon_as_found(self):
        loc = searching()
        loc.spin_start(0.0, 0.0)
        for _ in range(localize.STEADY):
            loc.on_pose(TIGHT)
        self.assertEqual(loc.phase, 'found')
        self.assertFalse(loc.spin_update(0.1, 0.1))

    def test_manual_pose_wins(self):
        loc = searching()
        loc.spin_start(0.0, 0.0)
        loc.manual()
        self.assertTrue(loc.shows_pose)
        self.assertFalse(loc.spin_update(0.1, 0.1))
        self.assertFalse(loc.spin_abort())


class DashboardSpinTests(unittest.TestCase):
    """DDS 없이 대시보드의 회전·정지 코드만 돌린다. 끝날 때 0 속도가 나가는지가 핵심."""

    def robot(self):
        robot = SimpleNamespace(lock=threading.RLock(), localizer=Localizer(), odom_yaw=0.0, driving=False,
                                cmd_vel=Mock(), command_lock=threading.Lock(), backup_handle=None,
                                release_at=None, destroy_publisher=Mock(), vision_driving=lambda: False)
        robot.send_zero = lambda: Robot.send_zero(robot)
        robot.velocity = lambda: Robot.velocity(robot)
        robot.stop_spin = lambda: Robot.stop_spin(robot)
        robot.localizer.started()
        robot.localizer.spin_start(time.monotonic(), 0.0)   # spin_tick이 실제 시계로 시간 초과를 본다
        return robot

    @staticmethod
    def speeds(robot):
        return [c.args[0].angular.z for c in robot.cmd_vel.publish.call_args_list]

    def test_spin_publishes_turn_then_zero_when_found(self):
        robot = self.robot()
        Robot.spin_tick(robot)
        self.assertEqual(self.speeds(robot), [localize.SPIN_SPEED])
        for _ in range(localize.STEADY):
            robot.localizer.on_pose(TIGHT)
        Robot.spin_tick(robot)
        Robot.spin_tick(robot)                 # 이미 멈췄으면 더 보내지 않는다
        self.assertEqual(self.speeds(robot), [localize.SPIN_SPEED, 0.0, 0.0, 0.0])

    def test_stop_button_sends_zero_even_without_nav2_goal(self):
        robot = self.robot()
        Robot.spin_tick(robot)
        rejected = SimpleNamespace(return_code=CancelGoal.Response.ERROR_REJECTED)   # 취소할 Nav2 목표가 없다
        answer = Mock(add_done_callback=lambda cb: cb(None), result=Mock(return_value=rejected))
        robot.cancel = Mock(wait_for_service=Mock(return_value=True), call_async=Mock(return_value=answer))
        message = Robot.command(robot, 'stop', {})
        self.assertIn('제자리 회전 멈춤', message)
        self.assertEqual(self.speeds(robot)[-3:], [0.0, 0.0, 0.0])
        Robot.spin_tick(robot)                 # 멈춘 뒤 회전 속도가 다시 나가지 않는다
        self.assertEqual(self.speeds(robot)[-1], 0.0)
        self.assertEqual(robot.localizer.phase, 'unsure')

    def test_stop_without_spin_still_reports_cancel_errors(self):
        robot = self.robot()
        robot.localizer.manual()
        robot.cancel = Mock(wait_for_service=Mock(return_value=False))
        with self.assertRaises(CommandError) as caught:
            Robot.command(robot, 'stop', {})
        self.assertEqual(caught.exception.code, 'cancel_unavailable')
        robot.cmd_vel.publish.assert_not_called()

    def test_spin_refused_while_navigating_or_offline(self):
        for online, active, code in ((False, False, 'offline'), (True, True, 'nav_active')):
            with self.subTest(code):
                robot = SimpleNamespace(lock=threading.RLock(), localizer=Localizer(), vision_driving=lambda: False,
                                        snapshot=lambda: dict(online=online, nav=dict(active=active)))
                with self.assertRaises(CommandError) as caught:
                    Robot.start_spin(robot)
                self.assertEqual(caught.exception.code, code)

    def test_auto_spin_starts_turning_as_soon_as_search_starts(self):
        for auto, phase in ((True, 'spinning'), (False, 'searching')):
            with self.subTest(auto=auto):
                robot = SimpleNamespace(localizer=Localizer(), auto_spin=auto, odom_yaw=0.0,
                                        vision_driving=lambda: False)
                Robot.on_global_started(robot, None)
                self.assertEqual(robot.localizer.phase, phase)

    def test_pose_hidden_until_found(self):
        from test_fleet import FleetTests
        robot = FleetTests.nav_robot(FleetTests())
        robot.localizer = searching()
        self.assertIsNone(Robot.snapshot(robot)['pose'])
        self.assertEqual(Robot.snapshot(robot)['localize']['phase'], 'searching')
        for _ in range(localize.STEADY):
            robot.localizer.on_pose(TIGHT)
        self.assertEqual(Robot.snapshot(robot)['pose'], {'x': 0})


if __name__ == '__main__':
    unittest.main()
