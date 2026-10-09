"""관제 ↔ vision_drive: 영상·상태를 받고, 화면에서 본 대기 번호로만 출발 신호를 보낸다."""
import io
import json
import os
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

import pinky_fleet.vision_drive as vd
from pinky_fleet.dashboard_http import handler_for
from pinky_fleet.fleet import Fleet
from pinky_fleet.fleet_common import CommandError
from pinky_fleet.robot import Robot, finite_json

ISOLATED = {'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST', 'ROS_STATIC_PEERS': '',
            'FASTRTPS_DEFAULT_PROFILES_FILE': '', 'ROS_DISCOVERY_SERVER': '',
            'ROS_SUPER_CLIENT': '', 'CYCLONEDDS_URI': ''}


def fake_robot(state=None, age=0.0, listeners=1, image=b'\xff\xd8jpeg'):
    """Robot의 vision 부분만: 받은 상태·영상과 신호 발행기."""
    now = time.monotonic()
    robot = SimpleNamespace(lock=threading.RLock(), robot_name='robot1',
                            vision_state=state, vision_state_at=now - age,
                            vision_jpeg=image, vision_jpeg_at=now - age if image else 0.0,
                            vision_command_pub=Mock(**{'get_subscription_count.return_value': listeners}))
    robot.vision_view = lambda: Robot.vision_view(robot)
    return robot


def sent(robot):
    return [json.loads(call.args[0].data) for call in robot.vision_command_pub.publish.call_args_list]


HELD = dict(mode='drive', policy='횡단보도: 출발 신호 대기 #3', linear=0.0, angular=0.0,
            hold=dict(id=3, reason='crosswalk', text='횡단보도: 출발 신호 대기', age_s=4.0))


def test_go_sends_the_hold_seen_on_screen():
    robot = fake_robot(HELD)
    assert '#3' in Robot.vision_command(robot, 'vision_go', {'hold': 3})
    assert sent(robot) == [{'action': 'go', 'hold': 3}]


@pytest.mark.parametrize('state, age, listeners, body, code', [
    (HELD, 0.0, 1, {'hold': 2}, 'vision_hold_changed'),          # 화면이 늦었다: 지금은 #3
    (HELD, 0.0, 1, {}, 'bad_hold'),
    (HELD, 0.0, 1, {'hold': True}, 'bad_hold'),
    (dict(HELD, hold=None), 0.0, 1, {'hold': 3}, 'vision_not_waiting'),
    (HELD, 5.0, 1, {'hold': 3}, 'vision_offline'),               # 상태가 끊긴 채로는 출발시키지 않는다
    (None, 0.0, 1, {'hold': 3}, 'vision_offline'),
    (HELD, 0.0, 0, {'hold': 3}, 'vision_no_listener'),
])
def test_go_is_refused_without_a_matching_live_hold(state, age, listeners, body, code):
    robot = fake_robot(state, age, listeners)
    with pytest.raises(CommandError) as error:
        Robot.vision_command(robot, 'vision_go', body)
    assert error.value.code == code and not sent(robot)


def test_stop_needs_only_a_listener():
    robot = fake_robot(None)
    assert '정지' in Robot.vision_command(robot, 'vision_stop', {})
    assert sent(robot) == [{'action': 'stop'}]
    with pytest.raises(CommandError):
        Robot.vision_command(fake_robot(None, listeners=0), 'vision_stop', {})


def test_view_hides_hold_and_image_when_stale():
    assert Robot.vision_view(fake_robot(None)) is None            # 소식이 없던 로봇은 화면에 안 그린다
    live = Robot.vision_view(fake_robot(HELD))
    assert live['online'] and live['hold']['id'] == 3 and live['image'] == '/vision/robot1.jpg'
    stale = Robot.vision_view(fake_robot(HELD, age=5.0))
    assert not stale['online'] and stale['hold'] is None and stale['image'] is None


def test_state_with_nan_stays_valid_json():
    robot = fake_robot(None)
    Robot.on_vision_state(robot, String(data='{"lane_error": NaN, "hold": null, "x": [Infinity, 1.5]}'))
    assert robot.vision_state == {'lane_error': None, 'hold': None, 'x': [None, 1.5]}
    json.dumps(Robot.vision_view(robot), allow_nan=False)
    Robot.on_vision_state(robot, String(data='not json'))           # 깨진 메시지는 버린다
    assert finite_json({'a': float('inf')}) == {'a': None}


class FleetRobot:
    def __init__(self):
        self.vision_command = Mock(return_value='출발 신호 보냄 (대기 #3)')
        self.command = Mock()
        self.lock = threading.RLock()
        self.map_data = None                                       # 지도 없음(Nav2 없이 띄운 관제)
        self.vision_image = Mock(return_value=b'\xff\xd8jpeg')

    def snapshot(self):
        return dict(id='robot1', map_id=None)


def test_fleet_routes_vision_signals_without_map_or_nav2():
    robot = FleetRobot()
    fleet = Fleet(dict(robot1=robot, robot2=FleetRobot()))
    assert fleet.command('robot1', 'vision_go', {'hold': 3}) == '출발 신호 보냄 (대기 #3)'
    robot.vision_command.assert_called_once_with('vision_go', {'hold': 3})
    robot.command.assert_not_called()


def get(fleet, path):
    handler = object.__new__(handler_for(fleet))
    handler.path = path
    handler.send = Mock()
    handler.do_GET()
    return handler.send.call_args.args


def test_http_serves_vision_image():
    robot = FleetRobot()
    fleet = Fleet(dict(robot1=robot, robot2=FleetRobot()))
    assert get(fleet, '/vision/robot1.jpg') == (200, b'\xff\xd8jpeg', 'image/jpeg')
    robot.vision_image.return_value = None
    assert get(fleet, '/vision/robot1.jpg')[0] == 503
    assert get(fleet, '/vision/robot9.jpg')[0] == 503


def test_dashboard_robot_has_no_cmd_vel_until_it_spins():
    """vision_drive는 다른 cmd_vel 발행자가 있으면 주행을 거부한다. 지켜보기만 하는 관제가 막으면 안 된다."""
    with patch.dict(os.environ, ISOLATED):
        robot = Robot('probe', 79)
    try:
        assert robot.cmd_vel is None
        assert not robot.get_publishers_info_by_topic('/cmd_vel')
        robot.send_zero()                                          # 보낸 적이 없으면 아무것도 안 한다
        with robot.lock:
            robot.velocity()
        assert len(robot.get_publishers_info_by_topic('/cmd_vel')) == 1
    finally:
        robot.close()


def test_dashboard_and_vision_drive_talk_over_ros():
    """실제 노드끼리: vision_drive 시작 대기 #1이 관제에 보이고, 관제 ▶ 출발로 풀린다."""
    with patch.dict(os.environ, ISOLATED):
        robot = Robot('robot1', 80)
        context = Context()
        rclpy.init(context=context, domain_id=80)
    node = None
    try:
        node = vd.VisionDriveNode('drive', context=context)      # 주행 모드는 시작 대기 #1
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        deadline = time.monotonic() + 10.0
        view = None
        while time.monotonic() < deadline:
            node.publish_state(dict(mode='drive', policy=vd._hold_reason(node.hold) if node.hold else '차선 영역 추종',
                                    linear=0.0, angular=0.0,
                                    hold=node.hold and dict(id=node.hold['id'], reason=node.hold['reason'],
                                                            text=vd.HOLD_TEXT[node.hold['reason']], age_s=0.1)))
            node.publish_overlay(np.zeros((240, 320, 3), np.uint8))
            executor.spin_once(timeout_sec=0.05)
            view = robot.vision_view()
            if view and view['hold'] and view['image'] and robot.vision_command_pub.get_subscription_count():
                break
        assert view['online'] and view['hold']['id'] == 1 and robot.vision_image()[:2] == b'\xff\xd8'
        robot.vision_command('vision_go', {'hold': 1})
        while node.hold is not None and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert node.hold is None
    finally:
        if node is not None:
            node.destroy_node()
        context.try_shutdown()
        robot.close()


def test_no_dashboard_spin_while_vision_drive_runs():
    """vision_drive가 켜져 있으면 관제는 로봇을 돌리지 않는다(버튼·자동 위치 찾기 모두)."""
    from pinky_fleet.localize import Localizer
    robot = SimpleNamespace(vision_driving=lambda: True)
    with pytest.raises(CommandError) as error:
        Robot.start_spin(robot)
    assert error.value.code == 'vision_active'
    quiet = SimpleNamespace(localizer=Localizer(), auto_spin=True, odom_yaw=0.0, vision_driving=lambda: True)
    quiet.localizer.amcl_ready(0.0)
    Robot.on_global_started(quiet, None)
    assert quiet.localizer.phase == 'searching'                    # 돌지 않고 가만히 찾는다


def test_spin_hands_over_and_releases_cmd_vel():
    """처음 위치 찾기 회전 중에 vision_drive가 켜지면 멈추고, 끝난 뒤 cmd_vel 발행자를 지운다."""
    with patch.dict(os.environ, ISOLATED):
        robot = Robot('probe', 81, auto_spin=True)
    try:
        with robot.lock:
            robot.localizer.amcl_ready(0.0)
            robot.localizer.spin_start(time.monotonic(), 0.0)
        robot.spin_tick()
        assert robot.cmd_vel is not None and robot.driving             # 회전 중: 관제가 cmd_vel을 쥔다
        robot.vision_state, robot.vision_state_at = dict(mode='drive', hold=None), time.monotonic()
        robot.spin_tick()                                              # vision_drive가 켜졌다 → 멈추고 0 속도
        assert not robot.driving and robot.localizer.phase == 'unsure' and robot.release_at is not None
        robot.release_at = time.monotonic() - 0.1                      # 2초가 지났다고 치고
        robot.spin_tick()
        assert robot.cmd_vel is None                                   # 발행자를 지워 vision_drive가 몰 수 있다
        deadline = time.monotonic() + 3.0
        while robot.get_publishers_info_by_topic('/cmd_vel') and time.monotonic() < deadline:
            time.sleep(0.05)
        assert not robot.get_publishers_info_by_topic('/cmd_vel')
    finally:
        robot.close()
