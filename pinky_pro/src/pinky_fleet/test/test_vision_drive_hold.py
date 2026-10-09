"""vision_drive 출발 신호: 시작·횡단보도·정지 요청은 멈춘 채 ▶ 출발(대기 번호)을 기다린다."""
import json
import math
import os
import time
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

import pinky_fleet.vision_drive as vd

MASK = np.zeros((240, 320), np.uint8)
CROSSWALK = ('crosswalk', 0.8, 100.0, 120.0, 220.0, 180.0)   # 화면 아래 가운데: 진행 경로 위
ISOLATED = {'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST', 'ROS_STATIC_PEERS': '',
            'FASTRTPS_DEFAULT_PROFILES_FILE': '', 'ROS_DISCOVERY_SERVER': '',
            'ROS_SUPER_CLIENT': '', 'CYCLONEDDS_URI': ''}


def go(step, hold_id):
    return vd._command(step.node, json.dumps({'action': 'go', 'hold': hold_id}), step.state.now)


def stop(step):
    return vd._command(step.node, json.dumps({'action': 'stop'}), step.state.now)


@pytest.fixture
def drive(monkeypatch):
    """가짜 시계로 _policy를 한 프레임씩 돌린다. 차선은 기본으로 정면."""
    state = SimpleNamespace(now=100.0, error=0.0)
    monkeypatch.setattr(vd, 'time', SimpleNamespace(monotonic=lambda: state.now))
    monkeypatch.setattr(vd, '_lane_error', lambda mask: state.error)
    args = vd.parse_args(['--robot-ip', '192.0.2.1', '--model', 'm.pt', '--mode', 'drive'])
    node = SimpleNamespace(front_range=lambda min_width: math.inf, front_clear=lambda: math.inf,
                           wall_ahead=lambda distance: False, side_clearance=lambda: None)

    def step(*detections, dt=0.1, error=0.0):
        state.now += dt
        state.error = error
        return vd._policy(node, MASK, list(detections), args, state.now, 0.0)
    step.node, step.args, step.state = node, args, state
    return step


def test_default_crosswalk_action_waits_for_signal():
    args = vd.parse_args(['--robot-ip', '192.0.2.1', '--model', 'm.pt'])
    assert args.crosswalk_action == 'wait-signal'


def test_crosswalk_holds_until_go_with_matching_id(drive):
    assert drive()[0] > 0
    assert drive(CROSSWALK)[0] > 0                         # 한 프레임은 오검출일 수 있다
    linear, angular, reason = drive(CROSSWALK)
    assert (linear, angular) == (0.0, 0.0) and reason == '횡단보도: 출발 신호 대기 #1'
    # 10초 대기 방식은 감지가 끊긴 프레임마다 기어갔다(16:52 로그). 신호 전에는 계속 정지
    assert drive()[:2] == (0.0, 0.0)
    assert drive(dt=30.0)[:2] == (0.0, 0.0)
    assert not go(drive, 2)                                # 화면에서 본 대기가 아니면 무시
    assert drive(CROSSWALK)[:2] == (0.0, 0.0)
    assert go(drive, 1)
    linear, _, reason = drive(CROSSWALK)
    assert linear > 0 and reason == '횡단보도 통과'


def test_crossing_dropouts_keep_going_and_next_crosswalk_holds_again(drive):
    drive(CROSSWALK)
    drive(CROSSWALK)
    assert go(drive, 1)
    assert drive()[0] > 0                                  # 건너는 중 감지가 끊겼다가
    assert drive(dt=1.5)[0] > 0
    assert drive(CROSSWALK)[0] > 0                         # 2초 안에 다시 보이면 같은 횡단보도
    assert drive(CROSSWALK)[0] > 0
    assert drive(dt=2.5)[0] > 0                            # 2초 넘게 안 보이면 지나간 것
    drive(CROSSWALK)
    linear, _, reason = drive(CROSSWALK)
    assert linear == 0.0 and reason == '횡단보도: 출발 신호 대기 #2'


def test_stop_request_holds_until_go(drive):
    assert drive()[0] > 0
    assert stop(drive)
    linear, _, reason = drive()
    assert linear == 0.0 and reason == '정지: 출발 신호 대기 #1'
    assert stop(drive) and drive.node.hold['id'] == 1      # 이미 정지: 번호 그대로
    assert go(drive, 1)
    assert drive()[0] > 0


def test_go_without_hold_or_with_bad_message_is_ignored(drive):
    assert not go(drive, 1)
    for text in ('', 'go', '[]', '{"hold": 1}', '{"action": "go"}', '{"action": "go", "hold": "1"}',
                 '{"action": "go", "hold": true}', '{"action": "jump"}'):
        assert vd._command(drive.node, text, drive.state.now) is False, text
    assert getattr(drive.node, 'hold', None) is None


def test_corner_pivot_timer_pauses_while_held(drive):
    drive(error=0.1)
    assert drive(error=None)[1] == pytest.approx(-0.40)    # 차선 잃음: 오른쪽 회전 시작
    stop(drive)
    assert drive(error=None, dt=30.0)[:2] == (0.0, 0.0)    # 대기 중에는 돌지 않는다
    assert go(drive, 1)
    _, angular, reason = drive(error=None)                 # 기다린 시간은 회전 시간에 넣지 않는다
    assert angular == pytest.approx(-0.40) and '예상 방향' in reason


def test_stop_then_go_still_waits_fixed_time(drive):
    drive.args.crosswalk_action = 'stop-then-go'
    assert drive(CROSSWALK)[2].startswith('횡단보도 대기')
    assert drive(CROSSWALK, dt=5.0)[0] == 0.0
    linear, _, reason = drive(CROSSWALK, dt=5.1)
    assert linear > 0 and reason == '횡단보도 대기 완료'


def test_overlay_status_shows_hold_in_ascii():
    text = vd._overlay_status((0.0, 0.0, '횡단보도: 출발 신호 대기 #3'), dict(id=3, reason='crosswalk', since=0.0))
    assert text.isascii() and '#3' in text and 'CROSSWALK' in text


def test_drive_node_starts_held_and_takes_go_over_ros():
    """관제가 보내는 것과 같은 토픽·QoS로: 상태와 영상을 받고, 출발 신호로 시작 대기를 푼다."""
    with patch.dict(os.environ, ISOLATED):
        rclpy.init(domain_id=78)
    node = peer = observer = None
    try:
        node = vd.VisionDriveNode('drive')
        assert node.hold['reason'] == 'start' and node.hold['id'] == 1
        observer = vd.VisionDriveNode('observe')
        assert observer.hold is None                        # 관찰 모드는 움직이지 않으니 대기 없음
        peer = rclpy.create_node('vision_dashboard_peer')
        states, images = [], []
        peer.create_subscription(String, 'vision_drive/state', lambda m: states.append(json.loads(m.data)),
                                 qos_profile_sensor_data)
        peer.create_subscription(CompressedImage, 'vision_drive/overlay/compressed', images.append,
                                 qos_profile_sensor_data)
        command = peer.create_publisher(String, 'vision_drive/command', 10)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(peer)
        deadline = time.monotonic() + 10.0
        sent = False
        while (node.hold is not None or not states or not images) and time.monotonic() < deadline:
            if not sent and command.get_subscription_count():
                command.publish(String(data=json.dumps({'action': 'go', 'hold': 1})))
                sent = True
            node.publish_state(dict(policy='시험', hold=None))
            node.publish_overlay(np.zeros((240, 320, 3), np.uint8))
            executor.spin_once(timeout_sec=0.05)
        assert node.hold is None
        assert states and states[-1]['policy'] == '시험'
        assert images and images[-1].format == 'jpeg' and bytes(images[-1].data[:2]) == b'\xff\xd8'
    finally:
        for item in (node, peer, observer):
            if item is not None:
                item.destroy_node()
        rclpy.shutdown()


def test_first_crosswalk_frame_is_not_called_crossing(drive):
    linear, _, reason = drive(CROSSWALK)                   # 아직 대기 전 한 프레임: '통과'라고 하지 않는다
    assert linear > 0 and reason != '횡단보도 통과'


def test_state_with_nan_is_skipped_not_raised():
    sent = []
    node = SimpleNamespace(state_pub=SimpleNamespace(publish=sent.append))
    vd.VisionDriveNode.publish_state(node, dict(lane_error=float('nan')))
    vd.VisionDriveNode.publish_state(node, dict(lane_error=0.1))
    assert len(sent) == 1 and json.loads(sent[0].data) == dict(lane_error=0.1)
