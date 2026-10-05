"""ROS 2 state and command handling for one Pinky robot."""
import hashlib
import json
import math
import threading
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Path as NavPath, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Vector3
from nav2_msgs.action import BackUp, NavigateToPose
from pinky_interfaces.srv import SetLamp
from pinky_fleet.localize import Localizer, SPIN_SPEED
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory
from pinky_fleet.fleet_common import (
    LAMP, LAMP_HOLD, LAMP_TIMEOUT, LOC_TIMEOUT, NAV_ERRORS, NAV_STATES, NO_REASON,
    CommandError, await_future, remember, seconds, yaw,
)

class Robot(Node):
    def __init__(self, name, domain, use_sim_time=False, known_pose=False, auto_spin=False):
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
        # 전역 위치 찾기. launch가 초기 위치를 알려 준 로봇(시뮬)은 끈다
        self.localizer = Localizer(known=known_pose)
        self.auto_spin = auto_spin   # 전역 찾기를 시작하자마자 제자리에서 한 바퀴 돈다(사람 확인 없이)
        self.odom_yaw = None
        self.amcl_active = False  # AMCL lifecycle이 active인 걸 봤다
        self.loc_future, self.loc_sent = None, 0.0   # 아직 답을 기다리는 AMCL 요청 (future, client), 보낸 시각
        self.driving = False      # 제자리 회전 속도를 보내는 중
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
        self.backup = ActionClient(self, BackUp, 'backup')   # Nav2 behavior_server 후진(충돌 검사 포함)
        self.backup_handle = None
        self.lamp_client = self.create_client(SetLamp, 'set_lamp')
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_amcl_pose, transient)
        self.amcl_state = self.create_client(GetState, 'amcl/get_state')
        self.global_loc = self.create_client(Empty, 'reinitialize_global_localization')
        self.nomotion = self.create_client(Empty, 'request_nomotion_update')
        self.cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(0.1, self.update_pose)
        self.create_timer(0.1, self.spin_tick)
        self.create_timer(1.0, self.check_lamp)
        self.create_timer(1.0, self.localize_tick)
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
            self.odom_yaw = yaw(msg.pose.pose.orientation)   # 제자리 회전량을 잰다

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

    def on_amcl_pose(self, msg):
        with self.lock:
            self.localizer.on_pose(msg.pose.covariance)
        # 돌던 중 확인됐으면 spin_tick이 다음 0.1초에 0 속도를 보낸다

    def localize_tick(self):
        """1초마다: AMCL이 켜지면 전역 찾기를 시작한다. auto_spin이면 바로 한 바퀴 돌고,
        아니면 가만히 확인을 보낸다. 답을 기다리지 않는다."""
        now = time.monotonic()
        with self.lock:
            phase, has_map = self.localizer.phase, self.map_data is not None
            online = now - self.last_odom < 2   # 돌려면 odom으로 회전량을 재야 한다
            future, client = self.loc_future or (None, None)
        if future is not None and not future.done():
            if now - self.loc_sent < LOC_TIMEOUT:
                return
            client.remove_pending_request(future)   # 답이 끝내 안 왔다. 버리고 다시 한다
        if phase == 'waiting' and has_map and not self.amcl_active:
            # AMCL이 active가 되기 전에 전역 찾기를 부르면 지도 없이 후보를 뿌린다. 상태부터 본다
            if self.amcl_state.service_is_ready():
                self.call_amcl(self.amcl_state, GetState.Request(), self.on_amcl_state)
        elif phase == 'waiting' and has_map:
            with self.lock:
                start = self.localizer.amcl_ready(now)
            if start and self.global_loc.service_is_ready() and (online or not self.auto_spin):
                self.call_amcl(self.global_loc, Empty.Request(), self.on_global_started)
        elif phase == 'searching' and self.nomotion.service_is_ready():
            with self.lock:
                again = self.localizer.quiet_tick()
            if again:
                self.call_amcl(self.nomotion, Empty.Request())

    def call_amcl(self, client, request, on_done=None):
        future = client.call_async(request)
        with self.lock:
            self.loc_future, self.loc_sent = (future, client), time.monotonic()

        def done(f):
            if f.exception() is None and on_done:
                with self.lock:
                    on_done(f.result())
        future.add_done_callback(done)

    def on_global_started(self, _):
        # 잠금 안에서 불린다(call_amcl). 회전 속도는 spin_tick이 0.1초 안에 보내기 시작한다
        if self.auto_spin:
            self.localizer.spin_start(time.monotonic(), self.odom_yaw)
        else:
            self.localizer.started()

    def on_amcl_state(self, response):
        self.amcl_active = response.current_state.id == State.PRIMARY_STATE_ACTIVE

    def spin_tick(self):
        """0.1초마다: 돌면서 찾는 중이면 회전 속도를, 방금 끝났으면 0 속도를 보낸다.
        pinky_bringup은 cmd_vel이 끊겨도 마지막 속도로 계속 달리므로 끝날 때 반드시 0을 보낸다."""
        with self.lock:
            turning = self.localizer.spin_update(time.monotonic(), self.odom_yaw)
            was, self.driving = self.driving, turning
            # 잠금 안에서 보낸다: stop_spin이 0을 보낸 뒤에 회전 속도가 끼어들지 않게
            if turning:
                self.cmd_vel.publish(Twist(angular=Vector3(z=SPIN_SPEED)))
            elif was:
                self.send_zero()

    def stop_spin(self):
        """사람이 멈추거나 초기 위치를 찍거나 관제를 끌 때. 돌던 중이었으면 True."""
        with self.lock:
            spun = self.localizer.spin_abort()
            if spun or self.driving:
                self.driving = False
                self.send_zero()
        return spun

    def send_zero(self):
        for _ in range(3):   # Wi-Fi에서 하나가 늦어도 멈추게
            self.cmd_vel.publish(Twist())

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
            if unseen:
                # 취소된 예전 목표의 상태가 새 목표 상태보다 늦게 도착할 수 있다.
                # 방금 수락된 목표를 기준으로 보여 줘야 재개 직후 '취소됨'으로 깜빡이지 않는다.
                goal_id = self.sent['id']
                state, label = 'accepted', NAV_STATES[1][1]
                result = self.results.get(goal_id, {})
            nav = dict(id=goal_id and goal_id[:8], state=state, label=label,
                       active=bool(state in ('accepted', 'executing', 'canceling') or unseen),
                       target=self.targets.get(goal_id), feedback=self.goal['feedback'],
                       error=result.get('reason') if state == 'aborted' else None,
                       error_code=result.get('code'))
            # 전역 찾기 중에는 AMCL이 아직 엉뚱한 곳을 가리킬 수 있어 그리지 않는다(목적지도 못 보낸다)
            shown = online and self.localizer.shows_pose
            return dict(id=self.robot_name, domain=self.domain, online=online,
                        pose=self.pose if shown else None, map_id=self.map_id,
                        localize=self.localizer.snapshot(),
                        path=self.path if online else [], status=label, nav=nav, lamp=self.lamp,
                        clock_skew=round(getattr(self, 'clock_skew', 0.0), 2) if online else None,
                        nav_ready=self.navigator.server_is_ready())

    def command(self, action, body):
        # Serialize HTTP requests per robot; the ROS executor remains independent.
        with self.command_lock:
            if action == 'stop':
                spun = self.stop_spin()
                with self.lock:
                    backing, self.backup_handle = self.backup_handle, None
                if backing:   # 양보하느라 물러나던 중이면 그것도 멈춘다
                    backing.cancel_goal_async()
                try:
                    if not self.cancel.wait_for_service(timeout_sec=1):
                        raise CommandError('Nav2 취소 서비스가 준비되지 않았습니다.', 'cancel_unavailable')
                    response = await_future(self.cancel.call_async(CancelGoal.Request()))
                    if response.return_code != CancelGoal.Response.ERROR_NONE:
                        raise CommandError('Nav2가 취소 요청을 수락하지 않았습니다.', 'cancel_rejected')
                except CommandError:
                    if spun:   # 돌던 것만 멈췄다(Nav2 목표는 없었다)
                        return '제자리 회전 멈춤 — 0 속도 보냄'
                    raise
                return '이동 취소 요청 수락' + (' · 제자리 회전 멈춤' if spun else '')
            if action == 'spin':
                return self.start_spin()
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
                self.stop_spin()
                self.initial.publish(msg)
                with self.lock:
                    self.localizer.manual()   # 사람이 찍은 위치를 믿는다. 전역 찾기는 그만둔다
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

    def back_up(self, distance, speed):
        """Nav2 후진 동작으로 distance m 물러난다(뒤에 장애물이 있으면 Nav2가 멈춘다). 결과 future를 돌려준다.
        Nav2 목표는 먼저 취소해 둬야 한다(둘 다 cmd_vel을 보낸다)."""
        if not self.backup.wait_for_server(timeout_sec=1):
            raise CommandError('Nav2 후진 동작이 준비되지 않았습니다.', 'backup_unavailable')
        goal = BackUp.Goal()
        goal.target.x, goal.speed = float(distance), float(speed)
        goal.time_allowance.sec = int(distance / speed) + 5
        handle = await_future(self.backup.send_goal_async(goal))
        if not handle.accepted:
            raise CommandError('Nav2가 후진을 거부했습니다.', 'backup_rejected')
        with self.lock:
            self.backup_handle = handle
        return handle.get_result_async()

    def start_spin(self):
        """전역 찾기를 새로 뿌리고 제자리에서 한 바퀴 돈다. 사람이 로봇 주변을 보고 누르는 버튼에서만 부른다."""
        state = self.snapshot()
        if not state['online']:
            raise CommandError('로봇 연결이 끊겨 있어요.', 'offline')
        if state['nav']['active']:
            raise CommandError('이동 중에는 돌 수 없어요. ■ 이동 취소 뒤 다시 누르세요.', 'nav_active')
        with self.lock:
            phase = self.localizer.phase
        if phase == 'spinning':
            return '이미 돌면서 찾는 중이에요.'
        if phase == 'waiting' or not self.global_loc.wait_for_service(timeout_sec=1):
            raise CommandError('AMCL이 아직 준비되지 않았습니다.', 'amcl_not_ready')
        # 앞에서 한 곳으로 잘못 모였을 수 있어 후보를 지도 전체에 다시 뿌린 뒤 돈다
        await_future(self.global_loc.call_async(Empty.Request()))
        with self.lock:
            self.localizer.spin_start(time.monotonic(), self.odom_yaw)
        return '돌면서 위치 찾기 시작 — 한 바퀴 돌아요. 멈추려면 ■ 이동 취소'

    def close(self):
        # 돌던 중에 관제를 꺼도 로봇이 계속 돌지 않게 0 속도부터 보낸다
        try:
            self.stop_spin()
        except Exception:
            pass
        # launch는 SIGINT 후 5초면 SIGTERM으로 올리므로 로봇 2대 합쳐 그 안에 끝나야 한다.
        self.ros_executor.shutdown(timeout_sec=1)
        self.thread.join(timeout=1)
        self.destroy_node()
        self.ros_context.try_shutdown()

