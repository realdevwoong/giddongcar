"""전역 위치 찾기(AMCL global localization) 진행 판단. DDS 없이 테스트할 수 있게 ROS를 모른다.

흐름: 준비 대기(waiting) → 가만히 찾기(searching) → 확인됨(found)
                                  ↘ 못 찾음(unsure) → 사람이 버튼 → 돌면서 찾기(spinning) → found / unsure
사람이 초기 위치를 찍으면(manual) 그 뒤로는 손대지 않는다. launch가 위치를 알려 줬으면(known) 처음부터 끈다.
"""
import math

XY_STD = 0.20        # m. AMCL 위치 표준편차가 이보다 작아야
YAW_STD = 0.25       # rad(약 14°). 방향 표준편차가 이보다 작아야
STEADY = 3           # 연속으로 기준 안에 들어야 확인(한 번 우연히 모인 것은 믿지 않는다)
QUIET_UPDATES = 10   # 가만히 확인(request_nomotion_update) 횟수. 넘으면 못 찾음
SETTLE_S = 2.0       # AMCL이 active가 되고 지도를 받은 뒤 이만큼 기다렸다가 시작
SPIN_SPEED = 0.5     # rad/s. 제자리 회전 속도
SPIN_TURN = 2 * math.pi   # 한 바퀴 돌면 멈춘다
SPIN_TIMEOUT = 25.0  # s. odom이 안 와서 회전량을 못 재도 이 시간이 지나면 멈춘다

# 이 단계에서만 지도에 로봇을 그린다
SHOWN = ('found', 'manual', 'known')


def spread(covariance):
    """PoseWithCovariance 6x6(행 우선) → (위치 표준편차 m, 방향 표준편차 rad)."""
    return math.sqrt(max(covariance[0], covariance[7], 0.0)), math.sqrt(max(covariance[35], 0.0))


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class Localizer:
    def __init__(self, known=False):
        self.phase = 'known' if known else 'waiting'
        self.steady = 0         # 연속으로 기준 안에 든 amcl_pose 수
        self.quiet = 0          # 보낸 가만히 확인 횟수
        self.spread = None      # 마지막 (위치, 방향) 표준편차
        self.ready_at = None    # AMCL 준비를 처음 본 시각
        self.spin = None        # 회전 {start, yaw, turned}

    @property
    def shows_pose(self):
        return self.phase in SHOWN

    def amcl_ready(self, now):
        """AMCL active + 지도 수신을 봤다. SETTLE_S가 지나면 True(전역 찾기를 시작할 때)."""
        if self.phase != 'waiting':
            return False
        if self.ready_at is None:
            self.ready_at = now
        return now - self.ready_at >= SETTLE_S

    def started(self):
        """전역 찾기 요청이 받아들여졌다: 지도 전체에 후보를 뿌렸다."""
        self.phase, self.steady, self.quiet, self.spread = 'searching', 0, 0, None

    def quiet_tick(self):
        """가만히 확인을 한 번 더 보낼지. 다 썼으면 못 찾음으로 바꾸고 False."""
        if self.phase != 'searching':
            return False
        if self.quiet >= QUIET_UPDATES:
            self.phase = 'unsure'
            return False
        self.quiet += 1
        return True

    def on_pose(self, covariance):
        """amcl_pose를 받았다. 이번에 확인됐으면 True."""
        self.spread = spread(covariance)
        if self.phase not in ('searching', 'spinning', 'unsure'):
            return False
        xy, heading = self.spread
        self.steady = self.steady + 1 if xy < XY_STD and heading < YAW_STD else 0
        if self.steady >= STEADY:
            self.phase, self.spin = 'found', None
            return True
        return False

    def spin_start(self, now, odom_yaw):
        """전역 찾기를 새로 뿌린 뒤 회전을 시작한다."""
        self.phase, self.steady, self.spread = 'spinning', 0, None
        self.spin = dict(start=now, yaw=odom_yaw, turned=0.0)

    def spin_update(self, now, odom_yaw):
        """회전 중 0.1초마다. 계속 돌아야 하면 True. 한 바퀴를 돌았거나 시간이 지나면 못 찾음으로 바꾸고 False."""
        if self.phase != 'spinning':
            return False
        spin = self.spin
        if odom_yaw is not None:
            if spin['yaw'] is not None:
                spin['turned'] += abs(angle_diff(odom_yaw, spin['yaw']))
            spin['yaw'] = odom_yaw
        if spin['turned'] >= SPIN_TURN or now - spin['start'] >= SPIN_TIMEOUT:
            self.phase, self.spin = 'unsure', None
            return False
        return True

    def spin_abort(self):
        """사람이 멈췄다. 회전 중이었으면 True."""
        if self.phase != 'spinning':
            return False
        self.phase, self.spin = 'unsure', None
        return True

    def manual(self):
        """사람이 초기 위치를 찍었다."""
        self.phase, self.spin = 'manual', None

    def snapshot(self):
        spread = self.spread and [round(v, 3) for v in self.spread]
        turned = self.spin and round(self.spin['turned'] / SPIN_TURN, 2)
        return dict(phase=self.phase, spread=spread, quiet=[self.quiet, QUIET_UPDATES], turned=turned)
