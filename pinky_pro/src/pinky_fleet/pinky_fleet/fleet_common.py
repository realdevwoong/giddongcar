"""Shared dashboard constants and small value helpers."""
import math
import threading



# action_msgs/GoalStatus 번호 → (상태 키, 화면 글자). 0(UNKNOWN)이거나 목표가 없으면 대기.
NAV_STATES = {1: ('accepted', '목표 수락'), 2: ('executing', '이동 중'), 3: ('canceling', '취소 중'),
              4: ('succeeded', '도착'), 5: ('canceled', '취소됨'), 6: ('aborted', '이동 실패')}
# NavigateToPose 결과 error_code (nav2_msgs FollowPath 1xx, ComputePathToPose 2xx)
# 104: RPP "collision ahead" 같은 제어 실패가 controller_server의 failure_tolerance(0.3 s)보다 오래 이어질 때
NAV_ERRORS = {100: '경로 추종 중 알 수 없는 오류', 101: '경로 추종기 설정 오류', 102: '위치 변환(TF) 실패',
              103: '경로가 잘못됨', 104: '제어 실패가 계속됨 — 앞이 막혔을 수 있음(collision ahead 등)',
              105: '진전 없음 — 막혀서 못 움직임', 106: '앞에 장애물 — 안전한 속도를 못 찾음',
              107: '경로 추종 시간 초과', 200: '경로 계획 중 알 수 없는 오류', 201: '경로 계획기 설정 오류', 202: '위치 변환(TF) 실패',
              203: '출발 위치가 지도 밖', 204: '목적지가 지도 밖', 205: '출발 위치가 장애물 위',
              206: '목적지가 장애물 위', 207: '경로 계획 시간 초과', 208: '갈 수 있는 경로 없음'}
# 복구 동작을 다 쓰고 실패하면 error_code 0으로 끝나기도 한다. 그래도 이유 칸은 비우지 않는다.
NO_REASON = '이유 코드 없음 (Nav2 복구를 다 써도 실패)'


# 목표 상태 → 로봇 램프 (mode, (r, g, b), time ms, 화면 글자). 실물 pinky_lamp_control과 시뮬 sim_lamp가 같은 set_lamp로 받는다.
# mode: 1 켜기, 2 깜빡임, 3 숨쉬기
LAMP = {'idle': (3, (1.0, 1.0, 1.0), 1000, '흰색 숨쉬기'),
        'accepted': (2, (0.0, 0.4, 1.0), 500, '파랑 깜빡임'),
        'executing': (2, (0.0, 0.4, 1.0), 500, '파랑 깜빡임'),
        'canceling': (1, (1.0, 0.7, 0.0), 0, '노랑'),
        'canceled': (1, (1.0, 0.7, 0.0), 0, '노랑'),
        'succeeded': (1, (0.0, 1.0, 0.2), 0, '초록'),
        'aborted': (2, (1.0, 0.0, 0.0), 250, '빨강 빠른 깜빡임')}
LAMP_HOLD = 5.0     # 도착·취소 색을 보여 주는 시간(초). 그다음 대기로. 실패 빨강은 다음 목표까지 둔다(경보)
LAMP_TIMEOUT = 3.0  # 램프 서비스가 이 시간(초) 안에 답이 없으면 실패로 보고 다시 보낸다
LOC_TIMEOUT = 5.0   # AMCL 서비스(상태 확인·전역 찾기·가만히 확인)가 이 시간(초) 안에 답이 없으면 버리고 다시 한다

# 달리면서 양보: 겹치면 LEADER가 먼저 가고 FOLLOWER가 멈추거나 물러난다(사용자가 정한 고정 순위)
LEADER, FOLLOWER = 'robot1', 'robot2'
BACK_STEP = 0.15    # 한 번에 물러나는 거리 [m]
BACK_SPEED = 0.08   # 물러나는 속도 [m/s]
BACK_LIMIT = 3      # 이만큼 연달아 물러나도 길 위면 더 물러나지 않고 멈춰 기다린다(벽까지 계속 가지 않게)
# 막혀서 실패한 목표는 같은 목적지로 다시 보낸다. 좁은 문에서 RPP 충돌 검사가 문틀에 걸려 104로 끝나도
# 다시 보내면 지나가는 경우가 많다. 0 = 복구를 다 쓰고 이유 없이 끝남
RETRY_CODES = {0, 104, 105, 106}
RETRY_LIMIT = 3



def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class CommandError(ValueError):
    """명령 실패. code는 화면이 글자 대신 보고 판단하는 이름이다(예: 'map_mismatch')."""
    def __init__(self, message, code=None):
        super().__init__(message)
        self.code = code


def seconds(duration):
    return round(duration.sec + duration.nanosec / 1e9, 1)


def remember(table, key, value, keep=20):
    table[key] = value
    while len(table) > keep:
        table.pop(next(iter(table)))


def pose_input(body):
    if not isinstance(body, dict):
        raise CommandError('Expected a JSON object', 'bad_pose')
    try:
        values = [float(body[k]) for k in ('x', 'y', 'yaw')]
    except (KeyError, TypeError, ValueError):
        raise CommandError('x, y, yaw 숫자가 필요합니다.', 'bad_pose') from None
    if not all(math.isfinite(v) for v in values):
        raise CommandError('Coordinates must be finite', 'bad_pose')
    return values


def yield_view(snapshot):
    """Robot.snapshot() → 양보 규칙이 보는 {pose (x, y), active, path [(x, y), ...]}."""
    pose = snapshot['pose']
    return dict(pose=(pose['x'], pose['y']) if pose else None, active=snapshot['nav']['active'],
                path=[(p['x'], p['y']) for p in snapshot.get('path') or []])


def await_future(future, timeout=4):
    ready = threading.Event()
    future.add_done_callback(lambda _: ready.set())
    if not ready.wait(timeout):
        # Do not retry automatically: a timed-out request may still reach Nav2.
        raise TimeoutError('응답 시간 초과: 요청이 처리됐을 수 있습니다. 상태를 확인하세요.')
    return future.result()


