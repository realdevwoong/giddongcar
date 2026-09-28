# 교통 정리 직접 만들기 (힌트 + 정답)

목표: 좁은 문(x≈1.6, 폭 0.35 m)에서 두 로봇이 정면으로 만나 둘 다 실패하는 문제를 없앤다.
단계마다 **먼저 스스로 해 보고 → 막히면 힌트를 하나씩 → 마지막에 정답**을 연다.

## 전체 계획

방식은 **문 통과 순서표**다. 문 열쇠가 하나뿐이라고 생각하면 된다. 열쇠를 가진 로봇만 문을 지나가고, 다른 로봇은 열쇠가 돌아올 때까지 **출발하지 않는다**(Nav2에 목표를 늦게 보낸다). 달리는 로봇을 세우는 명령을 쓰지 않아서 1단계가 가장 안전하다.

| 단계 | 만들 것 | 로봇 동작 변화 |
|---|---|---|
| **1-1** | `traffic.py` 기하 계산 + 구역 YAML + 테스트 | 없음 |
| 1-2 | `DoorGate`: 누가 열쇠를 갖나, 언제 돌려주나 (순수 로직 + 테스트) | 없음 |
| 1-3 | 대시보드에 연결: 문을 건너는 목표를 줄 세워 보내기 | **있음** (시뮬에서 정면 대결 확인) |
| 2~ | 화면 표시, 틀린 지도에서 끄기, RViz 목표 막기, 비켜 서기, 경보 | 차례로 |

---

## 1-1. 구역 계산

### 무엇을 만드나

"로봇이 지금 있는 곳에서 목적지로 가려면 **문을 지나야 하나?**"를 계산한다.

good3 방은 문 하나로만 이어진 두 공간이다. 그래서 경로를 계산하지 않아도 이렇게 판단할 수 있다.

> **로봇이 있는 쪽 ≠ 목적지가 있는 쪽 → 문을 지난다**

(지도로 확인했다: 빈 칸 1,501개가 왼쪽 805개, 오른쪽 696개로 빠짐없이 나뉘고, 문 구역을 막으면 왼쪽에서 오른쪽으로 갈 수 없다.)

파일은 만들어 두었다. **채울 곳은 `traffic.py`의 `TODO`뿐이다.**

| 파일 | 상태 |
|---|---|
| `pinky_pro/src/pinky_fleet/params/traffic_good3.yaml` | 완성. 쪽(`sides`) 2개와 구역(`zones`) 1개(문)의 좌표 |
| `pinky_pro/src/pinky_fleet/pinky_fleet/traffic.py` | **빈칸.** 함수 이름·설명·TODO만 있고 `raise NotImplementedError`. ROS를 import하지 않는다 |
| `pinky_pro/src/pinky_fleet/test/test_traffic.py` | 완성(목표). 맨 위 `pytestmark = ...skip...` 한 줄 때문에 지금은 건너뛴다. **시작할 때 그 줄을 지운다** |

만들 함수:

| 함수 | 돌려주는 것 |
|---|---|
| `point_in_polygon(x, y, polygon)` | 점이 다각형 안이면 `True` |
| `distance_to_polygon(x, y, polygon)` | 다각형까지 거리[m], 안이면 `0.0` (1-3에서 "문에 너무 가까운 목적지" 거절에 씀) |
| `load_zones(path)` | YAML을 읽은 dict. 구역의 `between`이 모르는 쪽이면 `ValueError` |
| `side_of(x, y, config)` | `'left'` / `'right'` / 어느 쪽도 아니면 `None` |
| `zones_to_cross(start, goal, config)` | 지나야 하는 구역 id 목록. 예: `['door']`, 같은 쪽이면 `[]` |

좌표 (map 프레임, m):
- 왼쪽 방: x -0.2 ~ 1.585, y -0.2 ~ 1.45
- 오른쪽 구역: x 1.585 ~ 3.2, y -0.2 ~ 1.45
- 문 구역: x 1.35 ~ 1.85, y 0.80 ~ 1.28

<details>
<summary>힌트 1 — 어떻게 나누어 생각하나</summary>

- 다각형은 `[[x1, y1], [x2, y2], ...]` 점 목록이다. 사각형도 점 4개짜리 다각형으로 쓴다.
- `side_of`는 `sides`의 다각형을 하나씩 보며 `point_in_polygon`이 참인 이름을 돌려준다.
- `zones_to_cross`는 `side_of`를 두 번(출발, 목적지) 부르고, 둘이 다르면 `between`이 그 두 쪽인 구역을 찾는다.
- 먼저 테스트부터 쓰면 쉽다. 예: `side_of(0.5, 0.5)`는 `'left'`(robot1 생성 위치), `side_of(2.0, 0.5)`는 `'right'`.
</details>

<details>
<summary>힌트 2 — 점이 다각형 안인지 (광선 투사)</summary>

점에서 오른쪽으로 가로선을 쭉 긋는다. 이 선이 다각형의 변을 **홀수 번** 넘으면 안, 짝수 번이면 밖이다.

변 `(x1, y1) → (x2, y2)`마다:
1. 이 변이 점의 높이 `y`를 가로지르나? → `(y1 > y) != (y2 > y)`
2. 가로지르면 그 높이에서 변의 x는 `x1 + (y - y1) * (x2 - x1) / (y2 - y1)`
3. 점의 x가 그보다 작으면(점 오른쪽에서 만나면) `inside = not inside`

마지막 점과 첫 점을 잇는 변도 빼먹지 말 것: `zip(polygon, polygon[1:] + polygon[:1])`
</details>

<details>
<summary>힌트 3 — 다각형까지 거리와 YAML 검사</summary>

- 안이면 0. 밖이면 **변(선분)마다 거리**를 구해 가장 작은 값.
- 점 P에서 선분 A-B까지: P를 선분 위로 수직으로 내린 위치 `t = ((P-A)·(B-A)) / |B-A|²`를 구하고 0~1로 자른 뒤, `A + t(B-A)`와 P 사이 거리(`math.hypot`).
- `load_zones`: `yaml.safe_load(Path(path).read_text())`로 읽고, 구역마다 점이 3개 이상인지, `set(zone['between']) - set(sides)`가 비었는지 본다.
</details>

<details>
<summary>정답 — traffic_good3.yaml</summary>

```yaml
# good3 지도(maps/good3.yaml) 전용 교통 정리 구역. 좌표는 map 프레임 [m].
# 지도를 새로 만들면 좌표를 다시 재고 test/test_traffic.py를 돌린다.
sides:            # 구역(문)으로만 서로 이어진 공간
  left:  {name: 왼쪽 방,     polygon: [[-0.2, -0.2], [1.585, -0.2], [1.585, 1.45], [-0.2, 1.45]]}
  right: {name: 오른쪽 구역, polygon: [[1.585, -0.2], [3.2, -0.2], [3.2, 1.45], [1.585, 1.45]]}

zones:            # 한 번에 한 대만 지나가는 곳
  - id: door
    name: 문
    between: [left, right]
    # 문틈 x≈1.585, y 0.88~1.23(폭 0.35 m) + 양쪽 앞마당 약 0.25 m
    polygon: [[1.35, 0.80], [1.85, 0.80], [1.85, 1.28], [1.35, 1.28]]
```

</details>

<details>
<summary>정답 — traffic.py</summary>

```python
"""교통 정리 판단. ROS 없는 순수 파이썬이라 DDS 없이 pytest로 시험한다.

1-1단계: 구역 파일 읽기와 기하 계산(점이 다각형 안인가, 어느 쪽인가, 문을 건너는가).
"""
import math
from pathlib import Path

import yaml


def edges(polygon):
    """다각형의 변 (앞 점, 뒤 점). 마지막 점과 첫 점도 잇는다."""
    return zip(polygon, polygon[1:] + polygon[:1])


def point_in_polygon(x, y, polygon):
    """점 (x, y)가 다각형 안인가. 오른쪽으로 쏜 선이 변을 홀수 번 넘으면 안이다(광선 투사)."""
    inside = False
    for (x1, y1), (x2, y2) in edges(polygon):
        if (y1 > y) != (y2 > y):                               # 이 변이 점의 높이를 가로지른다
            cross_x = x1 + (y - y1) * (x2 - x1) / (y2 - y1)    # 그 높이에서 변의 x
            if x < cross_x:
                inside = not inside
    return inside


def distance_to_segment(x, y, a, b):
    """점에서 선분 a-b까지 가장 가까운 거리."""
    (ax, ay), (bx, by) = a, b
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    # 점을 선분 위에 수직으로 내린 위치 t (0=a, 1=b). 선분 밖이면 끝점으로 자른다
    t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / length2))
    return math.hypot(x - (ax + t * dx), y - (ay + t * dy))


def distance_to_polygon(x, y, polygon):
    """다각형까지 거리 [m]. 안에 있으면 0."""
    if point_in_polygon(x, y, polygon):
        return 0.0
    return min(distance_to_segment(x, y, a, b) for a, b in edges(polygon))


def load_zones(path):
    """구역 YAML을 읽고 기본 검사를 한다. 틀리면 ValueError."""
    config = yaml.safe_load(Path(path).read_text())
    sides, zones = config.get('sides') or {}, config.get('zones') or []
    for zone in zones:
        if len(zone['polygon']) < 3:
            raise ValueError(f"{zone['id']}: polygon은 점이 3개 이상이어야 합니다")
        unknown = set(zone['between']) - set(sides)
        if unknown:
            raise ValueError(f"{zone['id']}: 모르는 쪽 {sorted(unknown)}")
    return config


def side_of(x, y, config):
    """점이 속한 쪽 이름('left' 등). 어느 쪽에도 없으면 None."""
    for name, side in config['sides'].items():
        if point_in_polygon(x, y, side['polygon']):
            return name
    return None


def zones_to_cross(start, goal, config):
    """start (x, y)에서 goal (x, y)로 가려면 지나야 하는 구역 id 목록. 같은 쪽이면 []."""
    a, b = side_of(*start, config), side_of(*goal, config)
    if a is None or b is None or a == b:
        return []
    return [zone['id'] for zone in config['zones'] if set(zone['between']) == {a, b}]
```

</details>

<details>
<summary>정답 — test_traffic.py</summary>

```python
from pathlib import Path

import pytest

from pinky_fleet.traffic import distance_to_polygon, load_zones, point_in_polygon, side_of, zones_to_cross

ZONES = Path(__file__).resolve().parents[1] / 'params' / 'traffic_good3.yaml'
SQUARE = [[0, 0], [1, 0], [1, 1], [0, 1]]


def test_point_in_polygon():
    assert point_in_polygon(0.5, 0.5, SQUARE)
    assert not point_in_polygon(1.5, 0.5, SQUARE)
    assert not point_in_polygon(0.5, -0.1, SQUARE)


def test_distance_to_polygon():
    assert distance_to_polygon(0.5, 0.5, SQUARE) == 0.0                     # 안
    assert distance_to_polygon(1.3, 0.5, SQUARE) == pytest.approx(0.3)      # 오른쪽 변까지
    assert distance_to_polygon(2.0, 2.0, SQUARE) == pytest.approx(2 ** 0.5)  # 모서리까지


def test_good3_sides():
    config = load_zones(ZONES)
    assert side_of(0.5, 0.5, config) == 'left'    # robot1 생성 위치
    assert side_of(2.0, 0.5, config) == 'right'   # robot2 생성 위치
    assert side_of(5.0, 5.0, config) is None      # 지도 밖


def test_crossing_the_door():
    config = load_zones(ZONES)
    assert zones_to_cross((0.5, 0.5), (2.10, 1.05), config) == ['door']  # 왼쪽 → 오른쪽
    assert zones_to_cross((2.0, 0.5), (0.30, 1.00), config) == ['door']  # 오른쪽 → 왼쪽
    assert zones_to_cross((2.0, 0.5), (2.10, 0.20), config) == []        # 같은 쪽


def test_bad_zone_file(tmp_path):
    bad = tmp_path / 'bad.yaml'
    bad.write_text('sides: {left: {polygon: [[0, 0], [1, 0], [1, 1]]}}\n'
                   'zones: [{id: door, between: [left, nowhere], polygon: [[0, 0], [1, 0], [1, 1]]}]\n')
    with pytest.raises(ValueError):
        load_zones(bad)
```

</details>

### 확인 방법

```bash
source /opt/ros/jazzy/setup.bash && source ~/giddongcar/pinky_pro/install/setup.bash
cd ~/giddongcar/pinky_pro/src/pinky_fleet
# test/test_traffic.py 맨 위 pytestmark 줄을 지운 뒤
python3 -m pytest test/test_traffic.py -q        # 처음엔 5 failed (NotImplementedError) → 채울수록 줄어 5 passed
python3 -m pytest test -q                        # 기존 테스트도 전부 통과
```

함수 하나씩 채우며 돌려 보면 좋다. 추천 순서: `point_in_polygon` → `side_of` → `load_zones` → `zones_to_cross` → `distance_to_segment` → `distance_to_polygon`.

손으로도 한 번 불러 본다(파이썬 대화창):

```python
from pinky_fleet.traffic import load_zones, zones_to_cross
c = load_zones('params/traffic_good3.yaml')
zones_to_cross((0.5, 0.5), (2.1, 1.05), c)   # ['door']
```

다 되면 알려 주세요. 짠 코드를 리뷰하고 1-2(`DoorGate`: 열쇠 주고받기)로 넘어갑니다.
