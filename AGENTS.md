# AGENTS.md

팀원과 AI 코딩 에이전트(Claude Code, Codex, Cursor 등)가 함께 따르는 작업 규칙입니다.
처음 설치하는 방법은 [README.md](README.md)를 보세요.

## 프로젝트 목표

공유기 1대에 노트북 2대와 Pinky Pro 2대를 연결해서

1. 두 로봇이 **서로 부딪히지 않고** 지도 위 경로를 따라 주행하고
2. 로봇 **카메라 영상으로 탐지**한 결과를 관제 화면에 띄운다.

지금은 실물 로봇보다 **Gazebo 시뮬레이션에서 먼저** 검증하는 단계입니다.

## 환경

- Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic(`ros_gz_*`)
- Python 3.12, OpenCV(`python3-opencv`), `cv_bridge`
- DDS: 지금은 Fast DDS(ROS 기본값)를 쓴다. Cyclone DDS 전환은 아직 안 했다.

## 저장소 구조

| 경로 | 내용 |
|---|---|
| `pinky_pro/` | colcon 워크스페이스 루트. 빌드는 항상 여기서 한다 |
| `pinky_pro/src/pinky_*` | 제조사(pinklab) 원본 패키지. 구동(bringup), URDF(description), 시뮬(gz_sim), SLAM·Nav2(navigation), LED·LCD 등 |
| `pinky_pro/src/pinky_fleet/` | 팀 작성. **관제 메인.** PC에서 로봇별 Nav2 2개와 웹 대시보드를 띄운다 |
| `pinky_pro/src/pinky_gui/` | 팀 작성. PyQt 지도 뷰. 지금은 쓰지 않는다 |
| `jinho/` | `pinky_fleet`의 원본 작업 폴더. 작성자가 정리할 예정이라 수정하지 않는다 |
| `pinky_pro/chatter_bridge.yaml` | 실물용 `domain_bridge` 설정(도메인 15·17 ↔ 0) |

## 빌드

```bash
source /opt/ros/jazzy/setup.bash
cd ~/giddongcar/pinky_pro
colcon build --symlink-install                              # 전체
colcon build --symlink-install --packages-select <패키지>   # 하나만
source install/setup.bash
```

- `--symlink-install`로 빌드하면 파이썬 코드나 launch 파일을 고쳐도 다시 빌드할 필요가 없다. 새 파일이나 새 실행 파일(`setup.py`의 entry point)을 추가했을 때만 다시 빌드한다.
- 다른 워크스페이스(예: 수업 때 만든 `~/pinky_pro`)와 동시에 source하지 않는다. 패키지 이름이 같아서 어느 쪽 코드가 실행되는지 헷갈린다.

## Gazebo 시뮬레이션 (로봇 1대)

터미널마다 `source ~/giddongcar/pinky_pro/install/setup.bash`를 한 뒤 실행한다.

```bash
# 1. 시뮬 실행 (기본 월드: pinky_factory.world)
ros2 launch pinky_gz_sim launch_sim.launch.xml

# 2-a. 지도 만들기: SLAM + RViz + 키보드 조종 → 저장
ros2 launch pinky_navigation gz_map_building.launch.xml
ros2 launch pinky_navigation gz_map_view.launch.xml
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run nav2_map_server map_saver_cli -f <저장경로/지도이름>

# 2-b. 저장된 지도로 Nav2 주행
ros2 launch pinky_navigation gz_bringup_launch.xml map:=<지도.yaml>
ros2 launch pinky_navigation gz_nav2_view.launch.xml

# 카메라 영상 확인 (창에서 /camera/image_raw 선택)
ros2 run rqt_image_view rqt_image_view
```

시뮬에서 나오는 주요 토픽은 `/scan`, `/odom`, `/tf`, `/clock`, `/camera/image_raw`, `/camera/camera_info`이다. `/cmd_vel`은 시뮬로 들어가는 입력이다.

### 시뮬은 내 PC 안에서만 돌리기

로봇 공유기에 연결된 채로 시뮬을 돌리면 실물 로봇이나 다른 팀원의 시뮬 토픽과 섞일 수 있다. 시뮬 터미널에서는 먼저 아래를 실행한다.

```bash
unset ROS_STATIC_PEERS FASTRTPS_DEFAULT_PROFILES_FILE   # 실물용 DDS 설정 끄기(설정해 둔 경우)
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST          # 내 PC 안에서만 통신
```

## 실물 로봇 (참고)

- 로봇1은 `ROS_DOMAIN_ID=15`, 로봇2는 `17`을 쓴다. 로봇에서는 `pinky_bringup`만 켜고 Nav2는 PC에서 돌린다(`ros2 launch pinky_fleet multi_robot.launch.py`, 자세한 내용은 [pinky_fleet/README.md](pinky_pro/src/pinky_fleet/README.md)).
- 공유기가 DDS 멀티캐스트를 걸러낸다. 그래서 PC와 로봇이 서로 찾으려면 유니캐스트 peer 설정(`ROS_STATIC_PEERS` 등)이 필요하다.
- ⚠️ `pinky_bringup`에는 속도 명령 타임아웃이 없다. `cmd_vel`이 끊겨도 **마지막 속도로 계속 달린다.** 키보드 조종을 끈 뒤에는 0 속도를 한 번 보낸다.
  ```bash
  ROS_DOMAIN_ID=15 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
  ```

## 코드 작성 규칙

- 새 기능은 `pinky_pro/src/` 아래에 **새 ROS 패키지**로 만든다(예: 카메라 → `pinky_camera`). 사람 이름 폴더는 만들지 않는다.
- 경로, IP, 도메인 번호를 코드에 직접 쓰지 않는다. launch 인자나 ROS 파라미터로 받는다.
- 토픽은 상대 이름(`cmd_vel`, `camera/image_raw`)으로 쓴다. 나중에 로봇 2대를 네임스페이스로 구분하기 위해서다.
- launch 파일에서 `use_sim_time`을 넘겨준다(시뮬은 `True`, 실물은 `False`).
- WiFi로 영상을 보낼 때는 raw 대신 압축 토픽(`image_transport`의 compressed)을 쓴다.
- 제조사 원본 패키지는 꼭 필요할 때만 최소한으로 고치고, 커밋 메시지에 이유를 적는다.
- 다음은 커밋하지 않는다: `build/`·`install/`·`log/`, rosbag·녹화 영상·모델 가중치 같은 대용량 파일, 비밀번호·토큰.

## AI 에이전트 작업 규칙

- 변경은 작게, 한 번에 한 가지 목적만 담는다.
- 실물 로봇을 움직이는 명령(`cmd_vel` 발행, Nav2 목표 전송)은 사람이 확인하기 전에 실행하지 않는다.
- 수정한 뒤에는 해당 패키지를 빌드해서 확인한다. 직접 실행해 보지 못한 부분은 그렇다고 밝힌다.
- 설명과 커밋 메시지는 한국어로 쓴다. 코드 식별자는 영어로 쓴다.

## Git 컨벤션

### 브랜치

- `main`에 직접 push하지 않는다. 작업마다 브랜치를 만들고 PR로 합친다.
- 브랜치 이름은 `<타입>/<짧은-설명>` 형식이다. 예: `feat/camera-viewer`, `fix/nav2-footprint`, `docs/agents-md`
- 작업을 시작하기 전에 `git switch main && git pull`로 최신 상태를 받은 뒤 브랜치를 만든다.

### 커밋 메시지

```
<타입>(<범위>): <한국어 요약>
```

- 타입
  - `feat`: 기능 추가
  - `fix`: 버그 수정
  - `refactor`: 동작은 그대로 두고 구조만 개선
  - `docs`: 문서
  - `test`: 테스트
  - `chore`: 설정, 빌드, 기타
- 범위(선택)는 바꾼 영역이다. `camera`, `nav`, `sim`, `gui`, `fleet`, `bringup` 등.
- 요약은 50자 안팎으로, 무엇을 했는지 쓴다. 이유가 필요하면 한 줄 띄우고 본문에 쓴다.
- 한 커밋에는 한 가지 변경만 담는다. 빌드가 깨진 상태로 커밋하지 않는다.

```
feat(camera): 가제보 카메라 영상 구독 노드 추가
fix(nav): 좁은 통로에서 멈추는 footprint 값 수정
docs: AGENTS.md와 git 컨벤션 추가
```

### PR

- 제목은 커밋 메시지 규칙과 똑같이 쓴다.
- 본문에는 **무엇을 왜 바꿨는지**, **테스트 방법**(가제보인지 실물인지, 실행 명령)을 쓴다. 화면이 바뀌었으면 스크린샷을 붙인다.
- 작성자가 직접 머지한다. 리뷰는 선택이다. 머지는 **Squash and merge**로 하고, 머지한 브랜치는 삭제한다.
