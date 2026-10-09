# pinky_fleet

PC에서 도는 팀 관제 패키지입니다. jinho가 만든 `jinho/` 폴더의 대시보드를 팀 공용 ROS 패키지로 옮긴 것에서 시작했습니다.

| 하는 일 | 실행 | 안내 |
|---|---|---|
| 실물 로봇 2대 관제: 로봇마다 Nav2 + 브라우저 대시보드 | `fleet` 또는 `ros2 launch pinky_fleet multi_robot.launch.py` | [실물 관제 실행](#실물-관제-실행) |
| 같은 관제를 Gazebo에서 | `ros2 launch pinky_fleet sim.launch.py` | [Gazebo 실행](#gazebo-실행) |
| 카메라 차선 인식으로 로봇 한 대를 직접 모는 실험 | `ros2 run pinky_fleet vision_drive ...` | [vision_drive](#독립-카메라인식차선-주행-실험) |

## 구조

- 로봇1은 도메인 15, 로봇2는 도메인 17에서 기본 `pinky_bringup`만 실행합니다.
- PC가 도메인마다 Nav2를 하나씩 띄웁니다. 각 Nav2는 해당 로봇의 scan, odom, TF를 받습니다.
- 대시보드는 도메인마다 ROS context와 TF buffer를 따로 두고, 두 로봇을 하나의 공통 지도 위에 그립니다.
- 두 로봇의 지도 내용(해시)이 다르면 그 로봇은 지도에 그리지 않고 목표 전송도 막습니다.
- 위치는 odom이 아니라 `map → base_link` TF로 구합니다. 최근 odom이나 TF가 없으면 마커를 숨깁니다.
- AMCL은 `set_initial_pose: false`로 실행해서 두 로봇이 원점에 있다고 가정하지 않습니다. 위치를 모르면 제자리에서 한 바퀴 돌며 스스로 찾습니다(`auto_spin`). 위치를 이미 알 때(시뮬)는 `robot1_initial_pose:=x,y,yaw`로 넘기면 그 로봇만 켜지자마자 그 자리로 잡습니다.
- 교통 정리: 지도를 칸으로 나누고, 로봇은 지나갈 칸의 열쇠를 모두 받아야 출발합니다(먼저 온 순서). 달리는 중 경로가 겹치면 robot2가 양보합니다. 칸 파일 `params/traffic_good3.yaml`은 good3 지도 전용이라 다른 지도에서는 스스로 꺼집니다. 원리는 [docs/learn/traffic.md](../../../docs/learn/traffic.md).

## 실물 관제 실행

보통은 [docs/real.md](../../../docs/real.md)의 처음 설정(`~/.config/pinky_fleet.env`, `fleet` alias)을 해 두고 `fleet` 한 줄로 켭니다. `fleet`(`scripts/start_fleet.sh`)은 네트워크와 다른 관제를 점검한 뒤 아래 launch를 실행하고, Ctrl+C로 끄면 두 로봇에 0 속도를 보냅니다.

스크립트 없이 켤 때는 같은 도메인에서 Nav2가 두 번 뜨지 않도록 이미 켜져 있는 Nav2/AMCL을 먼저 끕니다. 카메라 YOLO까지 쓰려면 [Python 환경](#python-환경)의 `PYTHONPATH`도 같은 터미널에서 설정합니다.

```bash
source ~/giddongcar/pinky_pro/install/setup.bash
export ROS_STATIC_PEERS="<로봇1 IP>;<로봇2 IP>"
ros2 launch pinky_fleet multi_robot.launch.py
```

브라우저에서 http://localhost:8080 을 엽니다.

1. ⚠️ 로봇마다 AMCL이 켜지면 **제자리에서 한 바퀴 돌며** 위치를 찾습니다. 관제를 켜기 전에 로봇 주변을 비웁니다. 돌지 않게 하려면 `auto_spin:=false`.
2. 위치를 못 찾았거나 틀리면 그 로봇의 **↗ 초기 위치**를 누르고, 지도에서 로봇이 실제로 있는 곳을 누른 채 바라보는 방향으로 끌었다가 놓습니다.
3. 지도에 두 로봇(① ②)이 뜨면 **⚑ 목적지**로 목표를 같은 방법으로 지정합니다(끄는 방향 = 도착 방향).
4. launch 터미널에서 Ctrl+C를 누르면 Nav2 2개와 대시보드가 함께 종료됩니다.

### 설정

`fleet`은 `~/.config/pinky_fleet.env`를 읽습니다. 이 파일은 Git에 올리지 않습니다. 다른 파일을 쓰려면 `PINKY_FLEET_ENV=/경로/파일`.

| 설정 파일 값 | 기본값 | 설명 |
|---|---:|---|
| `ROBOT1_IP`, `ROBOT2_IP` | 필수 | 각 로봇의 Wi-Fi IP. DDS peer와 카메라 주소에 씁니다 |
| `ROBOT1_DOMAIN`, `ROBOT2_DOMAIN` | `15`, `17` | 각 로봇의 ROS 2 도메인 |
| `FLEET_PORT` | `8080` | 대시보드 포트 |

launch 인자는 `fleet` 뒤에 붙이거나(`fleet auto_spin:=false camera_port:=5001`) `ros2 launch`에 직접 넘깁니다.

| launch 인자 | 기본값 | 설명 |
|---|---:|---|
| `map`, `params_file` | 패키지의 `maps/good3.yaml`, `params/nav2_params.yaml` | 지도 파일과 Nav2 파라미터 |
| `robot1_domain`, `robot2_domain` | `15`, `17` | 로봇별 도메인 |
| `robot1_camera_host`, `robot2_camera_host` | 환경 변수 `ROBOT1_IP`, `ROBOT2_IP` | 영상 연결 주소. 비우면 해당 카메라를 사용하지 않음 |
| `camera_port` | `5000` | BLE 응답에 URL이 없을 때 쓰는 기본 카메라 포트 |
| `host` | `127.0.0.1` | 대시보드 bind 주소. 기본값은 PC 내부에서만 접속 가능 |
| `port` | `8080` | 대시보드 포트 |
| `robot1_initial_pose`, `robot2_initial_pose` | 비어 있음 | 초기 위치 `x,y,yaw` |
| `auto_spin` | `true` | 위치를 모를 때 로봇이 자동으로 한 바퀴 돌아 위치를 찾음 |
| `traffic_zones` | `params/traffic_good3.yaml` | 교통 정리 칸 파일. `traffic_zones:=`로 끔 |

```bash
ros2 launch pinky_fleet multi_robot.launch.py map:=/절대경로/map.yaml port:=8090 robot1_domain:=15 robot2_domain:=17
```

### 연결 확인

PC와 로봇 사이에 DDS 검색과 센서 데이터 통신이 되어야 합니다. 키보드 조종이 된다고 해서 scan이나 TF 수신까지 된다는 보장은 없습니다.

```bash
ROS_DOMAIN_ID=15 ros2 topic echo /scan sensor_msgs/msg/LaserScan --once
ROS_DOMAIN_ID=17 ros2 topic echo /scan sensor_msgs/msg/LaserScan --once
ROS_DOMAIN_ID=15 ros2 run tf2_ros tf2_echo map base_link
ROS_DOMAIN_ID=17 ros2 run tf2_ros tf2_echo map base_link
```

## Gazebo 실행

```bash
ros2 launch pinky_fleet sim.launch.py
```

실제 방(good3 지도)과 같은 Gazebo 월드에 로봇 2대를 띄우고(도메인 25/27) 실물과 같은 Nav2 2개와 대시보드를 시뮬 시간으로 실행합니다. 초기 위치는 생성 위치(robot1 (0.5, 0.5), robot2 (2.0, 0.5))로 자동으로 잡힙니다. 인자와 주의사항은 [docs/sim.md](../../../docs/sim.md).

## 대시보드 사용법

화면 머리줄의 **?**(도움말)에도 같은 내용이 있습니다.

- 지도 위 도구줄에 로봇마다 **[↗ 초기 위치] [⚑ 목적지]** 버튼이 있습니다. 초기 위치는 RViz의 2D Pose Estimate, 목적지는 Nav2 Goal과 같습니다.
- 지도에서 누른 채 끌면 위치와 방향이 함께 정해집니다. 끌지 않고 클릭만 하면 로봇의 지금 방향을 그대로 씁니다.
- 도구는 한 번 쓰면 꺼집니다. 이어서 여러 번 찍으려면 Shift를 누른 채 놓습니다. Esc는 도구만 끄고 Nav2 목표는 그대로 둡니다.
- "위치 못 찾음"이면 **⟳ 돌면서 찾기**(제자리 한 바퀴)나 **↗ 초기 위치**로 다시 잡습니다.
- **■ 이동 취소**는 Nav2 목표와 제자리 회전만 취소합니다. 하드웨어 비상정지가 아닙니다. 텔레옵으로 몰았다면 그 터미널에서 0 속도를 따로 보냅니다.
- 지도는 벽(점유 칸)을 흰 선, 바닥과 미탐색을 검정으로 그립니다. map_server의 `/map`이 오기 전에는 지도 YAML을 직접 읽어 먼저 보여 줍니다.
- 지도 바로 아래 좌표 줄에 두 로봇의 x, y, 방향과 커서 좌표가 나옵니다(m, °).
- 단축키: `1` `2` 로봇 선택, `P` 초기 위치, `G` 목적지, `Esc` 도구 끄기, `F` 지도 맞춤, `X` 선택한 로봇 이동 취소(`Shift+X` 두 로봇), `?` 도움말.
- 같은 도메인에서 `vision_drive`가 돌면 로봇 카드 아래에 그 화면과 **▶ 출발** / **■ 정지** 버튼이 뜹니다. [관제 웹에서 vision_drive 보기](#관제-웹에서-vision_drive-보기).

## 카메라 영상

대시보드 로봇 카드의 **카메라 시작/중지**로 로봇 카메라를 켜고 YOLO 탐지 결과를 겹쳐 봅니다. 관측 기능이라 속도 명령은 보내지 않습니다.

- **필요한 것**: 각 로봇의 BLE 서비스가 `status`와 `set_camera` 명령을 지원해야 합니다. PC Bluetooth가 켜져 있어야 하고, [Python 환경](#python-환경)의 `bleak`을 씁니다.
- **켜는 과정**: BLE에서 Pinky를 찾고 응답 IP가 설정한 로봇 IP와 맞는지 확인한 뒤 명령을 보냅니다. 스트림 주소는 BLE 응답 URL을 우선 쓰고, 응답에 주소가 없을 때만 `http://<로봇IP>:5000/`을 씁니다.
- **BLE 서버**: 로봇의 `/opt/pinky-ble/ble_server.py`는 이 저장소에 없고 로봇마다 따로 설치·관리합니다. 두 로봇 모두 카메라 제어가 되는 같은 버전이어야 합니다.
- **카메라 점유**: 로봇 카메라는 한 프로그램만 쓸 수 있습니다. 대시보드에서 켜기 전에 Pinky Studio의 영상 창을 닫고, `vision_drive`가 쓰는 로봇이면 대시보드 카메라 주소를 비웁니다.
- **영상 공유**: 대시보드가 로봇별 스트림을 한 번만 받고, 브라우저는 `/camera/robot1.jpg`(`robot2.jpg`)에서 약 150 ms 간격으로 최신 JPEG를 가져옵니다. 여러 브라우저 창이 같은 프레임을 공유합니다. 상태 줄의 `브라우저 표시 WxH`는 브라우저가 프레임을 디코딩했다는 뜻이라, 영상 수신과 브라우저 표시를 따로 확인할 수 있습니다.
- **YOLO**: 패키지의 `models/yolo11n.pt`(COCO 객체 모델)를 쓰고 GPU가 있으면 Ultralytics가 자동으로 고릅니다. 가중치가 없으면 첫 실행 때 인터넷에서 내려받습니다. 다운로드나 초기화에 실패해도 카메라와 Nav2는 계속 돌고 YOLO 상태에 오류가 뜹니다. 차선·횡단보도 모델이 아니며, 결과는 주행 명령에 연결되지 않습니다.

차선 안에서 Nav2 목표를 주행하고 횡단보도·장애물 정책을 적용하기 위한 계획은 [실물 차선 인식·주행 계획](../../../docs/lane_aware_driving_plan.md)에 있습니다.

### 카메라 문제 확인

- `unknown cmd: set_camera`: 그 로봇의 BLE 서비스가 구버전입니다. `/opt/pinky-ble/ble_server.py`를 카메라 지원 버전으로 갱신하고 BLE 서비스를 재시작하세요.
- `카메라 꺼짐` 또는 `Connection refused`: BLE 시작 응답과 대시보드에 표시된 로봇 주소를 확인하고, 기본 포트를 쓰면 로봇에서 `sudo ss -ltnp | grep ':5000'`을 확인하세요. BLE 응답이 별도 주소를 주면 그 주소가 우선입니다.
- `영상 수신 중`인데 브라우저 표시 크기가 안 뜸: 대시보드 JPEG 요청이나 브라우저 디코딩 문제입니다. `브라우저 이미지 디코딩 실패`가 표시되면 JPEG 응답을 확인하세요.
- `:5000/`이 `<img src="/snapshot?...">`가 든 HTML을 돌려주는 것은 정상일 수 있습니다. 대시보드는 그 `/snapshot`에서 JPEG를 받습니다. 이 로봇에서 확인한 Jupyter 포트는 `8888`입니다.

## Python 환경

YOLO, CUDA용 PyTorch, BLE 라이브러리(`bleak`)는 저장소 루트의 `.venv`에 한 번 설치합니다. 대시보드 카메라와 `vision_drive`가 씁니다. ROS Python과 패키지를 공유하도록 `--system-site-packages` 가상환경을 씁니다. 아래 CUDA 13.0 명령은 현재 관제 PC에서 확인한 조합입니다. 다른 PC는 [PyTorch 설치 선택기](https://pytorch.org/get-started/locally/)에서 OS·pip·Python·CUDA에 맞는 명령을 고르세요.

```bash
cd ~/giddongcar   # 저장소 루트
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu130
.venv/bin/python -m pip install ultralytics bleak
```

ROS 실행 파일은 venv를 켜도 `/usr/bin/python3`로 돌기 때문에, venv 패키지 경로를 `PYTHONPATH`에 직접 넣습니다. `fleet`(`start_fleet.sh`)은 자동으로 넣고, `ros2 launch`·`ros2 run`을 직접 쓸 때는 같은 터미널에서 먼저 실행합니다.

```bash
export PYTHONPATH="$HOME/giddongcar/.venv/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
```

GPU 확인(저장소 루트에서). `CUDA 사용 가능: True`면 GPU 가속이 됩니다.

```bash
source /opt/ros/jazzy/setup.bash
PYTHONPATH="$PWD/.venv/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -c "import torch, ultralytics; print('Ultralytics', ultralytics.__version__); print('PyTorch', torch.__version__); print('CUDA 사용 가능:', torch.cuda.is_available())"
```

`.pt` 가중치는 대용량 파일이라 Git에 넣지 않습니다.

## 독립 카메라·인식·차선 주행 실험

`vision_drive`는 대시보드와 별도로 로봇 한 대의 카메라를 켜고, 학습한 segmentation 모델로 주행 가능 영역(`driveable_area`)과 횡단보도(`crosswalk`)를 찾아 그 가운데를 따라가는 **저속 시각 추종 prototype**입니다. Nav2와 지도 목적지는 쓰지 않습니다. 기본은 **관찰 모드**(`--mode observe`)라 속도 명령을 내지 않습니다. 요구사항별 진행 상황은 [vision_drive MVP](../../../docs/vision_drive_mvp.md).

⚠️ 주행 모드 안전 조건

- 같은 로봇의 `cmd_vel`을 Nav2나 대시보드가 발행하지 않아야 합니다. 다른 발행자가 보이면 시작을 거부하고, 주행 중이면 정지합니다. `fleet`과 함께 돌릴 수 없으니 관제 화면은 [대시보드만](#관제-웹에서-vision_drive-보기) 띄웁니다.
- 카메라·추론·라이다가 끊기거나 주행 영역이 불분명하면 정지합니다.
- 로봇 bringup의 `cmd_vel` watchdog은 확인되지 않았습니다. 그래서 `--confirm-attended-test-without-watchdog`이 필요하고, 사람이 로봇 바로 옆에서 물리 비상정지를 잡고 감독하는 통제 구역에서만 씁니다. PC나 네트워크가 끊기면 정지를 보장하지 않습니다.
- `q`나 Ctrl+C로 끄면 0 속도를 반복 발행하고 카메라 중지를 요청합니다. 강제 종료 때는 정지를 보장하지 않습니다.

### 실행

ROS 도메인은 그 로봇 값(로봇1 `15`, 로봇2 `17`), `ROS_STATIC_PEERS`에는 로봇 IP를 둡니다. Pinky Studio 영상 창은 닫아 둡니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/giddongcar/pinky_pro/install/setup.bash
export ROS_DOMAIN_ID=15 ROS_STATIC_PEERS=192.168.0.6
export PYTHONPATH="$HOME/giddongcar/.venv/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"

# 영상 + segmentation/객체 인식만 확인 (속도 명령 없음)
ros2 run pinky_fleet vision_drive --robot-ip 192.168.0.6 --model /경로/학습모델.pt --mode observe

# 감독형 저속 차선 주행. 로봇 옆에서 직접 감독할 때만
ros2 run pinky_fleet vision_drive --robot-ip 192.168.0.6 \
  --preset ~/giddongcar/pinky_pro/src/pinky_fleet/config/vision_drive_supervised.yaml \
  --enable-motion --confirm-supervised-test --confirm-attended-test-without-watchdog
```

- 주변에 IP가 같은 Pinky가 여럿 있으면(공유기가 달라도 `192.168.0.x`가 겹칠 수 있음) 엉뚱한 로봇 카메라에 명령이 갈 수 있습니다. `--ble-name pinky_6422`처럼 로봇의 BLE 이름을 지정하세요. `pinky_driving`의 `tape_lane_drive`도 같은 인자를 받습니다.
- Ultralytics의 외부 DNS 확인을 끄고(오프라인 모드) 돌기 때문에, 모델 파일과 Python 의존성이 설치돼 있으면 인터넷 없이 시작합니다. 카메라 제어에는 로봇과 BLE, 영상 수신에는 같은 로컬 네트워크가 필요합니다.
- 창 없이 돌리려면 `--headless`.

### preset과 명령행 인자

`--preset`은 모델, 모드, 횡단보도 정책, 속도 상한, 조향·코너 값을 YAML에서 불러옵니다. 명령행 값이 preset보다 우선하므로, 값을 바꿔 비교할 때는 preset을 고치지 말고 명령행에 덧붙입니다(관찰만: `--mode observe`, 다른 모델: `--model /경로/모델.pt`).

- 로봇 IP와 모션 허용·감독 확인 옵션(`--enable-motion`, `--confirm-supervised-test`, `--watchdog-verified`, `--confirm-attended-test-without-watchdog`)은 YAML에 넣을 수 없고 매번 명령행에 적습니다. preset만 주면 주행 모드 확인에서 멈추고 로봇은 움직이지 않습니다.
- preset의 모델 경로 `~/vision_drive_observations/train_runs/lane_seg_v1/weights/best.pt`는 저장소 밖이라 PC마다 따로 둡니다.
- 설치된 preset은 `$(ros2 pkg prefix pinky_fleet)/share/pinky_fleet/config/vision_drive_supervised.yaml`에도 있습니다.

| 인자 | 기본값(= preset) | 허용 범위 | 뜻 |
|---|---:|---|---|
| `--max-linear` | 0.08 m/s | ≤ 0.10 | 최대 전진 속도 |
| `--max-angular` | 0.40 rad/s | ≤ 0.60 | 최대 회전 속도 |
| `--steering-gain` | 1.2 | ≤ 2.0 | 먼 쪽 mask 중심 오차에 곱하는 조향 gain |
| `--turn-radius-limit` | 0.08 m | | 급회전 때 `v / \|w\|`가 이 반경을 넘지 않게 전진 속도를 줄임 |
| `--stop-distance` | 0.35 m | ≥ 0.20 | 라이다 전방 정지 거리 |
| `--obstacle-min-width` | 0.12 m | 0.03 ~ 0.50 | 이 폭 이상인 물체만 장애물로 봄 |
| `--crosswalk-action` | `wait-signal` | `stop-then-go` `slow` `stop` `ignore` | 횡단보도 정책 |
| `--crosswalk-stop-seconds` | 10 | | `stop-then-go`의 정지 시간 |
| `--corner-wall-distance` | 0.45 m | | 정면 벽이 이 안이면 코너로 보고 제자리 회전 |
| `--corner-min-turn-deg` | 60° | | 정면이 막힌 코너에서 트인 복도를 못 찾았을 때 최소 회전각 |
| `--corner-max-turn-deg` | 100° | | 한쪽으로 차선을 찾을 최대 회전각 |

### 영상 창 키

| 키 | 동작 |
|---|---|
| `g` | 출발 신호. 시작·횡단보도·정지 대기를 풂 |
| `space` | 정지. 출발 신호를 받을 때까지 서 있음 |
| `r` | 원본/오버레이 MP4 녹화 시작·종료 |
| `s` | 원본 프레임, 오버레이, 탐지 결과 JSON 저장(관찰 모드만) |
| `q` | 종료. 0 속도를 보내고 카메라 중지 요청 |

저장 위치는 `~/vision_drive_observations`(실행 로그 포함), `--output-dir`로 바꿉니다.

### 주행 규칙

- **조향**: 여러 영상 높이에서 주행 영역 중심을 구해 먼 쪽에 더 큰 비중을 둡니다. 횡단보도 위에서 차선을 잃지 않도록 `crosswalk` mask를 주행 영역에 합칩니다. 오버레이의 자홍색 선이 조향 미리보기입니다.
- **장애물**: 전방 라이다에서 이어진 측정값들의 가로 폭이 `--obstacle-min-width` 이상인 물체가 `--stop-distance` 안에 있으면 멈추고, 멀어지면 다시 갑니다. 바닥의 작은 점이나 테이프 자국은 무시하는 대신 작은 장애물도 놓칠 수 있으니 주변을 비우고 시험합니다.
- **출발·정지**: 주행 모드는 정지한 채 시작해 출발 신호(관제 웹 **▶ 출발** 또는 `g`)를 기다립니다. 관제 웹 **■ 정지**나 `space`로 언제든 다시 세웁니다.
- **횡단보도(`wait-signal`)**: 경로 위 횡단보도가 두 프레임 연달아 보이면 멈추고, 출발 신호를 받아야 건넙니다. 감지가 잠깐 끊겨도 신호 전에는 움직이지 않고, 건너는 동안은 다시 멈추지 않습니다(2초 넘게 안 보이면 지나간 것으로 봄). 출발 신호에는 화면에서 본 대기 번호(`#1`, `#2` …)가 붙어, 그 사이 다른 대기로 바뀌었으면 무시됩니다.
- **90° 코너**: 차선이 사라지거나 정면 벽이 `--corner-wall-distance` 안에서 앞을 가득 채우면 바로 멈춰 제자리 회전합니다. 회전 중에는 전진하지 않고, 라이다가 끊기면 정지합니다.
  - 방향: 한쪽만 복도로 트였으면 그쪽, 아니면 좌우 라이다 여유 거리가 더 긴 쪽, 비슷하면 직전 조향 방향입니다.
  - 얼마나 도나: 정면이 0.9 m 안에서 막혀 있으면 라이다로 트인 복도 방향(20~150° 중 0.9 m 넘게 트인 구간의 가운데)까지 돈 뒤에야 차선을 보고 출발합니다. 복도를 못 찾으면 `--corner-min-turn-deg`를 돌기 전에는 끝내지 않습니다. 20~40°만 돌고 전진해 코너로 대각선으로 들어가던 문제 때문입니다. 정면이 트인 채 차선만 잃었으면 바로 다시 찾습니다.
  - 회전량은 odom으로 잽니다(끊기면 명령 속도×시간으로 추정). `--corner-max-turn-deg`까지 못 찾으면 반대쪽으로 같은 각도까지 찾고, 그래도 없으면 정지한 채 다시 돌지 않습니다.
  - 실제 판단은 로그의 `코너 진입:` 줄에 남습니다.

이 결과를 대시보드의 Nav2 경로에 연결하거나 속도·정지 기준을 현장에서 확정하려면 별도 보정과 안전 검토가 필요합니다.

### 관제 웹에서 vision_drive 보기

`vision_drive`는 같은 도메인에 영상 창 화면(`vision_drive/overlay/compressed`)과 판단 상태 JSON(`vision_drive/state`)을 내보내고, `vision_drive/command`로 관제의 출발·정지 신호를 받습니다. 로봇을 직접 모는 동안 관제 웹은 **Nav2 없이 대시보드만** 띄웁니다. 대시보드는 제자리 회전 버튼을 누르기 전에는 `cmd_vel`을 만들지 않아 `vision_drive`를 막지 않습니다.

```bash
# 터미널 A: vision_drive (위의 감독형 주행 명령 그대로). 시작하면 정지한 채 출발 신호를 기다린다
# 터미널 B: 대시보드만. 도메인 기본값 15/17, http://localhost:8080
source /opt/ros/jazzy/setup.bash && source ~/giddongcar/pinky_pro/install/setup.bash
export ROS_STATIC_PEERS="<로봇1 IP>;<로봇2 IP>"   # 카드에 로봇 연결 상태(odom)까지 보려면
ros2 run pinky_fleet fleet_dashboard
```

- 로봇 카드 아래에 영상 창과 같은 화면(차선 mask, 조향점, 상태 글자)이 뜹니다.
- 주황 띠는 출발 신호를 기다린다는 뜻입니다(시작, 횡단보도, 정지). 화면을 보고 **▶ 출발**을 누르면 그 대기(`#번호`)만 풀립니다. 그 사이 다른 대기로 바뀌었으면 거절하고 다시 보라고 알려 줍니다.
- **■ 정지**는 ▶ 출발을 누를 때까지 세웁니다. 비상 정지가 아니므로 로봇 옆 감독자는 그대로 둡니다.
- 대시보드 카메라 주소(`--robot1-camera-host`)는 비워 둡니다. 로봇 카메라는 `vision_drive`가 씁니다.
- 대기가 생기고 풀릴 때마다 오른쪽 이벤트 기록에 남습니다.
- Nav2가 없으므로 지도는 대시보드가 지도 YAML(기본 `maps/good3.yaml`, `--map`으로 변경)을 직접 읽어 보여 줍니다. 로봇 위치는 추정하지 않습니다(카드: 위치 없음). Nav2를 켜면 map_server의 `/map`이 우선입니다.
- 두 터미널의 DDS 설정(`RMW_IMPLEMENTATION`, peer 설정)이 같아야 합니다. Cyclone XML로 multicast를 끄고 `<Peers>`에 로봇만 적었다면 같은 PC의 두 프로세스가 서로 못 찾습니다. `<Peer Address="localhost"/>`를 넣으세요(2026-10-09 이 PC에서 확인).

### 모델

- **팀 학습 모델**: 차선 추종에는 `driveable_area` segmentation 클래스가 꼭 있어야 하고, 횡단보도 정책에는 `crosswalk` 클래스를 씁니다. 이름이 다르면 `--driveable-class`, `--crosswalk-class`로 바꿉니다.
- **COCO 객체 모델**: 보이면 정지할 기본 클래스는 `person`, `bicycle`, `car`, `motorcycle`, `bus`, `truck`, `bench`, `backpack`, `suitcase`, `chair`입니다(`--obstacle-classes`). 흰 테이프 차선, 횡단보도, 학습하지 않은 사용자 정의 장애물은 인식하지 않습니다. 실물 장애물 정지는 라이다 기준을 유지합니다.
- **YOLOTL**: 클래스가 `lane` 하나라 아래처럼 관찰합니다. 도로/BEV 시점 차선 데이터로 학습한 결과라 Pinky 바닥에서는 검증되지 않았습니다. 관찰·샘플 저장용이고, 저장한 원본 프레임을 코스 장면별로 골라 라벨링해 전용 모델을 학습해야 합니다.

  ```bash
  ros2 run pinky_fleet vision_drive --robot-ip 192.168.0.6 \
    --model "$HOME/.cache/pinky_fleet/yolotl/weights.pt" \
    --driveable-class lane --mode observe
  ```

- **공개 차선 모델** `models/lane_yolo11n_seg_best.pt`: 실외 자동차 영상으로 학습한 YOLOv11 segmentation 예제입니다(좌·우 실선/점선 네 클래스). `driveable_area`가 없어 `drive` 모드에 맞지 않으니 **실물 주행에 쓰지 마세요**. `observe`에서 출력 형식과 오버레이를 보는 용도입니다. `.gitignore` 대상이라 필요하면 원본 [저장소](https://github.com/SyedaEmanSaleem/Lane-Detection-Segmentation-using-YOLOv11)(MIT 라이선스 표시)에서 다시 받습니다.

흰 테이프를 밝기로 찾아 따라가는 실험은 별도 `pinky_driving` 패키지의 `tape_lane_drive`입니다. [pinky_driving README](../pinky_driving/README.md).

## 아직 안 되는 것

- 두 로봇의 작업 배정은 없습니다.
- 교통 정리 칸 파일은 good3 지도 전용입니다. 지도를 새로 만들면 칸 좌표를 다시 재야 합니다.
- 대시보드의 `nav_ready`는 Nav2 액션 서버가 보이는지만 봅니다. Nav2 내부 활성화 실패는 잡지 못합니다.
- `vision_drive`는 Nav2와 연결되지 않은 단독 실험이고, 코스 완주는 아직 검증되지 않았습니다.

## 테스트

```bash
cd ~/giddongcar/pinky_pro/src/pinky_fleet
python3 -m pytest test
```
