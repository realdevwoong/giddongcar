# pinky_fleet

PC 한 대에서 로봇 2대의 Nav2를 띄우고, 브라우저 관제 화면에서 두 로봇의 위치를 보고 목표를 지정합니다.
jinho가 만든 `jinho/` 폴더의 대시보드를 팀 공용 ROS 패키지로 옮긴 것입니다.

## 구조

- 로봇1은 도메인 15, 로봇2는 도메인 17에서 기본 `pinky_bringup`만 실행합니다.
- PC가 도메인마다 Nav2를 하나씩 띄웁니다. 각 Nav2는 해당 로봇의 scan, odom, TF를 받습니다.
- 대시보드는 도메인마다 ROS context와 TF buffer를 따로 두고, 두 로봇을 하나의 공통 지도 위에 그립니다.
- 두 로봇의 지도 내용(해시)이 다르면 그 로봇은 지도에 그리지 않고 목표 전송도 막습니다.
- 위치는 odom이 아니라 `map → base_link` TF로 구합니다. 최근 odom이나 TF가 없으면 마커를 숨깁니다.
- AMCL은 `set_initial_pose: false`로 실행해서, 두 로봇이 원점에 있다고 가정하지 않습니다. 위치를 이미 알 때(시뮬)는 `robot1_initial_pose:=x,y,yaw`로 넘기면 그 로봇만 켜지자마자 그 자리로 잡습니다.

## 실행 (실물 로봇)

같은 도메인에서 Nav2를 두 번 띄우지 않도록, 이미 켜져 있는 Nav2/AMCL launch는 먼저 끕니다.

```bash
source ~/giddongcar/pinky_pro/install/setup.bash
ros2 launch pinky_fleet multi_robot.launch.py
```

브라우저에서 http://localhost:8080 을 엽니다.

1. robot1 도구줄의 **↗ 초기 위치**를 누르고, 지도에서 로봇이 실제로 있는 곳을 누른 채 바라보는 방향으로 끌었다가 놓습니다.
2. robot2도 같은 방법으로 설정합니다.
3. 지도에 두 로봇(① ②)이 뜨면 **⚑ 목적지**로 목표를 같은 방법으로 지정합니다(끄는 방향 = 도착 방향).
4. launch 터미널에서 Ctrl+C를 누르면 Nav2 2개와 대시보드가 함께 종료됩니다.

인자를 바꾸려면:

```bash
ros2 launch pinky_fleet multi_robot.launch.py map:=/절대경로/map.yaml port:=8090 robot1_domain:=15 robot2_domain:=17
```

기본 지도는 이 패키지의 `maps/good3.yaml`입니다.

## 대시보드 사용법

화면 머리줄의 **?**(도움말)에도 같은 내용이 있습니다.

- 지도 위 도구줄에 로봇마다 **[↗ 초기 위치] [⚑ 목적지]** 버튼이 있습니다. 초기 위치는 RViz의 2D Pose Estimate, 목적지는 Nav2 Goal과 같습니다.
- 지도에서 누른 채 끌면 위치와 방향이 함께 정해집니다. 끌지 않고 클릭만 하면 로봇의 지금 방향을 그대로 씁니다.
- 도구는 한 번 쓰면 꺼집니다. 이어서 여러 번 찍으려면 Shift를 누른 채 놓습니다. Esc는 도구만 끄고 Nav2 목표는 그대로 둡니다.
- **■ 이동 취소**는 Nav2 목표만 취소합니다. 하드웨어 비상정지가 아닙니다. 텔레옵으로 몰았다면 그 터미널에서 0 속도를 따로 보냅니다.
- 지도 바로 아래 좌표 줄에 두 로봇의 x, y, 방향과 커서 좌표가 나옵니다(m, °).
- 단축키: `1` `2` 로봇 선택, `P` 초기 위치, `G` 목적지, `Esc` 도구 끄기, `F` 지도 맞춤, `X` 선택한 로봇 이동 취소(`Shift+X` 두 로봇), `?` 도움말.

## 연결 확인

PC와 로봇 사이에 DDS 검색과 센서 데이터 통신이 되어야 합니다. 키보드 조종이 된다고 해서 scan이나 TF 수신까지 된다는 보장은 없습니다.

```bash
ROS_DOMAIN_ID=15 ros2 topic echo /scan sensor_msgs/msg/LaserScan --once
ROS_DOMAIN_ID=17 ros2 topic echo /scan sensor_msgs/msg/LaserScan --once
ROS_DOMAIN_ID=15 ros2 run tf2_ros tf2_echo map base_link
ROS_DOMAIN_ID=17 ros2 run tf2_ros tf2_echo map base_link
```

## 카메라 영상

카메라를 제어하려면 **각 로봇의 BLE 서비스가 `status`와 `set_camera` 명령을 지원해야** 합니다. PC Bluetooth가 켜져 있어야 하며, 저장소 루트의 `.venv`에 설치한 `bleak`을 사용합니다. 대시보드의 **카메라 시작/중지** 버튼은 BLE에서 Pinky를 찾고 응답 IP가 설정한 로봇 IP와 맞는지 확인한 뒤 명령을 보냅니다. 스트림 주소는 BLE 응답 URL을 우선 사용하고, 응답에 주소가 없을 때만 `http://<로봇IP>:5000/`을 기본값으로 사용합니다.

로봇의 BLE 서버 파일(`/opt/pinky-ble/ble_server.py`)은 이 저장소에 포함되지 않으며, 로봇마다 별도로 설치·관리합니다. 두 로봇 모두 카메라 제어가 가능한 같은 버전의 서비스를 실행해야 합니다. `unknown cmd: set_camera`는 해당 로봇의 BLE 서비스가 구버전이라는 뜻이므로, 그 로봇의 서비스를 갱신하고 재시작하세요.

로봇 카메라는 한 프로그램만 점유할 수 있으므로 대시보드에서 켜기 전에 Pinky Studio의 영상 창을 닫으세요. 대시보드는 로봇별 영상 스트림을 한 번만 받고, 여러 브라우저 창은 대시보드가 보관한 최신 프레임을 공유합니다.

실물 PC의 IP와 주행 설정은 기본적으로 `~/.config/pinky_fleet.env`에 둡니다. 이 파일은 Git에 올리지 않습니다. 다른 설정 파일은 `PINKY_FLEET_ENV=/경로/파일`로 지정할 수 있습니다. `ROBOT1_IP`와 `ROBOT2_IP` 값은 DDS peer와 카메라 주소에 사용됩니다.

| 설정 | 기본값 | 설명 |
|---|---:|---|
| `ROBOT1_IP`, `ROBOT2_IP` | 필수 | 각 로봇의 Wi-Fi IP |
| `ROBOT1_DOMAIN`, `ROBOT2_DOMAIN` | `15`, `17` | 각 로봇의 ROS 2 도메인 |
| `FLEET_PORT` | `8080` | 대시보드 포트 |
| `robot1_camera_host`, `robot2_camera_host` | 각 로봇 IP | 영상 연결 주소. 비우면 해당 카메라를 사용하지 않음 |
| `camera_port` | `5000` | BLE 응답에 URL이 없을 때 쓰는 기본 카메라 포트 |
| `host` | `127.0.0.1` | 대시보드 bind 주소. 기본값은 PC 내부에서만 접속 가능 |
| `map`, `params_file` | 패키지 기본 지도·Nav2 설정 | 지도 파일과 Nav2 파라미터 |
| `robot1_initial_pose`, `robot2_initial_pose` | 비어 있음 | 초기 위치 `x,y,yaw` |
| `auto_spin` | `true` | 위치를 모를 때 로봇이 자동으로 한 바퀴 돌아 위치를 찾음 |
| `traffic_zones` | good3 구역 파일 | 로봇 간 교통 정리 구역. `traffic_zones:=`로 끌 수 있음 |

`start_fleet.sh`의 설정은 실행 시 launch 인자로 덮어쓸 수 있습니다.

```bash
ros2 launch pinky_fleet multi_robot.launch.py \
  robot1_camera_host:=<로봇1주소> robot2_camera_host:=<로봇2주소> camera_port:=5000
```

예: `fleet auto_spin:=false camera_port:=5001`. 일반 실행은 [실물 실행 안내](../../../docs/real.md)의 설정 파일과 `fleet` alias를 사용하면 됩니다.

카드를 통해 카메라를 켜면 대시보드가 로봇별 스트림을 한 번만 받고, 브라우저는 대시보드의 `/camera/robot1.jpg` 또는 `/camera/robot2.jpg`에서 약 150 ms 간격으로 최신 JPEG를 가져옵니다. 상태 줄의 `브라우저 표시 WxH`는 브라우저가 프레임을 디코딩했다는 뜻입니다. 영상 수신 상태와 브라우저 표시 상태를 따로 확인할 수 있습니다.

YOLO는 패키지의 고정 모델 경로 `pinky_fleet/models/yolo11n.pt`를 사용하며 실행 시 GPU가 있으면 Ultralytics가 자동으로 선택합니다. 모델 가중치가 없으면 첫 실행 때 인터넷에서 자동으로 내려받습니다. 차선·횡단보도 전용 모델은 아니며, YOLO 결과는 주행 명령에 연결되지 않습니다.

차선 안에서 Nav2 목표를 주행하고 횡단보도·장애물 정책을 적용하기 위한 데이터셋, 영상-로봇 좌표 보정, costmap 통합, 정지 감시 계획은 [실물 차선 인식·주행 계획](../../../docs/lane_aware_driving_plan.md)을 참고하세요.

## 독립 카메라·인식·차선 주행 실험

대시보드와 별도로 카메라 시작, 학습 모델 추론, 화면 표시를 하는 `vision_drive`를 실행할 수 있습니다. 첫 실행은 **관찰 모드**이며 속도 명령을 내지 않습니다. 학습된 Ultralytics segmentation 모델 파일은 직접 준비해 경로로 전달합니다. 모델에 적어도 `driveable_area` segmentation 클래스가 있어야 차선 추종 판단이 가능합니다. COCO 객체 모델용 기본 정지 클래스는 `person`, `bicycle`, `car`, `motorcycle`, `bus`, `truck`, `bench`, `backpack`, `suitcase`, `chair`입니다. 클래스 이름이 다르면 인자를 바꾸세요.

주행 실험은 Nav2/대시보드가 같은 로봇의 `/cmd_vel`을 발행하지 않는 상태에서만 가능합니다. 현재 구현은 지도 목적지를 쓰는 Nav2 주행이 아니라, 카메라 영상에서 주행 가능 mask의 중심을 따라가는 **저속 시각 추종 prototype**입니다. 카메라·추론·라이다가 stale하거나 주행 영역이 불분명하면 정지합니다. 로봇 watchdog이 확인되지 않은 상태에서 실행할 때는 `--confirm-attended-test-without-watchdog` 옵션을 추가해야 하며, 사람이 로봇 바로 옆에서 물리 비상정지를 잡고 감독하는 통제 구역에서만 사용하세요. 이 옵션은 PC나 네트워크가 끊겼을 때 정지를 보장하지 않습니다.

ROS 도메인은 해당 로봇 값(로봇1 `15`, 로봇2 `17`)을 사용하고, `ROS_STATIC_PEERS`에는 로봇 IP를 둡니다. Pinky Studio 영상 창은 닫아 두세요. `.pt` 모델 파일은 Git에 추가하지 않습니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/giddongcar/pinky_pro/install/setup.bash
export ROS_DOMAIN_ID=15 ROS_STATIC_PEERS=192.168.0.6
export PYTHONPATH="$HOME/giddongcar/.venv/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"

# 영상 + segmentation/객체 인식만 확인 (속도 명령 없음)
ros2 run pinky_fleet vision_drive --robot-ip 192.168.0.6 --model /경로/학습모델.pt --mode observe

# YAML preset으로 통제된 저속 lane-follow 실험. 로봇 옆에서 직접 감독할 때만 실행
ros2 run pinky_fleet vision_drive --robot-ip 192.168.0.6 \
  --preset ~/giddongcar/pinky_pro/src/pinky_fleet/config/vision_drive_supervised.yaml \
  --enable-motion --confirm-supervised-test --confirm-attended-test-without-watchdog
```

`--preset`은 모델, 모드, 횡단보도 정책, 속도 상한, 조향값을 불러옵니다. 명령행에서 지정한 값은 preset보다 우선합니다. 로봇 IP와 모션 허용·감독 확인 옵션(`--enable-motion`, `--confirm-supervised-test`, `--watchdog-verified`, `--confirm-attended-test-without-watchdog`)은 YAML에 넣을 수 없고 매번 명령행에서 직접 지정합니다. preset만 주면 주행 모드 확인에서 멈추고 로봇은 움직이지 않습니다. 관찰만 할 때는 `--mode observe`를 명령행에 추가해 preset의 `drive` 값을 덮어쓰고, `--model /경로/모델.pt`로 다른 모델을 사용할 수 있습니다. 설치된 preset은 `$(ros2 pkg prefix pinky_fleet)/share/pinky_fleet/config/vision_drive_supervised.yaml`에도 복사됩니다.

주변에 IP가 같은 Pinky가 여럿 있으면(공유기가 달라도 `192.168.0.x`가 겹칠 수 있음) 엉뚱한 로봇의 카메라에 명령이 갈 수 있습니다. 이때는 `--ble-name pinky_6422`처럼 로봇의 BLE 이름을 지정합니다. `tape_lane_drive`도 같은 인자를 받습니다.

`vision_drive`는 로컬 모델 추론을 위해 Ultralytics의 외부 DNS 연결 확인을 오프라인 모드로 실행합니다. 모델 파일과 Python 의존성이 이미 설치되어 있으면 인터넷 없이도 시작할 수 있습니다. 카메라 제어는 로봇과 BLE, 영상 수신은 같은 로컬 네트워크 연결이 필요합니다.

흰 테이프 차선 검출과 저속 라인 추종 실험은 별도 `pinky_driving` 패키지의 `tape_lane_drive`를 사용합니다. 실행법과 안전 조건은 [pinky_driving README](../pinky_driving/README.md)를 참고하세요.

관찰 창에서 `s`를 누르면 원본 프레임, 인식 오버레이, 탐지 결과 JSON을 `~/vision_drive_observations`에 저장합니다. `r`은 원본/오버레이 MP4 녹화를 시작·종료하고, `q`는 종료합니다. 저장 위치는 `--output-dir`로 바꿀 수 있습니다. 예를 들어 YOLOTL 가중치는 클래스가 `lane`이므로 아래처럼 관찰할 수 있습니다.

```bash
ros2 run pinky_fleet vision_drive --robot-ip 192.168.0.6 \
  --model "$HOME/.cache/pinky_fleet/yolotl/weights.pt" \
  --driveable-class lane --mode observe
```

YOLOTL의 `lane` 마스크는 도로/BEV 시점의 차선 데이터로 학습된 결과이며 Pinky 바닥의 주행 가능 영역으로 검증되지 않았습니다. 이 명령은 관찰·샘플 저장용입니다. 저장한 원본 프레임을 코스 장면별로 선별·라벨링해 전용 모델을 학습해야 합니다. COCO 기본 모델은 흰 테이프 차선, 횡단보도, 학습되지 않은 사용자 정의 장애물을 인식하지 않으며, YOLOTL은 `lane`만 탐지합니다. 실물 장애물 안전 정지는 LiDAR/Nav2 경로를 유지해야 합니다.

`q` 또는 Ctrl+C로 종료하면 0 속도를 반복 발행하고 카메라 중지를 요청합니다. 정상 종료 시 동작이며 강제 종료 시 정지를 보장하지 않습니다. 저속 실험에서 최대 전진 속도는 `0.05 m/s`, 최대 각속도는 `0.25 rad/s`로 제한됩니다. `--steering-gain` 기본값은 `1.2`이며, 영상 오버레이의 자홍색 선이 먼 쪽 mask를 바탕으로 계산한 조향 미리보기입니다. 급회전에서는 `v / |w|`가 기본 반경 `0.08 m`를 넘지 않게 전진 속도를 자동으로 줄여 넓게 밀고 나가는 동작을 억제합니다. `--turn-radius-limit`으로 바꿀 수 있습니다. `--stop-distance`는 0.20 m 이상이어야 합니다. 전방 LiDAR는 인접 측정값들이 이어진 물체의 가로 폭이 `--obstacle-min-width` 이상일 때만 장애물로 처리하며, 기본값은 0.12 m입니다. 바닥의 작은 점/테이프 자국 같은 작은 반사물은 무시될 수 있습니다. 더 큰 물체만 정지 대상으로 잡으려면 이 값을 키우세요. 작은 장애물은 감지하지 못할 수 있으므로 주변을 비운 감독 실험에서만 사용하세요. 기본 횡단보도 정책은 `stop-then-go`이며 10초 정지 후 재개합니다. `--crosswalk-action slow|stop|stop-then-go|ignore`와 `--crosswalk-stop-seconds`로 바꿀 수 있습니다. 주행 영역이 잠깐 사라지면 마지막 조향 방향으로 최대 0.6초 제자리 재탐색한 뒤, 복구되지 않으면 정지합니다. 속도·정지 기준의 현장 성능을 확인하고 이 prototype 결과를 대시보드의 Nav2 경로에 연결하려면 별도 보정 및 안전 검토가 필요합니다. [MVP 요구사항 대응표와 실행 예시](../../../docs/vision_drive_mvp.md)를 참고하세요.

### 내려받은 공개 차선 모델

실외 자동차 영상으로 학습한 YOLOv11 segmentation 예제를 `models/lane_yolo11n_seg_best.pt`에 내려받았습니다. 원본 저장소에는 좌·우 실선/점선 네 종류 클래스가 기록되어 있습니다. 이 모델은 `driveable_area` 클래스가 없어 현재 도구의 `drive` 모드에는 맞지 않으므로 **실물 주행에 사용하지 마세요**. `observe` 모드에서 출력 형식과 오버레이를 살펴보는 용도로만 두었습니다. 파일은 `.gitignore` 대상이라 Git에 커밋되지 않으며, 필요하면 원본 [저장소](https://github.com/SyedaEmanSaleem/Lane-Detection-Segmentation-using-YOLOv11)에서 다시 받을 수 있습니다. 원본 저장소는 MIT 라이선스를 표시합니다.


YOLO, CUDA용 PyTorch, BLE 라이브러리는 저장소 루트에서 한 번 설치합니다. ROS Python과 패키지를 공유하도록 `--system-site-packages` 가상환경을 사용합니다. 아래 CUDA 13.0 설치 명령은 현재 관제 PC에서 확인한 조합입니다. 다른 PC에서는 [PyTorch 설치 선택기](https://pytorch.org/get-started/locally/)에서 OS·pip·Python·CUDA를 선택해 해당 명령을 쓰세요.

```bash
cd ~/Desktop/giddongcar
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu130
.venv/bin/python -m pip install ultralytics bleak
```

GPU 사용 가능 여부와 설치 버전은 아래처럼 확인합니다. `CUDA 사용 가능: True`면 GPU 가속이 가능합니다.

```bash
source /opt/ros/jazzy/setup.bash
PYTHONPATH="$PWD/.venv/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -c "import torch, ultralytics; print('Ultralytics', ultralytics.__version__); print('PyTorch', torch.__version__); print('CUDA 사용 가능:', torch.cuda.is_available())"
```

실물 관제는 `scripts/start_fleet.sh`가 venv 경로를 ROS Python에 자동으로 추가합니다. 직접 `ros2 launch`할 때는 위의 `PYTHONPATH` 설정을 같은 터미널에서 먼저 실행해야 합니다. `.pt` 가중치는 대용량 파일이므로 Git에 넣지 않습니다. 가중치가 없으면 Ultralytics가 `YOLO('yolo11n.pt')` 로딩 중 자동으로 내려받습니다. 첫 다운로드에는 인터넷이 필요합니다. 다운로드나 모델 초기화에 실패해도 카메라와 Nav2는 계속 실행되며 YOLO 상태에 오류가 표시됩니다.

### 문제 확인

- `unknown cmd: set_camera`: 해당 로봇의 BLE 서비스가 카메라 제어를 지원하지 않습니다. `/opt/pinky-ble/ble_server.py`를 카메라 지원 버전으로 갱신하고 BLE 서비스를 재시작하세요.
- `카메라 꺼짐` 또는 `Connection refused`: BLE 시작 응답과 대시보드에 표시된 로봇 주소를 확인하고, 기본 포트를 쓰는 경우 로봇에서 `sudo ss -ltnp | grep ':5000'`을 확인하세요. BLE 응답이 별도 주소를 반환하면 그 주소가 우선입니다.
- `영상 수신 중`인데 브라우저 표시 크기가 안 뜸: 대시보드 JPEG 요청이나 브라우저 디코딩 문제입니다. `브라우저 이미지 디코딩 실패`가 표시되면 JPEG 응답을 확인하세요.
- `:5000/`이 `<img src="/snapshot?...">`가 포함된 HTML을 반환하는 것은 정상일 수 있으며, 대시보드는 해당 `/snapshot`에서 JPEG를 받습니다. 이 로봇에서 확인한 Jupyter 포트는 `8888`입니다.
- 대시보드의 카메라와 YOLO는 관측 기능이며 속도 명령을 보내지 않습니다. 별도 `vision_drive --mode drive`는 위에 설명한 감독형 prototype 주행 모드입니다.

## 실행 (Gazebo, 로봇 2대)

```bash
ros2 launch pinky_fleet sim.launch.py
```

실제 방(good3 지도)과 같은 Gazebo 월드에 로봇 2대를 띄우고(도메인 25/27) 위와 같은 Nav2 2개와 대시보드를 시뮬 시간으로 실행합니다. 초기 위치는 생성 위치(robot1 (0.5, 0.5), robot2 (2.0, 0.5))로 자동으로 잡힙니다. 인자와 주의사항은 [docs/sim.md](../../../docs/sim.md).

## 아직 안 되는 것

- 두 로봇의 작업 배정이나 좁은 통로 통행 우선순위 조정은 없습니다.
- 대시보드의 `nav_ready`는 Nav2 액션 서버가 보이는지만 봅니다. Nav2 내부 활성화 실패는 잡지 못합니다.

## 테스트

```bash
cd ~/giddongcar/pinky_pro/src/pinky_fleet
python3 -m pytest test
```
