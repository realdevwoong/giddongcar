# 코드는 어디에 넣나

## 원칙

코드를 "어디서 도는지"로 나눈다.

| 어디서 도나 | 예 | 패키지 |
|---|---|---|
| **PC에서, 여러 대를 조율** | 관제 UI, 로봇별 Nav2 실행, 두 대 충돌 방지, 시뮬 launch, 지도 | `pinky_fleet` |
| **센서 처리** (PC 또는 로봇) | 카메라 영상 구독, 객체·차선 탐지, 탐지 결과 발행 | `pinky_camera` (새로 만들 것) |
| **로봇 한 대 안에서** | 모터, 라이다, odom, LED, LCD | 제조사 `pinky_bringup` 등. 고치지 않는다 |
| **제조사 Nav2 설정** | `nav2_params.yaml`, `bringup_launch.xml` | `pinky_navigation`. 값을 바꾸고 싶으면 원본 대신 `pinky_fleet/params/`에 복사본을 두고 `params_file:=`로 넘긴다 |
| **제조사 설정 파일의 팀 수정본** | 가제보 월드(물리 step), Nav2 params | `pinky_fleet/worlds/`, `pinky_fleet/params/`. 복사본 첫 줄에 원본과 무엇이 다른지 적는다 |

이름은 `pinky_<역할>`. 사람 이름 폴더는 만들지 않는다.

## 자주 나올 질문

**주행 알고리즘을 새로 짜면?**
- Nav2 위에서 "어디로 갈지"를 정하는 로직(순찰 순서, 작업 배정, 두 대 교차 시 누가 먼저 갈지) → `pinky_fleet/pinky_fleet/<이름>.py` 노드. Nav2에 목표를 보내는 쪽이라 관제에 속한다.
- Nav2 안의 동작 자체를 바꾸는 것(경로 계획기, 경로 추종 컨트롤러 교체) → Nav2 플러그인은 C++ 이라 별도 `pinky_nav_plugins`(ament_cmake) 패키지. 먼저 `nav2_params.yaml`의 기존 플러그인 파라미터 튜닝으로 되는지 본다.
- Nav2를 안 쓰는 단순 주행(라인 트레이싱, 카메라 보고 직접 `cmd_vel`) → `pinky_camera`에서 탐지하고, 주행 결정 노드는 `pinky_driving`(새 패키지)에. 탐지와 주행을 분리해야 각각 시뮬에서 따로 테스트할 수 있다.

**두 대가 부딪히지 않게 하려면?**
- 1단계: 각 Nav2가 라이다로 상대를 장애물로 본다(지금 상태).
- 2단계: `pinky_fleet`에 교통 정리 노드. 두 로봇의 위치·계획 경로를 보고 좁은 구간에서 한 대를 잠깐 멈춘다(Nav2 목표 취소/재전송).
- 3단계: 상대 로봇 위치를 서로의 costmap에 넣는다(costmap 플러그인 또는 `PointCloud`로 발행).

**카메라 영상을 화면에 띄우려면?**
- 로봇(또는 시뮬 브릿지)이 `camera/image_raw`를 낸다 → `pinky_camera`가 압축·탐지 → `pinky_fleet` 대시보드가 받아서 표시한다.
- WiFi를 지날 때는 raw가 아니라 compressed 토픽을 쓴다.

## 패키지 안 구조 (ament_python)

```
pinky_<역할>/
├── package.xml            # 의존성. 새 import를 쓰면 여기에도 추가
├── setup.py               # 설치할 파일(launch/, params/, web/ …)과 실행 파일 이름(entry point)
├── setup.cfg
├── resource/pinky_<역할>  # 빈 파일. ROS가 패키지를 찾는 표식
├── pinky_<역할>/          # 파이썬 코드. 노드 하나 = 파일 하나, main() 함수
├── launch/                # *.launch.py
├── params/                # *.yaml
├── test/                  # pytest. DDS 없이 도는 단위 테스트
└── README.md              # 실행 방법
```

새 패키지 만들기:

```bash
cd ~/giddongcar/pinky_pro/src
ros2 pkg create --build-type ament_python pinky_camera --dependencies rclpy sensor_msgs cv_bridge
```

만든 뒤 `setup.py`, `package.xml`은 `pinky_fleet`을 참고해서 채운다.

## 노드 하나를 쓸 때 체크리스트

- 토픽 이름은 상대 이름(`camera/image_raw`). 로봇 2대를 네임스페이스로 구분할 수 있게.
- 경로·IP·도메인은 파라미터나 launch 인자로.
- `use_sim_time`은 launch에서 넘긴다. 코드에서 시간을 잴 때는 `node.get_clock().now()`.
- 시뮬에서 먼저 돌려보고 실물로 간다.
