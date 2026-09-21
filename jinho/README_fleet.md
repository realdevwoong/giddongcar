# PC에서 두 Pinky의 Nav2와 통합 관제 실행

## 실행

로봇1은 domain 15, 로봇2는 domain 17에서 기본 `pinky_bringup`만 실행합니다.
이미 실행 중인 로봇 또는 PC의 Nav2/AMCL/웹 Nav2 launch는 먼저 종료합니다.
같은 도메인에 Nav2를 중복 실행하지 마세요. 로봇의 기본 bringup은 유지합니다.

PC 터미널:

```bash
cd /home/fastcampus/Desktop/ros2
bash start_fleet.sh
```

이 스크립트는 ROS Jazzy와 `~/pinky_pro/install/setup.bash`를 자동으로 불러옵니다.
launch를 직접 실행할 때는 같은 터미널에서 다음 환경 설정이 필요합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinky_pro/install/setup.bash
cd /home/fastcampus/Desktop/ros2
ros2 launch ./multi_robot.launch.py
```

브라우저에서 http://localhost:8080 을 엽니다. 별도 빌드나 Flask 설치는 필요 없습니다.
PC의 `good.yaml`과 그 안에서 참조하는 `good3.pgm`을 양쪽 Nav2에 전달합니다.
로봇에 지도를 복사할 필요가 없습니다.

1. robot1을 선택하고 **초기 위치 설정** 모드에서 실제 위치를 누른 뒤 바라보는 방향으로 드래그합니다.
2. robot2도 각각의 실제 위치·방향을 설정합니다.
3. 두 로봇의 마커를 확인한 뒤 **목적지 지정** 모드에서 선택한 로봇의 목표를 드래그합니다.
4. 선택/전체 이동 취소 버튼은 Nav2 목표를 취소합니다. 하드웨어 비상정지나 키보드 명령 정지는 아닙니다.
5. launch 터미널에서 Ctrl+C를 누르면 PC의 두 Nav2와 관제 서버가 함께 종료됩니다.

다른 지도/포트:

```bash
ros2 launch ./multi_robot.launch.py map:=/절대경로/map.yaml port:=8090
```

## 구조

- PC Nav2 #1: domain 15에서 로봇1의 scan/odom/TF를 직접 수신
- PC Nav2 #2: domain 17에서 로봇2의 scan/odom/TF를 직접 수신
- 웹 서버: 도메인별 ROS context와 TF buffer를 분리하고 하나의 공통 지도 위에 표시
- 지도 내용·해시가 다르면 해당 로봇을 겹쳐 그리지 않고 목표 전송을 차단
- 위치는 odom 좌표가 아닌 `map -> base_link` TF. 최근 odom과 TF가 없으면 마커를 숨김
- 원본 설정을 임시 복사해 `set_initial_pose: false`로 실행. 두 로봇을 원점으로 가정하지 않음
- 지도는 해시가 바뀔 때만 받으며 상태/계획 경로는 0.5초 간격으로 갱신

기존 `bridge.yml`은 변경하지 않았습니다. domain 0의 키보드/odom 도구를 계속 사용하려면
기존 브릿지를 별도로 실행할 수 있습니다. 웹 관제 자체는 이 브릿지가 필요하지 않습니다.
Nav2 자율주행 중에는 같은 로봇에 키보드 속도 명령을 동시에 보내지 마세요.

PC와 로봇 사이에 DDS 검색 및 센서 데이터 통신이 되어야 합니다.
키보드 제어만 성공해도 scan/TF 수신까지 보장되는 것은 아닙니다.
PC/로봇 시계도 동기화되어 있어야 합니다.

```bash
ROS_DOMAIN_ID=15 ros2 topic echo /scan sensor_msgs/msg/LaserScan --once
ROS_DOMAIN_ID=17 ros2 topic echo /scan sensor_msgs/msg/LaserScan --once
ROS_DOMAIN_ID=15 ros2 run tf2_ros tf2_echo map base_link
ROS_DOMAIN_ID=17 ros2 run tf2_ros tf2_echo map base_link
```

지도 표시와 개별 Nav2 목표 제어를 제공합니다. 두 로봇의 작업 배정이나
좁은 통로 통행 우선순위를 조정하는 기능은 포함하지 않습니다.
서버는 기본적으로 PC의 localhost에서만 접근 가능합니다.

## 검증

```bash
python3 -m unittest discover -s tests -v
ros2 launch ./multi_robot.launch.py --show-args
```

실제 위치/초기 위치 설정/주행 성공 여부는 연결된 로봇으로 확인해야 합니다.
