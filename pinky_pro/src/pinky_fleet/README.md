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
