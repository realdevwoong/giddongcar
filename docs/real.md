# 실물 로봇

## 구성

| 기기 | ROS_DOMAIN_ID | 실행하는 것 |
|---|---|---|
| 로봇1 (Pinky Pro) | 15 | `pinky_bringup` (모터, 라이다, odom, TF) |
| 로봇2 (Pinky Pro) | 17 | `pinky_bringup` |
| 관제 PC | 프로세스마다 15 / 17 | `pinky_fleet multi_robot.launch.py`: Nav2 2개 + 웹 대시보드 |

Nav2는 로봇이 아니라 PC에서 돈다. 로봇에서는 bringup만 켠다.

## 순서

```bash
# 로봇에서 (ssh). 로봇 bashrc에 도메인이 이미 설정돼 있으면 앞부분 생략
ROS_DOMAIN_ID=15 ros2 launch pinky_bringup bringup_robot.launch.xml     # 로봇2는 17

# PC에서
source ~/giddongcar/pinky_pro/install/setup.bash
ROS_DOMAIN_ID=15 ros2 topic echo /scan sensor_msgs/msg/LaserScan --once  # 연결 확인 (17도)
ros2 launch pinky_fleet multi_robot.launch.py
```

브라우저 http://localhost:8080 → 로봇마다 초기 위치 설정 → 목적지 지정. 자세한 사용법은 [pinky_fleet/README.md](../pinky_pro/src/pinky_fleet/README.md).

## 네트워크 (공유기)

- 공유기가 DDS 멀티캐스트를 걸러서 PC와 로봇이 서로를 못 찾는다. 유니캐스트 peer 목록이 필요하다.
  - Fast DDS: `export ROS_STATIC_PEERS="로봇1IP;로봇2IP"`
  - Cyclone DDS: XML의 `<Peers>`
- IP가 바뀌면 peer 목록도 바뀐다. **근본 해결은 공유기 관리 페이지에서 DHCP 예약으로 로봇·PC IP를 고정하는 것.** 관리 페이지 주소는 공유기에 연결한 상태에서 `ip route | grep default`에 나오는 게이트웨이 주소.
- IP 표 (고정한 뒤 여기에 기록한다. 코드에는 적지 않는다):

| 기기 | IP | 비고 |
|---|---|---|
| 로봇1 | | 도메인 15 |
| 로봇2 | | 도메인 17 |
| 관제 PC | | |
| 노트북 2 | | |

- Cyclone DDS는 아직 미적용. 설정 초안이 PC마다 `~/.ros/cyclonedds_pc.xml`에 있다. 적용하려면:
  1. `sudo apt install ros-jazzy-rmw-cyclonedds-cpp`
  2. `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
  3. `export CYCLONEDDS_URI=file://$HOME/.ros/cyclonedds_pc.xml` (URI 안에서는 `~`가 풀리지 않는다)
  4. 로봇 2대도 같은 RMW로 맞춘다. 혼용은 권장되지 않는다.
  5. XML의 `<Peers>`가 IP 표와 맞는지 확인한다. 유니캐스트 모드에서는 한 PC의 노드가 10개를 넘으면 `MaxAutoParticipantIndex`도 올려야 한다.

## 안전

- ⚠️ `pinky_bringup`은 `cmd_vel` 타임아웃이 없다. 텔레옵을 끈 뒤 0 속도를 한 번 보낸다.
  ```bash
  ROS_DOMAIN_ID=15 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
  ```
- 대시보드의 "이동 취소"는 Nav2 목표만 취소한다. 비상정지가 아니다.
- 대시보드는 인증이 없다. `--host 0.0.0.0`으로 열면 같은 WiFi의 누구나 로봇을 움직일 수 있다. 기본값(127.0.0.1)을 유지한다.
- 시뮬을 돌리는 PC가 로봇 공유기에 붙어 있으면 반드시 격리한다. [docs/sim.md](sim.md)
