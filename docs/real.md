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

# 로봇에서 (선택) 상태 램프: 대시보드가 이동 중 파랑 깜빡임 / 도착 초록 / 실패 빨강 깜빡임으로 바꾼다
ROS_DOMAIN_ID=15 ros2 run pinky_lamp_control main_node                   # 로봇2는 17. 없으면 대시보드 카드에 "램프 노드 없음"
# ↑ 학원에서 처음 확인할 것: 램프 드라이버(커널 모듈, pinky_lamp_control/README.md)가 로봇 이미지에 설정돼 있는지, 일반 사용자로 켜지는지

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

## 학원 가서 할 일 (시뮬 → 실물 전환 체크리스트)

관제 코드는 시뮬과 같다. 아래 순서로 현장 보정만 한다.

1. **네트워크(가장 큰 변수).** 공유기 연결 → 로봇 IP 확인 → DDS peer 설정 → 시뮬 설정을 안 한 새 터미널에서 `ROS_DOMAIN_ID=15 ros2 topic echo /scan sensor_msgs/msg/LaserScan --once`가 되는지. 여기서 시간을 제일 많이 잡는다.
2. **지도.** 방이 그대로면 `good3`, 바뀌었으면 SLAM으로 새로 만들고 시뮬 월드도 다시 만든다([sim.md](sim.md) 월드). 초기 위치는 시뮬과 똑같이 대시보드에서 찍는다.
3. **안전 연습.** 로봇을 움직이기 전에 0 속도 발행(아래)을 한 번 해 본다. 주행 중에는 한 사람이 그 터미널 앞에 있는다.
4. **첫 목표.** 관제 화면에 로봇 2대가 뜨고, 목표 하나씩 도착하면 첫날 목표 달성. 좁은 통로에는 한 대씩 보낸다.
5. **주행 보정.** 미끄러짐·라이다 노이즈·WiFi 지연 때문에 속도 상한, footprint, inflation을 조정한다. 제조사 `nav2_params.yaml`을 `pinky_fleet/params/`에 복사해 고치고 `params_file:=`로 넘긴다(복사본은 아직 없다).
6. **카메라.** 로봇에서 카메라 노드(만들 예정, `pinky_camera`)를 처음 돌리며 해상도·프레임 속도를 맞춘다. 탐지 모델은 실물 영상으로 학습한다(영상·가중치는 커밋하지 않는다).

시뮬로는 못 잡는 것: WiFi 끊김·공유기 과부하·DDS 발견 실패, 실제 충돌과 배터리, 조명에 따른 탐지 성능.

## 안전

- ⚠️ `pinky_bringup`은 `cmd_vel` 타임아웃이 없다. 텔레옵을 끈 뒤 0 속도를 한 번 보낸다.
  ```bash
  ROS_DOMAIN_ID=15 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
  ```
- 대시보드의 "이동 취소"는 Nav2 목표만 취소한다. 비상정지가 아니다.
- `pkill -9`로 Nav2를 죽이면 0 속도가 나가지 않아 로봇이 계속 달린다. 정리는 launch 터미널에서 Ctrl+C로 하고, 그래도 썼다면 도메인마다 0 속도를 보낸다.
- 대시보드는 인증이 없다. `--host 0.0.0.0`으로 열면 같은 WiFi의 누구나 로봇을 움직일 수 있다. 기본값(127.0.0.1)을 유지한다.
- 시뮬을 돌리는 PC가 로봇 공유기에 붙어 있으면 반드시 격리한다. [docs/sim.md](sim.md)
