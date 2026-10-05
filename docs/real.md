# 실물 로봇

## 구성

| 기기 | ROS_DOMAIN_ID | 실행하는 것 |
|---|---|---|
| 로봇1 (Pinky Pro) | 15 | `pinky_bringup` (모터, 라이다, odom, TF) |
| 로봇2 (Pinky Pro) | 17 | `pinky_bringup` |
| 관제 PC | 프로세스마다 15 / 17 | `pinky_fleet multi_robot.launch.py`: Nav2 2개 + 웹 대시보드 |

Nav2는 로봇이 아니라 PC에서 돈다. 로봇에서는 bringup만 켠다.

## 순서

공유기에 인터넷이 없어도 된다. **처음 한 번 설정**(시계 자동 맞춤, 실행 스크립트)을 해 두면 그 뒤로는 로봇 bringup + PC에서 `fleet` 한 줄이다.

### 처음 한 번 (관제 PC와 로봇)

**① 시계 자동 맞춤.** 로봇 시계가 PC와 다르면 위치가 안 뜨거나 Nav2가 `Transform data too old`(TF 오류 102)로 목표를 바로 실패한다. PC를 시간 서버로 두고 로봇이 켜질 때마다 PC 시계를 따라가게 한다. 로봇이 이 PC IP로 시간을 받으므로 **PC IP를 공유기 DHCP 예약으로 고정**하고, 관제 PC는 이 한 대로 정한다.

```bash
# PC (인터넷 되는 곳에서 설치. 그 뒤로는 인터넷 없어도 PC 자기 시계로 알려 준다)
sudo apt install chrony
sudo tee /etc/chrony/conf.d/pinky.conf <<'CONF'
allow 192.168.0.0/24
local stratum 10
CONF
sudo systemctl restart chrony

# 로봇마다 (기본으로 들어 있는 systemd-timesyncd를 쓴다. 인터넷 필요 없음)
sudo mkdir -p /etc/systemd/timesyncd.conf.d
printf '[Time]\nNTP=192.168.0.<PC>\n' | sudo tee /etc/systemd/timesyncd.conf.d/pinky.conf
sudo systemctl restart systemd-timesyncd
timedatectl                                   # "System clock synchronized: yes"면 끝
```

급할 때 한 번만 맞추려면: `pinky_fleet/scripts/sync_robot_clock.sh 192.168.0.<로봇1> 15 192.168.0.<로봇2> 17` (PC에서, 로봇 비밀번호를 묻는다).

**② 실행 스크립트 설정.** 로봇 IP 확인: 공유기 관리 페이지(`ip route | grep default`의 게이트웨이) 또는 로봇에서 `hostname -I`.

```bash
cat > ~/.config/pinky_fleet.env <<'CONF'      # 이 PC 전용. git에 안 올린다
ROBOT1_IP=192.168.0.<로봇1>
ROBOT2_IP=192.168.0.<로봇2>
FLEET_PORT=8080                               # 8080을 다른 프로그램(Docker 등)이 쓰면 8081
CONF
echo "alias fleet='~/giddongcar/pinky_pro/src/pinky_fleet/scripts/start_fleet.sh'" >> ~/.bashrc
```

### 매번

```bash
# ── 1. 로봇에서 (ssh pinky@<IP>). 로봇2는 도메인 17
export ROS_DOMAIN_ID=15 ROS_STATIC_PEERS=192.168.0.<PC>
ros2 launch pinky_bringup bringup_robot.launch.xml
ros2 run pinky_lamp_control main_node        # (선택, 다른 ssh 창) 상태 램프. 없으면 카드에 "램프 응답 없음"
# ↑ 학원에서 처음 확인할 것: 램프 드라이버(커널 모듈, pinky_lamp_control/README.md)가 설정돼 있는지, 일반 사용자로 켜지는지

# ── 2. PC를 로봇 공유기 WiFi에 붙이고 새 터미널에서. ⚠️ 켜자마자 로봇이 제자리에서 한 바퀴 돈다(아래)
fleet                                         # launch 인자는 뒤에 붙인다. 예: fleet auto_spin:=false
```

`fleet`(`start_fleet.sh`)이 하는 일: Ctrl+Z로 멈춰 둔 예전 ROS 프로세스 정리(멈춘 프로세스도 DDS에 남아 로봇 데이터를 막는다) → PC가 공유기에 붙었는지·로봇 ping → `ROS_STATIC_PEERS` 채우기 → 도메인 15/17에 다른 PC의 AMCL이 없는지 → `multi_robot.launch.py` 실행(Ctrl+Z는 막아 둔다) → Ctrl+C로 끄면 도메인 15/17에 0 속도. 문제가 있으면 관제를 켜지 않고 이유를 알려 준다. 스크립트 없이 켜려면 `ROS_STATIC_PEERS`를 export하고 `ros2 launch pinky_fleet multi_robot.launch.py`.

- 교통 정리(칸 열쇠)는 기본으로 켜진다. 지도가 good3가 아니면 스스로 꺼지고 카드·상태에 "교통 정리 꺼짐"이 뜬다(그 지도용 칸 파일을 새로 만들어야 한다: `pinky_fleet/params/traffic_good3.yaml`).
- 끌 때: PC `fleet` 터미널에서 **Ctrl+C**(0 속도까지 스크립트가 보낸다. Ctrl+Z는 끄는 게 아니라 멈춰 두는 것이라 쌓인다) → 로봇 bringup Ctrl+C.

브라우저 http://localhost:8080 → 로봇마다 AMCL이 켜지면 **제자리에서 한 바퀴 돌며** 스스로 위치를 찾는다(AMCL 전역 위치 찾기, 끝나면 0 속도). ⚠️ 관제를 켜기 전에 로봇 주변을 비운다. 돌지 않게 하려면 `auto_spin:=false`(가만히 찾음). 확인되면 지도에 로봇이 나타난다. "위치 못 찾음"이면 로봇 주변을 보고 **⟳ 돌면서 찾기**(제자리 한 바퀴, ■ 이동 취소로 멈춤) 또는 **↗ 초기 위치**(`P`)로 직접 찍기. 나타난 위치가 틀렸어도 `P`로 고친다 → **⚑ 목적지**(`G`)로 목표 지정. 자세한 사용법은 [대시보드 사용법](../pinky_pro/src/pinky_fleet/README.md#대시보드-사용법).

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
5. **주행 보정.** 미끄러짐·라이다 노이즈·WiFi 지연 때문에 속도 상한, footprint, inflation을 조정한다. 팀용 복사본 `pinky_fleet/params/nav2_params.yaml`(관제 기본값)을 고친다. 바꾼 곳은 "팀:" 주석으로 남긴다(지금: 좁은 문에서 collision ahead 오판을 줄인 `failure_tolerance`, `max_allowed_time_to_collision_up_to_carrot`, 문을 나오며 코너를 질러 문틀에 붙지 않게 한 `min_lookahead_dist`, `use_regulated_linear_velocity_scaling`).
6. **카메라.** PC Bluetooth를 켜고, 대시보드 로봇 카드의 카메라 시작/중지 버튼을 쓴다. 관제가 BLE에서 Pinky를 찾아 응답 IP가 설정한 로봇 주소와 일치하는지 확인한 뒤 스트리밍을 제어한다. 로봇 이미지는 `pinky_pro_v1.9` 이상이어야 하며, 영상 주소는 BLE 카메라 응답 URL을 우선 사용하고, 응답에 주소가 없을 때만 `:5000/`을 시도한다. YOLO 가중치는 `pinky_fleet/models/yolo11n.pt`에서 자동으로 읽는다. COCO 모델 결과는 화면 확인용이고 주행에 반영되지 않는다. 차선·횡단보도는 실물 영상으로 별도 데이터·모델을 검증한다. 가중치 파일은 Git에 올리지 않는다. 상세 작업 목록은 [카메라·인식 기획](real_camera_yolo_plan.md).

시뮬로는 못 잡는 것: WiFi 끊김·공유기 과부하·DDS 발견 실패, 실제 충돌과 배터리, 조명에 따른 탐지 성능.

## 안전

- ⚠️ `pinky_bringup`은 `cmd_vel` 타임아웃이 없다. 텔레옵을 끈 뒤 0 속도를 한 번 보낸다.
  ```bash
  ROS_DOMAIN_ID=15 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
  ```
- 대시보드의 **■ 이동 취소**는 Nav2 목표만 취소한다. 비상정지가 아니다. Esc는 지도 도구만 끄고 로봇에는 아무것도 보내지 않는다.
- `pkill -9`로 Nav2를 죽이면 0 속도가 나가지 않아 로봇이 계속 달린다. 정리는 launch 터미널에서 Ctrl+C로 하고, 그래도 썼다면 도메인마다 0 속도를 보낸다.
- 대시보드는 인증이 없다. `--host 0.0.0.0`으로 열면 같은 WiFi의 누구나 로봇을 움직일 수 있다. 기본값(127.0.0.1)을 유지한다.
- 시뮬을 돌리는 PC가 로봇 공유기에 붙어 있으면 반드시 격리한다. [docs/sim.md](sim.md)
