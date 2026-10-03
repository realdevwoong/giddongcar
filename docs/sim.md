# Gazebo 시뮬레이션

PC 한 대로 로봇 2대를 돌린다. 실물과 같은 구조(로봇마다 도메인 하나, 토픽·프레임 이름 동일)라서 관제 코드가 그대로 돈다. 도메인 번호만 다르다: **시뮬 robot1 = 25, robot2 = 27** (실물은 15/17).

주행 문제와 바꾼 값(문에서 멈춤, 구석, 양보 규칙, 시계 경고 등)은 [sim_tuning.md](sim_tuning.md)에 사진과 함께 정리했다.

## 준비 (최초 1회)

```bash
source /opt/ros/jazzy/setup.bash
cd ~/giddongcar/pinky_pro && colcon build --symlink-install
source install/setup.bash
```

## 로봇 2대 + 관제 (한 명령)

```bash
ros2 launch pinky_fleet sim.launch.py
```

브라우저에서 http://localhost:8080 을 연다. 버튼·단축키는 [대시보드 사용법](../pinky_pro/src/pinky_fleet/README.md#대시보드-사용법).

1. 초기 위치는 **자동으로 잡힌다**(생성 위치 robot1 (0.5, 0.5), robot2 (2.0, 0.5)). 지도에 두 로봇(① ②)이 뜨면 준비 끝.
2. 로봇 도구줄의 **⚑ 목적지**(`G`)를 누르고 지도에서 누른 채 끌어 목표와 도착 방향을 정한다. 도구는 한 번 쓰면 꺼진다. 두 로봇에 동시에 보내도 된다.
3. 가제보에서 로봇을 손으로 옮겼거나 지도 위 위치가 실제와 다르면 **↗ 초기 위치**(`P`)로 다시 찍는다.
4. **■ 이동 취소**는 Nav2 목표만 취소한다(비상정지 아님).

로봇 뒤 램프 색이 목표 상태를 따라 바뀐다(이동 중 파랑 깜빡임, 도착 초록, 실패 빨강 깜빡임, 대기 흰색 숨쉬기). 도착·취소 뒤 약 5초면 흰색 숨쉬기로 돌아가고, 실패하면 다음 목표를 보낼 때까지 빨강으로 남는다(약 15분 뒤 Nav2가 지난 결과를 지우면 흰색으로 돌아간다). 실물 `pinky_lamp_control`과 같은 `set_lamp` 서비스를 시뮬 전용 `sim_lamp` 노드가 받아 가제보 색으로 바꾼다. 제조사 램프 플러그인(초록 숨쉬기)은 시뮬 URDF에서 뺐다.

| 인자 | 기본 | 뜻 |
|---|---|---|
| `gui:=false` | true | Gazebo 화면 없이 |
| `fleet:=false` | true | 로봇만 띄움(실물 전원만 켠 상태와 같음). Nav2·대시보드는 따로: `multi_robot.launch.py use_sim_time:=true robot1_domain:=25 robot2_domain:=27 robot1_initial_pose:=0.5,0.5,0 robot2_initial_pose:=2.0,0.5,0` |
| `headless_rendering:=true` | false | 디스플레이 없는 PC에서 라이다·카메라 렌더링 |
| `robot2_x:= robot2_y:= robot2_yaw:=` | 2.0, 0.5, 0 | 생성 위치 (robot1은 0.5, 0.5, 0) |
| `world:= map:=` | good_map / good3 | 다른 월드를 쓰면 그 월드 지도도 같이 |

## 월드

기본 월드 `pinky_fleet/worlds/good_map.world`는 **실제 방 지도 `good3`의 검은 칸을 그대로 벽으로 세운 것**이다. 시뮬과 실물이 같은 지도(`maps/good3.yaml`)를 쓰므로 학원에서는 초기 위치만 다시 찍으면 된다.

- 왼쪽 방과 오른쪽 방은 **위쪽 통로(x≈1.6, y 0.9~1.2, 폭 0.35 m) 하나로만** 이어진다. 한 번에 한 대만 지나간다. 오른쪽 방은 칸막이(x≈2.33, y 0.63 위쪽)로 나뉘어 아래쪽으로 돌아간다. 왼쪽 방 가운데 장애물은 책상 윤곽이 벽으로 세워진 것이다.
- 측정(2026-09-26): 한 대씩은 통로를 지나간다(왼→오 18초). 오→왼은 한 번 "collision ahead"로 실패했다가 재시도에 성공했다. **두 대가 반대 방향으로 동시에 들어가면 통로에서 서로 막혀 둘 다 실패한다.** 교통 정리가 필요한 이유다([structure.md](structure.md) 2단계).
- 원본은 `pinky_gz_sim/worlds/good_map.world`(`inwoong/map_to_world.py`로 생성). 이 복사본은 물리 step(1ms → 4ms)과 색(벽 어두운 남색, 바닥 흰색 — 원본은 둘 다 회색이라 구분이 안 됐다)을 바꿨다. 가제보가 step마다 `/clock`을 보내는데, 초당 1000번이면 시계를 받는 파이썬 노드 하나가 CPU 40%를 먹는다. 로봇이 느려서(초속 0.1~0.3 m) 4ms면 충분하다.
- 지도를 새로 만들면(SLAM) 월드도 다시 만든다: `python3 inwoong/map_to_world.py <지도.yaml> -o <월드>` 뒤 물리 step을 4ms로 고친다.
- 제조사 공장 월드(선반·카메라에 볼거리가 있다)도 쓸 수 있다. 지도가 다르니 같이 넘긴다.
  ```bash
  ros2 launch pinky_fleet sim.launch.py \
    world:=$(ros2 pkg prefix pinky_fleet)/share/pinky_fleet/worlds/pinky_factory.world \
    map:=$(ros2 pkg prefix pinky_navigation)/share/pinky_navigation/map/my_map.yaml \
    robot1_x:=0 robot1_y:=0 robot2_x:=0 robot2_y:=-1.0
  ```

## 격리: 실물 로봇과 절대 섞이지 않게

시뮬 Nav2가 실물 로봇에 `cmd_vel`을 보내는 사고를 두 겹으로 막는다.

1. **도메인이 다르다.** 시뮬 25/27, 실물 15/17. 어느 쪽 설정이 어떻든 서로 못 만난다. 이 PC에서 실물용 터미널(도메인 15/17)을 같이 써도 된다.
2. **`sim.launch.py`가 자식 프로세스를 이 PC 안에 가둔다.** `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`를 걸고 `ROS_STATIC_PEERS`, `FASTRTPS_DEFAULT_PROFILES_FILE`, `ROS_DISCOVERY_SERVER`, `ROS_SUPER_CLIENT`, `CYCLONEDDS_URI`를 지운다.

`multi_robot.launch.py use_sim_time:=true`를 직접 띄울 때는 위 설정이 안 돼 있거나, 도메인이 실물용 15/17이면 실행을 거부한다(`robot1_domain:=25 robot2_domain:=27`을 준다).

시뮬 도메인을 굳이 15/17로 바꾸지 말 것. LOCALHOST만으로는 실물 로봇이 이 PC를 peer로 알고 먼저 찾아오는 경우를 못 막는다.

⚠️ `inwoong/run_fleet_sim.sh`는 도메인 15/17을 격리 없이 쓴다(포트도 8080). 로봇 공유기에 붙은 PC나 `ROS_STATIC_PEERS`가 설정된 셸에서는 돌리지 않는다. 로봇 2대 시뮬은 `sim.launch.py`를 쓴다.

## 디버그 터미널

시뮬 토픽을 다른 터미널에서 보려면 같은 격리 설정이 필요하다. 설정이 다르면 토픽이 안 보인다.

```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_STATIC_PEERS FASTRTPS_DEFAULT_PROFILES_FILE ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT CYCLONEDDS_URI
ros2 daemon stop
ROS_DOMAIN_ID=25 ros2 topic echo /scan --once
ROS_DOMAIN_ID=27 ros2 run tf2_ros tf2_echo odom base_footprint
ROS_DOMAIN_ID=25 ros2 run teleop_twist_keyboard teleop_twist_keyboard   # robot1만 움직이면 정상
ROS_DOMAIN_ID=25 ros2 run rqt_image_view rqt_image_view /camera/image_raw   # robot1 카메라
```

카메라 브리지는 `lazy`라서 누가 `/camera/image_raw`를 구독할 때만(rqt_image_view, `ros2 topic hz` 등) 가제보가 그 로봇 카메라를 렌더링한다. 보는 동안에만 CPU가 크게 는다(두 대 카메라가 늘 켜져 있던 때 공장 월드 기준 55% → 132%). 해상도를 낮추는 것은 다음 단계.

`--no-daemon`은 발견이 덜 된 채 결과를 낼 때가 있다. 위처럼 daemon을 껐다 켜고 몇 초 뒤에 본다.

## 제조사 방식 (로봇 1대, 지도 만들기·카메라 확인)

```bash
ros2 launch pinky_gz_sim launch_sim.launch.xml                # 기본 월드 pinky_factory, /camera/image_raw 포함
ros2 launch pinky_navigation gz_map_building.launch.xml       # SLAM
ros2 launch pinky_navigation gz_map_view.launch.xml           # RViz
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run nav2_map_server map_saver_cli -f <경로/지도이름>
ros2 launch pinky_navigation gz_bringup_launch.xml            # Nav2 (기본 지도 my_map)
ros2 launch pinky_navigation gz_nav2_view.launch.xml
ros2 run rqt_image_view rqt_image_view                        # 창에서 /camera/image_raw 선택
```

이 방식은 격리를 자동으로 하지 않는다. 위 디버그 터미널 설정을 먼저 한다.

## 자주 보는 메시지

- `at least 2 nodes with the name /robot_state_publisher`: 도메인이 달라 실제 충돌은 없다. 무시.
- 종료할 때 Nav2 컨테이너가 `Magick: abort due to signal 11`을 내며 죽는다. Nav2 쪽 문제로, 이미 정리가 끝난 뒤라 무시.
- 토픽이 안 보이면 격리 env가 같은지 확인하고 `ros2 daemon stop` 후 다시 본다.
- Gazebo를 일시정지하면 `/odom`이 멈춰 대시보드에 "연결 끊김"(odom 2초 넘게 없음)이 뜬다. 정상. 가제보 창 왼쪽 아래 ▶로 다시 돌린다.
- 초기 위치는 10분 안에 찍으면 된다. 그 뒤엔 Nav2 costmap이 포기하고 되살아나지 않는다(다시 launch).
