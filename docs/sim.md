# Gazebo 시뮬레이션

PC 한 대로 로봇 2대를 돌린다. 실물과 같은 구조(로봇마다 도메인 하나, 토픽·프레임 이름 동일)라서 관제 코드가 그대로 돈다. 도메인 번호만 다르다: **시뮬 robot1 = 25, robot2 = 27** (실물은 15/17).

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

브라우저에서 http://localhost:8080 을 연다.

1. robot1 선택 → "초기 위치 설정" → 지도의 (0, 0)을 누르고 +x 방향(오른쪽)으로 드래그. 로봇이 생성되는 위치라 정확히 맞다.
2. robot2도 같은 방법으로 (0, -1.0).
3. "목적지 지정"으로 목표를 드래그한다. 두 로봇에 동시에 보내도 된다.

| 인자 | 기본 | 뜻 |
|---|---|---|
| `gui:=false` | true | Gazebo 화면 없이 |
| `fleet:=false` | true | 로봇만 띄움(실물 전원만 켠 상태와 같음). Nav2·대시보드는 따로: `multi_robot.launch.py use_sim_time:=true robot1_domain:=25 robot2_domain:=27 map:=<my_map.yaml 경로>` |
| `headless_rendering:=true` | false | 디스플레이 없는 PC에서 라이다·카메라 렌더링 |
| `robot2_x:= robot2_y:= robot2_yaw:=` | 0, -1.0, 0 | 생성 위치 (robot1도 같은 식) |
| `world:= map:=` | pinky_factory / my_map | 다른 월드를 쓰면 그 월드 지도도 같이 |

가제보 월드 `pinky_factory`의 지도는 `pinky_navigation/map/my_map.yaml`이다. `good3`는 실제 방 지도라 시뮬에 쓰면 위치가 맞지 않는다.

기본 월드는 제조사 파일의 복사본 `pinky_fleet/worlds/pinky_factory.world`다. 물리 step만 1ms → 4ms로 바꿨다. 가제보가 step마다 `/clock`을 보내는데, 초당 1000번이면 시계를 받는 파이썬 노드 하나가 CPU 40%를 먹는다. 로봇이 느려서(초속 0.1~0.3 m) 4ms면 충분하다.

## 격리: 실물 로봇과 절대 섞이지 않게

시뮬 Nav2가 실물 로봇에 `cmd_vel`을 보내는 사고를 두 겹으로 막는다.

1. **도메인이 다르다.** 시뮬 25/27, 실물 15/17. 어느 쪽 설정이 어떻든 서로 못 만난다. 이 PC에서 실물용 터미널(도메인 15/17)을 같이 써도 된다.
2. **`sim.launch.py`가 자식 프로세스를 이 PC 안에 가둔다.** `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`를 걸고 `ROS_STATIC_PEERS`, `FASTRTPS_DEFAULT_PROFILES_FILE`, `ROS_DISCOVERY_SERVER`, `CYCLONEDDS_URI`를 지운다.

`multi_robot.launch.py use_sim_time:=true`를 직접 띄울 때는 위 설정이 안 돼 있으면 실행을 거부한다.

시뮬 도메인을 굳이 15/17로 바꾸지 말 것. LOCALHOST만으로는 실물 로봇이 이 PC를 peer로 알고 먼저 찾아오는 경우를 못 막는다.

## 디버그 터미널

시뮬 토픽을 다른 터미널에서 보려면 같은 격리 설정이 필요하다. 설정이 다르면 토픽이 안 보인다.

```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST; unset ROS_STATIC_PEERS FASTRTPS_DEFAULT_PROFILES_FILE
ros2 daemon stop
ROS_DOMAIN_ID=25 ros2 topic echo /scan --once
ROS_DOMAIN_ID=27 ros2 run tf2_ros tf2_echo odom base_footprint
ROS_DOMAIN_ID=25 ros2 run teleop_twist_keyboard teleop_twist_keyboard   # robot1만 움직이면 정상
```

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
- Gazebo를 일시정지하면 `/odom`이 멈춰 대시보드에 "odom 수신 끊김"이 뜬다. 정상.
- 초기 위치는 10분 안에 찍으면 된다. 그 뒤엔 Nav2 costmap이 포기하고 되살아나지 않는다(다시 launch).
