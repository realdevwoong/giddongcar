#!/usr/bin/env bash
# 로봇 2대(가제보 + nav2/웹브릿지) 를 터미널 하나에서 한 번에 백그라운드로 띄운다.
# Ctrl+C 하면 4개 프로세스 전부 종료됨. 각 프로세스 출력은 로그 파일로 분리 저장.
#
# 사용법:
#   ./run_fleet_sim.sh                       # 기본: good.yaml 맵 + 그 맵으로 만든 good_map.world
#   ./run_fleet_sim.sh /path/to/other.yaml   # 다른 맵 사용 시 map_to_world.py로 world도 새로 만들 것

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")/pinky_pro"

# ROS2 setup.bash는 set -u(nounset)와 호환되지 않으므로 여기선 켜지 않는다.
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

# good.yaml(관제 PC 기본 지도)과, 그걸 map_to_world.py로 그대로 벽까지 세운 good_map.world를
# 같이 쓴다 — Gazebo 3D 화면과 관제 GUI의 2D 지도가 같은 공간을 보여줘야 하기 때문.
DEFAULT_MAP="/home/devwoong/good.yaml"
WORLD_NAME="good_map.world"
MAP="${1:-$DEFAULT_MAP}"
LOG_DIR="/tmp/pinky_fleet_sim"
mkdir -p "$LOG_DIR"

# 이미 떠 있는 세션(또는 예전 세션의 고아 프로세스)이 있으면 겹쳐 띄우지 않는다.
# joint_state_publisher는 부모(ros2 launch)를 죽여도 안 따라 죽는 경우가 많아 놓치기 쉽다 —
# 이게 남아있으면 CPU를 계속 잡아먹어서 새로 띄운 로봇의 nav2 스택이 못 뜬다.
STALE_PATTERN="gz sim|ros2 launch pinky_gz_sim|ros2 launch pinky_navigation|nav2_web_server|component_container_isolated|joint_state_publisher|robot_state_publisher|parameter_bridge"
if pgrep -f "$STALE_PATTERN" > /dev/null; then
    echo "이미 떠 있는(또는 고아가 된) 프로세스가 있습니다. 먼저 정리하세요:"
    echo "  ps aux | grep -E '$STALE_PATTERN' | grep -v grep"
    echo "  pkill -9 -f '$STALE_PATTERN'"
    echo "정리 후 다시 실행해주세요."
    exit 1
fi

PIDS=()

cleanup() {
    echo
    echo "종료 중... (${PIDS[*]})"
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null
}
trap cleanup EXIT INT TERM

start_robot() {
    local domain="$1" partition="$2" port="$3" gui="$4" name="$5"

    (
        export ROS_DOMAIN_ID="$domain"
        export GZ_PARTITION="$partition"
        exec ros2 launch pinky_gz_sim launch_sim.launch.xml gui:="$gui" world_name:="$WORLD_NAME"
    ) > "$LOG_DIR/${name}_gz.log" 2>&1 &
    PIDS+=("$!")
    echo "  [$name] gazebo 시작 (PID $!, log: $LOG_DIR/${name}_gz.log)"

    sleep 6  # gazebo + robot spawn 시간 확보

    (
        export ROS_DOMAIN_ID="$domain"
        export GZ_PARTITION="$partition"
        exec ros2 launch pinky_navigation gz_web_nav2.launch.xml map:="$MAP" port:="$port"
    ) > "$LOG_DIR/${name}_nav2.log" 2>&1 &
    PIDS+=("$!")
    echo "  [$name] nav2+web 시작 (PID $!, log: $LOG_DIR/${name}_nav2.log, port $port)"
}

echo "== robot1 (domain 15, port 8080) =="
start_robot 15 robot1 8080 true robot1

echo "== robot2 (domain 17, port 8081) =="
start_robot 17 robot2 8081 false robot2

echo
echo "모두 백그라운드로 떴습니다. 로그 확인: tail -f $LOG_DIR/*.log"
echo "상태 확인: curl http://localhost:8080/api/state / curl http://localhost:8081/api/state"
echo "종료하려면 이 터미널에서 Ctrl+C"
wait
