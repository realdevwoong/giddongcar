#!/usr/bin/env bash
# 실물 관제를 켠다: 점검 → multi_robot.launch.py → 끄면 0 속도.
#
# 사용: start_fleet.sh [launch 인자...]     예: start_fleet.sh auto_spin:=false
# 설정: ~/.config/pinky_fleet.env (git에 안 올린다. IP를 코드에 박지 않는다)
#   ROBOT1_IP=192.168.0.6
#   ROBOT2_IP=192.168.0.8
#   ROBOT1_DOMAIN=15        # 생략하면 15
#   ROBOT2_DOMAIN=17        # 생략하면 17
#   FLEET_PORT=8081         # 생략하면 8080
#   카메라 주소는 각 ROBOT*_IP, YOLO 모델은 pinky_fleet/models/yolo11n.pt를 쓴다.
#   로봇 BLE 서비스가 set_camera를 지원해야 한다(PC Bluetooth 필요).
#   unknown cmd: set_camera면 로봇의 /opt/pinky-ble/ble_server.py를 갱신하고 서비스를 재시작한다.
#   스트림 기본 포트는 5000.
#   카메라 포트만 다르면 launch 인자로 바꾼다: start_fleet.sh camera_port:=5001
#
# 하는 일
#   1. Ctrl+Z로 멈춰 둔 예전 ROS 프로세스를 없앤다. 멈춘 프로세스도 DDS에 남아 로봇 데이터를 막는다.
#      돌고 있는 관제가 있으면 건드리지 않고 멈춘다(강제로 끄면 로봇이 마지막 속도로 달릴 수 있다).
#   2. PC가 로봇 공유기에 붙어 있고 로봇 2대가 ping 되는지
#   3. ROS_STATIC_PEERS를 로봇 IP로 채운다
#   4. 두 도메인에 다른 PC의 AMCL이 이미 떠 있지 않은지
#   5. 관제 실행. Ctrl+Z는 막는다(멈춘 관제가 쌓이지 않게). 끌 때는 Ctrl+C
#   6. 끝나면 두 도메인에 0 속도
set -u

CONFIG=${PINKY_FLEET_ENV:-$HOME/.config/pinky_fleet.env}
WS=$(cd "$(dirname "$(readlink -f "$0")")/../../.." && pwd)   # pinky_pro

fail() { echo "✕ $*"; exit 1; }
ok() { echo "✓ $*"; }

[ -f "$CONFIG" ] || fail "설정 파일이 없어요: $CONFIG — 이 파일 맨 위 주석대로 만드세요"
# shellcheck disable=SC1090
source "$CONFIG"
: "${ROBOT1_IP:?$CONFIG에 ROBOT1_IP가 없어요}" "${ROBOT2_IP:?$CONFIG에 ROBOT2_IP가 없어요}"
ROBOT1_DOMAIN=${ROBOT1_DOMAIN:-15}
ROBOT2_DOMAIN=${ROBOT2_DOMAIN:-17}
FLEET_PORT=${FLEET_PORT:-8080}

# ROS setup hooks read optional variables that may be unset. Source them without
# nounset, then restore strict variable checking for the rest of this script.
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

# ROS entry points use /usr/bin/python3 even when a venv is active. Add the
# project YOLO venv packages to that interpreter's import path when present.
REPO_ROOT=$(dirname "$WS")
YOLO_SITE=$(find "$REPO_ROOT/.venv/lib" -mindepth 2 -maxdepth 2 -type d \
    -path '*/site-packages' -print -quit 2>/dev/null || true)
if [ -n "$YOLO_SITE" ]; then
    export PYTHONPATH="$YOLO_SITE${PYTHONPATH:+:$PYTHONPATH}"
fi

# ── 1. 남은 관제 정리
running=$(ps -eo pid=,stat=,args= | awk '$2 !~ /^T/ && /fleet_dashboard|multi_robot\.launch\.py/ && !/awk/ {print $1}')
[ -z "$running" ] || fail "이미 돌고 있는 관제가 있어요(pid $(echo $running)). 그 터미널에서 Ctrl+C로 끄고 다시 실행하세요"
# ROS 실행 파일만 고른다(/opt/ros, 이 워크스페이스 install). 멈춰 둔 편집기 등은 건드리지 않는다
stopped=$(ps -eo pid=,stat=,args= | awk -v ws="$WS/install/" '$2 ~ /^T/ && (index($0, "/opt/ros/") || index($0, ws)) {print $1}')
if [ -n "$stopped" ]; then
    # 멈춘(T) 프로세스는 cmd_vel을 보낼 수 없어 바로 없애도 안전하다
    kill -9 $stopped 2>/dev/null
    ok "Ctrl+Z로 멈춰 있던 ROS 프로세스 $(echo $stopped | wc -w)개 정리"
fi
ros2 daemon stop >/dev/null 2>&1   # 죽은 노드를 기억하고 있는 데몬을 비운다

# ── 2. 네트워크
subnet=${ROBOT1_IP%.*}.
pc_ip=$(ip -4 -o addr | awk -v s="$subnet" '{split($4, a, "/"); if (index(a[1], s) == 1) print a[1]}' | head -1)
[ -n "$pc_ip" ] || fail "PC에 ${subnet}x 주소가 없어요 — 로봇 공유기 WiFi에 연결하세요 (지금: $(nmcli -t -f ACTIVE,SSID dev wifi 2>/dev/null | awk -F: '$1=="yes" {print $2}'))"
ok "PC $pc_ip"
for ip in "$ROBOT1_IP" "$ROBOT2_IP"; do
    ping -c1 -W1 "$ip" >/dev/null 2>&1 || fail "로봇 $ip 이 ping 안 돼요 — 로봇 전원·WiFi, $CONFIG의 IP를 확인하세요"
done
ok "로봇 $ROBOT1_IP · $ROBOT2_IP 응답"

# ── 3. DDS
export ROS_STATIC_PEERS="$ROBOT1_IP;$ROBOT2_IP"
ok "ROS_STATIC_PEERS=$ROS_STATIC_PEERS · RMW=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp(기본)} — 로봇 bringup도 같은 RMW여야 해요"

# ── 4. 다른 관제(AMCL)가 이미 있는지
for domain in "$ROBOT1_DOMAIN" "$ROBOT2_DOMAIN"; do
    if ROS_DOMAIN_ID=$domain timeout 10 ros2 node list --no-daemon --spin-time 3 2>/dev/null | grep -qx '/amcl'; then
        fail "도메인 $domain 에 AMCL이 이미 있어요 — 다른 PC의 관제를 먼저 끄세요"
    fi
done
ok "도메인 $ROBOT1_DOMAIN/$ROBOT2_DOMAIN 에 다른 관제 없음"

# ── 5. 실행
stop_robots() {
    echo "== 0 속도 보내는 중 (도메인 $ROBOT1_DOMAIN/$ROBOT2_DOMAIN)"
    for domain in "$ROBOT1_DOMAIN" "$ROBOT2_DOMAIN"; do
        ROS_DOMAIN_ID=$domain timeout 5 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 &
    done
    wait
    echo "✓ 끝. 로봇 bringup은 로봇 창에서 Ctrl+C"
}
trap '' TSTP        # Ctrl+Z 무시. 무시 설정은 launch와 Nav2에도 그대로 이어진다
trap ':' INT        # Ctrl+C는 launch가 받아 정리한다. 이 셸은 죽지 않고 남아 0 속도를 보낸다
echo "== 관제 실행: http://localhost:$FLEET_PORT  (끌 때 Ctrl+C · Ctrl+Z는 막아 둠)"
ros2 launch pinky_fleet multi_robot.launch.py port:="$FLEET_PORT" \
    robot1_domain:="$ROBOT1_DOMAIN" robot2_domain:="$ROBOT2_DOMAIN" \
    robot1_camera_host:="$ROBOT1_IP" robot2_camera_host:="$ROBOT2_IP" "$@"
trap '' INT         # 0 속도를 보내는 동안에는 Ctrl+C로 끊기지 않게
stop_robots
