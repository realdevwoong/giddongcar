#!/usr/bin/env bash
# 지금 이 셸의 ROS_DOMAIN_ID(로봇 1대분)에서 관제 시스템이 필요로 하는 토픽/액션이
# 실제로 떠 있는지 확인한다. 로봇/시뮬레이션 쪽 launch를 먼저 띄워둔 상태에서 실행할 것.
#
# 사용법:
#   ROS_DOMAIN_ID=15 ./check_fleet_topics.sh          # 토픽만 확인
#   ROS_DOMAIN_ID=15 ./check_fleet_topics.sh 8080     # 토픽 + Flask /api/state 까지 확인

set -u

PORT="${1:-}"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "== ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} 기준 토픽 목록 =="
TOPIC_LIST="$(ros2 topic list -t 2>/dev/null)"
if [ -z "$TOPIC_LIST" ]; then
  echo -e "${RED}ros2 topic list 결과가 비어있음 — ROS2 환경(source install/setup.bash)이 안 되어 있거나 아직 아무 노드도 안 떴을 수 있음${NC}"
  exit 1
fi
echo "$TOPIC_LIST"
echo

check_topic() {
  local topic="$1"
  local label="$2"
  if echo "$TOPIC_LIST" | grep -qF -- "$topic "; then
    echo -e "  ${GREEN}OK${NC}   $topic   ($label)"
  else
    echo -e "  ${RED}MISS${NC} $topic   ($label)"
  fi
}

echo "== 레이어 1: Gazebo <-> ROS2 (하드웨어/센서) =="
check_topic "/scan" "LaserScan"
check_topic "/odom" "Odometry"
check_topic "/tf" "TFMessage"
check_topic "/cmd_vel" "Twist"
check_topic "/joint_states" "JointState"
check_topic "/robot_description" "String"
echo

echo "== 레이어 2: Nav2 스택 (map/plan/costmap/nav action) =="
check_topic "/map" "OccupancyGrid"
check_topic "/plan" "Path"
check_topic "/local_costmap/costmap" "Costmap"
check_topic "/global_costmap/costmap" "Costmap"
check_topic "/tf_static" "TFMessage"
check_topic "/initialpose" "PoseWithCovarianceStamped"
check_topic "/navigate_to_pose/_action/status" "GoalStatusArray"
echo

echo "== 노드 목록 =="
ros2 node list 2>/dev/null
echo

echo "== map -> base_link TF (AMCL이 잡았는지) =="
timeout 3 ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -n 6
echo

if [ -n "$PORT" ]; then
  echo "== Flask /api/state (localhost:${PORT}) =="
  RESP="$(curl -s --max-time 2 "http://localhost:${PORT}/api/state")"
  if [ -z "$RESP" ]; then
    echo -e "${RED}응답 없음 — nav2_web_server.py가 안 떠있거나 port가 다름${NC}"
  else
    echo "$RESP" | head -c 500
    echo
    if echo "$RESP" | grep -q '"pose"[^n]*null'; then
      echo -e "${YELLOW}pose가 null — RViz/웹 UI에서 initial pose를 아직 안 찍었을 수 있음${NC}"
    fi
  fi
fi
