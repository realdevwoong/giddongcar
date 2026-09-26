#!/usr/bin/env bash
# Load ROS and the Pinky overlay before resolving launch packages.
set -e
fleet_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
pinky_workspace="${PINKY_WORKSPACE:-$HOME/pinky_pro}"
if [[ ! -f "$pinky_workspace/install/setup.bash" ]]; then
    echo "Pinky 설치 환경을 찾을 수 없습니다: $pinky_workspace/install/setup.bash" >&2
    echo "워크스페이스 경로와 colcon build 완료 여부를 확인하세요." >&2
    exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$pinky_workspace/install/setup.bash"
if ! ros2 pkg prefix pinky_navigation >/dev/null; then
    echo "워크스페이스에 pinky_navigation 패키지가 설치되어 있는지 확인하세요." >&2
    exit 1
fi
exec ros2 launch "$fleet_dir/multi_robot.launch.py" "$@"
