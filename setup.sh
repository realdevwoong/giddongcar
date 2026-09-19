#!/usr/bin/env bash
# 팀원 워크스페이스 최초 설정 스크립트.
# 사용법: ~/giddongcar 에 클론한 뒤, ROS2 배포판을 source하고 이 스크립트를 실행.
#   cd ~/giddongcar && source /opt/ros/jazzy/setup.bash && ./setup.sh
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$REPO_ROOT/pinky_pro"

if [ -z "$ROS_DISTRO" ]; then
    echo "ROS_DISTRO가 설정되어 있지 않습니다. 먼저 'source /opt/ros/<distro>/setup.bash'를 실행하세요."
    exit 1
fi

if [ ! -d "$WS_DIR/src" ]; then
    echo "$WS_DIR/src 를 찾을 수 없습니다. 이 스크립트는 저장소 루트(~/giddongcar)에서 실행해야 합니다."
    exit 1
fi

echo "[1/3] rosdep 의존성 설치..."
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths "$WS_DIR/src" --ignore-src -r -y

echo "[2/3] colcon build..."
cd "$WS_DIR"
colcon build --symlink-install

echo "[3/3] 완료."
echo "새 터미널을 열 때마다 아래 한 줄을 실행하세요:"
echo "  source $WS_DIR/install/setup.bash"
