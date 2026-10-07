#!/usr/bin/env bash
# 인터넷 없는 공유기에서 로봇 시계를 이 PC 시계에 맞춘다.
# 로봇이 보내는 /scan 시각으로 차이를 재고, 로봇에서 "그 순간 시각 + 차이"로 고친다.
# (PC 시각을 미리 적어 보내면 비밀번호를 치는 몇 초만큼 틀어진다)
#
# 사용: sync_robot_clock.sh <로봇IP> <도메인> [<로봇IP> <도메인> ...]
#   예: sync_robot_clock.sh 192.168.0.6 15 192.168.0.8 17
# 필요: 로봇에서 bringup(라이다)이 돌고 있고, 이 터미널에서 ROS_STATIC_PEERS로 로봇이 보일 것.
set -u

# ROS setup hooks read optional variables that may be unset.
set +u
source /opt/ros/jazzy/setup.bash
set -u

measure() {   # PC 시각 - 로봇이 찍은 시각 [초]. 못 재면 빈 문자열
    ROS_DOMAIN_ID=$1 timeout 20 ros2 topic delay /scan --window 10 2>/dev/null \
        | awk '/average delay/ {print $3; exit}'
}

[ $# -ge 2 ] || { sed -n 2,8p "$0"; exit 1; }
while [ $# -ge 2 ]; do
    ip=$1; domain=$2; shift 2
    echo "== 로봇 $ip (도메인 $domain)"
    d=$(measure "$domain")
    if [ -z "$d" ]; then
        echo "   /scan을 못 받았다. 로봇 bringup, ROS_STATIC_PEERS, 도메인을 확인할 것"
        continue
    fi
    echo "   차이 ${d}초 (PC - 로봇). 로봇 시계를 이만큼 옮긴다. 비밀번호를 물으면 로봇 비밀번호"
    # 시각 계산은 sudo 인증이 끝난 뒤 로봇에서 한다
    ssh -t "pinky@$ip" "sudo python3 -c 'import subprocess, time; subprocess.run([\"date\", \"-s\", \"@%.3f\" % (time.time() + $d)], check=True)'"
    sleep 3
    echo "   맞춘 뒤 차이: $(measure "$domain")초  (0.0x 이면 OK)"
done
