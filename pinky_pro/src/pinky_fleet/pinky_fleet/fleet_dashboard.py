#!/usr/bin/env python3
"""One HTTP UI, separate ROS contexts and TF buffers for each robot."""
import argparse
import signal
import threading
from http.server import ThreadingHTTPServer

from pinky_fleet.fleet import Fleet
from pinky_fleet.robot import Robot
from pinky_fleet.traffic import load_zones
from pinky_fleet.fleet_common import *  # noqa: F401,F403 - preserve dashboard imports
from pinky_fleet.dashboard_http import handler_for

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot1-domain', type=int, default=15)
    parser.add_argument('--robot2-domain', type=int, default=17)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--use-sim-time', action='store_true', help='Gazebo: /clock 시간을 쓴다')
    parser.add_argument('--traffic-zones', default='', help='교통 정리 구역 YAML. 비우면 교통 정리를 끈다')
    parser.add_argument('--known-pose', action='append', default=[], choices=('robot1', 'robot2'),
                        help='launch가 초기 위치를 알려 준 로봇. 이 로봇은 전역 위치 찾기를 하지 않는다')
    parser.add_argument('--auto-spin', action='store_true',
                        help='전역 위치 찾기를 시작하자마자 제자리에서 한 바퀴 돈다(실물이 사람 확인 없이 움직인다)')
    args = parser.parse_args()
    zones = load_zones(args.traffic_zones) if args.traffic_zones else None
    # nohup이나 스크립트 백그라운드로 띄우면 SIGINT 무시가 상속돼 Ctrl+C로 안 꺼진다. 항상 KeyboardInterrupt를 받게 한다.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    robots = {}
    server = fleet = None
    try:
        for i, domain in enumerate((args.robot1_domain, args.robot2_domain), 1):
            name = f'robot{i}'
            robots[name] = Robot(name, domain, args.use_sim_time, name in args.known_pose, args.auto_spin)
        fleet = Fleet(robots, zones)
        threading.Thread(target=fleet.run_traffic, daemon=True).start()   # 교통 정리가 꺼져도 다시 보내기는 돈다
        server = ThreadingHTTPServer((args.host, args.port), handler_for(fleet))
        print(f'Fleet dashboard: http://{args.host}:{args.port}'
              + (' · 교통 정리 켬' if fleet.gate else ' · 교통 정리 끔'), flush=True)
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        # Terminal and parent launch can both send SIGINT during shutdown.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if fleet:
            fleet.traffic_stop.set()
        if server:
            server.server_close()
        for robot in robots.values():
            robot.close()


if __name__ == "__main__":
    main()
