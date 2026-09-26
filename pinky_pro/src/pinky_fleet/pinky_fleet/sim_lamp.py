#!/usr/bin/env python3
"""시뮬 전용 램프. 실물 pinky_lamp_control과 같은 set_lamp 서비스를 받아 Gazebo 속 램프 색을 바꾼다.

대시보드는 시뮬이든 실물이든 set_lamp만 부른다(cmd_vel이 실물은 모터로, 시뮬은 가제보 바퀴로 가는 것과 같은 구조).
mode 규칙은 실물 main_node.cpp와 같다: 0 끄기, 1 켜기, 2 깜빡임(time ms 켜고 time ms 끄기), 3 숨쉬기(time ms 동안 밝아졌다 어두워짐).
"""
import time

import rclpy
from rclpy.node import Node
from pinky_interfaces.srv import SetLamp
from ros_gz_interfaces.msg import MaterialColor

OFF = (0.05, 0.05, 0.05)  # 꺼진 램프(완전 검정이면 모양이 안 보여서 아주 어둡게)


def lamp_color(mode, color, period_ms, t):
    """켜진 지 t초 뒤 램프 색 (r, g, b)."""
    period = max(period_ms, 10) / 1000
    if mode == 1:
        return color
    if mode == 2:
        return color if int(t / period) % 2 == 0 else OFF
    if mode == 3:
        phase = t % (2 * period) / period  # 0→1 밝아지고 1→2 어두워진다
        level = phase if phase <= 1 else 2 - phase
        return tuple(c * level for c in color)
    return OFF


class SimLamp(Node):
    def __init__(self):
        super().__init__('sim_lamp')
        robot = self.declare_parameter('robot', 'robot1').value
        # 가제보 모델 이름(robot1)::링크::비주얼. 두 로봇의 비주얼 이름이 같아서 모델 이름까지 붙여야 한 대만 칠한다.
        self.visual = f'{robot}::robot_lamp::robot_lamp_visual'
        # 실물 램프가 켜질 때와 같은 기본값: 흰색 숨쉬기 1초
        self.mode, self.color, self.period_ms, self.since = 3, (1.0, 1.0, 1.0), 1000, time.monotonic()
        self.last = None
        self.pub = self.create_publisher(MaterialColor, 'lamp_color', 10)
        self.create_service(SetLamp, 'set_lamp', self.on_set_lamp)
        self.create_timer(0.05, self.tick)

    def on_set_lamp(self, request, response):
        self.mode, self.period_ms = request.mode, request.time
        self.color = (request.color.r, request.color.g, request.color.b)
        self.since, self.last = time.monotonic(), None
        response.result = True
        return response

    def tick(self):
        rgb = tuple(round(c, 2) for c in lamp_color(self.mode, self.color, self.period_ms,
                                                    time.monotonic() - self.since))
        if rgb == self.last:
            return  # 색이 그대로면 보내지 않는다
        self.last = rgb
        msg = MaterialColor()
        msg.entity.name = self.visual
        msg.entity_match = MaterialColor.FIRST
        for part in (msg.ambient, msg.diffuse, msg.emissive):
            part.r, part.g, part.b, part.a = (*rgb, 1.0)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SimLamp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
