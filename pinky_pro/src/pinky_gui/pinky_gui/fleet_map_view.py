#!/usr/bin/env python3
"""Pinky Pro 함대 관제 GUI - 맵 뷰 (1단계).

로봇 각각이 자체 도메인 안에서 띄우는 nav2_web_server.py(Flask, /api/state)를
HTTP로 폴링해서 지도 위에 로봇 2대의 실시간 위치를 겹쳐 그린다.
ROS2 도메인을 넘지 않으므로 이 GUI는 rclpy를 쓰지 않는다.
"""
import argparse
import math
import sys
from pathlib import Path

import requests
import yaml

from PyQt5 import QtCore, QtGui, QtWidgets

# 필요하면 여기 IP만 바꿔서 바로 실행 가능 (또는 --robot1-ip/--robot2-ip로 override)
ROBOT1_IP = "192.168.0.15"
ROBOT2_IP = "192.168.0.17"
ROBOT_PORT = 8080
POLL_INTERVAL_SEC = 0.4
REQUEST_TIMEOUT_SEC = 1.0

# 맵은 로봇에서 받아오지 않고, 관제 PC에 있는 map_server용 yaml/pgm을 직접 읽어서 그린다.
DEFAULT_MAP_YAML = "/home/devwoong/good.yaml"

ROBOT_COLORS = {
    "robot1": QtGui.QColor(0, 188, 212),   # cyan
    "robot2": QtGui.QColor(216, 27, 96),   # magenta
}
ROBOT_LABELS = {
    "robot1": "R1",
    "robot2": "R2",
}


def load_static_map(yaml_path):
    """map_server용 yaml+pgm을 관제 PC 로컬에서 직접 읽어 렌더링용 정보로 변환.

    ROS 맵 이미지는 위쪽 행이 그대로 화면 위쪽(+y)에 대응하므로,
    OccupancyGrid 토픽과 달리 별도로 뒤집을 필요가 없다.
    """
    yaml_path = Path(yaml_path)
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)

    image_path = yaml_path.parent / meta["image"]
    qimg = QtGui.QImage(str(image_path))
    if qimg.isNull():
        raise RuntimeError(f"맵 이미지를 읽을 수 없습니다: {image_path}")
    if meta.get("negate", 0):
        qimg.invertPixels()

    origin = meta["origin"]
    return {
        "image": qimg,
        "origin_x": origin[0],
        "origin_y": origin[1],
        "resolution": meta["resolution"],
        "width": qimg.width(),
        "height": qimg.height(),
    }


class RobotPollWorker(QtCore.QThread):
    dataReceived = QtCore.pyqtSignal(str, dict)
    connectionLost = QtCore.pyqtSignal(str)

    def __init__(self, robot_ns, ip, port=ROBOT_PORT, parent=None):
        super().__init__(parent)
        self.robot_ns = robot_ns
        self.url = f"http://{ip}:{port}/api/state"
        self._running = True

    def stop(self):
        self._running = False
        self.wait(2000)

    def run(self):
        while self._running:
            try:
                resp = requests.get(self.url, timeout=REQUEST_TIMEOUT_SEC)
                resp.raise_for_status()
                self.dataReceived.emit(self.robot_ns, resp.json())
            except Exception:
                self.connectionLost.emit(self.robot_ns)
            self.msleep(int(POLL_INTERVAL_SEC * 1000))


class FleetMapCanvas(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(480, 480)
        self.setMouseTracking(True)

        self.map_image = None
        self.map_info = None  # (origin_x, origin_y, resolution, width, height)
        self.robot_poses = {}  # robot_ns -> (x, y, yaw) or None

        self.scale = 50.0  # pixel / meter
        self.center = QtCore.QPointF(0.0, 0.0)

        self._dragging = False
        self._drag_last = None

    def set_map(self, map_data):
        self.map_image = map_data["image"]
        self.map_info = (
            map_data["origin_x"], map_data["origin_y"],
            map_data["resolution"], map_data["width"], map_data["height"],
        )
        self.update()

    def set_robot_pose(self, robot_ns, pose):
        self.robot_poses[robot_ns] = pose
        self.update()

    def world_to_screen(self, x, y):
        sx = self.width() / 2.0 + (x - self.center.x()) * self.scale
        sy = self.height() / 2.0 - (y - self.center.y()) * self.scale
        return sx, sy

    def fit_to_map(self):
        if self.map_info is None:
            return
        ox, oy, res, w, h = self.map_info
        if w == 0 or h == 0 or res <= 0:
            return
        self.center = QtCore.QPointF(ox + w * res / 2.0, oy + h * res / 2.0)
        margin = 0.9
        self.scale = margin * min(self.width() / (w * res), self.height() / (h * res))
        self.update()

    def paintEvent(self, _event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor(30, 30, 30))

        if self.map_image is not None and self.map_info is not None:
            ox, oy, res, w, h = self.map_info
            top_left = self.world_to_screen(ox, oy + h * res)
            bottom_right = self.world_to_screen(ox + w * res, oy)
            rect = QtCore.QRectF(QtCore.QPointF(*top_left), QtCore.QPointF(*bottom_right))
            painter.drawImage(rect, self.map_image)

        for robot_ns, pose in self.robot_poses.items():
            if pose is None:
                continue
            x, y, yaw = pose
            sx, sy = self.world_to_screen(x, y)
            color = ROBOT_COLORS.get(robot_ns, QtGui.QColor(200, 200, 200))
            label = ROBOT_LABELS.get(robot_ns, robot_ns)

            painter.save()
            painter.translate(sx, sy)
            painter.rotate(-math.degrees(yaw))
            body_len, body_w = 18, 12
            triangle = QtGui.QPolygonF([
                QtCore.QPointF(body_len * 0.6, 0),
                QtCore.QPointF(-body_len * 0.4, body_w * 0.5),
                QtCore.QPointF(-body_len * 0.4, -body_w * 0.5),
            ])
            painter.setPen(QtGui.QPen(QtCore.Qt.black, 1))
            painter.setBrush(color)
            painter.drawPolygon(triangle)
            painter.restore()

            painter.setPen(QtGui.QColor(255, 255, 255))
            painter.drawText(QtCore.QPointF(sx + 10, sy - 10), label)

        painter.end()

    def wheelEvent(self, event):
        factor = 1.0015 ** event.angleDelta().y()
        self.scale = max(2.0, min(2000.0, self.scale * factor))
        self.update()

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self._dragging = True
            self._drag_last = event.pos()

    def mouseMoveEvent(self, event):
        if self._dragging and self._drag_last is not None:
            delta = event.pos() - self._drag_last
            self._drag_last = event.pos()
            self.center = QtCore.QPointF(
                self.center.x() - delta.x() / self.scale,
                self.center.y() + delta.y() / self.scale,
            )
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self._dragging = False


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, robot1_ip, robot2_ip, map_yaml_path):
        super().__init__()
        self.setWindowTitle("Pinky Pro - 함대 지도 뷰")
        self.resize(1100, 750)

        self.canvas = FleetMapCanvas()
        map_data = load_static_map(map_yaml_path)
        self.canvas.set_map(map_data)
        self.canvas.fit_to_map()

        side = QtWidgets.QWidget()
        side.setFixedWidth(240)
        form = QtWidgets.QVBoxLayout(side)

        self.status_labels = {}
        for robot_ns in ("robot1", "robot2"):
            lbl = QtWidgets.QLabel(f"{ROBOT_LABELS[robot_ns]}: 연결 대기중")
            lbl.setWordWrap(True)
            lbl.setStyleSheet("font-weight: bold; padding: 6px; border-radius: 4px;")
            self.status_labels[robot_ns] = lbl
            form.addWidget(lbl)

        fit_btn = QtWidgets.QPushButton("지도 전체 보기")
        fit_btn.clicked.connect(self.canvas.fit_to_map)
        form.addWidget(fit_btn)
        form.addStretch(1)

        central = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(central)
        layout.addWidget(self.canvas, 1)
        layout.addWidget(side)
        self.setCentralWidget(central)

        self.workers = {
            "robot1": RobotPollWorker("robot1", robot1_ip),
            "robot2": RobotPollWorker("robot2", robot2_ip),
        }
        for w in self.workers.values():
            w.dataReceived.connect(self._on_data)
            w.connectionLost.connect(self._on_connection_lost)
            w.start()

    def closeEvent(self, event):
        for w in self.workers.values():
            w.stop()
        super().closeEvent(event)

    def _on_data(self, robot_ns, state):
        pose = state.get("pose")
        if pose is not None:
            self.canvas.set_robot_pose(robot_ns, (pose["x"], pose["y"], pose["yaw"]))
            self.status_labels[robot_ns].setText(
                f"{ROBOT_LABELS[robot_ns]}: 정상\nX: {pose['x']:+.2f}  Y: {pose['y']:+.2f}"
            )
            self.status_labels[robot_ns].setStyleSheet(
                "font-weight: bold; padding: 6px; border-radius: 4px; background-color: #2e7d32; color: white;"
            )
        else:
            self.canvas.set_robot_pose(robot_ns, None)
            self.status_labels[robot_ns].setText(f"{ROBOT_LABELS[robot_ns]}: TF 없음 (초기위치 필요)")
            self.status_labels[robot_ns].setStyleSheet(
                "font-weight: bold; padding: 6px; border-radius: 4px; background-color: #ef6c00; color: white;"
            )

    def _on_connection_lost(self, robot_ns):
        self.canvas.set_robot_pose(robot_ns, None)
        self.status_labels[robot_ns].setText(f"{ROBOT_LABELS[robot_ns]}: 연결 끊김")
        self.status_labels[robot_ns].setStyleSheet(
            "font-weight: bold; padding: 6px; border-radius: 4px; background-color: #c62828; color: white;"
        )


def main():
    parser = argparse.ArgumentParser(description="Pinky Pro fleet map view")
    parser.add_argument("--robot1-ip", default=ROBOT1_IP)
    parser.add_argument("--robot2-ip", default=ROBOT2_IP)
    parser.add_argument("--map", default=DEFAULT_MAP_YAML, help="관제 PC에 있는 map_server용 yaml 경로")
    args, _ = parser.parse_known_args()

    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow(args.robot1_ip, args.robot2_ip, args.map)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
