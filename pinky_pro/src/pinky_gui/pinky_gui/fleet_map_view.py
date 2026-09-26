#!/usr/bin/env python3
"""Pinky Pro 함대 관제 GUI - 맵 뷰 + 목표 전송.

로봇 각각이 자체 도메인 안에서 띄우는 nav2_web_server.py(Flask, /api/state, /api/goal,
/api/nav/stop)를 HTTP로 폴링/호출해서 지도 위에 로봇 2대의 실시간 위치를 겹쳐 그리고,
"조작 대상" 로봇을 골라 지도 클릭으로 Nav2 목표를 보내거나 정지시킬 수 있다.
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


class HttpPostWorker(QtCore.QThread):
    """목표 전송/정지 등 1회성 POST를 백그라운드에서 실행 (GUI 멈춤 방지)."""
    resultReady = QtCore.pyqtSignal(str, str, bool, str)  # robot_ns, action_label, success, message

    def __init__(self, robot_ns, action_label, url, payload, parent=None):
        super().__init__(parent)
        self.robot_ns = robot_ns
        self.action_label = action_label
        self.url = url
        self.payload = payload

    def run(self):
        try:
            resp = requests.post(self.url, json=self.payload, timeout=REQUEST_TIMEOUT_SEC)
            resp.raise_for_status()
            data = resp.json()
            ok = bool(data.get("success", True))
            self.resultReady.emit(self.robot_ns, self.action_label, ok, "" if ok else str(data))
        except Exception as e:
            self.resultReady.emit(self.robot_ns, self.action_label, False, str(e))


class FleetMapCanvas(QtWidgets.QWidget):
    goalRequested = QtCore.pyqtSignal(str, float, float)  # robot_ns, world_x, world_y

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(480, 480)
        self.setMouseTracking(True)

        self.map_image = None
        self.map_info = None  # (origin_x, origin_y, resolution, width, height)
        self.robot_poses = {}  # robot_ns -> (x, y, yaw) or None
        self.robot_paths = {}  # robot_ns -> [(x, y), ...] (Nav2 전역 경로)

        self.scale = 50.0  # pixel / meter
        self.center = QtCore.QPointF(0.0, 0.0)

        self._dragging = False
        self._drag_last = None

        self.goal_mode = False
        self.active_robot = "robot1"

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

    def set_robot_path(self, robot_ns, points):
        self.robot_paths[robot_ns] = points
        self.update()

    def world_to_screen(self, x, y):
        sx = self.width() / 2.0 + (x - self.center.x()) * self.scale
        sy = self.height() / 2.0 - (y - self.center.y()) * self.scale
        return sx, sy

    def screen_to_world(self, sx, sy):
        x = (sx - self.width() / 2.0) / self.scale + self.center.x()
        y = self.center.y() - (sy - self.height() / 2.0) / self.scale
        return x, y

    def set_goal_mode(self, enabled):
        self.goal_mode = enabled
        self.setCursor(QtCore.Qt.CrossCursor if enabled else QtCore.Qt.ArrowCursor)

    def set_active_robot(self, robot_ns):
        self.active_robot = robot_ns

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

        for robot_ns, points in self.robot_paths.items():
            if not points or len(points) < 2:
                continue
            color = ROBOT_COLORS.get(robot_ns, QtGui.QColor(200, 200, 200))
            pen = QtGui.QPen(color, 2.5)
            pen.setStyle(QtCore.Qt.DashLine)
            painter.setPen(pen)
            polyline = QtGui.QPolygonF([QtCore.QPointF(*self.world_to_screen(x, y)) for x, y in points])
            painter.drawPolyline(polyline)

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
            if self.goal_mode:
                wx, wy = self.screen_to_world(event.pos().x(), event.pos().y())
                self.goalRequested.emit(self.active_robot, wx, wy)
                return
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
    def __init__(self, robot1_ip, robot2_ip, map_yaml_path, robot1_port=ROBOT_PORT, robot2_port=ROBOT_PORT):
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

        form.addWidget(QtWidgets.QLabel("── 조작 ──"))

        self.robot_urls = {
            "robot1": f"http://{robot1_ip}:{robot1_port}",
            "robot2": f"http://{robot2_ip}:{robot2_port}",
        }
        self._pending_posts = []

        robot_select_row = QtWidgets.QHBoxLayout()
        self.robot_select_group = QtWidgets.QButtonGroup(self)
        self.robot_select_group.setExclusive(True)
        self.robot_select_buttons = {}
        for robot_ns in ("robot1", "robot2"):
            btn = QtWidgets.QPushButton(f"{ROBOT_LABELS[robot_ns]} 조작")
            btn.setCheckable(True)
            if robot_ns == "robot1":
                btn.setChecked(True)
            self.robot_select_group.addButton(btn)
            self.robot_select_buttons[robot_ns] = btn
            robot_select_row.addWidget(btn)
        self.robot_select_group.buttonClicked.connect(self._on_robot_selected)
        form.addLayout(robot_select_row)

        self.goal_mode_btn = QtWidgets.QPushButton("지도 클릭으로 목표 전송")
        self.goal_mode_btn.setCheckable(True)
        self.goal_mode_btn.toggled.connect(self.canvas.set_goal_mode)
        form.addWidget(self.goal_mode_btn)

        stop_btn = QtWidgets.QPushButton("정지 (조작 대상)")
        stop_btn.clicked.connect(self._on_stop_clicked)
        form.addWidget(stop_btn)

        self.action_log = QtWidgets.QLabel("")
        self.action_log.setWordWrap(True)
        self.action_log.setStyleSheet("color: #aaa; font-size: 11px;")
        form.addWidget(self.action_log)

        form.addStretch(1)

        central = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(central)
        layout.addWidget(self.canvas, 1)
        layout.addWidget(side)
        self.setCentralWidget(central)

        self.canvas.goalRequested.connect(self._on_goal_requested)

        self.workers = {
            "robot1": RobotPollWorker("robot1", robot1_ip, robot1_port),
            "robot2": RobotPollWorker("robot2", robot2_ip, robot2_port),
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
        path = state.get("path") or []
        self.canvas.set_robot_path(robot_ns, [(p["x"], p["y"]) for p in path])

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
        self.canvas.set_robot_path(robot_ns, [])
        self.status_labels[robot_ns].setText(f"{ROBOT_LABELS[robot_ns]}: 연결 끊김")
        self.status_labels[robot_ns].setStyleSheet(
            "font-weight: bold; padding: 6px; border-radius: 4px; background-color: #c62828; color: white;"
        )

    def _on_robot_selected(self, btn):
        for robot_ns, b in self.robot_select_buttons.items():
            if b is btn:
                self.canvas.set_active_robot(robot_ns)

    def _on_goal_requested(self, robot_ns, wx, wy):
        url = self.robot_urls[robot_ns] + "/api/goal"
        self._fire_post(robot_ns, "목표 전송", url, {"x": wx, "y": wy})

    def _on_stop_clicked(self):
        robot_ns = self.canvas.active_robot
        url = self.robot_urls[robot_ns] + "/api/nav/stop"
        self._fire_post(robot_ns, "정지", url, {})

    def _fire_post(self, robot_ns, action_label, url, payload):
        worker = HttpPostWorker(robot_ns, action_label, url, payload)
        worker.resultReady.connect(self._on_post_result)
        worker.finished.connect(lambda w=worker: self._cleanup_worker(w))
        self._pending_posts.append(worker)
        worker.start()

    def _cleanup_worker(self, w):
        if w in self._pending_posts:
            self._pending_posts.remove(w)
        w.deleteLater()

    def _on_post_result(self, robot_ns, action_label, success, message):
        label = ROBOT_LABELS.get(robot_ns, robot_ns)
        if success:
            self.action_log.setText(f"{label}: {action_label} 완료")
        else:
            self.action_log.setText(f"{label}: {action_label} 실패 ({message})")


def main():
    parser = argparse.ArgumentParser(description="Pinky Pro fleet map view")
    parser.add_argument("--robot1-ip", default=ROBOT1_IP)
    parser.add_argument("--robot2-ip", default=ROBOT2_IP)
    parser.add_argument("--robot1-port", type=int, default=ROBOT_PORT, help="시뮬레이션에서는 localhost 8080 등으로 구분")
    parser.add_argument("--robot2-port", type=int, default=ROBOT_PORT, help="시뮬레이션에서는 localhost 8081 등으로 구분")
    parser.add_argument("--map", default=DEFAULT_MAP_YAML, help="관제 PC에 있는 map_server용 yaml 경로")
    args, _ = parser.parse_known_args()

    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow(args.robot1_ip, args.robot2_ip, args.map, args.robot1_port, args.robot2_port)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
