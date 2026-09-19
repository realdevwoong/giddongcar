#!/usr/bin/env python3
"""Pinky Pro 실시간 위치/지도 모니터 (Qt).

SLAM/Nav2 시작 전에 로봇이 지금 어디에 있는지(map 기준으로 로컬라이즈됐는지,
odom만 살아있는지, TF 자체가 없는지)를 한눈에 보여주기 위한 독립 실행형 GUI.
nav2_web_server.py와 동일하게 map->base_link TF, /map, /scan을 사용한다.
"""
import math
import sys
import threading
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)

from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from PyQt5 import QtCore, QtGui, QtWidgets


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def occupancy_grid_to_qimage(msg):
    w, h = msg.info.width, msg.info.height
    grid = np.array(msg.data, dtype=np.int16).reshape((h, w))

    gray = np.full((h, w), 205, dtype=np.uint8)  # -1(unknown) -> 회색
    known = grid >= 0
    gray[known] = np.clip(255.0 - grid[known].astype(np.float32) * 2.55, 0, 255).astype(np.uint8)

    # OccupancyGrid는 row0 = 최소 y. 이미지 위쪽이 +y(북쪽)가 되도록 세로로 뒤집는다.
    gray = np.ascontiguousarray(np.flipud(gray))

    qimg = QtGui.QImage(gray.data, w, h, w, QtGui.QImage.Format_Grayscale8)
    return qimg.copy()  # numpy 버퍼 수명과 분리해 Qt가 데이터를 소유하게 함


class PoseMonitorNode(Node):
    def __init__(self):
        super().__init__("pinky_pose_monitor_qt")

        self.lock = threading.Lock()
        self.map_msg = None
        self.map_image = None
        self.path_points = []
        self.scan_points = []
        self.pose = None        # (x, y, yaw)
        self.pose_frame = None  # "map" | "odom" | None
        self.trail = deque(maxlen=3000)
        self._last_trail_pt = None

        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "map", self._on_map, map_qos)
        self.create_subscription(Path, "plan", self._on_path, 10)
        self.create_subscription(LaserScan, "scan", self._on_scan, qos_profile_sensor_data)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.create_timer(0.05, self._update_pose)

    def _on_map(self, msg):
        image = occupancy_grid_to_qimage(msg)
        with self.lock:
            self.map_msg = msg
            self.map_image = image

    def _on_path(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        with self.lock:
            self.path_points = pts

    def _on_scan(self, msg):
        # scan 프레임 -> map 변환을 직접 조회해서 라이다 마운트 오프셋까지 반영
        try:
            tf = self.tf_buffer.lookup_transform("map", msg.header.frame_id, Time())
        except Exception:
            return

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        tyaw = quat_to_yaw(tf.transform.rotation)
        cos_t, sin_t = math.cos(tyaw), math.sin(tyaw)

        pts = []
        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                lx = r * math.cos(angle)
                ly = r * math.sin(angle)
                pts.append((tx + lx * cos_t - ly * sin_t, ty + lx * sin_t + ly * cos_t))
            angle += msg.angle_increment

        with self.lock:
            self.scan_points = pts

    def _update_pose(self):
        # 1) map->base_link (AMCL/slam_toolbox 로컬라이즈됨) 우선 시도
        # 2) 실패하면 odom->base_footprint (dead-reckoning만 가능한 상태)로 폴백
        for target, base, label in (("map", "base_link", "map"),
                                     ("odom", "base_footprint", "odom")):
            try:
                tf = self.tf_buffer.lookup_transform(target, base, Time())
            except Exception:
                continue

            x = tf.transform.translation.x
            y = tf.transform.translation.y
            yaw = quat_to_yaw(tf.transform.rotation)
            with self.lock:
                self.pose = (x, y, yaw)
                self.pose_frame = label
                if label == "map":
                    last = self._last_trail_pt
                    if last is None or math.hypot(x - last[0], y - last[1]) > 0.02:
                        self.trail.append((x, y))
                        self._last_trail_pt = (x, y)
            return

        with self.lock:
            self.pose = None
            self.pose_frame = None

    def clear_trail(self):
        with self.lock:
            self.trail.clear()
            self._last_trail_pt = None

    def get_snapshot(self):
        with self.lock:
            return {
                "map_image": self.map_image,
                "map_info": self.map_msg.info if self.map_msg is not None else None,
                "pose": self.pose,
                "pose_frame": self.pose_frame,
                "trail": list(self.trail),
                "path": list(self.path_points),
                "scan": list(self.scan_points),
            }


class MapCanvas(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(480, 480)
        self.setMouseTracking(True)

        self.map_image = None
        self.map_info = None  # (origin_x, origin_y, resolution, width, height)
        self.pose = None
        self.pose_frame = None
        self.trail = []
        self.path = []
        self.scan = []

        self.follow = True
        self.scale = 50.0  # pixel / meter
        self.center = QtCore.QPointF(0.0, 0.0)

        self._dragging = False
        self._drag_last = None

    def update_data(self, snap):
        if snap["map_info"] is not None:
            info = snap["map_info"]
            self.map_image = snap["map_image"]
            self.map_info = (info.origin.position.x, info.origin.position.y,
                              info.resolution, info.width, info.height)

        self.pose = snap["pose"]
        self.pose_frame = snap["pose_frame"]
        self.trail = snap["trail"]
        self.path = snap["path"]
        self.scan = snap["scan"]

        if self.follow and self.pose is not None:
            self.center = QtCore.QPointF(self.pose[0], self.pose[1])

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

        if len(self.path) >= 2:
            painter.setPen(QtGui.QPen(QtGui.QColor(80, 160, 255), 2))
            pts = [QtCore.QPointF(*self.world_to_screen(x, y)) for x, y in self.path]
            painter.drawPolyline(QtGui.QPolygonF(pts))

        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(255, 210, 0, 160))
        for x, y in self.trail:
            painter.drawEllipse(QtCore.QPointF(*self.world_to_screen(x, y)), 1.5, 1.5)

        painter.setBrush(QtGui.QColor(255, 60, 60, 180))
        for x, y in self.scan:
            painter.drawEllipse(QtCore.QPointF(*self.world_to_screen(x, y)), 1.5, 1.5)

        if self.pose is not None:
            x, y, yaw = self.pose
            sx, sy = self.world_to_screen(x, y)
            color = (QtGui.QColor(60, 220, 90) if self.pose_frame == "map"
                     else QtGui.QColor(255, 150, 30))
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
        else:
            painter.setPen(QtGui.QColor(255, 90, 90))
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter,
                              "TF 없음 - bringup / robot_state_publisher 연결 확인 필요")

        painter.end()

    def wheelEvent(self, event):
        factor = 1.0015 ** event.angleDelta().y()
        self.scale = max(2.0, min(2000.0, self.scale * factor))
        self.update()

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self._dragging = True
            self._drag_last = event.pos()
            self.follow = False

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
    def __init__(self, node: PoseMonitorNode):
        super().__init__()
        self.node = node
        self.setWindowTitle("Pinky Pro - 실시간 위치 모니터")
        self.resize(1100, 750)

        self.canvas = MapCanvas()

        side = QtWidgets.QWidget()
        side.setFixedWidth(260)
        form = QtWidgets.QVBoxLayout(side)

        self.status_label = QtWidgets.QLabel("상태: -")
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet("font-weight: bold; padding: 8px; border-radius: 4px;")

        self.pos_label = QtWidgets.QLabel("X: -\nY: -\nYaw: -")
        self.pos_label.setStyleSheet("font-family: monospace; font-size: 13px;")

        self.map_label = QtWidgets.QLabel("지도: 수신 대기중 (/map)")
        self.map_label.setWordWrap(True)

        self.follow_box = QtWidgets.QCheckBox("로봇 자동 추적")
        self.follow_box.setChecked(True)
        self.follow_box.stateChanged.connect(self._on_follow_toggled)

        fit_btn = QtWidgets.QPushButton("지도 전체 보기")
        fit_btn.clicked.connect(self.canvas.fit_to_map)

        clear_btn = QtWidgets.QPushButton("이동 궤적 지우기")
        clear_btn.clicked.connect(self._clear_trail)

        legend = QtWidgets.QLabel(
            "● 초록 삼각형: map 기준 로컬라이즈\n"
            "● 주황 삼각형: odom만 사용(초기 위치 필요)\n"
            "● 노란 점: 이동 궤적\n"
            "● 빨간 점: 실시간 라이다 스캔\n"
            "● 파란 선: 현재 계획 경로(/plan)"
        )
        legend.setStyleSheet("color: #aaaaaa; font-size: 11px;")
        legend.setWordWrap(True)

        for w in (self.status_label, self.pos_label, self.map_label,
                  self.follow_box, fit_btn, clear_btn, legend):
            form.addWidget(w)
        form.addStretch(1)

        central = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(central)
        layout.addWidget(self.canvas, 1)
        layout.addWidget(side)
        self.setCentralWidget(central)

        self._fitted_once = False

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(100)

    def _on_follow_toggled(self, state):
        self.canvas.follow = bool(state)

    def _clear_trail(self):
        self.node.clear_trail()

    def _refresh(self):
        snap = self.node.get_snapshot()
        self.canvas.update_data(snap)

        if not self._fitted_once and snap["map_info"] is not None:
            self.canvas.fit_to_map()
            self._fitted_once = True

        frame = snap["pose_frame"]
        if frame == "map":
            self.status_label.setText("MAP 기준 정상 로컬라이즈됨")
            bg = "#2e7d32"
        elif frame == "odom":
            self.status_label.setText("ODOM만 존재 - AMCL 초기 위치(2D Pose Estimate) 설정 필요")
            bg = "#ef6c00"
        else:
            self.status_label.setText("TF 없음 - 로봇 bringup/네트워크 확인 필요")
            bg = "#c62828"
        self.status_label.setStyleSheet(
            f"font-weight: bold; padding: 8px; border-radius: 4px; background-color: {bg}; color: white;"
        )

        pose = snap["pose"]
        if pose is not None:
            x, y, yaw = pose
            self.pos_label.setText(f"X: {x:+.3f} m\nY: {y:+.3f} m\nYaw: {math.degrees(yaw):+.1f} deg")
        else:
            self.pos_label.setText("X: -\nY: -\nYaw: -")

        info = snap["map_info"]
        if info is not None:
            self.map_label.setText(f"지도: {info.width}x{info.height} @ {info.resolution:.3f} m/cell")


def main():
    rclpy.init()
    node = PoseMonitorNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow(node)
    win.show()
    ret = app.exec_()

    rclpy.shutdown()
    sys.exit(ret)


if __name__ == "__main__":
    main()
