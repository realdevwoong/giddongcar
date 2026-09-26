#!/usr/bin/env python3
"""Pinky Pro 관제 GUI - 독립 실행형 가짜 시뮬레이터 (ROS2/Gazebo 없음).

good.yaml 지도 위에서 로봇 2대를 순수 파이썬으로 흉내낸다. 물리 엔진도, ROS2 노드도,
네트워크 통신도 없다 — 지도 점유 격자에서 A*로 경로를 계산하고, 그 경로를 따라
로봇 아이콘을 일정 속도로 움직이는 것뿐이다. 목적은 "화면에서 클릭으로 조작하고,
검은 벽(장애물)은 피해서 이동하는" 관제 GUI 자체를 Gazebo/Nav2 없이 바로 확인하는 것.

실행: python3 fleet_sim_standalone.py [--map /home/devwoong/good.yaml]
필요한 건 PyQt5, numpy, PyYAML, Pillow뿐 — colcon build도, ros2 launch도 필요 없다.
"""
import argparse
import heapq
import math
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from PyQt5 import QtCore, QtGui, QtWidgets

DEFAULT_MAP_YAML = "/home/devwoong/good.yaml"
ROBOT_SPEED = 0.4  # m/s
INFLATE_RADIUS_M = 0.05  # 로봇 반경만큼 장애물을 부풀림. 지도가 작아서 너무 크면 문이 막힘
TICK_MS = 50

ROBOT_COLORS = {
    "robot1": QtGui.QColor(0, 188, 212),
    "robot2": QtGui.QColor(216, 27, 96),
}
ROBOT_LABELS = {"robot1": "R1", "robot2": "R2"}

APP_QSS = """
QMainWindow, QWidget { background-color: #0f1216; color: #e6e9ef; font-size: 13px; }
#header { background-color: #161b22; border-bottom: 1px solid #262d38; }
#appTitle { font-size: 16px; font-weight: 700; color: #e6e9ef; }
#appSubtitle { font-size: 11px; color: #8b93a1; }
#clockLabel { font-size: 12px; color: #4f9dff; }
#sidePanel { background-color: #161b22; border-left: 1px solid #262d38; }
QGroupBox {
    background-color: #1a2029; border: 1px solid #262d38; border-radius: 10px; margin-top: 10px;
    padding: 10px 8px 8px 8px; font-weight: 600; color: #8b93a1;
}
QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
QPushButton {
    background-color: #232b38; color: #e6e9ef; border: 1px solid #2d3646;
    border-radius: 6px; padding: 7px 10px;
}
QPushButton:hover { border-color: #4f9dff; }
QPushButton:checked { background-color: #4f9dff; color: #06121f; font-weight: 600; }
#dangerBtn { border-color: #7a2b2b; }
#dangerBtn:hover { border-color: #ff6b6b; }
#dispatchBtn { background-color: #3ddc84; color: #06121f; font-weight: 700; border: none; }
#dispatchBtn:hover { background-color: #4fe895; }
#actionLog { color: #8b93a1; font-size: 11px; }
QMenuBar { background-color: #161b22; color: #e6e9ef; }
QMenuBar::item:selected { background-color: #232b38; }
QMenu { background-color: #161b22; color: #e6e9ef; border: 1px solid #262d38; }
QMenu::item:selected { background-color: #232b38; }
QStatusBar { background-color: #161b22; color: #8b93a1; border-top: 1px solid #262d38; }
"""


def make_app_icon():
    pix = QtGui.QPixmap(64, 64)
    pix.fill(QtCore.Qt.transparent)
    painter = QtGui.QPainter(pix)
    painter.setRenderHint(QtGui.QPainter.Antialiasing)
    painter.setBrush(QtGui.QColor(15, 18, 22))
    painter.setPen(QtCore.Qt.NoPen)
    painter.drawRoundedRect(2, 2, 60, 60, 14, 14)
    painter.setBrush(ROBOT_COLORS["robot1"])
    painter.drawEllipse(14, 14, 20, 20)
    painter.setBrush(ROBOT_COLORS["robot2"])
    painter.drawEllipse(30, 30, 20, 20)
    painter.end()
    return QtGui.QIcon(pix)


# ---------------- 지도 로딩 ----------------

def load_map(yaml_path):
    yaml_path = Path(yaml_path)
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    image_path = yaml_path.parent / meta["image"]

    qimg = QtGui.QImage(str(image_path))
    if qimg.isNull():
        raise RuntimeError(f"맵 이미지를 읽을 수 없습니다: {image_path}")
    if meta.get("negate", 0):
        qimg.invertPixels()

    gray = np.array(Image.open(image_path).convert("L"), dtype=np.float64)
    if meta.get("negate", 0):
        prob = gray / 255.0
    else:
        prob = (255.0 - gray) / 255.0
    occupied_thresh = meta.get("occupied_thresh", 0.65)
    free_thresh = meta.get("free_thresh", 0.196)
    occupied = prob > occupied_thresh
    known_free = prob < free_thresh
    unknown = ~occupied & ~known_free  # 아직 안 탐색된 회색 영역 — 경로계획에서 못 지나가게 막는다

    resolution = meta["resolution"]
    origin_x, origin_y = meta["origin"][0], meta["origin"][1]
    height, width = occupied.shape

    # 벽(occupied)만 로봇 반경만큼 부풀린다 — unknown까지 부풀리면 이 작은 지도에서는
    # 문이 막혀 반대쪽 방이 도달 불가능해질 수 있다.
    inflate_cells = max(1, int(round(INFLATE_RADIUS_M / resolution)))
    inflated_occupied = occupied.copy()
    ys, xs = np.where(occupied)
    for y, x in zip(ys, xs):
        r0, r1 = max(0, y - inflate_cells), min(height, y + inflate_cells + 1)
        c0, c1 = max(0, x - inflate_cells), min(width, x + inflate_cells + 1)
        inflated_occupied[r0:r1, c0:c1] = True

    blocked = inflated_occupied | unknown

    return {
        "image": qimg,
        "occupied": blocked,
        "resolution": resolution,
        "origin_x": origin_x,
        "origin_y": origin_y,
        "width": width,
        "height": height,
    }


def world_to_grid(map_info, x, y):
    res = map_info["resolution"]
    col = int((x - map_info["origin_x"]) / res)
    row = int(map_info["height"] - (y - map_info["origin_y"]) / res)
    return row, col


def grid_to_world(map_info, row, col):
    res = map_info["resolution"]
    x = map_info["origin_x"] + (col + 0.5) * res
    y = map_info["origin_y"] + (map_info["height"] - row - 0.5) * res
    return x, y


def in_bounds(map_info, row, col):
    return 0 <= row < map_info["height"] and 0 <= col < map_info["width"]


def nearest_free_cell(map_info, row, col):
    if in_bounds(map_info, row, col) and not map_info["occupied"][row, col]:
        return row, col
    occ = map_info["occupied"]
    h, w = occ.shape
    for radius in range(1, max(h, w)):
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                r, c = row + dr, col + dc
                if in_bounds(map_info, r, c) and not occ[r, c]:
                    return r, c
    return None


def reachable_cells(map_info, start):
    """start에서 4방향으로 실제로 걸어갈 수 있는 칸 집합 (고립된 자유공간 배제용)."""
    occ = map_info["occupied"]
    h, w = occ.shape
    visited = np.zeros_like(occ, dtype=bool)
    if not in_bounds(map_info, *start) or occ[start]:
        return visited
    from collections import deque
    q = deque([start])
    visited[start] = True
    while q:
        r, c = q.popleft()
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < h and 0 <= nc < w and not occ[nr, nc] and not visited[nr, nc]:
                visited[nr, nc] = True
                q.append((nr, nc))
    return visited


# ---------------- A* 경로 계획 ----------------

def astar(occupied, start, goal):
    if start == goal:
        return [start]
    h, w = occupied.shape
    neighbors = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                 (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
                 (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]

    def heuristic(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    open_heap = [(heuristic(start, goal), 0.0, start)]
    came_from = {}
    g_score = {start: 0.0}
    visited = set()

    while open_heap:
        _, g, current = heapq.heappop(open_heap)
        if current in visited:
            continue
        visited.add(current)
        if current == goal:
            path = [current]
            while path[-1] in came_from:
                path.append(came_from[path[-1]])
            path.reverse()
            return path

        for dr, dc, cost in neighbors:
            nr, nc = current[0] + dr, current[1] + dc
            if not (0 <= nr < h and 0 <= nc < w) or occupied[nr, nc]:
                continue
            ng = g + cost
            if (nr, nc) not in g_score or ng < g_score[(nr, nc)]:
                g_score[(nr, nc)] = ng
                came_from[(nr, nc)] = current
                heapq.heappush(open_heap, (ng + heuristic((nr, nc), goal), ng, (nr, nc)))

    return None


# ---------------- 캔버스 ----------------

class FleetMapCanvas(QtWidgets.QWidget):
    targetPinned = QtCore.pyqtSignal(str, float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(480, 480)
        self.map_image = None
        self.map_info = None
        self.robot_poses = {}
        self.robot_paths = {}
        self.pending_targets = {}  # robot_ns -> (x, y) 아직 "전송" 안 누른 대기중 목표
        self.scale = 50.0
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

    def set_pending_target(self, robot_ns, point):
        self.pending_targets[robot_ns] = point
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
        painter.fillRect(self.rect(), QtGui.QColor(15, 18, 22))

        self._draw_grid(painter)

        if self.map_image is not None and self.map_info is not None:
            ox, oy, res, w, h = self.map_info
            top_left = self.world_to_screen(ox, oy + h * res)
            bottom_right = self.world_to_screen(ox + w * res, oy)
            rect = QtCore.QRectF(QtCore.QPointF(*top_left), QtCore.QPointF(*bottom_right))
            shadow_rect = rect.adjusted(-3, -3, 3, 3)
            painter.setPen(QtCore.Qt.NoPen)
            painter.setBrush(QtGui.QColor(0, 0, 0, 90))
            painter.drawRoundedRect(shadow_rect, 6, 6)
            painter.drawImage(rect, self.map_image)

        for robot_ns, points in self.robot_paths.items():
            if not points or len(points) < 2:
                continue
            color = ROBOT_COLORS.get(robot_ns, QtGui.QColor(200, 200, 200))
            pen = QtGui.QPen(color, 3.0)
            pen.setStyle(QtCore.Qt.DashLine)
            pen.setCapStyle(QtCore.Qt.RoundCap)
            painter.setPen(pen)
            polyline = QtGui.QPolygonF([QtCore.QPointF(*self.world_to_screen(x, y)) for x, y in points])
            painter.drawPolyline(polyline)

        for robot_ns, point in self.pending_targets.items():
            if point is None:
                continue
            color = ROBOT_COLORS.get(robot_ns, QtGui.QColor(200, 200, 200))
            label = ROBOT_LABELS.get(robot_ns, robot_ns)
            sx, sy = self.world_to_screen(*point)

            pen = QtGui.QPen(color, 2.0, QtCore.Qt.DashLine)
            painter.setPen(pen)
            painter.setBrush(QtCore.Qt.NoBrush)
            r = 10
            painter.drawEllipse(QtCore.QPointF(sx, sy), r, r)
            painter.drawLine(QtCore.QPointF(sx - r, sy), QtCore.QPointF(sx + r, sy))
            painter.drawLine(QtCore.QPointF(sx, sy - r), QtCore.QPointF(sx, sy + r))
            self._draw_label_pill(painter, sx + r + 4, sy - 9, f"{label} 목표", color)

        for robot_ns, pose in self.robot_poses.items():
            if pose is None:
                continue
            x, y, yaw = pose
            sx, sy = self.world_to_screen(x, y)
            color = ROBOT_COLORS.get(robot_ns, QtGui.QColor(200, 200, 200))
            label = ROBOT_LABELS.get(robot_ns, robot_ns)

            if robot_ns == self.active_robot:
                pulse_color = QtGui.QColor(color)
                pulse_color.setAlpha(70)
                painter.setPen(QtCore.Qt.NoPen)
                painter.setBrush(pulse_color)
                painter.drawEllipse(QtCore.QPointF(sx, sy), 18, 18)
                ring_pen = QtGui.QPen(color, 1.5)
                painter.setPen(ring_pen)
                painter.setBrush(QtCore.Qt.NoBrush)
                painter.drawEllipse(QtCore.QPointF(sx, sy), 14, 14)

            painter.save()
            painter.translate(sx, sy)
            painter.rotate(-math.degrees(yaw))
            painter.setPen(QtCore.Qt.NoPen)
            painter.setBrush(QtGui.QColor(0, 0, 0, 110))
            painter.drawEllipse(QtCore.QPointF(1, 1.5), 9, 9)

            body_len, body_w = 18, 13
            triangle = QtGui.QPolygonF([
                QtCore.QPointF(body_len * 0.6, 0),
                QtCore.QPointF(-body_len * 0.35, body_w * 0.5),
                QtCore.QPointF(-body_len * 0.15, 0),
                QtCore.QPointF(-body_len * 0.35, -body_w * 0.5),
            ])
            painter.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 200), 1))
            painter.setBrush(color)
            painter.drawPolygon(triangle)
            painter.restore()

            self._draw_label_pill(painter, sx + 12, sy - 10, label, color)

        painter.end()

    def _draw_grid(self, painter):
        if self.map_info is None:
            return
        ox, oy, res, w, h = self.map_info
        step_m = 1.0
        x0, y0 = self.screen_to_world(0, self.height())
        x1, y1 = self.screen_to_world(self.width(), 0)
        pen = QtGui.QPen(QtGui.QColor(255, 255, 255, 14), 1)
        painter.setPen(pen)
        gx = math.floor(x0 / step_m) * step_m
        while gx <= x1:
            sx, _ = self.world_to_screen(gx, 0)
            painter.drawLine(QtCore.QPointF(sx, 0), QtCore.QPointF(sx, self.height()))
            gx += step_m
        gy = math.floor(y0 / step_m) * step_m
        while gy <= y1:
            _, sy = self.world_to_screen(0, gy)
            painter.drawLine(QtCore.QPointF(0, sy), QtCore.QPointF(self.width(), sy))
            gy += step_m

    def _draw_label_pill(self, painter, sx, sy, text, accent_color):
        fm = painter.fontMetrics()
        tw = fm.horizontalAdvance(text)
        pad_x, pad_y = 6, 3
        rect = QtCore.QRectF(sx, sy, tw + pad_x * 2, fm.height() + pad_y * 2 - 2)
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(15, 18, 22, 215))
        painter.drawRoundedRect(rect, 5, 5)
        painter.setPen(QtGui.QPen(accent_color, 1))
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.drawRoundedRect(rect, 5, 5)
        painter.setPen(QtGui.QColor(230, 233, 239))
        painter.drawText(rect, QtCore.Qt.AlignCenter, text)

    def wheelEvent(self, event):
        factor = 1.0015 ** event.angleDelta().y()
        self.scale = max(2.0, min(2000.0, self.scale * factor))
        self.update()

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            if self.goal_mode:
                wx, wy = self.screen_to_world(event.pos().x(), event.pos().y())
                self.set_pending_target(self.active_robot, (wx, wy))
                self.targetPinned.emit(self.active_robot, wx, wy)
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


# ---------------- 로봇 시뮬레이션 상태 ----------------

BATTERY_DRAIN_PER_SEC = 0.4  # 이동 중일 때만 배터리가 준다 (%/s)
BATTERY_IDLE_DRAIN_PER_SEC = 0.02


class SimRobot:
    def __init__(self, x, y, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.path = []  # 남은 world 좌표 waypoint 목록
        self.battery = 100.0
        self.speed = 0.0
        self.total_distance = 0.0  # 누적 이동 거리 (표시용)

    def pose(self):
        return (self.x, self.y, self.yaw)

    def remaining_distance(self):
        if not self.path:
            return 0.0
        pts = [(self.x, self.y)] + self.path
        return sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
                   for i in range(len(pts) - 1))

    def step(self, dt):
        if not self.path:
            self.speed = 0.0
            self.battery = max(0.0, self.battery - BATTERY_IDLE_DRAIN_PER_SEC * dt)
            return

        self.speed = ROBOT_SPEED
        self.battery = max(0.0, self.battery - BATTERY_DRAIN_PER_SEC * dt)

        tx, ty = self.path[0]
        dx, dy = tx - self.x, ty - self.y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            self.path.pop(0)
            return
        self.yaw = math.atan2(dy, dx)
        move = ROBOT_SPEED * dt
        if move >= dist:
            self.x, self.y = tx, ty
            self.path.pop(0)
            self.total_distance += dist
        else:
            self.x += dx / dist * move
            self.y += dy / dist * move
            self.total_distance += move


# ---------------- 메인 윈도우 ----------------

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, map_yaml_path):
        super().__init__()
        self.setWindowTitle("Pinky Fleet Control")
        self.setWindowIcon(make_app_icon())
        self.resize(720, 1160)
        self._sim_seconds = 0.0

        self.map_info = load_map(map_yaml_path)

        self.canvas = FleetMapCanvas()
        self.canvas.set_map(self.map_info)
        self.canvas.fit_to_map()

        origin_cell = nearest_free_cell(self.map_info, *world_to_grid(self.map_info, 0.0, 0.0))
        # 로봇2는 '직선거리가 먼 자유칸'이 아니라, 로봇1에서 실제로 갈 수 있는(연결된) 칸 중
        # 가장 먼 곳에 둔다 — 안 그러면 벽 너머 고립된 칸에 스폰돼서 서로 오갈 수 없다.
        reach = reachable_cells(self.map_info, origin_cell)
        reach_ys, reach_xs = np.where(reach)
        reachable_list = list(zip(reach_ys.tolist(), reach_xs.tolist()))
        far_cell = max(reachable_list, key=lambda rc: (rc[0] - origin_cell[0]) ** 2 + (rc[1] - origin_cell[1]) ** 2)

        r1x, r1y = grid_to_world(self.map_info, *origin_cell)
        r2x, r2y = grid_to_world(self.map_info, *far_cell)

        self.robots = {
            "robot1": SimRobot(r1x, r1y),
            "robot2": SimRobot(r2x, r2y),
        }
        for ns, robot in self.robots.items():
            self.canvas.set_robot_pose(ns, robot.pose())

        self._build_menu()

        header = self._build_header(map_yaml_path)
        side = self._build_side_panel()

        content = QtWidgets.QWidget()
        content_layout = QtWidgets.QVBoxLayout(content)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(0)
        content_layout.addWidget(self.canvas, 1)
        content_layout.addWidget(side)

        central = QtWidgets.QWidget()
        central_layout = QtWidgets.QVBoxLayout(central)
        central_layout.setContentsMargins(0, 0, 0, 0)
        central_layout.setSpacing(0)
        central_layout.addWidget(header)
        central_layout.addWidget(content, 1)
        self.setCentralWidget(central)

        self.status_bar = self.statusBar()
        self.status_bar.showMessage("준비됨 — 지도 클릭으로 목표 전송을 켜고 지도를 클릭하세요.")

        self.canvas.targetPinned.connect(self._on_target_pinned)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._on_tick)
        self.timer.start(TICK_MS)

    def _build_menu(self):
        menu_bar = self.menuBar()
        file_menu = menu_bar.addMenu("파일")
        exit_action = QtWidgets.QAction("종료", self)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        view_menu = menu_bar.addMenu("보기")
        fit_action = QtWidgets.QAction("지도 전체 보기", self)
        fit_action.triggered.connect(lambda: self.canvas.fit_to_map())
        view_menu.addAction(fit_action)

    def _build_header(self, map_yaml_path):
        header = QtWidgets.QWidget()
        header.setObjectName("header")
        layout = QtWidgets.QHBoxLayout(header)
        layout.setContentsMargins(16, 10, 16, 10)
        layout.setSpacing(10)

        logo = QtWidgets.QLabel()
        logo.setPixmap(make_app_icon().pixmap(30, 30))
        layout.addWidget(logo)

        title = QtWidgets.QLabel("Pinky Fleet Control")
        title.setObjectName("appTitle")
        subtitle = QtWidgets.QLabel(f"독립 시뮬레이터 · {Path(map_yaml_path).name}")
        subtitle.setObjectName("appSubtitle")

        title_box = QtWidgets.QVBoxLayout()
        title_box.setSpacing(0)
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        title_wrap = QtWidgets.QWidget()
        title_wrap.setLayout(title_box)

        self.clock_label = QtWidgets.QLabel("sim t = 0.0s")
        self.clock_label.setObjectName("clockLabel")

        layout.addWidget(title_wrap)
        layout.addStretch(1)
        layout.addWidget(self.clock_label)

        shadow = QtWidgets.QGraphicsDropShadowEffect(self)
        shadow.setBlurRadius(18)
        shadow.setOffset(0, 2)
        shadow.setColor(QtGui.QColor(0, 0, 0, 160))
        header.setGraphicsEffect(shadow)
        return header

    def _build_side_panel(self):
        side = QtWidgets.QWidget()
        side.setObjectName("sidePanel")
        side.setFixedHeight(340)
        form = QtWidgets.QHBoxLayout(side)
        form.setContentsMargins(14, 12, 14, 12)
        form.setSpacing(12)

        status_group = QtWidgets.QGroupBox("로봇 상태")
        status_layout = QtWidgets.QVBoxLayout(status_group)
        self.status_labels = {}
        for robot_ns in ("robot1", "robot2"):
            lbl = QtWidgets.QLabel("")
            lbl.setObjectName(f"status_{robot_ns}")
            lbl.setTextFormat(QtCore.Qt.RichText)
            lbl.setWordWrap(True)
            self.status_labels[robot_ns] = lbl
            status_layout.addWidget(lbl)
        status_layout.addStretch(1)
        self._apply_card_shadow(status_group)
        form.addWidget(status_group, 1)

        control_group = QtWidgets.QGroupBox("조작")
        control_layout = QtWidgets.QVBoxLayout(control_group)

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
        control_layout.addLayout(robot_select_row)

        self.goal_mode_btn = QtWidgets.QPushButton("지도 클릭으로 목표 지점 찍기")
        self.goal_mode_btn.setCheckable(True)
        self.goal_mode_btn.setObjectName("goalModeBtn")
        self.goal_mode_btn.toggled.connect(self.canvas.set_goal_mode)
        control_layout.addWidget(self.goal_mode_btn)

        dispatch_btn = QtWidgets.QPushButton("찍은 목표로 동시 출발")
        dispatch_btn.setObjectName("dispatchBtn")
        dispatch_btn.clicked.connect(self._on_dispatch_clicked)
        control_layout.addWidget(dispatch_btn)

        stop_btn = QtWidgets.QPushButton("정지 (조작 대상)")
        stop_btn.clicked.connect(self._on_stop_clicked)
        control_layout.addWidget(stop_btn)

        stop_all_btn = QtWidgets.QPushButton("전체 정지")
        stop_all_btn.setObjectName("dangerBtn")
        stop_all_btn.clicked.connect(self._on_stop_all_clicked)
        control_layout.addWidget(stop_all_btn)

        fit_btn = QtWidgets.QPushButton("지도 전체 보기")
        fit_btn.clicked.connect(self.canvas.fit_to_map)
        control_layout.addWidget(fit_btn)
        control_layout.addStretch(1)

        self._apply_card_shadow(control_group)
        form.addWidget(control_group, 1)

        log_group = QtWidgets.QGroupBox("로그")
        log_layout = QtWidgets.QVBoxLayout(log_group)
        self.action_log = QtWidgets.QLabel("")
        self.action_log.setWordWrap(True)
        self.action_log.setAlignment(QtCore.Qt.AlignTop)
        self.action_log.setObjectName("actionLog")
        log_layout.addWidget(self.action_log)
        log_layout.addStretch(1)
        self._apply_card_shadow(log_group)
        form.addWidget(log_group, 1)

        return side

    @staticmethod
    def _apply_card_shadow(widget):
        shadow = QtWidgets.QGraphicsDropShadowEffect(widget)
        shadow.setBlurRadius(14)
        shadow.setOffset(0, 2)
        shadow.setColor(QtGui.QColor(0, 0, 0, 120))
        widget.setGraphicsEffect(shadow)

    def _on_robot_selected(self, btn):
        for robot_ns, b in self.robot_select_buttons.items():
            if b is btn:
                self.canvas.set_active_robot(robot_ns)

    def _on_target_pinned(self, robot_ns, wx, wy):
        label = ROBOT_LABELS[robot_ns]
        self.action_log.setText(f"{label}: 목표 지점 지정됨 — '찍은 목표로 동시 출발'을 누르세요")
        self.status_bar.showMessage(f"{label} 목표 지점 지정 ({wx:+.2f}, {wy:+.2f})", 3000)

    def _plan_path(self, robot, target_xy):
        """A* 경로 계산. 성공하면 world 좌표 리스트, 실패하면 (None, 이유 문자열)."""
        start_cell = nearest_free_cell(self.map_info, *world_to_grid(self.map_info, robot.x, robot.y))
        goal_cell = nearest_free_cell(self.map_info, *world_to_grid(self.map_info, *target_xy))

        if start_cell is None or goal_cell is None:
            return None, "목표 지점을 찾을 수 없음"

        path_cells = astar(self.map_info["occupied"], start_cell, goal_cell)
        if not path_cells:
            return None, "경로 없음 (막혀 있음)"

        return [grid_to_world(self.map_info, r, c) for r, c in path_cells], None

    def _on_dispatch_clicked(self):
        pending = {ns: pt for ns, pt in self.canvas.pending_targets.items() if pt is not None}
        if not pending:
            self.status_bar.showMessage("찍어둔 목표가 없습니다 — 먼저 지도를 클릭하세요", 3000)
            return

        dispatched, failed = [], []
        for robot_ns, target_xy in pending.items():
            robot = self.robots[robot_ns]
            world_path, err = self._plan_path(robot, target_xy)
            if err:
                failed.append(f"{ROBOT_LABELS[robot_ns]}({err})")
                continue
            robot.path = world_path
            self.canvas.set_robot_path(robot_ns, world_path)
            self.canvas.set_pending_target(robot_ns, None)
            dispatched.append(ROBOT_LABELS[robot_ns])

        # 같은 함수 호출 안에서 전부 경로를 배정했으므로, 다음 _on_tick부터 동시에 움직인다.
        msg_parts = []
        if dispatched:
            msg_parts.append(f"{', '.join(dispatched)} 동시 출발")
        if failed:
            msg_parts.append(f"실패: {', '.join(failed)}")
        msg = " / ".join(msg_parts)
        self.action_log.setText(msg)
        self.status_bar.showMessage(msg, 4000)

    def _on_stop_clicked(self):
        robot_ns = self.canvas.active_robot
        robot = self.robots[robot_ns]
        robot.path = []
        self.canvas.set_robot_path(robot_ns, [])
        self.canvas.set_pending_target(robot_ns, None)
        self.action_log.setText(f"{ROBOT_LABELS[robot_ns]}: 정지")
        self.status_bar.showMessage(f"{ROBOT_LABELS[robot_ns]} 정지시킴", 3000)

    def _on_stop_all_clicked(self):
        for robot_ns, robot in self.robots.items():
            robot.path = []
            self.canvas.set_robot_path(robot_ns, [])
            self.canvas.set_pending_target(robot_ns, None)
        self.action_log.setText("전체 정지")
        self.status_bar.showMessage("모든 로봇 정지시킴", 3000)

    def _on_tick(self):
        dt = TICK_MS / 1000.0
        self._sim_seconds += dt
        self.clock_label.setText(f"sim t = {self._sim_seconds:5.1f}s")

        for robot_ns, robot in self.robots.items():
            had_path = bool(robot.path)
            robot.step(dt)
            self.canvas.set_robot_pose(robot_ns, robot.pose())
            if had_path and not robot.path:
                self.canvas.set_robot_path(robot_ns, [])
                self.status_bar.showMessage(f"{ROBOT_LABELS[robot_ns]} 목표 도착", 3000)

            moving = bool(robot.path)
            label = ROBOT_LABELS[robot_ns]
            lbl = self.status_labels[robot_ns]
            accent = ROBOT_COLORS[robot_ns].name()
            status_text = "이동중" if moving else "대기"
            status_color = "#3ddc84" if moving else "#8b93a1"

            filled = max(0, min(10, round(robot.battery / 10)))
            bar = "█" * filled + "░" * (10 - filled)
            batt_color = "#3ddc84" if robot.battery > 50 else ("#f5a623" if robot.battery > 20 else "#ff6b6b")

            remaining = robot.remaining_distance()
            eta = f"{remaining / ROBOT_SPEED:4.1f}s" if moving else "—"

            lbl.setText(
                f"<div style='font-weight:700;color:{accent};font-size:13px;'>&#9679; {label}</div>"
                f"<div style='color:{status_color};font-weight:600;margin-top:2px;'>{status_text}</div>"
                f"<div style='color:{batt_color};font-family:monospace;font-size:11px;margin-top:4px;'>"
                f"{bar} {robot.battery:3.0f}%</div>"
                f"<div style='color:#8b93a1;font-size:11px;margin-top:2px;'>"
                f"{robot.speed:.2f} m/s · 남은거리 {remaining:.2f} m · ETA {eta}</div>"
                f"<div style='color:#565f70;font-size:10px;margin-top:2px;'>"
                f"X {robot.x:+.2f}  Y {robot.y:+.2f} · 누적 {robot.total_distance:.1f} m</div>"
            )
            lbl.setStyleSheet(
                f"padding: 8px; border-radius: 8px; background-color: #1a2029; "
                f"border: 1px solid {'#2d3646' if not moving else accent};"
            )


def main():
    parser = argparse.ArgumentParser(description="Pinky Pro fleet 독립 시뮬레이터 (PyQt, ROS2/Gazebo 없음)")
    parser.add_argument("--map", default=DEFAULT_MAP_YAML, help="map_server용 yaml 경로")
    args, _ = parser.parse_known_args()

    app = QtWidgets.QApplication(sys.argv)
    app.setStyleSheet(APP_QSS)
    win = MainWindow(args.map)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
