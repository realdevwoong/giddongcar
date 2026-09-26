#!/usr/bin/env python3
"""map_server용 yaml+pgm(good.yaml 등)을 읽어서, 검은 픽셀(벽)을 그대로 박스로 채운
Gazebo SDF 월드를 만든다.

목적: Nav2/AMCL이 이 지도를 기준으로 라이다 스캔을 정합시키려면, 시뮬레이션 안의
실제 벽 배치도 이 지도와 같아야 한다. pinky_factory.world 같은 기본 월드는 이 지도와
무관한 임의의 공간이라 위치가 어긋난다 — 이 스크립트로 지도 자체를 월드로 만든다.

사용법:
    python3 map_to_world.py /home/devwoong/good.yaml -o /path/to/good_map.world
    python3 map_to_world.py /home/devwoong/good.yaml -o ~/giddongcar/pinky_pro/src/pinky_gz_sim/worlds/good_map.world

이후 시뮬레이션 실행 시:
    ros2 launch pinky_gz_sim launch_sim.launch.xml world_name:=good_map.world
"""
import argparse
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

WALL_HEIGHT = 0.5
WALL_THICKNESS_MARGIN = 1.001  # 인접 박스 사이 미세한 틈 방지용 여유


def load_occupancy(yaml_path):
    yaml_path = Path(yaml_path)
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)

    image_path = yaml_path.parent / meta["image"]
    img = np.array(Image.open(image_path).convert("L"), dtype=np.float64)

    if meta.get("negate", 0):
        prob = img / 255.0
    else:
        prob = (255.0 - img) / 255.0

    occupied_thresh = meta.get("occupied_thresh", 0.65)
    occupied = prob > occupied_thresh

    return {
        "occupied": occupied,  # [row, col], row 0 = 이미지 맨 위 = world +y 쪽
        "resolution": meta["resolution"],
        "origin_x": meta["origin"][0],
        "origin_y": meta["origin"][1],
        "height": occupied.shape[0],
        "width": occupied.shape[1],
    }


def row_to_world_y(row, height, origin_y, resolution):
    # 이미지 row 0(맨 위) = world 상 가장 큰 y. fleet_map_view.py의 렌더링 규칙과 동일.
    return origin_y + (height - row - 0.5) * resolution


def col_to_world_x(col, origin_x, resolution):
    return origin_x + (col + 0.5) * resolution


def extract_wall_boxes(occ):
    """occupied 배열을 행 단위 run-length로 병합해서 (cx, cy, width, depth) 박스 목록 생성."""
    boxes = []
    occupied = occ["occupied"]
    resolution = occ["resolution"]
    height = occ["height"]
    origin_x = occ["origin_x"]
    origin_y = occ["origin_y"]

    for row in range(height):
        col = 0
        row_cells = occupied[row]
        width = len(row_cells)
        while col < width:
            if not row_cells[col]:
                col += 1
                continue
            start = col
            while col < width and row_cells[col]:
                col += 1
            end = col  # [start, end) 구간이 점유됨

            x0 = col_to_world_x(start, origin_x, resolution) - resolution / 2.0
            x1 = col_to_world_x(end - 1, origin_x, resolution) + resolution / 2.0
            cx = (x0 + x1) / 2.0
            box_w = (x1 - x0) * WALL_THICKNESS_MARGIN

            cy = row_to_world_y(row, height, origin_y, resolution)
            box_d = resolution * WALL_THICKNESS_MARGIN

            boxes.append((cx, cy, box_w, box_d))

    return boxes


def render_world_sdf(boxes, world_name):
    links = []
    for i, (cx, cy, w, d) in enumerate(boxes):
        links.append(f"""
      <link name="wall_{i}">
        <pose>{cx:.4f} {cy:.4f} {WALL_HEIGHT / 2:.4f} 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box><size>{w:.4f} {d:.4f} {WALL_HEIGHT:.4f}</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{w:.4f} {d:.4f} {WALL_HEIGHT:.4f}</size></box>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>
      </link>""")

    links_xml = "".join(links)

    return f"""<?xml version="1.0"?>
<sdf version="1.8">
  <world name="{world_name}">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 5 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.3 0.3 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="map_walls">
      <static>true</static>{links_xml}
    </model>

    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
  </world>
</sdf>
"""


def main():
    global WALL_HEIGHT

    parser = argparse.ArgumentParser(description="map_server yaml/pgm -> Gazebo SDF world")
    parser.add_argument("map_yaml", help="예: /home/devwoong/good.yaml")
    parser.add_argument("-o", "--output", required=True, help="생성할 .world 파일 경로")
    parser.add_argument("--wall-height", type=float, default=WALL_HEIGHT)
    args = parser.parse_args()

    WALL_HEIGHT = args.wall_height

    occ = load_occupancy(args.map_yaml)
    boxes = extract_wall_boxes(occ)
    world_name = Path(args.output).stem

    sdf = render_world_sdf(boxes, world_name)

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(sdf)

    print(f"{len(boxes)}개 벽 박스 생성 -> {out_path}")
    print(f"맵 크기: {occ['width']}x{occ['height']} px, resolution={occ['resolution']}, "
          f"origin=({occ['origin_x']}, {occ['origin_y']})")


if __name__ == "__main__":
    main()
