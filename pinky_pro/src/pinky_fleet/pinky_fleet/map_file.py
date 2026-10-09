"""Read a map_server map (YAML + image) into the dashboard map dict, without ROS.

The dashboard normally gets /map from Nav2's map_server. When only the dashboard
runs (vision_drive drives the robot, no Nav2), it shows the same map file itself.
Cell values follow nav2_map_server: 100 occupied, 0 free, -1 unknown, rows from
the bottom (origin) up.
"""
import hashlib
import json
from pathlib import Path

import yaml


def _read_pgm(raw):
    """P5 (binary) or P2 (ASCII) PGM bytes -> (width, height, maxval, values row-major from the top)."""
    tokens, index = [], 0
    while len(tokens) < 4:
        while index < len(raw) and raw[index:index + 1].isspace():
            index += 1
        if raw[index:index + 1] == b'#':
            end = raw.find(b'\n', index)
            index = len(raw) if end < 0 else end + 1
            continue
        start = index
        while index < len(raw) and not raw[index:index + 1].isspace():
            index += 1
        tokens.append(raw[start:index])
    magic, width, height, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    count = width * height
    if magic == b'P5':
        body = raw[index + 1:]
        if maxval < 256:
            values = list(body[:count])
        else:
            values = [int.from_bytes(body[i:i + 2], 'big') for i in range(0, 2 * count, 2)]
    elif magic == b'P2':
        values = [int(token) for token in raw[index:].split()[:count]]
    else:
        raise ValueError(f'PGM이 아닙니다: {magic!r}')
    if len(values) != count:
        raise ValueError('PGM 크기와 데이터 길이가 다릅니다')
    return width, height, maxval, values


def _read_image(path):
    raw = Path(path).read_bytes()
    if raw[:2] in (b'P5', b'P2'):
        return _read_pgm(raw)
    import cv2   # PNG 등: 필요할 때만
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f'지도 이미지를 읽을 수 없습니다: {path}')
    return image.shape[1], image.shape[0], 255, image.ravel().tolist()


def load_map(yaml_path):
    """map_server YAML -> (map_id, map dict) in the same shape as Robot.on_map."""
    yaml_path = Path(yaml_path).expanduser()
    config = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    width, height, maxval, pixels = _read_image(yaml_path.parent / config['image'])
    mode = config.get('mode', 'trinary')
    negate = bool(config.get('negate', 0))
    occupied, free = float(config['occupied_thresh']), float(config['free_thresh'])
    data = [0] * (width * height)
    for row in range(height):
        for col in range(width):
            value = pixels[row * width + col]
            if mode == 'raw':
                cell = value if value <= 100 else -1
            else:
                shade = value / maxval
                occ = shade if negate else 1.0 - shade
                if occ > occupied:
                    cell = 100
                elif occ < free:
                    cell = 0
                elif mode == 'scale':
                    cell = int(round((occ - free) / (occupied - free) * 100.0))
                else:
                    cell = -1
            data[(height - 1 - row) * width + col] = cell     # image top row = largest y
    origin = config.get('origin', [0.0, 0.0, 0.0])
    grid = dict(width=width, height=height, resolution=float(config['resolution']),
                origin=dict(x=float(origin[0]), y=float(origin[1]), yaw=float(origin[2])), data=data)
    digest = hashlib.sha256(json.dumps(grid, sort_keys=True).encode()).hexdigest()
    return digest, grid
