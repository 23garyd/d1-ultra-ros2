"""Saved-map store: <map_root>/<map_id>/{map.yaml, map.pgm, map.pcd}.

Mirrors the D1-Max layout /ota/alg_data/map/history_map/<map_id>/ and the
nav2 map_saver trinary PGM conventions, so files written here are readable by
standard ROS tooling and vice versa. Pure Python (PyYAML only) — no rclpy —
so it is directly unit-testable.
"""
import math
import os
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import yaml

REQUIRED_FILES = ('map.yaml', 'map.pgm', 'map.pcd')

FREE_PIXEL = 254
OCCUPIED_PIXEL = 0
UNKNOWN_PIXEL = 205
OCCUPIED_THRESH = 0.65
# 0.196 is the nav2 map_saver default; must stay below 205-unknown's
# (255-205)/255 = 0.19608 so unknown pixels reload as -1, not free.
FREE_THRESH = 0.196

PCD_EMPTY = """# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH 0
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 0
DATA ascii
"""


@dataclass
class GridData:
    """rclpy-free mirror of nav_msgs/OccupancyGrid content."""
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    cells: List[int] = field(repr=False, default_factory=list)  # row-major, row 0 = bottom

    def cell_at_world(self, x: float, y: float) -> Optional[int]:
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        if 0 <= col < self.width and 0 <= row < self.height:
            return self.cells[row * self.width + col]
        return None


def generate_map_id(now: Optional[time.struct_time] = None) -> str:
    return time.strftime('%Y_%m_%d_%H_%M_%S', now or time.localtime())


def save_map(grid: GridData, map_root: str, map_id: str) -> str:
    """Write map.pgm/map.yaml/map.pcd; returns the map directory path."""
    map_dir = os.path.join(os.path.expanduser(map_root), map_id)
    os.makedirs(map_dir, exist_ok=True)

    pgm_path = os.path.join(map_dir, 'map.pgm')
    with open(pgm_path, 'wb') as f:
        f.write(b'P5\n# created by d1_sim_wrapper\n%d %d\n255\n'
                % (grid.width, grid.height))
        # PGM rows run top-down; OccupancyGrid row 0 is the bottom.
        for row in range(grid.height - 1, -1, -1):
            start = row * grid.width
            f.write(bytes(_occupancy_to_pixel(v)
                          for v in grid.cells[start:start + grid.width]))

    with open(os.path.join(map_dir, 'map.yaml'), 'w') as f:
        yaml.safe_dump({
            'image': 'map.pgm',
            'mode': 'trinary',
            'resolution': float(grid.resolution),
            'origin': [float(grid.origin_x), float(grid.origin_y), float(grid.origin_yaw)],
            'negate': 0,
            'occupied_thresh': OCCUPIED_THRESH,
            'free_thresh': FREE_THRESH,
        }, f, default_flow_style=False, sort_keys=False)

    with open(os.path.join(map_dir, 'map.pcd'), 'w') as f:
        f.write(PCD_EMPTY)

    return map_dir


def _occupancy_to_pixel(v: int) -> int:
    if v < 0:
        return UNKNOWN_PIXEL
    if v > 100 * OCCUPIED_THRESH:
        return OCCUPIED_PIXEL
    if v < 100 * FREE_THRESH:
        return FREE_PIXEL
    return UNKNOWN_PIXEL


def validate_map_dir(pcd_path: str) -> Tuple[bool, str]:
    """Contract §3: the map directory must contain all required files."""
    if not pcd_path:
        return False, 'pcd_path is empty'
    map_dir = os.path.dirname(os.path.expanduser(pcd_path))
    if not os.path.isdir(map_dir):
        return False, f'map directory does not exist: {map_dir}'
    missing = [n for n in REQUIRED_FILES
               if not os.path.isfile(os.path.join(map_dir, n))]
    if missing:
        return False, f'map directory {map_dir} is missing: {", ".join(missing)}'
    return True, map_dir


def load_map(map_dir: str) -> GridData:
    """Load map.yaml + map.pgm back into a GridData (inverse of save_map)."""
    with open(os.path.join(map_dir, 'map.yaml')) as f:
        meta = yaml.safe_load(f)
    if meta.get('mode', 'trinary') != 'trinary':
        raise ValueError(f"unsupported map mode {meta.get('mode')!r}; only trinary")
    negate = int(meta.get('negate', 0))
    occupied_thresh = float(meta.get('occupied_thresh', OCCUPIED_THRESH))
    free_thresh = float(meta.get('free_thresh', FREE_THRESH))
    origin = meta.get('origin', [0.0, 0.0, 0.0])

    image = meta.get('image', 'map.pgm')
    pgm_path = image if os.path.isabs(image) else os.path.join(map_dir, image)
    width, height, maxval, pixels = _read_pgm_p5(pgm_path)

    cells = [-1] * (width * height)
    for pgm_row in range(height):
        grid_row = height - 1 - pgm_row  # PGM row 0 = top; grid row 0 = bottom
        for col in range(width):
            v = pixels[pgm_row * width + col]
            p = v / maxval if negate else (maxval - v) / maxval
            if p > occupied_thresh:
                occ = 100
            elif p < free_thresh:
                occ = 0
            else:
                occ = -1
            cells[grid_row * width + col] = occ

    return GridData(width=width, height=height,
                    resolution=float(meta['resolution']),
                    origin_x=float(origin[0]), origin_y=float(origin[1]),
                    origin_yaw=float(origin[2]) if len(origin) > 2 else 0.0,
                    cells=cells)


def _read_pgm_p5(path: str) -> Tuple[int, int, int, bytes]:
    with open(path, 'rb') as f:
        data = f.read()
    tokens = []
    pos = 0
    while len(tokens) < 4:
        while pos < len(data) and data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            while pos < len(data) and data[pos] != 0x0A:
                pos += 1
            continue
        start = pos
        while pos < len(data) and not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    if tokens[0] != b'P5':
        raise ValueError(f'unsupported PGM magic {tokens[0]!r}; only binary P5')
    width, height, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    pos += 1  # exactly one whitespace byte after maxval
    pixels = data[pos:pos + width * height]
    if len(pixels) != width * height:
        raise ValueError(f'PGM truncated: expected {width * height} bytes, got {len(pixels)}')
    return width, height, maxval, pixels


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """(x, y, z, w) for a pure-yaw rotation."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)
