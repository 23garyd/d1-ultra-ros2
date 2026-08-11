import os

import pytest

from d1_sim_wrapper import map_store
from d1_sim_wrapper.map_store import GridData


def make_grid():
    # 4x3 map: bottom row free, middle row unknown, top row occupied,
    # except cell (0,0) occupied to catch row-flip bugs.
    cells = [100, 0, 0, 0,
             -1, -1, -1, -1,
             100, 100, 100, 100]
    return GridData(width=4, height=3, resolution=0.05,
                    origin_x=-1.0, origin_y=-2.0, origin_yaw=0.0, cells=cells)


def test_save_load_round_trip(tmp_path):
    grid = make_grid()
    map_dir = map_store.save_map(grid, str(tmp_path), '2026_08_11_00_00_00')
    for name in map_store.REQUIRED_FILES:
        assert os.path.isfile(os.path.join(map_dir, name))

    loaded = map_store.load_map(map_dir)
    assert loaded.width == grid.width
    assert loaded.height == grid.height
    assert loaded.resolution == pytest.approx(grid.resolution)
    assert loaded.origin_x == pytest.approx(grid.origin_x)
    assert loaded.origin_y == pytest.approx(grid.origin_y)
    assert loaded.cells == grid.cells  # trinary values survive the round trip


def test_validate_map_dir(tmp_path):
    ok, msg = map_store.validate_map_dir('')
    assert not ok
    ok, msg = map_store.validate_map_dir(str(tmp_path / 'nope' / 'map.pcd'))
    assert not ok and 'does not exist' in msg

    map_dir = map_store.save_map(make_grid(), str(tmp_path), 'm1')
    os.remove(os.path.join(map_dir, 'map.pgm'))
    ok, msg = map_store.validate_map_dir(os.path.join(map_dir, 'map.pcd'))
    assert not ok and 'map.pgm' in msg

    map_dir = map_store.save_map(make_grid(), str(tmp_path), 'm2')
    ok, result = map_store.validate_map_dir(os.path.join(map_dir, 'map.pcd'))
    assert ok and result == map_dir


def test_cell_at_world():
    grid = make_grid()
    # bottom-left cell center is origin + half resolution; cells[0] == 100
    assert grid.cell_at_world(-0.975, -1.975) == 100
    assert grid.cell_at_world(-0.925, -1.975) == 0     # cells[1]
    assert grid.cell_at_world(-0.975, -1.925) == -1    # middle row unknown
    assert grid.cell_at_world(50.0, 50.0) is None      # outside


def test_load_rejects_non_trinary(tmp_path):
    map_dir = map_store.save_map(make_grid(), str(tmp_path), 'm3')
    yaml_path = os.path.join(map_dir, 'map.yaml')
    with open(yaml_path) as f:
        content = f.read().replace('trinary', 'scale')
    with open(yaml_path, 'w') as f:
        f.write(content)
    with pytest.raises(ValueError, match='trinary'):
        map_store.load_map(map_dir)


def test_map_id_format():
    map_id = map_store.generate_map_id()
    parts = map_id.split('_')
    assert len(parts) == 6 and all(p.isdigit() for p in parts)
