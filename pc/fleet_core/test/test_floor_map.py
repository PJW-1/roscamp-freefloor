"""자유바닥 지도(P1): 이미지에서 빈 바닥 읽기, 칸, 고정 선, 레인 지도와 같은 좌표."""
import math
from pathlib import Path

import pytest
from fleet_core.common.params import load_params
from fleet_core.traffic.floor.floor_map import (
    DEFAULT_FLOOR,
    FloorMap,
    FloorMapError,
    load_floor_map,
)
from fleet_core.traffic.lane_map import load_lane_map

REPO = Path(__file__).resolve().parents[3]
FEATURE_FLOOR = DEFAULT_FLOOR.parent / "feature_floor_map.yaml"


@pytest.fixture(scope="module")
def floor():
    return load_floor_map()


def test_the_image_is_the_gazebo_map_byte_for_byte():
    shipped = DEFAULT_FLOOR.parent / "lane_arena.pgm"
    gazebo = REPO / "sim" / "fleet_gz_sim" / "maps" / "lane_arena.pgm"
    assert shipped.read_bytes() == gazebo.read_bytes()


def test_the_grid_covers_the_image_in_tenth_metre_cells(floor):
    assert floor.cell_m == 0.10
    assert (floor.cols, floor.rows) == (40, 24)  # 80 x 49 px of 0.05 m; the half row is left out
    assert floor.bounds()[:2] == (-0.15, 0.1)


def test_walls_and_boxes_are_not_free(floor):
    assert floor.cell_of(-0.05, 1.0) not in floor.free  # west wall
    assert floor.cell_of(0.25, 0.35) not in floor.free  # block_sw
    assert floor.cell_of(2.90, 2.25) not in floor.free  # block_n
    assert floor.cell_of(1.85, 1.15) in floor.free  # the avenue


def test_clear_cells_keep_the_body_and_margin_off_the_walls(floor):
    body = floor.robot_radius_m + floor.cell_margin_m
    assert floor.clear < floor.free
    for cell in floor.clear:
        x, y = floor.centre(cell)
        assert floor._gap_to_wall(x, y) >= body
    assert floor.cell_of(0.05, 1.5) not in floor.clear  # 0.05 m from the west wall


def test_a_centre_anywhere_in_a_clear_cell_keeps_the_body_off_the_walls(floor):
    """The planner checks only which cell a point is in, so the whole cell must be safe."""
    body = floor.robot_radius_m + floor.cell_margin_m
    for cell in floor.clear:
        x0, y0 = floor.centre(cell)
        for dx in (-0.05, 0.0, 0.05):
            for dy in (-0.05, 0.0, 0.05):
                assert floor._gap_to_wall(x0 + dx, y0 + dy) >= body - 1e-9, (cell, dx, dy)


def test_cell_ids_go_both_ways(floor):
    cell = floor.cell_of(1.05, 1.55)
    assert floor.cell_at(floor.cell_id(cell)) == cell
    assert floor.cell_at("nonsense") is None
    x, y = floor.centre(cell)
    assert floor.cell_of(x, y) == cell
    assert floor.box_distance(cell, x, y) == 0.0
    assert math.isclose(floor.box_distance(cell, x + 0.25, y), 0.20)


def test_docks_and_parking_match_the_lane_map(floor):
    lanes = load_lane_map()
    for name, station in lanes.stations.items():
        assert floor.docks[name] == lanes.waypoint_pose(station.dock)
    assert [spot for spot, _ in floor.parking] == list(lanes.parking)
    for spot, pose in floor.parking:
        assert pose == lanes.waypoint_pose(spot)


def test_fixed_lines_follow_the_dock_lanes_in_their_direction(floor):
    lanes = load_lane_map()
    for line in floor.fixed_lines.values():
        for a, b in zip(line.points, line.points[1:], strict=False):
            ends = {(lane.ax, lane.ay, lane.bx, lane.by) for lane in lanes.lanes.values()}
            assert (*a, *b) in ends, f"{line.name} {a}->{b} is not a lane in its direction"
    assert floor.fixed_lines["work_in"].end == floor.docks["WORK"]
    assert floor.fixed_lines["work_out"].start == floor.docks["WORK"]


def test_the_radius_is_the_fleet_radius(floor):
    assert floor.robot_radius_m == load_params().robot_radius_m


def test_a_dock_on_a_wall_is_refused():
    data = {"bounds": [0, 0, 1, 1], "cell_m": 0.1, "robot_radius_m": 0.08,
            "cell_margin_m": 0.05, "docks": {"WORK": [0.05, 0.5]}}
    with pytest.raises(FloorMapError, match="WORK"):
        FloorMap(data)


def test_a_fixed_line_through_a_box_is_refused():
    data = {"bounds": [0, 0, 2, 1], "boxes": [[0.9, 0.0, 1.1, 1.0]], "cell_m": 0.1,
            "robot_radius_m": 0.08, "cell_margin_m": 0.05,
            "fixed_lines": {"across": [[0.3, 0.5], [1.7, 0.5]]}}
    with pytest.raises(FloorMapError, match="across"):
        FloorMap(data)


def test_the_feature_floor_matches_the_feature_lane_map():
    floor = load_floor_map(FEATURE_FLOOR)
    lanes = load_lane_map(REPO / "sim" / "fake_robots" / "maps" / "feature_map.yaml")
    for name, station in lanes.stations.items():
        assert floor.docks[name] == lanes.waypoint_pose(station.dock)
    assert [spot for spot, _ in floor.parking] == list(lanes.parking)
    # the tunnel is one cell set driven both ways
    east, west = floor.fixed_lines["tunnel_east"], floor.fixed_lines["tunnel_west"]
    assert east.points == tuple(reversed(west.points))


# ---- Codex P1 review --------------------------------------------------------------------


def floor_data(**changes):
    data = {"bounds": [0, 0, 1, 1], "cell_m": 0.1, "robot_radius_m": 0.08, "cell_margin_m": 0.05}
    data.update(changes)
    return data


def test_a_cell_that_is_not_whole_pixels_is_refused():
    with pytest.raises(FloorMapError, match="cell_m"):
        FloorMap(floor_data(cell_m=0.12))


def test_a_thin_box_still_blocks_its_pixels():
    floor = FloorMap(floor_data(bounds=[0, 0, 2, 1], boxes=[[1.0, 0.0, 1.01, 1.0]]))
    assert floor.cell_of(1.005, 0.5) not in floor.free


def write_pgm(path, header, pixels):
    path.write_bytes(header + bytes(pixels))
    (path.parent / "m.yaml").write_bytes(
        f"image: {path.name}\nresolution: 0.05\norigin: [0, 0, 0]\ncell_m: 0.1\n"
        "robot_radius_m: 0.08\ncell_margin_m: 0.05\n".encode())
    return path.parent / "m.yaml"


def test_a_pgm_with_another_maxval_is_refused(tmp_path):
    yaml_path = write_pgm(tmp_path / "m.pgm", b"P5\n4 4\n1000\n", [0] * 32)
    with pytest.raises(FloorMapError, match="maxval"):
        load_floor_map(yaml_path)


def test_a_cut_pgm_header_is_refused_not_looped(tmp_path):
    yaml_path = write_pgm(tmp_path / "m.pgm", b"P5\n4", [])
    with pytest.raises(FloorMapError, match="header"):
        load_floor_map(yaml_path)
