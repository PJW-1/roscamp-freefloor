"""경기장 생성기: 레인과 벽 사이 거리, 저장소의 월드·지도가 생성기 결과와 같은지."""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

lane_map_module = pytest.importorskip("fleet_core.traffic.lane_map")
from fleet_gz_sim import arena  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
LANES = lane_map_module.load_lane_map()


def cell(grid, resolution, x, y):
    ox, oy = arena.MAP_ORIGIN
    return grid[int((y - oy) // resolution)][int((x - ox) // resolution)]


def test_every_lane_keeps_clear_of_walls_and_features():
    value, lane, box = arena.min_clearance(LANES)
    assert value >= arena.CLEARANCE_M, f"{lane} is {value:.3f} m from {box}"


def test_committed_world_and_maps_match_the_generator(tmp_path):
    arena.generate(tmp_path, LANES)
    for rel in arena.OUTPUTS:
        assert (PKG / rel).read_bytes() == (tmp_path / rel).read_bytes(), (
            f"{rel} differs: run python3 -m fleet_gz_sim.arena")


@pytest.mark.parametrize("resolution", [arena.COARSE_M, arena.FINE_M])
def test_waypoints_are_free_and_walls_are_occupied(resolution):
    grid = arena.occupancy(resolution)
    for lane in LANES.lanes.values():
        for t in (0.0, 0.5, 1.0):
            x, y = lane.ax + t * (lane.bx - lane.ax), lane.ay + t * (lane.by - lane.ay)
            assert not cell(grid, resolution, x, y), (lane.a, lane.b, t)
    for box in arena.obstacles():
        assert cell(grid, resolution, (box.x0 + box.x1) / 2, (box.y0 + box.y1) / 2), box.name


def test_coarse_and_fine_maps_agree():
    coarse, fine = arena.occupancy(arena.COARSE_M), arena.occupancy(arena.FINE_M)
    ox, oy = arena.MAP_ORIGIN
    for j, row in enumerate(coarse):
        for i, occupied in enumerate(row):
            x, y = ox + (i + 0.5) * arena.COARSE_M, oy + (j + 0.5) * arena.COARSE_M
            assert occupied == cell(fine, arena.FINE_M, x, y), (x, y)


def test_map_yaml_matches_the_world_frame():
    for name, resolution in (("lane_arena", arena.COARSE_M), ("lane_arena_fine", arena.FINE_M)):
        data = yaml.safe_load((PKG / "maps" / f"{name}.yaml").read_text(encoding="utf-8"))
        assert data["image"] == f"{name}.pgm" and data["resolution"] == resolution
        assert data["origin"] == [*arena.MAP_ORIGIN, 0.0]
        width, height = (PKG / "maps" / f"{name}.pgm").read_bytes().split(b"\n")[1].split()
        assert (int(width), int(height)) == (round(arena.MAP_SIZE[0] / resolution),
                                             round(arena.MAP_SIZE[1] / resolution))


def test_world_walls_sit_at_map_coordinates():
    """월드 좌표 = 지도 좌표 = lane_map 좌표(원점 [0, 0])."""
    assert LANES.origin == (0.0, 0.0)
    world = ET.parse(PKG / "worlds" / "lane_arena.sdf").getroot()
    poses = {c.get("name"): c.find("pose").text.split()
             for c in world.iter("collision") if c.find("pose") is not None}
    x0 = arena.INSIDE[0] - arena.WALL_M / 2
    assert math.isclose(float(poses["wall_west"][0]), x0)
