"""Measured arena (2026-11-10): 2.00 x 1.00 m inside the walls and the two temporary objects of
that day. The site, its lane map and its scenario fit together and the corridors fit a Pinky."""
from pathlib import Path

import pytest
import yaml

lane_map_module = pytest.importorskip("fleet_core.traffic.lane_map")
from fleet_gz_sim import arena  # noqa: E402
from fleet_gz_sim.fleet_setup import arena_for, load, start_poses  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
FAKE = PKG.parent / "fake_robots"
SITE = arena.load_site(PKG / "sites" / "arena_1110.yaml")
SCENARIO = PKG / "scenarios" / "gz_arena_1110.yaml"
EPS = 1e-9


def test_the_walls_are_the_measured_two_by_one_metres():
    assert SITE.inside == (0.0, 0.0, 2.00, 1.00)
    assert SITE.lane_clearance_m == 0.10  # the WORK lane runs down the middle of the 0.20 m gap


def test_the_two_objects_are_where_the_lidar_saw_them_to_five_centimetres():
    """Measured outlines: x 0.64-0.83, y 0.43-0.64 and x 1.04-1.32, y 0.39-0.60."""
    west, east = sorted(SITE.features, key=lambda b: b.x0)
    for box, seen in ((west, (0.64, 0.43, 0.83, 0.64)), (east, (1.04, 0.39, 1.32, 0.60))):
        got = (box.x0, box.y0, box.x1, box.y1)
        assert all(abs(a - b) <= 0.025 + EPS for a, b in zip(got, seen)), (box.name, got)


def test_a_pinky_fits_round_the_objects_and_docks_between_them():
    """The gap is the WORK dock: 0.20 m (0.21 m measured). A Pinky is 0.11 m wide and sweeps
    0.083 m when it turns, so the centre line leaves 0.045 m beside it and 0.017 m in a turn."""
    west, east = sorted(SITE.features, key=lambda b: b.x0)
    x0, y0, x1, y1 = SITE.inside
    round_them = [west.x0 - x0, x1 - east.x1, west.y0 - y0, east.y0 - y0, y1 - west.y1,
                  y1 - east.y1]
    assert all(gap >= 0.35 - EPS for gap in round_them), round_them
    gap = east.x0 - west.x1
    assert gap == pytest.approx(0.20) and gap / 2 >= 0.083 + 0.01
    lanes = lane_map_module.load_lane_map(FAKE / "maps" / "arena_1110.yaml")
    dock = lanes.waypoint_pose(lanes.stations["WORK"].dock)
    assert dock[0] == pytest.approx((west.x1 + east.x0) / 2)
    assert max(west.y0, east.y0) <= dock[1] <= min(west.y1, east.y1)


def test_the_scenario_gets_its_world_maps_and_lane_map():
    found = arena_for(SCENARIO, PKG, FAKE)
    world, nav, _, fine, _ = SITE.outputs()
    assert found.name == SITE.name == "arena_1110"
    assert (found.world, found.nav_map, found.nav_map_fine) == tuple(
        str(PKG / rel) for rel in (world, nav, fine))
    assert found.lane_map == str(FAKE / "maps" / f"{SITE.lane_map}.yaml")
    for path in (found.world, found.nav_map, found.nav_map_fine, found.lane_map):
        assert Path(path).is_file(), path


def test_the_three_pinkys_start_on_the_three_parking_spots():
    site, scenario = load(SCENARIO, arena_for(SCENARIO, PKG, FAKE).lane_map)
    poses = start_poses(site, scenario)
    assert list(scenario.robots) == ["pinky1", "pinky2", "pinky3"]
    assert sorted(cfg.start for cfg in scenario.robots.values()) == sorted(site.parking)
    for robot, cfg in scenario.robots.items():
        assert poses[robot][:2] == pytest.approx(site.waypoint_pose(cfg.start))
    data = yaml.safe_load(SCENARIO.read_text(encoding="utf-8"))
    assert data["map"] == "arena_1110"
