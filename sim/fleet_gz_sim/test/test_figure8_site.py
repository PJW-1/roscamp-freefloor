"""Figure-8 arena: the real arena's walls with two islands, so the floor runs round both like an
8. The site, its lane map and its scenario fit together and the corridors fit a Pinky."""
import itertools
from pathlib import Path

import pytest
import yaml

lane_map_module = pytest.importorskip("fleet_core.traffic.lane_map")
from fleet_gz_sim import arena  # noqa: E402
from fleet_gz_sim.fleet_setup import arena_for, load, start_poses  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
FAKE = PKG.parent / "fake_robots"
SITE = arena.load_site(PKG / "sites" / "figure8_arena.yaml")
SCENARIO = PKG / "scenarios" / "gz_figure8_arena.yaml"
EPS = 1e-9


def islands() -> list[tuple[float, float, float, float]]:
    """Bounding boxes of the groups of boxes that touch each other, west first."""
    groups: list[list[arena.Box]] = []
    for box in SITE.features:
        near = [g for g in groups if any(
            box.x0 <= b.x1 + EPS and b.x0 <= box.x1 + EPS and box.y0 <= b.y1 + EPS
            and b.y0 <= box.y1 + EPS for b in g)]
        merged = [box, *itertools.chain.from_iterable(near)]
        groups = [g for g in groups if g not in near] + [merged]
    return sorted((min(b.x0 for b in g), min(b.y0 for b in g), max(b.x1 for b in g),
                   max(b.y1 for b in g)) for g in groups)


def test_the_site_has_the_real_arena_walls_and_two_islands():
    real = arena.load_site(PKG / "sites" / "real_arena.yaml")
    assert SITE.inside == real.inside == (0.0, 0.0, 2.05, 1.05)
    assert SITE.lane_clearance_m == real.lane_clearance_m
    assert len(islands()) == 2


def test_the_corridors_round_the_islands_fit_a_pinky():
    """0.28-0.35 m round the outside (Pinky 12 cm, clear cells need 0.09 m to a wall), and a
    wider middle where the two loops of the 8 meet."""
    (lx0, ly0, lx1, ly1), (rx0, ry0, rx1, ry1) = islands()
    x0, y0, x1, y1 = SITE.inside
    outer = [lx0 - x0, x1 - rx1, ly0 - y0, ry0 - y0, y1 - ly1, y1 - ry1]
    assert all(0.28 - EPS <= gap <= 0.35 + EPS for gap in outer), outer
    assert rx0 - lx1 >= 0.50 - EPS


def test_the_figure8_scenario_gets_its_world_maps_and_lane_map():
    found = arena_for(SCENARIO, PKG, FAKE)
    world, nav, _, fine, _ = SITE.outputs()
    assert found.name == SITE.name == "figure8_arena"
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
    assert data["map"] == "figure8_arena"
