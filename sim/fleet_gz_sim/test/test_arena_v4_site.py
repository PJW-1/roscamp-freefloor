"""Real arena, fourth layout (v4, 2026-11-10 night): 1.98 x 0.99 m inside the walls and four
objects, taken cell by cell from the map the 07d7 Pinky scanned. The site, its lane map and its
scenario fit together and the passages fit a Pinky."""
from pathlib import Path

import pytest
import yaml

lane_map_module = pytest.importorskip("fleet_core.traffic.lane_map")
from fleet_gz_sim import arena  # noqa: E402
from fleet_gz_sim.fleet_setup import arena_for, load, start_poses  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
FAKE = PKG.parent / "fake_robots"
SITE = arena.load_site(PKG / "sites" / "arena_v4.yaml")
SCENARIO = PKG / "scenarios" / "gz_arena_v4.yaml"
EPS = 1e-9
SWEPT_M = 0.083  # a Pinky turning on the spot (docs/실측_2026-11-10.md)
SEEN = {  # outlines in the scan map: x0, y0, x1, y1
    "object1": (0.37, 0.39, 0.54, 0.55),
    "object2": (0.68, 0.05, 0.85, 0.34),
    "object3": (0.70, 0.55, 0.83, 0.82),
    "object4": (1.08, 0.25, 1.22, 0.76),
}


def outlines() -> dict:
    found = {}
    for box in SITE.features:
        name = box.name.split("_")[0]
        x0, y0, x1, y1 = found.get(name, (9.0, 9.0, -9.0, -9.0))
        found[name] = (min(x0, box.x0), min(y0, box.y0), max(x1, box.x1), max(y1, box.y1))
    return found


def test_the_walls_are_the_scanned_size():
    assert SITE.inside == (0.0, 0.0, 1.98, 0.99)
    assert SITE.lane_map == "arena_v4"
    assert SITE.lane_clearance_m == 0.08  # the lane north of object 3 has 0.085 m to each side


def test_the_four_objects_are_where_the_scan_map_has_them():
    got = outlines()
    assert sorted(got) == sorted(SEEN)
    for name, seen in SEEN.items():
        assert got[name] == pytest.approx(seen, abs=1e-6), name


def test_a_pinky_fits_through_every_passage_of_the_figure_eight():
    o = outlines()
    x0, y0, x1, y1 = SITE.inside
    ways = {
        "south of object 4": o["object4"][1] - y0,
        "north of object 4": y1 - o["object4"][3],
        "middle": o["object4"][0] - max(o["object2"][2], o["object3"][2]),
        "north of object 3": y1 - o["object3"][3],
        "between objects 2 and 3": o["object3"][1] - o["object2"][3],
    }
    assert all(width / 2 >= SWEPT_M - EPS for width in ways.values()), ways
    assert ways["north of object 3"] == pytest.approx(0.17)  # the narrowest
    assert o["object2"][1] - y0 < 0.11  # no way south of object 2
    assert x1 - o["object4"][2] >= 0.75  # the east part is empty


def test_the_generated_maps_are_the_scan_map_cell_by_cell():
    """Every 1 cm cell an object box touches is occupied: the boxes are the rows of the scan
    map, so the generated fine map has the walls, the objects and nothing else."""
    grid = arena.occupancy(0.01, SITE)
    inner = [row[15:-15] for row in grid[15:-15]]  # without the 0.15 m border and the walls
    assert len(inner) == 99 and len(inner[0]) == 198
    taken = sum(cell for row in inner for cell in row)
    boxes = sum(round((b.x1 - b.x0) * 100) * round((b.y1 - b.y0) * 100) for b in SITE.features)
    assert taken == boxes > 0
    fine_pgm = SITE.outputs()[4]
    assert (PKG / fine_pgm).read_bytes() == arena.pgm(0.01, SITE)


def occupied(path: Path) -> set:
    magic, width, height, _, pixels = path.read_bytes().split(None, 4)
    assert magic == b"P5" and len(pixels) == int(width) * int(height)
    return {(k % int(width), k // int(width)) for k, value in enumerate(pixels) if value < 100}


def test_the_generated_map_has_the_occupied_cells_of_the_map_built_from_the_scans():
    """docs/freefloor/arena_v4/scan_map_fine.pgm is what build_v4.py made of the lidar scans;
    the site boxes were taken from it (pgm_to_site.py)."""
    scan = PKG.parents[1] / "docs" / "freefloor" / "arena_v4" / "scan_map_fine.pgm"
    cells = occupied(scan)
    assert len(cells) > 1000
    assert occupied(PKG / SITE.outputs()[4]) == cells


def test_the_scenario_gets_its_world_maps_and_lane_map():
    found = arena_for(SCENARIO, PKG, FAKE)
    world, nav, _, fine, _ = SITE.outputs()
    assert found.name == SITE.name == "arena_v4"
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
    assert data["map"] == "arena_v4"


@pytest.mark.parametrize("name, robots, orders", [
    ("gz_arena_v4_two", ["pinky1", "pinky2"], 10),
    ("gz_arena_v4_slow_work", ["pinky1", "pinky2", "pinky3"], 8),
])
def test_the_scenarios_that_take_the_shortcut_more_run_on_the_same_site(name, robots, orders):
    path = PKG / "scenarios" / f"{name}.yaml"
    found = arena_for(path, PKG, FAKE)
    assert found.name == "arena_v4"
    site, scenario = load(path, found.lane_map)
    assert list(scenario.robots) == robots
    assert {cfg.start for cfg in scenario.robots.values()} <= set(site.parking)
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert len(data["orders"]) == data["expect"]["orders_shipped"] == orders
