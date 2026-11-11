"""Real arena, fifth layout, a proposal (v5, 2026-11-11): the walls and the four objects of v4,
put elsewhere so that no passage is a narrow line. Two small islands on the middle line, the two
long objects against the north and the south wall in the west part. The site, its lane map and
its scenario fit together."""
from pathlib import Path

import pytest
import yaml

lane_map_module = pytest.importorskip("fleet_core.traffic.lane_map")
from fleet_gz_sim import arena  # noqa: E402
from fleet_gz_sim.fleet_setup import arena_for, load, start_poses  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
FAKE = PKG.parent / "fake_robots"
SITE = arena.load_site(PKG / "sites" / "arena_v5.yaml")
V4 = arena.load_site(PKG / "sites" / "arena_v4.yaml")
SCENARIO = PKG / "scenarios" / "gz_arena_v5.yaml"
TWO_WAY_M = 0.41  # 양쪽 방향으로 다니는 통로의 최소 폭
CROSS_M = 0.34    # 두 섬 사이 건너는 통로


def outlines(site) -> dict:
    found = {}
    for box in site.features:
        name = box.name.split("_")[0]
        x0, y0, x1, y1 = found.get(name, (9.0, 9.0, -9.0, -9.0))
        found[name] = (min(x0, box.x0), min(y0, box.y0), max(x1, box.x1), max(y1, box.y1))
    return found


def sides(box) -> list:
    x0, y0, x1, y1 = box
    return sorted([round(x1 - x0, 3), round(y1 - y0, 3)])


def test_the_walls_are_those_of_v4():
    assert SITE.inside == V4.inside == (0.0, 0.0, 1.98, 0.99)
    assert SITE.lane_map == "arena_v5"


def test_the_four_objects_have_the_sizes_the_scan_of_v4_gave_them():
    """자리와 방향만 바꿨다: 물체마다 긴 변과 짧은 변이 v4 스캔의 바깥 치수와 같다."""
    here, there = outlines(SITE), outlines(V4)
    assert sorted(here) == sorted(there) == ["object1", "object2", "object3", "object4"]
    for name in here:
        assert sides(here[name]) == sides(there[name]), name


def test_the_two_islands_leave_a_two_way_passage_north_and_south_and_a_way_between():
    o = outlines(SITE)
    _, y0, _, y1 = SITE.inside
    for name in ("object1", "object3"):
        assert o[name][1] - y0 >= TWO_WAY_M - 1e-9, name
        assert y1 - o[name][3] >= TWO_WAY_M - 1e-9, name
    assert o["object3"][0] - o["object1"][2] == pytest.approx(CROSS_M)


def test_the_two_long_objects_stand_against_the_walls_of_the_west_part():
    o = outlines(SITE)
    x0, y0, _, y1 = SITE.inside
    assert o["object4"][0] == x0 and o["object4"][3] == y1  # 북서쪽 구석
    assert o["object2"][0] == x0 and o["object2"][1] == y0  # 남서쪽 구석
    # 서쪽 구역: 섬 앞까지 0.5 m가 비어 있고, 두 선반 사이도 그만큼 넓다
    assert o["object1"][0] - x0 >= 0.50
    assert o["object4"][1] - o["object2"][3] >= 0.60
    # 동쪽 구역은 비어 있다
    assert SITE.inside[2] - max(box[2] for box in o.values()) >= 0.65


def test_the_generated_fine_map_is_the_one_the_site_gives():
    grid = arena.occupancy(0.01, SITE)
    inner = [row[15:-15] for row in grid[15:-15]]  # 테두리 0.15 m와 벽을 뺀 안쪽
    assert len(inner) == 99 and len(inner[0]) == 198
    assert (PKG / SITE.outputs()[4]).read_bytes() == arena.pgm(0.01, SITE)


def test_the_scenario_gets_its_world_maps_and_lane_map():
    found = arena_for(SCENARIO, PKG, FAKE)
    world, nav, _, fine, _ = SITE.outputs()
    assert found.name == SITE.name == "arena_v5"
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
    assert data["map"] == "arena_v5"
    assert len(data["orders"]) == data["expect"]["orders_shipped"] == 8
