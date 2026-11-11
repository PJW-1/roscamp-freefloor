"""v4 경기장 양방향 지도: 벽과 물체는 arena_v4와 칸 하나까지 같고, 레인 지도와 자유바닥 지도만
양방향 통로가 있는 것(arena_v4_twoway)이다. 시나리오가 월드·지도·레인 지도를 찾는다."""
from pathlib import Path

import pytest
import yaml

lane_map_module = pytest.importorskip("fleet_core.traffic.lane_map")
from fleet_gz_sim import arena  # noqa: E402
from fleet_gz_sim.fleet_setup import arena_for, load, start_poses  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
FAKE = PKG.parent / "fake_robots"
V4 = arena.load_site(PKG / "sites" / "arena_v4.yaml")
SITE = arena.load_site(PKG / "sites" / "arena_v4_twoway.yaml")
SCENARIO = PKG / "scenarios" / "gz_arena_v4_twoway.yaml"


def test_the_walls_and_objects_are_those_of_arena_v4():
    assert SITE.lane_map == "arena_v4_twoway"
    assert SITE.inside == V4.inside
    assert SITE.lane_clearance_m == V4.lane_clearance_m
    assert SITE.features == V4.features


def test_the_generated_maps_are_the_arena_v4_maps():
    """지도(PGM)는 벽과 물체로만 만든다: arena_v4의 것과 바이트까지 같다."""
    for mine, theirs in zip(SITE.outputs()[1:], V4.outputs()[1:]):
        if mine.endswith(".pgm"):
            assert (PKG / mine).read_bytes() == (PKG / theirs).read_bytes()
    _, _, coarse, _, fine = SITE.outputs()
    assert (PKG / coarse).read_bytes() == arena.pgm(arena.COARSE_M, SITE)
    assert (PKG / fine).read_bytes() == arena.pgm(arena.FINE_M, SITE)


def test_the_world_is_generated_from_the_twoway_lane_map():
    lanes = lane_map_module.load_lane_map(FAKE / "maps" / "arena_v4_twoway.yaml")
    world = PKG / SITE.outputs()[0]
    assert world.read_text(encoding="utf-8") == arena.world_sdf(lanes, site=SITE)


def test_the_scenario_gets_its_world_maps_and_lane_map():
    found = arena_for(SCENARIO, PKG, FAKE)
    world, nav, _, fine, _ = SITE.outputs()
    assert found.name == SITE.name == "arena_v4_twoway"
    assert (found.world, found.nav_map, found.nav_map_fine) == tuple(
        str(PKG / rel) for rel in (world, nav, fine))
    assert found.lane_map == str(FAKE / "maps" / "arena_v4_twoway.yaml")
    for path in (found.world, found.nav_map, found.nav_map_fine, found.lane_map):
        assert Path(path).is_file(), path


def test_the_lane_map_points_the_controller_at_the_twoway_floor():
    data = yaml.safe_load((FAKE / "maps" / "arena_v4_twoway.yaml").read_text(encoding="utf-8"))
    assert Path(data["floor_map"]).name == "floor_map_arena_v4_twoway.yaml"


def test_the_scenario_is_the_arena_v4_rehearsal_on_the_twoway_map():
    """주문, 로봇, 팔, 도킹은 gz_arena_v4와 같다. 다른 것은 지도뿐이다(비교할 수 있게)."""
    mine = yaml.safe_load(SCENARIO.read_text(encoding="utf-8"))
    theirs = yaml.safe_load((PKG / "scenarios" / "gz_arena_v4.yaml").read_text(encoding="utf-8"))
    assert mine.pop("map") == "arena_v4_twoway"
    assert theirs.pop("map") == "arena_v4"
    assert mine == theirs
    site, scenario = load(SCENARIO, arena_for(SCENARIO, PKG, FAKE).lane_map)
    poses = start_poses(site, scenario)
    assert sorted(cfg.start for cfg in scenario.robots.values()) == sorted(site.parking)
    for robot, cfg in scenario.robots.items():
        assert poses[robot][:2] == pytest.approx(site.waypoint_pose(cfg.start))
