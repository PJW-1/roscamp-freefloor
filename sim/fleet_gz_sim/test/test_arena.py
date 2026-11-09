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


# -- 지도마다 경기장(site) -------------------------------------------------------------------

SITES = sorted((PKG / "sites").glob("*.yaml"))
MAPS = PKG.parent / "fake_robots" / "maps"


def site_file(tmp_path, text: str):
    path = tmp_path / "toy.yaml"
    path.write_text(text, encoding="utf-8")
    return path


TOY = """
lane_map: ""
bounds_m: [0.0, 0.25, 3.70, 2.40]
boxes:
  - {name: rack_n, kind: rack, x0: 2.60, y0: 2.10, x1: 3.20, y1: 2.40, height_m: 0.45}
  - {name: post_sw, kind: pillar, x0: 0.00, y0: 0.25, x1: 0.10, y1: 0.35, height_m: 0.5}
"""


def test_the_built_in_lane_arena_keeps_its_map_frame():
    """기본 경기장의 지도 원점과 크기는 벽 안쪽에서 0.15 m씩 넓힌 것이다(저장소 파일 그대로)."""
    assert arena.LANE_ARENA.map_origin == arena.MAP_ORIGIN == (-0.15, 0.10)
    assert arena.LANE_ARENA.map_size == arena.MAP_SIZE == (4.00, 2.45)
    assert arena.LANE_ARENA.outputs() == arena.OUTPUTS


def test_a_site_file_gives_bounds_boxes_and_its_lane_map(tmp_path):
    site = arena.load_site(site_file(tmp_path, TOY))
    assert site.name == "toy" and site.inside == (0.0, 0.25, 3.70, 2.40) and site.lane_map == ""
    rack, post = site.features
    assert (rack.name, rack.kind, rack.height_m) == ("rack_n", "rack", 0.45)
    assert (post.x0, post.y1, post.kind) == (0.00, 0.35, "pillar")
    assert [b.name for b in arena.obstacles(site)][:4] == [
        "wall_south", "wall_north", "wall_west", "wall_east"]


@pytest.mark.parametrize("change, message", [
    (("bounds_m:", "extra: 1\nbounds_m:"), "unknown keys"),
    (("kind: rack", "kind: shelf"), "kind"),
    (("x1: 3.20", "x1: 3.90"), "outside"),
    (("height_m: 0.45", "height_m: 0.10"), "lidar"),
    (("name: post_sw", "name: rack_n"), "twice"),
    (("x0: 0.00, y0: 0.25, x1: 0.10", "x0: 0.10, y0: 0.25, x1: 0.00"), "empty"),
])
def test_a_bad_site_file_is_refused(tmp_path, change, message):
    with pytest.raises(arena.SiteError, match=message):
        arena.load_site(site_file(tmp_path, TOY.replace(*change)))


def test_a_site_world_and_maps_are_named_after_it(tmp_path):
    site = arena.load_site(site_file(tmp_path, TOY))
    out = tmp_path / "out"
    arena.generate(out, LANES, site=site)
    assert sorted(site.outputs()) == sorted(
        str(p.relative_to(out)).replace("\\", "/") for p in out.rglob("*") if p.is_file())
    world = ET.parse(out / "worlds" / "toy.sdf").getroot()
    assert world.find("world").get("name") == arena.WORLD  # 월드 이름은 모든 경기장이 같다
    poses = {c.get("name"): c.find("pose").text.split()
             for c in world.iter("collision") if c.find("pose") is not None}
    assert float(poses["rack_n"][2]) == pytest.approx(0.45 / 2)
    data = yaml.safe_load((out / "maps" / "toy.yaml").read_text(encoding="utf-8"))
    assert data["origin"] == [-0.15, 0.10, 0.0] and data["image"] == "toy.pgm"
    grid = arena.occupancy(arena.COARSE_M, site)
    ox, oy = site.map_origin
    assert grid[int((2.25 - oy) // 0.05)][int((2.9 - ox) // 0.05)]  # 랙 안은 점유


def test_a_site_box_too_close_to_a_lane_is_refused(tmp_path):
    near = TOY.replace("x0: 2.60, y0: 2.10", "x0: 1.60, y0: 1.90")  # P-6 -> N 레인 바로 위
    with pytest.raises(ValueError, match="needs"):
        arena.generate(tmp_path / "out", LANES, site=arena.load_site(site_file(tmp_path, near)))


@pytest.mark.parametrize("path", SITES, ids=[p.stem for p in SITES])
def test_committed_site_worlds_and_maps_match_the_generator(tmp_path, path):
    site = arena.load_site(path)
    lanes = lane_map_module.load_lane_map(MAPS / f"{site.lane_map}.yaml" if site.lane_map else None)
    arena.generate(tmp_path, lanes, site=site)
    for rel in site.outputs():
        assert (PKG / rel).read_bytes() == (tmp_path / rel).read_bytes(), (
            f"{rel} differs: run python3 -m fleet_gz_sim.arena --site {site.name}")


def test_an_arm_is_drawn_from_parts_but_collides_on_its_base_only(tmp_path):
    """팔은 움직이지 않는 모형이다. 받침만 충돌하고 그 위 부품은 라이다 평면보다 높다."""
    text = TOY + "  - {name: arm1, kind: arm, x0: 0.00, y0: 1.40, x1: 0.20, y1: 1.60}\n"
    site = arena.load_site(site_file(tmp_path, text))
    arm = site.features[-1]
    assert arm.height_m == arena.DEFAULT_HEIGHT_M["arm"] > arena.LIDAR_Z_M
    link = ET.fromstring(arena.world_sdf(LANES, site=site)).find(".//model[@name='arena']/link")
    assert [c.get("name") for c in link.iter("collision") if c.get("name").startswith("arm1")] == [
        "arm1"]
    parts = [v for v in link.iter("visual") if v.get("name").startswith("arm1")]
    assert len(parts) >= 5 and any(v.find(".//cylinder") is not None for v in parts)
    for v in parts[1:]:  # 받침 위 부품: 가장 낮은 점도 라이다 평면 위
        assert _lowest_z(v) >= arena.LIDAR_Z_M + arena.LIDAR_MARGIN_M, v.get("name")
        assert _lowest_z(v) >= arm.height_m - 1e-9, v.get("name")


def _lowest_z(visual) -> float:
    """Lowest point of a visual part: its centre minus the vertical half extent."""
    pose = [float(v) for v in visual.find("pose").text.split()]
    z, pitch = pose[2], pose[4]
    geometry = visual.find("geometry")
    if geometry.find("cylinder") is not None:
        radius = float(geometry.find(".//radius").text)
        length = float(geometry.find(".//length").text)
        return z - abs(math.cos(pitch)) * length / 2 - abs(math.sin(pitch)) * radius
    if geometry.find("sphere") is not None:
        return z - float(geometry.find(".//radius").text)
    sx, _, sz = (float(v) for v in geometry.find(".//size").text.split())
    return z - abs(math.cos(pitch)) * sz / 2 - abs(math.sin(pitch)) * sx / 2


@pytest.mark.parametrize("resolution", [arena.COARSE_M, arena.FINE_M])
def test_a_thin_off_grid_box_still_fills_the_cells_it_touches(tmp_path, resolution):
    """A box narrower than a cell, between cell centres, must not vanish from the map."""
    text = TOY + "  - {name: slat, kind: rack, x0: 2.63, y0: 1.003, x1: 2.645, y1: 1.008}\n"
    site = arena.load_site(site_file(tmp_path, text))
    grid = arena.occupancy(resolution, site)
    ox, oy = site.map_origin
    for x in (2.631, 2.644):
        assert grid[int((1.005 - oy) // resolution)][int((x - ox) // resolution)], x


@pytest.mark.parametrize("change", [
    ("x0: 2.60", "x0: .nan"),
    ("y1: 2.40, height_m", "y1: .inf, height_m"),
    ("height_m: 0.45", "height_m: .inf"),
    ("bounds_m: [0.0", "bounds_m: [.nan"),
])
def test_a_site_file_with_a_non_finite_number_is_refused(tmp_path, change):
    with pytest.raises(arena.SiteError, match="finite"):
        arena.load_site(site_file(tmp_path, TOY.replace(*change)))


def test_a_site_file_with_a_duplicate_key_is_refused(tmp_path):
    text = TOY.replace("kind: rack,", "kind: rack, kind: pillar,")
    with pytest.raises(arena.SiteError, match="duplicate"):
        arena.load_site(site_file(tmp_path, text))


def test_every_world_steps_physics_at_4_ms():
    """Gazebo publishes /clock once per physics step, and every node on use_sim_time wakes up
    for it. At 1 ms the Python nodes of the three-robot warehouse run took 50-90 % of a core
    each and the sim ran at 0.3-0.5 of real time (2026-11-08), so the worlds step at 4 ms
    (/clock at 250 Hz in real time)."""
    texts = {"generator": arena.world_sdf(LANES)}
    texts.update({p.name: p.read_text(encoding="utf-8")
                  for p in sorted((PKG / "worlds").glob("*.sdf"))})
    assert len(texts) >= 3
    for name, text in texts.items():
        physics = ET.fromstring(text).find("world/physics")
        assert float(physics.findtext("max_step_size")) == pytest.approx(0.004), name
        assert float(physics.findtext("real_time_factor")) == pytest.approx(1.0), name
