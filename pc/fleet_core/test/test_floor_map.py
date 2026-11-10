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


# ---- warehouse floor and choosing the floor map by lane map ------------------------------

WAREHOUSE_LANES = REPO / "sim" / "fake_robots" / "maps" / "warehouse.yaml"
WAREHOUSE_FLOOR = DEFAULT_FLOOR.parent / "floor_map_warehouse.yaml"


def test_the_warehouse_image_is_the_gazebo_map_byte_for_byte():
    shipped = DEFAULT_FLOOR.parent / "warehouse.pgm"
    gazebo = REPO / "sim" / "fleet_gz_sim" / "maps" / "warehouse.pgm"
    assert shipped.read_bytes() == gazebo.read_bytes()


def test_the_warehouse_floor_has_the_gazebo_map_origin_and_size():
    import yaml
    gazebo = yaml.safe_load((REPO / "sim" / "fleet_gz_sim" / "maps" / "warehouse.yaml")
                            .read_text(encoding="utf-8"))
    floor = load_floor_map(WAREHOUSE_FLOOR)
    assert floor.bounds()[:2] == tuple(gazebo["origin"][:2])
    assert floor.px_m == gazebo["resolution"]
    assert (floor.cols, floor.rows) == (63, 43)  # 126 x 86 px of 0.05 m


def test_the_warehouse_floor_matches_the_warehouse_lane_map():
    floor = load_floor_map(WAREHOUSE_FLOOR)
    lanes = load_lane_map(WAREHOUSE_LANES)
    assert set(floor.docks) == set(lanes.stations)
    for name, station in lanes.stations.items():
        assert floor.docks[name] == lanes.waypoint_pose(station.dock)
    assert [spot for spot, _ in floor.parking] == list(lanes.parking)
    for spot, pose in floor.parking:
        assert pose == lanes.waypoint_pose(spot)
    ends = {(lane.ax, lane.ay, lane.bx, lane.by) for lane in lanes.lanes.values()}
    for line in floor.fixed_lines.values():
        for a, b in zip(line.points, line.points[1:], strict=False):
            assert (*a, *b) in ends, f"{line.name} {a}->{b} is not a lane in its direction"
    for name, station in lanes.stations.items():
        dock = lanes.waypoint_pose(station.dock)
        [(queue_from, queue_to)] = station.queue
        line_in = floor.fixed_lines[f"{name.lower()}_in"]
        line_out = floor.fixed_lines[f"{name.lower()}_out"]
        assert line_in.points == (lanes.waypoint_pose(queue_from), lanes.waypoint_pose(queue_to))
        assert line_in.end == dock and line_out.start == dock
    east, west = floor.fixed_lines["tunnel_east"], floor.fixed_lines["tunnel_west"]
    assert east.points == tuple(reversed(west.points))
    assert {east.start, east.end} == {lanes.waypoint_pose("A2"), lanes.waypoint_pose("C2")}


def test_each_lane_map_names_its_floor_map():
    from fleet_core.traffic.floor.floor_map import floor_map_for
    feature = load_lane_map(REPO / "sim" / "fake_robots" / "maps" / "feature_map.yaml")
    for site, floor_file in [(load_lane_map(), DEFAULT_FLOOR),
                             (feature, FEATURE_FLOOR),
                             (load_lane_map(WAREHOUSE_LANES), WAREHOUSE_FLOOR)]:
        assert floor_map_for(site).source_text == floor_file.read_text(encoding="utf-8")


def test_a_lane_map_without_a_fitting_floor_map_is_refused(tmp_path):
    """The warehouse lane map without its floor_map key would get the lane_arena floor."""
    import yaml
    from fleet_core.traffic.floor.floor_map import floor_map_for
    data = yaml.safe_load(WAREHOUSE_LANES.read_text(encoding="utf-8"))
    data.pop("floor_map")
    path = tmp_path / "lanes.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    with pytest.raises(FloorMapError, match="does not fit the lane map"):
        floor_map_for(load_lane_map(path))


def test_a_floor_map_name_is_a_file_in_the_fleet_core_data(tmp_path):
    import yaml
    from fleet_core.traffic.floor.floor_map import floor_map_for
    data = yaml.safe_load(WAREHOUSE_LANES.read_text(encoding="utf-8"))
    for name in ("../data/floor_map_warehouse.yaml", "nowhere.yaml"):
        data["floor_map"] = name
        path = tmp_path / "lanes.yaml"
        path.write_text(yaml.safe_dump(data), encoding="utf-8")
        with pytest.raises(FloorMapError, match="floor_map"):
            floor_map_for(load_lane_map(path))


def copy_data(tmp_path, monkeypatch):
    """A copy of fleet_core/data that floor_map_for reads instead."""
    import shutil

    from fleet_core.traffic.floor import floor_map as module
    data = tmp_path / "data"
    shutil.copytree(DEFAULT_FLOOR.parent, data)
    monkeypatch.setattr(module, "DEFAULT_FLOOR", data / DEFAULT_FLOOR.name)
    return data


def test_a_floor_with_a_dock_the_lane_map_does_not_have_is_refused(tmp_path, monkeypatch):
    """A floor dock is live: a robot starting near it would take its out line."""
    import yaml
    from fleet_core.traffic.floor.floor_map import floor_map_for
    data = copy_data(tmp_path, monkeypatch)
    floor = yaml.safe_load(WAREHOUSE_FLOOR.read_text(encoding="utf-8"))
    floor["docks"]["SPARE"] = [3.0, 0.35]
    (data / WAREHOUSE_FLOOR.name).write_text(yaml.safe_dump(floor), encoding="utf-8")
    with pytest.raises(FloorMapError, match="SPARE"):
        floor_map_for(load_lane_map(WAREHOUSE_LANES))


def test_a_floor_map_name_that_links_out_of_the_data_folder_is_refused(tmp_path, monkeypatch):
    from fleet_core.traffic.floor.floor_map import floor_map_for
    data = copy_data(tmp_path, monkeypatch)
    outside = tmp_path / "outside"
    outside.mkdir()
    (outside / WAREHOUSE_FLOOR.name).write_text(
        (data / WAREHOUSE_FLOOR.name).read_text(encoding="utf-8").replace(
            "image: warehouse.pgm", f"image: {data / 'warehouse.pgm'}"), encoding="utf-8")
    (data / WAREHOUSE_FLOOR.name).unlink()
    (data / WAREHOUSE_FLOOR.name).symlink_to(outside / WAREHOUSE_FLOOR.name)
    with pytest.raises(FloorMapError, match="floor_map"):
        floor_map_for(load_lane_map(WAREHOUSE_LANES))


def test_an_install_of_links_into_one_data_folder_is_accepted(tmp_path, monkeypatch):
    """A symlink install links every data file to the source folder: that is one folder."""
    from fleet_core.traffic.floor import floor_map as module
    from fleet_core.traffic.floor.floor_map import floor_map_for
    source = copy_data(tmp_path, monkeypatch)
    links = tmp_path / "install"
    links.mkdir()
    for path in source.iterdir():
        (links / path.name).symlink_to(path)
    monkeypatch.setattr(module, "DEFAULT_FLOOR", links / DEFAULT_FLOOR.name)
    floor = floor_map_for(load_lane_map(WAREHOUSE_LANES))
    assert floor.source_text == WAREHOUSE_FLOOR.read_text(encoding="utf-8")


# ---- wall_margin_m: the clearance of clear cells, apart from the robot-to-robot margin ----


def corridor_data(**changes):
    """A 0.25 m wide east-west corridor (y 0.20 to 0.45) between two boxes."""
    data = {"bounds": [0, 0, 1.5, 0.65], "boxes": [[0, 0, 1.5, 0.20], [0, 0.45, 1.5, 0.65]],
            "cell_m": 0.05, "robot_radius_m": 0.08, "cell_margin_m": 0.05}
    data.update(changes)
    return data


def test_wall_margin_defaults_to_the_cell_margin(floor):
    assert floor.wall_margin_m == floor.cell_margin_m == 0.05
    explicit = FloorMap(corridor_data(wall_margin_m=0.05))
    default = FloorMap(corridor_data())
    assert default.wall_margin_m == 0.05
    assert explicit.clear == default.clear and explicit.free == default.free


def test_default_clear_set_is_the_old_one_for_the_shipped_floor(floor):
    body = floor.robot_radius_m + floor.cell_margin_m
    assert floor.clear == frozenset(c for c in floor.free if floor._box_gap_to_wall(c) >= body)


def test_a_narrow_gap_is_clear_only_with_a_small_wall_margin():
    from fleet_core.traffic.floor.planner import FloorPlanner
    wide, tight = FloorMap(corridor_data()), FloorMap(corridor_data(wall_margin_m=0.01))
    assert not any(wide.cell_of(x, 0.325) in wide.clear for x in (0.3, 0.7, 1.1))
    row = {tight.cell_of(x / 20 + 0.025, 0.325) for x in range(4, 26)}
    assert row <= tight.clear and tight.clear > wide.clear
    path = FloorPlanner(tight).plan((0.3, 0.325), (1.2, 0.325))
    assert path is not None and path[0] == (0.3, 0.325) and path[-1] == (1.2, 0.325)
    assert FloorPlanner(wide).plan((0.3, 0.325), (1.2, 0.325)) is None
    for cell in tight.clear:
        assert tight._box_gap_to_wall(cell) >= 0.08 + 0.01
    assert tight.cell_margin_m == 0.05  # the robot-to-robot margin is untouched


@pytest.mark.parametrize("bad", [-0.01, float("nan"), float("inf"), "wide", None, True,
                                 "0.01", pytest.param(10 ** 400, id="huge")])
def test_a_bad_wall_margin_is_refused(bad):
    with pytest.raises(FloorMapError, match="wall_margin_m"):
        FloorMap(corridor_data(wall_margin_m=bad))


def test_the_lock_body_ignores_the_wall_margin():
    from fleet_core.traffic.floor.controller import FloorController
    site = load_lane_map()
    for wall in (None, 0.01, 0.0):
        extra = {} if wall is None else {"wall_margin_m": wall}
        floor = FloorMap(corridor_data(**extra))
        ctl = FloorController(site, load_params(), ["r1"], lambda: 0.0, floor=floor)
        assert ctl._body == pytest.approx(0.13)
        assert ctl.planner.floor.cell_margin_m == 0.05


def test_fixed_lines_are_locked_whole_unless_the_map_says_rolling(floor):
    assert floor.rolling_lines == frozenset()
    data = {"bounds": [0, 0, 2, 1], "cell_m": 0.1, "robot_radius_m": 0.08,
            "cell_margin_m": 0.05, "fixed_lines": {"one_way": [[0.3, 0.5], [1.7, 0.5]]},
            "rolling_lines": ["one_way"]}
    assert FloorMap(data).rolling_lines == frozenset({"one_way"})


@pytest.mark.parametrize("bad", [["nowhere"], "one_way", [1]])
def test_a_rolling_line_that_is_not_a_fixed_line_is_refused(bad):
    data = {"bounds": [0, 0, 2, 1], "cell_m": 0.1, "robot_radius_m": 0.08,
            "cell_margin_m": 0.05, "fixed_lines": {"one_way": [[0.3, 0.5], [1.7, 0.5]]},
            "rolling_lines": bad}
    with pytest.raises(FloorMapError, match="rolling_lines"):
        FloorMap(data)


def test_the_half_width_is_read_only_when_the_map_gives_it(floor):
    assert floor.robot_half_width_m is None
    data = {"bounds": [0, 0, 2, 1], "cell_m": 0.1, "robot_radius_m": 0.08,
            "cell_margin_m": 0.05, "robot_half_width_m": 0.06}
    assert FloorMap(data).robot_half_width_m == 0.06


def test_the_side_margin_is_the_cell_margin_unless_the_map_gives_it(floor):
    assert floor.side_margin_m == floor.cell_margin_m
    assert FloorMap(floor_data(side_margin_m=0.01)).side_margin_m == 0.01
    assert FloorMap(floor_data(side_margin_m=0)).side_margin_m == 0.0


@pytest.mark.parametrize("bad", [-0.01, 0.06, float("nan"), "0.01", True, None])
def test_a_side_margin_outside_zero_to_the_cell_margin_is_refused(bad):
    with pytest.raises(FloorMapError, match="side_margin_m"):
        FloorMap(floor_data(side_margin_m=bad))


@pytest.mark.parametrize("bad", [0.0, -0.01, 0.09, float("nan"), "0.06", True])
def test_a_half_width_that_is_not_inside_the_radius_is_refused(bad):
    data = {"bounds": [0, 0, 2, 1], "cell_m": 0.1, "robot_radius_m": 0.08,
            "cell_margin_m": 0.05, "robot_half_width_m": bad}
    with pytest.raises(FloorMapError, match="robot_half_width_m"):
        FloorMap(data)


def gap_data(**extra):
    """Two boxes 0.20 m apart (x 0.65 to 0.85). Clear cells keep 0.09 m from a wall, so no 3 cm
    cell between the boxes is clear; the centre line x = 0.75 keeps 0.10 m."""
    data = {"bounds": [0, 0, 1.5, 0.9], "resolution": 0.01, "cell_m": 0.03,
            "boxes": [[0.45, 0.30, 0.65, 0.60], [0.85, 0.30, 1.05, 0.60]],
            "robot_radius_m": 0.08, "cell_margin_m": 0.03, "wall_margin_m": 0.01,
            "docks": {"WORK": [0.75, 0.45]},
            "fixed_lines": {"work_in": [[0.75, 0.75], [0.75, 0.45]],
                            "work_out": [[0.75, 0.45], [0.75, 0.15]]},
            "rolling_lines": ["work_in", "work_out"],
            "narrow_lines": ["work_in", "work_out"]}
    data.update(extra)
    return data


def test_a_line_through_a_gap_with_no_clear_cell_is_refused_unless_the_map_says_narrow():
    with pytest.raises(FloorMapError, match="leaves clear floor"):
        FloorMap(gap_data(narrow_lines=[], docks={}))
    with pytest.raises(FloorMapError, match="WORK"):
        FloorMap(gap_data(narrow_lines=[], fixed_lines={}, rolling_lines=[]))
    floor = FloorMap(gap_data())
    assert floor.narrow_lines == frozenset({"work_in", "work_out"})


def test_a_narrow_line_keeps_the_body_and_wall_margin_off_the_walls_by_itself():
    """The robot drives on the line, so the line keeps radius + wall margin from the walls; the
    cells it runs over need not, and they stay out of the clear floor."""
    floor = FloorMap(gap_data())
    between = {floor.cell_of(0.75, y / 100) for y in range(31, 60)}
    assert between <= floor.line_floor and not between & floor.clear
    assert floor.line_floor <= floor.free
    for name in floor.narrow_lines:
        (ax, ay), (bx, by) = floor.fixed_lines[name].points
        for k in range(101):
            point = (ax + (bx - ax) * k / 100, ay + (by - ay) * k / 100)
            assert floor._gap_to_wall(*point) >= 0.08 + 0.01 - 1e-9


@pytest.mark.parametrize("x", [0.73, 0.77])
def test_a_narrow_line_nearer_a_wall_than_the_body_and_margin_is_refused(x):
    lines = {"work_in": [[x, 0.75], [x, 0.45]], "work_out": [[x, 0.45], [x, 0.15]]}
    with pytest.raises(FloorMapError, match="work_in"):
        FloorMap(gap_data(fixed_lines=lines, docks={}))
    with pytest.raises(FloorMapError, match="WORK"):  # the dock on it is refused first
        FloorMap(gap_data(fixed_lines=lines, docks={"WORK": [x, 0.45]}))


def test_a_dock_between_the_boxes_has_to_stand_on_the_narrow_line():
    with pytest.raises(FloorMapError, match="WORK"):
        FloorMap(gap_data(docks={"WORK": [0.73, 0.45]}))  # same cell column, 0.08 m from a box
    with pytest.raises(FloorMapError, match="P-1"):
        FloorMap(gap_data(parking=[{"id": "P-1", "x": 0.75, "y": 0.45}]))  # parking: clear only


@pytest.mark.parametrize("bad", [["nowhere"], "work_in", [3]])
def test_a_narrow_line_that_is_not_a_fixed_line_is_refused(bad):
    with pytest.raises(FloorMapError, match="narrow_lines"):
        FloorMap(gap_data(narrow_lines=bad))


def test_the_default_floor_has_no_narrow_line(floor):
    assert floor.narrow_lines == frozenset() and floor.line_floor == frozenset()


def test_the_cells_under_a_narrow_line_that_are_clear_anyway_are_not_line_floor():
    floor = FloorMap(gap_data())
    assert not floor.line_floor & floor.clear
    assert floor.cell_of(0.75, 0.75) in floor.clear  # the open end of the way in


def test_a_narrow_line_keeps_its_distance_between_the_points_that_are_checked_too():
    """Checked every quarter pixel, each point half that spacing beyond the limit, so no point
    of the line is nearer: a line at exactly radius + wall margin is refused."""
    lines = {"work_in": [[0.74, 0.75], [0.74, 0.45]], "work_out": [[0.74, 0.45], [0.74, 0.15]]}
    with pytest.raises(FloorMapError, match="work_in"):
        FloorMap(gap_data(fixed_lines=lines, docks={}))


def test_docks_are_checked_before_lines_as_before():
    data = {"bounds": [0, 0, 2, 1], "boxes": [[0.9, 0.0, 1.1, 1.0]], "cell_m": 0.1,
            "robot_radius_m": 0.08, "cell_margin_m": 0.05, "docks": {"WORK": [0.05, 0.5]},
            "fixed_lines": {"across": [[0.3, 0.5], [1.7, 0.5]]}}
    with pytest.raises(FloorMapError, match="WORK"):
        FloorMap(data)


def test_a_narrow_line_has_to_be_a_rolling_line():
    """Whole-line locks and the one-robot rule of the other lines are not worked out for a robot
    that leaves a gap along lines it was not sent by."""
    with pytest.raises(FloorMapError, match="narrow_lines"):
        FloorMap(gap_data(rolling_lines=["work_in"]))


def test_each_narrow_line_has_its_own_floor():
    floor = FloorMap(gap_data())
    assert set(floor.line_floor_of) == {"work_in", "work_out"}
    assert floor.line_floor == floor.line_floor_of["work_in"] | floor.line_floor_of["work_out"]
    assert floor.cell_of(0.75, 0.55) in floor.line_floor_of["work_in"]
    assert floor.cell_of(0.75, 0.55) not in floor.line_floor_of["work_out"]


# ---- shortcuts: another way in to the same dock, taken on the way when it is free in time ----


def shortcut_data(**extra):
    """Open floor 2 x 1 m. The way in goes up the east side, west along the north side and down
    to the dock; the shortcut leaves it on the east side and joins it again above the dock."""
    data = {"bounds": [0, 0, 2, 1], "cell_m": 0.1, "robot_radius_m": 0.08, "cell_margin_m": 0.05,
            "docks": {"OUTBOUND": [0.35, 0.25]},
            "fixed_lines": {
                "outbound_in": [[1.65, 0.25], [1.65, 0.75], [0.35, 0.75], [0.35, 0.55],
                                [0.35, 0.25]],
                "outbound_short": [[1.65, 0.25], [1.65, 0.45], [0.75, 0.45], [0.35, 0.55],
                                   [0.35, 0.25]]},
            "rolling_lines": ["outbound_in", "outbound_short"],
            "shortcuts": {"outbound_short": "outbound_in"}}
    data.update(extra)
    return data


def test_a_floor_without_shortcuts_has_none(floor):
    assert floor.shortcuts == {}


def test_a_shortcut_leaves_its_line_and_joins_it_again():
    floor = FloorMap(shortcut_data())
    assert floor.shortcuts == {"outbound_short": "outbound_in"}
    fork, merge = floor.shortcut_parts["outbound_short"]
    assert fork == pytest.approx(0.20)  # on the line up to (1.65, 0.45), between two of its points
    assert merge == pytest.approx(0.20 + 0.90 + math.hypot(0.40, 0.10))  # (0.35, 0.55) on


@pytest.mark.parametrize("bad", [
    "outbound_in", ["outbound_short"], [], 0, {"outbound_short": "nowhere"},
    {"nowhere": "outbound_in"}, {"outbound_short": "outbound_short"},
    {"outbound_short": "outbound_in", "outbound_in": "outbound_short"}])
def test_a_shortcut_that_does_not_name_two_fixed_lines_is_refused(bad):
    with pytest.raises(FloorMapError, match="shortcuts"):
        FloorMap(shortcut_data(shortcuts=bad))


@pytest.mark.parametrize("points", [
    [[1.65, 0.35], [1.65, 0.45], [0.75, 0.45], [0.35, 0.55], [0.35, 0.25]],  # starts elsewhere
    [[1.65, 0.25], [1.65, 0.45], [0.75, 0.45], [0.35, 0.55], [0.35, 0.35]],  # ends elsewhere
    [[1.65, 0.25], [1.65, 0.75], [0.35, 0.75], [0.35, 0.25]],  # never leaves the line
], ids=["start", "end", "same"])
def test_a_shortcut_has_to_start_and_end_where_its_line_does_and_leave_it_between(points):
    lines = dict(shortcut_data()["fixed_lines"], outbound_short=points)
    with pytest.raises(FloorMapError, match="outbound_short"):
        FloorMap(shortcut_data(fixed_lines=lines))


def test_a_shortcut_and_its_line_have_to_be_rolling_lines():
    """The robot leaves its line for the shortcut on the way, without stopping: there is no turn
    to wait for on a rolling line (a line locked whole has one)."""
    with pytest.raises(FloorMapError, match="shortcuts"):
        FloorMap(shortcut_data(rolling_lines=["outbound_in"]))
    with pytest.raises(FloorMapError, match="shortcuts"):
        FloorMap(shortcut_data(rolling_lines=["outbound_short"]))


def test_a_line_has_one_shortcut_at_most():
    data = shortcut_data()
    data["fixed_lines"]["outbound_other"] = data["fixed_lines"]["outbound_short"]
    data["rolling_lines"].append("outbound_other")
    with pytest.raises(FloorMapError, match="shortcuts"):
        FloorMap(dict(data, shortcuts={"outbound_short": "outbound_in",
                                       "outbound_other": "outbound_in"}))


def test_a_shortcut_that_meets_another_line_again_away_from_its_dock_is_refused():
    """The traffic control holds the stretch a shortcut shares with the other lines from its fork
    at once. Meeting another line again further on (not only where it touches the robot at the
    dock, as the shortcut's own line does) would be a place it stands in the way, not held."""
    from fleet_core.traffic.floor.controller import FloorController
    site = load_lane_map()
    FloorController(site, load_params(), ["r1"], lambda: 0.0, floor=FloorMap(shortcut_data()))
    data = shortcut_data()
    data["fixed_lines"]["across"] = [[1.15, 0.75], [1.15, 0.25]]  # crosses it at x 1.15
    with pytest.raises(ValueError, match="outbound_short"):
        FloorController(site, load_params(), ["r1"], lambda: 0.0, floor=FloorMap(data))
