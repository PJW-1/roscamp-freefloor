"""관제 화면: 레인, 로봇, 쥔 구역, 막힌 레인을 마커 명세로 바꾸는지와 RViz 설정."""
from dataclasses import dataclass, field
from pathlib import Path

import pytest
import yaml

lane_map_module = pytest.importorskip("fleet_core.traffic.lane_map")
from fleet_gz_sim import fleet_view as fv  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
LANES = lane_map_module.load_lane_map(None)


@dataclass
class Status:
    robot_id: str
    fsm_state: str = "IDLE"
    task_type: str = ""
    order_id: str = ""
    fault: str = "NONE"
    fault_reason: str = ""
    x: float = 1.0
    y: float = 1.0
    yaw: float = 0.0


@dataclass
class Zone:
    zone_id: str
    holder: str = ""
    queue: list = field(default_factory=list)


def by_ns(specs, ns):
    return [s for s in specs if s.ns == ns]


def xy(points):
    return {(round(p[0], 3), round(p[1], 3)) for p in points}


def segments(points):
    pairs = zip(points[0::2], points[1::2], strict=True)
    return {(xy([a]).pop(), xy([b]).pop()) for a, b in pairs}


def pose_of(name):
    x, y = LANES.waypoint_pose(name)
    return round(x, 3), round(y, 3)


# ---- 레인과 지점 --------------------------------------------------------------------------


def test_lanes_are_thin_gray_lines_one_per_lane_pair():
    specs = fv.lane_specs(LANES)
    [lines] = by_ns(specs, "lanes")
    assert lines.kind == "LINE_LIST"
    assert lines.color[:3] == fv.GRAY[:3]
    assert lines.scale[0] <= 0.02
    # 양방향 쌍(AVENUE)은 선 하나로 그린다
    pairs = {frozenset(key) for key in LANES.lanes}
    assert len(lines.points) == 2 * len(pairs)
    assert (pose_of("S"), pose_of("L")) in segments(lines.points)


def test_waypoints_get_dots_and_labels():
    specs = fv.lane_specs(LANES)
    [dots] = by_ns(specs, "waypoints")
    assert dots.kind == "SPHERE_LIST"
    assert len(dots.points) == len(LANES.zone_of)
    labels = {s.text: s.pose for s in by_ns(specs, "waypoint_labels")}
    assert {"W", "A", "P-3", "QO"} <= set(labels)
    x, y = LANES.waypoint_pose("P-3")
    assert abs(labels["P-3"][0] - x) < 0.1 and abs(labels["P-3"][1] - y) < 0.1
    ids = [(s.ns, s.id) for s in specs]
    assert len(ids) == len(set(ids))


# ---- 로봇 ----------------------------------------------------------------------------------


def test_robot_label_reads_id_state_task_and_order():
    text, color = fv.robot_label(Status("pinky1", "MOVING", "TO_WORK", "R-003"), stale=False)
    assert text == "pinky1  MOVING  TO_WORK R-003"
    assert color == fv.ROBOT_COLORS["pinky1"]


def test_each_robot_has_its_own_fixed_color():
    colors = {fv.robot_color(r) for r in ("pinky1", "pinky2", "pinky3")}
    assert len(colors) == 3
    assert fv.RED not in colors and fv.GRAY not in colors


def test_blocked_robot_label_is_red():
    text, color = fv.robot_label(Status("pinky2", "BLOCKED", "TO_OUTBOUND", "R-7"), stale=False)
    assert "BLOCKED" in text
    assert color == fv.RED


def test_fault_shows_with_its_reason_in_red():
    status = Status("pinky3", "IDLE", fault="ERROR", fault_reason="BLOCKED")
    text, color = fv.robot_label(status, stale=False)
    assert text == "pinky3  IDLE  ERROR:BLOCKED"
    assert color == fv.RED


def test_robot_body_arrow_and_label_sit_at_the_pose():
    view = fv.FleetView(LANES)
    view.set_statuses([Status("pinky1", "MOVING", "TO_WORK", "R-003")])
    view.set_pose("pinky1", 1.2, 1.3, 1.57, now=10.0)
    specs = view.specs(now=10.5)
    [body] = by_ns(specs, "robot_body")
    [arrow] = by_ns(specs, "robot_heading")
    [label] = by_ns(specs, "robot_label")
    assert body.kind == "CYLINDER" and arrow.kind == "ARROW" and label.kind == "TEXT"
    assert body.pose[:2] == (1.2, 1.3) and arrow.pose[3] == 1.57
    assert body.scale[0] == pytest.approx(0.12)
    assert body.color == fv.ROBOT_COLORS["pinky1"]
    assert label.text == "pinky1  MOVING  TO_WORK R-003"
    assert label.pose[2] > body.pose[2]


def test_robot_without_a_pose_is_not_drawn():
    view = fv.FleetView(LANES)
    view.set_statuses([Status("pinky1")])
    assert not by_ns(view.specs(now=1.0), "robot_body")


def test_robot_without_a_status_still_shows_its_name():
    view = fv.FleetView(LANES)
    view.set_pose("pinky2", 1.0, 1.0, 0.0, now=0.0)
    [label] = by_ns(view.specs(now=0.1), "robot_label")
    assert label.text.startswith("pinky2")


def test_robot_with_no_pose_for_three_seconds_turns_gray():
    view = fv.FleetView(LANES)
    view.set_statuses([Status("pinky1", "MOVING")])
    view.set_pose("pinky1", 1.0, 1.0, 0.0, now=0.0)
    [fresh] = by_ns(view.specs(now=2.9), "robot_body")
    [old] = by_ns(view.specs(now=3.1), "robot_body")
    [label] = by_ns(view.specs(now=3.1), "robot_label")
    assert fresh.color == fv.ROBOT_COLORS["pinky1"]
    assert old.color == fv.GRAY
    assert label.color == fv.GRAY and "STALE" in label.text


def test_comm_lost_robot_is_gray_even_with_a_recent_pose():
    view = fv.FleetView(LANES)
    view.set_statuses([Status("pinky1", "MOVING", fault="COMM_LOST")])
    view.set_pose("pinky1", 1.0, 1.0, 0.0, now=0.0)
    [body] = by_ns(view.specs(now=0.1), "robot_body")
    assert body.color == fv.GRAY


# ---- 구역 ----------------------------------------------------------------------------------


def test_waypoint_zone_is_the_waypoint_and_the_lanes_ending_there():
    points, lines = fv.zone_geometry(LANES, "N")
    assert xy(points) == {pose_of("N")}
    assert lines == [(pose_of("P-6"), pose_of("N"))]


def test_two_way_bundle_covers_both_ends_and_every_lane_into_them():
    points, lines = fv.zone_geometry(LANES, "AVENUE")
    assert xy(points) == {pose_of("A"), pose_of("B")}
    assert set(lines) == {(pose_of(a), pose_of(b))
                          for a, b in (("A", "B"), ("B", "A"), ("W", "A"), ("O", "B"))}


def test_unit_zone_from_the_map_covers_all_members():
    data = yaml.safe_load(LANES.source_text)
    data["units"] = {"DOCK_W": ["W", "Q"]}
    lanes = lane_map_module.LaneMap(data)
    points, lines = fv.zone_geometry(lanes, "DOCK_W")
    assert xy(points) == {pose_of("W"), pose_of("Q")}
    assert (pose_of("L"), pose_of("Q")) in lines and (pose_of("Q"), pose_of("W")) in lines


def test_every_zone_of_the_default_map_has_geometry():
    for zone in LANES.zones:
        points, lines = fv.zone_geometry(LANES, zone)
        assert points and lines, zone


def test_crossing_pair_zones_are_drawn_each_as_its_own_zone():
    # 교차 쌍은 따로 된 id가 아니라 구역 id 두 개다. N에서 S로 가는 레인이 AVENUE를 가로지른다
    pairs = LANES.conflict_pairs(0.25, 0.25)
    assert frozenset({"S", "AVENUE"}) in pairs
    for pair in pairs:
        for zone in pair:
            assert fv.zone_geometry(LANES, zone) is not None


def test_unknown_zone_id_is_skipped():
    assert fv.zone_geometry(LANES, "NOPE") is None
    view = fv.FleetView(LANES)
    view.set_zones([Zone("NOPE", holder="pinky1", queue=["pinky2"]), Zone("N", "pinky2")])
    specs = view.specs(now=0.0)
    [held] = by_ns(specs, "zone_lanes")
    assert held.color[:3] == fv.ROBOT_COLORS["pinky2"][:3]


def test_held_zone_is_thick_in_the_holder_color():
    view = fv.FleetView(LANES)
    view.set_zones([Zone("AVENUE", holder="pinky3"), Zone("W")])
    specs = view.specs(now=0.0)
    [held] = by_ns(specs, "zone_lanes")
    [dots] = by_ns(specs, "zone_points")
    assert held.kind == "LINE_LIST"
    assert held.scale[0] >= 0.03
    assert held.color[:3] == fv.ROBOT_COLORS["pinky3"][:3]
    assert len(held.points) == 8
    assert xy(dots.points) == {pose_of("A"), pose_of("B")}


def test_zone_queue_is_listed_in_text():
    view = fv.FleetView(LANES)
    view.set_zones([Zone("N", holder="pinky1", queue=["pinky2", "pinky3"])])
    [text] = by_ns(view.specs(now=0.0), "zone_queue")
    assert "pinky2" in text.text and "pinky3" in text.text


def test_free_zone_without_queue_draws_nothing():
    view = fv.FleetView(LANES)
    view.set_zones([Zone("N")])
    specs = view.specs(now=0.0)
    assert not by_ns(specs, "zone_lanes") and not by_ns(specs, "zone_queue")


# ---- 막힌 레인 -----------------------------------------------------------------------------


def test_blocked_lane_is_red_until_cleared():
    view = fv.FleetView(LANES)
    view.on_event("LANE_BLOCKED", "pinky2", "S to L blocked for 12 s")
    [red] = by_ns(view.specs(now=0.0), "blocked")
    assert red.color[:3] == fv.RED[:3]
    assert segments(red.points) == {(pose_of("S"), pose_of("L"))}
    view.on_event("LANE_CLEARED", "pinky2", "S to L is clear")
    assert not by_ns(view.specs(now=0.0), "blocked")


def test_operator_clear_and_hyphenated_waypoints():
    view = fv.FleetView(LANES)
    view.on_event("LANE_BLOCKED", "pinky1", "P-5 to P-6 blocked for 10 s")
    view.on_event("LANE_BLOCKED", "pinky3", "A to B blocked for 10 s")
    assert view.blocked_lanes() == {("P-5", "P-6"), ("A", "B")}
    view.on_event("LANE_CLEARED", "pinky1", "P-5 to P-6 cleared by the operator")
    assert view.blocked_lanes() == {("A", "B")}


def test_bad_or_unrelated_events_are_ignored():
    view = fv.FleetView(LANES)
    view.on_event("LANE_BLOCKED", "pinky1", "nonsense")
    view.on_event("LANE_BLOCKED", "pinky1", "X to Y blocked for 10 s")
    view.on_event("LANE_BLOCKED", "pinky1", "W to L blocked for 10 s")  # 없는 레인
    view.on_event("BLOCKED_TOO_LONG", "pinky1", "S to L blocked for 60 s")
    view.on_event("LANE_CLEARED", "pinky2", "S to L is clear")  # 표시된 적 없음
    assert view.blocked_lanes() == set()


def test_new_block_from_the_same_robot_replaces_the_old_one():
    view = fv.FleetView(LANES)
    view.on_event("LANE_BLOCKED", "pinky1", "S to L blocked for 10 s")
    view.on_event("LANE_BLOCKED", "pinky1", "L to Q blocked for 10 s")
    assert view.blocked_lanes() == {("L", "Q")}


def test_grid_map_waypoints_parse_from_tuples():
    lanes = lane_map_module.LaneMap({
        "origin_m": [0, 0], "half_width_m": 0.05, "lane_entry_m": 0.1,
        "waypoints": [{"id": [1, 2], "x": 0.0, "y": 0.0}, {"id": [3, 2], "x": 1.0, "y": 0.0}],
        "lanes": [{"from": [1, 2], "to": [3, 2]}]})
    view = fv.FleetView(lanes)
    view.on_event("LANE_BLOCKED", "pinky1", "(1, 2) to (3, 2) blocked for 10 s")
    assert view.blocked_lanes() == {((1, 2), (3, 2))}
    labels = {s.text for s in by_ns(fv.lane_specs(lanes), "waypoint_labels")}
    assert labels == {"c1_2", "c3_2"}


# ---- RViz 설정 -----------------------------------------------------------------------------


def test_rviz_config_parses_and_shows_the_three_topics():
    text = (PKG / "rviz" / "fleet_view.rviz").read_text(encoding="utf-8")
    config = yaml.safe_load(text)
    manager = config["Visualization Manager"]
    assert manager["Global Options"]["Fixed Frame"] == "map"
    topics = {d["Class"].split("/")[-1]: d for d in manager["Displays"] if "Topic" in d}
    assert topics["Map"]["Topic"]["Value"] == "/pinky1/map"
    assert topics["Map"]["Topic"]["Durability Policy"] == "Transient Local"
    marker_topics = {d["Topic"]["Value"] for d in manager["Displays"]
                     if d["Class"].endswith("MarkerArray")}
    assert marker_topics == {"/fleet_view/markers", "/sim/obstacle_markers"}
    assert manager["Views"]["Current"]["Class"].endswith("TopDownOrtho")


def test_setup_installs_the_rviz_config_and_the_node():
    setup = (PKG / "setup.py").read_text(encoding="utf-8")
    assert "rviz/*.rviz" in setup
    assert "fleet_view = fleet_gz_sim.fleet_view_node:main" in setup


def test_a_recovering_robot_gets_the_amber_label_not_the_fault_red():
    text, color = fv.robot_label(Status("pinky1", "RECOVERING"), stale=False)
    assert "RECOVERING" in text
    assert color == fv.AMBER
    assert fv.AMBER not in (fv.RED, fv.GRAY, *fv.ROBOT_COLORS.values())


# ---- free-floor: held cells ------------------------------------------------------------


def floor_space():
    from dataclasses import replace

    from fleet_core.common.params import load_params
    from fleet_core.traffic.modes import zone_space
    return zone_space(LANES, replace(load_params(), traffic_mode="freefloor"))


def cell_near(space, x, y):
    return space.floor.cell_id(space.floor.cell_of(x, y))


def test_held_cells_are_squares_in_the_holder_color():
    space = floor_space()
    a, b = cell_near(space, *pose_of("W")), cell_near(space, pose_of("W")[0] + 0.1, pose_of("W")[1])
    c = cell_near(space, *pose_of("N"))
    view = fv.FleetView(space)
    view.set_zones([Zone(a, "pinky1"), Zone(b, "pinky1"), Zone(c, "pinky2"), Zone("W")])
    cells = by_ns(view.specs(now=0.0), "zone_cells")
    assert [s.kind for s in cells] == ["CUBE_LIST", "CUBE_LIST"]
    mine = {s.color[:3]: s for s in cells}
    one = mine[fv.ROBOT_COLORS["pinky1"][:3]]
    assert len(one.points) == 2
    assert one.scale[0] == pytest.approx(space.floor.cell_m)
    x0, y0, x1, y1 = space.cell_box(a)
    assert xy([((x0 + x1) / 2, (y0 + y1) / 2)]) <= xy(one.points)
    assert len(mine[fv.ROBOT_COLORS["pinky2"][:3]].points) == 1
    assert not by_ns(view.specs(now=0.0), "zone_lanes")  # cells are not lane zones


def test_cells_in_front_of_an_object_are_red():
    space = floor_space()
    view = fv.FleetView(space)
    view.set_zones([Zone(cell_near(space, *pose_of("W")), "#blocked")])
    [cells] = by_ns(view.specs(now=0.0), "zone_cells")
    assert cells.color[:3] == fv.RED[:3]


def test_lane_zones_still_draw_with_a_floor_space():
    view = fv.FleetView(floor_space())
    view.set_zones([Zone("AVENUE", holder="pinky3")])
    assert by_ns(view.specs(now=0.0), "zone_lanes")
    assert not by_ns(view.specs(now=0.0), "zone_cells")
