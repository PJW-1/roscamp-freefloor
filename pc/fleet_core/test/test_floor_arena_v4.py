"""Free-floor on the real arena, fourth layout (v4, 2026-11-10 night: 1.98 x 0.99 m, four objects,
map built from the lidar scans of the 07d7 Pinky). The stations stand where the user set them for
v3 (outbound before the middle of the west wall, the work dock in the south-east corner, waiting
in the north-east) and the way round is the figure eight the user asked for: the middle passage
is driven northwards by both loops, the east loop goes round object 4 (north passage east, WORK,
south passage west) and the west loop round object 3 (north of it into the west part, past the
outbound dock from north to south, out through the gap between objects 2 and 3). Every way into
the west part is under 0.22 m wide, so both outbound lines are narrow lines, and they go on
through the passages round object 4: a robot plans its own way in the empty east part only."""
import itertools
import math
import sys
from dataclasses import replace
from pathlib import Path

import pytest
import yaml
from fleet_core.common.commands import SHIPPED, EventOut, MoveCmd, ReportOrderCmd
from fleet_core.common.params import load_params
from fleet_core.manager import FleetManager
from fleet_core.traffic.floor.cells import disk_cells, swept_cells
from fleet_core.traffic.floor.floor_map import DEFAULT_FLOOR, floor_map_for
from fleet_core.traffic.floor.planner import FloorPlanner
from fleet_core.traffic.lane_map import load_lane_map

REPO = Path(__file__).resolve().parents[3]
LANES = load_lane_map(REPO / "sim" / "fake_robots" / "maps" / "arena_v4.yaml")
FLOOR = floor_map_for(LANES)
REAL = floor_map_for(load_lane_map(REPO / "sim" / "fake_robots" / "maps" / "real_arena.yaml"))
BODY = FLOOR.robot_radius_m + FLOOR.cell_margin_m
SPEED = 0.15
# floor_shortcut: the shortcut is off unless asked for; the tests here drive it
LOCKS = {"floor_lock_ahead_m": 0.15, "floor_extend_step_m": 0.05, "release_distance_m": 0.17,
         "floor_shortcut": True}
THREE = (("pinky1", "P-3"), ("pinky2", "P-2"), ("pinky3", "P-1"))
KINDS = [{"A": 1, "C": 1}, {"B": 1}, {"D": 1, "A": 1}, {"C": 1}, {"B": 1, "D": 1}, {"A": 1}]
MIDDLE_X = (0.85, 1.08)   # between objects 2 and 3 and object 4
OBJECT_1 = (0.37, 0.39, 0.54, 0.55)
OBJECT_3 = (0.70, 0.55, 0.83, 0.82)
OBJECT_2_TOP = 0.34
OBJECT_4 = (1.08, 0.25, 1.22, 0.76)
MERGE = (0.97, 0.445)     # the way out of the west part joins the middle passage here
MOUTH = (1.30, 0.875)     # east mouth of the passage north of object 4: the way out ends here

sys.path.insert(0, str(REPO / "tools"))
import compare_floor  # noqa: E402


def parts(cells) -> list[set]:
    left, found = set(cells), []
    while left:
        stack = [left.pop()]
        part = set(stack)
        while stack:
            i, j = stack.pop()
            for n in ((i + a, j + b) for a in (-1, 0, 1) for b in (-1, 0, 1)):
                if n in left:
                    left.discard(n)
                    part.add(n)
                    stack.append(n)
        found.append(part)
    return found


def cells_of(line) -> set:
    return set(swept_cells(FLOOR, list(line.points), BODY))


def leg(start, goal, out_line=None, in_line=None):
    return FloorPlanner(FLOOR).leg(start, goal, out_line=out_line, in_line=in_line)


def points(route):
    steps = [k * 0.01 for k in range(int(route.length / 0.01) + 1)]
    return [route.at(s) for s in [*steps, route.length]]


def scenario(robots=THREE, count=6, period_s=20.0, noise_m=0.03, timeout_s=600):
    return {
        "map": "arena_v4", "seed": 1, "speed_factor": 5.0, "timeout_s": timeout_s,
        "stock": {"arm1": {"A": 40, "B": 40}, "arm2": {"C": 40, "D": 40}},
        "orders": [{"at_s": period_s * k, "id": f"R-{k + 1:03d}", "items": KINDS[k % 6]}
                   for k in range(count)],
        "robots": {name: {"start": spot, "speed_mps": SPEED, "pose_noise_m": noise_m}
                   for name, spot in robots},
        "arm": {"seconds_per_item": 4.0, "swap_seconds": 8.0, "fail_rate": 0.0},
        "dock": {"duration_s": 3.0, "error_cm": 1.0}, "faults": [],
        "expect": {"orders_shipped": count},
    }


# ---- the map -----------------------------------------------------------------------------


def test_the_lane_map_names_this_floor_and_the_map_is_the_one_centimetre_scan_map():
    assert FLOOR.source_text == (DEFAULT_FLOOR.parent / "floor_map_arena_v4.yaml").read_text(
        encoding="utf-8")
    assert FLOOR.px_m == 0.01 and list(FLOOR.origin) == [-0.15, -0.15]
    image = (DEFAULT_FLOOR.parent / "arena_v4_fine.pgm").read_bytes()
    assert image.split(None, 4)[:4] == [b"P5", b"228", b"129", b"255"]  # 1.98 x 0.99 m + border
    assert image == (REPO / "sim" / "fleet_gz_sim" / "maps" / "arena_v4_fine.pgm").read_bytes()


def test_the_robot_and_cells_are_those_of_the_real_arena_but_the_wall_margin_is_smaller():
    """0.17 m north of object 3 leaves 0.085 m to each side of the centre line; a narrow line
    needs robot_radius_m + wall_margin_m and a quarter pixel, so the margin is 0.003 here."""
    names = ("robot_radius_m", "cell_margin_m", "cell_m", "robot_half_width_m", "side_margin_m")
    assert [getattr(FLOOR, n) for n in names] == [getattr(REAL, n) for n in names]
    assert FLOOR.wall_margin_m == 0.003 and REAL.wall_margin_m == 0.01


def test_the_clear_floor_is_the_east_part_with_its_passages_and_the_west_part():
    """No clear cell joins the two: every way into the west part is a narrow line. The only
    other clear cells are a few on the way out, where it widens between objects 1, 2 and 3."""
    east, west, *rest = sorted(parts(FLOOR.clear), key=len, reverse=True)
    way_out = cells_of(FLOOR.fixed_lines["outbound_out"])
    assert sum(len(part) for part in rest) <= 6 and all(part <= way_out for part in rest)
    assert FLOOR.cell_of(*FLOOR.docks["WORK"]) in east
    assert FLOOR.cell_of(*MERGE) in east
    assert FLOOR.cell_of(*FLOOR.docks["OUTBOUND"]) in west
    xs = [FLOOR.centre(c)[0] for c in FLOOR.clear]
    ys = [FLOOR.centre(c)[1] for c in FLOOR.clear]
    assert 0.06 < min(xs) < 0.16 and 1.82 < max(xs) < 1.92
    assert 0.08 < min(ys) < 0.16 and 0.84 < max(ys) < 0.91


def test_the_stations_stand_where_the_user_set_them():
    work, out = FLOOR.docks["WORK"], FLOOR.docks["OUTBOUND"]
    assert out[0] <= 0.25 and abs(out[1] - 0.50) <= 0.05
    assert work[0] >= 1.65 and work[1] <= 0.25
    spots = [pose for _, pose in FLOOR.parking]
    assert len(spots) == 3 and all(x >= 1.20 and y >= 0.50 for x, y in spots)
    assert all(FLOOR.cell_of(*p) in FLOOR.clear for p in [work, out, *spots])


def test_the_outbound_lines_are_narrow_and_the_work_line_is_locked_whole():
    """A robot that waits for the work dock stands before the line (a line that is not rolling):
    on the line it would stand in the way of the robot that leaves along the south wall."""
    assert sorted(FLOOR.fixed_lines) == ["outbound_in", "outbound_out", "outbound_short", "work_in"]
    assert FLOOR.narrow_lines == FLOOR.rolling_lines == frozenset(
        {"outbound_in", "outbound_out", "outbound_short"})
    for name in ("outbound_in", "outbound_out", "outbound_short"):
        assert FLOOR.line_floor_of[name], name


def test_the_way_in_goes_up_the_middle_and_north_of_object_three_to_the_dock_from_the_north():
    pts = points(leg(FLOOR.docks["WORK"], FLOOR.docks["OUTBOUND"], None, "outbound_in"))
    east = [p for p in pts if OBJECT_4[0] < p[0] < OBJECT_4[2]]
    assert east and all(p[1] < OBJECT_4[1] for p in east)  # south of object 4
    middle = [p for p in pts if OBJECT_2_TOP < p[1] < OBJECT_3[3] and p[0] > OBJECT_3[0]]
    assert middle and all(MIDDLE_X[0] < p[0] < MIDDLE_X[1] for p in middle)
    ys = [p[1] for p in pts if MIDDLE_X[0] < p[0] < MIDDLE_X[1] and 0.20 < p[1] < 0.85]
    assert ys == sorted(ys)  # northwards
    over = [p for p in pts if OBJECT_3[0] < p[0] < OBJECT_3[2]]
    assert over and all(p[1] > OBJECT_3[3] for p in over)
    assert pts[-1] == pytest.approx(FLOOR.docks["OUTBOUND"])
    last = [p for p in pts if math.dist(p, FLOOR.docks["OUTBOUND"]) < 0.20]
    assert [p[1] for p in last] == sorted((p[1] for p in last), reverse=True)  # southwards


def test_the_way_out_goes_on_south_through_the_gap_and_up_the_middle_to_the_north_passage():
    """Not back the way it came: the robot passes the dock from north to south."""
    pts = points(leg(FLOOR.docks["OUTBOUND"], FLOOR.docks["WORK"], "outbound_out", "work_in"))
    first = [p for p in pts if math.dist(p, FLOOR.docks["OUTBOUND"]) < 0.18]
    assert [p[1] for p in first] == sorted((p[1] for p in first), reverse=True)
    gap = [p for p in pts if OBJECT_3[0] < p[0] < OBJECT_3[2]]
    assert gap and all(OBJECT_2_TOP < p[1] < OBJECT_3[1] for p in gap)
    ys = [p[1] for p in pts if MIDDLE_X[0] < p[0] < MIDDLE_X[1] and 0.45 < p[1] < 0.85]
    assert ys and ys == sorted(ys)  # northwards, as the way in
    east = [p for p in pts if OBJECT_4[0] < p[0] < OBJECT_4[2]]
    assert east and all(p[1] > OBJECT_4[3] for p in east)  # north of object 4
    assert pts[-1] == pytest.approx(FLOOR.docks["WORK"])


def test_the_shortcut_goes_west_between_objects_two_and_three_and_north_between_one_and_three():
    """The red way the user drew (2026-11-11): up the middle passage as the way in, west through
    the gap between objects 2 and 3 on the way out's centre line, north-west through the gap
    between objects 1 and 3, and on the way in again above the dock, into it from the north."""
    short, way_in = FLOOR.fixed_lines["outbound_short"], FLOOR.fixed_lines["outbound_in"]
    assert FLOOR.shortcuts == {"outbound_short": "outbound_in"}
    assert short.start == way_in.start and short.end == way_in.end
    assert short.points[-2:] == way_in.points[-2:]  # the last stretch into the dock is the same
    assert short.length() <= way_in.length() - 0.30
    pts = points(leg(FLOOR.docks["WORK"], FLOOR.docks["OUTBOUND"], None, "outbound_short"))
    assert pts[-1] == pytest.approx(FLOOR.docks["OUTBOUND"])
    gap = [p for p in pts if OBJECT_3[0] < p[0] < OBJECT_3[2]]
    assert gap and all(OBJECT_2_TOP < p[1] < OBJECT_3[1] for p in gap)  # never north of object 3
    way_out = FLOOR.fixed_lines["outbound_out"].points
    assert (0.97, 0.445) in short.points and (0.70, 0.445) in short.points
    assert (0.97, 0.445) in way_out and (0.70, 0.445) in way_out  # the same centre line
    crossing = [p for p in pts  # object 1 top, object 3 foot (not the stretch into the dock)
                if abs(p[1] - OBJECT_3[1]) < 0.005 and p[0] > OBJECT_1[0]]
    assert crossing and all(OBJECT_1[2] < p[0] < OBJECT_3[0] for p in crossing)
    last = [p for p in pts if math.dist(p, FLOOR.docks["OUTBOUND"]) < 0.20]
    assert [p[1] for p in last] == sorted((p[1] for p in last), reverse=True)  # southwards


def test_the_shortcut_keeps_further_from_the_objects_than_the_way_in_north_of_object_three():
    """No smaller margin: the gap between objects 1 and 3 (0.19 m) leaves more room on its centre
    line than the passage north of object 3 (0.17 m) does on the way in."""
    def nearest(line):
        return min(FLOOR._gap_to_wall(x, y, 0.3) for x, y in FLOOR._samples(line, 0.0025))
    assert FLOOR.wall_margin_m == 0.003
    assert nearest(FLOOR.fixed_lines["outbound_short"]) >= nearest(
        FLOOR.fixed_lines["outbound_in"])
    assert nearest(FLOOR.fixed_lines["outbound_short"]) >= FLOOR.robot_radius_m + 0.01


def test_a_parked_robot_leaves_the_lines_and_the_other_spots_free():
    line_cells = set().union(*(cells_of(line) for line in FLOOR.fixed_lines.values()))
    bodies = {spot: set(disk_cells(FLOOR, x, y, BODY)) for spot, (x, y) in FLOOR.parking}
    for spot, cells in bodies.items():
        assert not cells & line_cells, spot
    for (a, one), (b, other) in itertools.combinations(bodies.items(), 2):
        assert not one & other, (a, b)
    for (_, one), (_, other) in itertools.combinations(FLOOR.parking, 2):
        assert math.dist(one, other) >= 0.28


def test_docks_parking_and_line_ends_are_the_lane_map_waypoints():
    for station, pose in FLOOR.docks.items():
        assert math.dist(pose, LANES.waypoint_pose(LANES.stations[station].dock)) < 1e-9
    assert [spot for spot, _ in FLOOR.parking] == list(LANES.parking)
    for spot, pose in FLOOR.parking:
        assert math.dist(pose, LANES.waypoint_pose(spot)) < 1e-9
    lines = FLOOR.fixed_lines
    assert lines["work_in"].start == LANES.waypoint_pose("QW")
    assert lines["work_in"].end == LANES.waypoint_pose("W")
    assert lines["outbound_in"].end == lines["outbound_out"].start == LANES.waypoint_pose("O")
    assert lines["outbound_out"].end == LANES.waypoint_pose("E1") == MOUTH
    assert MERGE in lines["outbound_out"].points and MERGE == LANES.waypoint_pose("M1")
    assert lines["outbound_in"].start == LANES.waypoint_pose("S1")


# ---- the shortcut, taken on the way -------------------------------------------------------

NORTH, SOUTH, EAST = math.pi / 2, -math.pi / 2, 0.0
DOCK = (*FLOOR.docks["OUTBOUND"], SOUTH)
ON_THE_WAY = (0.97, 0.20, NORTH)  # going up the middle passage, 0.245 m before the fork


def fleet(poses, left=None, **over):
    """Traffic of a fleet manager on this floor, robots standing at poses (x, y, yaw).
    left: robot -> seconds of arm work it has left at its station. over: other parameters."""
    params = replace(load_params(), traffic_mode="freefloor", **{**LOCKS, **over})
    core = FleetManager(LANES, params, list(poses), lambda: 0.0)
    for _ in range(3):
        for name, (x, y, yaw) in poses.items():
            core.on_robot_state(name, x, y, yaw, "IDLE")
    traffic = core.traffic
    traffic._plan_budget = 99
    traffic.station_left = lambda name: (left or {}).get(name)
    return traffic


def at_work(traffic, name):
    traffic._missions[name].phase = "AT_STATION"
    traffic._phase_at[name] = ("AT_STATION", 0.0)


def heads_in(traffic, name):
    """name sets off for the outbound dock and drives once."""
    m = traffic._missions[name]
    traffic._start_leg(m, "TO_OUTBOUND", "OUTBOUND", "O")
    assert traffic._legs[name].in_line == "outbound_in"  # planned the usual way first
    traffic._drive(m)
    return traffic._legs[name]


def moved(traffic, name, x, y, yaw, reports=20):
    """name now reports from (x, y): it drove there (the averaged place starts again there).
    Its progress is followed over that many reports (it goes on by a bounded step a report)."""
    m = traffic._missions[name]
    m.x, m.y, m.yaw = x, y, yaw
    for memory in (traffic._stand, traffic._recent):
        memory.pop(name, None)
    for _ in range(reports):
        m.last_seen = (m.last_seen or 0.0) + 0.1  # a new report each time
        traffic._follow(m)
    return m


def cell(traffic, x, y):
    return traffic.floor.cell_id(traffic.floor.cell_of(x, y))


GAP_23, GAP_13 = (0.78, 0.445), (0.628, 0.54)


def test_a_robot_takes_the_shortcut_when_it_is_through_before_the_docked_robot_comes_out():
    """The robot at the dock has 8 s of its swap left: the robot on its way is through the gaps
    long before that one comes out, so it leaves the way in at the fork."""
    traffic = fleet({"a": DOCK, "b": ON_THE_WAY}, left={"a": 8.0})
    at_work(traffic, "a")
    leg = heads_in(traffic, "b")
    assert leg.in_line == "outbound_short"
    assert points(leg.route)[-1] == pytest.approx(FLOOR.docks["OUTBOUND"])
    # the whole stretch it shares with the way out is held at once, through both gaps
    held = traffic.zones.held_set("b")
    assert {cell(traffic, *GAP_23), cell(traffic, *GAP_13)} <= held
    move = [c for c in traffic._out if isinstance(c, MoveCmd) and c.robot == "b"][-1]
    assert move.y > OBJECT_3[1] + 0.05 and move.x < GAP_13[0]  # sent past the gaps in one go
    assert [c.code for c in traffic._out
            if isinstance(c, EventOut) and c.robot == "b"].count("FLOOR_SHORTCUT") == 1


def test_no_robot_takes_the_shortcut_while_the_switch_is_off():
    """floor_shortcut is off unless asked for: the floor is then the one without the shortcut
    (its line, its gap cells), and a robot that would take it stays on the way in."""
    for name in ("fleet_params.yaml", "fleet_params_real_arena.yaml"):
        assert load_params(DEFAULT_FLOOR.parent / name).floor_shortcut is False
    traffic = fleet({"a": DOCK, "b": ON_THE_WAY}, left={"a": 8.0}, floor_shortcut=False)
    assert traffic.floor.shortcuts == {} and "outbound_short" not in traffic.floor.fixed_lines
    assert traffic.floor.cell_of(*GAP_13) not in traffic.floor.line_floor
    at_work(traffic, "a")
    leg = heads_in(traffic, "b")
    assert leg.in_line == "outbound_in"
    assert cell(traffic, *GAP_13) not in traffic.zones.held_set("b")
    assert not [c for c in traffic._out if isinstance(c, EventOut) and c.code == "FLOOR_SHORTCUT"]
    on = fleet({"a": DOCK, "b": ON_THE_WAY}, left={"a": 8.0})
    assert on.floor.shortcuts == {"outbound_short": "outbound_in"}  # the tests here turn it on


def test_a_robot_sent_again_to_the_dock_from_the_gap_goes_on_along_the_shortcut():
    """A new leg to the same dock (sent again after a stop) from between objects 2 and 3: the
    way in does not lead out of the gap, the shortcut does."""
    traffic = fleet({"a": DOCK, "b": ON_THE_WAY}, left={"a": 8.0})
    at_work(traffic, "a")
    assert heads_in(traffic, "b").in_line == "outbound_short"
    b = traffic._missions["b"]
    b = moved(traffic, "b", GAP_23[0], GAP_23[1], math.pi)
    traffic._start_leg(b, "TO_OUTBOUND", "OUTBOUND", "O")
    leg = traffic._legs["b"]
    assert leg.in_line == "outbound_short" and leg.route is not None
    pts = points(leg.route)
    assert pts[0] == pytest.approx(GAP_23) and pts[-1] == pytest.approx(FLOOR.docks["OUTBOUND"])
    assert all(p[0] <= GAP_23[0] + 1e-6 for p in pts)  # on west, never back east


AHEAD = (0.55, 0.636)  # on the shortcut past the gap between objects 1 and 3, in the stretch


def test_a_robot_on_the_shortcut_keeps_the_stretch_ahead_while_it_is_planned_again():
    """Planned again inside the shared stretch with no plan to spare this tick (the route goes and
    the goal is cancelled): the cells of the stretch still ahead stay its own until it is through,
    so a robot coming out never takes the far end of the gap while this one is in it."""
    traffic = fleet({"a": DOCK, "b": ON_THE_WAY}, left={"a": 8.0})
    at_work(traffic, "a")
    assert heads_in(traffic, "b").in_line == "outbound_short"
    assert cell(traffic, *AHEAD) in traffic.zones.held_set("b")
    b = traffic._missions["b"]
    b = moved(traffic, "b", GAP_23[0], GAP_23[1], math.pi)
    traffic._plan_budget = 0
    traffic._plan(b, traffic._legs["b"])
    assert traffic._legs["b"].route is None
    traffic._active["b"].clear()  # the cancelled MoveTo has ended
    b.goal = None
    for cell_id in traffic.zones.held_set("b"):
        b.outside[cell_id] = 99  # beyond the release distance for long
    traffic._release_cells(b)
    held = traffic.zones.held_set("b")
    assert cell(traffic, *AHEAD) in held and cell(traffic, *GAP_13) in held
    assert cell(traffic, 0.97, 0.30) not in held  # behind it: let go as before


def test_a_robot_on_the_shortcut_takes_the_stretch_back_when_an_object_in_it_is_gone():
    """An object reported in the gap marks the cells ahead blocked (moved from the robot to the
    mark). When it is gone the robot holds the cells of the stretch still ahead again, before
    a robot coming out can take them."""
    traffic = fleet({"a": DOCK, "b": ON_THE_WAY}, left={"a": 8.0})
    at_work(traffic, "a")
    assert heads_in(traffic, "b").in_line == "outbound_short"
    b = traffic._missions["b"]
    b = moved(traffic, "b", GAP_23[0], GAP_23[1], math.pi)
    traffic._mark_blocked(b)
    traffic._active["b"].clear()  # the cancelled MoveTo has ended: the cells go to the mark
    traffic._take_ahead(b)
    assert traffic.zones.holder(cell(traffic, *GAP_13)) == "#blocked"
    b.robot_state = "IDLE"
    for _ in range(traffic.params.release_confirm_samples):
        traffic._note_block_report(b)
    assert "b" not in traffic._marks
    assert traffic.zones.holder(cell(traffic, *GAP_13)) == "b"
    assert traffic.zones.holder(cell(traffic, *AHEAD)) == "b"


def test_a_robot_through_the_stretch_lets_it_go_as_before():
    traffic = fleet({"a": DOCK, "b": ON_THE_WAY}, left={"a": 8.0})
    at_work(traffic, "a")
    assert heads_in(traffic, "b").in_line == "outbound_short"
    b = traffic._missions["b"]
    b = moved(traffic, "b", 0.40, 0.678, math.pi)  # past the end of the stretch at (0.50, 0.65)
    traffic._active["b"].clear()
    b.goal = None
    for cell_id in traffic.zones.held_set("b"):
        b.outside[cell_id] = 99
    traffic._release_cells(b)
    assert cell(traffic, *GAP_13) not in traffic.zones.held_set("b")


def test_a_robot_on_the_shortcut_waits_at_the_end_of_the_stretch_while_the_dock_is_taken():
    """The docked robot works on for long. From the end of the shared stretch the shortcut passes
    the dock closer than two bodies and a report's noise, so a lock rolled on there and the docked
    robot's place wait for each other's cells (a deadlock nobody can step out of). The robot stops
    at the end of the stretch, where it is in nobody's way, until the cells before the dock are
    free, and then takes the rest of the way at once."""
    traffic = fleet({"a": DOCK, "b": ON_THE_WAY}, left={"a": 30.0})
    at_work(traffic, "a")
    leg = heads_in(traffic, "b")
    assert leg.in_line == "outbound_short"
    end = leg.s_lock  # the end of the shared stretch
    x, y = leg.route.at(end)
    b = moved(traffic, "b", x, y, math.atan2(0.12, -0.44))
    b.goal = None
    traffic._active["b"].clear()
    for _ in range(3):
        traffic._drive(b)
    assert traffic._legs["b"].s_lock == pytest.approx(end)
    held = traffic.zones.held_set("b")
    docked = (FLOOR.docks["OUTBOUND"][0] + 0.03, FLOOR.docks["OUTBOUND"][1] + 0.04)  # noisy report
    assert held.isdisjoint(traffic._body_cells(*docked))
    assert traffic._legs["b"].in_line == "outbound_short" and traffic._legs["b"].route is not None
    # the docked robot has left: the rest of the way to the dock is taken whole, never a part
    # (a lock rolled on would wait beside the dock again if another robot got there first)
    a = traffic._missions["a"]
    for cell_id in list(traffic.zones.held_set("a")):
        traffic.zones.release("a", cell_id)
    a.x, a.y = 0.97, 0.80
    traffic._drive(b)
    assert traffic._legs["b"].s_lock == pytest.approx(traffic._legs["b"].route.length)


def test_a_robot_on_the_shortcut_rolls_on_past_the_stretch_when_the_dock_is_free():
    traffic = fleet({"a": (1.86, 0.58, 0.0), "b": ON_THE_WAY})
    leg = heads_in(traffic, "b")
    assert leg.in_line == "outbound_short"
    end = leg.s_lock
    x, y = leg.route.at(end - 0.10)
    b = moved(traffic, "b", x, y, math.atan2(0.12, -0.44))
    traffic._drive(b)
    assert traffic._legs["b"].s_lock == pytest.approx(traffic._legs["b"].route.length)


def test_a_report_thrown_aside_before_the_last_corner_does_not_put_the_robot_past_it():
    """Above the dock the shortcut folds back by 105 degrees, and the whole way there is sent in
    one go. A report thrown 8 cm south, 0.16 m before the corner, lies nearer the stretch after
    the corner than the one the robot is on. Its progress must not jump past the corner: the
    cells it turns on there would be let go. (Headless run, 3 robots, seed 4: 0.26 m in one
    report, then "on cells it does not hold" at the corner.)"""
    traffic = fleet({"a": (1.86, 0.58, 0.0), "b": ON_THE_WAY})
    leg = heads_in(traffic, "b")
    assert leg.in_line == "outbound_short"
    assert leg.s_sent == pytest.approx(leg.route.length)  # the dock is free: all of it
    west = math.atan2(0.12, -0.44)
    corner = leg.route.length - 0.22
    while leg.s < corner - 0.16 - 0.10:
        moved(traffic, "b", *leg.route.at(leg.s + 0.10), west)
    moved(traffic, "b", *leg.route.at(corner - 0.16), west)
    assert leg.s == pytest.approx(corner - 0.16, abs=0.01)
    b = moved(traffic, "b", 0.223, 0.641, west, reports=1)
    assert leg.s < corner
    once = leg.s
    for _ in range(5):  # the ticks until the next report see the same one: no further
        traffic._follow(b)
    assert leg.s == once


def test_a_robot_that_was_silent_for_a_while_is_followed_up_to_where_it_is():
    """The progress goes on by at most FOLLOW_AHEAD_M a report, so it catches up with a robot
    that drove on unheard within a few reports and stays behind it until then."""
    traffic = fleet({"a": (1.86, 0.58, 0.0), "b": ON_THE_WAY})
    leg = heads_in(traffic, "b")
    x, y = leg.route.at(0.60)
    moved(traffic, "b", x, y, math.pi, reports=1)
    assert leg.s == pytest.approx(0.15)  # FOLLOW_AHEAD_M
    moved(traffic, "b", x, y, math.pi, reports=1)
    assert leg.s == pytest.approx(0.30)
    moved(traffic, "b", x, y, math.pi, reports=5)
    assert leg.s == pytest.approx(0.60, abs=0.01)


def test_a_robot_running_through_the_stretch_to_a_taken_dock_counts_as_moving():
    """Others give way to a robot that will pass soon (_moving). One on the shortcut that will
    stop at the end of the stretch is still passing while it runs there."""
    traffic = fleet({"a": DOCK, "b": ON_THE_WAY}, left={"a": 30.0})
    at_work(traffic, "a")
    leg = heads_in(traffic, "b")
    assert leg.in_line == "outbound_short"
    assert traffic._moving("b")
    x, y = leg.route.at(leg.s_lock)
    b = moved(traffic, "b", x, y, math.atan2(0.12, -0.44))
    b.goal = None
    traffic._active["b"].clear()
    traffic._drive(b)
    assert not traffic._moving("b")  # stands at the end of the stretch


@pytest.mark.parametrize("report", [(1.04, 0.085), (1.04, 0.125), (1.049, 0.155)])
def test_a_robot_whose_report_is_off_the_line_takes_the_shortcut_from_where_it_is(report):
    """The shortcut goes on from the robot's place. (Planned like a new leg, a report a few cm
    beside the line led back east to the start of the line and along it again: 0.6 m more,
    against the robots behind.)"""
    traffic = fleet({"a": (1.86, 0.58, 0.0), "b": (1.25, 0.125, math.pi)})
    leg = heads_in(traffic, "b")
    assert leg.in_line == "outbound_in"
    for x in (1.20, 1.15, 1.10):
        moved(traffic, "b", x, 0.125, math.pi)
        traffic._drive(traffic._missions["b"])
    assert not traffic._legs["b"].short_decided
    b = moved(traffic, "b", *report, math.pi)
    traffic._drive(b)
    leg = traffic._legs["b"]
    assert leg.in_line == "outbound_short"
    assert max(x for x, _ in points(leg.route)) <= report[0] + 1e-9
    assert leg.route.length < 1.60
    assert leg.route.fixed == [(0.0, leg.route.length, "outbound_short")]


def test_a_robot_takes_the_shortcut_behind_one_already_past_the_merge():
    """One on the last stretch into the dock (past the merge) is not raced: both ways end
    behind it, so the shortcut is taken."""
    traffic = fleet({"c": (0.17, 0.66, SOUTH), "b": ON_THE_WAY})
    traffic._start_leg(traffic._missions["c"], "TO_OUTBOUND", "OUTBOUND", "O")
    assert heads_in(traffic, "b").in_line == "outbound_short"


@pytest.mark.parametrize("left", [0.0, 2.0])
def test_a_robot_keeps_to_the_way_in_when_the_docked_robot_comes_out_first(left):
    traffic = fleet({"a": DOCK, "b": ON_THE_WAY}, left={"a": left})
    at_work(traffic, "a")
    leg = heads_in(traffic, "b")
    assert leg.in_line == "outbound_in"
    assert not {cell(traffic, *GAP_23), cell(traffic, *GAP_13)} & traffic.zones.held_set("b")


def test_a_robot_keeps_to_the_way_in_while_another_leaves_the_dock_for_the_gaps():
    traffic = fleet({"a": DOCK, "b": ON_THE_WAY})
    traffic._start_leg(traffic._missions["a"], "TO_PARK", None, "P-1")
    assert traffic._legs["a"].out_line == "outbound_out"
    assert heads_in(traffic, "b").in_line == "outbound_in"


def test_a_robot_takes_the_shortcut_when_the_one_that_left_is_past_the_gaps():
    traffic = fleet({"a": (1.10, 0.875, EAST), "b": ON_THE_WAY})
    traffic._start_leg(traffic._missions["a"], "TO_PARK", None, "P-1")
    assert heads_in(traffic, "b").in_line == "outbound_short"


def test_a_robot_keeps_to_the_way_in_behind_one_already_on_it_to_the_same_dock():
    """Two ways that join above the dock are not raced: the one behind follows."""
    traffic = fleet({"c": (0.80, 0.905, math.pi), "b": ON_THE_WAY})
    traffic._start_leg(traffic._missions["c"], "TO_OUTBOUND", "OUTBOUND", "O")
    assert heads_in(traffic, "b").in_line == "outbound_in"


def test_a_robot_behind_on_the_same_way_does_not_stop_the_one_in_front_taking_it():
    traffic = fleet({"b": ON_THE_WAY, "c": (1.32, 0.125, math.pi)})
    traffic._start_leg(traffic._missions["c"], "TO_OUTBOUND", "OUTBOUND", "O")
    assert heads_in(traffic, "b").in_line == "outbound_short"


def test_a_robot_past_the_fork_stays_on_the_way_in():
    traffic = fleet({"b": (0.97, 0.60, NORTH)})
    assert heads_in(traffic, "b").in_line == "outbound_in"


def test_a_robot_far_before_the_fork_does_not_choose_yet():
    traffic = fleet({"a": DOCK, "b": (1.25, 0.125, math.pi)}, left={"a": 2.0})
    at_work(traffic, "a")
    leg = heads_in(traffic, "b")  # the docked robot would come out first, but not yet known
    assert leg.in_line == "outbound_in" and not leg.short_decided


# ---- runs ---------------------------------------------------------------------------------


def run(tmp_path, monkeypatch, seed, robots, count, period_s, timeout_s, noise_m=0.03):
    path = tmp_path / "arena_v4_run.yaml"
    path.write_text(yaml.safe_dump(scenario(robots, count, period_s, noise_m, timeout_s)),
                    encoding="utf-8")
    monkeypatch.setattr(compare_floor, "OVERRIDES", dict(LOCKS))
    row, world = compare_floor.run("arena_v4", str(path), seed, "freefloor")
    assert world is not None, row["error"]
    assert row["finished"], {k: row[k] for k in ("error", "end_s", "orders_shipped")}
    assert row["orders_shipped"] == count
    assert row["unresolved"] == 0 and row["isolations"] == 0
    assert world.cell_violations == 0
    assert row["min_gap_m"] >= 0.22  # two bodies and their cell margins
    return row, world


@pytest.mark.parametrize("seed", [1, 2])
def test_three_pinkys_ship_six_orders_and_park_again(tmp_path, monkeypatch, seed):
    run(tmp_path, monkeypatch, seed, THREE, 6, 20, 600)


def test_three_pinkys_ship_six_orders_when_every_report_is_exact(tmp_path, monkeypatch):
    run(tmp_path, monkeypatch, 1, THREE, 6, 20, 600, noise_m=0.0)


def test_three_pinkys_keep_up_when_orders_come_faster_than_they_ship(tmp_path, monkeypatch):
    run(tmp_path, monkeypatch, 1, THREE, 16, 15, 900)


def test_two_pinkys_ship_six_orders(tmp_path, monkeypatch):
    run(tmp_path, monkeypatch, 1, THREE[:2], 6, 15, 600)


def test_every_robot_drives_the_figure_eight_one_way(monkeypatch):
    """Middle passage northwards, south passage westwards, north passage eastwards, and past
    the outbound dock southwards, for the whole run."""
    monkeypatch.setattr(compare_floor, "OVERRIDES", dict(LOCKS))
    world = compare_floor.build(scenario(count=6, noise_m=0.0), "arena_v4", 1, "freefloor")
    before = {name: (r.x, r.y) for name, r in world.virtual.items()}
    wrong, seen = [], {name: set() for name in world.virtual}
    ways = {  # name: (box x0, y0, x1, y1, axis, sign)
        "middle": (0.87, 0.46, 1.07, 0.80, 1, 1),
        "south": (OBJECT_4[0], 0.0, OBJECT_4[2], OBJECT_4[1], 0, -1),
        "north": (OBJECT_4[0], OBJECT_4[3], OBJECT_4[2], 0.99, 0, 1),
        "dock": (0.0, 0.36, 0.30, 0.70, 1, -1),
    }
    while world.t < 400.0:
        world.step()
        for name, r in world.virtual.items():
            was, now = before[name], (r.x, r.y)
            before[name] = now
            for way, (x0, y0, x1, y1, axis, sign) in ways.items():
                if x0 < now[0] < x1 and y0 < now[1] < y1:
                    seen[name].add(way)
                    if (now[axis] - was[axis]) * sign < -1e-6:
                        wrong.append((round(world.t, 1), name, way, was, now))
        if len([c for _, c in world.commands
                if isinstance(c, ReportOrderCmd) and c.status == SHIPPED]) == 6:
            break
    assert not wrong, wrong[:5]
    assert all(seen[name] == set(ways) for name in seen), seen
    assert sum(isinstance(c, ReportOrderCmd) and c.status == SHIPPED
               for _, c in world.commands) == 6


def test_robots_take_the_shortcut_in_a_run_and_ship_every_order(tmp_path, monkeypatch):
    _, world = run(tmp_path, monkeypatch, 1, THREE, 6, 20, 600)
    assert any(e.code == "FLOOR_SHORTCUT" for e in world.events)


def test_robots_ship_every_order_without_the_shortcut_while_the_switch_is_off(tmp_path,
                                                                              monkeypatch):
    monkeypatch.setitem(LOCKS, "floor_shortcut", False)
    _, world = run(tmp_path, monkeypatch, 1, THREE, 6, 20, 600)
    assert world.core.traffic.metrics["shortcuts"] == 0
    assert not any(e.code == "FLOOR_SHORTCUT" for e in world.events)
