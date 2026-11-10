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
from pathlib import Path

import pytest
import yaml
from fleet_core.common.commands import SHIPPED, ReportOrderCmd
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
LOCKS = {"floor_lock_ahead_m": 0.15, "floor_extend_step_m": 0.05, "release_distance_m": 0.17}
THREE = (("pinky1", "P-3"), ("pinky2", "P-2"), ("pinky3", "P-1"))
KINDS = [{"A": 1, "C": 1}, {"B": 1}, {"D": 1, "A": 1}, {"C": 1}, {"B": 1, "D": 1}, {"A": 1}]
MIDDLE_X = (0.85, 1.08)   # between objects 2 and 3 and object 4
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
    assert sorted(FLOOR.fixed_lines) == ["outbound_in", "outbound_out", "work_in"]
    assert FLOOR.narrow_lines == FLOOR.rolling_lines == frozenset({"outbound_in", "outbound_out"})
    for name in ("outbound_in", "outbound_out"):
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
    return row


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
