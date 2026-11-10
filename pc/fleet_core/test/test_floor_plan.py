"""자유바닥 경로 계획(P2): A* -> 시선 지름길 -> 원호, 고정 선, Route 계산."""
import math

import pytest
from fleet_core.traffic.floor.cells import line_cells, sample_line
from fleet_core.traffic.floor.floor_map import FloorMap, load_floor_map
from fleet_core.traffic.floor.planner import NEIGHBOURS, FloorPlanner, Route, polyline_length


@pytest.fixture(scope="module")
def floor():
    return load_floor_map()


@pytest.fixture(scope="module")
def planner(floor):
    return FloorPlanner(floor)


def on_clear_floor(floor, points):
    return line_cells(floor, points) <= floor.clear


def turns(points):
    """꺾인 각의 합(rad)."""
    total = 0.0
    for a, b, c in zip(points, points[1:], points[2:], strict=False):
        u = math.atan2(b[1] - a[1], b[0] - a[0])
        v = math.atan2(c[1] - b[1], c[0] - b[0])
        total += abs((v - u + math.pi) % (2 * math.pi) - math.pi)
    return total


def test_open_floor_gives_a_straight_line(planner):
    path = planner.plan((1.0, 0.8), (2.5, 0.8))
    assert path == [(1.0, 0.8), (2.5, 0.8)]


def test_a_box_is_driven_around_on_clear_floor():
    floor = FloorMap({"bounds": [0, 0, 2, 1.4], "boxes": [[0.9, 0.0, 1.1, 0.8]], "cell_m": 0.1,
                      "robot_radius_m": 0.08, "cell_margin_m": 0.05})
    planner = FloorPlanner(floor)
    path = planner.plan((0.35, 0.35), (1.65, 0.35))
    assert path[0] == (0.35, 0.35) and path[-1] == (1.65, 0.35)
    assert on_clear_floor(floor, path)
    assert max(y for _, y in path) > 0.8  # over the box
    # any-angle + arcs: far shorter than the 4-connected detour and with few turns
    assert polyline_length(path) < 2.6


def test_shortcuts_remove_grid_staircases(planner, floor):
    start, goal = (0.40, 1.30), (2.40, 2.00)
    cells = planner.astar(floor.cell_of(*start), floor.cell_of(*goal), floor.clear)
    raw = [start] + [floor.centre(c) for c in cells[1:-1]] + [goal]
    path = planner.plan(start, goal)
    assert turns(path) < turns(raw)
    assert polyline_length(path) <= polyline_length(raw) + 1e-9


def test_corners_become_arcs_with_small_steps():
    floor = FloorMap({"bounds": [0, 0, 2, 2], "boxes": [[0.0, 0.0, 1.4, 1.4]], "cell_m": 0.1,
                      "robot_radius_m": 0.08, "cell_margin_m": 0.05})
    planner = FloorPlanner(floor)
    path = planner.plan((0.3, 1.75), (1.75, 0.3))
    assert on_clear_floor(floor, path)
    # no sharp turn left: every bend between successive steps is small
    for a, b, c in zip(path, path[1:], path[2:], strict=False):
        u = math.atan2(b[1] - a[1], b[0] - a[0])
        v = math.atan2(c[1] - b[1], c[0] - b[0])
        assert abs((v - u + math.pi) % (2 * math.pi) - math.pi) < math.radians(50)


def test_no_path_through_a_wall_gives_none():
    floor = FloorMap({"bounds": [0, 0, 2, 1], "boxes": [[0.9, 0.0, 1.1, 1.0]], "cell_m": 0.1,
                      "robot_radius_m": 0.08, "cell_margin_m": 0.05})
    assert FloorPlanner(floor).plan((0.4, 0.5), (1.6, 0.5)) is None


def test_avoided_cells_are_not_entered(planner, floor):
    wall = {floor.cell_of(1.85, 0.4 + 0.1 * k) for k in range(14)}
    path = planner.plan((1.2, 1.0), (2.5, 1.0), avoid=frozenset(wall))
    assert not line_cells(floor, path) & wall


def test_a_penalty_steers_the_path(planner, floor):
    straight = planner.plan((1.0, 1.15), (2.6, 1.15))
    busy = {floor.cell_of(*p) for p in sample_line(straight, 0.05)}
    busy -= {floor.cell_of(1.0, 1.15), floor.cell_of(2.6, 1.15)}

    def penalty(cell, s):
        return 5.0 if cell in busy else 0.0

    bent = planner.plan((1.0, 1.15), (2.6, 1.15), penalty=penalty)
    assert {floor.cell_of(*p) for p in sample_line(bent, 0.05)} != busy | {
        floor.cell_of(1.0, 1.15), floor.cell_of(2.6, 1.15)}
    assert polyline_length(bent) > polyline_length(straight)


def test_a_dock_leg_ends_on_the_fixed_line(planner, floor):
    route = planner.leg((0.35, 1.95), floor.docks["WORK"], in_line="work_in")
    line = floor.fixed_lines["work_in"]
    assert route.points[-1] == line.end
    assert line.start in route.points
    s0, s1, name = route.fixed[-1]
    assert name == "work_in" and math.isclose(s1, route.length)
    assert math.isclose(s1 - s0, line.length())
    assert on_clear_floor(floor, route.points)


def test_leaving_a_dock_follows_the_out_line_first(planner, floor):
    route = planner.leg(floor.docks["WORK"], (1.20, 2.05), out_line="work_out")
    assert route.points[1] == floor.fixed_lines["work_out"].end
    assert route.fixed[0][:1] == (0.0,) and route.fixed[0][2] == "work_out"


def test_route_measures_and_projects():
    route = Route([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])
    assert route.length == 2.0
    assert route.at(1.5) == (1.0, 0.5)
    assert route.between(0.5, 1.5) == [(0.5, 0.0), (1.0, 0.0), (1.0, 0.5)]
    s, d = route.project(0.4, 0.1)
    assert math.isclose(s, 0.4) and math.isclose(d, 0.1)
    s, _ = route.project(0.9, 0.9, s_from=1.2)  # not before s_from
    assert s >= 1.2
    route.fixed = [(1.0, 2.0, "line")]
    assert route.fixed_at(1.5) == (1.0, 2.0, "line") and route.fixed_at(0.5) is None


def test_a_robot_already_on_the_in_line_joins_it_where_it_stands(planner, floor):
    """A robot that waited on the approach line goes straight up it, not back round to its start
    (where the next robot in the queue waits)."""
    line = floor.fixed_lines["work_in"]
    (x0, y0), (x1, y1) = line.points[0], line.points[-1]
    start = (x0 + 0.45 * (x1 - x0) + 0.02, y0 + 0.45 * (y1 - y0) - 0.02)  # a little off the line
    route = planner.leg(start, line.points[-1], in_line="work_in")
    assert route.points[0] == start
    assert route.length < 0.6 * line.length() + 0.05
    assert route.fixed == [(0.0, route.length, "work_in")]


def test_reservation_time_is_the_distance_driven_not_the_cost(planner, floor):
    """The penalty is asked for the time a cell is reached: distance along the path, without
    earlier penalties or the fixed-line cost folded in."""
    seen = []

    def penalty(cell, s):
        seen.append((cell, s))
        return 100.0  # far more than any distance on this floor

    a, b = floor.cell_of(1.0, 0.8), floor.cell_of(1.5, 0.8)
    planner.astar(a, b, floor.clear, penalty)
    assert seen
    assert max(s for _, s in seen) < 10.0


def strip(floor):
    """(1.8, 1.15)를 가로지르는 세로 띠(두 칸 두께): 남이 지나가는 길."""
    return {floor.cell_of(1.75 + 0.1 * i, 0.85 + 0.1 * k) for i in range(2) for k in range(7)}


def waiting(band, free_at):
    """띠는 거리 시각 free_at까지 남이 쓴다: 그 전에 닿으면 빌 때까지 기다린다."""
    def wait(cell, s, heading):
        return (max(0.0, free_at - s), 0.0) if cell in band else (0.0, 0.0)
    return wait


def test_a_short_wait_on_the_way_keeps_the_straight_way(planner, floor):
    straight = planner.plan((1.0, 1.15), (2.6, 1.15))
    waited = planner.plan((1.0, 1.15), (2.6, 1.15), wait=waiting(strip(floor), 0.9))
    assert waited == straight  # 0.2 m어치 기다림이 띠를 도는 길보다 싸다


def test_a_long_wait_on_the_way_goes_round(planner, floor):
    band = strip(floor)
    cells = planner.astar(floor.cell_of(1.05, 1.15), floor.cell_of(2.65, 1.15), floor.clear,
                          wait=waiting(band, 20.0))
    assert not set(cells) & band


def driven(floor, cells) -> float:
    return sum(math.dist(floor.centre(p), floor.centre(q))
               for p, q in zip(cells, cells[1:], strict=False))


def timed(planner, floor, a, b, wait):
    """(칸 경로, 그 경로로 b에 닿을 때 wait가 받은 s)."""
    seen = {}

    def spy(cell, s, heading):
        seen[cell, heading] = s
        return wait(cell, s, heading)

    cells = planner.astar(a, b, floor.clear, wait=spy)
    last = (cells[-1][0] - cells[-2][0], cells[-1][1] - cells[-2][1])
    return cells, seen[b, last]


def test_cells_after_a_wait_are_reached_that_much_later(planner, floor):
    """기다린 만큼 그 뒤 칸에 늦게 닿는다(띠의 둘째 칸에서 또 기다리지 않는다)."""
    band = strip(floor)
    a, b = floor.cell_of(1.05, 1.15), floor.cell_of(2.65, 1.15)
    free = planner.astar(a, b, floor.clear)
    enters = driven(floor, free[:next(k for k, c in enumerate(free) if c in band) + 1])
    cells, s = timed(planner, floor, a, b, waiting(band, enters + 0.2))
    assert cells == free  # 0.2 m어치 기다림: 길은 그대로
    assert s == pytest.approx(driven(floor, free) + 0.2)


def test_a_flat_cost_from_the_wait_callback_does_not_delay_later_cells(planner, floor):
    band = strip(floor)
    a, b = floor.cell_of(1.05, 1.15), floor.cell_of(2.65, 1.15)
    cells, s = timed(planner, floor, a, b,
                     lambda cell, s, heading: (0.0, 0.01) if cell in band else (0.0, 0.0))
    assert set(cells) & band
    assert s == pytest.approx(driven(floor, cells))


def test_the_wait_callback_gets_the_step_direction(planner, floor):
    seen = set()

    def wait(cell, s, heading):
        seen.add(heading)
        return (0.0, 0.0)

    planner.astar(floor.cell_of(1.05, 1.15), floor.cell_of(1.55, 1.15), floor.clear, wait=wait)
    assert (1, 0) in seen and all(h in NEIGHBOURS for h in seen)


def test_joining_the_line_from_a_cell_too_close_to_a_wall_is_refused():
    floor = FloorMap({"bounds": [0, 0, 2, 1.4], "boxes": [[0.8, 0.9, 1.2, 1.4]], "cell_m": 0.1,
                      "robot_radius_m": 0.08, "cell_margin_m": 0.05,
                      "fixed_lines": {"l": [[0.4, 0.65], [1.6, 0.65]]}})
    planner = FloorPlanner(floor)
    start = (1.0, 0.76)  # 0.11 m off the line, but its cell is too close to the box
    assert floor.cell_of(*start) not in floor.clear
    route = planner.leg(start, (1.6, 0.65), in_line="l")
    assert route is None or route.points[0] != start or route.fixed[0][0] > 0.0


# ---- a start that drifted off clear floor ----------------------------------------------------
# A pinky can stop a few cm into the wall margin (Gazebo warehouse run, 2026-11-09: 7 cm, after
# stepping aside in a wait cycle). Its cell is free but not clear, and planning from it gave no
# path on every retry, so the robot stood there for good.

def box_floor():
    return FloorMap({"bounds": [0, 0, 2, 1.4], "boxes": [[0.8, 0.9, 1.2, 1.4]], "cell_m": 0.1,
                     "robot_radius_m": 0.08, "cell_margin_m": 0.05})


def test_a_start_just_off_clear_floor_steps_onto_it_first():
    floor = box_floor()
    start = (1.0, 0.76)
    assert floor.cell_of(*start) in floor.free and floor.cell_of(*start) not in floor.clear
    path = FloorPlanner(floor).plan(start, (1.6, 0.4))
    assert path is not None and path[0] == start
    assert floor.cell_of(*path[1]) in floor.clear and math.dist(start, path[1]) <= 0.2
    assert line_cells(floor, path[:2]) <= floor.free
    assert on_clear_floor(floor, path[1:])


def test_the_step_onto_clear_floor_avoids_cells_of_others():
    floor = box_floor()
    start = (1.0, 0.76)
    nearest = floor.cell_of(*FloorPlanner(floor).plan(start, (1.6, 0.4))[1])
    path = FloorPlanner(floor).plan(start, (1.6, 0.4), avoid=frozenset({nearest}))
    assert path is not None and floor.cell_of(*path[1]) != nearest


def test_a_start_inside_a_box_still_gives_none():
    floor = box_floor()
    assert FloorPlanner(floor).plan((1.0, 1.1), (1.6, 0.4)) is None


def test_when_the_nearest_clear_cell_leads_nowhere_the_next_one_is_tried():
    floor = box_floor()
    start = (1.0, 0.76)
    dead_end = floor.centre((3, 3))
    ring = {(3 + di, 3 + dj) for di in (-1, 0, 1) for dj in (-1, 0, 1)} - {(3, 3)}

    class DeadEndFirst(FloorPlanner):
        # on this floor the clear cells next to the start touch each other, so put a walled-in
        # clear cell first to see the next one tried
        def _steps_onto_clear(self, start, avoid=frozenset()):
            return [dead_end, *super()._steps_onto_clear(start, avoid)]

    path = DeadEndFirst(floor).plan(start, (1.6, 0.4), avoid=frozenset(ring))
    assert path is not None and path[0] == start
    assert path[1] != dead_end and floor.cell_of(*path[1]) in floor.clear


def test_the_step_onto_clear_floor_does_not_run_over_cells_of_others():
    floor = box_floor()
    start = (1.0, 0.76)
    # others hold the two nearest clear cells; the straight steps to the two next ones run
    # over those cells, so there is no step to take
    others = frozenset({floor.cell_of(1.05, 0.65), floor.cell_of(0.95, 0.65)})
    path = FloorPlanner(floor).plan(start, (1.6, 0.4), avoid=others)
    if path is not None:
        assert not (line_cells(floor, path[:2]) - {floor.cell_of(*start)}) & others
    assert path is None


def test_the_step_onto_clear_floor_leaves_a_start_cell_held_by_a_close_robot():
    floor = box_floor()
    start = (1.0, 0.76)
    path = FloorPlanner(floor).plan(start, (1.6, 0.4), avoid=frozenset({floor.cell_of(*start)}))
    assert path is not None and floor.cell_of(*path[1]) in floor.clear


# ---- narrow lines: a dock in a gap that holds no clear cell --------------------------------

def _gap_floor():
    from test_floor_map import gap_data
    from fleet_core.traffic.floor.floor_map import FloorMap
    return FloorMap(gap_data())


def test_a_leg_goes_into_and_out_of_a_dock_on_a_narrow_line():
    from fleet_core.traffic.floor.planner import FloorPlanner
    floor = _gap_floor()
    planner = FloorPlanner(floor)
    way_in = planner.leg((0.21, 0.75), (0.75, 0.45), in_line="work_in")
    assert way_in is not None and way_in.points[-2:] == [(0.75, 0.75), (0.75, 0.45)]
    way_out = planner.leg((0.75, 0.45), (1.29, 0.15), out_line="work_out")
    assert way_out is not None and way_out.points[:2] == [(0.75, 0.45), (0.75, 0.15)]


def test_a_robot_stopped_on_a_narrow_line_goes_on_along_it():
    from fleet_core.traffic.floor.planner import FloorPlanner
    planner = FloorPlanner(_gap_floor())
    route = planner.leg((0.75, 0.57), (0.75, 0.45), in_line="work_in")
    assert route is not None and route.points == [(0.75, 0.57), (0.75, 0.45)]


def test_free_paths_keep_out_of_the_gap_of_a_narrow_line():
    from fleet_core.traffic.floor.planner import FloorPlanner
    planner = FloorPlanner(_gap_floor())
    path = planner.plan((0.75, 0.75), (0.75, 0.15))  # straight through would be the shortest
    assert path is not None
    route_points = [(a[0] + (b[0] - a[0]) * k / 50, a[1] + (b[1] - a[1]) * k / 50)
                    for a, b in zip(path, path[1:]) for k in range(51)]
    assert not [p for p in route_points if 0.65 < p[0] < 0.85 and 0.30 < p[1] < 0.60]
    assert planner.plan((0.21, 0.75), (0.75, 0.45)) is None  # the dock is reached by its line only


def _in_gap(points):
    return [p for p in points if 0.65 < p[0] < 0.85 and 0.30 < p[1] < 0.60]


def test_a_robot_in_the_gap_without_a_line_to_follow_leaves_forward_along_the_narrow_lines():
    """Told to go somewhere else while on the way in (no out line, no in line): on to the dock
    and out along the way out, tagged as fixed line all the way, then over the floor."""
    from fleet_core.traffic.floor.planner import FloorPlanner
    planner = FloorPlanner(_gap_floor())
    route = planner.leg((0.75, 0.55), (0.21, 0.75))
    assert route is not None
    assert route.points[:3] == [(0.75, 0.55), (0.75, 0.45), (0.75, 0.15)]
    assert [name for _, _, name in route.fixed] == ["work_in", "work_out"]
    assert route.fixed[0][0] == 0.0 and route.fixed[-1][1] == pytest.approx(0.40)
    assert all(p[0] == 0.75 for p in _in_gap(route.points))
    assert route.points[-1] == (0.21, 0.75)


@pytest.mark.parametrize("x", [0.725, 0.775])
def test_a_report_off_the_centre_line_in_the_gap_goes_on_without_a_sideways_step(x):
    """Reports are a few cm off. Every point the robot is sent to is on the centre line, and it
    joins the line 0.05 m ahead, not square to it (a turn on the spot in the gap), whichever
    line the robot is on. This is where it is sent, not proof that the body clears the walls:
    a robot that really stands 2.5 cm off the line has 0.075 m to the box, less than the 0.09 m
    the line keeps."""
    from fleet_core.traffic.floor.planner import FloorPlanner
    planner = FloorPlanner(_gap_floor())
    way_in = planner.leg((x, 0.55), (0.75, 0.45), in_line="work_in")
    assert way_in is not None
    assert way_in.points == [(x, 0.55), pytest.approx((0.75, 0.50)), (0.75, 0.45)]
    way_out = planner.leg((x, 0.40), (1.29, 0.15), out_line="work_out")
    assert way_out is not None
    assert way_out.points[:3] == [(x, 0.40), pytest.approx((0.75, 0.35)), (0.75, 0.15)]
    lost = planner.leg((x, 0.40), (1.29, 0.15))
    assert lost is not None
    assert lost.points[:3] == [(x, 0.40), pytest.approx((0.75, 0.35)), (0.75, 0.15)]
    assert [name for _, _, name in lost.fixed] == ["work_out"]
    docked = planner.leg((x, 0.455), (1.29, 0.15), out_line="work_out")  # report behind the dock
    assert docked is not None
    assert docked.points[:3] == [(x, 0.455), pytest.approx((0.75, 0.40)), (0.75, 0.15)]
    near_end = planner.leg((x, 0.48), (0.75, 0.45), in_line="work_in")  # 0.03 m before the dock
    assert near_end is not None and near_end.points == [(x, 0.48), (0.75, 0.45)]


def test_no_free_path_starts_in_the_gap():
    from fleet_core.traffic.floor.planner import FloorPlanner
    planner = FloorPlanner(_gap_floor())
    assert planner.plan((0.75, 0.50), (0.21, 0.75)) is None
    assert planner.plan((0.77, 0.40), (0.21, 0.75)) is None


def test_a_robot_on_clear_floor_beside_the_way_in_does_not_cut_into_the_gap():
    """0.06 m beside the line just past its start: not straight to the dock (that would enter
    the gap off its centre line) but to the line start first."""
    from fleet_core.traffic.floor.planner import FloorPlanner
    planner = FloorPlanner(_gap_floor())
    route = planner.leg((0.69, 0.72), (0.75, 0.45), in_line="work_in")
    assert route is not None and route.points[-2:] == [(0.75, 0.75), (0.75, 0.45)]
    assert all(p[0] == 0.75 for p in _in_gap(route.points))


def test_a_report_beside_the_mouth_of_the_gap_is_not_taken_for_a_robot_in_it():
    """A robot passing the mouth whose report falls a few cm into the wall margin (clear floor
    is within 0.10 m) steps back onto clear floor as anywhere else; it is not sent through
    the dock."""
    from fleet_core.traffic.floor.planner import FloorPlanner
    planner = FloorPlanner(_gap_floor())
    assert planner._gap_line((0.69, 0.62)) is None
    assert planner._gap_line((0.75, 0.50)) == "work_in"
    route = planner.leg((0.69, 0.62), (0.21, 0.75))
    assert route is not None and not _in_gap(route.points)
    assert not route.fixed


def test_a_robot_past_the_end_of_a_narrow_way_out_is_not_sent_back_into_the_gap():
    from fleet_core.traffic.floor.planner import FloorPlanner
    planner = FloorPlanner(_gap_floor())
    route = planner.leg((0.75, 0.14), (1.29, 0.15), out_line="work_out")
    assert route is not None and not _in_gap(route.points)
    assert route.points[0] == (0.75, 0.14) and route.points[-1] == (1.29, 0.15)


def test_a_robot_on_the_dock_of_a_narrow_line_is_there_already():
    from fleet_core.traffic.floor.planner import FloorPlanner
    planner = FloorPlanner(_gap_floor())
    route = planner.leg((0.75, 0.45), (0.75, 0.45), in_line="work_in")
    assert route is not None and route.points == [(0.75, 0.45)] and route.length == 0.0


def test_from_the_mouth_of_a_gap_a_start_steps_only_onto_clear_cells_beside_the_line():
    """Just inside the mouth (clear floor within 0.10 m) a start steps onto clear floor as
    anywhere else, but only onto clear cells whose centre is within a cell of the narrow
    line, also when the nearest of them are taken. (The next test checks the steps keep off
    the walls, from every point of the mouths.)"""
    from fleet_core.traffic.floor.planner import FloorPlanner
    floor = _gap_floor()
    planner = FloorPlanner(floor)
    start = (0.75, 0.62)
    assert planner._gap_line(start) is None  # the mouth, not deep in the gap
    steps = planner._steps_onto_clear(start)
    assert steps and all(abs(x - 0.75) <= floor.cell_m for x, _ in steps)
    taken = frozenset(floor.cell_of(*xy) for xy in steps[:1])
    later = planner._steps_onto_clear(start, taken)
    assert later and all(abs(x - 0.75) <= floor.cell_m for x, _ in later)
    assert not {floor.cell_of(*xy) for xy in later} & taken
    shut = frozenset(floor.cell_of(*xy) for xy in steps[:4])  # both columns of the line taken
    assert planner._steps_onto_clear(start, shut) == []  # it waits; nothing slantwise instead
    off_floor = planner._steps_onto_clear((0.30, 0.52))  # a wall margin away from any gap
    assert off_floor and any(abs(x - 0.30) > floor.cell_m for x, _ in off_floor)


def mouth_steps(floor, planner, parts=(0.2, 0.5, 0.8)):
    """(start, step) for points (parts of a cell, squared) in every cell beside a narrow line
    that is not deep in the gap, and the starts among them that get no step at all."""
    found, stuck = [], []
    near = {(c[0] + di, c[1] + dj) for c in floor.line_floor
            for di in range(-6, 7) for dj in range(-6, 7)}  # JOIN_M and the mouths, in cells
    for i, j in sorted((floor.free - floor.clear) & near):
        for a in parts:
            for b in parts:
                start = (floor.origin[0] + (i + a) * floor.cell_m,
                         floor.origin[1] + (j + b) * floor.cell_m)
                if planner._near_line(start) is None or planner._gap_line(start):
                    continue
                steps = planner._steps_onto_clear(start)
                found += [(start, step) for step in steps]
                if not steps:
                    stuck.append(start)
    return found, stuck


def nearer_a_wall(floor, start, step) -> float:
    """How much nearer a wall the straight step comes than its start is."""
    along = [(start[0] + (step[0] - start[0]) * k / 20, start[1] + (step[1] - start[1]) * k / 20)
             for k in range(21)]
    limit = 0.12  # the starts are in a wall margin, nearer than this; farther is not looked for
    return floor._gap_to_wall(*start, limit) - min(floor._gap_to_wall(*p, limit) for p in along)


def test_every_point_in_the_mouths_of_a_gap_has_a_step_that_keeps_off_the_walls():
    """No point beside the narrow lines is left without a way back onto clear floor, and no
    step comes more than half a cell (1.5 cm) nearer a wall than the point it starts from."""
    from fleet_core.traffic.floor.planner import FloorPlanner
    floor = _gap_floor()
    found, stuck = mouth_steps(floor, FloorPlanner(floor))
    assert found and not stuck
    assert max(nearer_a_wall(floor, start, step) for start, step in found) <= floor.cell_m / 2

