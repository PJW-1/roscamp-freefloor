"""자유바닥 경로 계획(P2): A* -> 시선 지름길 -> 원호, 고정 선, Route 계산."""
import math

import pytest
from fleet_core.traffic.floor.cells import line_cells, sample_line
from fleet_core.traffic.floor.floor_map import FloorMap, load_floor_map
from fleet_core.traffic.floor.planner import FloorPlanner, Route, polyline_length


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
