"""Judge and SafetyMeter fed cell names (free-floor /fleet/zones). ROS-free."""
from dataclasses import replace

import pytest
from conftest import GRID_MAP
from fake_robots.judge import ScenarioJudge
from fake_robots.safety_meter import SafetyMeter
from fake_robots.scenario import parse_scenario
from fleet_core.common.params import load_params
from fleet_core.traffic.floor.floor_map import load_floor_map
from fleet_core.traffic.lane_map import load_lane_map
from fleet_core.traffic.modes import zone_space

SITE = load_lane_map(GRID_MAP)
FLOOR = load_floor_map()
SPACE = zone_space(SITE, replace(load_params(), traffic_mode="freefloor"), FLOOR)
X, Y = SITE.waypoint_pose((9, 1))  # on a lane zone of the grid map
CELL = FLOOR.cell_id(FLOOR.cell_of(X, Y))


def scenario():
    return parse_scenario({
        "seed": 1, "speed_factor": 1.0, "timeout_s": 100,
        "stock": {"arm1": {"A": 9}, "arm2": {"C": 9}},
        "orders": [{"at_s": 0, "id": "O-1", "items": {"A": 1}}],
        "robots": {"pinky1": {"start": "P-5", "speed_mps": 0.2, "pose_noise_m": 0.0},
                   "pinky2": {"start": "P-6", "speed_mps": 0.2, "pose_noise_m": 0.0}},
        "faults": [], "expect": {},
    }, SITE)


def judge():
    return ScenarioJudge(SPACE, scenario(), margin_m=0.13, grace_s=1.0)


def test_judge_release_of_a_cell_far_away_is_fine():
    j = judge()
    j.on_true_pose("pinky1", X + 1.0, Y)
    j.on_zones(0.0, {CELL: "pinky1"})
    j.on_zones(1.0, {CELL: ""})
    assert j.release_violations == 0
    assert j.release_rows[0][3] > 0.3


def test_judge_release_of_a_cell_the_body_is_in_is_a_violation():
    j = judge()
    j.on_true_pose("pinky1", X, Y)
    j.on_zones(0.0, {CELL: "pinky1"})
    j.on_zones(1.0, {CELL: ""})
    assert j.release_violations == 1


def test_judge_ticks_with_cell_holders_without_lane_zone_complaints():
    j = judge()
    j.on_true_pose("pinky1", X, Y)
    j.on_true_pose("pinky2", X + 0.5, Y)
    j.on_zones(0.0, {CELL: "pinky1"})
    for t in (1.0, 5.0):
        j.tick(t)
    assert j.unheld_violations == 0
    assert j.violations == []


def test_judge_still_checks_unheld_lane_zones_in_lanes_mode():
    j = ScenarioJudge(SITE, scenario(), margin_m=0.13, grace_s=1.0)
    j.on_true_pose("pinky1", X, Y)
    j.on_true_pose("pinky2", X + 3, Y)
    for t in (0.0, 5.0, 6.0):
        j.tick(t)
    assert j.unheld_violations == 1


def meter():
    return SafetyMeter(SPACE, 0.08)


def test_meter_cell_release_far_is_not_early():
    m = meter()
    m.on_pose("r1", X + 1.0, Y)
    m.on_zones(0.0, {CELL: "r1"})
    m.on_zones(1.0, {CELL: ""})
    assert m.early == 0
    assert m.releases == 1
    assert m.min_release_m == pytest.approx(SPACE.distance_to_zone(X + 1.0, Y, CELL))


def test_meter_cell_release_with_the_body_inside_is_early_without_half_width():
    m = meter()
    m.on_pose("r1", X, Y)
    m.on_zones(0.0, {CELL: "r1"})
    m.on_zones(1.0, {CELL: ""})
    assert m.early == 1
    assert m.min_release_m == 0.0


# -- occupancy by floor cell ------------------------------------------------------------

def test_judge_two_bodies_in_one_cell_count_two_and_violate():
    j = judge()
    j.on_true_pose("pinky1", X, Y)
    j.on_true_pose("pinky2", X + 0.001, Y)
    j.tick(1.0)
    assert j.max_occupancy == 2
    assert any("at the same time" in v for v in j.violations)


def test_judge_bodies_in_different_cells_count_one():
    j = judge()
    j.on_true_pose("pinky1", X, Y)
    j.on_true_pose("pinky2", X + 0.5, Y)
    j.tick(1.0)
    assert j.max_occupancy == 1
    assert j.violations == []


def test_judge_robots_taken_off_the_map_are_not_counted_as_sharing_a_cell():
    """Fake pinkys taken off the course all wait at the same point outside the map."""
    j = judge()
    j.on_true_pose("pinky1", -5.0, -5.0)
    j.on_true_pose("pinky2", -5.0, -5.0)
    j.tick(1.0)
    assert j.max_occupancy == 0
    assert j.violations == []


# -- sparse snapshot: a missing cell is released ----------------------------------------

def test_judge_cell_missing_from_the_next_message_is_released():
    j = judge()
    j.on_true_pose("pinky1", X, Y)  # body inside the cell: a release must be flagged
    j.on_zones(0.0, {CELL: "pinky1"})
    j.on_zones(1.0, {})
    assert j.holders[CELL] == ""
    assert j.held_by("pinky1") == []
    assert j.release_violations == 1


def test_meter_cell_missing_from_the_next_message_is_released():
    m = meter()
    m.on_pose("r1", X, Y)
    m.on_zones(0.0, {CELL: "r1"})
    m.on_zones(1.0, {})
    assert m.releases == 1
    assert m.early == 1


def test_lanes_keep_a_holder_missing_from_the_message():
    j = ScenarioJudge(SITE, scenario(), margin_m=0.13, grace_s=1.0)
    j.on_zones(0.0, {"Z-R1": "pinky1"})
    j.on_zones(1.0, {})
    assert j.holders["Z-R1"] == "pinky1"


# -- '#blocked' is not a robot ----------------------------------------------------------

def test_judge_blocked_pseudo_holder_release_is_not_checked():
    j = judge()
    j.on_zones(0.0, {CELL: "#blocked"})
    j.on_zones(1.0, {CELL: ""})
    assert j.release_violations == 0
    assert j.violations == []


def test_meter_blocked_pseudo_holder_release_is_not_counted():
    m = meter()
    m.on_pose("#blocked", X, Y)
    m.on_zones(0.0, {CELL: "#blocked"})
    m.on_zones(1.0, {CELL: ""})
    assert m.releases == 0


# -- off course on the floor ------------------------------------------------------------

def test_judge_off_the_lanes_but_on_free_floor_is_on_course():
    j = judge()
    free = sorted(FLOOR.free)
    far = max(free, key=lambda c: min(
        SITE.distance_to_zone(*FLOOR.centre(c), z) for z in set(SITE.zone_of.values())))
    j.on_true_pose("pinky1", *FLOOR.centre(far))
    assert not j.off_course("pinky1")


def test_judge_outside_the_floor_map_is_off_course():
    j = judge()
    x0, y0, _, _ = FLOOR.bounds()
    j.on_true_pose("pinky1", x0 - 1.0, y0 - 1.0)
    assert j.off_course("pinky1")


def test_judge_on_a_blocked_cell_is_off_course():
    j = judge()
    blocked = next(c for c in FLOOR._ids if c not in FLOOR.free)
    j.on_true_pose("pinky1", *FLOOR.centre(blocked))
    assert j.off_course("pinky1")


# -- laps on the free floor ----------------------------------------------------------------
# Free-floor routes are not tied to lanes: a pinky sent to the work station may cross a
# parking zone or wait on it for the dock line. The lap count then follows the manager's task
# for the robot: entering a parking zone counts as parking only when the robot has no station
# task, and reaching the work zone counts as a direct return only when it was sent there.
WAITING = (3, 4)  # P-6 on the grid map
OUT = (9, 3)  # Z-OUT dock
WORK = (5, 3)  # Z-WORK dock
LANE = (9, 1)


def lap(j, robot, cells, start=1.0):
    t = start
    for cell in cells:
        x, y = SITE.waypoint_pose(cell)
        j.on_robot_state(t, robot, x, y, "MOVING")
        j.tick(t)
        t += 1.0


def backlog_judge(space=SPACE):
    j = ScenarioJudge(space, scenario(), margin_m=0.13, grace_s=1.0)
    j.on_board(["O-1", "O-2"])  # two unbound orders, two robots
    return j


def test_floor_crossing_a_parking_zone_on_the_way_to_work_is_not_parking():
    j = backlog_judge()
    j.on_assignments(0.5, {"pinky1": "TO_WORK"})
    lap(j, "pinky1", [OUT, LANE, WAITING, WORK])
    assert j.parked_while_backlog == 0
    assert j.direct_returns == 1


def test_floor_waiting_with_a_backlog_still_counts_as_parking():
    j = backlog_judge()
    j.on_assignments(0.5, {"pinky1": "WAIT"})
    lap(j, "pinky1", [OUT, LANE, WAITING])
    assert j.parked_while_backlog == 1


def test_floor_passing_the_work_zone_on_another_errand_is_no_direct_return():
    j = backlog_judge()
    j.on_assignments(0.5, {"pinky1": "WAIT"})
    lap(j, "pinky1", [OUT, LANE, WORK])
    assert j.direct_returns == 0


def test_floor_without_assignments_falls_back_to_zones():
    j = backlog_judge()
    lap(j, "pinky1", [OUT, LANE, WAITING])
    assert j.parked_while_backlog == 1


def test_lanes_ignore_assignments_for_laps():
    j = backlog_judge(SITE)
    j.on_assignments(0.5, {"pinky1": "TO_WORK"})
    lap(j, "pinky1", [OUT, LANE, WAITING, WORK])
    assert j.parked_while_backlog == 1
    assert j.direct_returns == 0
