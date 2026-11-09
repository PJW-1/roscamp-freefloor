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
    j.on_true_pose("pinky2", X, Y)
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
