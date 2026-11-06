"""The two safety numbers of the lane simulation. Pure, no ROS."""
import pytest
from gg_fleet_core.traffic.lane_map import LaneMap
from gg_sim.safety_meter import SafetyMeter


def site():
    return LaneMap({"half_width_m": 0.05, "lane_entry_m": 0.25, "origin_m": [0, 0],
                    "waypoints": [{"id": "a", "x": 0, "y": 0}, {"id": "b", "x": 1, "y": 0}],
                    "lanes": [{"from": "a", "to": "b"}], "stations": {}, "parking": []})


def meter(**kw):
    return SafetyMeter(site(), 0.08, **kw)


def test_robots_apart_count_nothing():
    m = meter()
    m.on_pose("r1", 0.0, 0.0)
    m.on_pose("r2", 1.0, 0.0)
    m.tick(0.0)
    assert m.near == 0
    assert m.min_gap_m == pytest.approx(1.0)


def test_coming_close_counts_once_per_time_for_each_pair():
    m = meter()
    m.on_pose("r1", 0.0, 0.0)
    m.on_pose("r2", 0.1, 0.0)
    m.tick(0.0)
    m.tick(0.1)
    assert m.near == 1
    m.on_pose("r2", 0.5, 0.0)
    m.tick(0.2)
    m.on_pose("r2", 0.1, 0.0)
    m.tick(0.3)
    assert m.near == 2
    assert m.min_gap_m == pytest.approx(0.1)


def test_a_robot_off_the_course_does_not_count():
    m = meter(on_course=lambda robot: robot != "gone")
    m.on_pose("r1", 0.0, 0.0)
    m.on_pose("gone", 0.0, 0.0)
    m.tick(0.0)
    assert m.near == 0


def test_a_release_with_the_body_next_to_the_zone_is_early():
    m = meter()
    m.on_pose("r1", 0.0, 0.0)
    m.on_zones(0.0, {"a": "r1"})
    m.on_pose("r1", 0.07, 0.0)
    m.on_zones(1.0, {"a": ""})
    assert m.early == 1
    assert m.min_release_m == pytest.approx(0.07)


def test_a_release_past_the_radius_is_not_early():
    m = meter()
    m.on_pose("r1", 0.0, 0.0)
    m.on_zones(0.0, {"a": "r1"})
    m.on_pose("r1", 0.2, 0.0)
    m.on_zones(1.0, {"a": ""})
    assert (m.early, m.releases) == (0, 1)


def test_summary_is_plain_data_with_nothing_measured():
    summary = meter().summary()
    assert summary["early"] == 0 and summary["near"] == 0
    assert summary["min_gap_m"] is None and summary["min_release_m"] is None
