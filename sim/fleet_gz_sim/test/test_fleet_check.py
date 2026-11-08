"""fleet_check: closest approach between robots, stops, and positions outside held zones.
No ROS."""
import math

import pytest

lane_map_module = pytest.importorskip("fleet_core.traffic.lane_map")
from fleet_gz_sim.fleet_check import FleetCheck  # noqa: E402

SITE = lane_map_module.load_lane_map()
ROBOTS = ["pinky1", "pinky2", "pinky3"]
W = SITE.waypoint_pose("W")
ZONE_W = SITE.zone_of["W"]


def check(**kw):
    return FleetCheck(SITE, ROBOTS, **kw)


def test_min_distance_uses_samples_taken_at_about_the_same_time():
    c = check()
    c.on_truth("pinky1", 1.00, 0.0, 0.0, 0.0, 0.0)
    c.on_truth("pinky2", 1.02, 0.5, 0.0, 0.0, 0.0)
    c.on_truth("pinky3", 1.03, 3.0, 0.0, 0.0, 0.0)
    c.on_truth("pinky1", 5.00, 0.45, 0.0, 0.0, 0.0)  # pinky2's last sample is 4 s old
    s = c.summary()
    assert s["min_distance_m"] == pytest.approx(0.5)
    assert s["min_distance_pair"] == ["pinky1", "pinky2"]
    assert s["min_distance_at_s"] == pytest.approx(1.02)


def test_min_distance_compares_every_sample_inside_the_sync_window():
    """pinky1 sends two samples before pinky2 sends one; the earlier one is the closer pair."""
    c = check(sync_s=0.1)
    c.on_truth("pinky1", 1.00, 0.30, 0.0, 0.0, 0.0)
    c.on_truth("pinky1", 1.03, 0.60, 0.0, 0.0, 0.0)
    c.on_truth("pinky2", 1.05, 0.0, 0.0, 0.0, 0.0)
    assert c.summary()["min_distance_m"] == pytest.approx(0.30)


def test_no_pair_seen_together_gives_no_distance():
    c = check()
    c.on_truth("pinky1", 1.0, 0.0, 0.0, 0.0, 0.0)
    assert c.summary()["min_distance_m"] is None


def drive(c, robot, t0, t1, speed, dt=0.1, turn=0.0):
    t = t0
    while t < t1 - 1e-9:
        c.on_truth(robot, t, 0.0, 0.0, speed, turn)
        t += dt
    return t


def test_a_stop_is_counted_once_after_moving_and_standing_long_enough():
    c = check(stop_min_s=0.5)
    t = drive(c, "pinky1", 0.0, 2.0, 0.2)
    t = drive(c, "pinky1", t, t + 0.3, 0.0)  # too short
    t = drive(c, "pinky1", t, t + 1.0, 0.2)
    t = drive(c, "pinky1", t, t + 3.0, 0.0)  # one stop, however long
    s = c.summary()["robots"]["pinky1"]
    assert s["stops"] == 1


def test_turning_on_the_spot_is_not_a_stop():
    c = check(stop_min_s=0.5)
    t = drive(c, "pinky1", 0.0, 1.0, 0.2)
    drive(c, "pinky1", t, t + 2.0, 0.0, turn=0.8)
    assert c.summary()["robots"]["pinky1"]["stops"] == 0


def test_standing_from_the_start_is_not_a_stop():
    c = check(stop_min_s=0.5)
    drive(c, "pinky1", 0.0, 5.0, 0.0)
    assert c.summary()["robots"]["pinky1"]["stops"] == 0


def test_stops_while_the_robot_reports_moving_are_counted_apart():
    c = check(stop_min_s=0.5)
    c.on_state("pinky1", "MOVING")
    t = drive(c, "pinky1", 0.0, 1.0, 0.2)
    t = drive(c, "pinky1", t, t + 1.0, 0.0)
    c.on_state("pinky1", "IDLE")
    t = drive(c, "pinky1", t, t + 1.0, 0.2)
    drive(c, "pinky1", t, t + 1.0, 0.0)
    s = c.summary()["robots"]["pinky1"]
    assert s["stops"] == 2 and s["stops_while_moving"] == 1


def test_positions_count_as_outside_only_after_the_fleet_publishes_zones():
    c = check()
    c.on_truth("pinky1", 1.0, *W, 0.0, 0.0)
    assert c.summary()["robots"]["pinky1"]["outside_held_samples"] == 0
    c.on_zones({ZONE_W: "pinky2"})
    c.on_truth("pinky1", 1.1, *W, 0.0, 0.0)
    c.on_zones({ZONE_W: "pinky1"})
    c.on_truth("pinky1", 1.2, *W, 0.0, 0.0)
    c.on_truth("pinky1", 1.3, W[0] + 0.01, W[1], 0.0, 0.0)
    s = c.summary()["robots"]["pinky1"]
    assert s["outside_held_samples"] == 1 and s["zone_samples"] == 3
    assert c.summary()["outside_held_samples"] == 1


def test_far_from_every_held_zone_is_outside():
    c = check()
    c.on_zones({ZONE_W: "pinky1"})
    c.on_truth("pinky1", 1.0, W[0] + 2.0, W[1] + 2.0, 0.0, 0.0)
    assert c.summary()["robots"]["pinky1"]["outside_held_samples"] == 1


def a_point_off_zone_w(lo, hi):
    for r in range(1, 60):
        for k in range(16):
            angle = k * math.pi / 8
            p = (W[0] + 0.01 * r * math.cos(angle), W[1] + 0.01 * r * math.sin(angle))
            if lo < SITE.distance_to_zone(*p, ZONE_W) < hi:
                return p
    raise AssertionError("no such point")


def test_margin_forgives_a_few_centimetres():
    off = a_point_off_zone_w(0.02, 0.08)
    strict, loose = check(margin_m=0.0), check(margin_m=0.10)
    for c in (strict, loose):
        c.on_zones({ZONE_W: "pinky1"})
        c.on_truth("pinky1", 1.0, *off, 0.0, 0.0)
    assert strict.summary()["robots"]["pinky1"]["outside_held_samples"] == 1
    assert loose.summary()["robots"]["pinky1"]["outside_held_samples"] == 0


def test_summary_reports_time_spans_and_real_time_factor():
    c = check()
    c.on_truth("pinky1", 10.0, 0.0, 0.0, 0.0, 0.0, wall=100.0)
    c.on_truth("pinky1", 20.0, 0.0, 0.0, 0.0, 0.0, wall=120.0)
    s = c.summary()
    assert s["sim_span_s"] == pytest.approx(10.0)
    assert s["wall_span_s"] == pytest.approx(20.0)
    assert s["real_time_factor"] == pytest.approx(0.5)


def test_unknown_robots_are_ignored():
    c = check()
    c.on_truth("pinky9", 1.0, 0.0, 0.0, 0.0, 0.0)
    c.on_state("pinky9", "MOVING")
    assert set(c.summary()["robots"]) == set(ROBOTS)
