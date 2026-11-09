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


def test_a_late_sample_still_meets_the_samples_of_its_time():
    """Topics are handled with some delay; pinky2's 0.05 s sample may come after pinky1's 0.30 s."""
    c = check(sync_s=0.1)
    c.on_truth("pinky1", 0.00, 0.20, 0.0, 0.0, 0.0)
    c.on_truth("pinky1", 0.30, 0.90, 0.0, 0.0, 0.0)
    c.on_truth("pinky2", 0.05, 0.0, 0.0, 0.0, 0.0)
    assert c.summary()["min_distance_m"] == pytest.approx(0.20)


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
    c.on_zones({ZONE_W: "pinky1"})
    c.on_truth("pinky1", 1.1, W[0] + 2.0, W[1] + 2.0, 0.0, 0.0)
    c.on_truth("pinky1", 1.2, *W, 0.0, 0.0)
    c.on_truth("pinky1", 1.3, W[0] + 0.01, W[1], 0.0, 0.0)
    s = c.summary()["robots"]["pinky1"]
    assert s["outside_held_samples"] == 1 and s["zone_samples"] == 3
    assert c.summary()["outside_held_samples"] == 1


def test_holding_no_zone_is_counted_apart_from_outside():
    """A robot that waits in a parking cell may hold nothing; that is not 'outside'."""
    c = check()
    c.on_zones({ZONE_W: "pinky2"})
    c.on_truth("pinky1", 1.0, *W, 0.0, 0.0)
    s = c.summary()
    assert s["robots"]["pinky1"]["outside_held_samples"] == 0
    assert s["robots"]["pinky1"]["no_zone_held_samples"] == 1
    assert s["no_zone_held_samples"] == 1 and s["zone_samples"] == 1


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


# ---- free-floor: /fleet/zones holds cell names (c12_17) ----------------------------------


def floor_space():
    from dataclasses import replace

    from fleet_core.common.params import load_params
    from fleet_core.traffic.modes import zone_space
    return zone_space(SITE, replace(load_params(), traffic_mode="freefloor"))


def test_free_floor_cells_count_as_held_zones():
    space = floor_space()
    floor = space.floor
    cell = floor.cell_of(*W)
    middle = floor.centre(cell)  # W sits on a cell corner, where float error is not 0
    c = FleetCheck(space, ROBOTS)
    c.on_zones({floor.cell_id(cell): "pinky1"})
    c.on_truth("pinky1", 1.0, *middle, 0.0, 0.0)
    c.on_truth("pinky1", 1.1, middle[0] + 2.0, middle[1], 0.0, 0.0)
    s = c.summary()["robots"]["pinky1"]
    assert s["outside_held_samples"] == 1 and s["zone_samples"] == 2
    assert s["worst_outside_m"] > 1.8


def test_the_check_space_follows_the_params_file(tmp_path):
    from fleet_gz_sim.fleet_setup import load_zone_space
    assert load_zone_space("", "") is not None and not hasattr(load_zone_space("", ""), "is_cell")
    import yaml
    from fleet_core.common.params import DEFAULT_PARAMS
    params = tmp_path / "fleet_params.yaml"
    values = yaml.safe_load(DEFAULT_PARAMS.read_text(encoding="utf-8"))
    params.write_text(yaml.safe_dump({**values, "traffic_mode": "freefloor"}), encoding="utf-8")
    space = load_zone_space("", str(params))
    assert space.is_cell(space.floor.cell_id(space.floor.cell_of(*W)))
