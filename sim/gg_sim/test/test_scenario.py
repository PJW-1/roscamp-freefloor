"""Scenario YAML parsing and validation. No ROS needed."""
from pathlib import Path

import pytest
from conftest import GRID_MAP
from gg_fleet_core.traffic.lane_map import load_lane_map
from gg_sim.common import load_fleet_params, split_fake_arms
from gg_sim.scenario import ScenarioError, load_scenario, parse_scenario

SCENARIOS = Path(__file__).parents[1] / "scenarios"
SIX = {"basic_orders", "order_burst", "shared_lane_failure", "stock_shortage", "stuck_robot",
       "operator_recovery"}


@pytest.fixture(scope="module")
def site():
    return load_lane_map(GRID_MAP)


def minimal(**over):
    data = {
        "seed": 1,
        "speed_factor": 2.0,
        "timeout_s": 100,
        "stock": {"arm1": {"A": 2}, "arm2": {"C": 1}},
        "orders": [{"at_s": 0, "id": "O-1", "items": {"A": 1}}],
        "robots": {"pinky1": {"start": "P-6", "speed_mps": 0.2, "pose_noise_m": 0.0}},
    }
    data.update(over)
    return data


def test_defaults_are_filled(site):
    sc = parse_scenario(minimal(), site)
    assert sc.arm.seconds_per_item == 4.0 and sc.arm.swap_seconds == 8.0
    assert sc.arm.fail_rate == 0.0 and sc.dock.duration_s == 3.0 and sc.faults == []
    assert sc.expect.zone_holder_max == 1 and sc.expect.all_orders_finished is True
    assert sc.expect.orders_shipped is None and sc.expect.orders_shipped_min == 0
    assert sc.expect.fifo_violations == 0 and sc.expect.dual_arm_overlaps == 0
    assert sc.robots["pinky1"].report_delay_s == 0.0 and sc.robots["pinky1"].stop_error_m == 0.0


def test_a_robot_can_report_late_and_stop_off_target(site):
    robots = {"pinky1": {"start": "P-6", "report_delay_s": 0.5, "stop_error_m": 0.02}}
    cfg = parse_scenario(minimal(robots=robots), site).robots["pinky1"]
    assert (cfg.report_delay_s, cfg.stop_error_m) == (0.5, 0.02)


@pytest.mark.parametrize("key", ["report_delay_s", "stop_error_m"])
def test_a_negative_delay_or_stop_error_is_refused(site, key):
    robots = {"pinky1": {"start": "P-6", key: -0.1}}
    with pytest.raises(ScenarioError, match=key):
        parse_scenario(minimal(robots=robots), site)


def test_inventory_defaults_to_the_stock_total(site):
    sc = parse_scenario(minimal(stock={"arm1": {"A": 2, "B": 1}, "arm2": {"A": 3}}), site)
    assert sc.inventory == {"A": 5, "B": 1}


def test_inventory_is_taken_as_written(site):
    sc = parse_scenario(minimal(inventory={"A": 9, "Z": 0}), site)
    assert sc.inventory == {"A": 9, "Z": 0}


def test_orders_are_in_registration_order(site):
    sc = parse_scenario(minimal(orders=[
        {"at_s": 5, "id": "late", "items": {"A": 1}},
        {"at_s": 0, "id": "first", "items": {"A": 1}},
        {"at_s": 0, "id": "second", "items": {"A": 1}},
    ]), site)
    assert sc.order_ids == ["first", "second", "late"]


def test_start_zone_name_and_cell_both_resolve(site):
    by_zone = parse_scenario(minimal(), site).robots["pinky1"].start
    assert by_zone == (3, 4)
    data = minimal()
    data["robots"]["pinky1"]["start"] = [3, 4]
    assert parse_scenario(data, site).robots["pinky1"].start == (3, 4)


def test_multi_cell_zone_is_not_a_valid_start(site):
    data = minimal()
    data["robots"]["pinky1"]["start"] = "Z-R1"
    with pytest.raises(ScenarioError):
        parse_scenario(data, site)


@pytest.mark.parametrize("bad", [
    {"stations": {"X": 1}},
    {"arrivals": []},
    {"stock": {"arm3": {"A": 1}}},
    {"stock": {"arm1": {"A": -1}}},
    {"inventory": {"A": -1}},
    {"orders": [{"at_s": 0, "items": {"A": 1}}]},
    {"orders": [{"at_s": 0, "id": "O-1", "items": {}}]},
    {"orders": [{"at_s": 0, "id": "O-1", "items": {"A": 0}}]},
    {"orders": [{"at_s": -1, "id": "O-1", "items": {"A": 1}}]},
    {"orders": [{"at_s": 0, "id": "O-1", "items": {"A": 1}},
                {"at_s": 0, "id": "O-1", "items": {"A": 1}}]},
    {"faults": [{"at_s": 5, "robot": "pinky9", "type": "stuck"}],
     "expect": {"orders_shipped_min": 1}},
    {"faults": [{"at_s": 5, "robot": "pinky1", "type": "explode"}],
     "expect": {"orders_shipped_min": 1}},
    {"faults": [{"at_s": 5, "arm": "arm3", "type": "arm_fail"}],
     "expect": {"orders_shipped_min": 1}},
    {"faults": [{"at_s": 5, "robot": "pinky9", "type": "operator_remove"}],
     "expect": {"orders_shipped_min": 1}},
    {"expect": {"no_such_check": 1}},
    {"expect": {"all_boxes_delivered": True}},
    {"speed_factor": 0},
])
def test_invalid_scenarios_are_rejected(site, bad):
    with pytest.raises(ScenarioError):
        parse_scenario(minimal(**bad), site)


@pytest.mark.parametrize("expect", [
    {}, {"orders_shipped": 0}, {"orders_shipped_min": 0}, {"orders_partial": 3},
])
def test_a_failure_scenario_cannot_pass_with_nothing_shipped(site, expect):
    faults = [{"at_s": 5, "robot": "pinky1", "type": "stuck"}]
    with pytest.raises(ScenarioError, match="shipped"):
        parse_scenario(minimal(faults=faults, expect=expect), site)


@pytest.mark.parametrize("expect", [{"orders_shipped": 1}, {"orders_shipped_min": 1}])
def test_a_failure_scenario_expecting_shipped_orders_is_fine(site, expect):
    faults = [{"at_s": 5, "robot": "pinky1", "type": "stuck"}]
    assert parse_scenario(minimal(faults=faults, expect=expect), site).faults


def test_fault_fields_for_arms(site):
    sc = parse_scenario(minimal(expect={"orders_shipped_min": 1}, faults=[
        {"at_s": 1, "arm": "arm1", "type": "arm_fail", "count": 3, "after_items": 1},
        {"at_s": 2, "type": "swap_partial", "count": 2},
    ]), site)
    fail, partial = sc.faults
    assert (fail.arm, fail.count, fail.after_items) == ("arm1", 3, 1)
    assert (partial.type, partial.count) == ("swap_partial", 2)


def test_gone_robots_are_the_stuck_and_the_removed(site):
    sc = parse_scenario(minimal(
        robots={r: {"start": s} for r, s in (("pinky1", "P-6"), ("pinky2", "P-5"),
                                             ("pinky3", "P-4"))},
        expect={"orders_shipped_min": 1},
        faults=[{"at_s": 1, "robot": "pinky1", "type": "stuck"},
                {"at_s": 1, "robot": "pinky2", "type": "operator_remove"},
                {"at_s": 1, "robot": "pinky3", "type": "comm_loss", "duration_s": 1}]), site)
    assert sc.gone_robots == {"pinky1", "pinky2"}


def shipped():
    return {f.stem: f for f in sorted(SCENARIOS.glob("*.yaml"))}


def test_exactly_the_six_scenarios_are_shipped(site):
    assert set(shipped()) == SIX
    for f in shipped().values():
        load_scenario(f, site)


@pytest.mark.parametrize("name", sorted(SIX))
def test_every_robot_starts_in_a_waiting_cell(site, name):
    sc = load_scenario(SCENARIOS / f"{name}.yaml", site)
    assert {r.start for r in sc.robots.values()} <= set(site.parking)


@pytest.mark.parametrize("name", sorted(SIX))
def test_every_scenario_checks_order_of_work_and_the_arms(site, name):
    raw = load_scenario(SCENARIOS / f"{name}.yaml", site).expect
    assert raw.fifo_violations == 0 and raw.dual_arm_overlaps == 0
    assert max(raw.orders_shipped or 0, raw.orders_shipped_min) >= 1


def test_basic_orders_is_six_orders_without_faults(site):
    sc = load_scenario(SCENARIOS / "basic_orders.yaml", site)
    assert len(sc.orders) == 6 and not sc.faults and sc.expect.orders_shipped == 6


def test_order_burst_is_eight_at_once_and_more_later(site):
    sc = load_scenario(SCENARIOS / "order_burst.yaml", site)
    at_start = [o for o in sc.orders if o.at_s == 0]
    assert len(at_start) == 8 and len(sc.orders) > 8
    assert sc.expect.direct_returns_min >= 1 and sc.expect.parked_while_backlog == 0


def test_shared_lane_failure_has_both_faults(site):
    sc = load_scenario(SCENARIOS / "shared_lane_failure.yaml", site)
    assert {f.type for f in sc.faults} == {"comm_loss", "nav_abort"}


def fillable(sc, order):
    """What the shelves can give this order, ignoring the other orders."""
    return {k: min(c, sum(s.get(k, 0) for s in sc.stock.values())) for k, c in order.items.items()}


def test_stock_shortage_derivation_matches_the_expected_counts(site):
    sc = load_scenario(SCENARIOS / "stock_shortage.yaml", site)
    capacity = load_fleet_params()["basket_capacity"]
    outcome = {"SHIPPED": 0, "PARTIAL": 0, "HELD": 0}
    for order in sc.orders:
        got = fillable(sc, order)
        if sum(order.items.values()) > capacity or not any(got.values()):
            outcome["HELD"] += 1
        elif got == order.items:
            outcome["SHIPPED"] += 1
        else:
            outcome["PARTIAL"] += 1
    ex = sc.expect
    assert outcome == {"SHIPPED": ex.orders_shipped, "PARTIAL": ex.orders_partial,
                       "HELD": ex.orders_held}
    on_shelf = {k for s in sc.stock.values() for k in s}
    wanted = {k for o in sc.orders for k in o.items}
    gone = wanted - on_shelf
    assert {k for k in gone if sc.inventory.get(k, 0) == 0}, "no OUT_OF_STOCK case"
    assert {k for k in gone if sc.inventory.get(k, 0) > 0}, "no STOCK_MISMATCH case"
    assert {"OUT_OF_STOCK", "STOCK_MISMATCH"} <= set(ex.events_include)
    assert (ex.event_counts["OUT_OF_STOCK"], ex.event_counts["STOCK_MISMATCH"]) == (2, 2)


def test_stuck_scenario_expects_warnings_and_unfinished_orders(site):
    sc = load_scenario(SCENARIOS / "stuck_robot.yaml", site)
    assert sc.expect.all_orders_finished is False and sc.expect.orders_shipped == 2
    assert {"COMM_LOST", "ZONE_WAIT_LONG"} <= set(sc.expect.events_include)


def test_operator_recovery_fails_three_times_then_clears(site):
    sc = load_scenario(SCENARIOS / "operator_recovery.yaml", site)
    fail = next(f for f in sc.faults if f.type == "arm_fail")
    remove = next(f for f in sc.faults if f.type == "operator_remove")
    assert fail.count == 3 and remove.at_s > fail.at_s
    assert sc.expect.orders_shipped == len(sc.orders)
    assert sc.expect.all_orders_finished is True
    assert "ORDER_REQUEUED" in sc.expect.events_include


def test_params_file_gives_the_basket_capacity():
    assert load_fleet_params()["basket_capacity"] >= 1


def test_fake_arms_together_or_not_at_all(site):
    assert split_fake_arms("arm1 arm2 arm3", site) == (True, ["arm1", "arm2"], ["arm3"])
    assert split_fake_arms("arm3", site) == (False, [], ["arm3"])
    assert split_fake_arms("arm1 arm2", site) == (True, ["arm1", "arm2"], [])
    assert split_fake_arms("", site) == (False, [], [])


@pytest.mark.parametrize("bad", ["arm1", "arm2 arm3", "arm9", "arm1 arm2 workcell"])
def test_fake_arms_rejects_half_a_workcell_and_unknown_names(site, bad):
    with pytest.raises(ValueError):
        split_fake_arms(bad, site)


def test_stock_shortage_results_are_tied_to_each_order(site):
    sc = load_scenario(SCENARIOS / "stock_shortage.yaml", site)
    capacity = load_fleet_params()["basket_capacity"]
    derived = {}
    for order in sc.orders:
        got = fillable(sc, order)
        if sum(order.items.values()) > capacity or not any(got.values()):
            derived[order.id] = "HELD"
        else:
            derived[order.id] = "SHIPPED" if got == order.items else "PARTIAL"
    assert sc.expect.order_results == derived


def test_only_the_stuck_scenario_may_run_out_of_time(site):
    declared = {path.stem for path in SCENARIOS.glob("*.yaml")
                if load_scenario(path, site).expect.timed_out}
    assert declared == {"stuck_robot"}


def test_order_results_must_name_a_scenario_order_and_a_known_result(site):
    data = {"robots": {"pinky1": {"start": "P-6"}},
            "orders": [{"id": "O-1", "items": {"A": 1}}]}
    for bad in ({"O-9": "SHIPPED"}, {"O-1": "LOST"}):
        with pytest.raises(ScenarioError):
            parse_scenario({**data, "expect": {"order_results": bad}}, site)


def test_a_slow_cancel_is_an_arm_option(site):
    sc = load_scenario(SCENARIOS / "operator_recovery.yaml", site)
    assert sc.arm.cancel_seconds > 0
    assert parse_scenario({"robots": {"pinky1": {"start": "P-6"}}}, site).arm.cancel_seconds == 0.0
