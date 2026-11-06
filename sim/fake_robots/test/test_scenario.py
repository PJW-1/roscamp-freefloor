"""시나리오 YAML 읽기와 검증. ROS가 필요 없다."""
from pathlib import Path

import pytest
from conftest import GRID_MAP
from fake_robots.common import load_fleet_params, split_fake_arms
from fake_robots.scenario import ScenarioError, load_scenario, parse_scenario, scenario_map_file
from fleet_core.traffic.lane_map import load_lane_map

SCENARIOS = Path(__file__).parents[1] / "scenarios"
SIX = {"basic_orders", "order_burst", "shared_lane_failure", "stock_shortage", "stuck_robot",
       "operator_recovery", "blocked_lane", "feature_basic_orders", "feature_blocked_lane",
       "auto_isolation", "feature_auto_isolation", "off_lane", "reinstate"}


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
    {"faults": [{"at_s": 5, "robot": "pinky1", "type": "blocked"}],
     "expect": {"orders_shipped_min": 1}},
    {"robots": {"pinky1": {"start": "P-6", "localizing_s": -1.0}}},
    {"faults": [{"at_s": 5, "robot": "pinky1", "type": "drift", "offset_m": 0.3}],
     "expect": {"orders_shipped_min": 1}},
    {"faults": [{"at_s": 5, "robot": "pinky1", "type": "drift", "duration_s": 5}],
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


def _with_reinstate(*extra, basket=None):
    back = {"at_s": 20, "robot": "pinky1", "type": "operator_reinstate", "start": "P-6"}
    if basket is not None:
        back["basket"] = basket
    return minimal(expect={"orders_shipped_min": 1}, faults=[
        {"at_s": 10, "robot": "pinky1", "type": "operator_remove"}, back, *extra])


def test_operator_reinstate_puts_a_removed_robot_back_on_a_waypoint(site):
    sc = parse_scenario(_with_reinstate(), site)
    back = sc.faults[1]
    assert back.type == "operator_reinstate" and back.start == (3, 4)
    assert back.basket == "EMPTY"
    assert parse_scenario(_with_reinstate(basket="NONE"), site).faults[1].basket == "NONE"
    assert sc.gone_robots == set(), "a robot put back is not gone"


@pytest.mark.parametrize("change", [
    {"start": None},
    {"start": "NOPE"},
    {"basket": "FULL"},
    {"at_s": 5},  # 치우기 전
])
def test_a_wrong_reinstate_is_rejected(site, change):
    data = _with_reinstate()
    back = data["faults"][1]
    for key, value in change.items():
        if value is None:
            back.pop(key)
        else:
            back[key] = value
    with pytest.raises(ScenarioError):
        parse_scenario(data, site)


def test_a_stuck_robot_removed_and_put_back_is_not_gone(site):
    """FaultSchedule.stuck과 같은 뜻: 재투입은 그 전의 stuck을 끝낸다."""
    data = _with_reinstate()
    data["faults"].insert(0, {"at_s": 5, "robot": "pinky1", "type": "stuck"})
    assert parse_scenario(data, site).gone_robots == set()
    data["faults"].append({"at_s": 30, "robot": "pinky1", "type": "stuck"})
    assert parse_scenario(data, site).gone_robots == {"pinky1"}


def test_a_robot_removed_again_after_its_reinstate_is_gone(site):
    data = _with_reinstate({"at_s": 30, "robot": "pinky1", "type": "operator_remove"})
    assert parse_scenario(data, site).gone_robots == {"pinky1"}


def test_the_reinstate_scenario_clears_puts_back_and_expects_work_from_the_robot(site):
    sc = load_scenario(SCENARIOS / "reinstate.yaml", site)
    remove = next(f for f in sc.faults if f.type == "operator_remove")
    back = next(f for f in sc.faults if f.type == "operator_reinstate")
    assert back.robot == remove.robot and back.at_s > remove.at_s
    assert sc.expect.orders_shipped == len(sc.orders)
    assert sc.expect.bound_after_reinstate_min >= 1
    assert sc.expect.event_counts.get("ROBOT_REINSTATED") == 1
    assert sc.expect.event_counts.get("ROBOT_NOT_REPORTING") == 0


def shipped():
    return {f.stem: f for f in sorted(SCENARIOS.glob("*.yaml"))}


def site_of(path, default):
    """시나리오가 `map`으로 고른 지도, 없으면 기본 지도."""
    chosen = scenario_map_file(path)
    return load_lane_map(chosen) if chosen else default


def test_exactly_the_six_scenarios_are_shipped(site):
    assert set(shipped()) == SIX
    for f in shipped().values():
        load_scenario(f, site_of(f, site))


def test_a_scenario_can_choose_a_map_from_the_maps_folder():
    assert scenario_map_file(SCENARIOS / "basic_orders.yaml") == ""
    chosen = Path(scenario_map_file(SCENARIOS / "feature_basic_orders.yaml"))
    assert chosen == (SCENARIOS.parent / "maps" / "feature_map.yaml").resolve()
    assert load_scenario(SCENARIOS / "feature_blocked_lane.yaml").robots["pinky1"].start


def test_the_feature_block_scenario_expects_a_detour():
    sc = load_scenario(SCENARIOS / "feature_blocked_lane.yaml")
    assert "DETOUR" in sc.expect.events_include


def test_an_unknown_map_name_is_refused(tmp_path):
    path = tmp_path / "x.yaml"
    path.write_text("map: no_such_map\n", encoding="utf-8")
    with pytest.raises(ScenarioError, match="no_such_map"):
        scenario_map_file(path)


@pytest.mark.parametrize("name", sorted(SIX))
def test_every_robot_starts_in_a_waiting_cell(site, name):
    chosen = site_of(SCENARIOS / f"{name}.yaml", site)
    sc = load_scenario(SCENARIOS / f"{name}.yaml", chosen)
    assert {r.start for r in sc.robots.values()} <= set(chosen.parking)


@pytest.mark.parametrize("name", sorted(SIX))
def test_every_scenario_checks_order_of_work_and_the_arms(site, name):
    path = SCENARIOS / f"{name}.yaml"
    raw = load_scenario(path, site_of(path, site)).expect
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


def test_nav_abort_target_must_be_a_waypoint_and_only_for_nav_abort():
    site = load_lane_map(SCENARIOS.parent / "maps" / "feature_map.yaml")

    def with_fault(fault):
        robots = {"pinky1": {"start": "P-3", "speed_mps": 0.2, "pose_noise_m": 0.0}}
        return minimal(robots=robots, faults=[fault], expect={"orders_shipped": 1})
    sc = parse_scenario(with_fault({"at_s": 0, "robot": "pinky1", "type": "nav_abort",
                                    "target": "W"}), site)
    assert sc.faults[0].target == "W"
    with pytest.raises(ScenarioError, match="target"):
        parse_scenario(with_fault({"at_s": 0, "robot": "pinky1", "type": "nav_abort",
                                   "target": "NOWHERE"}), site)
    with pytest.raises(ScenarioError, match="target"):
        parse_scenario(with_fault({"at_s": 0, "robot": "pinky1", "type": "stuck",
                                   "target": "W"}), site)


def fillable(sc, order):
    """다른 주문은 무시하고 선반이 이 주문에 줄 수 있는 것."""
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


def test_auto_isolation_ships_before_the_operator_and_requeues_once(site):
    sc = load_scenario(SCENARIOS / "auto_isolation.yaml", site)
    fail = next(f for f in sc.faults if f.type == "arm_fail")
    remove = next(f for f in sc.faults if f.type == "operator_remove")
    assert fail.count == 3 and remove.at_s > fail.at_s
    first = sc.orders[0]
    assert all(sc.stock[fail.arm].get(kind, 0) == 0 for kind in first.items), \
        "the first order needs no item of the failing arm"
    assert sc.expect.shipped_before_operator_min >= 1
    assert sc.expect.event_counts["ORDER_REQUEUED"] == 1, "the clear does not requeue again"
    assert sc.expect.event_counts["ROBOT_ISOLATED"] == 1
    assert sc.expect.orders_shipped == len(sc.orders)


def test_feature_auto_isolation_stops_mid_lane_and_ships_without_the_operator():
    sc = load_scenario(SCENARIOS / "feature_auto_isolation.yaml")
    abort = next(f for f in sc.faults if f.type == "nav_abort")
    remove = next(f for f in sc.faults if f.type == "operator_remove")
    assert abort.robot == remove.robot and abort.count == 3 and remove.at_s > abort.at_s
    assert abort.target == "F2", "the abort hits the F1->F2 lane, not a time"
    assert sc.expect.shipped_before_operator_min >= 4 and sc.expect.orders_shipped == len(sc.orders)
    assert "DETOUR" in sc.expect.events_include
    assert sc.expect.detours_around_isolated_min >= 1
    assert sc.expect.event_counts["ROBOT_ISOLATED"] == 1


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
                if load_scenario(path, site_of(path, site)).expect.timed_out}
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


def test_blocked_and_localizing_are_read(site):
    sc = parse_scenario(minimal(
        robots={"pinky1": {"start": "P-6", "localizing_s": 2.0}},
        expect={"orders_shipped_min": 1},
        faults=[{"at_s": 8, "robot": "pinky1", "type": "blocked", "duration_s": 15}]), site)
    (block,) = sc.faults
    assert (block.type, block.at_s, block.duration_s) == ("blocked", 8.0, 15.0)
    assert sc.robots["pinky1"].localizing_s == 2.0
    assert not sc.gone_robots, "a blocked robot goes on"
    assert parse_scenario(minimal(), site).robots["pinky1"].localizing_s == 0.0


def test_blocked_lane_blocks_one_robot_long_enough_to_mark_the_lane(site):
    sc = load_scenario(SCENARIOS / "blocked_lane.yaml", site)
    (block,) = sc.faults
    params = load_fleet_params()
    assert block.type == "blocked"
    assert block.duration_s > params["blocked_mark_s"] + 2.0
    assert block.duration_s < params["blocked_timeout_s"]
    assert any(r.localizing_s > 0 for r in sc.robots.values())
    assert sc.expect.event_counts["LANE_BLOCKED"] == 1
    assert sc.expect.event_counts["LANE_CLEARED"] == 1


def test_drift_fields_are_read(site):
    sc = parse_scenario(minimal(expect={"orders_shipped_min": 1}, faults=[
        {"at_s": 5, "robot": "pinky1", "type": "drift", "duration_s": 8, "offset_m": -0.3}]), site)
    (drift,) = sc.faults
    assert (drift.type, drift.at_s, drift.duration_s, drift.offset_m) == ("drift", 5.0, 8.0, -0.3)
    assert sc.expect.released_while_off_lane == 0
    assert not sc.gone_robots, "a drifting robot comes back"


def test_off_lane_drifts_one_robot_and_expects_it_back_with_nothing_released(site):
    sc = load_scenario(SCENARIOS / "off_lane.yaml", site)
    (drift,) = sc.faults
    assert drift.type == "drift"
    assert abs(drift.offset_m) > load_fleet_params()["lane_error_warn_m"]
    assert drift.duration_s < load_fleet_params()["off_lane_timeout_s"], "it must come back"
    ex = sc.expect
    assert ex.released_while_off_lane == 0 and ex.all_orders_finished
    assert ex.event_counts["LANE_ERROR"] == 1 and ex.event_counts["BACK_ON_LANE"] == 1
    assert ex.event_counts["OFF_LANE"] == 0
