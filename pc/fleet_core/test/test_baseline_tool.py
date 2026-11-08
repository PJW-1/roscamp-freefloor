"""Check tools/baseline.py without ROS: metric extraction, CSV columns, summary, smoke runs."""
import csv
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))

import baseline  # noqa: E402

DT = 0.1


class StubZones:
    def __init__(self):
        self.waiting = {}

    def waiting_for(self, robot):
        return self.waiting.get(robot)


class StubCore:
    def __init__(self, robots):
        self.zones = StubZones()
        self.views = {r: {"state": "MOVING", "path": ["A", "B"]} for r in robots}

    def robot_view(self, robot):
        return self.views[robot]


class StubSite:
    poses = {"A": (0.0, 0.0), "B": (1.0, 0.0), "P": (5.0, 5.0)}

    def waypoint_pose(self, waypoint):
        return self.poses[waypoint]


def robot(x=0.0, y=0.0, **over):
    fields = {"x": x, "y": y, "removed": False, "docked_at": "", "goal_kind": "move",
              "stopped_at": None, "blocked": False}
    return SimpleNamespace(**{**fields, **over})


def stub_world(*names):
    return SimpleNamespace(t=0.0, virtual={n: robot() for n in names}, core=StubCore(names),
                           site=StubSite(), events=[])


def advance(world, meter, **moves):
    """One 0.1 s step: robots named in moves go to the given x, then the meter looks."""
    world.t = round(world.t + DT, 6)
    for name, x in moves.items():
        world.virtual[name].x = x
    meter.observe(world)


# ---- metric extraction ----------------------------------------------------------------------


def test_a_stop_is_moving_then_still_before_the_end_of_the_path():
    world = stub_world("r1")
    meter = baseline.Meter(["r1"], DT)
    meter.observe(world)
    advance(world, meter, r1=0.1)  # moving
    advance(world, meter, r1=0.1)  # still, B is still ahead: a stop
    advance(world, meter, r1=0.1)  # still again: the same stop
    advance(world, meter, r1=0.2)
    advance(world, meter, r1=0.2)  # a second stop
    assert meter.stops == 2


def test_arriving_at_the_last_waypoint_of_the_path_is_not_a_stop():
    world = stub_world("r1")
    meter = baseline.Meter(["r1"], DT)
    meter.observe(world)
    advance(world, meter, r1=0.5)
    advance(world, meter, r1=1.0)
    advance(world, meter, r1=1.0)  # at B, the end of the path
    world.core.views["r1"]["path"] = []
    advance(world, meter, r1=1.0)
    assert meter.stops == 0


def test_a_robot_that_never_moved_has_no_stop():
    world = stub_world("r1")
    meter = baseline.Meter(["r1"], DT)
    for _ in range(5):
        advance(world, meter)
    assert meter.stops == 0


def test_waiting_time_is_the_time_each_robot_holds_a_zone_ticket():
    world = stub_world("r1", "r2")
    meter = baseline.Meter(["r1", "r2"], DT)
    world.core.zones.waiting["r1"] = "Z1"
    for _ in range(3):
        advance(world, meter)
    world.core.zones.waiting["r2"] = "Z2"
    del world.core.zones.waiting["r1"]
    advance(world, meter)
    assert meter.wait == pytest.approx({"r1": 0.3, "r2": 0.1})


def test_object_time_counts_every_still_robot_until_the_blocked_robot_moves_again():
    world = stub_world("r1", "r2", "r3")
    meter = baseline.Meter(["r1", "r2", "r3"], DT)
    meter.observe(world)
    advance(world, meter, r1=0.1, r2=0.1, r3=0.1)
    advance(world, meter)  # nobody met an object yet: no object time
    world.virtual["r1"].stopped_at = world.t  # r1 stops in front of an object
    world.virtual["r3"].docked_at = "WORK"  # r3 works at a station: not because of the object
    advance(world, meter)  # r1 and r2 still
    advance(world, meter, r2=0.2)  # r2 moves, r1 still
    world.virtual["r1"].stopped_at = None
    advance(world, meter)  # object gone, r1 still waits for its goal again: r1 and r2
    advance(world, meter, r1=0.2)  # r1 moves again: the window closes
    advance(world, meter)  # everyone still, but the window is closed
    assert meter.object_still == pytest.approx(0.6)
    assert meter.object_hits == 1


def test_parked_robots_do_not_count_as_standing_because_of_the_object():
    world = stub_world("r1", "r2")
    meter = baseline.Meter(["r1", "r2"], DT)
    meter.observe(world)
    world.core.views["r2"]["state"] = "PARKED"
    world.virtual["r1"].stopped_at = 0.0
    advance(world, meter)
    advance(world, meter)
    assert meter.object_still == pytest.approx(0.2)


def test_a_second_robot_stopped_by_an_object_keeps_its_own_window():
    world = stub_world("r1", "r2", "r3")
    meter = baseline.Meter(["r1", "r2", "r3"], DT)
    world.virtual["r3"].docked_at = "WORK"  # never counted
    meter.observe(world)
    world.virtual["r1"].stopped_at = 0.1
    advance(world, meter)  # r1, r2 still: 0.2
    world.virtual["r2"].stopped_at = 0.2  # r2 meets an object while r1's window is open
    advance(world, meter)  # 0.4
    world.virtual["r2"].stopped_at = None  # r2's object is gone, r2 has not moved yet
    advance(world, meter)  # 0.6
    world.virtual["r1"].stopped_at = None
    advance(world, meter, r1=0.1)  # r1 moves: its window closes, r2 still: 0.7
    advance(world, meter, r1=0.2)  # r2 still stands because of its object: 0.8
    advance(world, meter, r1=0.3, r2=0.1)  # r2 moves: its window closes
    advance(world, meter)  # no window open
    assert meter.object_hits == 2
    assert meter.object_still == pytest.approx(0.8)


def test_overlapping_windows_count_a_still_robot_once_per_step():
    world = stub_world("r1", "r2")
    meter = baseline.Meter(["r1", "r2"], DT)
    meter.observe(world)
    world.virtual["r1"].stopped_at = 0.1
    world.virtual["r2"].stopped_at = 0.1
    advance(world, meter)
    assert meter.object_hits == 2 and meter.object_still == pytest.approx(0.2)


def test_a_robot_entering_error_is_counted_once_per_entry():
    world = stub_world("r1")
    meter = baseline.Meter(["r1"], DT)
    for state in ("MOVING", "ERROR", "ERROR", "OUT_OF_SERVICE", "PARKED", "ERROR"):
        world.core.views["r1"]["state"] = state
        advance(world, meter)
    assert meter.error_entries == 2


def test_removed_robots_are_not_measured():
    world = stub_world("r1")
    meter = baseline.Meter(["r1"], DT)
    meter.observe(world)
    advance(world, meter, r1=0.1)
    world.virtual["r1"].removed = True
    world.core.zones.waiting["r1"] = "Z1"
    advance(world, meter)
    assert meter.stops == 0 and meter.wait == {"r1": 0.0}


# ---- scenario to world ----------------------------------------------------------------------


def test_scenario_settings_reach_the_world():
    data = baseline.load_scenario("blocked_lane")
    world = baseline.build_world(data, "lane_map", seed=3, mode="point")
    assert world.seed == 3
    assert world.params.lane_goal_mode == "point"
    assert world.stock == {"A": 6, "B": 6, "C": 6, "D": 6}
    assert world.arm_s_per_item == 4.0 and world.swap_step_s == 4.0 and world.dock_s == 3.0
    assert [o["id"] for o in world.orders] == [f"O-00{i}" for i in range(1, 7)]
    assert world.faults == [{"at_s": 8.0, "robot": "pinky1", "type": "blocked",
                             "duration_s": 15.0}]
    assert world.robots["pinky3"]["localizing_s"] == 2.0


def test_the_default_plan_only_puts_scenarios_on_maps_that_have_their_start_points():
    for map_name, scenarios in baseline.DEFAULT_PLAN.items():
        site = baseline.load_map(map_name)
        for name in scenarios:
            data = baseline.load_scenario(name)
            for spec in data["robots"].values():
                assert spec["start"] in site.zones, (map_name, name)


def test_missing_arm_and_dock_times_take_the_scenario_parser_defaults():
    data = baseline.load_scenario("basic_orders")
    del data["arm"], data["dock"]
    world = baseline.build_world(data, "lane_map", seed=1, mode="path")
    # fake_robots/scenario.py: 4 s per item, 8 s swap, 3 s dock
    assert (world.arm_s_per_item, world.swap_step_s, world.dock_s) == (4.0, 4.0, 3.0)


def test_a_scenario_whose_central_record_differs_from_the_shelves_is_refused():
    data = {**baseline.load_scenario("basic_orders"), "inventory": {"A": 6, "B": 6, "C": 6}}
    with pytest.raises(ValueError, match="inventory"):
        baseline.build_world(data, "lane_map", seed=1, mode="path")


def test_an_inventory_equal_to_the_summed_stock_is_accepted():
    data = {**baseline.load_scenario("basic_orders"),
            "inventory": {"A": 6, "B": 6, "C": 6, "D": 6}}
    assert baseline.build_world(data, "lane_map", seed=1, mode="path").stock == data["inventory"]


def test_zero_counts_do_not_make_inventory_and_stock_differ():
    data = {**baseline.load_scenario("basic_orders"),
            "inventory": {"A": 6, "B": 6, "C": 6, "D": 6, "E": 0}}
    world = baseline.build_world(data, "lane_map", seed=1, mode="path")
    assert world.stock == {"A": 6, "B": 6, "C": 6, "D": 6}


def test_an_unknown_fault_type_is_refused():
    data = {**baseline.load_scenario("basic_orders"),
            "faults": [{"at_s": 0, "arm": "arm1", "type": "arm_fail", "count": 3}]}
    with pytest.raises(ValueError, match="arm_fail"):
        baseline.build_world(data, "lane_map", seed=1, mode="path")


# ---- CSV and summary ------------------------------------------------------------------------


def row(seed, finish, finished=True, **over):
    base = {"map": "lane_map", "scenario": "basic_orders", "mode": "path", "seed": seed,
            "finished": finished, "error": "", "orders_total": 6, "orders_shipped": 6,
            "finish_s": finish, "wait_total_s": 10.0, "object_still_s": "",
            "object_hits": "", "interventions": 0, "error_entries": 0, "stops": 4,
            "detours": 0, "wait_pinky1_s": 1.0}
    return {**base, **over}


def test_the_csv_has_the_fixed_columns_then_one_wait_column_per_robot(tmp_path):
    path = tmp_path / "out.csv"
    baseline.write_rows(path, [row(1, 100.0), row(2, 110.0, wait_pinky2_s=2.0)])
    with path.open(encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    assert reader.fieldnames == [*baseline.COLUMNS, "wait_pinky1_s", "wait_pinky2_s"]
    assert rows[0]["finish_s"] == "100.0" and rows[0]["wait_pinky2_s"] == ""


def test_the_summary_gives_mean_min_max_over_finished_seeds_and_skips_empty_cells():
    rows = [row(1, 100.0, stops=2), row(2, 110.0, stops=4),
            row(3, "", finished=False, stops=9)]
    summary = {(s["metric"]): s for s in baseline.summarize(rows)}
    finish = summary["finish_s"]
    assert (finish["runs"], finish["n"], finish["mean"], finish["min"], finish["max"]) == (
        3, 2, 105.0, 100.0, 110.0)
    # a run cut by the timeout keeps counting waits and stops, so it is left out
    assert summary["stops"]["n"] == 2 and summary["stops"]["mean"] == 3.0
    assert summary["object_still_s"]["n"] == 0 and summary["object_still_s"]["mean"] == ""
    assert summary["finished"]["n"] == 3
    assert summary["finished"]["mean"] == pytest.approx(2 / 3)


def test_the_summary_keeps_map_scenario_and_mode_apart():
    rows = [row(1, 100.0), row(1, 200.0, mode="point")]
    groups = {(s["mode"], s["metric"]): s["mean"] for s in baseline.summarize(rows)}
    assert groups[("path", "finish_s")] == 100.0 and groups[("point", "finish_s")] == 200.0


# ---- smoke runs on the real core ------------------------------------------------------------


def test_a_basic_run_on_the_default_map_finishes_all_six_orders():
    result = baseline.run_one("lane_map", "basic_orders", seed=1, mode="path")
    assert result["finished"] and result["error"] == ""
    assert result["orders_shipped"] == result["orders_total"] == 6
    assert 0 < result["finish_s"] < 900
    assert result["object_still_s"] == "" and result["interventions"] == 0
    assert result["stops"] >= 0 and result["detours"] == 0
    assert set(result) >= {"wait_pinky1_s", "wait_pinky2_s", "wait_pinky3_s"}
    assert result["wait_total_s"] == pytest.approx(
        sum(result[f"wait_pinky{i}_s"] for i in (1, 2, 3)))


def test_the_same_seed_gives_the_same_row():
    assert (baseline.run_one("lane_map", "basic_orders", seed=2, mode="point")
            == baseline.run_one("lane_map", "basic_orders", seed=2, mode="point"))


def test_a_blocked_lane_run_measures_time_lost_to_the_object():
    result = baseline.run_one("lane_map", "blocked_lane", seed=1, mode="path")
    assert result["finished"]
    assert result["object_hits"] == 1 and result["object_still_s"] >= 15.0


def test_detours_are_counted_from_the_core_events(tmp_path):
    # the settings of test_detour.feature_world: pinky1 meets an object at 42 s for 15 s
    data = baseline.load_scenario("feature_blocked_lane")
    data.update(orders=[{"at_s": 0, "id": f"O-00{i}", "items": {"AB"[i % 2]: 1, "CD"[i % 2]: 1}}
                        for i in range(1, 7)],
                stock={"arm1": {"A": 20, "B": 20}, "arm2": {"C": 20, "D": 20}},
                faults=[{"at_s": 42.0, "robot": "pinky1", "type": "blocked",
                         "duration_s": 15.0}])
    for spec in data["robots"].values():
        spec.pop("localizing_s", None)
    path = tmp_path / "detour.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    result = baseline.run_one("feature_map", str(path), seed=7, mode="path")
    assert result["finished"] and result["scenario"] == "detour"
    assert result["detours"] >= 1


def test_the_reinstate_scenario_counts_both_operator_actions():
    result = baseline.run_one("lane_map", "reinstate", seed=1, mode="path")
    assert result["finished"], result
    assert result["interventions"] == 2


def test_finish_time_is_the_tick_of_the_last_final_report():
    result, world = baseline.run_with_world("lane_map", "basic_orders", seed=1, mode="path")
    last = max(t for t, c in world.commands if type(c).__name__ == "ReportOrderCmd")
    assert result["finish_s"] == pytest.approx(last)


def test_a_run_cut_before_every_robot_parks_is_not_finished():
    full = baseline.run_one("lane_map", "basic_orders", seed=1, mode="path")
    assert full["finished"] and full["end_s"] > full["finish_s"] + 1.0
    cut = baseline.run_one("lane_map", "basic_orders", seed=1, mode="path",
                           timeout_s=full["finish_s"] + 0.5)
    assert not cut["finished"] and cut["error"] == ""
    assert cut["finish_s"] == full["finish_s"]  # the orders did finish
    assert cut["end_s"] == pytest.approx(full["finish_s"] + 0.5)


def no_order_scenario(tmp_path, faults=()):
    # the fakes' parser wants a shipped order whenever there are faults
    data = {**baseline.load_scenario("basic_orders"), "orders": [], "faults": list(faults),
            "expect": {"orders_shipped": 1} if faults else {}}
    path = tmp_path / "empty.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return str(path)


def test_a_scenario_without_orders_finishes_at_zero(tmp_path):
    result = baseline.run_one("lane_map", no_order_scenario(tmp_path), seed=1, mode="path")
    assert result["error"] == "" and result["finished"]
    assert result["finish_s"] == 0.0 and result["orders_total"] == 0


def test_a_run_ends_when_every_robot_has_been_taken_off_the_floor(tmp_path):
    faults = [{"at_s": 1.0, "robot": r, "type": "operator_remove"}
              for r in ("pinky1", "pinky2", "pinky3")]
    result = baseline.run_one("lane_map", no_order_scenario(tmp_path, faults), seed=1,
                              mode="path")
    assert result["finished"] and result["interventions"] == 3
    assert result["end_s"] < 10.0


def test_a_refused_confirm_clear_leaves_the_robot_on_the_floor():
    data = baseline.load_scenario("basic_orders")
    world = baseline.build_world(data, "lane_map", seed=1, mode="path")
    world.step()
    world.core.confirm_clear = lambda *args: None
    operator = baseline.Operator(world, [{"at_s": 0.0, "robot": "pinky1",
                                          "type": "operator_remove"}])
    operator.act()
    assert not world.virtual["pinky1"].removed
    assert (operator.accepted, operator.refused) == (0, 1)
    assert operator.pending  # tried again later


def test_a_refused_reinstate_does_not_move_the_robot():
    data = baseline.load_scenario("basic_orders")
    world = baseline.build_world(data, "lane_map", seed=1, mode="path")
    world.step()
    r = world.virtual["pinky1"]
    before = (r.x, r.y, r.removed)
    operator = baseline.Operator(world, [{"at_s": 0.0, "robot": "pinky1",
                                          "type": "operator_reinstate", "start": "P-3",
                                          "basket": "EMPTY"}])
    operator.act()  # pinky1 was never removed, so the fleet refuses
    assert (r.x, r.y, r.removed) == before
    assert (operator.accepted, operator.refused) == (0, 1)


def test_the_command_line_writes_the_runs_and_the_summary(tmp_path, capsys):
    out = tmp_path / "runs.csv"
    code = baseline.main(["--maps", "lane_map", "--scenarios", "basic_orders", "--modes",
                          "path", "--seeds", "1", "--out", str(out)])
    assert code == 0
    with out.open(encoding="utf-8") as f:
        assert len(list(csv.DictReader(f))) == 1
    summary = tmp_path / "runs_summary.csv"
    with summary.open(encoding="utf-8") as f:
        metrics = {r["metric"] for r in csv.DictReader(f)}
    assert {"finish_s", "wait_total_s", "stops", "detours", "interventions"} <= metrics
    assert "basic_orders" in capsys.readouterr().out
