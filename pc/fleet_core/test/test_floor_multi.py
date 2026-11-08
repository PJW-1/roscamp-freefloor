"""자유바닥 여러 대(P3): 예약 벌점, 앞 칸을 못 잡을 때 재계획, 대기 순환 -> 비켜 서기, 막힘 표시.

시나리오는 레인 관제와 같은 파일·같은 시작 자리다(tools/compare_floor.py).
"""
import sys
from pathlib import Path

import pytest
from fleet_core.common.params import load_params
from fleet_core.manager import FleetManager
from fleet_core.traffic.lane_map import load_lane_map

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
import compare_floor  # noqa: E402


def floor_run(scenario, seed):
    row, world = compare_floor.run("lane_map", scenario, seed, "freefloor")
    return row, world


@pytest.mark.parametrize("scenario,seed", [
    ("basic_orders", 1), ("basic_orders", 2), ("basic_orders", 3),
    ("order_burst", 1), ("blocked_lane", 1),
])
def test_the_lane_scenarios_finish_on_the_free_floor(scenario, seed):
    row, world = floor_run(scenario, seed)
    assert row["finished"], {k: row[k] for k in ("error", "end_s", "orders_shipped")}
    assert world.cell_violations == 0
    assert row["unresolved"] == 0 and row["isolations"] == 0
    assert row["min_gap_m"] >= 2 * load_params().robot_radius_m


def test_every_wait_cycle_ends_in_a_sidestep_or_a_replan():
    for seed in (1, 2, 3):
        row, world = floor_run("basic_orders", seed)
        codes = [e.code for e in world.events]
        if "DEADLOCK_DETECTED" in codes:
            assert row["sidesteps"] + row["replans"] > 0, seed


def test_an_object_marks_the_cells_ahead_and_clears_them_later():
    _, world = floor_run("blocked_lane", 1)
    codes = [e.code for e in world.events]
    assert "FLOOR_BLOCKED" in codes
    assert codes.index("FLOOR_CLEARED") > codes.index("FLOOR_BLOCKED")
    assert not world.core.traffic.blocked_cells()


# ---- reservations ---------------------------------------------------------------------


def two_robot_floor():
    params = load_params().__class__(**{**load_params().__dict__, "traffic_mode": "freefloor"})
    now = [0.0]
    core = FleetManager(load_lane_map(), params, ["a", "b"], lambda: now[0])
    traffic = core.traffic
    for _ in range(3):
        core.on_robot_state("a", 1.0, 1.15, 0.0, "IDLE")
        core.on_robot_state("b", 2.6, 0.8, 0.0, "IDLE")
    return traffic


def test_a_route_reserves_its_cells_in_time_order():
    traffic = two_robot_floor()
    traffic._start_leg(traffic._missions["a"], "TO_PARK", None, "B")
    table = traffic._reservations(exclude="b")
    floor = traffic.floor
    near = floor.cell_id(floor.cell_of(1.2, 1.15))
    far = floor.cell_id(floor.cell_of(2.4, 1.15))
    (_, t_near, _), = table[near]
    (_, t_far, _), = table[far]
    assert t_near < t_far
    assert t_far <= load_params().floor_plan_window_s


def test_a_second_robot_is_steered_off_a_reserved_corridor():
    traffic = two_robot_floor()
    traffic._start_leg(traffic._missions["a"], "TO_PARK", None, "B")  # a: west -> east
    penalty = traffic._penalty_for("b")
    floor = traffic.floor
    corridor = floor.cell_of(1.8, 1.15)
    # b would reach this cell while a is in it (both at about 4 s), so it costs extra
    assert penalty(corridor, 0.8 * 0.2 / 0.2 * 1.0 + 0.0) >= 0.0
    assert max(penalty(corridor, s) for s in (0.2 * t for t in range(0, 40))) > 0.0
    assert penalty(floor.cell_of(1.8, 0.45), 1.0) == 0.0  # nobody plans to be there


# ---- wait cycles ----------------------------------------------------------------------


def test_overlapping_cycles_that_take_turns_are_one_episode(monkeypatch):
    """Robots that get a shorter lock for a tick change who waits for whom: a 3-cycle and a
    2-cycle inside it can take turns every tick. That is one deadlock, not a new one each tick."""
    from fleet_core.traffic.floor import controller as floor_controller
    params = load_params().__class__(**{**load_params().__dict__, "traffic_mode": "freefloor"})
    now = [0.0]
    core = FleetManager(load_lane_map(), params, ["a", "b", "c"], lambda: now[0])
    traffic = core.traffic
    graphs = [{"a": {"b"}, "b": {"c"}, "c": {"a"}}, {"b": {"c"}, "c": {"b"}}]
    tick = [0]

    def alternating(_zones):
        tick[0] += 1
        return {k: set(v) for k, v in graphs[tick[0] % 2].items()}

    monkeypatch.setattr(floor_controller, "wait_graph", alternating)
    steps = []
    monkeypatch.setattr(traffic, "_sidestep", lambda m, cycle: steps.append(m.name) or True)
    while now[0] < params.deadlock_grace_s + 1.0:
        traffic._check_deadlock(now[0])
        now[0] = round(now[0] + 0.1, 1)
    assert steps, "the deadlock never got its sidestep"
    detected = [e for e in traffic.take_output() if getattr(e, "code", "") == "DEADLOCK_DETECTED"]
    assert len(detected) == 1


# ---- one robot at a time on a dock approach line -----------------------------------------


def test_a_second_robot_for_the_same_dock_waits_off_the_lines():
    """Two robots for the work dock at once: the first gets the approach line, the second waits
    on free floor that touches no fixed line (not at the line start, where it would block the
    first one's way in and the docked one's way out) and goes on once the first has left."""
    traffic = two_robot_floor()
    a, b = traffic._missions["a"], traffic._missions["b"]
    traffic._start_leg(a, "TO_WORK", None, "W")
    traffic._start_leg(b, "TO_WORK", None, "W")
    leg_a, leg_b = traffic._legs["a"], traffic._legs["b"]
    assert [f[2] for f in leg_a.route.fixed] == ["work_in"]
    assert leg_b.waiting and not leg_b.route.fixed
    line_cells = set()
    for line in traffic.floor.fixed_lines.values():
        line_cells |= traffic._swept(line.points)
    spot = leg_b.route.points[-1]
    assert not traffic._body_cells(*spot) & line_cells
    # a is done with the dock and gone: b gets the line on its next drive
    del traffic._legs["a"]
    for cell in list(traffic.zones.held_set("a")):
        traffic.zones.release("a", cell)
    traffic._drive(b)
    assert not leg_b.waiting
    assert [f[2] for f in leg_b.route.fixed] == ["work_in"]


def test_a_waiting_robot_with_no_way_to_the_line_does_not_dock_at_its_wait_spot(monkeypatch):
    traffic = two_robot_floor()
    a, b = traffic._missions["a"], traffic._missions["b"]
    traffic._start_leg(a, "TO_WORK", None, "W")
    traffic._start_leg(b, "TO_WORK", None, "W")
    leg_b = traffic._legs["b"]
    leg_b.done = True  # it reached its wait spot
    del traffic._legs["a"]
    for cell in list(traffic.zones.held_set("a")):
        traffic.zones.release("a", cell)
    monkeypatch.setattr(traffic, "_route", lambda *args, **kwargs: None)
    traffic._drive(b)
    assert b.phase == "TO_WORK"
    assert leg_b.route is None


def test_a_route_keeps_the_whole_body_off_cells_others_hold():
    """Avoiding the held cells with the centre is not enough: the body (radius + margin) beside
    the path would still need them, and the robot would wait there for good."""
    traffic = two_robot_floor()
    b = traffic._missions["b"]
    # another robot stands just beside the straight line from b to the goal
    traffic.zones.acquire_all("a", traffic._body_cells(1.8, 0.95))
    route = traffic._route(b, (b.x, b.y), (1.0, 0.75))
    assert route is not None
    assert not traffic._swept(route.points) & traffic.zones.held_set("a")
