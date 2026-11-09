"""자유바닥 여러 대(P3): 예약 벌점, 앞 칸을 못 잡을 때 재계획, 대기 순환 -> 비켜 서기, 막힘 표시.

시나리오는 레인 관제와 같은 파일·같은 시작 자리다(tools/compare_floor.py).
"""
import collections
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


def _cycle_spans(monkeypatch):
    """순환마다 (이어진 시간, 순환에 든 뒤 그 로봇들이 한 비켜 서기 + 재계획 수).
    1초 안에 다시 생기면 같은 순환이고, 한 번에 순환 하나만 본다(관제와 같다)."""
    from fleet_core.traffic.deadlock import find_cycle, wait_graph
    from fleet_core.traffic.floor.controller import BLOCK_OWNER
    real, spans = compare_floor.build, []

    def build(*args):
        world = real(*args)
        traffic = world.core.traffic
        acts = collections.Counter()
        replan, sidestep = traffic._replan_if_stuck, traffic._sidestep

        def counted_replan(m, leg, grew):
            before = traffic.metrics["replans"]
            replan(m, leg, grew)
            acts[m.name] += traffic.metrics["replans"] - before

        def counted_sidestep(m, cycle):
            done = sidestep(m, cycle)
            acts[m.name] += bool(done)
            return done

        traffic._replan_if_stuck, traffic._sidestep = counted_replan, counted_sidestep
        check, now = traffic._check_deadlock, {"since": None, "seen": 0.0, "members": set(),
                                                "acted": 0, "last": collections.Counter()}

        def close():
            spans.append((now["seen"] - now["since"], now["acted"]))
            now["since"] = None

        def counted_check(t):  # 관제가 순환을 보는 바로 그때 같은 그래프를 본다
            graph = wait_graph(traffic.zones)
            graph.pop(BLOCK_OWNER, None)
            if now["since"] is not None and t - now["seen"] > 1.0:
                close()
            if now["since"] is not None:  # 지난번 검사 때 이미 순환에 든 로봇의 행동만 센다
                now["acted"] += sum(acts[n] - now["last"][n] for n in now["members"])
            now["last"] = collections.Counter(acts)
            cycle = set(find_cycle(graph) or ())
            if cycle:
                if now["since"] is None:
                    now.update(since=t, members=set(), acted=0)
                now["members"] |= cycle
                now["seen"] = t
            return check(t)

        traffic._check_deadlock = counted_check
        world.close_cycle = lambda: now["since"] is not None and close()
        return world

    monkeypatch.setattr(compare_floor, "build", build)
    return spans


def test_every_wait_cycle_ends_in_a_sidestep_or_a_replan(monkeypatch):
    """순환은 비켜 서기 유예(deadlock_grace_s) 안에 저절로 풀리거나(지나가던 로봇이 비킴)
    그동안 재계획·비켜 서기로 풀린다. 남아 있는 순환은 없다."""
    spans = _cycle_spans(monkeypatch)
    grace = load_params().deadlock_grace_s
    for seed in (1, 2, 3):
        spans.clear()
        row, world = floor_run("basic_orders", seed)
        world.close_cycle()
        codes = [e.code for e in world.events]
        assert row["finished"] and "DEADLOCK_UNRESOLVED" not in codes, seed
        assert bool(spans) == ("DEADLOCK_DETECTED" in codes), (seed, spans)
        for span, acted in spans:
            assert span < grace or acted > 0, (seed, span, acted)


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
    traffic._plan_budget = 99  # these tests plan several legs outside a tick
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


def test_a_floor_wait_cycle_is_a_warning_and_an_unresolved_one_an_error(monkeypatch):
    """Robots on the free floor meet head-on in narrow aisles all the time and step aside, so a
    wait cycle is a warning there. One that cannot be resolved stays an error."""
    from fleet_core.traffic.floor import controller as floor_controller
    params = load_params().__class__(**{**load_params().__dict__, "traffic_mode": "freefloor"})
    now = [0.0]
    traffic = FleetManager(load_lane_map(), params, ["a", "b"], lambda: now[0]).traffic
    monkeypatch.setattr(floor_controller, "wait_graph", lambda _z: {"a": {"b"}, "b": {"a"}})
    monkeypatch.setattr(traffic, "_sidestep", lambda m, cycle: False)
    while now[0] < params.deadlock_grace_s + 1.0:
        traffic._check_deadlock(now[0])
        now[0] = round(now[0] + 0.1, 1)
    levels = {e.code: e.level for e in traffic.take_output() if hasattr(e, "code")}
    assert levels["DEADLOCK_DETECTED"] == "WARN"
    assert levels["DEADLOCK_UNRESOLVED"] == "ERROR"


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


# ---- review fixes (Codex P3) ------------------------------------------------------------


def test_an_isolated_robot_on_its_way_to_a_dock_does_not_hold_the_line_queue():
    traffic = two_robot_floor()
    a, b = traffic._missions["a"], traffic._missions["b"]
    traffic._start_leg(a, "TO_WORK", None, "W")
    traffic._start_leg(b, "TO_WORK", None, "W")
    assert traffic._legs["b"].waiting
    traffic._fail(a, "DEADLOCK_UNRESOLVED")
    for cell in list(traffic.zones.held_set("a")):  # the operator cleared it off the floor
        traffic.zones.release("a", cell)
    traffic._drive(b)
    assert not traffic._legs["b"].waiting


def test_only_robots_in_the_current_cycle_step_aside(monkeypatch):
    from fleet_core.traffic.floor import controller as floor_controller
    params = load_params().__class__(**{**load_params().__dict__, "traffic_mode": "freefloor"})
    now = [0.0]
    core = FleetManager(load_lane_map(), params, ["a", "b", "c"], lambda: now[0])
    traffic = core.traffic
    graphs = [{"a": {"b"}, "b": {"a"}}, {"b": {"c"}, "c": {"b"}}]
    tick = [0]

    def alternating(_zones):
        tick[0] += 1
        return {k: set(v) for k, v in graphs[tick[0] % 2].items()}

    monkeypatch.setattr(floor_controller, "wait_graph", alternating)
    steps = []
    monkeypatch.setattr(traffic, "_sidestep", lambda m, cycle: steps.append(
        (m.name, set(cycle), set(graphs[tick[0] % 2]))) or True)
    while now[0] < params.deadlock_grace_s + 1.0:
        traffic._check_deadlock(now[0])
        now[0] = round(now[0] + 0.1, 1)
    assert steps
    for name, cycle, current in steps:
        assert name in current and cycle == current


def test_a_sidestep_elsewhere_does_not_postpone_an_unresolved_cycle(monkeypatch):
    from fleet_core.traffic.floor import controller as floor_controller
    from fleet_core.traffic.floor.controller import _Leg
    params = load_params().__class__(**{**load_params().__dict__, "traffic_mode": "freefloor"})
    now = [0.0]
    core = FleetManager(load_lane_map(), params, ["a", "b", "c"], lambda: now[0])
    traffic = core.traffic
    monkeypatch.setattr(floor_controller, "wait_graph", lambda _z: {"a": {"b"}, "b": {"a"}})
    monkeypatch.setattr(traffic, "_sidestep", lambda m, cycle: True)
    traffic._legs["c"] = _Leg(None, (0.0, 0.0), resume=("TO_PARK", None, "P-3"))  # stuck aside
    end = 2 * params.deadlock_grace_s + floor_controller.SIDESTEP_HOLD_S + 1.0
    while now[0] < end:
        traffic._check_deadlock(now[0])
        now[0] = round(now[0] + 0.1, 1)
    codes = [getattr(e, "code", "") for e in traffic.take_output()]
    assert "DEADLOCK_UNRESOLVED" in codes


def test_cells_ahead_of_an_object_stay_with_the_robot_until_its_goal_has_ended():
    """The robot may still follow the cancelled MoveTo until the cancel result: the cells of that
    path stay its own; they go to the blocked mark only after the goal has ended."""
    traffic = two_robot_floor()
    b = traffic._missions["b"]
    traffic._start_leg(b, "TO_PARK", None, "P-3")
    traffic._drive(b)
    goal = b.goal
    assert goal is not None
    sent = set(traffic.zones.held_set("b"))
    b.robot_state = "BLOCKED"
    traffic._mark_blocked(b)
    assert sent <= traffic.zones.held_set("b")
    # BLOCKED is a pause on the goal: until the cancel result the robot may still follow it
    for _ in range(3):
        traffic._follow(b)
        traffic._note_block_report(b)
    assert sent <= traffic.zones.held_set("b")
    traffic.on_move_result("b", goal.id, False, "CANCELED")
    for _ in range(3):
        traffic._note_block_report(b)
    assert traffic.blocked_cells()


def test_a_robot_out_of_contact_keeps_its_turn_on_the_approach_line():
    traffic = two_robot_floor()
    a, b = traffic._missions["a"], traffic._missions["b"]
    traffic._start_leg(a, "TO_WORK", None, "W")
    traffic._start_leg(b, "TO_WORK", None, "W")
    for cell in list(traffic.zones.held_set("a")):  # still far from the line
        traffic.zones.release("a", cell)
    a.comm = "COMM_LOST"
    traffic._drive(b)
    assert traffic._legs["b"].waiting


def test_only_the_robot_found_unresolved_is_isolated(monkeypatch):
    """The episode goes on while the cycle shrinks inside it; the robot isolated at the end is
    one that was found unable to step aside, not a newcomer to the cycle."""
    from fleet_core.traffic.floor import controller as floor_controller
    params = load_params().__class__(**{**load_params().__dict__, "traffic_mode": "freefloor"})
    now = [0.0]
    core = FleetManager(load_lane_map(), params, ["a", "b", "c"], lambda: now[0])
    traffic = core.traffic
    graph = [{"a": {"b"}, "b": {"c"}, "c": {"a"}}]
    monkeypatch.setattr(floor_controller, "wait_graph", lambda _z: {k: set(v) for k, v in
                                                                    graph[0].items()})
    monkeypatch.setattr(traffic, "_sidestep", lambda m, cycle: False)
    failed = []
    found = []
    monkeypatch.setattr(traffic, "_fail",
                        lambda m, reason="": failed.append((m.name, list(found))))
    while now[0] < params.deadlock_grace_s + params.deadlock_unresolved_s * 3:
        traffic._check_deadlock(now[0])
        for e in traffic.take_output():
            if getattr(e, "code", "") == "DEADLOCK_UNRESOLVED":
                found.append(e.robot)
                if len(found) == 1:  # the cycle now leaves out the robot just found
                    rest = sorted({"a", "b", "c"} - {e.robot})
                    graph[0] = {rest[0]: {rest[1]}, rest[1]: {rest[0]}}
        now[0] = round(now[0] + 0.1, 1)
    assert failed
    for name, found_before in failed:
        assert name in found_before[1:], (name, found_before)


def test_a_robot_told_to_wait_where_it_stands_stops_its_old_goal():
    """A robot already in a good wait spot gets no new MoveTo; the goal of its previous leg must
    end, or it drives on to the old target (here, onto the approach line) while 'waiting'."""
    from fleet_core.common.commands import CancelCmd
    traffic = two_robot_floor()
    a, b = traffic._missions["a"], traffic._missions["b"]
    traffic._start_leg(a, "TO_WORK", None, "W")  # a has the line
    for _ in range(3):
        traffic.on_robot_state("b", 0.9, 2.05, 0.0, "IDLE")
    traffic._start_leg(b, "TO_PARK", None, "P-3")
    traffic._drive(b)
    assert b.goal is not None
    traffic.take_output()
    traffic._start_leg(b, "TO_WORK", None, "W")  # b waits where it is
    leg = traffic._legs["b"]
    assert leg.waiting and leg.route.length == 0.0
    assert b.goal is None
    assert any(isinstance(c, CancelCmd) for c in traffic.take_output())


def test_one_route_is_planned_per_tick():
    """A plan can take ~10 ms on this floor; planning every robot in one tick went over 20 ms.
    The second robot gets its route on the next tick."""
    traffic = two_robot_floor()
    a, b = traffic._missions["a"], traffic._missions["b"]
    clock = traffic._clock
    traffic.tick_progress(clock())
    traffic._start_leg(a, "TO_PARK", None, "B")
    traffic._start_leg(b, "TO_PARK", None, "P-3")
    assert traffic._legs["a"].route is not None
    assert traffic._legs["b"].route is None
    traffic.tick_progress(clock())
    traffic._drive(b)
    assert traffic._legs["b"].route is not None


def test_a_deferred_plan_stops_the_old_goal_and_claims_no_line():
    traffic = two_robot_floor()
    a, b = traffic._missions["a"], traffic._missions["b"]
    traffic._start_leg(b, "TO_PARK", None, "P-3")
    traffic._drive(b)
    assert b.goal is not None
    traffic.tick_progress(traffic._clock())  # budget 1
    traffic._start_leg(a, "TO_PARK", None, "B")  # uses it
    traffic._start_leg(b, "TO_WORK", None, "W")  # deferred
    leg = traffic._legs["b"]
    assert leg.route is None
    assert b.goal is None  # not driving on to the old target meanwhile
    assert leg.waiting  # and not counted as going up the approach line
