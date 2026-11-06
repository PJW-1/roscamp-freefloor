"""자동 격리(지시서 5.5): 로봇 하나가 ERROR가 되면 구역은 쥐고 주문은 바로 대기 맨 앞으로 간다.

사람은 물리적으로 치운 뒤 confirm_clear로 구역만 푼다.
"""
from core_helpers import cmds, codes
from fleet_core.common.commands import DispatchCmd, OrderRecord, RobotIsolated
from fleet_core.common.params import load_params
from fleet_core.manager import FleetManager
from fleet_core.orders.orchestrator import OrderOrchestrator
from fleet_core.traffic.controller import TrafficController
from fleet_core.traffic.lane_map import load_lane_map
from grid_map import grid_site
from sim_world import SimWorld

O1 = OrderRecord("O-001", {"A": 1, "C": 1})
O2 = OrderRecord("O-002", {"B": 1})


def order(name, at_s=0.0, **items):
    return {"at_s": at_s, "id": name, "items": items}


def two_robots(first="P-6", second="Z-R3"):
    return {"pinky1": {"start": first, "speed_mps": 0.20, "pose_noise_m": 0.03},
            "pinky2": {"start": second, "speed_mps": 0.20, "pose_noise_m": 0.03}}


def isolated(events):
    return [e for e in cmds(events, "EventOut") if e.code == "ROBOT_ISOLATED"]


# ---- traffic 혼자 --------------------------------------------------------------------------


def placed_traffic(robots=("a", "b"), at=("P-5", "P-6")):
    site = load_lane_map()
    core = TrafficController(site, load_params(), list(robots), lambda: 0.0)
    for robot, waypoint in zip(robots, at, strict=True):
        x, y = site.waypoint_pose(waypoint)
        for _ in range(core.params.release_confirm_samples):
            core.on_robot_state(robot, x, y, 0.0, "IDLE")
    core.take_output()
    return core


def test_an_error_isolates_the_robot_once_and_keeps_its_zones():
    core = placed_traffic()
    held = core.zones.held_by("a")
    core.on_robot_fault("a", "GRIPPER")
    out = core.take_output()
    assert [(c.robot, c.reason) for c in out if isinstance(c, RobotIsolated)] == [("a", "GRIPPER")]
    events = isolated(out)
    assert len(events) == 1 and events[0].level == "ERROR" and events[0].robot == "a"
    assert "GRIPPER" in events[0].detail
    assert core.zones.held_by("a") == held, "it is still there"
    core.on_robot_fault("a", "again")
    again = core.take_output()
    assert not [c for c in again if isinstance(c, RobotIsolated)]
    assert not isolated(again), "once per fault"


def test_a_robot_that_was_never_placed_is_not_isolated():
    """PLACEMENT_CONFLICT는 구역도 주문도 없고, 놓이면 스스로 풀린다."""
    site = load_lane_map()
    core = TrafficController(site, load_params(), ["a", "b"], lambda: 0.0)
    x, y = site.waypoint_pose("A")
    core.on_robot_state("a", x, y, 0.0, "IDLE")
    lane = site.lane("N", "S")
    core.on_robot_state("b", lane.ax + lane.ux * 0.3, lane.ay + lane.uy * 0.3, 0.0, "IDLE")
    out = core.take_output()
    assert "PLACEMENT_CONFLICT" in codes(out)
    assert not isolated(out)
    assert not [c for c in out if isinstance(c, RobotIsolated)]


def test_a_recovery_conflict_isolates_and_fails_the_named_dispatch():
    core = placed_traffic()
    core.on_dispatch(7, "OUTBOUND", "a")
    core.take_output()
    m = core._missions["a"]
    m.comm = "RECOVERING"
    core.zones.release("a", core.zones.held_by("a")[0])
    core.zones.force_acquire("b", core._zone(core.site.nearest_waypoint(m.x, m.y)))
    core._finish_recovery(m)
    out = core.take_output()
    assert "RECOVERY_CONFLICT" in codes(out)
    assert [c.robot for c in out if isinstance(c, RobotIsolated)] == ["a"]
    results = cmds(out, "DispatchResult")
    assert [(r.goal_id, r.success) for r in results] == [(7, False)]


# ---- 주문 쪽 혼자 ---------------------------------------------------------------------------


def orchestrator(orders=(O1, O2)):
    now = [0.0]
    core = OrderOrchestrator(grid_site().stations, load_params(), ["r1", "r2"], lambda: now[0])
    core.on_order_board("RUN", list(orders))
    core.take_output()
    return core, now


def bind(core, robot="r1"):
    core.on_dispatch_result(core._open_calls[0], True, robot)
    core.take_output()


def test_isolation_puts_the_order_first_with_what_the_basket_held():
    core, _ = orchestrator()
    bind(core)
    core.step("r1")
    fill = cmds(core.take_output(), "FillCmd")[0]
    core.on_fill_result(fill.goal_id, True, {"A": 1, "C": 1}, {})
    core.step("r1")
    core.take_output()
    core.on_robot_isolated("r1", "GOAL_FAILED")
    out = core.take_output()
    requeued = [e for e in cmds(out, "EventOut") if e.code == "ORDER_REQUEUED"]
    assert len(requeued) == 1
    assert requeued[0].detail.startswith("O-001")
    assert "'A': 1" in requeued[0].detail and "GOAL_FAILED" in requeued[0].detail
    assert [c for c in out if isinstance(c, DispatchCmd) and not c.robot_id], "a call for it"
    assert core.order_of("r1") == ""
    assert core.robot_view("r1")["basket"] == "NONE"
    assert [o.order_id for o in core._pending()] == ["O-001", "O-002"], "first in line"
    core.on_robot_removed("r1")
    assert "ORDER_REQUEUED" not in codes(core.take_output()), "the clear only frees zones"


def test_isolation_cancels_a_running_arm_job():
    core, _ = orchestrator()
    bind(core)
    core.step("r1")
    fill = cmds(core.take_output(), "FillCmd")[0]
    core.on_robot_isolated("r1", "RECOVERY_CONFLICT")
    out = core.take_output()
    assert [(c.robot, c.goal_id) for c in cmds(out, "CancelCmd")] == [("workcell", fill.goal_id)]
    assert "ORDER_REQUEUED" in codes(out)
    core.on_fill_result(fill.goal_id, False, {"A": 1}, {}, "CANCELED")
    assert "ARM_LATE_RESULT" in codes(core.take_output()), "something went into that basket"


def test_an_arm_with_no_result_keeps_the_order_bound_until_the_clear():
    """결과 없는 팔은 일을 끝냈을 수 있다. 되돌리면 두 번 출고될 수 있어 운영자를 기다린다."""
    core, now = orchestrator()
    bind(core)
    core.step("r1")
    core.take_output()
    now[0] += core.params.fill_timeout_s + 0.1
    core.check_arm_timeouts(now[0])
    now[0] += core.params.arm_cancel_wait_s + 0.1
    core.check_arm_timeouts(now[0])
    core.take_output()
    core.on_robot_isolated("r1", "workcell goal never reported")
    assert "ORDER_REQUEUED" not in codes(core.take_output())
    assert core.order_of("r1") == "O-001"
    core.on_robot_removed("r1")
    assert "ORDER_REQUEUED" in codes(core.take_output())


# ---- 둘을 이어서 ----------------------------------------------------------------------------


def report(core, robot, cell):
    x, y = core.site.waypoint_pose(cell)
    for _ in range(core.params.release_confirm_samples):
        core.on_robot_state(robot, x, y, 0.0, "IDLE")


def drive(core, robot, until, limit=300):
    seen = []
    for _ in range(limit):
        seen += core.tick()
        m = core.traffic._missions[robot]
        goal = m.goal
        if goal is not None and goal.kind == "move":
            report(core, robot, m.path[goal.target_idx])
            core.on_move_result(robot, goal.id, True, "")
        elif goal is not None:
            core.on_dock_result(robot, goal.id, True)
        report(core, robot, m.path[m.idx])
        if until():
            return seen + core.tick()
    raise AssertionError(f"not reached: {core.robot_view(robot)}")


def test_a_full_basket_failing_on_the_way_out_goes_back_without_the_operator():
    now = [0.0]
    core = FleetManager(grid_site(), load_params(), ["r1"], lambda: now[0])
    report(core, "r1", (4, 3))
    core.on_order_board("RUN", [O1, O2])
    out = drive(core, "r1", lambda: core.robot_view("r1")["order"] == "O-001")
    fill = cmds(out, "FillCmd")[0]
    core.on_fill_result(fill.goal_id, True, {"A": 1, "C": 1}, {})
    seen = []
    for _ in range(100):
        seen += core.tick()
        m = core.traffic._missions["r1"]
        if m.fault:
            break
        if m.goal is not None and m.goal.kind == "move":
            core.on_move_result("r1", m.goal.id, False, "ABORTED")
        now[0] += core.params.move_retry_wait_s + 0.1
        report(core, "r1", m.path[m.idx])
    seen += core.tick()
    assert "GOAL_FAILED" in codes(seen)
    assert len(isolated(seen)) == 1
    requeued = [e for e in cmds(seen, "EventOut") if e.code == "ORDER_REQUEUED"]
    assert len(requeued) == 1 and requeued[0].detail.startswith("O-001")
    assert "'C': 1" in requeued[0].detail
    view = core.robot_view("r1")
    assert (view["state"], view["order"], view["basket"]) == ("ERROR", None, "NONE")
    assert core.zones.held_by("r1"), "zones stay with it"
    assert not [c for c in seen if isinstance(c, RobotIsolated)], "it stays inside"
    held = core.zones.held_by("r1")
    core.confirm_clear("r1", held, "lifted off")
    after = core.tick()
    assert "ZONES_CLEARED" in codes(after)
    assert "ORDER_REQUEUED" not in codes(after)


def test_arm_failing_three_times_isolates_and_the_other_robot_finishes_after_the_clear():
    """operator_recovery와 같은 고장. 주문은 사람을 기다리지 않고 바로 맨 앞으로 간다."""
    orders = [order("O-001", A=2, C=2), order("O-002", B=1), order("O-003", D=1)]
    world = SimWorld(orders=orders, robots=two_robots("P-6", "P-5"),
                     faults=[{"at_s": 0.0, "type": "fill_fail", "count": 3, "place": 1}])
    assert world.run_until(lambda: world.state("pinky1") == "ERROR", 200)
    world.step()
    assert "FILL_FAILED" in world.event_codes("ERROR")
    assert world.event_codes("ERROR").count("ROBOT_ISOLATED") == 1
    requeued = [e for e in world.events if e.code == "ORDER_REQUEUED"]
    assert len(requeued) == 1 and requeued[0].detail.startswith("O-001")
    assert "basket held {'A': 2, 'C': 1}" in requeued[0].detail, "what stays in its basket"
    assert world.core.robot_view("pinky1")["order"] is None
    assert world.core.robot_view("pinky1")["basket"] == "NONE"
    held = world.core.zones.held_by("pinky1")
    assert held
    world.run(20)
    assert world.core.zones.held_by("pinky1") == held

    world.confirm_clear("pinky1", held, "took it off")
    world.step()
    assert [e.code for e in world.events].count("ORDER_REQUEUED") == 1, "not twice"
    assert world.run_until(lambda: world.all_orders_final() and world.all_parked(["pinky2"]))
    assert world.report_statuses() == {o["id"]: "SHIPPED" for o in orders}
    assert world.shipped_order() == ["O-001", "O-002", "O-003"], "it keeps its place"
    assert {r.robot for r in world.reports} == {"pinky2"}


def test_the_rest_ships_while_the_isolated_robot_waits_for_the_operator():
    """pinky1이 출고하러 간 사이 pinky2가 채움에서 격리된다. 출고는 사람을 기다리지 않는다."""
    orders = [order("O-001", C=1), order("O-002", A=1), order("O-003", D=1)]
    world = SimWorld(orders=orders, robots=two_robots("P-6", "P-5"), swap_step_s=10.0)
    assert world.run_until(lambda: world.core.robot_view("pinky1")["basket"] == "FULL", 300)
    world.arm_faults["fill_fail"] = [0, 0, 0]
    assert world.run_until(lambda: "ROBOT_ISOLATED" in world.event_codes(), 300)
    assert world.core.shipped == 0
    assert world.run_until(lambda: world.core.shipped >= 1, 300)
    assert world.reports[0].order_id == "O-001"
    assert "ZONES_CLEARED" not in world.event_codes(), "no operator yet"


def test_sixty_seconds_blocked_isolates_and_holds_the_body_zones():
    from test_lane_blocked import MARK_S, PARAMS, Pilot, blocked_on_the_way, first_goal

    pilot = Pilot()
    blocked_on_the_way(pilot)
    goal = first_goal(pilot)
    pilot.run(MARK_S + 0.2)
    pilot.core.on_move_result("pinky1", goal, False, "CANCELED")
    pilot.run(PARAMS.blocked_timeout_s - MARK_S + 0.5)
    events = pilot.events("ROBOT_ISOLATED")
    assert len(events) == 1 and "BLOCKED" in events[0][1].detail
    assert pilot.events("BLOCKED_TOO_LONG")
    assert "N" in pilot.held()


# ---- 리뷰 수정(9c65b9f) -----------------------------------------------------------------------


def swapping_then_isolated(orders=(O1, O2)):
    """r1이 O-001을 담아 출고 도크에서 교환하는 중에 격리된다. 교환 goal과 그때 출력."""
    core, now = orchestrator(orders)
    bind(core)
    core.step("r1")
    fill = cmds(core.take_output(), "FillCmd")[0]
    core.on_fill_result(fill.goal_id, True, {"A": 1, "C": 1}, {})
    core.step("r1")
    goal = [c for c in core.take_output() if isinstance(c, DispatchCmd) and c.robot_id][0]
    core.on_dispatch_result(goal.goal_id, True, "r1")
    core.step("r1")
    swap = cmds(core.take_output(), "SwapCmd")[0]
    core.on_robot_isolated("r1", "RECOVERY_CONFLICT")
    return core, now, swap, core.take_output()


def test_a_swap_in_progress_holds_the_order_and_a_late_take_ships_it_once():
    core, _, swap, out = swapping_then_isolated()
    assert [(c.robot, c.goal_id) for c in cmds(out, "CancelCmd")] == [("arm3", swap.goal_id)]
    assert "ORDER_REQUEUED" not in codes(out), "the arm may already have the basket"
    assert core.order_of("r1") == "O-001"
    core.on_swap_result(swap.goal_id, False, True, False, "CANCELED")
    out = core.take_output()
    assert [(r.order_id, r.status) for r in cmds(out, "ReportOrderCmd")] == [("O-001", "SHIPPED")]
    assert "ORDER_REQUEUED" not in codes(out)
    assert core.shipped == 1 and core.order_of("r1") == ""
    assert core.robot_view("r1")["basket"] == "NONE"
    assert [o.order_id for o in core._pending()] == ["O-002"]
    assert not cmds(out, "ReleaseCmd"), "an isolated robot stays where it is"
    core.on_robot_removed("r1")
    assert "ORDER_REQUEUED" not in codes(core.take_output())


def test_a_swap_that_did_not_take_the_basket_requeues_then():
    core, _, swap, _ = swapping_then_isolated()
    core.on_swap_result(swap.goal_id, False, False, False, "CANCELED")
    out = core.take_output()
    requeued = [e for e in cmds(out, "EventOut") if e.code == "ORDER_REQUEUED"]
    assert len(requeued) == 1 and requeued[0].detail.startswith("O-001")
    assert not cmds(out, "ReportOrderCmd")
    assert core.order_of("r1") == "" and core.robot_view("r1")["basket"] == "NONE"
    assert [o.order_id for o in core._pending()] == ["O-001", "O-002"]
    assert len(open_calls_in(out)) == 1, "a call for the order that came back"
    core.on_robot_removed("r1")
    assert "ORDER_REQUEUED" not in codes(core.take_output())


def test_a_swap_with_no_result_waits_for_a_late_result_or_the_clear():
    core, now, swap, _ = swapping_then_isolated()
    now[0] += core.params.arm_cancel_wait_s + 0.1
    core.check_arm_timeouts(now[0])
    out = core.take_output()
    assert "ARM_NO_RESULT" in codes(out)
    assert "ORDER_REQUEUED" not in codes(out)
    assert core.order_of("r1") == "O-001"
    core.on_robot_removed("r1")
    assert codes(core.take_output()).count("ORDER_REQUEUED") == 1


def test_a_swap_with_no_result_still_ships_on_a_late_take():
    core, now, swap, _ = swapping_then_isolated()
    now[0] += core.params.arm_cancel_wait_s + 0.1
    core.check_arm_timeouts(now[0])
    core.take_output()
    core.on_swap_result(swap.goal_id, False, True, False, "CANCELED")
    out = core.take_output()
    assert [r.status for r in cmds(out, "ReportOrderCmd")] == ["SHIPPED"]
    core.on_robot_removed("r1")
    assert "ORDER_REQUEUED" not in codes(core.take_output())


def test_a_fill_in_progress_still_requeues_at_once_and_late_items_never_ship():
    core, _ = orchestrator()
    bind(core)
    core.step("r1")
    fill = cmds(core.take_output(), "FillCmd")[0]
    core.on_robot_isolated("r1", "RECOVERY_CONFLICT")
    assert "ORDER_REQUEUED" in codes(core.take_output())
    core.on_fill_result(fill.goal_id, True, {"A": 1, "C": 1}, {})
    out = core.take_output()
    assert "ARM_LATE_RESULT" in codes(out)
    assert not cmds(out, "ReportOrderCmd") and core.shipped == 0


def test_a_swap_in_progress_isolates_the_robot_only_once_through_the_manager():
    now = [0.0]
    core = FleetManager(grid_site(), load_params(), ["r1"], lambda: now[0])
    report(core, "r1", (4, 3))
    core.on_order_board("RUN", [O1])
    out = drive(core, "r1", lambda: core.robot_view("r1")["order"] == "O-001")
    core.on_fill_result(cmds(out, "FillCmd")[0].goal_id, True, {"A": 1, "C": 1}, {})
    out = drive(core, "r1", lambda: core.robot_view("r1")["phase"] == "SWAPPING")
    out += core.tick()
    swap = cmds(out, "SwapCmd")[0]
    seen = []
    core.traffic.on_robot_fault("r1", "RECOVERY_CONFLICT")
    core._pump()
    seen += core.tick()
    now[0] += core.params.arm_cancel_wait_s + 0.1
    seen += core.tick()
    assert "ARM_NO_RESULT" in codes(seen)
    assert len(isolated(seen)) == 1, "not isolated twice"
    assert core.robot_view("r1")["order"] == "O-001"
    core.on_swap_result(swap.goal_id, False, True, True, "CANCELED")
    assert [r.status for r in cmds(core.tick(), "ReportOrderCmd")] == ["SHIPPED"]


def open_calls_in(out):
    return [c for c in out if isinstance(c, DispatchCmd) and not c.robot_id]


def test_an_isolation_by_not_at_target_keeps_a_passed_zone_in_the_same_tick():
    """마지막 NOT_AT_TARGET 시도에서, 지나온 구역 밖 보고가 이미 다 쌓였어도 그 구역은 쥔다."""
    now = [0.0]
    site = load_lane_map()
    core = TrafficController(site, load_params(), ["a"], lambda: now[0])
    x, y = site.waypoint_pose("P-6")
    for _ in range(core.params.release_confirm_samples):
        core.on_robot_state("a", x, y, 0.0, "IDLE")
    m = core._missions["a"]
    behind = core._zone("P-6")
    ahead = core._zone("N")
    core.zones.force_acquire("a", ahead)
    m.path, m.idx, m.phase = ["P-6", "N"], 1, "TO_PARK"
    m.attempts = core.params.move_retry_limit
    m.settle_idx = 1
    m.arrival_by = 1.0
    lane = site.lane("P-6", "N")
    s = core.params.release_distance_m + 0.15
    for _ in range(core.params.release_confirm_samples):
        core.on_robot_state("a", lane.ax + lane.ux * s, lane.ay + lane.uy * s, 0.0, "IDLE")
    assert m.heard_since_success, "reports came after the success"
    assert m.outside.get(behind, 0) >= core.params.release_confirm_samples
    now[0] = 1.5
    core.tick_progress(now[0])
    assert m.fault == "ERROR" and m.fault_reason == "NOT_AT_TARGET"
    assert behind in core.zones.held_by("a"), "an isolated robot keeps its zones"


def test_requeued_orders_keep_registration_order_ahead_of_never_bound_ones():
    O3 = OrderRecord("O-003", {"D": 1})
    core, _ = orchestrator((O1, O2, O3))
    bind(core, "r1")
    bind(core, "r2")
    core.on_robot_isolated("r1", "GOAL_FAILED")
    core.on_robot_isolated("r2", "GOAL_FAILED")
    core.take_output()
    assert [o.order_id for o in core._pending()] == ["O-001", "O-002", "O-003"]


def test_a_clear_before_the_swap_result_requeues_at_once_and_a_late_take_only_warns():
    """정책: 교환 결과를 모르는 주문도 confirm_clear는 되돌린다. 늦은 took_full은 경고만 낸다."""
    now = [0.0]
    core = FleetManager(grid_site(), load_params(), ["r1"], lambda: now[0])
    report(core, "r1", (4, 3))
    core.on_order_board("RUN", [O1])
    out = drive(core, "r1", lambda: core.robot_view("r1")["order"] == "O-001")
    core.on_fill_result(cmds(out, "FillCmd")[0].goal_id, True, {"A": 1, "C": 1}, {})
    out = drive(core, "r1", lambda: core.robot_view("r1")["phase"] == "SWAPPING")
    out += core.tick()
    swap = cmds(out, "SwapCmd")[0]
    core.traffic.on_robot_fault("r1", "RECOVERY_CONFLICT")
    core._pump()
    seen = core.tick()
    assert "ORDER_REQUEUED" not in codes(seen), "held for the swap result"
    core.confirm_clear("r1", core.zones.held_by("r1"), "lifted")
    seen = core.tick()
    assert codes(seen).count("ORDER_REQUEUED") == 1
    assert core.robot_view("r1")["order"] is None
    assert [o.order_id for o in core.orchestrator._pending()] == ["O-001"]
    assert len(core.orchestrator._open_calls) == 1, "a call for it is open again"
    core.on_swap_result(swap.goal_id, False, True, False, "CANCELED")
    late = list(core._out)
    assert [(type(c).__name__, getattr(c, "code", "")) for c in late] == [
        ("EventOut", "ARM_LATE_RESULT")]
    assert core.shipped == 0
