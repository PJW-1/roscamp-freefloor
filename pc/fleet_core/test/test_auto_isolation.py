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
