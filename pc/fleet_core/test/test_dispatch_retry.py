"""A failed outbound dispatch is sent again (work order 1.1, H3 item 1).

Before, on_dispatch_result only cleared outbound_goal and step() resent only a robot without a
basket, so a robot with a full basket was left with no next job.
"""
from dataclasses import replace

from core_helpers import cmds, codes
from fleet_core.common.commands import DispatchCmd, DispatchResult, OrderRecord, RobotFaultCmd
from fleet_core.common.params import load_params
from fleet_core.manager import FleetManager
from fleet_core.orders.orchestrator import OrderOrchestrator
from grid_map import grid_site
from test_auto_isolation import drive, report

O1 = OrderRecord("O-001", {"A": 1, "C": 1})
PARAMS = replace(load_params(), dispatch_retry_wait_s=2.0, dispatch_retry_limit=3)


def make(orders=(O1,)):
    now = [0.0]
    core = OrderOrchestrator(grid_site().stations, PARAMS, ["r1", "r2"], lambda: now[0])
    core.on_order_board("RUN", list(orders))
    core.take_output()
    return core, now


def outbound(out, robot="r1"):
    return [c for c in out if isinstance(c, DispatchCmd) and c.robot_id == robot]


def full_basket():
    """r1 has filled O-001 and has just been named for the outbound station."""
    core, now = make()
    core.on_dispatch_result(core._open_calls[0], True, "r1")
    core.take_output()
    core.step("r1")
    fill = cmds(core.take_output(), "FillCmd")[0]
    core.on_fill_result(fill.goal_id, True, dict(fill.items), {})
    core.step("r1")
    goal = outbound(core.take_output())[0]
    return core, now, goal


def fail(core, goal, reason="NOT_AVAILABLE"):
    core.on_dispatch_result(goal.goal_id, False, "r1", reason)
    return core.take_output()


def test_a_failed_outbound_dispatch_is_sent_again_after_the_wait():
    core, now, goal = full_basket()
    fail(core, goal)
    core.step("r1")
    assert not outbound(core.take_output()), "not before dispatch_retry_wait_s"
    now[0] += PARAMS.dispatch_retry_wait_s
    core.step("r1")
    out = core.take_output()
    again = outbound(out)
    assert [(c.station_id, c.robot_id) for c in again] == [("OUTBOUND", "r1")]
    assert again[0].goal_id != goal.goal_id
    retry = [e for e in cmds(out, "EventOut") if e.code == "DISPATCH_RETRY"]
    assert len(retry) == 1 and retry[0].level == "WARN" and retry[0].robot == "r1"
    assert "NOT_AVAILABLE" in retry[0].detail
    core.on_dispatch_result(again[0].goal_id, True, "r1")
    core.step("r1")
    assert cmds(core.take_output(), "SwapCmd"), "it goes on to the swap"
    assert core.order_of("r1") == "O-001"


def test_a_success_resets_the_count():
    core, now, goal = full_basket()
    for _ in range(PARAMS.dispatch_retry_limit):
        fail(core, goal)
        now[0] += PARAMS.dispatch_retry_wait_s
        core.step("r1")
        goal = outbound(core.take_output())[0]
    core.on_dispatch_result(goal.goal_id, True, "r1")
    assert core._robots["r1"].dispatch_failures == 0


def test_failures_past_the_limit_are_an_error_like_the_other_retry_limits():
    core, now, goal = full_basket()
    for _ in range(PARAMS.dispatch_retry_limit):
        fail(core, goal)
        now[0] += PARAMS.dispatch_retry_wait_s
        core.step("r1")
        goal = outbound(core.take_output())[0]
    fail(core, goal, "BUSY")
    now[0] += PARAMS.dispatch_retry_wait_s
    core.step("r1")
    out = core.take_output()
    assert not outbound(out), "no fifth try"
    errors = [e for e in cmds(out, "EventOut") if e.code == "DISPATCH_FAILED"]
    assert len(errors) == 1 and errors[0].level == "ERROR" and "BUSY" in errors[0].detail
    assert [c.robot for c in out if isinstance(c, RobotFaultCmd)] == ["r1"]
    assert not cmds(out, "ReportOrderCmd"), "no HELD: the isolation puts the order back"
    core.on_robot_isolated("r1", errors[0].detail)
    after = core.take_output()
    assert "ORDER_REQUEUED" in codes(after)
    now[0] += PARAMS.dispatch_retry_wait_s
    core.step("r1")
    assert not core.take_output(), "said once"


def test_an_isolated_robot_is_not_sent_again():
    """Traffic fails the dispatch, then isolates the robot in the same batch."""
    core, now, goal = full_basket()
    fail(core, goal, "GOAL_FAILED")
    core.on_robot_isolated("r1", "GOAL_FAILED")
    core.take_output()
    now[0] += PARAMS.dispatch_retry_wait_s
    core.step("r1")
    out = core.take_output()
    assert not outbound(out)
    assert "DISPATCH_RETRY" not in codes(out) and "DISPATCH_FAILED" not in codes(out)


def test_a_cleared_robot_is_not_sent_again():
    core, now, goal = full_basket()
    fail(core, goal, "ROBOT_REMOVED")
    core.on_robot_removed("r1")
    core.take_output()
    now[0] += PARAMS.dispatch_retry_wait_s
    core.step("r1")
    assert not outbound(core.take_output())


def test_a_robot_without_a_basket_also_waits_between_tries():
    """Before, a reinstated robot with no basket was sent again on every step."""
    core, now = make(())
    core.on_robot_reinstated("r1", "NONE")
    core.step("r1")
    goal = outbound(core.take_output())[0]
    fail(core, goal)
    core.step("r1")
    assert not outbound(core.take_output())
    now[0] += PARAMS.dispatch_retry_wait_s
    core.step("r1")
    assert outbound(core.take_output())


def test_through_the_manager_a_refused_outbound_dispatch_still_ships():
    now = [0.0]
    core = FleetManager(grid_site(), PARAMS, ["r1"], lambda: now[0])
    refused = []
    on_dispatch = core.traffic.on_dispatch

    def refuse_first_outbound(goal_id, station_id, robot_id=""):
        if station_id == "OUTBOUND" and not refused:
            refused.append(goal_id)
            core.traffic._out.append(DispatchResult(goal_id, False, robot_id, "NOT_AVAILABLE"))
            return
        on_dispatch(goal_id, station_id, robot_id)

    core.traffic.on_dispatch = refuse_first_outbound
    report(core, "r1", (4, 3))
    core.on_order_board("RUN", [O1])
    out = drive(core, "r1", lambda: core.robot_view("r1")["order"] == "O-001")
    fill = cmds(out, "FillCmd")[0]
    core.on_fill_result(fill.goal_id, True, {"A": 1, "C": 1}, {})
    seen = core.tick()
    assert refused
    now[0] += PARAMS.dispatch_retry_wait_s
    m = core.traffic._missions["r1"]
    seen += drive(core, "r1", lambda: m.phase == "AT_STATION" and m.dest == "OUTBOUND")
    assert "DISPATCH_RETRY" in codes(seen)
    assert cmds(seen, "SwapCmd"), "docked at the outbound station and swapping"
