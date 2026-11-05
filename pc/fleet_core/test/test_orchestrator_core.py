"""OrchestratorCore alone: it asks for robots and arms, it never sees a lane or a pose."""
from core_helpers import cmds, codes
from fleet_core.commands import (
    CancelDispatchCmd,
    DispatchCmd,
    OrderRecord,
    ReleaseCmd,
    RobotFaultCmd,
)
from fleet_core.orchestrator_core import OrchestratorCore
from fleet_core.params import load_params
from grid_map import grid_site

WORK = "WORK"
OUTBOUND = "OUTBOUND"
O1 = OrderRecord("O-001", {"A": 1, "C": 1})
O2 = OrderRecord("O-002", {"B": 1})
O3 = OrderRecord("O-003", {"D": 2})


def make(orders=(), robots=("r1", "r2"), params=None):
    now = [0.0]
    core = OrchestratorCore(grid_site().stations, params or load_params(), list(robots),
                            lambda: now[0])
    core.on_order_board("RUN", list(orders))
    return core, now


def only(out, kind):
    return [c for c in out if isinstance(c, kind)]


def open_calls(core, out):
    """Goal ids of the unnamed WORK dispatches the output left open."""
    return [c.goal_id for c in only(out, DispatchCmd) if not c.robot_id]


def bound(core, robot="r1"):
    """Bind the oldest order to a robot through a WORK result."""
    goal = core._open_calls[0]
    core.on_dispatch_result(goal, True, robot)
    return core.take_output()


def filling(orders=(O1,), robot="r1"):
    core, now = make(orders)
    core.take_output()
    bound(core, robot)
    core.step(robot)
    fill = cmds(core.take_output(), "FillCmd")[0]
    return core, now, fill


def outbound_goal(core, robot="r1"):
    """The named dispatch a finished fill just sent."""
    core.step(robot)
    return [c for c in only(core.take_output(), DispatchCmd) if c.robot_id == robot][0]


def swapping(robot="r1"):
    core, now, fill = filling((O1,), robot)
    core.on_fill_result(fill.goal_id, True, dict(fill.items), {})
    goal = outbound_goal(core, robot)
    core.on_dispatch_result(goal.goal_id, True, robot)
    core.take_output()
    core.step(robot)
    swap = cmds(core.take_output(), "SwapCmd")[0]
    return core, now, swap


def test_open_dispatches_follow_the_unbound_order_count():
    core, _ = make()
    assert open_calls(core, core.take_output()) == []
    core.on_order_board("RUN", [O1, O2])
    assert len(open_calls(core, core.take_output())) == 2
    core.on_order_board("RUN", [O1, O2, O3])
    assert len(open_calls(core, core.take_output())) == 1, "only the new one"
    core.on_order_board("RUN", [O3])
    assert len(only(core.take_output(), CancelDispatchCmd)) == 2


def test_open_dispatches_ask_for_the_fill_station_and_name_no_robot():
    core, _ = make([O1])
    sent = only(core.take_output(), DispatchCmd)
    assert [(c.station_id, c.robot_id) for c in sent] == [(WORK, "")]


def test_a_work_result_binds_the_oldest_order_and_closes_that_dispatch():
    core, _ = make([O1, O2])
    core.take_output()
    out = bound(core, "r2")
    assert core.order_of("r2") == "O-001"
    assert "ORDER_BOUND" in codes(out)
    assert not only(out, DispatchCmd) and not only(out, CancelDispatchCmd), "one less of each"
    assert len(core._open_calls) == 1


def test_a_work_result_with_nothing_left_to_bind_releases_the_robot():
    core, _ = make([O1])
    core.take_output()
    goal = core._open_calls[0]
    core._reported.add("O-001")
    core.on_dispatch_result(goal, True, "r1")
    assert only(core.take_output(), ReleaseCmd) == [ReleaseCmd("r1")]


def test_filling_asks_the_workcell_for_what_is_missing():
    core, _, fill = filling()
    assert (fill.robot, fill.station_id, fill.order_id, fill.items) == (
        "r1", WORK, "O-001", {"A": 1, "C": 1})
    core.step("r1")
    assert not cmds(core.take_output(), "FillCmd"), "the job is still running"


def test_a_finished_fill_names_the_robot_for_the_outbound_station():
    core, _, fill = filling()
    core.on_fill_result(fill.goal_id, True, {"A": 1, "C": 1}, {})
    goal = outbound_goal(core)
    assert (goal.station_id, goal.robot_id) == (OUTBOUND, "r1")
    assert core.robot_view("r1")["basket"] == "FULL"


def test_nothing_placed_reports_held_and_releases_the_robot():
    core, _, fill = filling()
    core.on_fill_result(fill.goal_id, True, {}, {"A": 1, "C": 1})
    core.step("r1")
    out = core.take_output()
    assert [c.status for c in cmds(out, "ReportOrderCmd")] == ["HELD"]
    assert only(out, ReleaseCmd) == [ReleaseCmd("r1")]
    assert core.order_of("r1") == ""


def test_an_order_over_capacity_is_held_without_calling_an_arm():
    big = OrderRecord("O-BIG", {"A": load_params().basket_capacity + 1})
    core, _ = make([big])
    core.take_output()
    bound(core)
    core.step("r1")
    out = core.take_output()
    assert not cmds(out, "FillCmd")
    assert [c.status for c in cmds(out, "ReportOrderCmd")] == ["HELD"]
    assert only(out, ReleaseCmd) == [ReleaseCmd("r1")]


def test_an_outbound_result_starts_the_swap_and_the_finished_swap_releases():
    core, _, swap = swapping()
    assert (swap.arm, swap.take_full, swap.put_empty) == ("arm3", True, True)
    assert swap.order_id == "O-001"
    core.on_swap_result(swap.goal_id, True, True, True)
    out = core.take_output()
    assert [c.status for c in cmds(out, "ReportOrderCmd")] == ["SHIPPED"]
    assert only(out, ReleaseCmd) == [ReleaseCmd("r1")]
    assert core.shipped == 1
    assert core.robot_view("r1")["basket"] == "EMPTY"


def test_the_order_is_reported_on_the_take_even_if_the_put_still_has_to_be_done():
    core, now, swap = swapping()
    core.on_swap_result(swap.goal_id, False, True, False, "GRIPPER")
    out = core.take_output()
    assert [c.status for c in cmds(out, "ReportOrderCmd")] == ["SHIPPED"]
    assert not only(out, ReleaseCmd), "no basket on the tray yet"
    assert core.robot_view("r1")["basket"] == "NONE"
    now[0] += core.params.arm_retry_wait_s
    core.step("r1")
    retry = cmds(core.take_output(), "SwapCmd")[0]
    assert (retry.take_full, retry.put_empty) == (False, True)


def test_running_out_of_arm_attempts_faults_the_robot_on_the_traffic_side():
    core, now, fill = filling()
    for _ in range(core.params.move_retry_limit):
        core.on_fill_result(fill.goal_id, False, {}, {}, "GRIPPER")
        now[0] += core.params.arm_retry_wait_s
        core.step("r1")
        fill = cmds(core.take_output(), "FillCmd")[0]
    core.on_fill_result(fill.goal_id, False, {}, {}, "GRIPPER")
    out = core.take_output()
    assert "FILL_FAILED" in codes(out)
    assert [c.robot for c in only(out, RobotFaultCmd)] == ["r1"]
    assert core.order_of("r1") == "O-001", "kept for the operator"


def test_an_arm_that_never_answers_a_cancel_faults_the_robot():
    core, now, _ = filling()
    now[0] += core.params.fill_timeout_s + 0.1
    core.check_arm_timeouts(now[0])
    assert cmds(core.take_output(), "CancelCmd")
    now[0] += core.params.arm_cancel_wait_s
    core.check_arm_timeouts(now[0])
    out = core.take_output()
    assert "ARM_NO_RESULT" in codes(out)
    assert [c.robot for c in only(out, RobotFaultCmd)] == ["r1"]


def test_stop_cancels_the_arm_and_keeps_every_dispatch_open():
    core, _, fill = filling((O1, O2))
    core.on_order_board("STOP", [O1, O2])
    out = core.take_output()
    assert [(c.robot, c.goal_id) for c in cmds(out, "CancelCmd")] == [("workcell", fill.goal_id)]
    assert not only(out, CancelDispatchCmd)


def test_removing_a_robot_requeues_its_order_once_and_opens_a_dispatch_for_it():
    core, _, fill = filling((O1, O2))
    assert len(core._open_calls) == 1
    core.on_robot_removed("r1")
    out = core.take_output()
    assert codes(out).count("ORDER_REQUEUED") == 1
    assert len(open_calls(core, out)) == 1
    assert [(c.robot, c.goal_id) for c in cmds(out, "CancelCmd")] == [("workcell", fill.goal_id)]
    core.on_robot_removed("r1")
    assert "ORDER_REQUEUED" not in codes(core.take_output())
    assert [o.order_id for o in core._pending()] == ["O-001", "O-002"]


def test_no_robot_feedback_becomes_the_warning_naming_the_waiting_orders():
    core, _ = make([O1, O2])
    core.take_output()
    core.on_dispatch_feedback(core._open_calls[0], "", "NO_ROBOT")
    event = [e for e in cmds(core.take_output(), "EventOut")][0]
    assert event.code == "NO_ROBOT_AVAILABLE" and event.level == "WARN"
    assert "O-001" in event.detail and "O-002" in event.detail


def test_a_failed_outbound_dispatch_keeps_the_order_for_the_operator():
    core, _, fill = filling()
    core.on_fill_result(fill.goal_id, True, dict(fill.items), {})
    goal = outbound_goal(core)
    core.on_dispatch_result(goal.goal_id, False, "r1", "GOAL_FAILED")
    assert core.order_of("r1") == "O-001"
    assert core.robot_view("r1")["basket"] == "FULL"
    core.step("r1")
    assert not core.take_output()
