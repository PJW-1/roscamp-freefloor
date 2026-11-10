"""ArmJobs alone, the orders asking it, and the two joined: the split keeps the old order."""
from core_helpers import cmds, codes
from fleet_core.arm.jobs import ArmJobs
from fleet_core.common.commands import (
    FILL,
    SWAP,
    ArmDeclined,
    ArmDropCmd,
    ArmFailedCmd,
    ArmFillCmd,
    ArmFillResult,
    ArmLate,
    ArmLost,
    ArmResetCmd,
    ArmStopCmd,
    ArmSwapCmd,
    ArmSwapResult,
    CancelCmd,
    EventOut,
    FillCmd,
    OrderRecord,
    ReportOrderCmd,
    RobotFaultCmd,
    SwapCmd,
)
from fleet_core.common.params import load_params
from fleet_core.orders.orchestrator import OrderOrchestrator
from grid_map import grid_site
from orders_and_arm import OrdersAndArm

O1 = OrderRecord("O-001", {"A": 1, "C": 1})
ASK_FILL = ArmFillCmd("r1", "WORK", "O-001", {"A": 1, "C": 1})
ASK_SWAP = ArmSwapCmd("r1", "OUTBOUND", "O-001", take_full=True, put_empty=True)


def arm(params=None):
    now = [0.0]
    jobs = ArmJobs(grid_site().stations, params or load_params(), ["r1", "r2"], lambda: now[0])
    return jobs, now


def started(ask=ASK_FILL):
    jobs, now = arm()
    jobs.on_command(ask)
    return jobs, now, jobs.take_output()[0]


def test_a_fill_asked_for_goes_out_as_one_goal_to_the_station_server():
    jobs, _, goal = started()
    assert goal == FillCmd(1, "r1", "WORK", "O-001", {"A": 1, "C": 1})
    assert jobs.sent_at("r1") == 0.0 and jobs.sent_at("r2") is None


def test_a_swap_asked_for_names_the_server_of_its_station():
    _, _, goal = started(ASK_SWAP)
    assert goal == SwapCmd("arm3", 1, "r1", "OUTBOUND", "O-001", True, True)


def test_a_result_goes_to_the_orders_and_frees_the_robot():
    jobs, _, goal = started()
    jobs.on_fill_result(goal.goal_id, True, {"A": 1}, {"C": 1})
    assert jobs.take_output() == [ArmFillResult("r1", True, {"A": 1})]
    assert jobs.sent_at("r1") is None
    jobs, _, goal = started(ASK_SWAP)
    jobs.on_swap_result(goal.goal_id, False, True, False, "GRIPPER")
    assert jobs.take_output() == [ArmSwapResult("r1", True, False)], "the orders judge it"


def test_a_failed_attempt_is_counted_only_when_the_orders_say_so():
    jobs, _, goal = started()
    jobs.on_fill_result(goal.goal_id, False, {}, {}, "GRIPPER")
    jobs.take_output()
    assert jobs.attempts("r1") == 0
    jobs.on_command(ArmFailedCmd("r1"))
    out = jobs.take_output()
    assert codes(out) == ["FILL_RETRY"] and jobs.attempts("r1") == 1
    jobs.on_command(ArmResetCmd("r1"))
    assert jobs.attempts("r1") == 0


def test_no_new_goal_goes_out_during_the_wait_after_a_failure():
    jobs, now, goal = started()
    jobs.on_fill_result(goal.goal_id, False, {}, {}, "GRIPPER")
    jobs.on_command(ArmFailedCmd("r1"))
    jobs.take_output()
    jobs.on_command(ASK_FILL)
    assert jobs.take_output() == [ArmDeclined("r1")]
    now[0] += jobs.params.arm_retry_wait_s
    jobs.on_command(ASK_FILL)
    assert cmds(jobs.take_output(), "FillCmd")[0].goal_id == goal.goal_id + 1


def test_past_the_retry_limit_the_robot_is_reported_faulty():
    jobs, now, goal = started(ASK_SWAP)
    for _ in range(jobs.params.move_retry_limit):
        jobs.on_swap_result(goal.goal_id, False, False, False, "GRIPPER")
        jobs.on_command(ArmFailedCmd("r1"))
        now[0] += jobs.params.arm_retry_wait_s
        jobs.on_command(ASK_SWAP)
        goal = cmds(jobs.take_output(), "SwapCmd")[0]
    jobs.on_swap_result(goal.goal_id, False, False, False, "GRIPPER")
    jobs.on_command(ArmFailedCmd("r1"))
    out = jobs.take_output()
    assert [type(c) for c in out] == [ArmSwapResult, RobotFaultCmd, EventOut]
    assert out[1].reason == "arm3: GRIPPER" and codes(out) == ["SWAP_FAILED"]


def test_a_goal_stopped_by_the_run_mode_is_no_failed_attempt():
    jobs, _, goal = started()
    jobs.on_run_mode("STOP")
    assert jobs.take_output() == [CancelCmd("workcell", goal.goal_id)]
    jobs.on_run_mode("STOP")
    assert not jobs.take_output(), "canceled once"
    jobs.on_fill_result(goal.goal_id, False, {"A": 1}, {}, "CANCELED")
    jobs.on_command(ArmFailedCmd("r1"))
    assert jobs.take_output() == [ArmFillResult("r1", False, {"A": 1})]
    assert jobs.attempts("r1") == 0


def test_a_goal_past_its_time_is_canceled_and_then_given_up():
    jobs, now, goal = started()
    now[0] = jobs.params.fill_timeout_s + 0.1
    jobs.check_timeouts(now[0])
    assert jobs.take_output() == [CancelCmd("workcell", goal.goal_id)]
    now[0] += jobs.params.arm_cancel_wait_s
    jobs.check_timeouts(now[0])
    out = jobs.take_output()
    assert out[0] == ArmLost("r1", goal.goal_id, "workcell", FILL)
    assert [type(c) for c in out[1:]] == [RobotFaultCmd, EventOut]
    assert codes(out) == ["ARM_NO_RESULT"] and jobs.sent_at("r1") is None
    jobs.on_fill_result(goal.goal_id, True, {"A": 1}, {})
    assert jobs.take_output() == [ArmLate(goal.goal_id, FILL, placed={"A": 1})]


def test_a_dropped_job_is_canceled_and_its_server_gets_time_to_stop():
    jobs, now, goal = started()
    jobs.on_command(ArmDropCmd("r1", True, "O-001", 4))
    assert jobs.take_output() == [
        CancelCmd("workcell", goal.goal_id),
        ArmLost("r1", goal.goal_id, "workcell", FILL, cleared=True, isolated=True,
                order_id="O-001", bound=4)]
    jobs.on_command(ArmFillCmd("r2", "WORK", "O-002", {"B": 1}))
    assert jobs.take_output() == [ArmDeclined("r2")], "the server is still stopping"
    now[0] += jobs.params.arm_cancel_wait_s
    jobs.on_command(ArmFillCmd("r2", "WORK", "O-002", {"B": 1}))
    assert cmds(jobs.take_output(), "FillCmd")


def test_a_stop_from_the_orders_cancels_the_running_goal_once():
    jobs, _, goal = started(ASK_SWAP)
    jobs.on_command(ArmStopCmd("r1"))
    jobs.on_command(ArmStopCmd("r1"))
    assert jobs.take_output() == [CancelCmd("arm3", goal.goal_id)]
    jobs.on_command(ArmStopCmd("r2"))
    assert not jobs.take_output(), "nothing runs for r2"


def test_a_result_nobody_waits_for_is_passed_on_as_late():
    jobs, _ = arm()
    jobs.on_swap_result(7, True, True, False)
    assert jobs.take_output() == [ArmLate(7, SWAP, took_full=True, put_empty_done=False)]


def test_the_arm_ignores_a_robot_it_does_not_know():
    jobs, _ = arm()
    for command in (ArmFillCmd("ghost", "WORK", "O-001", {"A": 1}), ArmStopCmd("ghost"),
                    ArmDropCmd("ghost"), ArmFailedCmd("ghost"), ArmResetCmd("ghost")):
        jobs.on_command(command)
    assert jobs.take_output() == [] and jobs.attempts("ghost") == 0


# ---- the orders alone ---------------------------------------------------------------------


def test_the_orders_ask_the_arm_and_send_no_goal_themselves():
    now = [0.0]
    orders = OrderOrchestrator(grid_site().stations, load_params(), ["r1"], lambda: now[0])
    orders.on_order_board("RUN", [O1])
    orders.on_dispatch_result(orders._open_calls[0], True, "r1")
    orders.take_output()
    orders.step("r1")
    assert orders.take_output() == [ASK_FILL]
    orders.step("r1")
    assert orders.take_output() == [], "one request until the arm answers"
    orders.on_arm_report(ArmDeclined("r1"))
    orders.step("r1")
    assert orders.take_output() == [ASK_FILL], "asked again after a refusal"
    orders.on_order_board("STOP", [O1])
    assert not cmds(orders.take_output(), "CancelCmd"), "the arm side stops its own goals"


# ---- joined: the order of what comes out is the one of the single orchestrator ----------------


def joined_swapping():
    now = [0.0]
    core = OrdersAndArm(grid_site().stations, load_params(), ["r1", "r2"], lambda: now[0])
    core.on_order_board("RUN", [O1])
    core.on_dispatch_result(core._open_calls[0], True, "r1")
    core.step("r1")
    fill = cmds(core.take_output(), "FillCmd")[0]
    core.on_fill_result(fill.goal_id, True, dict(fill.items), {})
    core.step("r1")
    goal = cmds(core.take_output(), "DispatchCmd")[-1]
    core.on_dispatch_result(goal.goal_id, True, "r1")
    core.step("r1")
    return core, now, cmds(core.take_output(), "SwapCmd")[0]


def test_a_swap_that_took_the_basket_and_failed_reports_the_order_before_the_retry_warning():
    core, _, swap = joined_swapping()
    core.on_swap_result(swap.goal_id, False, True, False, "GRIPPER")
    out = core.take_output()
    assert [type(c) for c in out] == [ReportOrderCmd, EventOut] and codes(out) == ["SWAP_RETRY"]


def test_removing_a_robot_cancels_its_goal_before_the_order_goes_back():
    core, _, swap = joined_swapping()
    core.on_robot_removed("r1")
    out = core.take_output()
    assert out[0] == CancelCmd("arm3", swap.goal_id)
    assert codes(out)[0] == "ORDER_REQUEUED"
