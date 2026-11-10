"""OrderOrchestrator와 ArmJobs를 이은 것: 로봇을 요청하고 팔 goal을 낼 뿐 레인이나 자세는 모른다."""
from core_helpers import cmds, codes
from fleet_core.common.commands import (
    CancelDispatchCmd,
    DispatchCmd,
    OrderRecord,
    ReleaseCmd,
    RobotFaultCmd,
)
from fleet_core.common.params import load_params
from grid_map import grid_site
from orders_and_arm import OrdersAndArm

WORK = "WORK"
OUTBOUND = "OUTBOUND"
O1 = OrderRecord("O-001", {"A": 1, "C": 1})
O2 = OrderRecord("O-002", {"B": 1})
O3 = OrderRecord("O-003", {"D": 2})


def make(orders=(), robots=("r1", "r2"), params=None):
    now = [0.0]
    core = OrdersAndArm(grid_site().stations, params or load_params(), list(robots),
                            lambda: now[0])
    core.on_order_board("RUN", list(orders))
    return core, now


def only(out, kind):
    return [c for c in out if isinstance(c, kind)]


def open_calls(core, out):
    """출력이 열어 둔 이름 없는 WORK 디스패치의 목표 id."""
    return [c.goal_id for c in only(out, DispatchCmd) if not c.robot_id]


def bound(core, robot="r1"):
    """가장 오래된 주문을 WORK 결과로 로봇에 묶는다."""
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
    """채움이 끝나며 방금 보낸 이름 있는 디스패치."""
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


def test_a_fill_the_server_canceled_by_itself_counts_as_an_attempt_with_the_retry_wait():
    """여기선 아무도 요청하지 않았다. 세지 않으면 틱마다 한없이 다시 보낸다."""
    core, now, fill = filling()
    core.on_fill_result(fill.goal_id, False, {"A": 1}, {}, "CANCELED")
    assert "FILL_RETRY" in codes(core.take_output())
    assert core.robot_view("r1")["attempts"] == 1
    assert core.robot_view("r1")["placed"] == {"A": 1}
    core.step("r1")
    assert not cmds(core.take_output(), "FillCmd"), "the retry wait applies"
    now[0] += core.params.arm_retry_wait_s
    core.step("r1")
    assert cmds(core.take_output(), "FillCmd")


def test_a_swap_the_server_canceled_by_itself_counts_as_an_attempt_with_the_retry_wait():
    core, now, swap = swapping()
    core.on_swap_result(swap.goal_id, False, False, False, "CANCELED")
    assert "SWAP_RETRY" in codes(core.take_output())
    assert core.robot_view("r1")["attempts"] == 1
    core.step("r1")
    assert not cmds(core.take_output(), "SwapCmd"), "the retry wait applies"
    now[0] += core.params.arm_retry_wait_s
    core.step("r1")
    assert cmds(core.take_output(), "SwapCmd")


def test_a_server_that_keeps_canceling_by_itself_runs_out_of_attempts():
    core, now, fill = filling()
    for _ in range(core.params.move_retry_limit):
        core.on_fill_result(fill.goal_id, False, {}, {}, "CANCELED")
        now[0] += core.params.arm_retry_wait_s
        core.step("r1")
        fill = cmds(core.take_output(), "FillCmd")[0]
    core.on_fill_result(fill.goal_id, False, {}, {}, "CANCELED")
    out = core.take_output()
    assert "FILL_FAILED" in codes(out)
    assert [c.robot for c in only(out, RobotFaultCmd)] == ["r1"]


def test_a_cancel_this_fms_asked_for_stays_harmless():
    core, _, fill = filling()
    core.on_order_board("STOP", [O1])
    core.take_output()
    core.on_fill_result(fill.goal_id, False, {"A": 1}, {}, "CANCELED")
    assert "FILL_RETRY" not in codes(core.take_output())
    assert core.robot_view("r1")["attempts"] == 0


def lose_result(core, now, limit):
    """실행 중인 팔 작업을 시간 초과시키고 취소에 응답하지 않게 한다."""
    now[0] += limit + 0.1
    core.check_arm_timeouts(now[0])
    core.take_output()
    now[0] += core.params.arm_cancel_wait_s + 0.1
    core.check_arm_timeouts(now[0])
    assert "ARM_NO_RESULT" in codes(core.take_output())


def lost_fill():
    core, now, fill = filling()
    lose_result(core, now, core.params.fill_timeout_s)
    return core, fill


def lost_swap():
    core, now, swap = swapping()
    core.take_output()
    lose_result(core, now, core.params.swap_timeout_s)
    return core, swap


def late_event(out):
    events = [e for e in cmds(out, "EventOut") if e.code == "ARM_LATE_RESULT"]
    assert len(events) == 1
    assert (events[0].level, events[0].robot) == ("WARN", "r1")
    return events[0].detail


def test_a_late_fill_result_goes_on_the_order_that_is_still_bound():
    core, fill = lost_fill()
    core.on_fill_result(fill.goal_id, False, {"A": 1}, {}, "CANCELED")
    out = core.take_output()
    detail = late_event(out)
    assert "{'A': 1}" in detail and "workcell" in detail and str(fill.goal_id) in detail
    assert core.robot_view("r1")["placed"] == {"A": 1}
    assert core.order_of("r1") == "O-001"
    assert not only(out, ReleaseCmd), "the operator still has to look"
    assert core.robot_view("r1")["attempts"] == 0, "the attempt is over, not failed again"


def test_a_late_swap_that_took_the_basket_reports_the_order_as_on_time():
    core, swap = lost_swap()
    core.on_swap_result(swap.goal_id, False, True, False, "CANCELED")
    out = core.take_output()
    detail = late_event(out)
    assert "took_full=True" in detail and "put_empty_done=False" in detail
    reports = cmds(out, "ReportOrderCmd")
    assert [(r.order_id, r.status) for r in reports] == [("O-001", "SHIPPED")]
    assert reports[0].placed == {"A": 1, "C": 1}
    assert core.shipped == 1
    assert core.order_of("r1") == ""
    assert core.robot_view("r1")["basket"] == "NONE"
    assert not only(out, ReleaseCmd), "the operator still has to look"


def test_a_late_swap_that_finished_both_steps_leaves_an_empty_basket_and_no_release():
    core, swap = lost_swap()
    core.on_swap_result(swap.goal_id, True, True, True)
    out = core.take_output()
    assert [r.status for r in cmds(out, "ReportOrderCmd")] == ["SHIPPED"]
    assert core.robot_view("r1")["basket"] == "EMPTY"
    assert not only(out, ReleaseCmd)


def test_after_a_late_swap_result_clearing_the_robot_requeues_nothing_and_reports_nothing():
    core, swap = lost_swap()
    core.on_swap_result(swap.goal_id, False, True, False, "CANCELED")
    core.take_output()
    core.on_robot_removed("r1")
    out = core.take_output()
    assert "ORDER_REQUEUED" not in codes(out)
    assert not cmds(out, "ReportOrderCmd")
    assert not only(out, DispatchCmd), "nobody is called for it"
    assert core._pending() == [], "the board still lists it, and it is not to be worked again"


def test_a_late_swap_that_did_not_take_the_basket_changes_nothing():
    core, swap = lost_swap()
    core.on_swap_result(swap.goal_id, False, False, False, "CANCELED")
    out = core.take_output()
    assert "took_full=False" in late_event(out)
    assert not cmds(out, "ReportOrderCmd")
    assert core.order_of("r1") == "O-001"
    assert core.robot_view("r1")["basket"] == "FULL"


def test_a_late_result_counts_only_once():
    core, swap = lost_swap()
    core.on_swap_result(swap.goal_id, False, True, False, "CANCELED")
    core.take_output()
    core.on_swap_result(swap.goal_id, False, True, False, "CANCELED")
    assert core.take_output() == []
    assert core.shipped == 1


def test_a_late_swap_result_after_the_clear_warns_of_a_second_shipment_and_changes_nothing():
    core, swap = lost_swap()
    core.on_robot_removed("r1")
    assert "ORDER_REQUEUED" in codes(core.take_output())
    core.on_swap_result(swap.goal_id, False, True, False, "CANCELED")
    out = core.take_output()
    detail = late_event(out)
    assert "took_full=True" in detail and "O-001" in detail and "twice" in detail
    assert not cmds(out, "ReportOrderCmd")
    assert core.shipped == 0
    assert [o.order_id for o in core._pending()] == ["O-001"], "the requeue stays"


def test_a_late_fill_result_after_the_clear_warns_of_a_second_fill_and_changes_nothing():
    core, fill = lost_fill()
    core.on_robot_removed("r1")
    core.take_output()
    core.on_fill_result(fill.goal_id, True, {"A": 1}, {})
    detail = late_event(core.take_output())
    assert "{'A': 1}" in detail and "O-001" in detail and "twice" in detail
    assert core.robot_view("r1")["placed"] == {}


def test_a_late_put_after_the_order_was_reported_only_says_so():
    core, now, swap = swapping()
    core.on_swap_result(swap.goal_id, False, True, False, "GRIPPER")
    core.take_output()
    now[0] += core.params.arm_retry_wait_s
    core.step("r1")
    retry = cmds(core.take_output(), "SwapCmd")[0]
    lose_result(core, now, core.params.swap_timeout_s)
    core.on_swap_result(retry.goal_id, True, False, True)
    out = core.take_output()
    assert "twice" not in late_event(out)
    assert not cmds(out, "ReportOrderCmd")
    assert core.shipped == 1


def test_a_fill_the_clear_canceled_that_still_placed_something_warns_of_a_second_fill():
    """운영자는 작업이 실행 중임을 알았지 그 뒤 무엇을 했는지는 모른다. 결과만 알려 준다."""
    core, _, fill = filling()
    core.on_robot_removed("r1")
    core.take_output()
    core.on_fill_result(fill.goal_id, False, {"A": 1}, {}, "CANCELED")
    detail = late_event(core.take_output())
    assert "{'A': 1}" in detail and "O-001" in detail and "twice" in detail
    assert core.robot_view("r1")["placed"] == {}
    assert [o.order_id for o in core._pending()] == ["O-001"], "the requeue stays"


def test_a_swap_the_clear_canceled_that_took_the_basket_warns_of_a_second_shipment():
    core, _, swap = swapping()
    core.on_robot_removed("r1")
    assert "ORDER_REQUEUED" in codes(core.take_output())
    core.on_swap_result(swap.goal_id, False, True, False, "CANCELED")
    out = core.take_output()
    detail = late_event(out)
    assert "took_full=True" in detail and "O-001" in detail and "twice" in detail
    assert not cmds(out, "ReportOrderCmd")
    assert core.shipped == 0


def test_the_result_of_a_job_the_clear_canceled_stays_silent_when_the_arm_did_nothing():
    core, _, fill = filling()
    core.on_robot_removed("r1")
    core.take_output()
    core.on_fill_result(fill.goal_id, False, {}, {}, "CANCELED")
    assert core.take_output() == []
    core, _, swap = swapping()
    core.on_robot_removed("r1")
    core.take_output()
    core.on_swap_result(swap.goal_id, False, False, False, "CANCELED")
    assert core.take_output() == []


def test_a_job_the_clear_canceled_is_reported_once():
    core, _, fill = filling()
    core.on_robot_removed("r1")
    core.take_output()
    core.on_fill_result(fill.goal_id, False, {"A": 1}, {}, "CANCELED")
    core.take_output()
    core.on_fill_result(fill.goal_id, False, {"A": 1}, {}, "CANCELED")
    assert core.take_output() == []


def test_the_cancel_warning_waits_as_long_as_the_core_waits_for_an_arm():
    """노드의 CANCEL_UNCONFIRMED는 코어가 아직 기다리는 팔을 일찍 부르면 안 된다."""
    params = load_params()
    assert params.cancel_warn_after * params.cancel_retry_s >= params.arm_cancel_wait_s


def test_a_result_nobody_waits_for_stays_silent():
    core, _, fill = filling()
    core.on_fill_result(fill.goal_id + 100, True, {"A": 1}, {})
    core.on_swap_result(fill.goal_id + 100, True, True, True)
    assert core.take_output() == []
