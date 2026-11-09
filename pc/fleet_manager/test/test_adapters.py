"""fleet_core 레코드와 ROS 메시지 사이의 변환은 모듈 하나에 모여 있다."""
import math

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces")

from action_msgs.msg import GoalStatus  # noqa: E402
from fleet_core.common.commands import (  # noqa: E402
    AssignmentView,
    DockCmd,
    EventOut,
    FillCmd,
    MoveCmd,
    OrderRecord,
    ReportOrderCmd,
    RobotStatusView,
    SwapCmd,
    ZoneView,
)
from fleet_core.orders.orchestrator import CANCELED  # noqa: E402
from fleet_core.traffic.mission import (  # noqa: E402
    BLOCKED,
    HEARTBEAT_LOST,
    LOCALIZING,
    SELF_RECOVERING,
)
from fleet_interfaces.action import Dock, FillBasket, MoveTo, SwapBasket  # noqa: E402
from fleet_interfaces.msg import (  # noqa: E402
    Assignment,
    FleetEvent,
    Order,
    OrderBoard,
    OrderItem,
    RobotState,
    RobotStatus,
)
from fleet_interfaces.srv import (
    ConfirmClear,  # noqa: E402
    ReportOrder,  # noqa: E402
)
from fleet_manager import adapters  # noqa: E402


def test_time_splits_seconds_and_nanoseconds():
    stamp = adapters.to_time(12.25)
    assert (stamp.sec, stamp.nanosec) == (12, 250_000_000)


def test_time_rounding_up_carries_into_the_seconds():
    stamp = adapters.to_time(1.9999999999)
    assert (stamp.sec, stamp.nanosec) == (2, 0)


def test_zone_view_without_holder_becomes_empty_strings():
    msg = adapters.zones_to_msg([ZoneView("Z-OUT", "", (), None)], 3.0)
    assert msg.zones[0].zone_id == "Z-OUT"
    assert msg.zones[0].holder == ""
    assert list(msg.zones[0].queue) == []
    assert msg.stamp.sec == 3


def test_zone_view_with_queue_is_kept_in_order():
    msg = adapters.zones_to_msg([ZoneView("Z-OUT", "pinky1", ("pinky3", "pinky2"), 5.5)], 6.0)
    assert list(msg.zones[0].queue) == ["pinky3", "pinky2"]
    assert msg.zones[0].acquired_at.sec == 5


def test_assignment_uses_the_message_constants():
    msg = adapters.assignments_to_msg(
        [AssignmentView("pinky1", "O-001", "WORK", Assignment.TYPE_FILL),
         AssignmentView("pinky2", "", "", Assignment.TYPE_WAIT)], 1.0)
    first, second = msg.assignments
    assert (first.robot_id, first.order_id, first.station_id, first.task_type) == (
        "pinky1", "O-001", "WORK", "FILL")
    assert (second.order_id, second.station_id, second.task_type) == ("", "", "WAIT")


def test_event_keeps_level_code_and_names_the_robot():
    msg = adapters.event_to_msg(EventOut(FleetEvent.LEVEL_WARN, "COMM_LOST", "gone", "pinky2"), 2.0)
    assert (msg.level, msg.code, msg.source) == ("WARN", "COMM_LOST", "pinky2")


def test_event_without_robot_uses_the_manager_as_source():
    msg = adapters.event_to_msg(EventOut("INFO", "RUN", "started"), 2.0)
    assert msg.source == "fleet_manager"


def test_order_board_becomes_plain_records_in_board_order():
    board = OrderBoard(run_mode=OrderBoard.MODE_RUN, orders=[
        Order(order_id="O-002", items=[OrderItem(kind="A", count=1), OrderItem(kind="C", count=2)]),
        Order(order_id="O-003", items=[OrderItem(kind="B", count=1)]),
    ])
    assert adapters.order_board_orders(board) == [
        OrderRecord("O-002", {"A": 1, "C": 2}), OrderRecord("O-003", {"B": 1})]


def test_goals_copy_every_field():
    move = adapters.move_goal(MoveCmd("pinky1", 7, "c9_2", 1.0, 2.0, 0.5))
    assert (move.target_id, move.target_pose.x, move.target_pose.y, move.target_pose.theta) == (
        "c9_2", 1.0, 2.0, 0.5)
    assert adapters.dock_goal(DockCmd("pinky1", 8, "WORK")).dock_id == "WORK"
    assert list(move.path) == [], "point mode sends no path"


def test_a_move_goal_carries_the_lane_path_as_poses():
    path = ((0.5, 1.0, 0.0), (1.0, 1.0, math.pi / 2), (1.0, 2.0, math.pi / 2))
    move = adapters.move_goal(MoveCmd("pinky1", 7, "N", 1.0, 2.0, math.pi / 2, path))
    assert [(p.position.x, p.position.y) for p in move.path] == [(0.5, 1.0), (1.0, 1.0),
                                                                    (1.0, 2.0)]
    q = move.path[1].orientation
    assert (q.z, q.w) == pytest.approx((math.sin(math.pi / 4), math.cos(math.pi / 4)))
    assert move.path[0].orientation.w == pytest.approx(1.0)


def test_a_move_goal_says_whether_its_end_is_the_last_point():
    """버전 2 horizon 최소판(계약 0.8-draft 제안)."""
    assert adapters.move_goal(MoveCmd("pinky1", 7, "N", 1.0, 2.0, 0.0)).final_goal is True
    move = adapters.move_goal(MoveCmd("pinky1", 7, "N", 1.0, 2.0, 0.0, (), False))
    assert move.final_goal is False


def test_fill_goal_copies_every_field():
    goal = adapters.fill_goal(FillCmd(9, "pinky1", "WORK", "O-001", {"A": 2, "C": 1}))
    assert (goal.order_id, goal.robot_id, goal.station_id) == ("O-001", "pinky1", "WORK")
    assert {i.kind: i.count for i in goal.items} == {"A": 2, "C": 1}


@pytest.mark.parametrize("take_full", [True, False])
def test_swap_goal_copies_every_field(take_full):
    goal = adapters.swap_goal(SwapCmd("arm3", 9, "pinky1", "OUTBOUND", "O-001", take_full, True))
    assert (goal.order_id, goal.robot_id, goal.station_id) == ("O-001", "pinky1", "OUTBOUND")
    assert (goal.take_full, goal.put_empty) == (take_full, True)


def test_report_request_copies_every_field():
    request = adapters.report_request(ReportOrderCmd(
        "O-001", "pinky1", ReportOrder.Request.STATUS_PARTIAL, {"A": 1}, {"C": 1}, "short"))
    assert (request.order_id, request.robot_id, request.status, request.detail) == (
        "O-001", "pinky1", "PARTIAL", "short")
    assert {i.kind: i.count for i in request.placed} == {"A": 1}
    assert {i.kind: i.count for i in request.missing} == {"C": 1}


def test_item_lists_add_up_a_kind_named_twice():
    items = [OrderItem(kind="A", count=1), OrderItem(kind="A", count=2)]
    assert adapters.from_order_items(items) == {"A": 3}


def test_robot_report_reads_every_field_the_core_needs():
    msg = RobotState(state=RobotState.STATE_MOVING, battery=87.5)
    msg.pose.x, msg.pose.y, msg.pose.theta = 1.0, 2.0, 0.5
    assert adapters.robot_report(msg) == (1.0, 2.0, 0.5, RobotState.STATE_MOVING, 87.5)


def test_a_self_recovering_robot_reaches_the_core_and_the_status_as_it_is():
    """로봇 상태 RECOVERING(자가 복구, 제안)은 그대로 지나간다. fault는 NONE이다."""
    msg = RobotState(state=RobotState.STATE_RECOVERING)
    assert adapters.robot_report(msg)[3] == SELF_RECOVERING
    view = RobotStatusView("pinky1", SELF_RECOVERING, "TO_WORK", "O-001", "NONE", "", "",
                           (), 50.0, 0.0, 0.0, 0.0)
    status = adapters.robots_to_msg([view], 0.0).robots[0]
    assert (status.fsm_state, status.fault) == ("RECOVERING", "NONE")


def test_every_field_of_the_robot_status_reaches_the_message():
    view = RobotStatusView("pinky2", "MOVING", "TO_OUTBOUND", "O-003", "ERROR", "DOCK_FAILED",
                           "AVENUE", ("pinky1", "pinky3"), 87.5, 1.5, 2.5, 0.25)
    msg = adapters.robots_to_msg([view], 12.5)
    assert msg.stamp.sec == 12 and msg.stamp.nanosec == 500_000_000
    out = msg.robots[0]
    assert (out.robot_id, out.fsm_state, out.task_type, out.order_id) == (
        "pinky2", "MOVING", "TO_OUTBOUND", "O-003")
    assert (out.fault, out.fault_reason) == (RobotStatus.FAULT_ERROR, "DOCK_FAILED")
    assert (out.wait_zone, list(out.wait_robots)) == ("AVENUE", ["pinky1", "pinky3"])
    assert (out.battery, out.x, out.y, out.yaw) == (87.5, 1.5, 2.5, 0.25)



def test_action_results_are_read_here_and_not_in_the_node():
    move = MoveTo.Result(success=False, reason=MoveTo.Result.REASON_TIMEOUT)
    assert adapters.move_result(move) == (False, MoveTo.Result.REASON_TIMEOUT)
    dock = Dock.Result(success=True, error_x_cm=1.0, error_y_cm=2.0, error_yaw_deg=3.0)
    assert adapters.dock_result(dock) == (True, 1.0, 2.0, 3.0)
    fill = FillBasket.Result(success=False, placed=[OrderItem(kind="A", count=1)],
                             missing=[OrderItem(kind="C", count=2)], reason="GRIPPER")
    assert adapters.fill_result(fill) == (False, {"A": 1}, {"C": 2}, "GRIPPER")
    swap = SwapBasket.Result(success=False, took_full=True, put_empty_done=False, reason="x")
    assert adapters.swap_result(swap) == (False, True, False, "x")


def test_order_board_mode_is_read_here_too():
    msg = OrderBoard(run_mode=OrderBoard.MODE_PAUSE)
    assert adapters.order_board_mode(msg) == OrderBoard.MODE_PAUSE


def test_confirm_clear_request_and_response_are_handled_here():
    request = ConfirmClear.Request(robot_id="pinky1", zone_ids=["Z-OUT"], note="moved by hand")
    assert adapters.confirm_clear_args(request) == ("pinky1", ["Z-OUT"], "moved by hand")
    response = adapters.confirm_clear_response(ConfirmClear.Response(), ["Z-OUT"])
    assert response.accepted is True and list(response.released_zones) == ["Z-OUT"]


def test_confirm_clear_with_no_zones_is_still_accepted():
    """한 번도 보고하지 않은 로봇을 운행 제외해도 풀 것이 없으며 정상이다."""
    assert adapters.confirm_clear_response(ConfirmClear.Response(), []).accepted is True


def test_confirm_clear_for_an_unknown_robot_is_refused():
    response = adapters.confirm_clear_response(ConfirmClear.Response(), None)
    assert response.accepted is False and list(response.released_zones) == []


@pytest.mark.parametrize("accepted", [True, False])
def test_the_reply_to_a_report_is_read_here_too(accepted):
    assert adapters.report_accepted(ReportOrder.Response(accepted=accepted)) is accepted


def _finished(action, status, **fields):
    return action.Impl.GetResultService.Response(status=status, result=action.Result(**fields))


@pytest.mark.parametrize("action", [MoveTo, FillBasket, SwapBasket])
def test_a_canceled_goal_with_no_reason_reads_as_canceled(action):
    result = adapters.goal_result(_finished(action, GoalStatus.STATUS_CANCELED, success=False))
    assert result.reason == "CANCELED"


def test_an_aborted_goal_with_no_reason_reads_as_aborted():
    result = adapters.goal_result(_finished(FillBasket, GoalStatus.STATUS_ABORTED))
    assert result.reason == "ABORTED"


@pytest.mark.parametrize("status", [GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED])
def test_the_reason_of_the_server_stays(status):
    result = adapters.goal_result(_finished(SwapBasket, status, reason="GRIPPER"))
    assert result.reason == "GRIPPER"


def test_a_succeeded_goal_gets_no_reason():
    result = adapters.goal_result(_finished(MoveTo, GoalStatus.STATUS_SUCCEEDED, success=True))
    assert result.reason == ""


def test_the_rest_of_the_result_comes_through_with_the_status():
    response = _finished(FillBasket, GoalStatus.STATUS_CANCELED,
                         placed=adapters.to_order_items({"A": 1}))
    assert adapters.fill_result(adapters.goal_result(response)) == (
        False, {"A": 1}, {}, "CANCELED")


@pytest.mark.parametrize("status", [GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED,
                                    GoalStatus.STATUS_SUCCEEDED])
def test_a_result_with_no_reason_field_keeps_working(status):
    result = adapters.goal_result(_finished(Dock, status, success=True, error_x_cm=1.5))
    assert adapters.dock_result(result) == (True, 1.5, 0.0, 0.0)


def test_the_arm_actions_name_the_same_cancel_reason_the_core_looks_for():
    assert FillBasket.Result.REASON_CANCELED == CANCELED
    assert SwapBasket.Result.REASON_CANCELED == CANCELED
    assert MoveTo.Result.REASON_CANCELED == CANCELED


def test_the_robot_values_the_core_reads_are_the_message_constants():
    """이름이 어긋나면 관제가 BLOCKED, LOCALIZING, HEARTBEAT_LOST를 못 알아본다(계약 3·4절)."""
    assert RobotState.STATE_BLOCKED == BLOCKED
    assert RobotState.STATE_LOCALIZING == LOCALIZING
    assert RobotState.STATE_RECOVERING == SELF_RECOVERING
    assert MoveTo.Result.REASON_HEARTBEAT_LOST == HEARTBEAT_LOST


# ---- 잘못된 입력(11-06 1번) ----


def test_robot_report_id_is_read_separately():
    assert adapters.robot_report_id(RobotState(robot_id="pinky2")) == "pinky2"


def test_a_count_of_zero_or_less_is_not_hidden_by_adding_up():
    """합치면 A: 2와 A: -1이 1이 되어 검사를 빠져나간다."""
    board = OrderBoard(run_mode=OrderBoard.MODE_RUN, orders=[
        Order(order_id="O-1", items=[OrderItem(kind="A", count=2), OrderItem(kind="A", count=-1)]),
        Order(order_id="O-2", items=[OrderItem(kind="B", count=0)]),
    ])
    assert adapters.order_board_orders(board) == [
        OrderRecord("O-1", {"A": -1}), OrderRecord("O-2", {"B": 0})]


def test_a_refused_clear_carries_its_reason():
    response = adapters.confirm_clear_response(ConfirmClear.Response(), None, "no such zone")
    assert response.accepted is False and response.reason == "no such zone"
    accepted = adapters.confirm_clear_response(ConfirmClear.Response(), ["P-6"], "")
    assert accepted.accepted is True and accepted.reason == ""
