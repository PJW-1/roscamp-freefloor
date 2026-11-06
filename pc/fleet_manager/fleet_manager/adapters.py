"""Conversions between fleet_core records and ROS messages.

When a message field changes after the team agrees on the interfaces, this file is the
only place in the fleet manager that needs editing.
"""
from __future__ import annotations

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from fleet_core.common.commands import OrderRecord
from fleet_interfaces.action import Dock, FillBasket, MoveTo, SwapBasket
from fleet_interfaces.msg import (
    Assignment,
    AssignmentArray,
    FleetEvent,
    OrderBoard,
    OrderItem,
    RobotStatus,
    RobotStatusArray,
    ZoneState,
    ZoneStateArray,
)
from fleet_interfaces.srv import ReportOrder

SOURCE = "fleet_manager"
REASON_OF_STATUS = {
    GoalStatus.STATUS_CANCELED: MoveTo.Result.REASON_CANCELED,
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


def to_time(seconds: float) -> Time:
    sec, nanosec = divmod(int(round(seconds * 1e9)), 1_000_000_000)
    return Time(sec=sec, nanosec=nanosec)


def to_order_items(items: dict[str, int]) -> list[OrderItem]:
    return [OrderItem(kind=kind, count=int(count)) for kind, count in items.items()]


def from_order_items(items) -> dict[str, int]:
    counts: dict[str, int] = {}
    for item in items:
        counts[item.kind] = counts.get(item.kind, 0) + int(item.count)
    return counts


def zones_to_msg(views, now: float) -> ZoneStateArray:
    zones = []
    for view in views:
        zone = ZoneState(zone_id=view.zone_id, holder=view.holder or "", queue=list(view.queue))
        if view.acquired_at is not None:
            zone.acquired_at = to_time(view.acquired_at)
        zones.append(zone)
    return ZoneStateArray(zones=zones, stamp=to_time(now))


def assignments_to_msg(views, now: float) -> AssignmentArray:
    items = [
        Assignment(robot_id=v.robot_id, order_id=v.order_id or "",
                   station_id=v.station_id or "", task_type=v.task_type)
        for v in views
    ]
    return AssignmentArray(assignments=items, stamp=to_time(now))


def event_to_msg(event, now: float) -> FleetEvent:
    return FleetEvent(
        level=event.level,
        source=event.robot or SOURCE,
        code=event.code,
        detail=event.detail,
        stamp=to_time(now),
    )


def robots_to_msg(views, now: float) -> RobotStatusArray:
    robots = [
        RobotStatus(robot_id=v.robot_id, fsm_state=v.fsm_state, task_type=v.task_type,
                    order_id=v.order_id, fault=v.fault, fault_reason=v.fault_reason,
                    wait_zone=v.wait_zone, wait_robots=list(v.wait_robots),
                    battery=float(v.battery), x=float(v.x), y=float(v.y), yaw=float(v.yaw))
        for v in views
    ]
    return RobotStatusArray(robots=robots, stamp=to_time(now))


def robot_report(msg) -> tuple[float, float, float, str, float]:
    return msg.pose.x, msg.pose.y, msg.pose.theta, msg.state, msg.battery


def goal_result(response):
    """The result of a finished goal. A server may say how it ended by the status alone."""
    result = response.result
    if getattr(result, "reason", None) == "":
        result.reason = REASON_OF_STATUS.get(response.status, "")
    return result


def move_result(result) -> tuple[bool, str]:
    return result.success, result.reason


def dock_result(result) -> tuple[bool, float, float, float]:
    return result.success, result.error_x_cm, result.error_y_cm, result.error_yaw_deg


def fill_result(result) -> tuple[bool, dict[str, int], dict[str, int], str]:
    return (result.success, from_order_items(result.placed), from_order_items(result.missing),
            result.reason)


def swap_result(result) -> tuple[bool, bool, bool, str]:
    return result.success, result.took_full, result.put_empty_done, result.reason


def confirm_clear_args(request) -> tuple[str, list[str], str]:
    return request.robot_id, list(request.zone_ids), request.note


def confirm_clear_response(response, released: list[str] | None):
    response.accepted = released is not None
    response.released_zones = list(released or [])
    return response


def order_board_mode(msg: OrderBoard) -> str:
    return msg.run_mode


def order_board_orders(msg: OrderBoard) -> list[OrderRecord]:
    return [OrderRecord(o.order_id, from_order_items(o.items)) for o in msg.orders]


def move_goal(cmd) -> MoveTo.Goal:
    goal = MoveTo.Goal(target_id=cmd.target_id)
    goal.target_pose.x = float(cmd.x)
    goal.target_pose.y = float(cmd.y)
    goal.target_pose.theta = float(cmd.yaw)
    return goal


def dock_goal(cmd) -> Dock.Goal:
    return Dock.Goal(dock_id=cmd.dock_id)


def fill_goal(cmd) -> FillBasket.Goal:
    return FillBasket.Goal(order_id=cmd.order_id, robot_id=cmd.robot, station_id=cmd.station_id,
                           items=to_order_items(cmd.items))


def swap_goal(cmd) -> SwapBasket.Goal:
    return SwapBasket.Goal(order_id=cmd.order_id, robot_id=cmd.robot, station_id=cmd.station_id,
                           take_full=bool(cmd.take_full), put_empty=bool(cmd.put_empty))


def report_accepted(response) -> bool:
    return response.accepted


def report_request(cmd) -> ReportOrder.Request:
    return ReportOrder.Request(
        order_id=cmd.order_id,
        robot_id=cmd.robot,
        status=cmd.status,
        placed=to_order_items(cmd.placed),
        missing=to_order_items(cmd.missing),
        detail=cmd.detail,
    )
