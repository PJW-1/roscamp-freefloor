"""The FMS process: traffic_core and orchestrator_core behind one ROS node.

All decisions are made in fleet_core. This node forwards messages in and turns the commands
into goals. Callbacks run on one executor thread, so the cores never see concurrent calls.
Names and QoS come from contract.yaml, coordinates from the site map, and policy values from
fleet_params.yaml.
"""
from __future__ import annotations

from functools import partial

import rclpy
from ament_index_python.packages import get_package_share_directory
from fleet_core.commands import (
    CancelCmd,
    DockCmd,
    EventOut,
    FillCmd,
    MoveCmd,
    ReportOrderCmd,
    SwapCmd,
)
from fleet_core.contract import load_contract, to_ros_qos
from fleet_core.fms_core import FmsCore
from fleet_core.params import load_params
from fleet_core.site_map import load_site_map
from fleet_interfaces.action import Dock, FillBasket, MoveTo, SwapBasket
from fleet_interfaces.msg import (
    AssignmentArray,
    FleetEvent,
    OrderBoard,
    RobotState,
    ZoneStateArray,
)
from fleet_interfaces.srv import ConfirmClear, ReportOrder
from rclpy.action import ActionClient
from rclpy.node import Node

from fleet_manager import adapters
from fleet_manager.goal_tracker import GoalTracker, ReportOutbox


class FmsNode(Node):
    def __init__(self):
        super().__init__("fms_node")
        self.declare_parameter("robots", ["pinky1", "pinky2", "pinky3"])
        self.declare_parameter("contract_file", "")
        self.declare_parameter("map_file", "")
        self.declare_parameter("params_file", "")
        self.declare_parameter("tick_hz", 10.0)
        robots = list(self.get_parameter("robots").value)

        contract_file = self.get_parameter("contract_file").value or (
            get_package_share_directory("fleet_interfaces") + "/contract.yaml")
        self.contract = load_contract(contract_file)
        self.site = load_site_map(self.get_parameter("map_file").value or None)
        self.core = FmsCore(
            self.site, load_params(self.get_parameter("params_file").value or None),
            robots, self._now)

        self._goals = GoalTracker()
        self._reports = ReportOutbox()
        self._last_zones = None
        self._last_assignments = None
        self._create_interfaces(robots)
        self.create_timer(1.0 / self.get_parameter("tick_hz").value, self._on_tick)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _qos(self, key):
        return to_ros_qos(self.contract.qos(key))

    def _create_interfaces(self, robots) -> None:
        name = self.contract.name
        self._zones_pub = self.create_publisher(
            ZoneStateArray, name("zones"), self._qos("zones"))
        self._assignments_pub = self.create_publisher(
            AssignmentArray, name("assignments"), self._qos("assignments"))
        self._events_pub = self.create_publisher(FleetEvent, name("events"), self._qos("events"))
        self.create_subscription(
            OrderBoard, name("order_board"), self._on_order_board, self._qos("order_board"))
        self._move = {}
        self._dock = {}
        for robot in robots:
            self.create_subscription(
                RobotState, name("robot_state", pinky=robot),
                partial(self._on_robot_state, robot), self._qos("robot_state"))
            self._move[robot] = ActionClient(self, MoveTo, name("move_to", pinky=robot))
            self._dock[robot] = ActionClient(self, Dock, name("dock", pinky=robot))
        self._fill = {}
        self._swap = {}
        for station in self.site.stations.values():
            if station.action == "fill":
                self._fill[station.id] = ActionClient(self, FillBasket, name("fill_basket"))
            else:
                self._swap[station.id] = ActionClient(
                    self, SwapBasket, name("swap_basket", arm=station.server))
        self._report_client = self.create_client(ReportOrder, name("report_order"))
        self.create_service(ConfirmClear, name("confirm_clear"), self._on_confirm_clear)

    # ------------------------------------------------------------------ inputs
    def _on_robot_state(self, robot, msg: RobotState) -> None:
        self.core.on_robot_state(robot, *adapters.robot_report(msg))

    def _on_order_board(self, msg: OrderBoard) -> None:
        self.core.on_order_board(adapters.order_board_mode(msg), adapters.order_board_orders(msg))

    def _on_confirm_clear(self, request, response):
        released = self.core.confirm_clear(*adapters.confirm_clear_args(request))
        return adapters.confirm_clear_response(response, released)

    # ------------------------------------------------------------------ output loop
    def _on_tick(self) -> None:
        now = self._now()
        if now <= 0.0:
            return  # waiting for the first /clock message under simulated time
        for command in self.core.tick():
            self._dispatch(command, now)
        self._flush_reports()
        self._publish_snapshots(now)

    def _flush_reports(self) -> None:
        for command in self._reports.drain(self._report_client.service_is_ready()):
            self._report_client.call_async(adapters.report_request(command))

    def _publish_snapshots(self, now: float) -> None:
        zones = self.core.zones_view()
        if zones != self._last_zones:
            self._last_zones = zones
            self._zones_pub.publish(adapters.zones_to_msg(zones, now))
        assignments = self.core.assignments_view()
        if assignments != self._last_assignments:
            self._last_assignments = assignments
            self._assignments_pub.publish(adapters.assignments_to_msg(assignments, now))

    def _dispatch(self, command, now: float) -> None:
        if isinstance(command, MoveCmd):
            self._send(self._move[command.robot], adapters.move_goal(command),
                       (command.robot, command.goal_id), self._move_done, self._robot_rejected)
        elif isinstance(command, DockCmd):
            self._send(self._dock[command.robot], adapters.dock_goal(command),
                       (command.robot, command.goal_id), self._dock_done, self._robot_rejected)
        elif isinstance(command, FillCmd):
            server = self.site.stations[command.station_id].server
            self._send(self._fill[command.station_id], adapters.fill_goal(command),
                       (server, command.goal_id), self._fill_done, self._fill_rejected)
        elif isinstance(command, SwapCmd):
            self._send(self._swap[command.station_id], adapters.swap_goal(command),
                       (command.arm, command.goal_id), self._swap_done, self._swap_rejected)
        elif isinstance(command, CancelCmd):
            self._goals.cancel((command.robot, command.goal_id))
        elif isinstance(command, ReportOrderCmd):
            self._reports.add(command)
        elif isinstance(command, EventOut):
            self._events_pub.publish(adapters.event_to_msg(command, now))
            self.get_logger().info(f"{command.level} {command.code} {command.detail}")

    # ------------------------------------------------------------------ action plumbing
    def _send(self, client, goal, key, on_result, on_rejected) -> None:
        if not client.server_is_ready():
            on_rejected(key)
            return
        self._goals.sent(key)
        future = client.send_goal_async(goal)
        future.add_done_callback(partial(self._goal_response, key, on_result, on_rejected))

    def _goal_response(self, key, on_result, on_rejected, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # a broken future must not leave the core waiting forever
            self.get_logger().error(f"goal {key} failed to send: {exc}")
            self._goals.finished(key)
            on_rejected(key)
            return
        if not handle.accepted:
            self._goals.finished(key)
            on_rejected(key)
            return
        self._goals.accepted(key, handle)
        handle.get_result_async().add_done_callback(
            partial(self._goal_result, key, on_result, on_rejected))

    def _goal_result(self, key, on_result, on_rejected, future) -> None:
        self._goals.finished(key)
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().error(f"goal {key} lost its result: {exc}")
            on_rejected(key)
            return
        on_result(key, result)

    def _robot_rejected(self, key) -> None:
        self.core.on_goal_rejected(key[0], key[1])

    def _fill_rejected(self, key) -> None:
        self.core.on_fill_result(key[1], False, {}, {}, "REJECTED")

    def _swap_rejected(self, key) -> None:
        self.core.on_swap_result(key[1], False, False, False, "REJECTED")

    def _move_done(self, key, result) -> None:
        self.core.on_move_result(key[0], key[1], *adapters.move_result(result))

    def _dock_done(self, key, result) -> None:
        self.core.on_dock_result(key[0], key[1], *adapters.dock_result(result))

    def _fill_done(self, key, result) -> None:
        self.core.on_fill_result(key[1], *adapters.fill_result(result))

    def _swap_done(self, key, result) -> None:
        self.core.on_swap_result(key[1], *adapters.swap_result(result))


def main() -> None:
    rclpy.init()
    node = FmsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:  # SIGINT can land inside a message take; a real error has ROS still up
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
