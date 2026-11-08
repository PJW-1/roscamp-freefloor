"""FMS 프로세스. fleet_core의 교통·주문 모듈을 ROS 노드 하나로 감싼다.

판단은 모두 fleet_core가 한다. 이 노드는 메시지를 넘기고 명령을 goal로 바꾼다.
콜백은 executor 스레드 하나에서 돌아 코어는 동시 호출을 받지 않는다.
이름과 QoS는 contract.yaml, 좌표는 맵, 정책 값은 fleet_params.yaml에서 온다.
"""
from __future__ import annotations

import argparse
import time
from functools import partial

import rclpy
from ament_index_python.packages import get_package_share_directory
from fleet_core.common.commands import (
    CancelCmd,
    DockCmd,
    EventOut,
    FillCmd,
    MoveCmd,
    ReportOrderCmd,
    SwapCmd,
)
from fleet_core.common.contract import load_contract, to_ros_qos
from fleet_core.common.params import load_params
from fleet_core.manager import FleetManager
from fleet_core.traffic.lane_map import load_lane_map
from fleet_interfaces.action import Dock, FillBasket, MoveTo, SwapBasket
from fleet_interfaces.msg import (
    AssignmentArray,
    FleetEvent,
    OrderBoard,
    RobotState,
    RobotStatusArray,
    ZoneStateArray,
)
from fleet_interfaces.srv import ConfirmClear, Reinstate, ReportOrder
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Header

from fleet_manager import adapters
from fleet_manager.goal_tracker import GoalTracker, ReportOutbox


class FleetManagerNode(Node):
    def __init__(self, record=None):
        super().__init__("fleet_manager")
        self.declare_parameter("robots", ["pinky1", "pinky2", "pinky3"])
        self.declare_parameter("contract_file", "")
        self.declare_parameter("map_file", "")
        self.declare_parameter("params_file", "")
        self.declare_parameter("tick_hz", 10.0)
        robots = list(self.get_parameter("robots").value)

        contract_file = self.get_parameter("contract_file").value or (
            get_package_share_directory("fleet_interfaces") + "/contract.yaml")
        self.contract = load_contract(contract_file)
        self.site = load_lane_map(self.get_parameter("map_file").value or None)
        self.core = FleetManager(
            self.site, load_params(self.get_parameter("params_file").value or None),
            robots, self._now)
        self._record_file = None
        if record:
            self.core.check_recordable()  # 거부될 기록이 옛 파일을 지우지 않게 먼저 본다
            # 한 줄씩 써서, 프로세스가 죽어도 기록은 그 자리에서 끝난다.
            self._record_file = open(record, "w", buffering=1, encoding="utf-8")
            self.core.record_to(lambda line: self._record_file.write(line + "\n"))

        self._goals = GoalTracker()
        self._reports = ReportOutbox()
        self._reached = set()
        self._last_zones = None
        self._last_assignments = None
        self._create_interfaces(robots)
        self.create_timer(1.0 / self.get_parameter("tick_hz").value, self._on_tick)
        self.create_timer(1.0, self._publish_robots)
        params = self.core.params
        # 이전 FMS의 goal을 수행 중인 로봇은 heartbeat가 timeout만큼 끊기면 멈춘다.
        # 그때까지 이 FMS는 아무것도 보내지 않고 보고도 받지 않는다. 실제 시계를 쓴다.
        # 프로세스를 보는 것이고, 시뮬레이션 시간은 부하 걸린 머신의 메시지보다 앞서가기 때문이다.
        self._silent_until = (time.monotonic() + params.heartbeat_timeout_s
                              + params.heartbeat_period_s)
        self.create_timer(params.heartbeat_period_s, self._publish_heartbeat,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

    def destroy_node(self) -> None:
        if self._record_file:
            self._record_file.close()
        super().destroy_node()

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def silent(self) -> bool:
        return time.monotonic() < self._silent_until

    def _qos(self, key):
        return to_ros_qos(self.contract.qos(key))

    def _create_interfaces(self, robots) -> None:
        name = self.contract.name
        self._zones_pub = self.create_publisher(
            ZoneStateArray, name("zones"), self._qos("zones"))
        self._assignments_pub = self.create_publisher(
            AssignmentArray, name("assignments"), self._qos("assignments"))
        self._robots_pub = self.create_publisher(
            RobotStatusArray, name("robots"), self._qos("robots"))
        self._heartbeat_pub = self.create_publisher(
            Header, name("heartbeat"), self._qos("heartbeat"))
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
        self.create_service(Reinstate, name("reinstate"), self._on_reinstate)

    # ------------------------------------------------------------------ inputs
    def _on_robot_state(self, robot, msg: RobotState) -> None:
        if not self.silent() and self._reachable(robot):
            self.core.on_robot_state(robot, *adapters.robot_report(msg),
                                     adapters.robot_report_id(msg))

    def _reachable(self, robot) -> bool:
        """goal이 닿을 수 있어야 로봇으로 친다. 새 FMS는 액션 서버를 찾기 전에 보고를 먼저
        듣는데, 그때 보낸 goal은 거절되어 재시도만 소모한다."""
        if (robot not in self._reached and self._move[robot].server_is_ready()
                and self._dock[robot].server_is_ready()):
            self._reached.add(robot)
        return robot in self._reached

    def _on_order_board(self, msg: OrderBoard) -> None:
        self.core.on_order_board(adapters.order_board_mode(msg), adapters.order_board_orders(msg))

    def _on_confirm_clear(self, request, response):
        released = self.core.confirm_clear(*adapters.confirm_clear_args(request))
        return adapters.confirm_clear_response(response, released, self.core.last_clear_reason)

    def _on_reinstate(self, request, response):
        accepted = self.core.reinstate(*adapters.reinstate_args(request))
        return adapters.reinstate_response(response, accepted, self.core.last_reinstate_reason)

    # ------------------------------------------------------------------ output loop
    def _on_tick(self) -> None:
        now = self._now()
        if now <= 0.0 or self.silent():
            return  # 시뮬레이션 시간에서 첫 /clock이 아직 안 왔다
        for command in self.core.tick():
            self._dispatch(command, now)
        self._retry_cancels(now)
        self._flush_reports(now)
        self._publish_snapshots(now)

    def _retry_cancels(self, now: float) -> None:
        params = self.core.params
        for server, goal_id in self._goals.retry_cancels(now, params.cancel_retry_s,
                                                         params.cancel_warn_after):
            waited = params.cancel_warn_after * params.cancel_retry_s
            # "다시 요청"이 아니다. 수락된 적 없는 goal에는 취소를 보낼 핸들이 없다.
            detail = (f"goal {goal_id} on {server} has no result {waited:.0f} s after it was "
                      "canceled; the cancel stays open while a result can still come")
            self._dispatch(EventOut("WARN", "CANCEL_UNCONFIRMED", detail, server), now)
        for robot, goal_id in self._goals.take_dropped():
            # 결과를 잃은 취소는 로봇 goal뿐이다(_goal_result). 코어가 끝난 것으로 보고 이어 간다
            self.core.on_goal_lost(robot, goal_id)

    def _flush_reports(self, now: float) -> None:
        for report_id, command in self._reports.due(now, self._report_client.service_is_ready(),
                                                    self.core.params.report_retry_s):
            future = self._report_client.call_async(adapters.report_request(command))
            future.add_done_callback(partial(self._report_answered, report_id, command))

    def _report_answered(self, report_id, command, future) -> None:
        try:
            accepted = adapters.report_accepted(future.result())
        except Exception as exc:  # outbox에 남아 다시 나간다
            self.get_logger().warning(f"report of {command.order_id} got no reply: {exc}")
            return
        if self._reports.done(report_id) and not accepted:
            self._dispatch(EventOut("ERROR", "ORDER_REPORT_REFUSED",
                                    f"{command.order_id} {command.status} refused by central, "
                                    "nobody will work or report it now", command.robot),
                           self._now())

    def _publish_snapshots(self, now: float) -> None:
        zones = self.core.zones_view()
        # 전원이 보고하기 전에는 잠금을 모른다. 그것이 비어 있다는 뜻은 아니다.
        if self.core.traffic.everyone_placed and zones != self._last_zones:
            self._last_zones = zones
            self._zones_pub.publish(adapters.zones_to_msg(zones, now))
        assignments = self.core.assignments_view()
        if assignments != self._last_assignments:
            self._last_assignments = assignments
            self._assignments_pub.publish(adapters.assignments_to_msg(assignments, now))

    def _publish_heartbeat(self) -> None:
        # 전원이 맵에 오르기 전에는 아무도 안 움직여 heartbeat가 필요 없다. 늦게 수락된
        # 이전 FMS의 goal이 이 heartbeat를 자기 것으로 알고 계속 갈 수 있다.
        if not self.silent() and self.core.traffic.everyone_placed:
            self._heartbeat_pub.publish(Header(stamp=self.get_clock().now().to_msg(),
                                               frame_id=self.get_name()))

    def _publish_robots(self) -> None:
        now = self._now()
        if now > 0.0:
            self._robots_pub.publish(adapters.robots_to_msg(self.core.robots_view(), now))

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
            self._goals.cancel((command.robot, command.goal_id), now)
        elif isinstance(command, ReportOrderCmd):
            self._reports.add(command)
        elif isinstance(command, EventOut):
            self._events_pub.publish(adapters.event_to_msg(command, now))
            self._log_event(command)

    def _log_event(self, event) -> None:
        # 호출 위치마다 심각도 하나. rclpy는 같은 위치에서 심각도가 바뀌면 예외를 낸다.
        text = f"{event.code} {event.detail}"
        if event.level == "ERROR":
            self.get_logger().error(text)
        elif event.level == "WARN":
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)

    # ------------------------------------------------------------------ action plumbing
    def _send(self, client, goal, key, on_result, on_rejected) -> None:
        if not client.server_is_ready():
            if on_rejected == self._robot_rejected:
                self.core.on_goal_not_sent(key[0], key[1])  # 실패한 시도가 아니다
            else:
                on_rejected(key)
            return
        self._goals.sent(key)
        future = client.send_goal_async(goal)
        future.add_done_callback(partial(self._goal_response, key, on_result, on_rejected))

    def _goal_response(self, key, on_result, on_rejected, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # 깨진 future 때문에 코어가 영원히 기다리면 안 된다
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
        try:
            result = adapters.goal_result(future.result())
        except Exception as exc:
            self.get_logger().error(f"goal {key} lost its result: {exc}")
            if on_rejected == self._robot_rejected and self._goals.canceling(key):
                # 취소한 로봇 goal은 끝났는지 모른다. 취소를 남겨 CANCEL_UNCONFIRMED로 알리고,
                # 그만둘 때 코어에 on_goal_lost로 알린다(_retry_cancels)
                self._goals.lost(key)
                return
            self._goals.finished(key)
            on_rejected(key)
            return
        self._goals.finished(key)
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
    parser = argparse.ArgumentParser(description="The FMS node.")
    parser.add_argument("--record", default=None, metavar="PATH",
                        help="keep a record for tools/replay.py, one JSON line per input")
    args = parser.parse_args(remove_ros_args()[1:])
    rclpy.init()
    node = FleetManagerNode(record=args.record)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:  # SIGINT가 메시지 take 중에 걸릴 수 있다. 진짜 오류면 ROS가 아직 살아 있다
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
