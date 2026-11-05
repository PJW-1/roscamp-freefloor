"""The node against hand-written servers on a real graph: names, types, goals and cancels.

No fake robots here: this checks the wrapper's plumbing, not the scenarios. It joins a graph,
so it runs only when the caller has isolated it from the robot network.
"""
import os
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces")

from fleet_core.contract import to_ros_qos  # noqa: E402
from fleet_core.site_map import load_site_map  # noqa: E402
from fleet_interfaces.action import Dock, FillBasket, MoveTo, SwapBasket  # noqa: E402
from fleet_interfaces.msg import (  # noqa: E402
    AssignmentArray,
    Order,
    OrderBoard,
    OrderItem,
    RobotState,
)
from fleet_interfaces.srv import ReportOrder  # noqa: E402
from fleet_manager.fms_node import FmsNode  # noqa: E402
from rclpy.action import ActionServer, CancelResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402

pytestmark = pytest.mark.skipif(
    os.environ.get("ROS_DOMAIN_ID", "") in ("", "6")
    or os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST",
    reason="set ROS_DOMAIN_ID (not 6) and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST first")


def items(counts):
    return [OrderItem(kind=k, count=c) for k, c in counts.items()]


class World(Node):
    """One robot that does what it is told at once, the two arm servers, and central."""

    def __init__(self, contract, site, cell):
        super().__init__("wiring_world")
        group = ReentrantCallbackGroup()
        self.site = site
        self.pose = site.cell_pose(cell)
        self.fills = []
        self.swaps = []
        self.reports = []
        self.assignments = []
        self.hold_fill = threading.Event()  # while set, a fill runs until it is canceled
        self.canceled_fills = []
        name = contract.name
        self.board = self.create_publisher(
            OrderBoard, name("order_board"), to_ros_qos(contract.qos("order_board")))
        self.state = self.create_publisher(
            RobotState, name("robot_state", pinky="pinky1"),
            to_ros_qos(contract.qos("robot_state")))
        self.create_subscription(
            AssignmentArray, name("assignments"),
            lambda m: self.assignments.append(list(m.assignments)),
            to_ros_qos(contract.qos("assignments")))
        self.create_timer(0.02, self._publish_state, callback_group=group)
        self.create_service(ReportOrder, name("report_order"), self._on_report)
        self._servers = [
            ActionServer(self, MoveTo, name("move_to", pinky="pinky1"), self._on_move,
                         callback_group=group),
            ActionServer(self, Dock, name("dock", pinky="pinky1"), self._on_dock,
                         callback_group=group),
            ActionServer(self, FillBasket, name("fill_basket"), self._on_fill,
                         callback_group=group, cancel_callback=lambda _: CancelResponse.ACCEPT),
            ActionServer(self, SwapBasket, name("swap_basket", arm="arm3"), self._on_swap,
                         callback_group=group),
        ]

    def _publish_state(self):
        msg = RobotState(robot_id="pinky1", state=RobotState.STATE_IDLE)
        msg.pose.x, msg.pose.y = self.pose
        self.state.publish(msg)

    def _on_move(self, handle):
        self.pose = (handle.request.target_pose.x, handle.request.target_pose.y)
        handle.succeed()
        return MoveTo.Result(success=True)

    def _on_dock(self, handle):
        handle.succeed()
        return Dock.Result(success=True)

    def _on_fill(self, handle):
        wanted = {i.kind: i.count for i in handle.request.items}
        self.fills.append((handle.request.order_id, handle.request.robot_id,
                           handle.request.station_id, wanted))
        while self.hold_fill.is_set():
            if handle.is_cancel_requested:
                first = sorted(wanted)[0]
                handle.canceled()
                self.canceled_fills.append(handle.request.order_id)
                return FillBasket.Result(success=False, placed=items({first: 1}),
                                         reason="CANCELED")
            time.sleep(0.01)
        handle.succeed()
        return FillBasket.Result(success=True, placed=items(wanted))

    def _on_swap(self, handle):
        self.swaps.append((handle.request.order_id, handle.request.take_full,
                           handle.request.put_empty))
        handle.succeed()
        return SwapBasket.Result(success=True, took_full=handle.request.take_full,
                                 put_empty_done=True)

    def _on_report(self, request, response):
        self.reports.append(request)
        response.accepted = True
        return response


@pytest.fixture
def graph():
    rclpy.init(args=["--ros-args", "-p", "robots:=[pinky1]", "-p", "tick_hz:=50.0"])
    node = FmsNode()
    world = World(node.contract, load_site_map(), "Q")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(world)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    yield world
    executor.shutdown()
    thread.join(timeout=5)
    world.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


def publish(world, mode, counts=None):
    orders = [Order(order_id="O-001", items=items(counts))] if counts else []
    world.board.publish(OrderBoard(run_mode=mode, orders=orders))


def wait_for(predicate, timeout_s=40.0):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def test_an_order_goes_through_fill_and_swap_to_its_report(graph):
    publish(graph, "RUN", {"A": 1, "C": 2})
    assert wait_for(lambda: graph.reports), f"fills {graph.fills}, swaps {graph.swaps}"
    report = graph.reports[0]
    assert (report.order_id, report.robot_id, report.status) == ("O-001", "pinky1", "SHIPPED")
    assert {i.kind: i.count for i in report.placed} == {"A": 1, "C": 2}
    assert graph.fills == [("O-001", "pinky1", "WORK", {"A": 1, "C": 2})]
    assert graph.swaps == [("O-001", True, True)]
    seen = [(a.order_id, a.station_id, a.task_type) for array in graph.assignments for a in array]
    assert ("O-001", "WORK", "FILL") in seen
    assert ("O-001", "OUTBOUND", "SWAP") in seen


def test_stop_cancels_the_fill_on_its_server_and_run_sends_what_remains(graph):
    graph.hold_fill.set()
    publish(graph, "RUN", {"A": 1, "C": 2})
    assert wait_for(lambda: graph.fills), "the fill never reached the server"
    publish(graph, "STOP", {"A": 1, "C": 2})
    assert wait_for(lambda: graph.canceled_fills), "the cancel never reached the server"
    graph.hold_fill.clear()
    publish(graph, "RUN", {"A": 1, "C": 2})
    assert wait_for(lambda: graph.reports), f"fills {graph.fills}"
    assert graph.fills[1][3] == {"C": 2}, "A was placed before the stop"
    assert graph.reports[0].status == "SHIPPED"
