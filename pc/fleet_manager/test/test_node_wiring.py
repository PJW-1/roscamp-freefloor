"""The node against hand-written servers on a real graph: names, types, goals and cancels.

No fake robots here: this checks the wrapper's plumbing, not the scenarios. It joins a graph,
so it runs only when the caller has isolated it from the robot network.
"""
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces")

from fleet_core.contract import to_ros_qos  # noqa: E402
from fleet_core.site_map import load_site_map  # noqa: E402
from fleet_interfaces.action import Dock, FillBasket, MoveTo, SwapBasket  # noqa: E402
from fleet_interfaces.msg import (  # noqa: E402
    AssignmentArray,
    FleetEvent,
    Order,
    OrderBoard,
    OrderItem,
    RobotState,
    RobotStatusArray,
    ZoneStateArray,
)
from fleet_interfaces.srv import ReportOrder  # noqa: E402
from fleet_manager.fms_node import FmsNode  # noqa: E402
from rclpy.action import ActionServer, CancelResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import Header  # noqa: E402

REPLAY = Path(__file__).parents[3] / "tools" / "replay.py"

pytestmark = pytest.mark.skipif(
    os.environ.get("ROS_DOMAIN_ID", "") in ("", "6")
    or os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST",
    reason="set ROS_DOMAIN_ID (not 6) and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST first")


def items(counts):
    return [OrderItem(kind=k, count=c) for k, c in counts.items()]


class World(Node):
    """One robot that does what it is told at once, the two arm servers, and central."""

    def __init__(self, contract, site, cell, robot="pinky1", robot_servers=True):
        super().__init__("wiring_world")
        self.robot = robot
        group = ReentrantCallbackGroup()
        self.site = site
        self.pose = site.cell_pose(cell)
        self.fills = []
        self.swaps = []
        self.reports = []
        self.assignments = []
        self.hold_fill = threading.Event()  # while set, a fill runs until it is canceled
        self.canceled_fills = []
        self.fill_cancel_reason = "CANCELED"
        self.refuse_fill_cancel = False
        self.fill_cancel_asks = 0
        name = contract.name
        self.events = []
        self.create_subscription(
            FleetEvent, name("events"), self.events.append, to_ros_qos(contract.qos("events")))
        self.board = self.create_publisher(
            OrderBoard, name("order_board"), to_ros_qos(contract.qos("order_board")))
        self.state = self.create_publisher(
            RobotState, name("robot_state", pinky=robot),
            to_ros_qos(contract.qos("robot_state")))
        self.create_subscription(
            AssignmentArray, name("assignments"),
            lambda m: self.assignments.append(list(m.assignments)),
            to_ros_qos(contract.qos("assignments")))
        self.zones = []
        self.create_subscription(
            ZoneStateArray, name("zones"), self.zones.append, to_ros_qos(contract.qos("zones")))
        self.status = []
        self.create_subscription(
            RobotStatusArray, name("robots"), self.status.append,
            to_ros_qos(contract.qos("robots")))
        self.beats = []
        self.create_subscription(
            Header, name("heartbeat"), lambda _m: self.beats.append(time.monotonic()),
            to_ros_qos(contract.qos("heartbeat")))
        self.create_timer(0.02, self._publish_state, callback_group=group)
        self.create_service(ReportOrder, name("report_order"), self._on_report)
        self._servers = [
            ActionServer(self, FillBasket, name("fill_basket"), self._on_fill,
                         callback_group=group, cancel_callback=self._on_fill_cancel),
            ActionServer(self, SwapBasket, name("swap_basket", arm="arm3"), self._on_swap,
                         callback_group=group),
        ]
        self._contract, self._group = contract, group
        if robot_servers:
            self.serve_robot()

    def serve_robot(self):
        name, group = self._contract.name, self._group
        self._servers += [
            ActionServer(self, MoveTo, name("move_to", pinky=self.robot), self._on_move,
                         callback_group=group),
            ActionServer(self, Dock, name("dock", pinky=self.robot), self._on_dock,
                         callback_group=group),
        ]

    def _publish_state(self):
        msg = RobotState(robot_id=self.robot, state=RobotState.STATE_IDLE)
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
                                         reason=self.fill_cancel_reason)
            time.sleep(0.01)
        handle.succeed()
        return FillBasket.Result(success=True, placed=items(wanted))

    def _on_fill_cancel(self, _handle):
        self.fill_cancel_asks += 1
        return CancelResponse.REJECT if self.refuse_fill_cancel else CancelResponse.ACCEPT

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
    world.fms = node
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(world)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    yield world
    world.hold_fill.clear()  # a test that failed must not leave a server thread spinning
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


def test_the_status_of_every_robot_comes_out_about_once_a_second(graph):
    graph.status.clear()
    assert wait_for(lambda: len(graph.status) >= 3), f"got {len(graph.status)}"
    first, last = graph.status[0], graph.status[-1]
    span = (last.stamp.sec - first.stamp.sec) + (last.stamp.nanosec - first.stamp.nanosec) / 1e9
    assert 0.7 <= span / (len(graph.status) - 1) <= 1.3
    robot = last.robots[0]
    assert (robot.robot_id, robot.fsm_state, robot.fault) == ("pinky1", "IDLE", "NONE")
    assert (robot.x, robot.y) != (0.0, 0.0)


def test_a_new_fms_stays_silent_until_any_goal_of_an_old_one_must_have_stopped(graph):
    """Read first, then check it is still silent: time only runs one way."""
    fms = graph.fms
    while True:
        on_map, beats = fms.core.traffic.robot_view("pinky1")["on_map"], list(graph.beats)
        if not fms.silent():
            break
        assert not on_map, "took a report during the silence"
        assert not beats, "a heartbeat during the silence"
        time.sleep(0.02)
    assert wait_for(lambda: fms.core.traffic.robot_view("pinky1")["on_map"], 10.0)


def test_a_robot_counts_only_once_its_goals_can_reach_it():
    """A new FMS hears reports before it has found the action servers. A goal sent then would be
    refused, and the retries would be gone in a moment. Until all count it says nothing about
    the locks either: it does not know them yet, and that is not the same as free. Nor does it
    beat: a goal of the FMS before this one would take that for its own and keep going.

    Not pinky1: a server of the tests before this one can linger in the graph for a moment.
    """
    rclpy.init(args=["--ros-args", "-p", "robots:=[pinky3]", "-p", "tick_hz:=50.0"])
    node = FmsNode()
    world = World(node.contract, load_site_map(), "Q", robot="pinky3", robot_servers=False)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(world)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert wait_for(lambda: not node.silent(), 20.0)
        time.sleep(1.0)
        assert not node.core.traffic.robot_view("pinky3")["on_map"]
        assert not world.zones
        assert not world.beats
        world.serve_robot()
        assert wait_for(lambda: node.core.traffic.robot_view("pinky3")["on_map"], 20.0)
        assert wait_for(lambda: world.zones, 5.0)
        assert wait_for(lambda: world.beats, 5.0)
    finally:
        executor.shutdown()
        thread.join(timeout=5)
        world.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def test_a_record_of_the_running_node_plays_back_to_the_same_commands(tmp_path):
    """One thread for the node, as in main(): a record is one call after another, and on several
    threads the result of a goal can run alongside a tick."""
    record = tmp_path / "fms.jsonl"
    rclpy.init(args=["--ros-args", "-p", "robots:=[pinky1]", "-p", "tick_hz:=50.0"])
    node = FmsNode(record=record)
    world = World(node.contract, load_site_map(), "Q")
    executors = [SingleThreadedExecutor(), MultiThreadedExecutor(num_threads=4)]
    executors[0].add_node(node)
    executors[1].add_node(world)
    threads = [threading.Thread(target=e.spin, daemon=True) for e in executors]
    for thread in threads:
        thread.start()
    try:
        publish(world, "RUN", {"A": 1, "C": 2})
        assert wait_for(lambda: world.reports), f"fills {world.fills}, swaps {world.swaps}"
    finally:
        for executor, thread in zip(executors, threads, strict=True):
            executor.shutdown()
            thread.join(timeout=5)
        world.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
    played = subprocess.run([sys.executable, str(REPLAY), str(record)], capture_output=True,
                            text=True, timeout=120)
    assert played.returncode == 0, played.stdout + played.stderr
    assert "same as recorded" in played.stdout
    assert '"out": "ReportOrderCmd"' in record.read_text(encoding="utf-8")


def test_the_heartbeat_comes_twice_a_second(graph):
    assert wait_for(lambda: len(graph.beats) >= 5, 20.0), graph.beats
    gaps = [b - a for a, b in zip(graph.beats, graph.beats[1:], strict=False)]
    assert 0.3 <= sum(gaps) / len(gaps) <= 0.8, gaps


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


def test_a_stop_answered_with_the_status_alone_is_no_failed_attempt(graph):
    """The server calls canceled() and leaves the reason empty: the goal status says it all."""
    graph.hold_fill.set()
    graph.fill_cancel_reason = ""
    publish(graph, "RUN", {"A": 1, "C": 2})
    assert wait_for(lambda: graph.fills), "the fill never reached the server"
    publish(graph, "STOP", {"A": 1, "C": 2})
    assert wait_for(lambda: graph.canceled_fills), "the cancel never reached the server"
    orchestrator = graph.fms.core.orchestrator
    assert wait_for(lambda: orchestrator.robot_view("pinky1")["placed"]), "no result came in"
    assert orchestrator.robot_view("pinky1")["attempts"] == 0


def events_of(world, code):
    return [e for e in world.events if e.code == code]


def test_a_refused_cancel_is_sent_again_and_warned_about_once_until_the_result_comes(graph):
    graph.hold_fill.set()
    graph.refuse_fill_cancel = True
    publish(graph, "RUN", {"A": 1, "C": 2})
    assert wait_for(lambda: graph.fills), "the fill never reached the server"
    publish(graph, "STOP", {"A": 1, "C": 2})
    assert wait_for(lambda: events_of(graph, "CANCEL_UNCONFIRMED"), 10.0), graph.fill_cancel_asks
    assert graph.fill_cancel_asks >= 3
    event = events_of(graph, "CANCEL_UNCONFIRMED")[0]
    assert (event.level, event.source) == ("WARN", "workcell")
    assert "goal" in event.detail
    asks = graph.fill_cancel_asks
    assert wait_for(lambda: graph.fill_cancel_asks > asks, 5.0), "it stopped trying"
    graph.hold_fill.clear()
    assert wait_for(lambda: not graph.fms._goals._cancels, 5.0), "the result ended nothing"
    asks = graph.fill_cancel_asks
    time.sleep(1.5)
    assert graph.fill_cancel_asks == asks, "the result came, so nobody has to ask any more"
    assert len(events_of(graph, "CANCEL_UNCONFIRMED")) == 1
