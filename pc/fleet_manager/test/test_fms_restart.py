"""Section 5 of the work order: kill the FMS process, then start a new one.

Robots stop on their own within the heartbeat timeout, on the wall clock. The new FMS starts over
from waiting for every robot to report: pinky3 is silent when it comes up, so nothing moves until
pinky3 is back.
"""
import json
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")

from action_msgs.msg import GoalStatus, GoalStatusArray  # noqa: E402
from ament_index_python.packages import (  # noqa: E402
    PackageNotFoundError,
    get_package_share_directory,
)
from fake_robots.common import TRUTH_TOPIC  # noqa: E402
from fleet_core.contract import load_contract, to_ros_qos  # noqa: E402
from fleet_core.params import load_params  # noqa: E402
from fleet_interfaces.msg import FleetEvent, RobotState  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_action_status_default  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402

DOMAIN = 56  # the scenario integration test takes 50 to 55
ROBOTS = ["pinky1", "pinky2", "pinky3"]
WALL_TIMEOUT_S = 420
STILL_M = 0.001
SILENT_S = 0.5
SLACK_S = 1.0  # a loaded machine delivers late: the last beat, the kill and the stop are all real
PINKY3_SILENT_S = (50.0, 120.0)  # the comm_loss in fms_restart.yaml
SCENARIO = Path(__file__).parent / "fms_restart.yaml"  # means nothing without the kill below


def _accepted_at(status) -> tuple[int, int]:
    return status.goal_info.stamp.sec, status.goal_info.stamp.nanosec


class Probe(Node):
    def __init__(self, context, contract):
        super().__init__("restart_probe", context=context)
        self.sim_now = 0.0
        self.state = {}
        self.pose = {}
        self.moved = {}  # robot -> wall time its true position last changed
        self.heard = {}  # robot -> wall time of its last report
        self.last_goal = {}  # robot -> status of the move goal it was given last
        self.codes = []
        qos = to_ros_qos(contract.qos("robot_state"))
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(
            FleetEvent, contract.name("events"), lambda m: self.codes.append(m.code),
            to_ros_qos(contract.qos("events")))
        for robot in ROBOTS:
            self.create_subscription(
                RobotState, contract.name("robot_state", pinky=robot),
                lambda m, r=robot: self._on_state(r, m), qos)
            self.create_subscription(
                RobotState, TRUTH_TOPIC.format(robot=robot),
                lambda m, r=robot: self._on_truth(r, m), qos)
            self.create_subscription(
                GoalStatusArray, contract.name("move_to", pinky=robot) + "/_action/status",
                lambda m, r=robot: self._on_goals(r, m), qos_profile_action_status_default)

    def _on_clock(self, msg) -> None:
        self.sim_now = msg.clock.sec + msg.clock.nanosec / 1e9

    def _on_state(self, robot, msg) -> None:
        self.state[robot] = msg.state
        self.heard[robot] = time.monotonic()

    def _on_truth(self, robot, msg) -> None:
        old = self.pose.get(robot)
        new = (msg.pose.x, msg.pose.y)
        if old is None or max(abs(new[0] - old[0]), abs(new[1] - old[1])) > STILL_M:
            self.pose[robot] = new
            self.moved[robot] = time.monotonic()

    def _on_goals(self, robot, msg) -> None:
        if msg.status_list:
            # Reversed: of two goals in one clock step, the later one in the list counts.
            self.last_goal[robot] = max(reversed(msg.status_list), key=_accepted_at).status

    def moving(self) -> list[str]:
        """Who says it is moving. A robot that has gone silent says nothing."""
        now = time.monotonic()
        return [r for r in ROBOTS if self.state.get(r) == RobotState.STATE_MOVING
                and now - self.heard[r] < SILENT_S]


@pytest.fixture
def probe(monkeypatch):
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    try:
        contract = load_contract(
            get_package_share_directory("fleet_interfaces") + "/contract.yaml")
    except PackageNotFoundError as exc:
        pytest.skip(f"not built in this workspace: {exc}")
    context = rclpy.Context()
    rclpy.init(context=context, domain_id=DOMAIN)
    node = Probe(context, contract)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    yield node
    executor.shutdown()
    thread.join(timeout=5)
    node.destroy_node()
    rclpy.shutdown(context=context)


def wait_for(predicate, timeout_s):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def stop(process, sig=signal.SIGINT) -> None:
    if process.poll() is None:
        os.killpg(process.pid, sig)
        try:
            process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def test_robots_stop_without_the_fms_and_a_new_one_waits_for_all_of_them(probe, tmp_path):
    try:
        get_package_share_directory("fake_robots")
    except PackageNotFoundError as exc:
        pytest.skip(f"not built in this workspace: {exc}")
    env = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN), ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
    params = load_params()
    timeout_s = params.heartbeat_timeout_s
    log = (tmp_path / "launch.log").open("w", encoding="utf-8")

    def start(*command):
        return subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)

    def start_fms():
        return start("ros2", "run", "fleet_manager", "fms_node", "--ros-args",
                     "-p", "use_sim_time:=true", "-p", f"robots:={ROBOTS}")

    fakes = start("ros2", "launch", "fake_robots", "fakes.launch.py", f"scenario:={SCENARIO}",
                  f"result_dir:={tmp_path}")
    fms = start_fms()
    try:
        assert wait_for(lambda: len(probe.moving()) == len(ROBOTS), 120), probe.state
        print(f"killing at {probe.sim_now:.1f} s, moving: {probe.moving()}")
        assert probe.sim_now < PINKY3_SILENT_S[1], "the new FMS has to find pinky3 silent"
        os.killpg(fms.pid, signal.SIGKILL)
        killed = time.monotonic()
        fms.wait()

        time.sleep(timeout_s + SLACK_S)
        took = {r: probe.moved[r] - killed for r in ROBOTS}
        print(f"robots moved for this long after the kill: {took}")
        assert all(t <= timeout_s + SLACK_S for t in took.values()), took
        # Nobody got there by itself: every one of them had its goal ended short of the target.
        assert wait_for(lambda: all(probe.last_goal.get(r) == GoalStatus.STATUS_ABORTED
                                    for r in ROBOTS), 10), probe.last_goal
        print(f"last goal of each: {probe.last_goal}")
        assert not probe.moving(), probe.state

        # One report before the silence and the new FMS would call it a comm loss instead.
        assert wait_for(lambda: probe.sim_now > PINKY3_SILENT_S[0], 60), probe.sim_now
        still = dict(probe.moved)
        fms = start_fms()
        assert wait_for(lambda: "ROBOT_NOT_REPORTING" in probe.codes, 60), probe.codes
        print(f"the new FMS misses a robot at {probe.sim_now:.1f} s")
        assert probe.sim_now < PINKY3_SILENT_S[1], "pinky3 came back before it was missed"
        assert probe.moved == still, "a robot moved while the new FMS was still waiting for one"

        assert wait_for(lambda: any(probe.moved[r] > still[r] for r in ROBOTS), 60)
        print(f"robots move again at {probe.sim_now:.1f} s")
        assert probe.sim_now >= PINKY3_SILENT_S[1], "a robot moved before pinky3 was back"
        fakes.wait(timeout=WALL_TIMEOUT_S)
    finally:
        stop(fms)
        stop(fakes)
        log.close()

    tail = (tmp_path / "launch.log").read_text(encoding="utf-8")[-3000:]
    summary_path = tmp_path / "summary.json"
    assert summary_path.exists(), f"no summary was written. log tail:\n{tail}"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    print(f"orders: {summary['orders_shipped']} shipped of {summary['orders_total']} "
          f"in {summary['duration_s']:.0f} s. violations: {summary['violations']}")
    assert summary["orders_shipped"] == summary["orders_total"], summary
    assert summary["violations"] == [], summary
