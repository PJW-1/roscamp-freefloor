"""FakePinky driven through its real action and topic interfaces."""
import time

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces.msg")

from conftest import CONTRACT, wait_until, write_scenario  # noqa: E402
from fleet_core.contract import load_contract, to_ros_qos  # noqa: E402
from fleet_core.site_map import load_site_map  # noqa: E402
from fleet_interfaces.action import Dock, MoveTo  # noqa: E402
from fleet_interfaces.msg import RobotState  # noqa: E402
from geometry_msgs.msg import Pose2D  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402

CONTRACT_OBJ = load_contract(CONTRACT)
SITE = load_site_map()


class Probe:
    """A client node's worth of subscriptions and action clients for pinky1."""

    def __init__(self, harness):
        import rclpy.node
        self.node = harness.add(rclpy.node.Node("probe"))
        self.states = []
        self.node.create_subscription(
            RobotState, CONTRACT_OBJ.name("robot_state", pinky="pinky1"),
            lambda m: self.states.append((time.monotonic(), m)),
            to_ros_qos(CONTRACT_OBJ.qos("robot_state")))
        self.move = ActionClient(self.node, MoveTo, CONTRACT_OBJ.name("move_to", pinky="pinky1"))
        self.dock = ActionClient(self.node, Dock, CONTRACT_OBJ.name("dock", pinky="pinky1"))
        assert self.move.wait_for_server(10.0) and self.dock.wait_for_server(10.0)
        self.feedback = []

    def goal(self, cell):
        x, y = SITE.cell_pose(cell)
        msg = MoveTo.Goal(target_id=f"c{cell}", target_pose=Pose2D(x=x, y=y))
        fut = self.move.send_goal_async(msg, feedback_callback=self.feedback.append)
        assert wait_until(fut.done, 5.0)
        return fut.result()


def start(harness, tmp_path, **scenario):
    from fake_robots.fake_pinky_node import FakePinkyNode
    harness.add(FakePinkyNode(parameter_overrides=harness.params(
        robot_id="pinky1", scenario_file=write_scenario(tmp_path, **scenario),
        state_rate_hz=20.0)))
    return Probe(harness)


def result_of(handle, timeout=15.0):
    fut = handle.get_result_async()
    assert wait_until(fut.done, timeout), "no result"
    return fut.result()


def test_move_to_publishes_state_feedback_and_succeeds(harness, tmp_path):
    p = start(harness, tmp_path)
    assert wait_until(lambda: p.states, 5.0)
    assert p.states[-1][1].robot_id == "pinky1" and p.states[-1][1].state == RobotState.STATE_IDLE
    handle = p.goal((9, 4))
    assert handle.accepted
    res = result_of(handle).result
    assert res.success is True
    last = p.states[-1][1]
    x, y = SITE.cell_pose((9, 4))
    assert (last.pose.x, last.pose.y) == pytest.approx((x, y), abs=1e-3)
    assert any(s.state == RobotState.STATE_MOVING for _, s in p.states)
    assert p.feedback


def test_new_goal_supersedes_the_active_one(harness, tmp_path):
    p = start(harness, tmp_path, robots={
        "pinky1": {"start": "P-6", "speed_mps": 0.2, "pose_noise_m": 0.0}})
    first = p.goal((2, 0))
    assert first.accepted
    second = p.goal((9, 4))
    old = result_of(first).result
    assert old.success is False and old.reason == MoveTo.Result.REASON_CANCELED
    assert result_of(second).result.success is True


def test_cancel_is_honoured(harness, tmp_path):
    p = start(harness, tmp_path, robots={
        "pinky1": {"start": "P-6", "speed_mps": 0.2, "pose_noise_m": 0.0}})
    handle = p.goal((2, 0))
    cancel = handle.cancel_goal_async()
    assert wait_until(cancel.done, 5.0)
    res = result_of(handle).result
    assert res.success is False and res.reason == MoveTo.Result.REASON_CANCELED


def test_nav_abort_hits_the_next_goals_only(harness, tmp_path):
    p = start(harness, tmp_path, faults=[
        {"at_s": 0, "robot": "pinky1", "type": "nav_abort", "count": 1}])
    time.sleep(0.3)
    res = result_of(p.goal((9, 4))).result
    assert res.success is False and res.reason == MoveTo.Result.REASON_NAV_ABORTED
    assert result_of(p.goal((9, 4))).result.success is True


def test_comm_loss_silences_state_rejects_goals_and_recovers(harness, tmp_path):
    p = start(harness, tmp_path, faults=[
        {"at_s": 2.0, "robot": "pinky1", "type": "comm_loss", "duration_s": 3.0}])
    assert wait_until(lambda: p.states, 5.0)

    def silent():
        return time.monotonic() - p.states[-1][0] > 0.5

    assert wait_until(silent, 5.0), "state kept coming during comm_loss"
    assert not p.goal((9, 4)).accepted
    mark = time.monotonic()
    assert wait_until(lambda: p.states[-1][0] > mark, 6.0), "state did not resume"


def test_true_pose_has_no_noise_and_keeps_coming_during_comm_loss(harness, tmp_path):
    from fake_robots.common import TRUTH_TOPIC
    p = start(harness, tmp_path, robots={
        "pinky1": {"start": "P-5", "speed_mps": 0.2, "pose_noise_m": 0.05}},
        faults=[{"at_s": 1.0, "robot": "pinky1", "type": "comm_loss", "duration_s": 60.0}])
    truth = []
    p.node.create_subscription(
        RobotState, TRUTH_TOPIC.format(robot="pinky1"),
        lambda m: truth.append((time.monotonic(), m)),
        to_ros_qos(CONTRACT_OBJ.qos("robot_state")))
    assert wait_until(lambda: p.states and truth, 5.0)

    assert wait_until(lambda: time.monotonic() - p.states[-1][0] > 0.5, 5.0), "state kept coming"
    mark = time.monotonic()
    assert wait_until(lambda: truth[-1][0] > mark, 2.0), "truth stopped during comm_loss"
    x, y = SITE.cell_pose((3, 5))  # P-5
    assert (truth[-1][1].pose.x, truth[-1][1].pose.y) == pytest.approx((x, y), abs=1e-9)


def test_start_accepts_a_waiting_cell_name(harness, tmp_path):
    p = start(harness, tmp_path)  # pinky1 starts at P-6
    assert wait_until(lambda: p.states, 5.0)
    x, y = SITE.cell_pose((3, 4))
    assert (p.states[-1][1].pose.x, p.states[-1][1].pose.y) == pytest.approx((x, y), abs=1e-6)


def test_operator_remove_takes_the_robot_off_the_course_for_good(harness, tmp_path):
    from fake_robots.common import TRUTH_TOPIC
    p = start(harness, tmp_path, robots={
        "pinky1": {"start": "P-6", "speed_mps": 0.2, "pose_noise_m": 0.0}},
        faults=[{"at_s": 1.5, "robot": "pinky1", "type": "operator_remove"}])
    truth = []
    p.node.create_subscription(
        RobotState, TRUTH_TOPIC.format(robot="pinky1"), truth.append,
        to_ros_qos(CONTRACT_OBJ.qos("robot_state")))
    moving = p.goal((2, 0))
    assert moving.accepted
    res = result_of(moving).result
    assert res.success is False  # the goal ended instead of hanging
    assert wait_until(lambda: truth and truth[-1].pose.x < 0.0, 5.0)
    assert all(SITE.distance_to_zone(truth[-1].pose.x, truth[-1].pose.y, z) > 0.5
               for z in SITE.zones)
    assert wait_until(lambda: time.monotonic() - p.states[-1][0] > 0.5, 5.0), "still reporting"
    assert not p.goal((9, 4)).accepted


def test_dock_reports_phase_and_error(harness, tmp_path):
    p = start(harness, tmp_path)
    phases = []
    fut = p.dock.send_goal_async(
        Dock.Goal(dock_id="OUTBOUND"),
        feedback_callback=lambda m: phases.append(m.feedback.phase))
    assert wait_until(fut.done, 5.0)
    res = result_of(fut.result()).result
    assert res.success is True and res.error_x_cm == pytest.approx(1.0, abs=0.5)
    assert phases
    assert p.states[-1][1].state == RobotState.STATE_DOCKED
