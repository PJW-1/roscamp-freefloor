"""실제 액션과 토픽 인터페이스로 FakePinky를 시험한다."""
import math
import time

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces.msg")

from conftest import CONTRACT, wait_until, write_scenario  # noqa: E402
from fleet_core.common.contract import load_contract, to_ros_qos  # noqa: E402
from fleet_core.traffic.lane_map import (
    _point_segment,  # noqa: E402
    load_lane_map,  # noqa: E402
)
from fleet_interfaces.action import Dock, MoveTo  # noqa: E402
from fleet_interfaces.msg import RobotState  # noqa: E402
from geometry_msgs.msg import Point, Pose, Pose2D  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402

CONTRACT_OBJ = load_contract(CONTRACT)
SITE = load_lane_map()


class Probe:
    """pinky1용 구독과 액션 클라이언트 한 벌."""

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
        x, y = SITE.waypoint_pose(cell)
        msg = MoveTo.Goal(target_id=f"c{cell}", target_pose=Pose2D(x=x, y=y))
        fut = self.move.send_goal_async(msg, feedback_callback=self.feedback.append)
        assert wait_until(fut.done, 5.0)
        return fut.result()


def start(harness, tmp_path, heartbeat_timeout_s=0.0, node=None, **scenario):
    """여기엔 FMS가 없으니 기본적으로 로봇은 FMS 소식을 기다리지 않는다. node: 노드 파라미터."""
    from fake_robots.fake_pinky_node import FakePinkyNode
    harness.add(FakePinkyNode(parameter_overrides=harness.params(
        robot_id="pinky1", scenario_file=write_scenario(tmp_path, **scenario),
        state_rate_hz=20.0, heartbeat_timeout_s=heartbeat_timeout_s, **(node or {}))))
    return Probe(harness)


def result_of(handle, timeout=15.0):
    fut = handle.get_result_async()
    assert wait_until(fut.done, timeout), "no result"
    return fut.result()


def test_move_to_publishes_state_feedback_and_succeeds(harness, tmp_path):
    p = start(harness, tmp_path)
    assert wait_until(lambda: p.states, 5.0)
    assert p.states[-1][1].robot_id == "pinky1" and p.states[-1][1].state == RobotState.STATE_IDLE
    handle = p.goal("QO")
    assert handle.accepted
    res = result_of(handle).result
    assert res.success is True
    last = p.states[-1][1]
    x, y = SITE.waypoint_pose("QO")
    assert (last.pose.x, last.pose.y) == pytest.approx((x, y), abs=1e-3)
    assert any(s.state == RobotState.STATE_MOVING for _, s in p.states)
    assert p.feedback


def test_a_path_goal_follows_the_lanes_round_the_corner(harness, tmp_path):
    """path 모드(지시서 7번): 꺾인 선을 그대로 따르고 끝에서 성공한다."""
    p = start(harness, tmp_path, robots={
        "pinky1": {"start": "P-6", "speed_mps": 0.5, "pose_noise_m": 0.0}})
    assert wait_until(lambda: p.states, 5.0)
    points = [SITE.waypoint_pose(w) for w in ("P-6", "N", "S")]
    msg = MoveTo.Goal(target_id="S", target_pose=Pose2D(x=points[-1][0], y=points[-1][1]))
    msg.path = [Pose(position=Point(x=x, y=y)) for x, y in points]
    fut = p.move.send_goal_async(msg)
    assert wait_until(fut.done, 5.0) and fut.result().accepted
    assert result_of(fut.result()).result.success is True
    seen = [(s.pose.x, s.pose.y) for _, s in p.states]
    off = max(min(_point_segment(x, y, *a, *b) for a, b in zip(points, points[1:], strict=False))
              for x, y in seen)
    assert off < 0.01, "it cut the corner"
    assert seen[-1] == pytest.approx(points[-1], abs=1e-3)


def test_new_goal_supersedes_the_active_one(harness, tmp_path):
    p = start(harness, tmp_path, robots={
        "pinky1": {"start": "P-6", "speed_mps": 0.2, "pose_noise_m": 0.0}})
    first = p.goal("S")
    assert first.accepted
    second = p.goal("QO")
    old = result_of(first).result
    assert old.success is False and old.reason == MoveTo.Result.REASON_CANCELED
    assert result_of(second).result.success is True


def test_cancel_is_honoured(harness, tmp_path):
    p = start(harness, tmp_path, robots={
        "pinky1": {"start": "P-6", "speed_mps": 0.2, "pose_noise_m": 0.0}})
    handle = p.goal("S")
    cancel = handle.cancel_goal_async()
    assert wait_until(cancel.done, 5.0)
    res = result_of(handle).result
    assert res.success is False and res.reason == MoveTo.Result.REASON_CANCELED


def test_nav_abort_hits_the_next_goals_only(harness, tmp_path):
    p = start(harness, tmp_path, faults=[
        {"at_s": 0, "robot": "pinky1", "type": "nav_abort", "count": 1}])
    time.sleep(0.3)
    res = result_of(p.goal("QO")).result
    assert res.success is False and res.reason == MoveTo.Result.REASON_NAV_ABORTED
    assert result_of(p.goal("QO")).result.success is True


def test_a_nav_abort_comes_before_the_robot_reaches_the_release_distance(harness, tmp_path):
    """관제는 해제 거리(0.2 m)를 지나야 선을 늘린다. 그 전에 중단해야 늘림과 겹치지 않는다."""
    p = start(harness, tmp_path, robots={
        "pinky1": {"start": "P-6", "speed_mps": 0.2, "pose_noise_m": 0.0}}, faults=[
        {"at_s": 0, "robot": "pinky1", "type": "nav_abort", "count": 1}])
    assert wait_until(lambda: p.states, 5.0)
    x0, y0 = p.states[-1][1].pose.x, p.states[-1][1].pose.y
    res = result_of(p.goal("S")).result
    assert res.reason == MoveTo.Result.REASON_NAV_ABORTED
    last = p.states[-1][1].pose
    assert ((last.x - x0) ** 2 + (last.y - y0) ** 2) ** 0.5 < 0.1


def test_a_nav_abort_is_not_lost_on_a_goal_superseded_before_it_fires(harness, tmp_path):
    """FMS가 선을 늘려 목표를 바꿔도 중단 횟수만큼 NAV_ABORTED가 난다."""
    p = start(harness, tmp_path, robots={
        "pinky1": {"start": "P-6", "speed_mps": 0.2, "pose_noise_m": 0.0}}, faults=[
        {"at_s": 0, "robot": "pinky1", "type": "nav_abort", "count": 1}])
    time.sleep(0.3)
    first = p.goal("S")
    second = p.goal("S")
    assert result_of(first).result.reason == MoveTo.Result.REASON_CANCELED
    res = result_of(second).result
    assert res.success is False and res.reason == MoveTo.Result.REASON_NAV_ABORTED


def test_comm_loss_silences_state_rejects_goals_and_recovers(harness, tmp_path):
    p = start(harness, tmp_path, faults=[
        {"at_s": 2.0, "robot": "pinky1", "type": "comm_loss", "duration_s": 3.0}])
    assert wait_until(lambda: p.states, 5.0)

    def silent():
        return time.monotonic() - p.states[-1][0] > 0.5

    assert wait_until(silent, 5.0), "state kept coming during comm_loss"
    assert not p.goal("QO").accepted
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
    x, y = SITE.waypoint_pose("P-5")
    assert (truth[-1][1].pose.x, truth[-1][1].pose.y) == pytest.approx((x, y), abs=1e-9)


def truth_of(p):
    from fake_robots.common import TRUTH_TOPIC
    truth = []
    p.node.create_subscription(
        RobotState, TRUTH_TOPIC.format(robot="pinky1"), truth.append,
        to_ros_qos(CONTRACT_OBJ.qos("robot_state")))
    return truth


def test_a_late_report_says_where_the_robot_was_when_it_was_measured(harness, tmp_path):
    p = start(harness, tmp_path, robots={"pinky1": {
        "start": "P-6", "speed_mps": 0.1, "pose_noise_m": 0.0, "report_delay_s": 1.0}})
    truth = truth_of(p)
    assert wait_until(lambda: p.states and truth, 5.0)
    p.goal("N")
    assert wait_until(lambda: p.states[-1][1].state == RobotState.STATE_MOVING, 5.0)
    assert wait_until(lambda: p.states[-1][1].pose.x != SITE.waypoint_pose("P-6")[0], 5.0)
    time.sleep(0.5)  # 지연 내내 최고 속도로 움직인다
    told, now = p.states[-1][1], truth[-1]
    gap = ((told.pose.x - now.pose.x) ** 2 + (told.pose.y - now.pose.y) ** 2) ** 0.5
    assert gap == pytest.approx(0.1, abs=0.04)  # 속도 * 지연
    age = (p.node.get_clock().now().nanoseconds / 1e9
           - (told.stamp.sec + told.stamp.nanosec / 1e9))
    assert age == pytest.approx(1.0, abs=0.3)


def test_reports_measured_during_a_comm_loss_never_come(harness, tmp_path):
    p = start(harness, tmp_path, robots={"pinky1": {
        "start": "P-6", "speed_mps": 0.1, "pose_noise_m": 0.0, "report_delay_s": 1.0}},
        faults=[{"at_s": 2.0, "robot": "pinky1", "type": "comm_loss", "duration_s": 2.0}])
    assert wait_until(lambda: p.states, 5.0)
    assert wait_until(lambda: time.monotonic() - p.states[-1][0] > 0.5, 6.0), "never went silent"
    back = len(p.states)
    assert wait_until(lambda: len(p.states) > back, 6.0), "never came back"
    came = p.states[back][1].stamp
    # 통신은 시계 첫 틱 4초 뒤에 돌아온다. 그 전에 잰 값은 사라진다.
    first = p.states[0][1].stamp
    assert (came.sec + came.nanosec / 1e9) - (first.sec + first.nanosec / 1e9) > 2.5


def test_a_move_ends_off_its_target_by_the_stop_error(harness, tmp_path):
    p = start(harness, tmp_path, robots={"pinky1": {
        "start": "P-6", "speed_mps": 1.0, "pose_noise_m": 0.0, "stop_error_m": 0.05}})
    truth = truth_of(p)
    target = SITE.waypoint_pose("N")
    assert result_of(p.goal("N")).result.success
    assert wait_until(lambda: truth, 5.0)
    time.sleep(0.2)
    stopped = truth[-1].pose
    off = ((stopped.x - target[0]) ** 2 + (stopped.y - target[1]) ** 2) ** 0.5
    assert 0.0 < off < 0.3


def test_start_accepts_a_waiting_cell_name(harness, tmp_path):
    p = start(harness, tmp_path)  # pinky1은 P-6에서 시작
    assert wait_until(lambda: p.states, 5.0)
    x, y = SITE.waypoint_pose("P-6")
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
    moving = p.goal("S")
    assert moving.accepted
    res = result_of(moving).result
    assert res.success is False  # 목표가 멈추지 않고 끝났다
    assert wait_until(lambda: truth and truth[-1].pose.x < 0.0, 5.0)
    assert all(SITE.distance_to_zone(truth[-1].pose.x, truth[-1].pose.y, z) > 0.5
               for z in SITE.zones)
    assert wait_until(lambda: time.monotonic() - p.states[-1][0] > 0.5, 5.0), "still reporting"
    assert not p.goal("QO").accepted


def test_operator_reinstate_puts_the_robot_back_and_it_reports_again(harness, tmp_path):
    p = start(harness, tmp_path, robots={
        "pinky1": {"start": "P-6", "speed_mps": 0.2, "pose_noise_m": 0.0}},
        faults=[{"at_s": 0.5, "robot": "pinky1", "type": "operator_remove"},
                {"at_s": 2.0, "robot": "pinky1", "type": "operator_reinstate", "start": "P-5"}])
    assert wait_until(lambda: p.states, 5.0)
    assert wait_until(lambda: time.monotonic() - p.states[-1][0] > 0.5, 5.0), "still reporting"
    x, y = SITE.waypoint_pose(next(iter(SITE.zones["P-5"])))
    assert wait_until(lambda: time.monotonic() - p.states[-1][0] < 0.3
                      and (p.states[-1][1].pose.x, p.states[-1][1].pose.y)
                      == pytest.approx((x, y), abs=1e-6), 8.0), "never back on P-5"
    assert p.states[-1][1].state == RobotState.STATE_IDLE
    assert p.goal("QO").accepted


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


def test_a_dock_reports_its_dock_id_as_the_current_goal(harness, tmp_path):
    """Pinky contract section 2: current_goal is the target_id or dock_id of the running goal."""
    p = start(harness, tmp_path, dock={"duration_s": 30.0, "error_cm": 1.0})
    fut = p.dock.send_goal_async(Dock.Goal(dock_id="WORK"))
    assert wait_until(fut.done, 5.0) and fut.result().accepted
    assert wait_until(lambda: any(s.state == RobotState.STATE_DOCKING for _, s in p.states), 5.0)
    assert {s.current_goal for _, s in p.states if s.state == RobotState.STATE_DOCKING} == {"WORK"}


# ---- the heartbeat ----------------------------------------------------------------------

SLOW = {"pinky1": {"start": "P-6", "speed_mps": 0.1, "pose_noise_m": 0.0}}
BEAT_TIMEOUT_S = 0.5


def beat(p):
    """FMS 역할: 지금부터 하트비트를 보낸다."""
    from std_msgs.msg import Header
    pub = p.node.create_publisher(Header, CONTRACT_OBJ.name("heartbeat"),
                                  to_ros_qos(CONTRACT_OBJ.qos("heartbeat")))
    return p.node.create_timer(0.1, lambda: pub.publish(Header(frame_id="fms")))


def watch_truth(p):
    from fake_robots.common import TRUTH_TOPIC
    truth = []
    p.node.create_subscription(
        RobotState, TRUTH_TOPIC.format(robot="pinky1"),
        lambda m: truth.append((time.monotonic(), m.pose.x, m.pose.y)),
        to_ros_qos(CONTRACT_OBJ.qos("robot_state")))
    return truth


def test_a_move_stops_when_no_heartbeat_comes(harness, tmp_path):
    p = start(harness, tmp_path, heartbeat_timeout_s=BEAT_TIMEOUT_S, robots=SLOW)
    assert wait_until(lambda: p.states, 5.0)
    truth = watch_truth(p)
    sent = time.monotonic()
    handle = p.goal("QO")
    assert handle.accepted, "the goal itself shows somebody is there"
    res = result_of(handle, 5.0).result
    took = time.monotonic() - sent
    assert res.success is False and res.reason == MoveTo.Result.REASON_HEARTBEAT_LOST
    assert BEAT_TIMEOUT_S * 0.8 <= took <= BEAT_TIMEOUT_S + 1.0, took
    assert wait_until(lambda: p.states[-1][1].state == RobotState.STATE_IDLE, 2.0)
    mark = time.monotonic()
    assert wait_until(lambda: truth and truth[-1][0] > mark + 0.3, 3.0)
    stopped = [(x, y) for when, x, y in truth if when >= mark]
    assert len(set(stopped)) == 1, "it kept going"


def test_a_heartbeat_keeps_a_move_going_past_the_timeout(harness, tmp_path):
    p = start(harness, tmp_path, heartbeat_timeout_s=BEAT_TIMEOUT_S, robots={
        "pinky1": {"start": "P-5", "speed_mps": 0.2, "pose_noise_m": 0.0}})
    beat(p)
    time.sleep(0.3)
    target = "P-6"
    sent = time.monotonic()
    res = result_of(p.goal(target), 10.0).result
    assert res.success is True
    assert time.monotonic() - sent > BEAT_TIMEOUT_S, "the move has to outlast the timeout"


def test_a_robot_that_stopped_takes_the_next_goal(harness, tmp_path):
    p = start(harness, tmp_path, heartbeat_timeout_s=BEAT_TIMEOUT_S, robots=SLOW)
    assert result_of(p.goal("QO"), 5.0).result.success is False
    beat(p)
    second = p.goal("P-5")
    assert second.accepted
    assert result_of(second, 15.0).result.success is True


def test_a_dock_stops_too(harness, tmp_path):
    p = start(harness, tmp_path, heartbeat_timeout_s=BEAT_TIMEOUT_S,
              dock={"duration_s": 30.0, "error_cm": 1.0})
    fut = p.dock.send_goal_async(Dock.Goal(dock_id="WORK"))
    assert wait_until(fut.done, 5.0) and fut.result().accepted
    assert result_of(fut.result(), 5.0).result.success is False
    assert wait_until(lambda: p.states[-1][1].state == RobotState.STATE_IDLE, 2.0)


def test_no_heartbeat_gets_through_a_comm_loss(harness, tmp_path):
    p = start(harness, tmp_path, heartbeat_timeout_s=BEAT_TIMEOUT_S, robots=SLOW, faults=[
        {"at_s": 1.5, "robot": "pinky1", "type": "comm_loss", "duration_s": 60.0}])
    truth = watch_truth(p)
    beat(p)
    assert wait_until(lambda: p.states, 5.0)
    assert p.goal("QO").accepted
    assert wait_until(lambda: time.monotonic() - p.states[-1][0] > 0.2, 5.0), "state kept coming"
    time.sleep(BEAT_TIMEOUT_S + 0.5)
    mark = time.monotonic()
    assert wait_until(lambda: truth[-1][0] > mark + 0.3, 3.0)
    assert len({(x, y) for when, x, y in truth if when >= mark}) == 1, "it kept going"


def test_a_robot_with_nothing_to_do_does_not_mind_the_silence(harness, tmp_path):
    p = start(harness, tmp_path, heartbeat_timeout_s=BEAT_TIMEOUT_S)
    assert wait_until(lambda: p.states, 5.0)
    time.sleep(BEAT_TIMEOUT_S + 0.3)
    assert p.states[-1][1].state == RobotState.STATE_IDLE
    assert time.monotonic() - p.states[-1][0] < 0.5, "it keeps reporting"



# ---- 앞 물체와 위치 잡기(계약 3절 6번, 4절) -----------------------------------------------

ONE = {"pinky1": {"start": "P-6", "speed_mps": 0.2, "pose_noise_m": 0.0}}
NO_RECOVERY = {"recovery_attempts": 0}  # 자기 복구 없이 BLOCKED로 바로 가는 동작(계약 4절)


def states_since(p, mark=0.0):
    return [s.state for when, s in p.states if when >= mark]


def test_an_obstacle_holds_a_move_that_says_blocked_and_then_goes_on(harness, tmp_path):
    """1초 서면 BLOCKED. 목표는 끝내지 않고, 물체가 1초 안 보이면 이어 가서 도착한다."""
    p = start(harness, tmp_path, robots=ONE, node=NO_RECOVERY, faults=[
        {"at_s": 3.0, "robot": "pinky1", "type": "blocked", "duration_s": 3.0}])
    truth = watch_truth(p)
    assert wait_until(lambda: p.states, 5.0)
    handle = p.goal("N")
    assert handle.accepted
    assert wait_until(lambda: p.states[-1][1].state == RobotState.STATE_BLOCKED, 8.0)
    mark = time.monotonic()
    assert wait_until(lambda: truth and truth[-1][0] > mark + 0.3, 2.0)
    assert len({(x, y) for when, x, y in truth if when >= mark}) == 1, "it moved while blocked"
    res = result_of(handle, 20.0).result
    assert res.success is True
    seen = states_since(p)
    assert RobotState.STATE_MOVING in seen[seen.index(RobotState.STATE_BLOCKED):]


def test_blocked_lasts_past_a_cancel_while_the_obstacle_is_there(harness, tmp_path):
    p = start(harness, tmp_path, robots=ONE, node=NO_RECOVERY, faults=[
        {"at_s": 1.0, "robot": "pinky1", "type": "blocked", "duration_s": 5.0}])
    assert wait_until(lambda: p.states, 5.0)
    handle = p.goal("N")
    assert wait_until(lambda: p.states[-1][1].state == RobotState.STATE_BLOCKED, 8.0)
    cancel = handle.cancel_goal_async()
    assert wait_until(cancel.done, 5.0)
    res = result_of(handle).result
    assert res.success is False and res.reason == MoveTo.Result.REASON_CANCELED
    mark = time.monotonic()
    time.sleep(0.5)
    assert set(states_since(p, mark)) == {RobotState.STATE_BLOCKED}
    assert wait_until(lambda: p.states[-1][1].state == RobotState.STATE_IDLE, 8.0)


def test_a_robot_put_back_does_not_keep_saying_blocked(harness, tmp_path):
    """막혀 선 로봇을 치웠다 다른 곳에 놓으면 앞 막힘을 잊는다(병합 리뷰 2번)."""
    began = time.monotonic()
    p = start(harness, tmp_path, robots=ONE, node=NO_RECOVERY, faults=[
        {"at_s": 1.0, "robot": "pinky1", "type": "blocked", "duration_s": 20.0},
        {"at_s": 3.0, "robot": "pinky1", "type": "operator_remove"},
        {"at_s": 4.0, "robot": "pinky1", "type": "operator_reinstate", "start": "P-5"}])
    truth = watch_truth(p)
    assert wait_until(lambda: p.states, 5.0)
    p.goal("N")
    assert wait_until(lambda: p.states[-1][1].state == RobotState.STATE_BLOCKED, 8.0)
    assert wait_until(lambda: time.monotonic() - p.states[-1][0] > 0.5, 5.0), "still reporting"
    assert wait_until(lambda: time.monotonic() - p.states[-1][0] < 0.3, 8.0), "never back"
    mark = time.monotonic()
    time.sleep(1.0)
    assert set(states_since(p, mark - 0.3)) == {RobotState.STATE_IDLE}
    # 물체는 들어낸 자리에 남았다. 다시 달려도 막히지 않는다(물체 구간은 21초까지)
    assert wait_until(lambda: truth, 2.0)
    x0, y0 = truth[-1][1], truth[-1][2]
    px, py = SITE.waypoint_pose("P-5")
    assert ((x0 - px) ** 2 + (y0 - py) ** 2) ** 0.5 < 0.1, "not yet back on P-5"
    assert time.monotonic() - began < 14.0, "too late: the goal must start well inside the window"
    mark = time.monotonic()
    assert p.goal("P-3").accepted

    def moved():
        return max((((x - x0) ** 2 + (y - y0) ** 2) ** 0.5
                    for when, x, y in list(truth) if when >= mark), default=0.0)

    assert wait_until(lambda: moved() > 0.2, 5.0), f"it moved only {moved():.2f} m"
    seen = set(states_since(p, mark))
    assert RobotState.STATE_BLOCKED not in seen, seen  # 도착 뒤 IDLE은 괜찮다
    assert time.monotonic() - began < 20.0, "the watch must end inside the obstacle window"


def test_a_robot_says_localizing_until_it_has_a_pose(harness, tmp_path):
    p = start(harness, tmp_path, robots={
        "pinky1": {"start": "P-6", "speed_mps": 0.2, "pose_noise_m": 0.0, "localizing_s": 3.0}})
    assert wait_until(lambda: p.states and p.states[-1][1].state == RobotState.STATE_IDLE, 8.0)
    seen = [s for _, s in p.states]
    first = next(i for i, s in enumerate(seen) if s.state == RobotState.STATE_IDLE)
    assert first > 0, "no LOCALIZING report came first"
    assert {s.state for s in seen[:first]} == {RobotState.STATE_LOCALIZING}
    x, y = SITE.waypoint_pose("P-6")
    assert (seen[first].pose.x, seen[first].pose.y) == pytest.approx((x, y), abs=1e-9)


def test_the_heartbeat_is_heard_with_the_reliable_profile_of_the_contract(harness, tmp_path):
    """지시서 1.5: FakePinky도 heartbeat QoS를 계약에서 읽는다."""
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
    p = start(harness, tmp_path)
    topic, fake = CONTRACT_OBJ.name("heartbeat"), harness.nodes[0].get_name()

    def mine():
        return [i for i in p.node.get_subscriptions_info_by_topic(topic) if i.node_name == fake]

    assert wait_until(mine, 5.0), "FakePinky has no heartbeat subscription"
    qos = mine()[0].qos_profile
    assert (qos.reliability, qos.durability) == (ReliabilityPolicy.RELIABLE,
                                                 DurabilityPolicy.VOLATILE)
    # depth는 탐색 정보에 실리지 않으므로 구독 객체에서 본다
    depths = [s.qos_profile.depth for s in harness.nodes[0].subscriptions
              if s.topic_name == topic]
    assert depths == [1]


# ---- path connection (지시서 7.5) -------------------------------------------------------

def send_path(p, points, target_id):
    tx, ty = points[-1]
    msg = MoveTo.Goal(target_id=target_id, target_pose=Pose2D(x=tx, y=ty))
    msg.path = [Pose(position=Point(x=x, y=y)) for x, y in points]
    fut = p.move.send_goal_async(msg)
    assert wait_until(fut.done, 5.0) and fut.result().accepted
    return fut.result()


def lane_points(*waypoints, shift=0.0):
    return [(x + shift, y + shift) for x, y in (SITE.waypoint_pose(w) for w in waypoints)]


def test_a_path_that_misses_the_robot_is_refused_and_the_robot_stays(harness, tmp_path):
    """path가 로봇 자리에서 lane_error_warn_m(0.15 m)보다 멀면 따라가지 않는다."""
    p = start(harness, tmp_path, robots=SLOW)
    assert wait_until(lambda: p.states, 5.0)
    truth = watch_truth(p)
    res = result_of(send_path(p, lane_points("P-6", "N", "S", shift=0.2), "S")).result
    assert (res.success, res.reason) == (False, MoveTo.Result.REASON_PATH_NOT_CONNECTED)
    time.sleep(0.5)
    assert truth and {(x, y) for _, x, y in truth} == {SITE.waypoint_pose("P-6")}


def test_a_path_within_the_tolerance_is_followed(harness, tmp_path):
    """0.15 m 안이면 연결된 것이다. 위치 잡음과 보고 지연만큼은 받아 준다."""
    p = start(harness, tmp_path, robots={
        "pinky1": {"start": "P-6", "speed_mps": 0.5, "pose_noise_m": 0.0}})
    assert wait_until(lambda: p.states, 5.0)
    res = result_of(send_path(p, lane_points("P-6", "N", "S", shift=0.07), "S")).result
    assert res.success is True


def test_a_refused_path_ends_the_old_goal_and_a_path_from_here_arrives(harness, tmp_path):
    """이전 목표는 새 목표를 받는 순간 끝나므로(R6) 로봇은 선다. 지금 자리에서 다시 만든 path로
    도착한다. 관제는 이 실패를 한 번으로 세고 move_retry_wait_s 뒤 그렇게 다시 보낸다."""
    p = start(harness, tmp_path, robots={
        "pinky1": {"start": "P-6", "speed_mps": 0.2, "pose_noise_m": 0.0}})
    assert wait_until(lambda: p.states, 5.0)
    first = send_path(p, lane_points("P-6", "N", "S"), "S")
    assert wait_until(lambda: p.states[-1][1].state == RobotState.STATE_MOVING, 5.0)
    time.sleep(0.5)
    truth = watch_truth(p)
    bad = send_path(p, lane_points("P-6", "N", "S", shift=0.3), "S")
    old, res = result_of(first).result, result_of(bad).result
    assert (old.success, old.reason) == (False, MoveTo.Result.REASON_CANCELED)
    assert (res.success, res.reason) == (False, MoveTo.Result.REASON_PATH_NOT_CONNECTED)
    time.sleep(0.3)
    mark = len(truth)
    time.sleep(0.5)
    assert len(truth) > mark and len({(x, y) for _, x, y in truth[mark - 1:]}) == 1, "it moved"
    here = p.states[-1][1].pose
    rest = [w for w in ("N", "S")
            if math.dist((here.x, here.y), SITE.waypoint_pose(w)) > 1e-3]
    again = send_path(p, [(here.x, here.y), *lane_points(*rest)], "S")
    assert result_of(again).result.success is True


# ---- 자기 복구(제안, 2026-11-09 공유 명세 '로봇 쪽') ----------------------------------------

RECOVERING = RobotState.STATE_RECOVERING
LONG = {"pinky1": {"start": "P-6", "speed_mps": 0.2, "pose_noise_m": 0.0}}  # QO까지 2.3 m


def runs(states):
    """연달아 같은 상태를 하나로."""
    out = []
    for s in states:
        if not out or out[-1] != s:
            out.append(s)
    return out


def along(truth, start="P-6", end="QO"):
    """시작점에서 목표 쪽으로 간 거리(진실 자세)."""
    (x0, y0), (x1, y1) = SITE.waypoint_pose(start), SITE.waypoint_pose(end)
    n = math.dist((x0, y0), (x1, y1))
    return [((x - x0) * (x1 - x0) + (y - y0) * (y1 - y0)) / n for _, x, y in truth]


def back_ups(distances, eps=1e-4):
    """앞으로 가다 뒤로 가기 시작한 횟수."""
    count, going = 0, 1
    for a, b in zip(distances, distances[1:], strict=False):
        if b - a < -eps and going > 0:
            count, going = count + 1, -1
        elif b - a > eps:
            going = 1
    return count


def ended(handle):
    fut = handle.get_result_async()
    time.sleep(0.2)
    return fut.done()


def test_a_persistent_obstacle_gives_two_recovering_attempts_then_blocked(harness, tmp_path):
    """명세 3~5번: 1 s 서면 RECOVERING, 0.15 m 후진 뒤 다시 가서 또 서면 두 번째 시도,
    그래도 서면 BLOCKED. 목표는 끝내지 않는다."""
    p = start(harness, tmp_path, robots=LONG, faults=[
        {"at_s": 3.0, "robot": "pinky1", "type": "blocked", "duration_s": 60.0}])
    truth = watch_truth(p)
    assert wait_until(lambda: p.states, 5.0)
    handle = p.goal("QO")
    assert wait_until(lambda: p.states[-1][1].state == RobotState.STATE_BLOCKED, 20.0)
    seen = runs(states_since(p))
    assert seen[seen.index(RobotState.STATE_MOVING):] == [
        RobotState.STATE_MOVING, RECOVERING, RobotState.STATE_BLOCKED]
    d = along(truth)
    assert back_ups(d) == 2
    assert max(d) > 0.4, "too near the start: it would back up less than 0.15 m"
    assert max(d) - min(d[d.index(max(d)):]) == pytest.approx(0.15, abs=0.02)
    span = [w for w, s in p.states if s.state == RECOVERING]
    # 후진 0.15 m(0.08 m/s), 되돌아가기(0.2 m/s), 1 s 서기를 두 번
    assert span[-1] - span[0] == pytest.approx(2 * (0.15 / 0.08 + 0.15 / 0.2 + 1.0), abs=0.8)
    assert not ended(handle), "the goal is held, not ended"


def test_an_obstacle_at_the_path_start_gives_no_back_up(harness, tmp_path):
    """명세 3번 d < 0.02: 처음부터 물체가 앞에 있으면 후진하지 않고 RECOVERING 두 번 뒤 BLOCKED.
    pinky_fsm 비교 시험(물체 0.2 m 앞)과 같은 순서다."""
    p = start(harness, tmp_path, robots=LONG, faults=[
        {"at_s": 0.0, "robot": "pinky1", "type": "blocked", "duration_s": 60.0}])
    truth = watch_truth(p)
    assert wait_until(lambda: p.states, 5.0)
    sent = time.monotonic()
    p.goal("QO")
    assert wait_until(lambda: p.states[-1][1].state == RobotState.STATE_BLOCKED, 8.0)
    assert time.monotonic() - sent == pytest.approx(3.0, abs=0.6)
    assert runs(states_since(p, sent)) == [
        RobotState.STATE_MOVING, RECOVERING, RobotState.STATE_BLOCKED]
    assert {(x, y) for _, x, y in truth} == {SITE.waypoint_pose("P-6")}


def stood(p, truth, still_s=0.3, timeout=10.0):
    """물체 앞에 서기를 기다린다. 선 자리를 돌려준다."""
    def standing():
        recent = [(x, y) for w, x, y in list(truth) if w >= time.monotonic() - still_s]
        return len(recent) > 3 and len(set(recent)) == 1 and recent[0] != SITE.waypoint_pose("P-6")
    assert wait_until(standing, timeout), "it never stood"
    return truth[-1][1], truth[-1][2]


def test_no_back_up_into_a_robot_behind(harness, tmp_path):
    """명세 3번: 다른 로봇이 뒤로 후진 거리 + 반지름 안에 있으면 후진하지 않는다. 시도는 센다."""
    from fake_robots.common import TRUTH_TOPIC
    robots = {**LONG, "pinky2": {"start": "P-5"}}  # 시나리오의 다른 로봇만 본다
    p = start(harness, tmp_path, robots=robots, faults=[
        {"at_s": 3.0, "robot": "pinky1", "type": "blocked", "duration_s": 60.0}])
    truth = watch_truth(p)
    other = p.node.create_publisher(RobotState, TRUTH_TOPIC.format(robot="pinky2"),
                                    to_ros_qos(CONTRACT_OBJ.qos("robot_state")))
    assert wait_until(lambda: p.states, 5.0)
    p.goal("QO")
    x, y = stood(p, truth)
    assert p.states[-1][1].state == RobotState.STATE_MOVING, "too late: already recovering"
    assert max(along(truth)) > 0.4, "too near the start: it would back up less than 0.15 m"
    (x0, y0), (x1, y1) = SITE.waypoint_pose("P-6"), SITE.waypoint_pose("QO")
    n = math.dist((x0, y0), (x1, y1))
    bx, by = x - 0.25 * (x1 - x0) / n, y - 0.25 * (y1 - y0) / n  # 후진 끝에서 0.10 m
    p.node.create_timer(0.05, lambda: other.publish(RobotState(
        robot_id="pinky2", pose=Pose2D(x=bx, y=by))))
    assert wait_until(lambda: p.states[-1][1].state == RobotState.STATE_BLOCKED, 10.0)
    assert RECOVERING in states_since(p)
    assert back_ups(along(truth)) == 0, "it backed up into the robot behind"


def test_no_back_up_into_an_obstacle_behind(harness, tmp_path):
    p = start(harness, tmp_path, robots=LONG, faults=[
        {"at_s": 3.0, "robot": "pinky1", "type": "blocked", "duration_s": 60.0},
        {"at_s": 0.0, "robot": "pinky1", "type": "obstacle_behind", "duration_s": 60.0,
         "distance_m": 0.1}])
    truth = watch_truth(p)
    assert wait_until(lambda: p.states, 5.0)
    p.goal("QO")
    assert wait_until(lambda: p.states[-1][1].state == RobotState.STATE_BLOCKED, 10.0)
    assert RECOVERING in states_since(p)
    assert max(along(truth)) > 0.4, "too near the start: it would back up less than 0.15 m"
    assert back_ups(along(truth)) == 0


def test_after_the_obstacle_goes_it_resumes_says_moving_and_arrives(harness, tmp_path):
    p = start(harness, tmp_path, robots=LONG, faults=[
        {"at_s": 3.0, "robot": "pinky1", "type": "blocked", "duration_s": 2.0}])
    truth = watch_truth(p)
    assert wait_until(lambda: p.states, 5.0)
    res = result_of(p.goal("QO"), 30.0).result
    assert res.success is True
    seen = runs(states_since(p))
    assert seen[seen.index(RobotState.STATE_MOVING):] == [
        RobotState.STATE_MOVING, RECOVERING, RobotState.STATE_MOVING, RobotState.STATE_IDLE]
    assert back_ups(along(truth)) == 1


def test_attempts_reset_after_the_robot_gets_three_tenths_of_a_metre_on(harness, tmp_path):
    """두 시도를 쓴 뒤 0.3 m 넘게 가면 다음 물체에서 다시 RECOVERING부터 한다(명세 6번)."""
    p = start(harness, tmp_path, robots=LONG, faults=[
        {"at_s": 3.0, "robot": "pinky1", "type": "blocked", "duration_s": 5.0},
        {"at_s": 13.0, "robot": "pinky1", "type": "blocked", "duration_s": 2.0}])
    truth = watch_truth(p)
    assert wait_until(lambda: p.states, 5.0)
    assert result_of(p.goal("QO"), 40.0).result.success is True
    seen = runs(states_since(p))
    assert RobotState.STATE_BLOCKED not in seen
    assert seen[seen.index(RobotState.STATE_MOVING):] == [
        RobotState.STATE_MOVING, RECOVERING, RobotState.STATE_MOVING, RECOVERING,
        RobotState.STATE_MOVING, RobotState.STATE_IDLE]
    assert back_ups(along(truth)) == 3, "two in the first episode, one after the reset"


def test_cancel_during_recovering_ends_canceled_and_idle(harness, tmp_path):
    """명세 7번: 후진을 멈추고 CANCELED, 상태는 IDLE(물체가 아직 있어도)."""
    p = start(harness, tmp_path, robots=LONG, faults=[
        {"at_s": 3.0, "robot": "pinky1", "type": "blocked", "duration_s": 60.0}])
    truth = watch_truth(p)
    assert wait_until(lambda: p.states, 5.0)
    handle = p.goal("QO")
    assert wait_until(lambda: p.states[-1][1].state == RECOVERING, 10.0)
    time.sleep(0.5)  # 후진하는 중
    cancel = handle.cancel_goal_async()
    assert wait_until(cancel.done, 5.0)
    res = result_of(handle).result
    assert res.success is False and res.reason == MoveTo.Result.REASON_CANCELED
    assert wait_until(lambda: p.states[-1][1].state == RobotState.STATE_IDLE, 2.0)
    mark = time.monotonic()
    time.sleep(1.5)
    assert set(states_since(p, mark)) == {RobotState.STATE_IDLE}
    assert len({(x, y) for w, x, y in truth if w >= mark}) == 1, "it kept backing up"


def test_a_replacing_goal_during_recovering_cancels_the_old_one(harness, tmp_path):
    """명세 7번: 이전 목표는 CANCELED, 후진은 멈추고 새 목표를 간다. 시도 수는 이어진다."""
    p = start(harness, tmp_path, robots=LONG, faults=[
        {"at_s": 3.0, "robot": "pinky1", "type": "blocked", "duration_s": 60.0}])
    assert wait_until(lambda: p.states, 5.0)
    first = p.goal("QO")
    assert wait_until(lambda: p.states[-1][1].state == RECOVERING, 10.0)
    second = p.goal("QO")
    old = result_of(first).result
    assert (old.success, old.reason) == (False, MoveTo.Result.REASON_CANCELED)
    mark = time.monotonic()
    # 첫 목표에서 시도 하나를 썼다. 새 목표에서 하나 더 쓰고 BLOCKED다
    assert wait_until(lambda: p.states[-1][1].state == RobotState.STATE_BLOCKED, 10.0)
    assert runs(states_since(p, mark))[-2:] == [RECOVERING, RobotState.STATE_BLOCKED]
    assert not ended(second)


def test_heartbeat_loss_during_recovering_stops_everything(harness, tmp_path):
    p = start(harness, tmp_path, heartbeat_timeout_s=BEAT_TIMEOUT_S, robots=LONG, faults=[
        {"at_s": 3.0, "robot": "pinky1", "type": "blocked", "duration_s": 60.0}])
    truth = watch_truth(p)
    timer = beat(p)
    assert wait_until(lambda: p.states, 5.0)
    handle = p.goal("QO")
    assert wait_until(lambda: p.states[-1][1].state == RECOVERING, 10.0)
    timer.cancel()
    res = result_of(handle, 5.0).result
    assert (res.success, res.reason) == (False, MoveTo.Result.REASON_HEARTBEAT_LOST)
    assert wait_until(lambda: p.states[-1][1].state == RobotState.STATE_IDLE, 2.0)
    mark = time.monotonic()
    time.sleep(0.8)
    assert len({(x, y) for w, x, y in truth if w >= mark}) == 1, "it kept moving"
