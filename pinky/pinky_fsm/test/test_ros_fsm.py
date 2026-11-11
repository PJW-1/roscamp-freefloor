"""실제 액션·토픽 인터페이스로 pinky_fsm 노드를 시험한다. Nav2 대신 FakeNav를 쓴다."""
import math
import time

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces.msg")
pytest.importorskip("nav2_msgs.action")

from conftest import CONTRACT, wait_until  # noqa: E402
from fake_nav import FakeNav  # noqa: E402
from fleet_interfaces.action import Dock, MoveTo  # noqa: E402
from fleet_interfaces.msg import RobotState  # noqa: E402
from probe import WAYPOINTS, Probe, lane_goal  # noqa: E402


def start(harness, heartbeat_timeout_s=0.0, nav=True, beat_hz=0.0, **fsm_params):
    from pinky_fsm.node import PinkyFsmNode
    fake = harness.add(FakeNav()) if nav else None
    harness.add(PinkyFsmNode(parameter_overrides=harness.params(
        robot_id="pinky1", contract_file=str(CONTRACT),
        heartbeat_timeout_s=heartbeat_timeout_s, **fsm_params)))
    probe = harness.add(Probe(beat_hz=beat_hz))
    assert probe.ready()
    if nav:
        assert wait_until(lambda: (s := probe.last_state()) is not None
                          and s.state == RobotState.STATE_IDLE, 10.0)
    return fake, probe


def test_reports_localizing_without_a_pose(harness):
    _, probe = start(harness, nav=False)
    assert wait_until(lambda: len(probe.states) >= 3, 5.0)
    s = probe.last_state()
    assert s.state == RobotState.STATE_LOCALIZING and s.robot_id == "pinky1"


def test_state_is_published_at_ten_hertz_with_the_map_pose(harness):
    _, probe = start(harness)
    t0 = time.monotonic()
    time.sleep(2.0)
    n = len([1 for t, _ in probe.states if t >= t0])
    assert 17 <= n <= 23
    s = probe.last_state()
    assert (s.pose.x, s.pose.y) == (pytest.approx(0.35, abs=1e-3), pytest.approx(1.55, abs=1e-3))
    assert s.current_goal == ""


def test_path_goal_arrives_within_the_tolerance(harness):
    _, probe = start(harness)
    h = probe.send(lane_goal(["P-3", "P-4"]))
    assert wait_until(lambda: probe.last_state().state == RobotState.STATE_MOVING, 2.0)
    assert probe.last_state().current_goal == "P-4"
    res = probe.result(h)
    assert res.result.success is True and res.result.reason == ""
    time.sleep(0.3)
    s = probe.last_state()
    assert math.dist((s.pose.x, s.pose.y), WAYPOINTS["P-4"]) <= 0.15
    assert s.state == RobotState.STATE_IDLE


def test_extension_replaces_the_goal_without_stopping(harness):
    """안내 2.1: 경로를 늘린 새 move_to가 와도 서지 않는다."""
    fake, probe = start(harness)
    first = probe.send(lane_goal(["P-3", "P-4"]))
    time.sleep(0.4)
    t_ext = time.monotonic()
    second = probe.send(lane_goal(["P-3", "P-4", "P-5"]))
    old = probe.result(first)
    assert old.result.success is False and old.result.reason == MoveTo.Result.REASON_CANCELED
    new = probe.result(second)
    assert new.result.success is True
    assert fake.cancels == []  # FollowPath를 취소하지 않고 새 목표를 바로 보냈다
    assert len(fake.goals) >= 2  # 셋째부터는 끝 근처 재전송(R13)이다
    assert not fake.stopped_between(t_ext - 0.2, t_ext + 0.4)


def test_cancel_stops_and_returns_canceled(harness):
    fake, probe = start(harness)
    h = probe.send(lane_goal(["P-3", "P-4", "P-5"]))
    time.sleep(0.3)
    h.cancel_goal_async()
    res = probe.result(h)
    assert res.result.success is False and res.result.reason == MoveTo.Result.REASON_CANCELED
    assert wait_until(lambda: len(fake.cancels) >= 1, 2.0)
    assert wait_until(lambda: fake.v == 0.0, 1.0)
    assert wait_until(lambda: probe.last_state().state == RobotState.STATE_IDLE, 1.0)


def test_disconnected_path_fails_and_is_not_followed(harness):
    """안내 2.2: 로봇에서 0.15 m 넘게 떨어진 경로는 따라가지 않는다."""
    fake, probe = start(harness)
    goal = lane_goal(["P-5", "P-6"])  # 로봇은 P-3에 있다
    res = probe.result(probe.send(goal))
    assert res.result.success is False
    assert res.result.reason == MoveTo.Result.REASON_PATH_NOT_CONNECTED
    assert fake.goals == []


def test_point_goal_drives_to_the_target(harness):
    _, probe = start(harness)
    res = probe.result(probe.send(lane_goal(["P-3", "P-4"], path=False)))
    assert res.result.success is True


def test_heartbeat_loss_stops_the_robot(harness):
    fake, probe = start(harness, heartbeat_timeout_s=1.0)
    t0 = time.monotonic()
    h = probe.send(lane_goal(["P-3", "P-4", "P-5", "P-6", "N"]))
    res = probe.result(h)
    took = time.monotonic() - t0
    assert res.result.success is False
    assert res.result.reason == MoveTo.Result.REASON_HEARTBEAT_LOST
    assert 0.9 <= took <= 1.5
    assert wait_until(lambda: fake.v == 0.0 and len(fake.cancels) >= 1, 1.0)


def test_heartbeat_keeps_the_goal_running(harness):
    _, probe = start(harness, heartbeat_timeout_s=1.0, beat_hz=2.0)
    res = probe.result(probe.send(lane_goal(["P-3", "P-4", "P-5"])))
    assert res.result.success is True


def test_obstacle_ahead_reports_blocked_and_resumes(harness):
    """핑키_계약 4절: 서고 2초 안에 BLOCKED, 목표는 끝내지 않고, 비키면 이어 간다.
    recovery_attempts=0은 자가 복구 두 번을 이미 쓴 것과 같다."""
    fake, probe = start(harness, recovery_attempts=0)
    fake.obstacle = (0.35, 1.95)  # P-4 위
    h = probe.send(lane_goal(["P-3", "P-4", "P-5"]))
    assert wait_until(lambda: fake.v == 0.0 and fake.pose[1] > 1.6, 5.0)
    t_stop = time.monotonic()
    assert wait_until(lambda: probe.last_state().state == RobotState.STATE_BLOCKED, 2.5)
    assert time.monotonic() - t_stop <= 2.0 + 0.2
    time.sleep(1.5)  # FakeNav는 1초 뒤 FollowPath를 실패로 끝낸다
    assert not h.get_result_async().done()
    assert probe.last_state().state == RobotState.STATE_BLOCKED
    fake.obstacle = None
    res = probe.result(h, timeout=10.0)
    assert res.result.success is True
    assert len(fake.goals) >= 2  # 비킨 뒤 남은 경로를 다시 보냈다


def test_default_distance_rule_only_clears_the_costmap_close_to_the_path_start(harness):
    """0.15 m from the path start is inside release + eps (0.23 m): no back-up, only a clear."""
    fake, probe = start(harness)
    fake.obstacle = (0.35, 1.95)
    probe.send(lane_goal(["P-3", "P-4", "P-5"]))
    assert wait_until(lambda: len(fake.clears) >= 1, 6.0)
    assert fake.backups == []


def test_obstacle_ahead_backs_up_twice_then_reports_blocked(harness):
    """핑키_계약 3절 7번(2026-11-09 제안): 물러나고 지도를 비운 뒤 다시 가 보기를 두 번까지
    하고, 그동안 RECOVERING으로 보고한다. 그래도 막히면 4절의 BLOCKED로 넘긴다."""
    # FakeNav의 레인은 짧다. release_distance_m=0으로 물러날 자리를 만든다(거리 규칙은 test_core)
    fake, probe = start(harness, release_distance_m=0.0)
    fake.obstacle = (0.35, 1.95)  # P-4 위. FakeNav는 0.25 m 앞(y 1.70)에서 선다
    h = probe.send(lane_goal(["P-3", "P-4", "P-5"]))
    assert wait_until(lambda: probe.last_state().state == RobotState.STATE_RECOVERING, 5.0)
    assert wait_until(lambda: probe.last_state().state == RobotState.STATE_BLOCKED, 12.0)
    # s = 0.15 m: min(0.15, 0.15 - (0 + 0.03)) = 0.12 m
    assert [d for _, d, _ in fake.backups] == [pytest.approx(0.12, abs=0.015)] * 2
    assert all(v == pytest.approx(0.08) for _, _, v in fake.backups)
    assert len(fake.clears) == 2
    assert not h.get_result_async().done()  # 목표는 열어 둔다(4절)
    fake.obstacle = None
    res = probe.result(h, timeout=10.0)
    assert res.result.success is True


def test_something_behind_stops_the_backup_and_the_path_is_tried_again(harness):
    fake, probe = start(harness, release_distance_m=0.0)
    fake.obstacle = (0.35, 1.95)
    fake.wall_behind_m = 0.01  # Nav2 BackUp: COLLISION_AHEAD로 끝난다
    probe.send(lane_goal(["P-3", "P-4", "P-5"]))
    assert wait_until(lambda: len(fake.clears) >= 1, 6.0)
    assert fake.backup_ends[0] == ("collision", 714)
    assert wait_until(lambda: len(fake.goals) >= 2, 1.0)  # 남은 경로를 다시 보냈다
    assert probe.last_state().state == RobotState.STATE_RECOVERING


def test_dock_fails_honestly_while_pinky_dock_is_missing(harness):
    _, probe = start(harness)
    res = probe.result(probe.send(Dock.Goal(dock_id="WORK"), client=probe.dock))
    assert res.result.success is False


def _retreat_client(probe):
    from fleet_interfaces.action import Retreat
    from probe import C
    from rclpy.action import ActionClient
    client = ActionClient(probe, Retreat, C.name("retreat", pinky="pinky1"))
    assert client.wait_for_server(10.0)
    return client


def test_a_retreat_stops_the_move_and_backs_up_with_the_same_backup_code(harness):
    """버전 2 1-A(계약 0.9-draft 제안): 이동을 멈추고 Nav2 BackUp으로 곧게 물러난다."""
    from fleet_interfaces.action import Retreat
    fake, probe = start(harness)
    move = probe.send(lane_goal(["P-3", "P-4", "P-5"]))
    assert wait_until(lambda: len(fake.goals) >= 1, 5.0)
    h = probe.send(Retreat.Goal(distance_m=0.1, speed_mps=0.1), client=_retreat_client(probe))
    assert probe.result(move, timeout=5.0).result.reason == MoveTo.Result.REASON_CANCELED
    res = probe.result(h, timeout=10.0).result
    assert (res.success, res.reason) == (True, "")
    assert [(d, v) for _, d, v in fake.backups] == [(pytest.approx(0.1), pytest.approx(0.1))]


def test_something_behind_ends_a_retreat_with_obstacle_behind(harness):
    from fleet_interfaces.action import Retreat
    fake, probe = start(harness)
    fake.wall_behind_m = 0.01
    h = probe.send(Retreat.Goal(distance_m=0.1, speed_mps=0.1), client=_retreat_client(probe))
    res = probe.result(h, timeout=10.0).result
    assert (res.success, res.reason) == (False, Retreat.Result.REASON_OBSTACLE_BEHIND)


def _back_path(*ids):
    from geometry_msgs.msg import Pose
    out = []
    for i in ids:
        p = Pose()
        p.position.x, p.position.y = WAYPOINTS[i]
        out.append(p)
    return out


def test_a_retreat_with_a_path_backs_along_it_on_the_reverse_controller(harness):
    """Retreat.path(계약 0.9-draft 제안): 지나온 길을 거꾸로 적은 경로를 FollowPathReverse로
    후진해 따른다. BackUp은 쓰지 않고, 로봇은 돌지 않는다(북쪽을 본 채 남쪽으로)."""
    from fleet_interfaces.action import Retreat
    fake, probe = start(harness)
    assert probe.result(probe.send(lane_goal(["P-3", "P-4"]))).result.success is True
    h = probe.send(Retreat.Goal(distance_m=0.4, speed_mps=0.05, path=_back_path("P-4", "P-3")),
                   client=_retreat_client(probe))
    res = probe.result(h, timeout=10.0).result
    assert (res.success, res.reason) == (True, "")
    assert fake.controller_ids[-1] == "FollowPathReverse" and fake.backups == []
    assert all(yaw == pytest.approx(math.pi / 2) for _, _, yaw in fake.path)
    assert math.dist(fake.pose[:2], WAYPOINTS["P-3"]) <= 0.05
    assert fake.pose[2] == pytest.approx(math.pi / 2)


def test_something_behind_on_the_path_ends_a_retreat_with_obstacle_behind(harness):
    """FakeNav는 RPP처럼 움직이는 쪽(후진이면 뒤)의 물체에 서고 NO_VALID_CONTROL로 끝낸다."""
    from fleet_interfaces.action import Retreat
    fake, probe = start(harness)
    assert probe.result(probe.send(lane_goal(["P-3", "P-4"]))).result.success is True
    fake.obstacle = (0.35, 1.60)  # 뒤, P-3 근처
    h = probe.send(Retreat.Goal(distance_m=0.4, speed_mps=0.05, path=_back_path("P-4", "P-3")),
                   client=_retreat_client(probe))
    res = probe.result(h, timeout=10.0).result
    assert (res.success, res.reason) == (False, Retreat.Result.REASON_OBSTACLE_BEHIND)
    assert fake.pose[1] > 1.80


def test_a_retreat_path_away_from_the_robot_fails(harness):
    from fleet_interfaces.action import Retreat
    fake, probe = start(harness)
    h = probe.send(Retreat.Goal(distance_m=0.4, speed_mps=0.05, path=_back_path("P-6", "P-5")),
                   client=_retreat_client(probe))
    res = probe.result(h, timeout=5.0).result
    assert (res.success, res.reason) == (False, Retreat.Result.REASON_FAILED)
    assert fake.goals == [] and fake.backups == []


def _fake_node(late):
    import threading
    from types import SimpleNamespace
    infos = []
    node = SimpleNamespace(
        _lock=threading.RLock(), _nav_pending={3}, _late=late, _nav_handles={3: object()},
        get_logger=lambda: SimpleNamespace(warn=infos.append, info=infos.append),
        _pose=lambda: None, _obstacle=lambda _p: False)
    return node, infos


def _broken():
    raise RuntimeError("lost")


def test_a_follow_path_goal_whose_send_fails_counts_as_rejected():
    """Codex 리뷰 2: future 예외로 콜백이 죽으면 그 FollowPath는 끝내 결과가 없다."""
    from types import SimpleNamespace

    from pinky_fsm.node import PinkyFsmNode
    late = []
    node, infos = _fake_node(late)
    PinkyFsmNode._on_nav_response(node, 3, SimpleNamespace(result=_broken))
    assert late == [(3, "rejected")] and node._nav_pending == set() and infos


class _CancelFuture:
    def __init__(self, response=None, error=None):
        self.callbacks, self._response, self._error = [], response, error

    def add_done_callback(self, cb):
        self.callbacks.append(cb)

    def result(self):
        if self._error:
            raise self._error
        return self._response


def test_only_an_accepted_cancel_lets_the_core_forget_a_follow_path():
    """재검토 1: 취소 응답에 그 goal이 들어 있을 때만 core에 알린다(거절, 오류는 알리지 않는다)."""
    from types import SimpleNamespace

    from action_msgs.srv import CancelGoal
    from pinky_fsm.node import PinkyFsmNode
    told = []
    node, infos = _fake_node([])
    node._fsm = SimpleNamespace(nav_cancel_accepted=lambda seq, now: told.append(seq) or [])
    node._run = lambda cmds: None
    yes = CancelGoal.Response(goals_canceling=[_goal_info()])
    PinkyFsmNode._on_nav_cancel(node, 3, _CancelFuture(yes))
    PinkyFsmNode._on_nav_cancel(node, 4, _CancelFuture(CancelGoal.Response(
        return_code=CancelGoal.Response.ERROR_REJECTED)))
    PinkyFsmNode._on_nav_cancel(node, 5, _CancelFuture(error=RuntimeError("lost")))
    assert told == [3] and infos


def test_a_cancel_answered_goal_terminated_or_unknown_also_lets_the_core_forget():
    """재검토 2: 이미 끝났거나 서버가 모르는 goal은 돌지 않는다. 결과를 잃은 goal이 끝나 있으면
    취소는 이렇게 답하고, 그때 잊지 않으면 그 뒤 모든 Retreat가 TIMEOUT이다."""
    from types import SimpleNamespace

    from action_msgs.srv import CancelGoal
    from pinky_fsm.node import PinkyFsmNode
    told = []
    node, _ = _fake_node([])
    node._nav_handles = {3: object(), 4: object()}
    node._fsm = SimpleNamespace(nav_cancel_accepted=lambda seq, now: told.append(seq) or [])
    node._run = lambda cmds: None
    for seq, code in ((3, CancelGoal.Response.ERROR_GOAL_TERMINATED),
                      (4, CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID)):
        PinkyFsmNode._on_nav_cancel(node, seq, _CancelFuture(CancelGoal.Response(return_code=code)))
    assert told == [3, 4] and not node._nav_handles


def _goal_info():
    from action_msgs.msg import GoalInfo
    return GoalInfo()


def test_every_follow_path_cancel_reports_its_answer():
    from types import SimpleNamespace

    from pinky_fsm.node import PinkyFsmNode
    futures = {}
    handles = {7: SimpleNamespace(cancel_goal_async=lambda: futures.setdefault(7,
                                                                             _CancelFuture()))}
    node = SimpleNamespace(_nav_handles=handles, _nav_pending={8}, _cancel_on_accept=set())
    node._on_nav_cancel = lambda seq, fut: None
    PinkyFsmNode._cancel_all_nav(node)
    assert len(futures[7].callbacks) == 1 and node._cancel_on_accept == {8}


def test_a_lost_follow_path_result_keeps_the_goal_and_cancels_it():
    """재검토 2: 결과를 잃어도 goal은 돌 수 있다. 핸들을 두고 취소를 보내며, core에는
    nav_lost로 알린다(이동은 aborted처럼 끝나지만 취소가 받아들여질 때까지 잊지 않는다)."""
    from types import SimpleNamespace

    from pinky_fsm.node import PinkyFsmNode
    ran, futures = [], []
    node, infos = _fake_node([])
    handle = SimpleNamespace(cancel_goal_async=lambda: futures.append(_CancelFuture())
                             or futures[-1])
    node._nav_handles = {3: handle}
    node._fsm = SimpleNamespace(nav_lost=lambda *a: ran.append(a[0]) or [])
    node._run = lambda cmds: None
    node._on_nav_cancel = lambda seq, fut: None
    PinkyFsmNode._on_nav_result(node, 3, SimpleNamespace(result=_broken))
    assert ran == [3] and node._nav_handles == {3: handle} and infos
    assert len(futures) == 1 and len(futures[0].callbacks) == 1


def test_a_goal_at_a_lock_boundary_publishes_a_speed_limit_and_lifts_it_at_the_end(harness):
    """버전 2 4절 horizon 최소판(계약 0.8-draft 제안): final_goal이 거짓이면 끝 근처에서
    nav2 speed_limit으로 줄이고, 목표가 끝나면 푼다."""
    fake, probe = start(harness)
    goal = lane_goal(["P-3", "P-4"])
    goal.final_goal = False
    res = probe.result(probe.send(goal))
    assert res.result.success is True
    assert wait_until(lambda: fake.speed_limits and fake.speed_limits[-1] == 0.0, 2.0)
    assert any(0.0 < v < 0.3 for v in fake.speed_limits)


def test_the_speed_limit_latency_margin_is_a_node_parameter(harness):
    from pinky_fsm.core import FsmParams
    from pinky_fsm.node import PinkyFsmNode

    def node(**overrides):
        return harness.add(PinkyFsmNode(parameter_overrides=harness.params(
            robot_id="pinky1", contract_file=str(CONTRACT), **overrides)))
    assert node().get_parameter("horizon_latency_s").value == FsmParams().horizon_latency_s
    assert node(horizon_latency_s=0.1)._fsm.p.horizon_latency_s == 0.1


def test_a_final_goal_publishes_no_speed_limit(harness):
    fake, probe = start(harness)
    assert probe.result(probe.send(lane_goal(["P-3", "P-4"]))).result.success is True
    time.sleep(0.3)
    assert all(v == 0.0 for v in fake.speed_limits), "처음에 푸는 0만 있다"


def test_without_a_through_controller_every_goal_uses_the_normal_controller(harness):
    """Default through_controller_id is empty: exactly the old behaviour."""
    fake, probe = start(harness)
    goal = lane_goal(["P-3", "P-4"])
    goal.final_goal = False
    assert probe.result(probe.send(goal)).result.success is True
    assert fake.controller_ids and set(fake.controller_ids) == {"FollowPath"}


def test_a_goal_at_a_lock_boundary_uses_the_through_controller(harness):
    """Nav2 RPP slows down near every path end; a non-final goal goes to a second RPP without
    that approach slow-down and keeps the sqrt(2 a d) speed limit as its only slow-down."""
    fake, probe = start(harness, through_controller_id="FollowPathThrough")
    goal = lane_goal(["P-3", "P-4"])
    goal.final_goal = False
    assert probe.result(probe.send(goal)).result.success is True
    assert fake.controller_ids and set(fake.controller_ids) == {"FollowPathThrough"}
    assert any(0.0 < v < 0.3 for v in fake.speed_limits)


def test_a_final_goal_keeps_the_normal_controller_with_a_through_controller_set(harness):
    fake, probe = start(harness, through_controller_id="FollowPathThrough")
    assert probe.result(probe.send(lane_goal(["P-3", "P-4"]))).result.success is True
    assert fake.controller_ids and set(fake.controller_ids) == {"FollowPath"}


def test_a_final_goal_replacing_a_partial_one_switches_back_to_the_normal_controller(harness):
    fake, probe = start(harness, through_controller_id="FollowPathThrough")
    partial = lane_goal(["P-3", "P-4", "P-5"])
    partial.final_goal = False
    probe.send(partial)
    assert wait_until(lambda: fake.controller_ids, 5.0)
    res = probe.result(probe.send(lane_goal(["P-3", "P-4", "P-5"])))
    assert res.result.success is True
    assert fake.controller_ids[0] == "FollowPathThrough"
    assert fake.controller_ids[-1] == "FollowPath"
    assert not fake.cancels, "the switch replaces the goal without stopping (guide 2.1)"
