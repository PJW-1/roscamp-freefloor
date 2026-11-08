"""핑키 FSM의 판단(안내 2절, 핑키_계약 3·4절). ROS 없이 돈다.

시각은 벽시계 초(time.monotonic과 같은 축)로 준다.
"""
import math
from itertools import pairwise

import pytest
from pinky_fsm.core import (
    BLOCKED,
    CANCELED,
    DOCKING,
    HEARTBEAT_LOST,
    IDLE,
    LOCALIZING,
    MOVING,
    NAV_ABORTED,
    PATH_NOT_CONNECTED,
    TIMEOUT,
    CancelNav,
    Finish,
    FsmParams,
    PinkyFsm,
    SendPath,
)

P3, P4, P5, P6 = (0.35, 1.55), (0.35, 1.95), (0.80, 2.05), (1.20, 2.05)
NORTH = math.pi / 2
STILL = (0.0, 0.0)
DRIVING = (0.2, 0.0)


def no_obstacle(_points):
    return False


def obstacle(_points):
    return True


def ready(t=0.0, pose=(*P3, NORTH), **params):
    fsm = PinkyFsm(FsmParams(**params))
    fsm.tick(t, pose, STILL, no_obstacle)
    return fsm


def move(fsm, gid, t, path=(P3, P4, P5), target=P5, pose=(*P3, NORTH), target_id="P-5"):
    yaw = math.atan2(target[1] - path[-2][1], target[0] - path[-2][0]) if len(path) > 1 else 0.0
    return fsm.move(gid, t, target_id, (*target, yaw), list(path), pose)


def kinds(cmds):
    return [type(c).__name__ for c in cmds]


def sends(cmds):
    return [c for c in cmds if isinstance(c, SendPath)]


# ---- 시작 -------------------------------------------------------------------------------


def test_starts_localizing_until_a_pose_arrives():
    fsm = PinkyFsm(FsmParams())
    assert fsm.state() == LOCALIZING
    fsm.tick(0.0, None, STILL, no_obstacle)
    assert fsm.state() == LOCALIZING
    fsm.tick(0.1, (*P3, NORTH), STILL, no_obstacle)
    assert fsm.state() == IDLE and fsm.current_goal() == ""


def test_goal_without_a_pose_fails_at_once():
    fsm = PinkyFsm(FsmParams())
    cmds = move(fsm, "g1", 0.0, pose=None)
    assert cmds == [Finish("g1", False, NAV_ABORTED)]


def test_losing_the_pose_while_moving_stops_and_fails():
    """Do not drive without a pose. The state turns LOCALIZING, so the fleet manager cannot
    trust where the robot is either."""
    fsm = ready()
    move(fsm, "g1", 0.0)
    fsm.heartbeat(0.5)
    cmds = fsm.tick(0.5, None, DRIVING, no_obstacle)
    assert cmds == [Finish("g1", False, NAV_ABORTED), CancelNav()]
    assert fsm.state() == LOCALIZING and fsm.current_goal() == ""


# ---- 경로 따라가기 -----------------------------------------------------------------------


def test_path_goal_sends_the_lane_from_the_robot_to_the_target():
    fsm = ready()
    cmds = move(fsm, "g1", 1.0)
    [send] = sends(cmds)
    assert kinds(cmds) == ["SendPath"]
    assert send.poses[0][:2] == pytest.approx(P3)
    assert send.poses[-1][:2] == pytest.approx(P5)
    assert send.poses[-1][2] == pytest.approx(math.atan2(0.10, 0.45))
    gaps = [math.dist(a[:2], b[:2]) for a, b in pairwise(send.poses)]
    assert max(gaps) <= 0.05 + 1e-9
    assert fsm.state() == MOVING and fsm.current_goal() == "P-5"


def test_point_goal_drives_a_straight_line_to_the_target():
    fsm = ready()
    cmds = fsm.move("g1", 1.0, "P-4", (*P4, NORTH), [], (*P3, NORTH))
    [send] = sends(cmds)
    assert send.poses[0][:2] == pytest.approx(P3) and send.poses[-1] == pytest.approx((*P4, NORTH))
    assert all(p[0] == pytest.approx(0.35) for p in send.poses)


def test_path_starting_behind_the_robot_is_followed_from_the_nearest_point():
    """핑키_계약 2절: 막 지난 지점이 앞에 남을 수 있으니 가장 가까운 점부터 따른다."""
    fsm = ready()
    here = (0.35, 1.80, NORTH)
    [send] = sends(move(fsm, "g1", 1.0, pose=here))
    assert send.poses[0][:2] == pytest.approx((0.35, 1.80))


def test_path_ending_short_of_the_target_still_ends_at_the_target():
    fsm = ready()
    [send] = sends(fsm.move("g1", 1.0, "P-5", (*P5, 0.2), [P3, P4], (*P3, NORTH)))
    assert send.poses[-1] == pytest.approx((*P5, 0.2))


# ---- 목표 교체 (안내 2.1) ------------------------------------------------------------------


def test_new_goal_ends_the_old_one_without_stopping():
    fsm = ready()
    first = sends(move(fsm, "g1", 1.0, path=(P3, P4), target=P4))[0]
    cmds = move(fsm, "g2", 3.0, path=(P3, P4, P5), pose=(0.35, 1.75, NORTH))
    assert cmds[0] == Finish("g1", False, CANCELED)
    assert kinds(cmds) == ["Finish", "SendPath"]  # CancelNav 없음: 멈추지 않는다
    assert sends(cmds)[0].seq > first.seq
    assert fsm.current_goal() == "P-5"


def test_old_nav_results_after_a_replacement_are_ignored():
    fsm = ready()
    first = sends(move(fsm, "g1", 1.0, path=(P3, P4), target=P4))[0]
    move(fsm, "g2", 3.0, pose=(0.35, 1.75, NORTH))
    assert fsm.nav_result(first.seq, "aborted", 3.1, (0.35, 1.76, NORTH), no_obstacle) == []
    assert fsm.state() == MOVING


# ---- 경로 연결 확인 (안내 2.2, R20) ----------------------------------------------------------


def test_path_far_from_the_robot_fails_and_the_robot_stops():
    fsm = ready()
    move(fsm, "g1", 1.0)
    away = [(0.70, 1.55), (0.70, 1.95)]  # 로봇에서 0.35 m 옆
    cmds = fsm.move("g2", 2.0, "X", (0.70, 1.95, NORTH), away, (0.35, 1.70, NORTH))
    assert cmds == [Finish("g1", False, CANCELED), CancelNav(),
                    Finish("g2", False, PATH_NOT_CONNECTED)]
    assert fsm.state() == IDLE


def test_path_within_the_tolerance_is_followed():
    fsm = ready()
    near = [(0.45, 1.55), (0.45, 1.95)]  # 0.10 m 옆
    cmds = fsm.move("g1", 1.0, "X", (0.45, 1.95, NORTH), near, (*P3, NORTH))
    assert kinds(cmds) == ["SendPath"]


def test_connection_is_judged_on_the_received_path_not_the_line_to_the_target():
    """path 끝이 target_pose와 다르면 target까지 잇는 선을 붙인다. 그 선이 로봇을 지나도
    받은 path가 떨어져 있으면 따라가지 않는다(안내 2.2, Codex 리뷰)."""
    fsm = ready()
    away = [(0.70, 1.95), (0.70, 1.55)]  # 로봇에서 0.35 m 옆, 끝이 target과 다르다
    cmds = fsm.move("g1", 1.0, "X", (*P3, NORTH), away, (*P3, NORTH))
    assert "SendPath" not in kinds(cmds)
    assert cmds[-1] == Finish("g1", False, PATH_NOT_CONNECTED)


def test_point_goals_are_not_checked_for_connection():
    fsm = ready()
    cmds = fsm.move("g1", 1.0, "X", (2.0, 2.0, 0.0), [], (*P3, NORTH))
    assert kinds(cmds) == ["SendPath"]


# ---- 취소 (R7) -----------------------------------------------------------------------------


def test_cancel_stops_and_reports_canceled_once():
    fsm = ready()
    move(fsm, "g1", 1.0)
    assert fsm.cancel("g1", 2.0) == [Finish("g1", False, CANCELED), CancelNav()]
    assert fsm.state() == IDLE
    assert fsm.cancel("g1", 3.0) == []  # 같은 취소가 다시 와도 아무것도 하지 않는다


def test_cancel_of_an_old_goal_does_not_touch_the_new_one():
    fsm = ready()
    move(fsm, "g1", 1.0)
    move(fsm, "g2", 2.0)
    assert fsm.cancel("g1", 2.5) == []
    assert fsm.state() == MOVING


# ---- 도착 (R4) ----------------------------------------------------------------------------


def test_success_only_counts_within_the_arrival_tolerance():
    fsm = ready()
    [send] = sends(move(fsm, "g1", 1.0))
    cmds = fsm.nav_result(send.seq, "succeeded", 9.0, (0.78, 2.00, 0.2), no_obstacle)
    assert cmds == [Finish("g1", True, "")] + [CancelNav()]  # no older FollowPath may run on
    assert fsm.state() == IDLE and fsm.current_goal() == ""


def test_success_far_from_the_target_sends_the_rest_again():
    """Nav2가 옛 경로의 끝에서 성공을 보고한 경우(Gazebo 정리 4.3)."""
    fsm = ready()
    [send] = sends(move(fsm, "g1", 1.0))
    cmds = fsm.nav_result(send.seq, "succeeded", 5.0, (0.35, 1.90, NORTH), no_obstacle)
    [again] = sends(cmds)
    assert kinds(cmds) == ["SendPath"] and again.seq > send.seq
    assert again.poses[0][:2] == pytest.approx((0.35, 1.90), abs=1e-6)
    assert fsm.state() == MOVING


def test_repeated_far_successes_give_up_with_nav_aborted():
    fsm = ready()
    seq = sends(move(fsm, "g1", 1.0))[0].seq
    far = (0.35, 1.90, NORTH)
    for i in range(3):
        seq = sends(fsm.nav_result(seq, "succeeded", 2.0 + i, far, no_obstacle))[0].seq
    assert fsm.nav_result(seq, "succeeded", 6.0, far, no_obstacle) == [
        Finish("g1", False, NAV_ABORTED), CancelNav()]


# ---- 실패 (R8) ----------------------------------------------------------------------------


def test_nav_abort_without_an_obstacle_fails_with_nav_aborted():
    fsm = ready()
    [send] = sends(move(fsm, "g1", 1.0))
    cmds = fsm.nav_result(send.seq, "aborted", 4.0, (0.35, 1.70, NORTH), no_obstacle)
    assert cmds == [Finish("g1", False, NAV_ABORTED), CancelNav()]
    assert fsm.state() == IDLE


def test_an_abort_while_the_pose_is_lost_also_cancels_older_follow_paths():
    """Ending a goal on a FollowPath result always cancels: a replaced FollowPath whose
    acceptance is still on its way must not start driving afterwards."""
    fsm = ready()
    move(fsm, "g1", 0.0)
    fsm.heartbeat(0.5)
    fsm.tick(0.5, (0.35, 1.70, NORTH), DRIVING, no_obstacle)
    [second] = sends(move(fsm, "g2", 0.6, pose=(0.35, 1.70, NORTH)))
    cmds = fsm.nav_result(second.seq, "aborted", 0.7, None, no_obstacle)
    assert cmds == [Finish("g2", False, NAV_ABORTED), CancelNav()]


def test_rejected_nav_goal_fails_with_nav_aborted():
    fsm = ready()
    [send] = sends(move(fsm, "g1", 1.0))
    assert fsm.nav_result(send.seq, "rejected", 1.1, (*P3, NORTH), no_obstacle) == [
        Finish("g1", False, NAV_ABORTED), CancelNav()]


def test_rejected_replacement_stops_the_follow_path_still_running():
    """A replacement does not cancel the previous FollowPath (guide 2.1), so when the new one
    is rejected the previous one may still be driving. It must be cancelled."""
    fsm = ready()
    move(fsm, "g1", 0.0)
    fsm.heartbeat(0.5)
    fsm.tick(0.5, (0.35, 1.70, NORTH), DRIVING, no_obstacle)
    [second] = sends(move(fsm, "g2", 0.6, pose=(0.35, 1.70, NORTH)))
    cmds = fsm.nav_result(second.seq, "rejected", 0.7, (0.35, 1.72, NORTH), no_obstacle)
    assert cmds == [Finish("g2", False, NAV_ABORTED), CancelNav()]


# ---- 끝 근처 재전송 (R13) -------------------------------------------------------------------


def test_near_the_end_the_rest_is_sent_again_every_half_second():
    fsm = ready()
    move(fsm, "g1", 0.0, path=(P3, P4), target=P4)
    far = (0.35, 1.70, NORTH)
    fsm.heartbeat(1.0)
    assert sends(fsm.tick(1.0, far, DRIVING, no_obstacle)) == []
    near = (0.35, 1.85, NORTH)  # 끝까지 0.10 m
    fsm.heartbeat(2.0)
    [a] = sends(fsm.tick(2.0, near, DRIVING, no_obstacle))
    assert a.poses[0][:2] == pytest.approx((0.35, 1.85), abs=1e-6)
    assert a.poses[-1][:2] == pytest.approx(P4)
    assert sends(fsm.tick(2.3, near, DRIVING, no_obstacle)) == []
    assert len(sends(fsm.tick(2.5, near, DRIVING, no_obstacle))) == 1


# ---- heartbeat (R10) ----------------------------------------------------------------------


def test_heartbeat_loss_stops_a_running_goal():
    fsm = ready()
    move(fsm, "g1", 0.0)
    fsm.heartbeat(1.0)
    assert fsm.tick(2.4, (0.35, 1.7, NORTH), DRIVING, no_obstacle) == []
    cmds = fsm.tick(2.6, (0.35, 1.7, NORTH), DRIVING, no_obstacle)
    assert cmds == [Finish("g1", False, HEARTBEAT_LOST), CancelNav()]
    assert fsm.state() == IDLE


def test_heartbeat_loss_stops_exactly_at_the_timeout():
    """The promise is a stop command 1.5 s after the last heartbeat, not one tick later."""
    fsm = ready()
    move(fsm, "g1", 0.0)
    fsm.heartbeat(1.0)
    assert fsm.tick(2.45, (0.35, 1.7, NORTH), DRIVING, no_obstacle) == []
    cmds = fsm.tick(2.5, (0.35, 1.7, NORTH), DRIVING, no_obstacle)
    assert cmds == [Finish("g1", False, HEARTBEAT_LOST), CancelNav()]


def test_goal_acceptance_counts_as_a_heartbeat():
    fsm = ready()
    move(fsm, "g1", 10.0)
    assert fsm.tick(11.4, (*P3, NORTH), DRIVING, no_obstacle) == []
    assert Finish("g1", False, HEARTBEAT_LOST) in fsm.tick(11.6, (*P3, NORTH), DRIVING,
                                                           no_obstacle)


def test_idle_robot_ignores_a_missing_heartbeat():
    fsm = ready()
    assert fsm.tick(100.0, (*P3, NORTH), STILL, no_obstacle) == []
    assert fsm.state() == IDLE


def test_zero_heartbeat_timeout_never_stops():
    fsm = ready(heartbeat_timeout_s=0.0)
    move(fsm, "g1", 0.0)
    assert fsm.tick(100.0, (0.35, 1.7, NORTH), DRIVING, no_obstacle) == []


# ---- BLOCKED (핑키_계약 4절, R9) -------------------------------------------------------------


def test_blocked_after_one_second_stopped_with_an_obstacle_ahead():
    fsm = ready()
    move(fsm, "g1", 0.0)
    here = (0.35, 1.70, NORTH)
    fsm.heartbeat(1.0)
    fsm.tick(1.0, here, STILL, obstacle)
    fsm.tick(1.5, here, STILL, obstacle)
    assert fsm.state() == MOVING
    fsm.heartbeat(2.0)
    fsm.tick(2.05, here, STILL, obstacle)
    assert fsm.state() == BLOCKED


def test_stopped_without_an_obstacle_is_not_blocked():
    fsm = ready()
    move(fsm, "g1", 0.0)
    for t in (1.0, 1.5, 2.1):
        fsm.heartbeat(t)
        fsm.tick(t, (0.35, 1.70, NORTH), STILL, no_obstacle)
    assert fsm.state() == MOVING


def test_turning_in_place_is_not_stopped():
    fsm = ready()
    move(fsm, "g1", 0.0)
    for t in (1.0, 1.5, 2.1):
        fsm.heartbeat(t)
        fsm.tick(t, (0.35, 1.70, NORTH), (0.0, 0.6), obstacle)
    assert fsm.state() == MOVING


def test_obstacle_check_looks_half_a_metre_along_the_path():
    fsm = ready()
    move(fsm, "g1", 0.0)
    seen = []

    def spy(points):
        seen.append(list(points))
        return False

    here = (0.35, 1.70, NORTH)
    for t in (1.0, 2.1):
        fsm.heartbeat(t)
        fsm.tick(t, here, STILL, spy)
    pts = seen[-1]
    assert pts[0] == pytest.approx((0.35, 1.70), abs=1e-6)
    route_left = [math.dist(a, b) for a, b in pairwise(pts)]
    assert sum(route_left) == pytest.approx(0.5, abs=0.03)
    assert max(route_left) <= 0.03


def test_blocked_goal_survives_a_nav_abort_and_resumes_when_clear():
    fsm = ready()
    [send] = sends(move(fsm, "g1", 0.0))
    here = (0.35, 1.70, NORTH)
    for t in (1.0, 2.1):
        fsm.heartbeat(t)
        fsm.tick(t, here, STILL, obstacle)
    assert fsm.nav_result(send.seq, "aborted", 2.2, here, obstacle) == []
    assert fsm.state() == BLOCKED
    fsm.heartbeat(3.0)
    assert sends(fsm.tick(3.0, here, STILL, no_obstacle)) == []  # 1초는 기다린다
    assert fsm.state() == BLOCKED
    fsm.heartbeat(4.0)
    [again] = sends(fsm.tick(4.05, here, STILL, no_obstacle))
    assert again.seq > send.seq and again.poses[0][:2] == pytest.approx(here[:2], abs=1e-6)
    assert fsm.state() == MOVING


def test_nav_abort_with_an_obstacle_waits_even_before_blocked_is_reported():
    fsm = ready()
    [send] = sends(move(fsm, "g1", 0.0))
    here = (0.35, 1.70, NORTH)
    fsm.tick(0.5, here, STILL, obstacle)
    assert fsm.nav_result(send.seq, "aborted", 0.6, here, obstacle) == []
    fsm.heartbeat(1.0)
    fsm.tick(1.0, here, STILL, obstacle)
    fsm.heartbeat(1.6)
    fsm.tick(1.6, here, STILL, obstacle)
    assert fsm.state() == BLOCKED


def test_blocked_stays_after_cancel_while_the_obstacle_remains():
    fsm = ready()
    move(fsm, "g1", 0.0)
    here = (0.35, 1.70, NORTH)
    for t in (1.0, 2.1):
        fsm.heartbeat(t)
        fsm.tick(t, here, STILL, obstacle)
    assert fsm.cancel("g1", 3.0) == [Finish("g1", False, CANCELED), CancelNav()]
    fsm.tick(3.5, here, STILL, obstacle)
    assert fsm.state() == BLOCKED and fsm.current_goal() == ""
    fsm.tick(4.0, here, STILL, no_obstacle)
    assert fsm.state() == BLOCKED
    fsm.tick(5.05, here, STILL, no_obstacle)
    assert fsm.state() == IDLE


def test_blocked_goal_ends_with_timeout_when_the_obstacle_stays():
    """A goal that cannot go on must end with a failure or TIMEOUT. The fleet manager cancels a
    blocked goal after blocked_mark_s (5 s), so block_timeout_s (90 s) only matters when the
    fleet manager does not."""
    fsm = ready()
    [send] = sends(move(fsm, "g1", 0.0))
    here = (0.35, 1.70, NORTH)
    fsm.tick(1.0, here, STILL, obstacle)
    assert fsm.nav_result(send.seq, "aborted", 2.0, here, obstacle) == []
    for t in (5.0, 60.0, 91.9):
        fsm.heartbeat(t)
        assert fsm.tick(t, here, STILL, obstacle) == []
    fsm.heartbeat(92.0)
    assert fsm.tick(92.0, here, STILL, obstacle) == [Finish("g1", False, TIMEOUT), CancelNav()]
    assert fsm.state() == BLOCKED and fsm.current_goal() == ""


def test_a_goal_waiting_after_an_obstacle_abort_still_cancels_when_it_ends():
    """The latest FollowPath ended, but an older one may still be on its way: cancel."""
    fsm = ready()
    [send] = sends(move(fsm, "g1", 0.0))
    here = (0.35, 1.70, NORTH)
    assert fsm.nav_result(send.seq, "aborted", 1.0, here, obstacle) == []
    assert fsm.cancel("g1", 2.0) == [Finish("g1", False, CANCELED), CancelNav()]


def test_an_obstacle_that_cleared_in_time_resends_instead_of_timing_out():
    """Both the clear time and block_timeout_s pass before one tick: the clear wins."""
    fsm = ready()
    [send] = sends(move(fsm, "g1", 0.0))
    here = (0.35, 1.70, NORTH)
    fsm.nav_result(send.seq, "aborted", 2.0, here, obstacle)
    fsm.heartbeat(91.5)
    assert fsm.tick(91.5, here, STILL, no_obstacle) == []
    fsm.heartbeat(92.6)
    cmds = fsm.tick(92.6, here, STILL, no_obstacle)
    assert kinds(cmds) == ["SendPath"]


def test_blocked_goal_times_out_while_nav2_still_holds_it():
    """BLOCKED for block_timeout_s also ends a goal Nav2 never gave up on, and stops Nav2."""
    fsm = ready()
    move(fsm, "g1", 0.0)
    here = (0.35, 1.70, NORTH)
    for t in (1.0, 2.0, 60.0, 91.9):
        fsm.heartbeat(t)
        assert fsm.tick(t, here, STILL, obstacle) == []
    assert fsm.state() == BLOCKED
    fsm.heartbeat(92.0)
    assert fsm.tick(92.0, here, STILL, obstacle) == [Finish("g1", False, TIMEOUT), CancelNav()]


def test_block_timeout_counts_again_after_the_obstacle_clears():
    fsm = ready()
    [send] = sends(move(fsm, "g1", 0.0))
    here = (0.35, 1.70, NORTH)
    fsm.nav_result(send.seq, "aborted", 1.0, here, obstacle)
    fsm.heartbeat(1.5)
    fsm.tick(1.5, here, STILL, no_obstacle)
    fsm.heartbeat(2.6)
    [again] = sends(fsm.tick(2.6, here, STILL, no_obstacle))  # cleared: send the rest again
    fsm.nav_result(again.seq, "aborted", 9.0, here, obstacle)
    for t in (10.0, 98.9):
        fsm.heartbeat(t)
        assert Finish("g1", False, TIMEOUT) not in fsm.tick(t, here, STILL, obstacle)
    fsm.heartbeat(99.0)
    assert Finish("g1", False, TIMEOUT) in fsm.tick(99.0, here, STILL, obstacle)


def test_heartbeat_loss_while_blocked_ends_the_goal():
    fsm = ready()
    [send] = sends(move(fsm, "g1", 0.0))
    here = (0.35, 1.70, NORTH)
    fsm.tick(1.0, here, STILL, obstacle)
    fsm.nav_result(send.seq, "aborted", 1.2, here, obstacle)
    cmds = fsm.tick(1.6, here, STILL, obstacle)
    assert Finish("g1", False, HEARTBEAT_LOST) in cmds


# ---- 도킹 ---------------------------------------------------------------------------------


def test_dock_ends_the_move_and_fails_while_pinky_dock_is_missing():
    """pinky_dock은 아직 없다. 도킹을 했다고 꾸미지 않고 실패를 돌려준다."""
    fsm = ready()
    move(fsm, "g1", 0.0)
    cmds = fsm.dock("d1", 1.0, "WORK")
    assert cmds[:2] == [Finish("g1", False, CANCELED), CancelNav()]
    assert cmds[2].goal_id == "d1" and cmds[2].success is False
    assert fsm.state() == IDLE
    assert DOCKING  # 상태 값은 있다(pinky_dock이 들어오면 쓴다)


# ---- 남은 거리 ------------------------------------------------------------------------------


def test_distance_remaining_follows_the_path():
    fsm = ready()
    move(fsm, "g1", 0.0)
    assert fsm.distance_remaining((0.35, 1.75, NORTH)) == pytest.approx(
        0.20 + math.dist(P4, P5))
    assert fsm.distance_remaining(None) == pytest.approx(0.40 + math.dist(P4, P5))
