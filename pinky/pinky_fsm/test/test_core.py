"""핑키 FSM의 판단(안내 2절, 핑키_계약 3·4절). ROS 없이 돈다.

시각은 벽시계 초(time.monotonic과 같은 축)로 준다.
"""
import math
from itertools import pairwise

import pytest
from pinky_fsm.backup import BackUp, CancelBackUp
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
    RECOVERING,
    TIMEOUT,
    CancelNav,
    ClearCostmap,
    Finish,
    FsmParams,
    LimitSpeed,
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
    """With both self-recovery attempts used (here: none allowed)."""
    fsm = ready(recovery_attempts=0)
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
# 자가 복구를 다 쓴 뒤의 동작이다. recovery_attempts=0은 두 번을 이미 쓴 것과 같다.


def test_blocked_after_one_second_stopped_with_an_obstacle_ahead():
    fsm = ready(recovery_attempts=0)
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
    fsm = ready(recovery_attempts=0)
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
    fsm = ready(recovery_attempts=0)
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
    fsm = ready(recovery_attempts=0)
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
    fsm = ready(recovery_attempts=0)
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
    fsm = ready(recovery_attempts=0)
    [send] = sends(move(fsm, "g1", 0.0))
    here = (0.35, 1.70, NORTH)
    assert fsm.nav_result(send.seq, "aborted", 1.0, here, obstacle) == []
    assert fsm.cancel("g1", 2.0) == [Finish("g1", False, CANCELED), CancelNav()]


def test_an_obstacle_that_cleared_in_time_resends_instead_of_timing_out():
    """Both the clear time and block_timeout_s pass before one tick: the clear wins."""
    fsm = ready(recovery_attempts=0)
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
    fsm = ready(recovery_attempts=0)
    move(fsm, "g1", 0.0)
    here = (0.35, 1.70, NORTH)
    for t in (1.0, 2.0, 60.0, 91.9):
        fsm.heartbeat(t)
        assert fsm.tick(t, here, STILL, obstacle) == []
    assert fsm.state() == BLOCKED
    fsm.heartbeat(92.0)
    assert fsm.tick(92.0, here, STILL, obstacle) == [Finish("g1", False, TIMEOUT), CancelNav()]


def test_block_timeout_counts_again_after_the_obstacle_clears():
    fsm = ready(recovery_attempts=0)
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
    fsm = ready(recovery_attempts=0)
    [send] = sends(move(fsm, "g1", 0.0))
    here = (0.35, 1.70, NORTH)
    fsm.tick(1.0, here, STILL, obstacle)
    fsm.nav_result(send.seq, "aborted", 1.2, here, obstacle)
    cmds = fsm.tick(1.6, here, STILL, obstacle)
    assert Finish("g1", False, HEARTBEAT_LOST) in cmds


# ---- 자가 복구 (핑키_계약 3절 7번, 4절; 2026-11-09 제안) ---------------------------------------

# 후진 거리 = min(0.15, 마지막으로 지난 꼭짓점(경로 시작 포함)부터 온 거리 - (0.2 + 0.03)).
# 0.2는 관제 release_distance_m: 꼭짓점에서 이만큼 가면 뒤 레인을 놓고 뒤차가 꼭짓점에 설 수 있다
STUCK = (0.35, 1.87, NORTH)  # P3(경로 시작)에서 0.32 m. 0.32 - 0.23 = 0.09 m
BEHIND = (0.35, 1.78, NORTH)  # 0.09 m 물러난 자리
EAST = math.atan2(P5[1] - P4[1], P5[0] - P4[0])


def along(a, b, s):
    """a에서 b 쪽으로 s m 간 점."""
    d = math.dist(a, b)
    return (a[0] + (b[0] - a[0]) * s / d, a[1] + (b[1] - a[1]) * s / d)


def stand(fsm, t0, pose, seen=obstacle):
    """t0부터 1초 넘게 서 있는다. 마지막 tick의 명령을 돌려준다."""
    cmds = []
    for t in (t0, t0 + 1.05):
        fsm.heartbeat(t)
        cmds = fsm.tick(t, pose, STILL, seen)
    return cmds


def backups(cmds):
    return [c for c in cmds if isinstance(c, BackUp)]


def test_standing_at_an_obstacle_backs_up_instead_of_reporting_blocked():
    fsm = ready()
    move(fsm, "g1", 0.0)
    cmds = stand(fsm, 1.0, STUCK)
    # FollowPath first: BackUp and FollowPath both drive cmd_vel
    assert kinds(cmds) == ["CancelNav", "BackUp"]
    [b] = backups(cmds)
    assert b.distance_m == pytest.approx(0.09)
    assert b.speed_mps == pytest.approx(0.08) and b.time_allowance_s == pytest.approx(5.0)
    assert fsm.state() == RECOVERING and fsm.current_goal() == "P-5"


def test_after_backing_up_the_costmap_is_cleared_and_the_rest_followed_again():
    fsm = ready()
    move(fsm, "g1", 0.0)
    [b] = backups(stand(fsm, 1.0, STUCK))
    fsm.heartbeat(3.5)
    cmds = fsm.backup_result(b.seq, "succeeded", 3.5, BEHIND)
    assert kinds(cmds) == ["ClearCostmap", "SendPath"]
    assert sends(cmds)[0].poses[0][:2] == pytest.approx(BEHIND[:2], abs=1e-6)
    assert sends(cmds)[0].poses[-1][:2] == pytest.approx(P5)
    assert fsm.state() == RECOVERING  # 막힌 곳을 지나야 끝난다


def test_getting_past_the_stuck_point_ends_the_recovery():
    fsm = ready()
    move(fsm, "g1", 0.0)
    [b] = backups(stand(fsm, 1.0, STUCK))
    fsm.backup_result(b.seq, "succeeded", 3.0, BEHIND)
    fsm.heartbeat(4.0)
    fsm.tick(4.0, (0.35, 1.90, NORTH), DRIVING, no_obstacle)  # 0.03 m past
    assert fsm.state() == RECOVERING
    fsm.heartbeat(4.2)
    fsm.tick(4.2, (0.35, 1.93, NORTH), DRIVING, no_obstacle)  # 0.06 m past
    assert fsm.state() == MOVING


def test_two_failed_attempts_fall_back_to_blocked():
    fsm = ready()
    move(fsm, "g1", 0.0)
    [b1] = backups(stand(fsm, 1.0, STUCK))
    fsm.backup_result(b1.seq, "succeeded", 3.0, BEHIND)
    [b2] = backups(stand(fsm, 4.0, STUCK))  # 다시 같은 곳에서 섰다
    assert b2.seq > b1.seq and fsm.state() == RECOVERING
    fsm.backup_result(b2.seq, "succeeded", 6.0, BEHIND)
    cmds = stand(fsm, 7.0, STUCK)
    assert backups(cmds) == [] and ClearCostmap() not in cmds
    assert fsm.state() == BLOCKED


def test_a_nav_abort_starts_a_recovery_even_without_an_obstacle():
    """RPP stops for its own collision check where the lane check sees nothing (the robot drifted
    toward a rack, 2026-11-08). The abort itself is the trigger."""
    fsm = ready()
    [send] = sends(move(fsm, "g1", 0.0))
    fsm.heartbeat(2.0)
    fsm.tick(2.0, STUCK, STILL, no_obstacle)
    cmds = fsm.nav_result(send.seq, "aborted", 2.1, STUCK, no_obstacle)
    assert kinds(cmds) == ["CancelNav", "BackUp"]
    assert fsm.state() == RECOVERING


def test_a_nav_abort_with_both_attempts_used_fails_with_nav_aborted():
    fsm = ready()
    [send] = sends(move(fsm, "g1", 0.0))
    seq = send.seq
    for t in (2.0, 5.0):
        fsm.heartbeat(t)
        fsm.tick(t, STUCK, STILL, no_obstacle)
        [b] = backups(fsm.nav_result(seq, "aborted", t + 0.1, STUCK, no_obstacle))
        [again] = sends(fsm.backup_result(b.seq, "succeeded", t + 1.0, BEHIND))
        seq = again.seq
    cmds = fsm.nav_result(seq, "aborted", 7.0, STUCK, no_obstacle)
    assert cmds == [Finish("g1", False, NAV_ABORTED), CancelNav()]
    assert fsm.state() == IDLE


def test_no_room_behind_only_clears_the_costmap_and_tries_again():
    fsm = ready()
    move(fsm, "g1", 0.0)
    near_start = (0.35, 1.79, NORTH)  # 0.24 m: 0.24 - 0.23 < backup_min_m 0.02
    cmds = stand(fsm, 1.0, near_start)
    assert kinds(cmds) == ["CancelNav", "ClearCostmap", "SendPath"]
    assert fsm.state() == RECOVERING


def test_a_skewed_robot_backs_up_less_to_stay_near_the_lane():
    fsm = ready()
    far = (0.35, 2.15)
    move(fsm, "g1", 0.0, path=(P3, far), target=far, target_id="F")
    # 0.01 m right of the lane, facing 30 deg left of it: backing up drifts further right.
    # The drift stops at backup_lateral_limit_m 0.03, the position error the fleet allows for
    skewed = (0.36, 1.95, NORTH + math.radians(30))
    [b] = backups(stand(fsm, 1.0, skewed))
    assert b.distance_m == pytest.approx(0.04, abs=1e-3)  # (0.01 + 0.03) / sin 30 deg


def test_a_robot_already_off_the_lane_does_not_back_up_further_off():
    fsm = ready()
    far = (0.35, 2.15)
    move(fsm, "g1", 0.0, path=(P3, far), target=far, target_id="F")
    skewed = (0.40, 1.95, NORTH + math.radians(30))  # 0.05 m right, beyond the 0.03 limit
    cmds = stand(fsm, 1.0, skewed)
    assert kinds(cmds) == ["CancelNav", "ClearCostmap", "SendPath"]  # no BackUp


def test_past_a_corner_the_distance_counts_from_the_corner():
    """Back from P4 toward P3 the robot would stand on the lane behind, which the fleet manager
    releases 0.2 m past P4. A follower may already wait at P4."""
    fsm = ready()
    move(fsm, "g1", 0.0)
    for t, pose in ((0.5, (0.35, 1.80, NORTH)), (0.8, (*P4, NORTH))):
        fsm.heartbeat(t)
        fsm.tick(t, pose, DRIVING, no_obstacle)
    [b] = backups(stand(fsm, 1.0, (*along(P4, P5, 0.30), EAST)))
    assert b.distance_m == pytest.approx(0.07, abs=1e-6)  # 0.30 - 0.23, not 0.15 from P3


def test_just_past_a_corner_only_clears_the_costmap():
    fsm = ready()
    move(fsm, "g1", 0.0)
    for t, pose in ((0.5, (0.35, 1.80, NORTH)), (0.8, (*P4, NORTH))):
        fsm.heartbeat(t)
        fsm.tick(t, pose, DRIVING, no_obstacle)
    cmds = stand(fsm, 1.0, (*along(P4, P5, 0.10), EAST))  # 0.50 m from P3, 0.10 m from P4
    assert kinds(cmds) == ["CancelNav", "ClearCostmap", "SendPath"]
    assert fsm.state() == RECOVERING


def test_follow_path_results_are_ignored_while_backing_up():
    fsm = ready()
    [send] = sends(move(fsm, "g1", 0.0))
    stand(fsm, 1.0, STUCK)
    assert fsm.nav_result(send.seq, "canceled", 2.1, STUCK, obstacle) == []
    assert fsm.state() == RECOVERING


def test_the_obstacle_watch_rests_while_backing_up():
    fsm = ready()
    move(fsm, "g1", 0.0)
    stand(fsm, 1.0, STUCK)
    for t in (2.5, 3.0, 4.0):
        fsm.heartbeat(t)
        assert fsm.tick(t, STUCK, STILL, obstacle) == []
    assert fsm.state() == RECOVERING


def test_a_lost_backup_result_does_not_hold_the_recovery():
    """BackUp has a 5 s time allowance. Without a result 2 s after that, stop it and go on."""
    fsm = ready()
    move(fsm, "g1", 0.0)
    stand(fsm, 1.0, STUCK)  # BackUp sent at 2.05
    fsm.heartbeat(9.0)
    assert fsm.tick(9.0, BEHIND, STILL, obstacle) == []
    fsm.heartbeat(9.1)
    cmds = fsm.tick(9.1, BEHIND, STILL, obstacle)
    assert kinds(cmds) == ["CancelBackUp", "ClearCostmap", "SendPath"]


def test_a_failed_backup_still_tries_the_path_again():
    fsm = ready()
    move(fsm, "g1", 0.0)
    [b] = backups(stand(fsm, 1.0, STUCK))
    cmds = fsm.backup_result(b.seq, "aborted", 2.5, STUCK)  # something behind
    assert kinds(cmds) == ["ClearCostmap", "SendPath"]
    assert fsm.state() == RECOVERING


def test_stale_backup_results_are_ignored():
    fsm = ready()
    move(fsm, "g1", 0.0)
    [b] = backups(stand(fsm, 1.0, STUCK))
    assert fsm.backup_result(b.seq + 100, "succeeded", 2.5, BEHIND) == []
    fsm.backup_result(b.seq, "succeeded", 2.6, BEHIND)
    assert fsm.backup_result(b.seq, "succeeded", 2.7, BEHIND) == []


def test_cancel_during_recovery_stops_the_backup_and_reports_idle():
    fsm = ready()
    move(fsm, "g1", 0.0)
    stand(fsm, 1.0, STUCK)
    assert fsm.cancel("g1", 2.5) == [Finish("g1", False, CANCELED), CancelBackUp()]
    fsm.tick(2.6, STUCK, STILL, obstacle)
    assert fsm.state() == IDLE


def test_a_new_goal_during_recovery_stops_the_backup_and_keeps_the_count():
    fsm = ready()
    move(fsm, "g1", 0.0)
    stand(fsm, 1.0, STUCK)
    cmds = move(fsm, "g2", 2.5, pose=STUCK)
    assert kinds(cmds) == ["Finish", "CancelBackUp", "SendPath"]
    assert fsm.state() == MOVING
    [b2] = backups(stand(fsm, 3.0, STUCK))  # the second attempt
    fsm.backup_result(b2.seq, "succeeded", 5.0, BEHIND)
    assert backups(stand(fsm, 6.0, STUCK)) == []
    assert fsm.state() == BLOCKED


def test_attempts_start_again_once_the_robot_drives_on():
    fsm = ready()
    move(fsm, "g1", 0.0)
    [b] = backups(stand(fsm, 1.0, STUCK))
    fsm.backup_result(b.seq, "succeeded", 3.0, BEHIND)
    later = (*along(P4, P5, 0.35), EAST)  # 0.38 m from where the attempt started
    for t, pose in ((3.5, (0.35, 1.93, NORTH)), (4.0, (*P4, NORTH)), (4.5, later)):
        fsm.heartbeat(t)
        fsm.tick(t, pose, DRIVING, no_obstacle)
    assert fsm.state() == MOVING
    [b1] = backups(stand(fsm, 5.0, later))
    fsm.backup_result(b1.seq, "succeeded", 7.0, (*along(P4, P5, 0.23), EAST))
    assert len(backups(stand(fsm, 8.0, later))) == 1  # two attempts again


def test_a_successful_goal_resets_the_attempts():
    """P4 is 0.08 m and the second stuck point 0.24 m from where the first attempt started, so
    only the success can reset the count."""
    fsm = ready()
    move(fsm, "g1", 0.0, path=(P3, P4), target=P4, target_id="P-4")
    [b] = backups(stand(fsm, 1.0, STUCK))
    [again] = sends(fsm.backup_result(b.seq, "succeeded", 3.0, BEHIND))
    [done, _stop] = fsm.nav_result(again.seq, "succeeded", 6.0, (*P4, NORTH), no_obstacle)
    assert done == Finish("g1", True, "")
    south = -NORTH
    move(fsm, "g2", 7.0, path=(P4, P3), target=P3, pose=(*P4, south), target_id="P-3")
    stuck = (0.35, 1.63, south)  # 0.32 m from P4
    [b1] = backups(stand(fsm, 8.0, stuck))
    fsm.backup_result(b1.seq, "succeeded", 10.0, (0.35, 1.72, south))
    assert len(backups(stand(fsm, 11.0, stuck))) == 1  # two attempts again


def test_no_progress_for_ten_seconds_counts_as_another_attempt():
    fsm = ready()
    move(fsm, "g1", 0.0)
    [b1] = backups(stand(fsm, 1.0, STUCK))
    fsm.backup_result(b1.seq, "succeeded", 3.0, BEHIND)  # path sent again at 3.0
    creeping = (0.35, 1.82, NORTH)  # 0.27 m from P3: room for a 0.04 m back-up
    for t in (5.0, 8.0, 12.9):
        fsm.heartbeat(t)
        assert fsm.tick(t, creeping, DRIVING, no_obstacle) == []
    fsm.heartbeat(13.0)
    cmds = fsm.tick(13.0, creeping, DRIVING, no_obstacle)
    assert kinds(cmds) == ["CancelNav", "BackUp"]
    [b2] = backups(cmds)
    fsm.backup_result(b2.seq, "succeeded", 15.0, BEHIND)  # sent again at 15.0
    fsm.heartbeat(25.0)
    assert fsm.tick(25.0, creeping, DRIVING, no_obstacle) == [
        Finish("g1", False, NAV_ABORTED), CancelNav()]


def test_heartbeat_loss_during_recovery_stops_everything():
    fsm = ready()
    move(fsm, "g1", 0.0)
    stand(fsm, 1.0, STUCK)  # last heartbeat at 2.05
    assert fsm.tick(3.6, BEHIND, STILL, obstacle) == [
        Finish("g1", False, HEARTBEAT_LOST), CancelBackUp()]
    assert fsm.state() == IDLE


def test_losing_the_pose_during_recovery_stops_everything():
    fsm = ready()
    move(fsm, "g1", 0.0)
    stand(fsm, 1.0, STUCK)
    fsm.heartbeat(2.5)
    assert fsm.tick(2.5, None, STILL, obstacle) == [
        Finish("g1", False, NAV_ABORTED), CancelBackUp()]


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


# ---- 버전 2 4절 horizon 최소판(MoveTo.final_goal, 계약 0.8-draft 제안) ----------------------


def limits(cmds):
    return [c.mps for c in cmds if isinstance(c, LimitSpeed)]


def near_p5(back_m):
    """P4에서 P5로 가는 선 위, 끝에서 back_m 앞."""
    d = math.dist(P4, P5)
    k = (d - back_m) / d
    return (P4[0] + (P5[0] - P4[0]) * k, P4[1] + (P5[1] - P4[1]) * k, 0.2)


def can_stop(left, p=FsmParams()):
    """What can still stop within left (m) after the limit takes horizon_latency_s to act."""
    margin = p.horizon_cap_mps * p.horizon_latency_s
    return math.sqrt(2 * p.horizon_decel_mps2 * max(0.0, left - margin))


def test_a_goal_at_a_lock_boundary_limits_the_speed_to_what_can_stop_at_its_end():
    fsm = ready()
    fsm.move("g1", 1.0, "P-5", (*P5, 0.2), [P3, P4, P5], (*P3, NORTH), final_goal=False)
    assert limits(fsm.tick(1.1, (*P3, NORTH), DRIVING, no_obstacle)) == [], "멀면 제한이 없다"
    # 0.25 m/s x 0.3 s = 0.075 m of the distance left goes by before the limit acts
    [v] = limits(fsm.tick(1.2, near_p5(0.15), DRIVING, no_obstacle))
    assert v == pytest.approx(0.19)  # sqrt(2 x 0.25 x 0.075) = 0.194, floored
    [v] = limits(fsm.tick(1.3, near_p5(0.10), DRIVING, no_obstacle))
    assert v == pytest.approx(0.11)  # sqrt(2 x 0.25 x 0.025) = 0.112, floored
    assert limits(fsm.tick(1.4, near_p5(0.10), DRIVING, no_obstacle)) == [], "같으면 다시 안 보낸다"
    [v] = limits(fsm.tick(1.5, near_p5(0.07), DRIVING, no_obstacle))
    assert v == pytest.approx(0.05), "아래 한계: the latency margin is used up"


def test_without_latency_the_limit_is_what_can_stop_at_the_end():
    fsm = ready(horizon_latency_s=0.0)
    fsm.move("g1", 1.0, "P-5", (*P5, 0.2), [P3, P4, P5], (*P3, NORTH), final_goal=False)
    [v] = limits(fsm.tick(1.2, near_p5(0.10), DRIVING, no_obstacle))
    assert v == pytest.approx(math.sqrt(2 * 0.25 * 0.10), abs=0.01)
    [v] = limits(fsm.tick(1.3, near_p5(0.02), DRIVING, no_obstacle))
    assert v == pytest.approx(math.sqrt(2 * 0.25 * 0.02), abs=0.01)


def test_a_final_goal_keeps_no_limit_for_the_precise_approach():
    fsm = ready()
    fsm.move("g1", 1.0, "P-5", (*P5, 0.2), [P3, P4, P5], (*P3, NORTH))
    assert limits(fsm.tick(1.2, near_p5(0.05), DRIVING, no_obstacle)) == []


def test_the_limit_is_lifted_when_the_goal_becomes_final_or_ends():
    fsm = ready()
    fsm.move("g1", 1.0, "P-5", (*P5, 0.2), [P3, P4, P5], (*P3, NORTH), final_goal=False)
    assert limits(fsm.tick(1.1, near_p5(0.05), DRIVING, no_obstacle))
    fsm.move("g2", 1.2, "P-6", (*P6, 0.0), [P3, P4, P5, P6], near_p5(0.05))
    assert limits(fsm.tick(1.3, near_p5(0.04), DRIVING, no_obstacle)) == [0.0]
    fsm.move("g3", 1.4, "P-5", (*P5, 0.2), [P3, P4, P5], near_p5(0.04), final_goal=False)
    assert limits(fsm.tick(1.5, near_p5(0.04), DRIVING, no_obstacle))
    fsm.cancel("g3", 1.6)
    assert limits(fsm.tick(1.7, near_p5(0.04), STILL, no_obstacle)) == [0.0]


def test_a_fresh_fsm_lifts_any_limit_left_from_before_a_restart():
    """horizon 1 Codex 검수 2: 재시작 전 FSM이 건 제한이 controller에 남아 있을 수 있다."""
    fsm = PinkyFsm(FsmParams())
    assert limits(fsm.tick(0.0, (*P3, NORTH), STILL, no_obstacle)) == [0.0]
    assert limits(fsm.tick(0.1, (*P3, NORTH), STILL, no_obstacle)) == []


@pytest.mark.parametrize("back_m", [0.15, 0.12, 0.09, 0.07, 0.03])
def test_the_limit_never_rounds_above_what_can_stop(back_m):
    fsm = ready()
    fsm.move("g1", 1.0, "P-5", (*P5, 0.2), [P3, P4, P5], (*P3, NORTH), final_goal=False)
    [v] = limits(fsm.tick(1.2, near_p5(back_m), DRIVING, no_obstacle))
    left = fsm.distance_remaining(near_p5(back_m))
    assert v <= max(FsmParams().horizon_min_mps, can_stop(left)) + 1e-12


def test_the_limit_never_asks_more_than_the_nav2_desired_speed():
    """RPP takes an absolute speed limit as its desired_linear_vel without comparing it to the
    configured one, so a limit of 0.26-0.29 m/s (0.13-0.18 m before the end) would speed up."""
    fsm = ready()
    fsm.move("g1", 1.0, "P-5", (*P5, 0.2), [P3, P4, P5], (*P3, NORTH), final_goal=False)
    asked = []
    for k in range(40):  # from 0.20 m before the end to the end, 5 mm apart
        asked += limits(fsm.tick(1.1 + 0.01 * k, near_p5(0.20 - 0.005 * k), DRIVING, no_obstacle))
    positive = [v for v in asked if v > 0.0]
    assert positive and max(positive) <= nav2_desired_mps()


def test_the_speed_cap_is_not_above_the_nav2_desired_speed():
    assert FsmParams().horizon_cap_mps <= nav2_desired_mps()


def nav2_desired_mps() -> float:
    import yaml
    from conftest import REPO
    nav2 = yaml.safe_load((REPO / "pinky" / "pinky_nav_config" / "params" / "nav2_params.yaml")
                          .read_text(encoding="utf-8"))
    return nav2["controller_server"]["ros__parameters"]["FollowPath"]["desired_linear_vel"]


# ---- pass-through controller: every FollowPath goal says whether its end is final -----------


def test_every_path_of_a_final_goal_is_marked_final():
    fsm = ready()
    [send] = sends(move(fsm, "g1", 1.0))
    assert send.final_goal is True


def test_every_path_of_a_goal_at_a_lock_boundary_is_marked_not_final():
    """The node picks the pass-through controller from this mark, also for the periodic
    resends near the end (R13)."""
    fsm = ready()
    [send] = sends(fsm.move("g1", 1.0, "P-5", (*P5, 0.2), [P3, P4, P5], (*P3, NORTH),
                            final_goal=False))
    assert send.final_goal is False
    resent = sends(fsm.tick(1.6, near_p5(0.10), DRIVING, no_obstacle))
    assert resent and all(s.final_goal is False for s in resent)
    [last] = sends(fsm.move("g2", 1.7, "P-5", (*P5, 0.2), [P3, P4, P5], near_p5(0.10)))
    assert last.final_goal is True, "the next goal that ends the route is final again"


def test_the_resend_after_a_self_recovery_keeps_the_mark():
    fsm = ready()
    [send] = sends(fsm.move("g1", 0.0, "P-5", (*P5, 0.2), [P3, P4, P5], (*P3, NORTH),
                            final_goal=False))
    fsm.heartbeat(2.0)
    fsm.tick(2.0, STUCK, STILL, no_obstacle)
    [b] = backups(fsm.nav_result(send.seq, "aborted", 2.1, STUCK, no_obstacle))
    [again] = sends(fsm.backup_result(b.seq, "succeeded", 3.0, BEHIND))
    assert again.final_goal is False
