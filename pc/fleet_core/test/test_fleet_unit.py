"""FleetCore 규칙을 손으로 쓴 자세와 결과로 하나씩 검증한다."""
import math
from dataclasses import replace

import pytest
from core_helpers import cmds, codes, make_core, place, serve
from fleet_core.common.commands import AssignmentView, OrderRecord, ZoneView
from fleet_core.common.params import load_params
from grid_map import grid_site
from legacy_core import FleetCore

WORK_DOCK = (5, 3)
OUT_DOCK = (9, 3)
O1 = OrderRecord("O-001", {"A": 1, "C": 1})
O2 = OrderRecord("O-002", {"B": 1})
O3 = OrderRecord("O-003", {"D": 2})


def started(robots=None, orders=(), params=None):
    """로봇을 지정 위치에 놓고 주문판을 게시한 상태."""
    robots = robots or {"r1": (3, 4), "r2": (3, 5), "r3": (3, 6)}
    core, now = make_core(list(robots), params)
    for name, cell in robots.items():
        place(core, name, cell)
    core.on_order_board("RUN", list(orders))
    return core, now


def view(core, robot):
    return core.robot_view(robot)


# ---- placement ------------------------------------------------------------------------------


def test_first_state_places_the_robot_in_the_zone_of_its_cell():
    core, _ = make_core(["r1"])
    place(core, "r1", (3, 5))
    assert core.zones.holder("P-5") == "r1"
    v = view(core, "r1")
    assert v["waypoint"] == (3, 5)
    assert v["state"] == "PARKED"
    assert v["held"] == ["P-5"]
    assert v["basket"] == "EMPTY", "every robot starts with an empty basket"


def test_placement_conflict_is_an_error_event():
    core, _ = make_core(["r1", "r2"])
    place(core, "r1", (3, 5))
    place(core, "r2", (3, 5))
    out = core.tick()
    assert "PLACEMENT_CONFLICT" in codes(out)
    assert core.zones.holder("P-5") == "r1"
    assert view(core, "r2")["state"] == "ERROR"


def test_no_call_or_move_before_the_first_board():
    core, _ = make_core(["r1"])
    place(core, "r1", (9, 5))
    assert not cmds(core.tick(), "MoveCmd")
    assert view(core, "r1")["called"] is False
    assert core.zones.held_by("r1") == ["Q-OUT-2"]


# ---- zones and goals ------------------------------------------------------------------------


def test_move_goes_only_as_far_as_the_granted_zones_allow():
    core, _ = started({"r1": (9, 5)})
    moves = cmds(core.tick(), "MoveCmd")
    assert len(moves) == 1
    # Q-OUT-2와 Q-OUT-1은 점유, Z-OUT은 아님: 허용되는 가장 먼 칸은 (9,4).
    assert moves[0].target_id == core.site.waypoint_id((9, 4))
    assert (moves[0].x, moves[0].y) == core.site.waypoint_pose((9, 4))
    assert core.zones.held_by("r1") == ["Q-OUT-2", "Q-OUT-1"]


def test_robot_waits_at_the_boundary_when_the_next_zone_is_taken():
    core, _ = started({"r1": (9, 5), "r2": (9, 4)})
    core.zones.request("x", "Z-OUT")  # r2의 출발을 막는다
    first = core.tick()
    assert [m.robot for m in cmds(first, "MoveCmd")] == [], "r1 waits for Q-OUT-1, r2 for Z-OUT"
    assert core.zones.queue("Q-OUT-1") == ("r1",)
    assert core.zones.queue("Z-OUT") == ("r2",)


POINT = replace(load_params(), lane_goal_mode="point")


def test_target_stops_before_a_corner():
    """point 모드: 직선 주행은 한 구역 안에서도 모서리를 가로지르면 안 된다."""
    core, _ = started({"r1": (9, 2)}, params=POINT)
    move = cmds(core.tick(), "MoveCmd")[0]
    # (9,2), (9,1), (9,0), (8,0)이 모두 허가됐지만 경로는 (9,0)에서 꺾인다.
    assert move.target_id == core.site.waypoint_id((9, 0))
    assert move.yaw == math.atan2(-1, 0)


def test_next_leg_starts_only_after_the_corner_was_reached():
    """point 모드: 모서리 근처에 서는 것으로는 부족하다. 실행 중 목표가 먼저 끝나야 한다."""
    core, _ = started({"r1": (9, 2)}, params=POINT)
    move = cmds(core.tick(), "MoveCmd")[0]
    place(core, "r1", (9, 0))
    assert not cmds(core.tick(), "MoveCmd"), "pose alone does not turn the robot"
    core.on_move_result("r1", move.goal_id, True, "")
    place(core, "r1", (9, 0))
    assert cmds(core.tick(), "MoveCmd")[0].target_id == core.site.waypoint_id((2, 0))


def test_path_mode_goes_round_the_corner_along_the_lanes():
    """path 모드: 허가된 데까지 꺾어 가되 선은 모서리를 지난다."""
    core, _ = started({"r1": (9, 2)})
    move = cmds(core.tick(), "MoveCmd")[0]
    corner = core.site.waypoint_pose((9, 0))
    assert move.target_id != core.site.waypoint_id((9, 0))
    assert corner in [p[:2] for p in move.path]


def test_new_grant_supersedes_the_running_goal_and_old_results_are_ignored():
    core, _ = started({"r1": (8, 0)})
    core.zones.force_acquire("x", "Z-R2")
    first = cmds(core.tick(), "MoveCmd")[0]
    assert first.target_id == core.site.waypoint_id((6, 0)), "Z-R2 is taken, Z-R1 ends at (6,0)"
    core.zones.release("x", "Z-R2")
    place(core, "r1", (7, 0))
    second = cmds(core.tick(), "MoveCmd")[0]
    assert second.target_id == core.site.waypoint_id((2, 0))
    assert second.goal_id != first.goal_id

    core.on_move_result("r1", first.goal_id, False, "CANCELED")
    assert view(core, "r1")["goal_id"] == second.goal_id
    assert view(core, "r1")["attempts"] == 0, "a superseded goal is not retried"
    assert "GOAL_RETRY" not in codes(core.tick())


def test_stop_canceled_goal_is_not_retried():
    core, _ = started({"r1": (8, 0)})
    move = cmds(core.tick(), "MoveCmd")[0]
    core.on_order_board("STOP", [])
    out = core.tick()
    assert [c.goal_id for c in cmds(out, "CancelCmd")] == [move.goal_id]
    core.on_move_result("r1", move.goal_id, False, "CANCELED")
    assert view(core, "r1")["attempts"] == 0
    assert "GOAL_RETRY" not in codes(core.tick())


def test_zone_is_released_only_by_pose_never_by_result_or_time():
    core, now = started({"r1": (9, 5)})
    move = cmds(core.tick(), "MoveCmd")[0]
    core.on_move_result("r1", move.goal_id, True, "")
    now[0] = 2.0
    place(core, "r1", (9, 5))
    core.tick()
    assert "Q-OUT-2" in core.zones.held_by("r1"), "goal success alone frees nothing"

    now[0] += core.params.move_retry_wait_s  # 자세가 따라오지 않아 이동을 다시 보낸다
    place(core, "r1", (9, 5))
    core.tick()
    assert "Q-OUT-2" in core.zones.held_by("r1"), "nor does time"

    x, y = core.site.waypoint_pose((9, 5))
    distance = core.params.release_distance_m
    # (9,4)로 가는 레인을 따라 y가 줄어든다. 해제 거리 바로 앞은 아직 가깝다.
    for _ in range(core.params.release_confirm_samples):
        core.on_robot_state("r1", x, y - distance + 0.001, 0.0, "MOVING")
    core.tick()
    assert "Q-OUT-2" in core.zones.held_by("r1")
    outside = y - distance - 0.001
    for _ in range(core.params.release_confirm_samples):
        core.on_robot_state("r1", x, outside, 0.0, "MOVING")
    core.tick()
    assert "Q-OUT-2" not in core.zones.held_by("r1")


def test_release_needs_consecutive_reports_outside_the_margin():
    """잡음 자세 하나로는 부족하고, 안쪽으로 돌아온 보고는 횟수를 다시 센다."""
    core, now = started({"r1": (9, 5)})
    move = cmds(core.tick(), "MoveCmd")[0]
    core.on_move_result("r1", move.goal_id, True, "")
    now[0] = 2.0
    x, y = core.site.waypoint_pose((9, 5))
    far = y - (core.params.release_distance_m + 0.01)
    needed = core.params.release_confirm_samples
    assert needed >= 2
    for _ in range(needed - 1):
        core.on_robot_state("r1", x, far, 0.0, "MOVING")
    core.tick()
    assert "Q-OUT-2" in core.zones.held_by("r1")
    core.on_robot_state("r1", x, y, 0.0, "MOVING")
    core.on_robot_state("r1", x, far, 0.0, "MOVING")
    core.tick()
    assert "Q-OUT-2" in core.zones.held_by("r1"), "the count restarts after a report inside"
    for _ in range(needed - 1):
        core.on_robot_state("r1", x, far, 0.0, "MOVING")
    core.tick()
    assert "Q-OUT-2" not in core.zones.held_by("r1")


def test_zone_is_kept_while_it_is_on_the_upcoming_path():
    """긴 구역은 로봇이 얼마나 진행했든 다가올 경로에 있는 동안 유지된다."""
    core, _ = started({"r1": (9, 0)})
    core.tick()
    place(core, "r1", (7, 0))
    core.tick()
    assert "Z-R1" in core.zones.held_by("r1")


def test_holds_at_most_two_zones_and_requests_the_third_only_after_a_release():
    core, _ = started({"r1": (9, 5)})
    core.tick()
    place(core, "r1", (9, 4))
    core.tick()
    assert len(core.zones.held_by("r1")) == 2


def test_retry_keeps_zones_and_resends_the_same_target():
    core, now = started({"r1": (9, 5)})
    goal = cmds(core.tick(), "MoveCmd")[0]
    held = core.zones.held_by("r1")
    core.on_move_result("r1", goal.goal_id, False, "NAV_ABORTED")
    now[0] += core.params.move_retry_wait_s
    place(core, "r1", (9, 5))
    retry = cmds(core.tick(), "MoveCmd")[0]
    assert retry.target_id == goal.target_id
    assert retry.goal_id != goal.goal_id
    assert core.zones.held_by("r1") == held


def test_rejected_goal_counts_as_a_failed_attempt():
    core, now = started({"r1": (9, 5)})
    for _ in range(core.params.move_retry_limit + 1):
        goal = cmds(core.tick(), "MoveCmd")[0]
        core.on_goal_rejected("r1", goal.goal_id)
        now[0] += core.params.move_retry_wait_s
        place(core, "r1", (9, 5))
    assert view(core, "r1")["state"] == "ERROR"
    assert "GOAL_FAILED" in codes(core.tick())


def test_long_zone_wait_warns_once_and_names_the_holder():
    """멈춘 로봇 뒤에 줄 선 로봇이 말없이 기다려선 안 된다."""
    core, now = started({"r1": (9, 5), "r2": (9, 4)})
    core._missions["r2"].fault = "ERROR"  # 화이트박스: r2는 서 있고 구역을 유지
    core.tick()
    assert core.zones.holder("Q-OUT-1") == "r2"
    warnings = []
    for _ in range(3):
        now[0] += core.params.zone_wait_warn_s / 2 + 0.1
        place(core, "r1", (9, 5))
        place(core, "r2", (9, 4))
        warnings += [e for e in cmds(core.tick(), "EventOut") if e.code == "ZONE_WAIT_LONG"]
    assert len(warnings) == 1, "one warning per wait"
    assert warnings[0].robot == "r1"
    assert "r2" in warnings[0].detail and "Q-OUT-1" in warnings[0].detail


# ---- comm loss ------------------------------------------------------------------------------


def test_comm_timeout_cancels_queue_place_but_keeps_held_zones():
    core, now = started({"r1": (9, 4), "r2": (9, 5)})
    core.zones.force_acquire("x", "Z-OUT")
    core.tick()
    assert core.zones.queue("Z-OUT") == ("r1",)
    assert core.zones.queue("Q-OUT-1") == ("r2",)
    now[0] = core.params.comm_timeout_s + 0.1
    place(core, "r2", (9, 5))
    out = core.tick()
    assert "COMM_LOST" in codes(out)
    assert view(core, "r1")["state"] == "COMM_LOST"
    assert core.zones.held_by("r1") == ["Q-OUT-1"]
    assert core.zones.queue("Z-OUT") == ()
    assert core.zones.queue("Q-OUT-1") == ("r2",), "the follower keeps waiting behind it"


def test_recovery_releases_only_zones_it_is_outside_of():
    """Q-OUT-1을 예약했으나 Q-OUT-2를 벗어난 적 없으면 예약을 돌려준다."""
    core, now = started({"r1": (9, 5)})
    goal = cmds(core.tick(), "MoveCmd")[0]
    assert core.zones.held_by("r1") == ["Q-OUT-2", "Q-OUT-1"]
    now[0] = core.params.comm_timeout_s + 0.1
    core.tick()
    assert view(core, "r1")["state"] == "COMM_LOST"
    now[0] += 1.0
    place(core, "r1", (9, 5))
    out = core.tick()
    assert [c.goal_id for c in cmds(out, "CancelCmd")] == [goal.goal_id]
    assert not cmds(out, "MoveCmd")
    assert view(core, "r1")["state"] == "RECOVERING"
    assert core.zones.held_by("r1") == ["Q-OUT-2", "Q-OUT-1"], "nothing changes during the check"
    now[0] += core.params.recover_check_s
    place(core, "r1", (9, 5))
    core.tick()
    assert view(core, "r1")["state"] != "RECOVERING"
    assert "Q-OUT-2" in core.zones.held_by("r1")


def test_recovery_keeps_the_zone_it_is_inside_and_lets_go_of_the_old_one():
    core, now = started({"r1": (9, 5)})
    core.tick()
    now[0] = core.params.comm_timeout_s + 0.1
    core.tick()
    now[0] += 1.0
    place(core, "r1", (9, 4))
    core.tick()
    now[0] += core.params.recover_check_s
    place(core, "r1", (9, 4))
    core.tick()
    held = core.zones.held_by("r1")
    assert "Q-OUT-1" in held
    assert "Q-OUT-2" not in held
    assert view(core, "r1")["waypoint"] == (9, 4)


def test_stop_during_comm_loss_does_not_lose_the_goal_to_cancel():
    """무응답 로봇에 보낸 취소는 닿지 않을 수 있어 복구 때 다시 보낸다."""
    core, now = started({"r1": (8, 0)})
    goal = cmds(core.tick(), "MoveCmd")[0]
    now[0] = core.params.comm_timeout_s + 0.1
    core.tick()
    core.on_order_board("STOP", [])
    assert not cmds(core.tick(), "CancelCmd")
    now[0] += 1.0
    place(core, "r1", (7, 0))
    assert [c.goal_id for c in cmds(core.tick(), "CancelCmd")] == [goal.goal_id]


def test_recovery_finds_the_cell_under_the_pose_on_the_remaining_path():
    """무응답 중 로봇이 계속 달렸으므로 칸은 마지막 보고가 아닌 자세에서 구한다."""
    core, now = started({"r1": (8, 0)})
    core.tick()
    now[0] = core.params.comm_timeout_s + 0.1
    core.tick()
    now[0] += 1.0
    place(core, "r1", (7, 0))
    core.tick()
    now[0] += core.params.recover_check_s
    place(core, "r1", (7, 0))
    core.tick()
    v = view(core, "r1")
    assert v["waypoint"] == (7, 0)
    assert "Z-R1" in v["held"]


def test_recovery_does_not_hand_over_the_zone_it_is_inside():
    """구역을 기다리는 뒤차에게 넘기면 안 된다. 로봇이 물리적으로 그 안에 있다."""
    core, now = started({"r1": (9, 4), "r2": (9, 5)})
    core.zones.force_acquire("x", "Z-OUT")
    core.tick()
    assert core.zones.queue("Q-OUT-1") == ("r2",)
    now[0] = core.params.comm_timeout_s + 0.1
    place(core, "r2", (9, 5))
    core.tick()
    now[0] += 1.0
    place(core, "r1", (9, 4))
    place(core, "r2", (9, 5))
    core.tick()
    now[0] += core.params.recover_check_s
    place(core, "r1", (9, 4))
    place(core, "r2", (9, 5))
    out = core.tick()
    assert core.zones.holder("Q-OUT-1") == "r1"
    assert core.zones.queue("Q-OUT-1") == ("r2",)
    assert "RECOVERY_CONFLICT" not in codes(out)
    assert view(core, "r1")["state"] != "ERROR"


def test_comm_lost_robot_loses_its_call_to_another():
    core, now = started({"r1": (2, 2), "r2": (9, 2)}, [O1])
    core.tick()
    assert view(core, "r1")["called"] is True
    assert view(core, "r2")["called"] is False
    now[0] = core.params.comm_timeout_s + 0.1
    place(core, "r2", view(core, "r2")["waypoint"])
    core.tick()
    assert view(core, "r1")["called"] is False
    assert view(core, "r2")["called"] is True


def test_robot_called_behind_a_waiting_robot_stays_out_of_it_after_recovery():
    """앞 로봇이 빠졌다 돌아와도 뒤 로봇은 그것을 기다리면 안 된다."""
    core, now = started({"r1": (3, 4), "r2": (3, 5)}, [O1])
    core.tick()
    assert view(core, "r1")["state"] == "TO_WORK"
    now[0] = core.params.comm_timeout_s + 0.1
    place(core, "r2", (3, 5))
    core.tick()
    assert view(core, "r1")["state"] == "COMM_LOST"
    assert view(core, "r2")["called"] is False, "r1 still stands ahead of it"
    now[0] += 1.0
    place(core, "r1", (3, 4))
    place(core, "r2", (3, 5))
    core.tick()
    now[0] += core.params.recover_check_s
    place(core, "r1", (3, 4))
    place(core, "r2", (3, 5))
    core.tick()
    assert view(core, "r1")["called"] is True
    assert view(core, "r2")["called"] is False


def test_recovery_during_filling_stays_at_the_dock_and_keeps_the_order():
    core, now, fill = filling()
    now[0] += core.params.comm_timeout_s + 0.1
    core.tick()
    assert view(core, "r1")["state"] == "COMM_LOST"
    assert view(core, "r1")["order"] == "O-001"
    core.on_fill_result(fill.goal_id, True, dict(fill.items), {})
    now[0] += 1.0
    place(core, "r1", WORK_DOCK)
    core.tick()
    now[0] += core.params.recover_check_s
    place(core, "r1", WORK_DOCK)
    core.tick()
    assert view(core, "r1")["state"] == "TO_OUTBOUND"


def test_recovery_with_a_full_basket_heads_to_outbound():
    core, now = started({"r1": (6, 5)})
    core._missions["r1"].basket = "FULL"  # 화이트박스: 무응답이 됐을 때 나가는 중
    core.tick()
    now[0] = core.params.comm_timeout_s + 0.1
    core.tick()
    now[0] += 1.0
    place(core, "r1", (6, 5))
    core.tick()
    now[0] += core.params.recover_check_s
    place(core, "r1", (6, 5))
    core.tick()
    assert view(core, "r1")["state"] == "TO_OUTBOUND"
    assert view(core, "r1")["destination"] == "OUTBOUND"


# ---- calling and commit ---------------------------------------------------------------------


def test_call_count_follows_the_unbound_orders():
    core, _ = started(None, [O1])
    core.tick()
    assert [view(core, r)["called"] for r in ("r1", "r2", "r3")] == [True, False, False]
    core.on_order_board("RUN", [O1, O2])
    core.tick()
    assert [view(core, r)["called"] for r in ("r1", "r2", "r3")] == [True, True, False]


def test_no_order_is_bound_before_the_dock():
    core, _ = started({"r1": (4, 3)}, [O1])
    core.tick()
    assert view(core, "r1")["called"] is True
    assert view(core, "r1")["order"] is None
    assert not [a for a in core.assignments_view() if a.order_id]


def test_first_docked_binds_the_oldest_order_whoever_was_called_first():
    """r1이 먼저 호출됐지만 멀고, r2는 나중에 호출돼 먼저 도킹한다."""
    core, _ = started({"r1": (8, 0), "r2": (3, 4)})
    core._missions["r1"].called = True  # 화이트박스: r2가 비기 전에 호출됨
    core.on_order_board("RUN", [O1])
    core.tick()
    assert view(core, "r1")["called"] is True and view(core, "r2")["called"] is False
    core.on_order_board("RUN", [O1, O2])
    out = serve(core, ["r1", "r2"], lambda: view(core, "r2")["order"] is not None)
    assert view(core, "r2")["order"] == "O-001"
    out += core.tick()
    assert [(e.detail, e.robot) for e in cmds(out, "EventOut") if e.code == "ORDER_BOUND"] == [
        ("O-001", "r2")]
    assert view(core, "r1")["order"] is None


def test_dock_with_no_order_left_binds_nothing_and_leaves():
    core, _ = started({"r1": (4, 3)}, [O1])
    move = cmds(core.tick(), "MoveCmd")[0]
    place(core, "r1", WORK_DOCK)
    core.on_move_result("r1", move.goal_id, True, "")
    dock = cmds(core.tick(), "DockCmd")[0]
    core.on_order_board("RUN", [])
    core.on_dock_result("r1", dock.goal_id, True)
    assert view(core, "r1")["order"] is None
    place(core, "r1", WORK_DOCK)
    out = core.tick()
    assert "ORDER_BOUND" not in codes(out)
    assert view(core, "r1")["state"] == "TO_PARK"


def test_destination_can_change_until_the_commit_zone_is_held():
    core, _ = started({"r1": (2, 3)}, [O1])
    core.zones.force_acquire("x", "Q-WORK-2")
    core.tick()
    assert core.zones.queue("Q-WORK-2") == ("r1",)
    assert view(core, "r1")["state"] == "TO_WORK"
    core.on_order_board("RUN", [])
    place(core, "r1", (2, 3))
    core.tick()
    v = view(core, "r1")
    assert v["state"] == "TO_PARK"
    assert v["path"][1] == (2, 4), "turned off into the waiting cells"
    assert core.zones.queue("Q-WORK-2") == (), "it gave up its place in the line"


def test_uncalled_past_the_commit_zone_laps_without_docking_and_ends_up_waiting():
    core, _ = started({"r1": (2, 3)}, [O1])
    core.tick()
    assert "Q-WORK-2" in core.zones.held_by("r1")
    core.on_order_board("RUN", [])
    place(core, "r1", (2, 3))
    core.tick()
    v = view(core, "r1")
    assert (v["state"], v["destination"]) == ("TO_WORK", "WORK"), "it keeps going"
    route = set(v["path"])

    def parked():
        route.update(view(core, "r1")["path"])
        return view(core, "r1")["state"] == "PARKED"

    out = serve(core, ["r1"], parked)
    assert not cmds(out, "DockCmd"), "nothing to do at either dock"
    assert WORK_DOCK in route and OUT_DOCK in route
    assert view(core, "r1")["waypoint"] == (3, 4), "the deepest waiting cell"


def test_called_again_past_the_commit_zone_docks_after_all():
    core, _ = started({"r1": (2, 3)}, [O1])
    core.tick()
    core.on_order_board("RUN", [])
    place(core, "r1", (2, 3))
    core.tick()
    core.on_order_board("RUN", [O2])
    serve(core, ["r1"], lambda: view(core, "r1")["order"] == "O-002")


def test_call_turns_a_robot_on_its_way_to_waiting_back_to_work():
    core, _ = started({"r1": (2, 1)})
    core.tick()
    assert view(core, "r1")["state"] == "TO_PARK"
    core.on_order_board("RUN", [O1])
    place(core, "r1", (2, 1))
    core.tick()
    assert view(core, "r1")["state"] == "TO_WORK"
    assert view(core, "r1")["destination"] == "WORK"
    assert view(core, "r1")["path"][-1] == WORK_DOCK


def test_retry_target_survives_a_new_destination():
    """실패한 목표의 재시도는 그사이 경로가 바뀌어도 같은 칸으로 간다."""
    core, now = started({"r1": (2, 1)}, [O1])
    core.zones.force_acquire("x", "Q-WORK-2")
    first = cmds(core.tick(), "MoveCmd")[0]
    assert first.target_id == core.site.waypoint_id((2, 3))
    core.on_move_result("r1", first.goal_id, False, "NAV_ABORTED")
    core.on_order_board("RUN", [])
    now[0] += core.params.move_retry_wait_s
    place(core, "r1", (2, 1))
    assert cmds(core.tick(), "MoveCmd")[0].target_id == first.target_id
    assert view(core, "r1")["phase"] == "TO_PARK"


def test_a_move_awaiting_its_pose_keeps_its_target_when_the_path_changes():
    core, _ = started({"r1": (2, 1)}, [O1])
    core.zones.force_acquire("x", "Q-WORK-2")
    first = cmds(core.tick(), "MoveCmd")[0]
    core.on_move_result("r1", first.goal_id, True, "")  # 보고가 아직 따라오지 못함
    core.on_order_board("RUN", [])
    seen = codes(core.tick())
    place(core, "r1", (2, 3))
    seen += codes(core.tick())
    assert "GOAL_RETRY" not in seen
    assert view(core, "r1")["waypoint"] == (2, 3)
    assert view(core, "r1")["phase"] == "TO_PARK"


def test_robot_called_then_uncalled_in_a_waiting_cell_does_not_lap():
    """출발 전에 호출이 취소되면 한 바퀴 돌지 않고 그 자리에 머문다."""
    core, _ = started({"r1": (3, 4)}, [O1])
    core.zones.force_acquire("x", "Q-WORK-2")
    core.tick()
    assert view(core, "r1")["state"] == "TO_WORK"
    core.on_order_board("RUN", [])
    place(core, "r1", (3, 4))
    core.tick()
    assert view(core, "r1")["path"] == [(3, 4)]
    assert view(core, "r1")["phase"] in ("TO_PARK", "PARKED")


def test_parked_robot_with_no_work_stays_at_the_deepest_cell():
    core, _ = started({"r1": (3, 4)})
    assert not cmds(core.tick(), "MoveCmd")
    assert view(core, "r1")["state"] == "PARKED"


def test_parked_robot_advances_when_the_cell_ahead_frees_up():
    core, _ = started({"r1": (3, 4), "r2": (3, 5)})
    assert not cmds(core.tick(), "MoveCmd")
    core.confirm_clear("r1", ["P-6"])
    move = cmds(core.tick(), "MoveCmd")[0]
    assert (move.robot, move.target_id) == ("r2", core.site.waypoint_id((3, 4)))


def test_failed_waiting_robot_keeps_those_behind_uncalled():
    core, _ = started({"r1": (3, 4), "r2": (3, 5)})
    core._missions["r1"].fault = "ERROR"  # 화이트박스
    core.on_order_board("RUN", [O1])
    core.tick()
    assert view(core, "r2")["called"] is False


def test_cleared_waiting_robot_no_longer_blocks_those_behind():
    """운영자가 로봇을 치웠으면 어디에 있었든 길이 빈다."""
    core, _ = started({"r1": (3, 4), "r2": (3, 5)})
    core._missions["r1"].fault = "ERROR"  # 화이트박스
    core.confirm_clear("r1", ["P-6"])
    core.on_order_board("RUN", [O1])
    place(core, "r2", (3, 5))
    core.tick()
    assert view(core, "r2")["called"] is True


# ---- filling --------------------------------------------------------------------------------


def filling(orders=(O1,), params=None, robots=None):
    """WORK에 로봇 하나가 도킹했고 첫 FillCmd가 나갔다. 다른 로봇은 비켜서 기다린다."""
    robots = robots or {"r1": (4, 3)}
    core, now = started(robots, orders, params)
    out = serve(core, list(robots), lambda: view(core, "r1")["phase"] == "FILLING")
    fills = cmds(out, "FillCmd")
    if not fills:
        fills = cmds(core.tick(), "FillCmd")
    return core, now, fills[0] if fills else None


def test_fill_asks_for_the_order_and_names_robot_station_and_order():
    core, _, fill = filling()
    assert (fill.robot, fill.station_id, fill.order_id) == ("r1", "WORK", "O-001")
    assert fill.items == {"A": 1, "C": 1}
    assert view(core, "r1")["state"] == "FILLING"
    assert "ORDER_BOUND" not in codes(core.tick()), "bound once, at the dock"


def test_fill_success_sends_the_full_basket_to_outbound():
    core, _, fill = filling()
    core.on_fill_result(fill.goal_id, True, {"A": 1, "C": 1}, {})
    place(core, "r1", WORK_DOCK)
    core.tick()
    v = view(core, "r1")
    assert (v["state"], v["basket"], v["destination"], v["order"]) == (
        "TO_OUTBOUND", "FULL", "OUTBOUND", "O-001")


def test_fill_retry_sends_only_what_is_still_missing():
    core, now, fill = filling([OrderRecord("O-001", {"A": 2, "C": 1})])
    core.on_fill_result(fill.goal_id, False, {"A": 1}, {}, "GRIPPER")
    now[0] += core.params.arm_retry_wait_s
    place(core, "r1", WORK_DOCK)
    out = core.tick()
    assert "FILL_RETRY" in codes(out)
    retry = cmds(out, "FillCmd")[0]
    assert retry.items == {"A": 1, "C": 1}
    assert retry.goal_id != fill.goal_id
    assert view(core, "r1")["placed"] == {"A": 1}


def test_failure_that_placed_everything_needs_no_retry():
    core, _, fill = filling()
    core.on_fill_result(fill.goal_id, False, {"A": 1, "C": 1}, {}, "LOST_CONTACT_AFTERWARDS")
    place(core, "r1", WORK_DOCK)
    assert not cmds(core.tick(), "FillCmd")
    assert view(core, "r1")["state"] == "TO_OUTBOUND"


def test_fill_retries_run_out_into_error_with_the_order_still_bound():
    core, now, fill = filling()
    seen = []
    for attempt in range(core.params.move_retry_limit + 1):
        core.on_fill_result(fill.goal_id, False, {}, {}, "GRIPPER")
        now[0] += core.params.arm_retry_wait_s
        place(core, "r1", WORK_DOCK)
        out = core.tick()
        seen += codes(out)
        fills = cmds(out, "FillCmd")
        if attempt < core.params.move_retry_limit:
            fill = fills[0]
        else:
            assert not fills
    assert seen.count("FILL_RETRY") == core.params.move_retry_limit
    v = view(core, "r1")
    assert (v["state"], v["order"]) == ("ERROR", "O-001")


def test_fill_failed_event_is_an_error():
    core, now, fill = filling()
    core.on_fill_result(fill.goal_id, False, {}, {}, "GRIPPER")
    now[0] += core.params.arm_retry_wait_s
    place(core, "r1", WORK_DOCK)
    fill = cmds(core.tick(), "FillCmd")[0]
    core.on_fill_result(fill.goal_id, False, {}, {}, "GRIPPER")
    now[0] += core.params.arm_retry_wait_s
    place(core, "r1", WORK_DOCK)
    fill = cmds(core.tick(), "FillCmd")[0]
    core._out.clear()
    core.on_fill_result(fill.goal_id, False, {}, {}, "GRIPPER")
    errors = [e for e in cmds(core.tick(), "EventOut") if e.code == "FILL_FAILED"]
    assert len(errors) == 1 and errors[0].level == "ERROR" and errors[0].robot == "r1"


def test_canceled_fill_does_not_count_as_an_attempt():
    core, _, fill = filling()
    core.on_order_board("STOP", [O1])
    assert [c.robot for c in cmds(core.tick(), "CancelCmd")] == ["workcell"]
    core.on_fill_result(fill.goal_id, False, {"A": 1}, {}, "CANCELED")
    out = core.tick()
    assert "FILL_RETRY" not in codes(out)
    assert view(core, "r1")["attempts"] == 0
    assert view(core, "r1")["placed"] == {"A": 1}, "what it placed before stopping still counts"


def test_partial_order_ships_what_was_placed():
    core, _, fill = filling()
    core.on_fill_result(fill.goal_id, True, {"A": 1}, {"C": 1})
    place(core, "r1", WORK_DOCK)
    core.tick()
    assert view(core, "r1")["state"] == "TO_OUTBOUND"
    assert view(core, "r1")["basket"] == "FULL"


def test_success_that_leaves_items_unaccounted_counts_them_as_missing():
    core, _, fill = filling()
    core.on_fill_result(fill.goal_id, True, {"A": 1}, {})
    place(core, "r1", WORK_DOCK)
    assert not cmds(core.tick(), "FillCmd"), "success means the server finished trying"
    assert core._missions["r1"].order.missing == {"C": 1}


def test_nothing_placed_reports_held_and_goes_on_with_the_empty_basket():
    core, _, fill = filling()
    core.on_fill_result(fill.goal_id, True, {}, {"A": 1, "C": 1})
    place(core, "r1", WORK_DOCK)
    out = core.tick()
    report = cmds(out, "ReportOrderCmd")[0]
    assert (report.order_id, report.robot, report.status, report.placed, report.missing) == (
        "O-001", "r1", "HELD", {}, {"A": 1, "C": 1})
    assert view(core, "r1")["order"] is None
    assert view(core, "r1")["basket"] == "EMPTY"
    assert core.shipped == 0
    place(core, "r1", WORK_DOCK)
    core.tick()
    assert view(core, "r1")["state"] == "TO_PARK"


def test_after_a_held_order_the_robot_at_the_dock_takes_the_next_one_in_place():
    core, _, fill = filling([O1, O2])
    core.on_fill_result(fill.goal_id, True, {}, {"A": 1, "C": 1})
    place(core, "r1", WORK_DOCK)
    core.tick()
    place(core, "r1", WORK_DOCK)
    out = core.tick()
    assert view(core, "r1")["order"] == "O-002"
    assert cmds(out, "FillCmd")[0].items == {"B": 1}


def test_order_over_the_capacity_is_held_without_calling_an_arm():
    too_big = OrderRecord("O-001", {"A": 3, "B": 2})
    assert sum(too_big.items.values()) > load_params().basket_capacity
    core, _ = started({"r1": (4, 3)}, [too_big, O2])
    out = serve(core, ["r1"], lambda: view(core, "r1")["phase"] == "FILLING")
    out = core.tick()
    assert "ORDER_BOUND" in codes(out), "it is bound at the dock like any other"
    assert not cmds(out, "FillCmd"), "no arm for an order that cannot fit"
    report = cmds(out, "ReportOrderCmd")[0]
    assert report.status == "HELD"
    assert "capacity" in report.detail
    assert report.placed == {}
    assert report.missing == {}, "no arm looked, so central must not correct its stock record"
    assert view(core, "r1")["basket"] == "EMPTY"
    place(core, "r1", WORK_DOCK)
    assert cmds(core.tick(), "FillCmd")[0].order_id == "O-002", "on to the next order"


def test_order_exactly_at_the_capacity_is_filled():
    core, _, fill = filling([OrderRecord("O-001", {"A": 2, "B": 2})])
    assert fill.items == {"A": 2, "B": 2}


def test_without_ship_partial_a_partial_basket_is_held_and_needs_the_operator():
    params = replace(load_params(), ship_partial=False)
    core, _, fill = filling(params=params)
    core.on_fill_result(fill.goal_id, True, {"A": 1}, {"C": 1})
    place(core, "r1", WORK_DOCK)
    out = core.tick()
    report = cmds(out, "ReportOrderCmd")[0]
    assert report.status == "HELD"
    assert "A" in report.detail, "the operator has to know what is in the basket"
    assert "FILL_FAILED" in codes(out)
    assert view(core, "r1")["state"] == "ERROR"
    assert view(core, "r1")["order"] is None
    assert not cmds(out, "MoveCmd")


def test_without_ship_partial_a_complete_basket_still_ships():
    params = replace(load_params(), ship_partial=False)
    core, _, fill = filling(params=params)
    core.on_fill_result(fill.goal_id, True, {"A": 1, "C": 1}, {})
    place(core, "r1", WORK_DOCK)
    core.tick()
    assert view(core, "r1")["state"] == "TO_OUTBOUND"


def test_reported_order_still_on_a_stale_board_is_not_taken_again():
    core, _, fill = filling([O1])
    core.on_fill_result(fill.goal_id, True, {}, {"A": 1, "C": 1})
    place(core, "r1", WORK_DOCK)
    assert cmds(core.tick(), "ReportOrderCmd")
    core.on_order_board("RUN", [O1])  # 중앙이 아직 보고를 처리하지 못함
    place(core, "r1", WORK_DOCK)
    out = core.tick()
    assert "ORDER_BOUND" not in codes(out)
    assert view(core, "r1")["called"] is False


# ---- swapping -------------------------------------------------------------------------------


def swapping(orders=(O1,), placed=None, missing=None, robots=None):
    """OUTBOUND에 가득 찬 바구니로 도킹한 로봇, 첫 SwapCmd가 나간 상태."""
    core, now, fill = filling(orders, robots=robots)
    placed = placed if placed is not None else dict(fill.items)
    core.on_fill_result(fill.goal_id, True, placed, missing or {})
    out = serve(core, list(robots or {"r1": 0}), lambda: view(core, "r1")["phase"] == "SWAPPING")
    swaps = cmds(out, "SwapCmd") or cmds(core.tick(), "SwapCmd")
    return core, now, swaps[0]


def test_swap_takes_the_full_basket_and_puts_an_empty_one():
    _, _, swap = swapping()
    assert (swap.arm, swap.robot, swap.station_id, swap.order_id) == (
        "arm3", "r1", "OUTBOUND", "O-001")
    assert (swap.take_full, swap.put_empty) == (True, True)


def test_complete_swap_reports_shipped_and_the_robot_goes_on():
    core, _, swap = swapping()
    assert core.shipped == 0
    core.on_swap_result(swap.goal_id, True, True, True)
    assert core.shipped == 1
    assert view(core, "r1")["basket"] == "EMPTY"
    place(core, "r1", OUT_DOCK)
    out = core.tick()
    report = cmds(out, "ReportOrderCmd")[0]
    assert (report.order_id, report.robot, report.status, report.placed, report.missing) == (
        "O-001", "r1", "SHIPPED", {"A": 1, "C": 1}, {})
    assert view(core, "r1")["state"] == "TO_PARK"


def test_order_with_missing_items_reports_partial():
    core, _, swap = swapping(placed={"A": 1}, missing={"C": 1})
    core.on_swap_result(swap.goal_id, True, True, True)
    place(core, "r1", OUT_DOCK)
    report = cmds(core.tick(), "ReportOrderCmd")[0]
    assert (report.status, report.placed, report.missing) == ("PARTIAL", {"A": 1}, {"C": 1})
    assert core.shipped == 1


def test_empty_robot_with_orders_waiting_goes_straight_back_to_work():
    core, _, swap = swapping([O1, O2])
    core.on_swap_result(swap.goal_id, True, True, True)
    place(core, "r1", OUT_DOCK)
    core.tick()
    v = view(core, "r1")
    assert (v["state"], v["destination"]) == ("TO_WORK", "WORK")
    assert not set(v["path"]) & set(core.site.parking)


def test_partial_swap_retries_only_the_open_step_and_reports_shipped_once():
    core, now, swap = swapping()
    core.on_swap_result(swap.goal_id, False, True, False, "GRIPPER")
    assert view(core, "r1")["basket"] == "NONE"
    assert view(core, "r1")["order"] is None, "the order left with the basket"
    now[0] += core.params.arm_retry_wait_s
    place(core, "r1", OUT_DOCK)
    out = core.tick()
    assert [r.status for r in cmds(out, "ReportOrderCmd")] == ["SHIPPED"], "right away"
    assert "SWAP_RETRY" in codes(out)
    retry = cmds(out, "SwapCmd")[0]
    assert (retry.take_full, retry.put_empty, retry.order_id) == (False, True, "O-001")
    core.on_swap_result(retry.goal_id, True, False, True)
    place(core, "r1", OUT_DOCK)
    out = core.tick()
    assert not cmds(out, "ReportOrderCmd"), "reported exactly once"
    assert core.shipped == 1
    assert view(core, "r1")["basket"] == "EMPTY"


def test_swap_server_repeating_took_full_on_a_retry_does_not_report_twice():
    core, now, swap = swapping()
    core.on_swap_result(swap.goal_id, False, True, False, "GRIPPER")
    now[0] += core.params.arm_retry_wait_s
    place(core, "r1", OUT_DOCK)
    retry = cmds(core.tick(), "SwapCmd")[0]
    core.on_swap_result(retry.goal_id, True, True, True)
    place(core, "r1", OUT_DOCK)
    assert not cmds(core.tick(), "ReportOrderCmd")
    assert core.shipped == 1


def test_swap_failing_before_the_take_retries_both_steps():
    core, now, swap = swapping()
    core.on_swap_result(swap.goal_id, False, False, False, "GRIPPER")
    now[0] += core.params.arm_retry_wait_s
    place(core, "r1", OUT_DOCK)
    out = core.tick()
    retry = cmds(out, "SwapCmd")[0]
    assert (retry.take_full, retry.put_empty) == (True, True)
    assert not cmds(out, "ReportOrderCmd")
    assert view(core, "r1")["order"] == "O-001"


def test_swap_retries_run_out_into_error():
    core, now, swap = swapping()
    seen = []
    for attempt in range(core.params.move_retry_limit + 1):
        core.on_swap_result(swap.goal_id, False, False, False, "GRIPPER")
        now[0] += core.params.arm_retry_wait_s
        place(core, "r1", OUT_DOCK)
        out = core.tick()
        seen += [e for e in cmds(out, "EventOut") if e.code == "SWAP_FAILED"]
        if attempt < core.params.move_retry_limit:
            swap = cmds(out, "SwapCmd")[0]
    assert len(seen) == 1 and seen[0].level == "ERROR"
    assert view(core, "r1")["state"] == "ERROR"
    assert view(core, "r1")["order"] == "O-001"


def test_canceled_swap_does_not_count_as_an_attempt():
    core, _, swap = swapping()
    core.on_order_board("STOP", [O1])
    assert [c.robot for c in cmds(core.tick(), "CancelCmd")] == ["arm3"]
    core.on_swap_result(swap.goal_id, False, False, False, "CANCELED")
    assert "SWAP_RETRY" not in codes(core.tick())
    assert view(core, "r1")["attempts"] == 0


def test_robot_with_an_empty_basket_passes_the_outbound_dock_without_docking():
    core, _ = started({"r1": (9, 5)})
    out = serve(core, ["r1"], lambda: view(core, "r1")["state"] == "PARKED")
    assert not cmds(out, "DockCmd")
    assert not cmds(out, "SwapCmd")


# ---- arm time limits ------------------------------------------------------------------------


def advance(core, now, seconds, cell=WORK_DOCK):
    now[0] += seconds
    place(core, "r1", cell)
    return core.tick()


def test_fill_timeout_cancels_then_a_result_in_time_is_used():
    core, now, fill = filling()
    out = advance(core, now, core.params.fill_timeout_s + 0.1)
    assert [(c.robot, c.goal_id) for c in cmds(out, "CancelCmd")] == [("workcell", fill.goal_id)]
    assert view(core, "r1")["state"] == "FILLING", "the arm gets time to stop first"
    core.on_fill_result(fill.goal_id, True, {"A": 1, "C": 1}, {})
    advance(core, now, core.params.arm_cancel_wait_s + 0.1)
    assert view(core, "r1")["state"] == "TO_OUTBOUND"


def test_fill_with_no_result_after_the_cancel_is_an_error_and_the_late_result_only_adds_to_it():
    core, now, fill = filling()
    advance(core, now, core.params.fill_timeout_s + 0.1)
    out = advance(core, now, core.params.arm_cancel_wait_s + 0.1)
    errors = [e for e in cmds(out, "EventOut") if e.code == "ARM_NO_RESULT"]
    assert len(errors) == 1 and errors[0].level == "ERROR" and errors[0].robot == "r1"
    v = view(core, "r1")
    assert (v["state"], v["order"]) == ("ERROR", "O-001")
    core.on_fill_result(fill.goal_id, True, {"A": 1, "C": 1}, {})
    assert "ARM_LATE_RESULT" in codes(core.tick())
    v = view(core, "r1")
    assert v["placed"] == {"A": 1, "C": 1}, "it is in the basket, so the order says so"
    assert (v["state"], v["order"]) == ("ERROR", "O-001"), "the operator still has to look"
    assert not cmds(core.tick(), "FillCmd")


def test_swap_with_no_result_after_the_cancel_is_an_error():
    core, now, swap = swapping()
    out = advance(core, now, core.params.swap_timeout_s + 0.1, OUT_DOCK)
    assert [(c.robot, c.goal_id) for c in cmds(out, "CancelCmd")] == [("arm3", swap.goal_id)]
    out = advance(core, now, core.params.arm_cancel_wait_s + 0.1, OUT_DOCK)
    assert "ARM_NO_RESULT" in codes(out)
    assert view(core, "r1")["state"] == "ERROR"
    assert view(core, "r1")["basket"] == "FULL"
    assert core.shipped == 0


def test_a_late_swap_result_ships_once_and_the_clear_does_not_put_the_order_back():
    core, now, swap = swapping()
    advance(core, now, core.params.swap_timeout_s + 0.1, OUT_DOCK)
    advance(core, now, core.params.arm_cancel_wait_s + 0.1, OUT_DOCK)
    core.on_swap_result(swap.goal_id, False, True, False, "CANCELED")
    out = core.tick()
    assert "ARM_LATE_RESULT" in codes(out)
    assert [c.status for c in cmds(out, "ReportOrderCmd")] == ["SHIPPED"]
    assert view(core, "r1")["state"] == "ERROR", "the operator still has to look"
    core.confirm_clear("r1", core.zones.held_by("r1"), "basket gone, tray empty")
    out = core.tick()
    assert "ORDER_REQUEUED" not in codes(out)
    assert not cmds(out, "ReportOrderCmd")
    assert core.shipped == 1


def test_arm_gets_the_cancel_wait_to_stop_not_the_shorter_recovery_check():
    """실제 팔은 recover_check_s 안에 멈출 수 없어, 시간 초과 시 취소 대기 시간을 받는다."""
    assert load_params().arm_cancel_wait_s > load_params().recover_check_s
    core, now, fill = filling()
    advance(core, now, core.params.fill_timeout_s + 0.1)
    out = advance(core, now, core.params.arm_cancel_wait_s - 0.1)
    assert "ARM_NO_RESULT" not in codes(out)
    core.on_fill_result(fill.goal_id, False, {"A": 1}, {}, "CANCELED")
    assert view(core, "r1")["state"] == "FILLING"
    assert view(core, "r1")["placed"] == {"A": 1}, "the result inside the wait counts"


def test_timed_out_swap_gets_the_cancel_wait_too():
    core, now, swap = swapping()
    advance(core, now, core.params.swap_timeout_s + 0.1, OUT_DOCK)
    assert "ARM_NO_RESULT" not in codes(
        advance(core, now, core.params.arm_cancel_wait_s - 0.1, OUT_DOCK))
    core.on_swap_result(swap.goal_id, True, True, True)
    assert core.shipped == 1


def test_cancel_wait_comes_from_the_params():
    params = replace(load_params(), arm_cancel_wait_s=20.0)
    core, now, _ = filling(params=params)
    advance(core, now, params.fill_timeout_s + 0.1)
    assert "ARM_NO_RESULT" not in codes(advance(core, now, 19.0))
    assert "ARM_NO_RESULT" in codes(advance(core, now, 1.1))


def test_timed_out_job_answering_canceled_uses_up_an_attempt():
    """그렇지 않으면 멈추지만 취소는 되는 팔을 끝없이 재시도한다."""
    core, now, fill = filling()
    for _ in range(core.params.move_retry_limit + 1):
        advance(core, now, core.params.fill_timeout_s + 0.1)
        core.on_fill_result(fill.goal_id, False, {}, {}, "CANCELED")
        out = advance(core, now, core.params.arm_retry_wait_s)
        if cmds(out, "FillCmd"):
            fill = cmds(out, "FillCmd")[0]
    assert view(core, "r1")["state"] == "ERROR"
    assert "FILL_FAILED" in codes(out)


def test_results_for_other_goals_change_nothing():
    core, _, fill = filling()
    core.on_fill_result(fill.goal_id + 100, True, {"A": 1}, {})
    core.on_swap_result(fill.goal_id, True, True, True)
    assert view(core, "r1")["placed"] == {}
    assert view(core, "r1")["basket"] == "EMPTY"


# ---- STOP and PAUSE -------------------------------------------------------------------------


def test_stop_cancels_the_fill_and_run_resumes_with_what_remains():
    core, _, fill = filling()
    core.on_order_board("STOP", [O1])
    place(core, "r1", WORK_DOCK)
    assert [c.goal_id for c in cmds(core.tick(), "CancelCmd")] == [fill.goal_id]
    core.on_fill_result(fill.goal_id, False, {"A": 1}, {}, "CANCELED")
    place(core, "r1", WORK_DOCK)
    assert not cmds(core.tick(), "FillCmd"), "nothing new during STOP"
    core.on_order_board("RUN", [O1])
    place(core, "r1", WORK_DOCK)
    assert cmds(core.tick(), "FillCmd")[0].items == {"C": 1}


def test_stop_during_a_swap_resumes_with_the_open_step():
    core, _, swap = swapping()
    core.on_order_board("STOP", [O1])
    place(core, "r1", OUT_DOCK)
    core.tick()
    core.on_swap_result(swap.goal_id, False, True, False, "CANCELED")
    assert core.shipped == 1, "the basket did leave"
    core.on_order_board("RUN", [])
    place(core, "r1", OUT_DOCK)
    retry = cmds(core.tick(), "SwapCmd")[0]
    assert (retry.take_full, retry.put_empty) == (False, True)


def test_stop_with_a_silent_arm_ends_in_arm_no_result():
    core, now, _ = filling()
    core.on_order_board("STOP", [O1])
    place(core, "r1", WORK_DOCK)
    core.tick()
    assert "ARM_NO_RESULT" in codes(advance(core, now, core.params.arm_cancel_wait_s + 0.1))


def test_stop_with_a_slow_arm_is_not_an_error():
    """실제 팔은 멈추는 데 recover_check_s보다 걸린다. STOP이 ERROR로 끝나면 안 된다."""
    core, now, fill = filling()
    core.on_order_board("STOP", [O1])
    place(core, "r1", WORK_DOCK)
    core.tick()
    out = advance(core, now, core.params.arm_cancel_wait_s - 0.1)
    assert "ARM_NO_RESULT" not in codes(out)
    assert view(core, "r1")["state"] == "FILLING"
    core.on_fill_result(fill.goal_id, False, {"A": 1}, {}, "CANCELED")
    assert view(core, "r1")["attempts"] == 0
    core.on_order_board("RUN", [O1])
    place(core, "r1", WORK_DOCK)
    assert cmds(core.tick(), "FillCmd")[0].items == {"C": 1}


def test_pause_lets_the_fill_finish_but_sends_nothing_new():
    core, _, fill = filling()
    core.on_order_board("PAUSE", [O1])
    place(core, "r1", WORK_DOCK)
    assert not cmds(core.tick(), "CancelCmd"), "what is running is left to finish"
    core.on_fill_result(fill.goal_id, True, {"A": 1, "C": 1}, {})
    place(core, "r1", WORK_DOCK)
    assert not cmds(core.tick(), "MoveCmd")
    core.on_order_board("RUN", [O1])
    place(core, "r1", WORK_DOCK)
    assert view(core, "r1")["state"] == "FILLING"
    core.tick()
    assert view(core, "r1")["state"] == "TO_OUTBOUND"


# ---- waiting before the next arm goal -------------------------------------------------------


def clear_mid_fill():
    """pinky1을 들어내는 중에 pinky2가 도킹하고 작업셀은 아직 멈추는 중이다."""
    core, now = make_core(["pinky1", "pinky2"])
    place(core, "pinky1", core.site.parking[5])
    place(core, "pinky2", core.site.parking[4])
    core.on_order_board("RUN", [O1, O2])
    serve(core, ["pinky1"], lambda: core._missions["pinky1"].arm is not None)
    canceled = core._missions["pinky1"].arm.goal_id
    core.confirm_clear("pinky1", core.zones.held_by("pinky1"), "lifted mid-fill")
    out = core.tick()
    assert [(c.robot, c.goal_id) for c in cmds(out, "CancelCmd")] == [("workcell", canceled)]
    assert "ORDER_REQUEUED" in codes(out)
    out = serve(core, ["pinky2"], lambda: view(core, "pinky2")["phase"] == "FILLING")
    out += core.tick()
    assert view(core, "pinky2")["order"] == "O-001", "the requeued order comes first"
    assert not cmds(out, "FillCmd"), "the workcell has not finished the canceled goal"
    return core, now, canceled


def test_no_fill_goes_out_before_the_retry_wait():
    core, now, fill = filling()
    core.on_fill_result(fill.goal_id, False, {}, {}, "ARM_FAULT")
    now[0] += core.params.arm_retry_wait_s - 0.1
    place(core, "r1", WORK_DOCK)
    out = core.tick()
    assert "FILL_RETRY" in codes(out), "the failure is reported right away"
    assert not cmds(out, "FillCmd")
    now[0] += 0.1
    place(core, "r1", WORK_DOCK)
    assert cmds(core.tick(), "FillCmd")


def test_no_swap_goes_out_before_the_retry_wait():
    core, now, swap = swapping()
    core.on_swap_result(swap.goal_id, False, False, False, "ARM_FAULT")
    now[0] += core.params.arm_retry_wait_s - 0.1
    place(core, "r1", OUT_DOCK)
    assert not cmds(core.tick(), "SwapCmd")
    now[0] += 0.1
    place(core, "r1", OUT_DOCK)
    assert cmds(core.tick(), "SwapCmd")


def test_a_rejected_goal_waits_like_a_failed_one():
    core, now, fill = filling()
    core.on_fill_result(fill.goal_id, False, {}, {}, "REJECTED")
    place(core, "r1", WORK_DOCK)
    assert not cmds(core.tick(), "FillCmd")
    now[0] += core.params.arm_retry_wait_s
    place(core, "r1", WORK_DOCK)
    assert cmds(core.tick(), "FillCmd")


def test_the_retry_wait_comes_from_the_params():
    params = replace(load_params(), arm_retry_wait_s=7.0)
    core, now, fill = filling(params=params)
    core.on_fill_result(fill.goal_id, False, {}, {}, "ARM_FAULT")
    now[0] += 6.9
    place(core, "r1", WORK_DOCK)
    assert not cmds(core.tick(), "FillCmd")
    now[0] += 0.2
    place(core, "r1", WORK_DOCK)
    assert cmds(core.tick(), "FillCmd")


def test_a_stop_cancel_does_not_add_a_retry_wait():
    core, _, fill = filling()
    core.on_order_board("STOP", [O1])
    place(core, "r1", WORK_DOCK)
    core.tick()
    core.on_fill_result(fill.goal_id, False, {"A": 1}, {}, "CANCELED")
    core.on_order_board("RUN", [O1])
    place(core, "r1", WORK_DOCK)
    assert cmds(core.tick(), "FillCmd")


def test_next_robot_waits_for_the_result_of_the_cleared_robots_goal():
    core, now, canceled = clear_mid_fill()
    now[0] += core.params.arm_cancel_wait_s - 0.1
    place(core, "pinky2", WORK_DOCK)
    assert not cmds(core.tick(), "FillCmd")
    core.on_fill_result(canceled, False, {"A": 1}, {}, "CANCELED")
    place(core, "pinky2", WORK_DOCK)
    out = core.tick()
    assert cmds(out, "FillCmd")[0].robot == "pinky2"
    assert not [c for c in codes(out) if c.startswith("FILL_")]
    assert view(core, "pinky2")["attempts"] == 0, "the previous job counts against nobody"


def test_next_robot_goes_ahead_when_the_canceled_goal_never_answers():
    core, now, _ = clear_mid_fill()
    now[0] += core.params.arm_cancel_wait_s
    place(core, "pinky2", WORK_DOCK)
    assert cmds(core.tick(), "FillCmd")[0].robot == "pinky2"
    assert view(core, "pinky2")["state"] == "FILLING"


def test_busy_workcell_rejections_spread_over_the_retry_wait():
    """데모를 깨뜨린 순서: 0.3초 안의 거절 세 번으로 pinky2가 ERROR가 됐다."""
    core, now, _ = clear_mid_fill()
    now[0] += core.params.arm_cancel_wait_s
    started_at = now[0]
    for _ in range(core.params.move_retry_limit):
        place(core, "pinky2", WORK_DOCK)
        fill = cmds(core.tick(), "FillCmd")[0]
        core.on_fill_result(fill.goal_id, False, {}, {}, "REJECTED")
        now[0] += 0.1
        place(core, "pinky2", WORK_DOCK)
        assert not cmds(core.tick(), "FillCmd"), "no resend right after a rejection"
        now[0] += core.params.arm_retry_wait_s
    assert view(core, "pinky2")["state"] == "FILLING"
    assert now[0] - started_at > 2 * core.params.arm_retry_wait_s
    place(core, "pinky2", WORK_DOCK)
    fill = cmds(core.tick(), "FillCmd")[0]
    core.on_fill_result(fill.goal_id, True, {"A": 1, "C": 1}, {})
    place(core, "pinky2", WORK_DOCK)
    core.tick()
    assert view(core, "pinky2")["basket"] == "FULL", "the arm got time to finish stopping"


def test_next_robot_at_the_swap_arm_waits_for_the_cleared_robots_goal():
    core, now = make_core(["r1", "r2"])
    place(core, "r1", (9, 4))
    place(core, "r2", (9, 5))
    core.on_order_board("RUN", [])
    core._missions["r1"].basket = core._missions["r2"].basket = "FULL"
    core._missions["r1"].last_order_id = core._missions["r2"].last_order_id = "O-001"
    serve(core, ["r1"], lambda: core._missions["r1"].arm is not None)
    canceled = core._missions["r1"].arm.goal_id
    core.confirm_clear("r1", core.zones.held_by("r1"), "lifted mid-swap")
    core.tick()
    out = serve(core, ["r2"], lambda: view(core, "r2")["phase"] == "SWAPPING")
    assert not cmds(out + core.tick(), "SwapCmd")
    core.on_swap_result(canceled, False, False, False, "CANCELED")
    place(core, "r2", OUT_DOCK)
    assert cmds(core.tick(), "SwapCmd")[0].robot == "r2"


def test_a_held_server_does_not_hold_the_other_server():
    core, _, canceled = clear_mid_fill()
    assert core._cleared_jobs["workcell"][0] == canceled
    assert core._arm_goal_allowed(core._missions["pinky2"], "arm3")


# ---- no robot left --------------------------------------------------------------------------


def no_robot_left():
    core, now = make_core(["r1", "r2"])
    place(core, "r1", (3, 4))
    place(core, "r2", (3, 5))
    core.on_order_board("RUN", [])
    core.tick()
    for robot in ("r1", "r2"):
        core.confirm_clear(robot, core.zones.held_by(robot), "gone")
    core.tick()
    return core, now


def warnings(out):
    return [e for e in cmds(out, "EventOut") if e.code == "NO_ROBOT_AVAILABLE"]


def test_orders_with_no_robot_left_warn_once():
    core, now = no_robot_left()
    core.on_order_board("RUN", [O1])
    found = warnings(core.tick())
    assert len(found) == 1 and found[0].level == "WARN" and "O-001" in found[0].detail
    for _ in range(20):
        now[0] += 1.0
        core.on_order_board("RUN", [O1, O2])
        assert not warnings(core.tick()), "one warning per stall"


def test_a_new_stall_warns_again():
    core, _ = no_robot_left()
    core.on_order_board("RUN", [O1])
    assert warnings(core.tick())
    core.on_order_board("RUN", [])
    assert not warnings(core.tick())
    core.on_order_board("RUN", [O2])
    assert warnings(core.tick())


def test_no_warning_without_orders_or_with_a_robot_left():
    core, _ = no_robot_left()
    assert not warnings(core.tick()), "nothing is waiting"
    core, _ = make_core(["r1", "r2"])
    place(core, "r1", (3, 4))
    place(core, "r2", (3, 5))
    core.on_order_board("RUN", [O1])
    core.tick()
    core.confirm_clear("r1", core.zones.held_by("r1"), "gone")
    assert not warnings(core.tick()), "r2 can still take it"


def test_last_robot_failing_is_a_stall_too():
    core, _, _ = filling([O1, O2])
    core._fail(core._missions["r1"])
    assert warnings(core.tick()), "O-002 waits and r1 needs the operator"


# ---- confirm_clear --------------------------------------------------------------------------


def test_confirm_clear_releases_listed_zones_and_marks_out_of_service():
    core, _ = started({"r1": (9, 5), "r2": (3, 4)})
    core.tick()
    released = core.confirm_clear("r1", ["Q-OUT-2", "Q-OUT-1", "Z-OUT"], "moved by hand")
    assert released == ["Q-OUT-2", "Q-OUT-1"], "a zone it does not hold is not reported"
    assert view(core, "r1")["state"] == "OUT_OF_SERVICE"
    out = core.tick()
    assert "ZONES_CLEARED" in codes(out)
    assert "ORDER_REQUEUED" not in codes(out), "it held no order"


def test_confirm_clear_for_an_unknown_robot_changes_nothing():
    """운영자 콘솔의 오타로 플릿 매니저가 죽으면 안 된다."""
    core, _ = make_core(["r1"])
    place(core, "r1", (9, 5))
    assert core.confirm_clear("pinky9", ["Q-OUT-2"], note="typo") is None
    assert core.zones.holder("Q-OUT-2") == "r1"
    assert "INPUT_REJECTED" in codes(core.tick()), "모르는 이름은 잘못된 입력이다"


def test_confirm_clear_with_a_zone_that_does_not_exist_changes_nothing():
    """이름 하나의 오타로 매니저가 죽거나 로봇이 반만 풀리면 안 된다."""
    core, _ = started({"r1": (9, 5), "r2": (3, 4)})
    core.tick()
    held = core.zones.held_by("r1")
    assert core.confirm_clear("r1", [*held, "NOPE"], "typo") is None
    assert core.zones.held_by("r1") == held
    assert view(core, "r1")["state"] != "OUT_OF_SERVICE"
    out = core.tick()
    assert "INPUT_REJECTED" in codes(out)
    assert "ZONES_CLEARED" not in codes(out)


def test_confirm_clear_with_no_zones_is_refused_while_the_robot_holds_some():
    """풀어 줄 것이 없다. 서비스 제외 로봇은 자세로 아무것도 놓지 않는다."""
    core, _ = started({"r1": (9, 5), "r2": (3, 4)})
    core.tick()
    held = core.zones.held_by("r1")
    assert held
    assert core.confirm_clear("r1", [], "forgot the list") is None
    assert core.zones.held_by("r1") == held
    assert view(core, "r1")["state"] != "OUT_OF_SERVICE"
    assert "CLEAR_REFUSED" in codes(core.tick())


def test_confirm_clear_with_some_of_the_zones_warns_and_another_call_frees_the_rest():
    """운영자의 뜻일 수 있어 받아들이되, 잠긴 채 남는 것을 경고해야 한다."""
    core, _ = started({"r1": (9, 5), "r2": (3, 4)})
    core.tick()
    held = core.zones.held_by("r1")
    assert len(held) >= 2
    assert core.confirm_clear("r1", held[:1], "one first") == held[:1]
    assert view(core, "r1")["state"] == "OUT_OF_SERVICE"
    assert "ZONES_STILL_HELD" in codes(core.tick())
    assert core.confirm_clear("r1", held[1:], "and the rest") == held[1:]
    assert core.zones.held_by("r1") == []
    assert "ZONES_STILL_HELD" not in codes(core.tick())


def test_confirm_clear_cancels_a_running_job_and_ignores_its_result():
    core, now, fill = filling()
    core.confirm_clear("r1", ["Z-WORK"], "operator took the robot")
    out = core.tick()
    assert [(c.robot, c.goal_id) for c in cmds(out, "CancelCmd")] == [("workcell", fill.goal_id)]
    core.on_fill_result(fill.goal_id, True, {"A": 1, "C": 1}, {})
    assert view(core, "r1")["placed"] == {}
    now[0] += core.params.fill_timeout_s + 1.0
    assert "ARM_NO_RESULT" not in codes(core.tick())


def test_requeue_puts_the_order_back_in_registration_order_with_what_was_in_the_basket():
    core, _, fill = filling([O1, O2, O3], robots={"r1": (4, 3), "r2": (3, 4)})
    core.on_fill_result(fill.goal_id, False, {"A": 1}, {}, "GRIPPER")
    core.confirm_clear("r1", ["Z-WORK"], "operator took the robot")
    event = next(e for e in cmds(core.tick(), "EventOut") if e.code == "ORDER_REQUEUED")
    assert event.detail.startswith("O-001")
    assert "'A': 1" in event.detail
    assert view(core, "r1")["order"] is None
    serve(core, ["r2"], lambda: view(core, "r2")["order"] is not None)
    assert view(core, "r2")["order"] == "O-001", "not O-002: it keeps its place"
    assert core._missions["r2"].order.placed == {}, "the new basket starts empty"


def test_requeue_while_another_order_is_being_filled_goes_ahead_of_the_rest():
    core, _ = started({"r1": (4, 3), "r2": (3, 3)}, [O1, O2, O3])
    serve(core, ["r1"], lambda: view(core, "r1")["phase"] == "FILLING")
    fill = cmds(core.tick(), "FillCmd")
    assert view(core, "r1")["order"] == "O-001"
    assert core._pending() == [O2, O3]
    if fill:
        core.on_fill_result(fill[0].goal_id, True, dict(fill[0].items), {})
    core.confirm_clear("r1", ["Z-WORK"], "operator took the robot")
    assert core._pending() == [O1, O2, O3]


def test_confirm_clear_after_the_take_does_not_requeue():
    core, _, swap = swapping()
    core.on_swap_result(swap.goal_id, False, True, False, "GRIPPER")
    core.confirm_clear("r1", ["Z-OUT"], "operator took the robot")
    out = core.tick()
    assert "ORDER_REQUEUED" not in codes(out)
    assert core._pending() == []


def test_confirm_clear_before_the_take_requeues_with_the_basket_contents():
    core, _, _ = swapping()
    core.confirm_clear("r1", ["Z-OUT"], "operator took the robot")
    event = next(e for e in cmds(core.tick(), "EventOut") if e.code == "ORDER_REQUEUED")
    assert event.detail.startswith("O-001") and "'C': 1" in event.detail
    assert core._pending() == [O1]


# ---- views and startup ----------------------------------------------------------------------


def test_views_use_the_published_types():
    core, _ = started()
    core.tick()
    zone_views = core.zones_view()
    assert all(isinstance(z, ZoneView) for z in zone_views)
    assert len(zone_views) == len(core.site.zones)
    p6 = next(z for z in zone_views if z.zone_id == "P-6")
    assert p6.holder == "r1"
    free = next(z for z in zone_views if z.zone_id == "Z-R1")
    assert (free.holder, free.queue, free.acquired_at) == ("", (), None)
    assert core.assignments_view() == [AssignmentView(r, "", "", "WAIT") for r in
                                       ("r1", "r2", "r3")]
    assert {"state", "basket", "order", "held", "path"} <= set(view(core, "r1"))


def assignment(core, robot="r1"):
    return next(a for a in core.assignments_view() if a.robot_id == robot)


def test_assignment_view_follows_the_mission():
    core, _ = started({"r1": (4, 3)}, [O1])
    core.tick()
    assert assignment(core) == AssignmentView("r1", "", "WORK", "TO_WORK")
    fill = cmds(serve(core, ["r1"], lambda: view(core, "r1")["phase"] == "FILLING"), "FillCmd")
    assert assignment(core) == AssignmentView("r1", "O-001", "WORK", "FILL")
    fill = fill or cmds(core.tick(), "FillCmd")
    core.on_fill_result(fill[0].goal_id, True, dict(fill[0].items), {})
    place(core, "r1", WORK_DOCK)
    core.tick()
    assert assignment(core) == AssignmentView("r1", "O-001", "OUTBOUND", "TO_OUTBOUND")
    serve(core, ["r1"], lambda: view(core, "r1")["phase"] == "SWAPPING")
    assert assignment(core) == AssignmentView("r1", "O-001", "OUTBOUND", "SWAP")


def test_out_of_service_robot_is_left_out_of_the_assignments():
    core, _ = started({"r1": (3, 4), "r2": (3, 5)})
    core.confirm_clear("r1", ["P-6"])
    assert [a.robot_id for a in core.assignments_view()] == ["r2"]


def test_map_too_small_for_the_release_distance_is_refused():
    """그보다 짧은 레인 끝에 선 로봇은 구역을 놓지 못한다."""
    with pytest.raises(ValueError, match="release distance"):
        FleetCore(grid_site(), replace(load_params(), release_distance_m=0.33), ["r1"],
                  lambda: 0.0)


def test_nothing_moves_until_every_robot_has_reported():
    """보고 안 한 로봇이 다른 로봇이 들어갈 구역에 서 있을 수 있다."""
    core, _ = make_core(["r1", "r2"])
    place(core, "r1", (3, 5))  # P-5, 앞의 P-6에는 r2가 아직 무응답
    core.on_order_board("RUN", [O1, O2])
    assert not cmds(core.tick(), "MoveCmd")
    assert core.zones.held_by("r1") == ["P-5"]
    assert core.zones.holder("P-6") is None

    place(core, "r2", (3, 4))
    assert "PLACEMENT_CONFLICT" not in codes(core.tick())
    assert core.zones.holder("P-6") == "r2"
    assert view(core, "r2")["state"] != "ERROR"


def test_robot_that_never_reports_raises_one_error_and_keeps_everyone_still():
    core, now = make_core(["r1", "r2"])
    place(core, "r1", (9, 5))
    core.on_order_board("RUN", [O1])
    core.tick()
    now[0] = core.params.startup_wait_s + 0.1
    place(core, "r1", (9, 5))
    out = core.tick()
    errors = [e for e in cmds(out, "EventOut") if e.code == "ROBOT_NOT_REPORTING"]
    assert len(errors) == 1 and "r2" in errors[0].detail
    assert not cmds(out, "MoveCmd")
    now[0] += core.params.startup_wait_s
    place(core, "r1", (9, 5))
    assert "ROBOT_NOT_REPORTING" not in codes(core.tick()), "reported once"


def test_a_silent_robot_taken_out_of_service_no_longer_holds_everyone_back():
    """그렇지 않으면 처음부터 고장 난 로봇 하나가 빠져나갈 길 없이 플릿을 세운다."""
    core, _ = make_core(["r1", "r2"])
    place(core, "r1", (9, 5))
    core.on_order_board("RUN", [O1])
    assert not cmds(core.tick(), "MoveCmd")

    core.confirm_clear("r2", [], note="r2 never started, taken off the floor")
    place(core, "r1", (9, 5))
    assert cmds(core.tick(), "MoveCmd"), "r1 goes on alone"
