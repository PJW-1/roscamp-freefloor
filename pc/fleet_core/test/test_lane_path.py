"""레인 폴리라인 목표(지시서 7번, FollowPath). point 모드는 비교용으로 남는다.

path 모드: 잠금을 쥔 데까지 방향이 바뀌어도 한 목표로 묶고, MoveCmd.path에 선을 싣는다.
첫 점은 로봇 위치, 끝 점은 목표 지점이다.
"""
import json
import math
from dataclasses import asdict, replace

import pytest
from core_helpers import cmds, codes
from fleet_core.common.commands import MoveCmd
from fleet_core.common.params import FleetParams, load_params
from fleet_core.manager import FleetManager
from fleet_core.traffic.controller import TrafficController
from fleet_core.traffic.lane_map import load_lane_map


def make(where, mode="path"):
    now = [0.0]
    params = replace(load_params(), lane_goal_mode=mode)
    core = TrafficController(load_lane_map(), params, list(where), lambda: now[0])
    for robot, node in where.items():
        place(core, robot, node)
    core.on_run_mode("RUN")
    return core, now


def place(core, robot, node, along=0.0, to=None, state="IDLE"):
    x, y = core.site.waypoint_pose(node)
    if to is not None:
        lane = core.site.lane(node, to)
        x, y = x + lane.ux * along, y + lane.uy * along
    for _ in range(core.params.release_confirm_samples):
        core.on_robot_state(robot, x, y, 0.0, state)
    return x, y


def send(core, robot, station):
    core.on_dispatch(sum(map(ord, robot + station)), station, robot)


def moves(out, robot="r1"):
    return [c for c in cmds(out, "MoveCmd") if c.robot == robot]


def held(core, robot="r1"):
    return core.robot_view(robot)["held"]


def xy(core, node):
    return core.site.waypoint_pose(node)


# ---- 묶음과 선 ------------------------------------------------------------------------------


def test_the_yaml_default_is_path_and_a_record_without_it_is_point():
    assert load_params().lane_goal_mode == "path"
    fields = asdict(load_params())
    del fields["lane_goal_mode"]
    assert FleetParams(**fields).lane_goal_mode == "point"


def test_path_mode_bundles_past_a_turn_as_far_as_the_locks_go():
    core, _ = make({"r1": "W"})
    send(core, "r1", "OUTBOUND")
    move = moves(core.tick())[0]
    assert held(core) == ["W", "AVENUE"]
    assert move.target_id == "B", "W to A turns onto the avenue, both held"
    assert (move.x, move.y) == xy(core, "B")
    assert move.yaw == pytest.approx(0.0), "faces along the last lane"
    assert [p[:2] for p in move.path] == [xy(core, "W"), xy(core, "A"), xy(core, "B")]


def test_point_mode_stops_at_the_turn_and_sends_no_path():
    core, _ = make({"r1": "W"}, mode="point")
    send(core, "r1", "OUTBOUND")
    move = moves(core.tick())[0]
    assert move.target_id == "A"
    assert move.path == ()


def test_each_point_faces_along_its_lane_and_the_last_takes_the_goal_yaw():
    core, _ = make({"r1": "W"})
    send(core, "r1", "OUTBOUND")
    move = moves(core.tick())[0]
    wa = core.site.lane("W", "A")
    assert move.path[0][2] == pytest.approx(math.atan2(wa.uy, wa.ux))
    assert move.path[1][2] == pytest.approx(0.0)
    assert move.path[-1][2] == pytest.approx(move.yaw)


def test_the_first_point_is_where_the_robot_is_not_the_waypoint():
    core, _ = make({"r1": "W"})
    x, y = place(core, "r1", "W", 0.03, "A")
    send(core, "r1", "OUTBOUND")
    move = moves(core.tick())[0]
    assert move.path[0][:2] == (x, y)
    assert [p[:2] for p in move.path[-2:]] == [xy(core, "A"), xy(core, "B")]


def test_a_waypoint_just_behind_stays_so_a_noisy_corner_is_not_cut():
    """0.15 m 안으로 지난 지점은 남긴다. 그 앞부분은 핑키가 건너뛴다."""
    core, _ = make({"r1": "W"})
    x, y = place(core, "r1", "W", 0.03, "A")
    send(core, "r1", "OUTBOUND")
    move = moves(core.tick())[0]
    assert [p[:2] for p in move.path] == [(x, y), xy(core, "W"), xy(core, "A"), xy(core, "B")]


def drive(core, node, along, to, state="MOVING"):
    """보고 세 번 뒤 tick. 나온 이동 명령을 돌려준다."""
    x, y = place(core, "r1", node, along, to, state)
    return (x, y), moves(core.tick())


def test_a_longer_goal_skips_the_waypoint_the_robot_has_left_behind():
    core, _ = make({"r1": "W"})
    send(core, "r1", "OUTBOUND")
    assert moves(core.tick())[0].target_id == "B"
    # A를 돌아 대로를 0.18 m 갔다(d 안). W를 놓고 R을 얻어 목표를 늘린다.
    (x, y), sent = drive(core, "A", 0.18, "B")
    if not sent:
        (x, y), sent = drive(core, "A", 0.18, "B")
    assert sent and sent[-1].target_id == "R"
    path = [p[:2] for p in sent[-1].path]
    assert path == [(x, y), xy(core, "B"), xy(core, "R")], "A is behind the robot"


def test_the_waypoint_ahead_on_the_lane_in_stays_even_close_to_a_turn():
    """돌기 직전에 목표를 늘려도 모서리를 가로지르지 않는다."""
    core, _ = make({"r1": "W"})
    send(core, "r1", "OUTBOUND")
    core.tick()
    span = core.site.lane("W", "A").span
    (x, y), sent = drive(core, "W", span - 0.06, "A")
    if not sent:
        (x, y), sent = drive(core, "W", span - 0.06, "A")
    assert sent and sent[-1].target_id == "R"
    path = [p[:2] for p in sent[-1].path]
    assert path == [(x, y), xy(core, "A"), xy(core, "B"), xy(core, "R")]


def extend_from_p5(mode):
    """P-5에서 출고로. P-6까지 받고, P-5를 d 넘게 떠나면 N 구역이 온다."""
    core, _ = make({"r1": "P-5"}, mode)
    send(core, "r1", "OUTBOUND")
    first = moves(core.tick())[0]
    d = core.params.release_distance_m
    sent = []
    for _ in range(2):
        sent += drive(core, "P-5", d + 0.05, "P-6")[1]
    return core, first, sent


def test_a_running_goal_is_extended_round_the_corner_when_the_next_zone_comes():
    core, first, sent = extend_from_p5("path")
    assert first.target_id == "P-6"
    assert "N" in held(core)
    assert sent and sent[-1].target_id == "N", "P-6 to N turns, the goal goes on round it"
    assert [p[:2] for p in sent[-1].path][1:] == [xy(core, "P-6"), xy(core, "N")]


def test_point_mode_keeps_one_straight_run_per_goal():
    core, first, sent = extend_from_p5("point")
    assert first.target_id == "P-6"
    assert "N" in held(core)
    assert not sent, "point mode waits for the turn"


def test_a_u_turn_on_a_two_way_lane_is_not_bundled():
    """되돌아가는 레인은 잇지 않는다. 경로를 바꿀 때마다 목표가 끝없이 늘어나지 않게 한다."""
    core, _ = make({"r1": "W"})
    goal = sum(map(ord, "r1OUTBOUND"))
    send(core, "r1", "OUTBOUND")
    first = moves(core.tick())[0]
    assert first.target_id == "B"
    core.on_dispatch_cancel(goal)
    m = core._missions["r1"]
    out = core.tick()
    assert m.path[m.goal.target_idx] == "B"
    assert m.path[m.goal.target_idx + 1] == "A", "the way back runs down the avenue"
    assert not moves(out), "the goal still ends at B"
    send(core, "r1", "OUTBOUND")
    core.on_dispatch_cancel(goal)
    core.tick()
    assert len(m.path) <= 10, m.path


def test_after_the_u_turn_point_the_way_back_is_a_new_goal():
    """전수 검사에서 찾음: 유턴 지점에 선 로봇이 새 목표를 못 받아 굶었다."""
    core, _ = make({"r1": "W"})
    goal = sum(map(ord, "r1OUTBOUND"))
    send(core, "r1", "OUTBOUND")
    first = moves(core.tick())[0]
    core.on_dispatch_cancel(goal)
    core.tick()
    place(core, "r1", "B")
    core.on_move_result("r1", first.goal_id, True, "")
    back = moves(core.tick())
    assert back and back[-1].target_id != "B", "it turns round and goes back down the avenue"
    assert [p[:2] for p in back[-1].path][:2] == [xy(core, "B"), xy(core, "A")]


# ---- 꺾임 너머 목표와 다른 규칙 ---------------------------------------------------------------


def test_driving_the_first_lane_of_a_turning_goal_is_not_a_lane_error():
    core, _ = make({"r1": "W"})
    send(core, "r1", "OUTBOUND")
    core.tick()
    out = []
    span = core.site.lane("W", "A").span
    for along in (0.05, 0.1, span / 2, span - 0.05):
        for _ in range(core.params.release_confirm_samples):
            place(core, "r1", "W", along, "A", "MOVING")
        out += core.tick()
    assert "LANE_ERROR" not in codes(out)


def test_off_the_first_lane_of_a_turning_goal_is_still_a_lane_error():
    core, _ = make({"r1": "W"})
    send(core, "r1", "OUTBOUND")
    core.tick()
    x, y = core.site.waypoint_pose("W")
    lane = core.site.lane("W", "A")
    off = core.params.lane_error_warn_m + 0.05
    for _ in range(3 * core.params.release_confirm_samples):
        core.on_robot_state("r1", x + lane.ux * 0.1 - lane.uy * off,
                            y + lane.uy * 0.1 + lane.ux * off, 0.0, "MOVING")
    assert codes(core.tick()).count("LANE_ERROR") == 1


def test_the_zone_behind_is_released_in_the_middle_of_a_turning_goal():
    core, _ = make({"r1": "W"})
    send(core, "r1", "OUTBOUND")
    move = moves(core.tick())[0]
    assert move.target_id == "B"
    d = core.params.release_distance_m
    place(core, "r1", "W", d + 0.01, "A", "MOVING")
    core.tick()
    assert "W" not in held(core)
    m = core._missions["r1"]
    assert m.goal and m.goal.target_idx >= m.path.index("B"), "no cancel, the goal runs on"


def test_success_away_from_a_target_past_the_turn_is_not_an_arrival():
    core, now = make({"r1": "W"})
    send(core, "r1", "OUTBOUND")
    move = moves(core.tick())[0]
    place(core, "r1", "A", state="MOVING")  # 꺾임에서 성공을 보냄
    core.on_move_result("r1", move.goal_id, True, "")
    m = core._missions["r1"]
    assert m.arrival_by is not None
    now[0] += core.params.arrival_settle_s + 0.1
    place(core, "r1", "A")
    out = core.tick()
    assert "GOAL_RETRY" in codes(out)
    assert m.idx < m.path.index("B")


def test_success_at_the_target_past_the_turn_is_an_arrival():
    core, _ = make({"r1": "W"})
    send(core, "r1", "OUTBOUND")
    move = moves(core.tick())[0]
    place(core, "r1", "B")
    core.on_move_result("r1", move.goal_id, True, "")
    m = core._missions["r1"]
    assert m.arrival_by is None
    assert m.path[m.idx] == "B"


def test_a_robot_stopped_at_the_turn_of_an_open_goal_is_a_stall():
    core, now = make({"r1": "W"})
    send(core, "r1", "OUTBOUND")
    core.tick()
    out = []
    for _ in range(round((core.params.stall_s + 4) / 0.1)):  # 평균 위치가 따라오는 시간
        now[0] = round(now[0] + 0.1, 6)
        core.on_robot_state("r1", *xy(core, "A"), 0.0, "MOVING")
        out += core.tick()
    assert "GOAL_STALLED" in codes(out)


def test_blocked_after_the_turn_marks_the_lane_after_the_turn():
    core, _ = make({"r1": "W"})
    send(core, "r1", "OUTBOUND")
    core.tick()
    d = core.params.release_distance_m
    place(core, "r1", "W", d + 0.05, "A", "MOVING")
    core.tick()
    place(core, "r1", "A", 0.3, "B", "BLOCKED")
    core.tick()
    assert core._missions["r1"].block.lane == ("A", "B")


def test_blocked_before_the_turn_marks_the_lane_before_it():
    core, _ = make({"r1": "W"})
    send(core, "r1", "OUTBOUND")
    core.tick()
    place(core, "r1", "W", 0.1, "A", "BLOCKED")
    core.tick()
    assert core._missions["r1"].block.lane == ("W", "A")


def test_a_path_goal_waits_for_the_cancel_result_after_a_mark():
    """파트 4 규칙: 표시 뒤 취소 결과 전에는 path 모드도 목표를 내지 않는다."""
    core, now = make({"r1": "W"})
    send(core, "r1", "OUTBOUND")
    core.tick()
    out = []
    for _ in range(round((core.params.blocked_mark_s + 1) / 0.1)):
        now[0] = round(now[0] + 0.1, 6)
        place(core, "r1", "W", 0.1, "A", "BLOCKED")
        out += core.tick()
    assert "LANE_BLOCKED" in codes(out)
    canceled = cmds(out, "CancelCmd")[-1].goal_id
    later = []
    for _ in range(10):
        now[0] = round(now[0] + 0.1, 6)
        place(core, "r1", "W", 0.1, "A", "IDLE")
        later += core.tick()
    assert not moves(later), "no goal before the cancel result"
    core.on_move_result("r1", canceled, False, "CANCELED")
    place(core, "r1", "W", 0.1, "A", "IDLE")
    resent = moves(core.tick())
    assert resent and resent[-1].path, "then the same goal goes again with its polyline"


# ---- 기록 --------------------------------------------------------------------------------


def test_a_point_mode_record_line_has_no_path_field():
    lines = []
    params = replace(load_params(), lane_goal_mode="point")
    now = [0.0]
    core = FleetManager(load_lane_map(), params, ["r1"], lambda: now[0])
    core.record_to(lines.append)
    for _ in range(3):
        core.on_robot_state("r1", *core.site.waypoint_pose("W"), 0.0, "IDLE")
    core.on_order_board("RUN", [])
    core.traffic.on_dispatch(7, "OUTBOUND", "r1")
    core.tick()
    out = [json.loads(line) for line in lines if '"out": "MoveCmd"' in line]
    assert out and all("path" not in o for o in out)


def test_a_path_mode_record_line_carries_the_path():
    lines = []
    now = [0.0]
    core = FleetManager(load_lane_map(), load_params(), ["r1"], lambda: now[0])
    core.record_to(lines.append)
    for _ in range(3):
        core.on_robot_state("r1", *core.site.waypoint_pose("W"), 0.0, "IDLE")
    core.on_order_board("RUN", [])
    core.traffic.on_dispatch(7, "OUTBOUND", "r1")
    core.tick()
    out = [json.loads(line) for line in lines if '"out": "MoveCmd"' in line]
    assert out and len(out[0]["path"]) == 3


def test_move_cmd_keeps_its_old_positional_fields():
    cmd = MoveCmd("r1", 1, "A", 1.0, 2.0, 0.5)
    assert cmd.path == ()
