"""레인 폴리라인 목표(지시서 7번, FollowPath). point 모드는 비교용으로 남는다.

path 모드: 잠금을 쥔 데까지 방향이 바뀌어도 한 목표로 묶고, MoveCmd.path에 선을 싣는다.
첫 점은 로봇 위치, 끝 점은 목표 지점이다.
"""
import json
import math
from dataclasses import asdict, replace
from pathlib import Path

import pytest
import yaml
from core_helpers import cmds, codes
from fleet_core.common.commands import MoveCmd
from fleet_core.common.params import FleetParams, load_params
from fleet_core.manager import FleetManager
from fleet_core.traffic.controller import TrafficController
from fleet_core.traffic.lane_map import LaneMap, load_lane_map
from fleet_core.traffic.mission import MOVE, _Goal

REPO = Path(__file__).resolve().parents[3]


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


@pytest.mark.parametrize("a, b, along, expect", [
    ("F0", "T2", 0.25, "T2"), ("F0", "T2", 0.78, "T2"), ("T2", "T1", 0.25, "T1")])
def test_an_acute_turn_is_not_passed_before_the_robot_gets_round_it(a, b, along, expect):
    """F0→T2→T1은 예각이다. F0→T2 위의 로봇도 T2→T1에 투영하면 0.2 m 넘게 앞서 보인다."""
    feature = load_lane_map(REPO / "sim/fake_robots/maps/feature_map.yaml")
    core = TrafficController(feature, replace(load_params(), lane_goal_mode="path"), ["r1"],
                             lambda: 0.0)
    m = core._missions["r1"]
    m.path, m.idx = ["O", "F0", "T2", "T1", "QW"], 1
    m.goal = _Goal(MOVE, 1, 4)
    lane, (x, y) = feature.lane(a, b), feature.waypoint_pose(a)
    m.x, m.y = x + lane.ux * along, y + lane.uy * along
    core._track_progress(m)
    assert m.path[m.idx] == expect


@pytest.mark.parametrize("a, b, along, expect", [
    ("F0", "T2", 0.25, ("F0", "T2")), ("F0", "T2", 0.5, ("F0", "T2")),
    ("T2", "T1", 0.25, ("T2", "T1"))])
def test_the_lane_error_is_measured_on_the_lane_before_an_acute_turn(a, b, along, expect):
    """이탈 기준 레인(파트 3)도 예각에서 다음 레인으로 앞서 넘어가면 안 된다. 넘어가면 멈춘다."""
    feature = load_lane_map(REPO / "sim/fake_robots/maps/feature_map.yaml")
    core = TrafficController(feature, replace(load_params(), lane_goal_mode="path"), ["r1"],
                             lambda: 0.0)
    m = core._missions["r1"]
    m.path, m.idx = ["O", "F0", "T2", "T1", "QW"], 2
    m.goal = _Goal(MOVE, 1, 4)
    lane, (x, y) = feature.lane(a, b), feature.waypoint_pose(a)
    m.x, m.y = x + lane.ux * along, y + lane.uy * along
    assert core._error_lane(m) == expect


def test_a_report_pushed_sideways_before_a_turn_is_measured_on_the_lane_it_is_on():
    """보고가 진행 방향 왼쪽으로 0.3 m 밀렸다(파트 3 drift, e2e 시험의 자리). 꺾인 다음 레인으로
    기준을 옮기면 돌아온 보고를 그 레인에서 재게 되어 끝내 레인에 돌아오지 못한다."""
    from sim_world import SimWorld
    site = SimWorld(seed=7).site
    core = TrafficController(site, replace(load_params(), lane_goal_mode="path"), ["r1"],
                             lambda: 0.0)
    m = core._missions["r1"]
    m.path, m.idx = [(4, 3), (5, 3), (6, 3), (6, 4), (6, 5)], 1
    m.goal = _Goal(MOVE, 1, 4)
    lane = site.lane((5, 3), (6, 3))
    m.x, m.y = lane.ax + 0.09, lane.ay + 0.3
    assert core._error_lane(m) in (((4, 3), (5, 3)), ((5, 3), (6, 3)))


@pytest.mark.parametrize("before, watched", [(0.14, ("P-6", "N")), (0.0, ("N", "S"))])
def test_a_goal_starting_short_of_its_first_waypoint_is_measured_on_the_lane_in(before, watched):
    """FMS 재시작(test_fms_restart): 로봇이 P-6→N 위 N 앞 0.14 m에 섰고 새 경로는 N부터다.
    path 목표는 N→S까지 묶이므로 N에 닿기 전에 N→S로 재면 0.26 m 이탈로 세운다.
    닿기 전에는 실제로 있는 들어오는 레인으로 잰다. N에 선 로봇은 첫 레인으로 잰다."""
    core, _ = make({"r1": "W"})
    m = core._missions["r1"]
    m.path, m.idx = ["N", "S", "L"], 0
    m.goal = _Goal(MOVE, 1, 2)
    lane, (x, y) = core.site.lane("P-6", "N"), xy(core, "N")
    m.x, m.y = x - lane.ux * before, y - lane.uy * before
    assert core._error_lane(m) == watched


def on_the_lane_into_t2(core, short):
    """특징 지도: 경로는 T2부터(T2 T1 QW), 로봇은 들어오는 레인 F0→T2 위 T2 앞 short m.

    F0→T2→T1은 예각이라 T2 앞의 로봇도 T2→T1에 투영하면 앞서 보인다(Codex 2, 3).
    """
    m = core._missions["r1"]
    m.path, m.idx = ["T2", "T1", "QW"], 0
    m.goal = _Goal(MOVE, 1, 2)
    lane, (x, y) = core.site.lane("F0", "T2"), core.site.waypoint_pose("T2")
    m.x, m.y = x - lane.ux * short, y - lane.uy * short
    return m


def feature_core():
    feature = load_lane_map(REPO / "sim/fake_robots/maps/feature_map.yaml")
    return TrafficController(feature, replace(load_params(), lane_goal_mode="path"), ["r1"],
                             lambda: 0.0)


def test_the_first_waypoint_not_reached_yet_stays_in_the_line():
    core = feature_core()
    m = on_the_lane_into_t2(core, 0.3)
    s, _ = core.site.lane_progress("T2", "T1", m.x, m.y)
    assert s > core.params.arrival_tolerance_m, "the case: it looks past T2 on T2 to T1"
    assert core._first_ahead(m, 2) == 0, "T2 stays, the corner is not cut"


def test_the_path_index_does_not_pass_a_first_waypoint_not_reached_yet():
    core = feature_core()
    m = on_the_lane_into_t2(core, 0.5)
    s, _ = core.site.lane_progress("T2", "T1", m.x, m.y)
    assert s > core.params.release_distance_m
    core._track_progress(m)
    assert m.idx == 0


def test_short_of_the_first_waypoint_the_lane_error_is_measured_on_the_lane_in():
    core = feature_core()
    m = on_the_lane_into_t2(core, 0.3)
    s, _ = core.site.lane_progress("T2", "T1", m.x, m.y)
    assert s >= 0
    assert core._error_lane(m) == ("F0", "T2")


def watched_off(core, m):
    """보고를 확인 횟수만큼 넣고 이탈로 세웠는지."""
    for _ in range(core.params.release_confirm_samples):
        core._watch_lane_error(m)
    return m.off_lane_warned


def test_drifting_off_the_lane_in_before_the_first_waypoint_is_caught():
    """첫 지점 앞이라고 면제하지 않는다. 진입 레인 옆 0.3 m는 이탈이다(Codex P1)."""
    core = feature_core()
    m = on_the_lane_into_t2(core, 0.3)
    lane = core.site.lane("F0", "T2")
    m.x, m.y = m.x - lane.uy * 0.3, m.y + lane.ux * 0.3  # T1 반대쪽으로
    assert core.site.lane_progress("T2", "T1", m.x, m.y)[0] < 0, "the case: short of T2"
    assert core._error_lane(m) == ("F0", "T2")
    assert watched_off(core, m)


def test_a_normal_approach_on_the_lane_in_is_not_caught():
    core = feature_core()
    m = on_the_lane_into_t2(core, 0.3)
    assert not watched_off(core, m), "T2 to T1 would read 0.25 m off"


def test_past_the_first_waypoint_the_lane_error_keeps_its_rule():
    core = feature_core()
    m = on_the_lane_into_t2(core, 0.0)
    out = core.site.lane("T2", "T1")
    m.x, m.y = m.x + out.ux * 0.5 - out.uy * 0.3, m.y + out.uy * 0.5 + out.ux * 0.3
    assert core._error_lane(m) == ("T2", "T1")
    assert watched_off(core, m)


def test_the_first_path_after_a_reinstate_keeps_a_waypoint_before_a_150_degree_turn():
    """재투입 직후 놓인 자리부터 만든 첫 path: 첫 지점 앞 0.19 m, 약 150도 꺾임(Codex P2).

    F0를 옮겨 F0→T2→T1을 150도로 만든다. T2→T1에 투영하면 0.16 m 앞서 보인다.
    """
    data = yaml.safe_load((REPO / "sim/fake_robots/maps/feature_map.yaml").read_text())
    for wp in data["waypoints"]:
        if wp["id"] == "F0":
            wp["x"], wp["y"] = 1.399, 0.928
    site = LaneMap(data, yaml.safe_dump(data))
    a, b = site.lane("F0", "T2"), site.lane("T2", "T1")
    assert math.degrees(math.acos(a.ux * b.ux + a.uy * b.uy)) == pytest.approx(150, abs=0.1)
    core = TrafficController(site, replace(load_params(), lane_goal_mode="path"), ["r1"],
                             lambda: 0.0)
    m = on_the_lane_into_t2(core, 0.19)
    assert core.site.lane_progress("T2", "T1", m.x, m.y)[0] > core.params.arrival_tolerance_m
    assert core._first_ahead(m, 2) == 0


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
