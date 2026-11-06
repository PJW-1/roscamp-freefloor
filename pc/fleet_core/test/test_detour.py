"""우회 규칙과 레인 막힘 2단계(지시서 5번). 특징 지도에서 본다."""
import math
import sys
from dataclasses import replace
from pathlib import Path

import pytest
from fleet_core.common.params import load_params
from fleet_core.traffic.controller import TrafficController
from fleet_core.traffic.lane_map import load_lane_map
from fleet_core.traffic.mission import AT_STATION, BLOCKED, MOVE, _Block, _Dispatch, _Goal
from sim_world import SimWorld

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))

import replay  # noqa: E402

FEATURE = REPO / "sim/fake_robots/maps/feature_map.yaml"
PARAMS = load_params()


def make(starts, **over):
    """starts: 로봇 -> 지점. 순서대로 tick에서 처리된다."""
    now = [0.0]
    site = load_lane_map(FEATURE)
    core = TrafficController(site, replace(PARAMS, **over), list(starts), lambda: now[0])
    for robot, waypoint in starts.items():
        core.on_robot_state(robot, *site.waypoint_pose(waypoint), 0.0, "IDLE")
    core.on_run_mode("RUN")
    return core, now


def codes(out, code):
    return [c for c in out if type(c).__name__ == "EventOut" and c.code == code]


def path(core, robot):
    m = core._missions[robot]
    return m.path[m.idx:]


def at_station(core, robot):
    m = core._missions[robot]
    m.phase, m.dest, m.hold = AT_STATION, m.path[0], True


def block(core, robot, a, b):
    """robot이 a->b 위에서 막혀 표시된 상태를 만든다. 양 끝 구역을 쥔다."""
    m = core._missions[robot]
    for waypoint in (a, b):
        core.zones.force_acquire(robot, core.site.zone_of[waypoint])
    m.block = _Block((a, b), marked=True)
    m.robot_state = BLOCKED


# ---- 값 ------------------------------------------------------------------------------------


def test_the_values():
    assert PARAMS.detour_enabled is True
    assert PARAMS.station_wait_estimate_s == 30.0
    assert PARAMS.detour_speed_mps == 0.2


def test_records_without_the_flag_replay_without_detours():
    # 옛 기록의 파라미터에는 이 값이 없다. 우회 전 동작으로 재생해야 한다
    fields = {f: getattr(PARAMS, f) for f in PARAMS.__dataclass_fields__
              if not f.startswith(("detour_", "station_wait"))}
    assert type(PARAMS)(**fields).detour_enabled is False


# ---- 비용 기반 경로 --------------------------------------------------------------------------


def test_the_cheapest_path_skips_a_lane_that_cannot_be_used():
    site = load_lane_map(FEATURE)
    cost, way = site.cheapest("F0", "W", lambda lane: lane.length)
    assert way == ["F0", "F1", "F2", "W"]
    shut = {("F1", "F2")}
    cost2, way2 = site.cheapest(
        "F0", "W", lambda lane: math.inf if (lane.a, lane.b) in shut else lane.length)
    assert way2 == ["F0", "T2", "T1", "QW", "W"]
    assert cost2 > cost


def test_no_path_is_none():
    site = load_lane_map(FEATURE)
    assert site.cheapest("O", "W", lambda lane: math.inf if lane.a == "O" else 1.0) is None


def test_plan_is_still_the_shortest_path():
    site = load_lane_map(FEATURE)
    assert site.plan("F0", "W") == site.cheapest("F0", "W", lambda lane: lane.length)[1]


# ---- 서 있는 로봇과 달리는 로봇 ----------------------------------------------------------------


def test_who_stands_and_for_how_long():
    core, now = make({"r1": "F0", "r2": "F1"})
    m = core._missions["r2"]
    m.goal = _Goal(MOVE, 9, 0)
    assert core._standing_wait_s(m) is None  # 달린다
    m.goal = None
    m.phase = AT_STATION
    assert core._standing_wait_s(m) == 30.0
    m.phase = "TO_WORK"
    core.zones.request("r2", "F2")  # 남이 없어 곧 받는다
    core.zones.release("r2", "F2")
    core.zones.force_acquire("r1", "F2")
    core.zones.request("r2", "F2")
    m.wait_zone, m.wait_since = "F2", 1.0
    now[0] = 4.0
    assert core._standing_wait_s(m) == pytest.approx(3.0)
    m.fault = "ERROR"
    assert core._standing_wait_s(m) == math.inf
    m.fault = ""
    m.comm = "COMM_LOST"
    assert core._standing_wait_s(m) == math.inf
    m.comm = ""
    m.block = _Block(("F1", "F2"))
    assert core._standing_wait_s(m) == math.inf


def test_a_robot_between_two_goals_on_its_way_runs():
    # 목표 하나를 막 끝내고 같은 tick에 다음 목표를 받을 로봇은 서 있지 않다
    core, _ = make({"r1": "F0", "r2": "F1"})
    m = core._missions["r2"]
    m.phase, m.path, m.idx = "TO_WORK", ["F1", "F2", "W"], 0
    assert core._standing_wait_s(m) is None
    m.phase = "DOCKING"
    m.path, m.idx = ["F1"], 0
    assert core._standing_wait_s(m) is None
    m.phase = "PARKED"
    assert core._standing_wait_s(m) == 30.0


def test_a_robot_waiting_for_its_cancel_result_stands_and_does_not_replan():
    # 파트 4: 표시 때 취소한 goal의 결과 전에는 단계가 없다. 우회도 다시 고르지 않는다
    core, _ = make({"r1": "F0", "r2": "F1"})
    m = core._missions["r2"]
    m.phase, m.path, m.idx, m.block_cancel = "TO_WORK", ["F1", "F2", "W"], 0, 41
    assert core._standing_wait_s(m) == 30.0
    core.on_robot_fault("r2", "x")
    m1 = core._missions["r1"]
    m1.phase, m1.dest, m1.path, m1.idx = "TO_WORK", "WORK", ["F0", "F1", "F2", "W"], 0
    m1.block_cancel = 42
    assert core._consider_detour(m1, "F1") is False
    m1.block_cancel = None
    assert core._consider_detour(m1, "F1") is True


def queued_behind_a_runner(queued_for):
    """A(r3)가 F1->F2 위 0.3 m 앞에서 달리고 B(r2)가 F1에서 그 뒤에 줄 서 BLOCKED를 보고한다."""
    core, now = make({"r1": "F0", "r2": "F1", "r3": "F2"})
    m3 = core._missions["r3"]
    m3.goal = _Goal(MOVE, 50, 0)
    (ax, ay), (bx, by) = core.site.waypoint_pose("F1")[:2], core.site.waypoint_pose("F2")[:2]
    k = 0.3 / math.hypot(bx - ax, by - ay)
    m3.x, m3.y = ax + k * (bx - ax), ay + k * (by - ay)
    m2 = core._missions["r2"]
    m2.phase, m2.path, m2.idx, m2.dest = "TO_WORK", ["F1", "F2", "W"], 0, "WORK"
    m2.goal, m2.robot_state, m2.queue_since = _Goal(MOVE, 51, 1), BLOCKED, 0.0
    now[0] = queued_for
    m1 = core._missions["r1"]
    m1.phase, m1.dest, m1.path, m1.idx = "TO_WORK", "WORK", ["F0", "F1", "F2", "W"], 0
    m1.dispatch = _Dispatch(3, "WORK")
    return core, m1, m2


def test_a_robot_queued_behind_a_runner_stands_for_the_time_it_has_queued():
    # 파트 4 줄 서기: 막힘 기록 없이 BLOCKED. 고장이 아니라 기다린 시간으로 친다
    core, m1, m2 = queued_behind_a_runner(2.0)
    assert core._standing_wait_s(m2) == pytest.approx(2.0)
    assert core._zone_penalty(m1, "F1", {})[0] == pytest.approx(2.0 * PARAMS.detour_speed_mps)
    assert core._consider_detour(m1, "F1") is False  # 갓 줄 섰다. 돌아가지 않는다
    assert path(core, "r1") == ["F0", "F1", "F2", "W"]
    m2.block = _Block(("F1", "F2"))
    assert core._standing_wait_s(m2) == math.inf  # 막힘 기록이 있으면 무한
    m2.block, m2.goal = None, None
    assert core._standing_wait_s(m2) == math.inf  # 목표 없이 BLOCKED도 무한


def test_a_queue_past_its_timeout_counts_as_blocked():
    # 줄 서기도 blocked_timeout_s를 넘기면 막힘 규칙을 쓴다(파트 4). 그 뒤의 시계는 낡았다
    core, _, m2 = queued_behind_a_runner(PARAMS.blocked_timeout_s + 1.0)
    assert core._standing_wait_s(m2) == math.inf


def test_a_queue_whose_front_robot_has_left_counts_as_blocked():
    # 앞 로봇이 떠났는데 다음 보고 전이라 시계가 남았다
    core, _, m2 = queued_behind_a_runner(2.0)
    m3 = core._missions["r3"]
    m3.x, m3.y = core.site.waypoint_pose("W")[:2]
    assert core._standing_wait_s(m2) == math.inf


def test_a_robot_that_waits_on_its_retry_stands_for_the_rest_of_it():
    core, now = make({"r1": "F0", "r2": "F1"})
    m = core._missions["r2"]
    m.phase, m.retry_at = "TO_WORK", 2.5
    now[0] = 1.0
    assert core._standing_wait_s(m) == pytest.approx(1.5)


# ---- 돌아가기 --------------------------------------------------------------------------------


def test_it_goes_around_a_broken_robot():
    core, _ = make({"r1": "F0", "r2": "F1"})
    core.on_robot_fault("r2", "x")
    core.on_dispatch(1, "WORK", "r1")
    out = core.tick()
    assert path(core, "r1") == ["F0", "T2", "T1", "QW", "W"]
    assert core.zones.holder("TUN") == "r1"
    (event,) = codes(out, "DETOUR")
    assert event.level == "INFO" and event.robot == "r1"
    assert "F1" in event.detail and "r2" in event.detail
    assert "F0 T2 T1 QW W" in event.detail


def test_it_goes_around_a_broken_robot_further_on():
    # 다음 구역은 비었지만 그 뒤 F2의 보유자가 고장이다. F1에 들어가면 돌 길이 없다
    core, _ = make({"r1": "F0", "r2": "F2"})
    core.on_robot_fault("r2", "x")
    core.on_dispatch(1, "WORK", "r1")
    out = core.tick()
    assert path(core, "r1") == ["F0", "T2", "T1", "QW", "W"]
    (event,) = codes(out, "DETOUR")
    assert "F2 held by r2" in event.detail


def test_a_broken_robot_further_on_with_no_way_around_is_waited_for():
    core, _ = make({"r1": "F1", "r2": "W"})
    core.on_robot_fault("r2", "x")
    core.on_dispatch(1, "WORK", "r1")
    out = core.tick()
    assert path(core, "r1") == ["F1", "F2", "W"]
    assert not codes(out, "DETOUR")


def test_a_standing_robot_further_on_is_not_worth_a_detour_yet():
    # 멀리 선 로봇은 도착할 때쯤 비킬 수 있다. 유한 벌점은 다음 구역만 본다
    core, _ = make({"r1": "F0", "r2": "F2"})
    at_station(core, "r2")
    core.on_dispatch(1, "WORK", "r1")
    out = core.tick()
    assert path(core, "r1") == ["F0", "F1", "F2", "W"]
    assert not codes(out, "DETOUR")


def test_with_detours_off_it_waits_in_line():
    core, _ = make({"r1": "F0", "r2": "F1"}, detour_enabled=False)
    core.on_robot_fault("r2", "x")
    core.on_dispatch(1, "WORK", "r1")
    out = core.tick()
    assert path(core, "r1") == ["F0", "F1", "F2", "W"]
    assert core.zones.waiting_for("r1") == "F1"
    assert not codes(out, "DETOUR")


def test_it_waits_when_there_is_no_other_way():
    core, _ = make({"r1": "O", "r2": "F0"})
    core.on_robot_fault("r2", "x")
    core.on_dispatch(1, "WORK", "r1")
    out = core.tick()
    assert path(core, "r1")[:2] == ["O", "F0"]
    assert core.zones.waiting_for("r1") == "F0"
    assert not codes(out, "DETOUR")


def test_it_does_not_go_around_through_the_waiting_cells():
    # F1에서 W로 돌아가는 길은 대기 줄(P-1..P-3)뿐이다. 줄은 우회로로 쓰지 않는다
    core, _ = make({"r1": "F1", "r2": "F2"})
    core.on_robot_fault("r2", "x")
    core.on_dispatch(1, "WORK", "r1")
    out = core.tick()
    assert path(core, "r1") == ["F1", "F2", "W"]
    assert core.zones.waiting_for("r1") == "F2"
    assert not codes(out, "DETOUR")


def test_a_running_holder_costs_nothing():
    core, _ = make({"r2": "F1", "r1": "F0"})  # r2가 먼저 목표를 받는다
    core.on_dispatch(1, "WORK", "r1")
    out = core.tick()
    assert core._missions["r2"].goal is not None
    assert path(core, "r1") == ["F0", "F1", "F2", "W"]
    assert not codes(out, "DETOUR")


def test_a_robot_at_a_station_is_worth_a_detour_by_its_estimate():
    core, _ = make({"r1": "F2", "r2": "W"})
    at_station(core, "r2")
    core.on_dispatch(5, "OUTBOUND", "r1")
    out = core.tick()
    assert path(core, "r1") == ["F2", "T1", "T2", "QO", "O"]
    assert len(codes(out, "DETOUR")) == 1


def test_a_short_estimate_is_not_worth_it():
    # 0.2 m 벌점은 돌아가는 길(약 1 m 더)보다 작다
    core, _ = make({"r1": "F2", "r2": "W"}, station_wait_estimate_s=1.0)
    at_station(core, "r2")
    core.on_dispatch(5, "OUTBOUND", "r1")
    out = core.tick()
    assert path(core, "r1")[:2] == ["F2", "W"]
    assert core.zones.waiting_for("r1") == "W"
    assert not codes(out, "DETOUR")


@pytest.mark.parametrize("waited, goes_around", [(1.0, False), (20.0, True)])
def test_a_waiting_holder_costs_the_time_it_has_waited(waited, goes_around):
    # 돌아가는 길은 약 1.65 m 더 길다. 0.2 m/s로 8초 넘게 기다린 보유자면 돌아간다
    core, now = make({"r1": "F0", "r2": "F1", "r3": "F2"})
    core._missions["r3"].goal = _Goal(MOVE, 50, 0)  # r3은 달린다. F2 벌점이 없다
    m2 = core._missions["r2"]
    m2.phase, m2.path, m2.idx, m2.dest = "TO_WORK", ["F1", "F2", "W"], 0, "WORK"
    m2.dispatch = _Dispatch(77, "WORK")  # 호출 재계산이 길을 바꾸지 않게
    core.zones.request("r2", "F2")
    m2.wait_zone, m2.wait_since = "F2", 0.0
    core.on_dispatch(1, "WORK", "r1")
    now[0] = waited
    for robot, waypoint in (("r1", "F0"), ("r2", "F1"), ("r3", "F2")):
        core.on_robot_state(robot, *core.site.waypoint_pose(waypoint), 0.0, "IDLE")
    out = core.tick()
    detours = [e for e in codes(out, "DETOUR") if e.robot == "r1"]
    assert bool(detours) == goes_around
    assert (path(core, "r1")[1] == "T2") == goes_around


def test_a_blocked_lane_further_on_is_avoided_now():
    core, _ = make({"r1": "O", "r2": "F1"})
    block(core, "r2", "F1", "F2")
    core.on_dispatch(1, "WORK", "r1")
    out = core.tick()
    assert path(core, "r1") == ["O", "F0", "T2", "T1", "QW", "W"]
    (event,) = codes(out, "DETOUR")
    assert "F1 to F2" in event.detail
    assert core.zones.holder("F0") == "r1"


def test_a_blocked_two_way_lane_is_shut_both_ways():
    core, _ = make({"r1": "F2", "r2": "T1", "r3": "W"})
    block(core, "r2", "T2", "T1")
    at_station(core, "r3")
    core.on_dispatch(5, "OUTBOUND", "r1")
    out = core.tick()
    # T1->T2도 막혔으니 W를 지나는 길밖에 없다. 기다린다
    assert path(core, "r1")[:2] == ["F2", "W"]
    assert not codes(out, "DETOUR")


def test_it_does_not_turn_back_at_the_same_waypoint():
    core, _ = make({"r1": "F2", "r2": "W", "r3": "T1"})
    at_station(core, "r2")
    core.on_dispatch(5, "OUTBOUND", "r1")
    core.tick()  # r3은 아직 목표가 없어 서 있다
    out = core.tick()
    assert path(core, "r1") == ["F2", "T1", "T2", "QO", "O"]
    assert len(codes(out, "DETOUR")) == 1
    assert core.zones.waiting_for("r1") == "TUN"  # r3이 아직 TUN을 쥐었다
    core.on_release("r2")
    out = core.tick()
    assert core._missions["r2"].goal is not None  # 이제 달린다. W 벌점이 없다
    out += core.tick()
    assert path(core, "r1") == ["F2", "T1", "T2", "QO", "O"]
    assert not codes(out, "DETOUR")


def test_it_turns_back_when_the_new_way_can_no_longer_be_passed():
    # 왕복 금지는 지금 길을 못 지날 때는 쉰다. 격리된 보유자 뒤에서 끝없이 서지 않는다
    core, _ = make({"r1": "F2", "r2": "W", "r3": "T1"})
    at_station(core, "r2")
    core.on_dispatch(5, "OUTBOUND", "r1")
    core.tick()
    core.tick()
    assert path(core, "r1") == ["F2", "T1", "T2", "QO", "O"]
    assert core.zones.waiting_for("r1") == "TUN"
    core.on_release("r2")
    core.tick()
    assert core._missions["r2"].goal is not None  # W 보유자는 달린다
    core.on_robot_fault("r3", "x")  # TUN 보유자가 고장이다. 못 지난다
    out = core.tick()
    assert path(core, "r1")[:2] == ["F2", "W"]
    (event,) = [e for e in codes(out, "DETOUR") if e.robot == "r1"]
    assert "TUN held by r3" in event.detail
    assert core.zones.waiting_for("r1") in (None, "W")


def test_a_zone_given_up_again_at_the_same_waypoint_is_listed_once():
    # 전수 검사에서 찾은 경우: 다른 규칙이 같은 지점에서 옛 길을 다시 주면 목록이 끝없이 길어졌다
    core, _ = make({"r1": "F2", "r2": "W"})
    at_station(core, "r2")
    core.on_dispatch(5, "OUTBOUND", "r1")
    core.tick()
    m = core._missions["r1"]
    assert m.detour_avoid == ("W",)
    for _ in range(3):
        m.path, m.idx, m.goal = core.site.plan("F2", "O"), 0, None
        assert core._consider_detour(m, "W") is True
        assert path(core, "r1") == ["F2", "T1", "T2", "QO", "O"]
    assert m.detour_avoid == ("W",)


def test_a_zone_given_up_further_on_does_not_make_the_path_impassable():
    # 피한 구역이 지금 길 뒤쪽에 있어도 지금 길의 비용은 실제 비용이다(0.2 m 벌점)
    core, _ = make({"r1": "F0", "r2": "F1"}, station_wait_estimate_s=1.0)
    at_station(core, "r2")
    m = core._missions["r1"]
    m.phase, m.dest, m.dispatch = "TO_WORK", "WORK", _Dispatch(3, "WORK")
    m.path, m.idx = ["F0", "F1", "F2", "W"], 0
    m.detour_at, m.detour_avoid = "F0", ("F2",)
    assert core._consider_detour(m, "F1") is False  # 돌아가는 길은 약 1.65 m 더 길다
    assert path(core, "r1") == ["F0", "F1", "F2", "W"]


def test_a_robot_that_holds_its_next_zone_keeps_its_path():
    core, _ = make({"r1": "F0", "r2": "F2"})
    core.on_dispatch(1, "WORK", "r1")
    core.zones.force_acquire("r1", "F1")  # 다음 구역을 이미 쥐었다
    core.on_robot_fault("r2", "x")
    out = core.tick()
    assert path(core, "r1") == ["F0", "F1", "F2", "W"]
    assert not codes(out, "DETOUR")


def test_a_new_path_that_never_leaves_the_zone_requests_nothing():
    # 전수 검사에서 찾은 경우: 도크에 선 채 한 바퀴 도는 길이 남았는데 그 길이 막혔다
    core, _ = make({"r1": "W", "r2": "E1"})
    block(core, "r2", "E1", "E2")
    m = core._missions["r1"]
    m.phase, m.dest, m.dispatch = "TO_WORK", "WORK", _Dispatch(3, "WORK")
    m.path, m.idx = ["W", "E0", "E1", "E2", "O", "F0", "F1", "F2", "W"], 0
    out = core.tick()
    assert len(codes(out, "DETOUR")) == 1
    assert core.zones.waiting_for("r1") is None
    assert core._missions["r1"].phase == "DOCKING"


def test_the_same_inputs_give_the_same_detour():
    runs = []
    for _ in range(2):
        core, _ = make({"r1": "F0", "r2": "F1"})
        core.on_robot_fault("r2", "x")
        core.on_dispatch(1, "WORK", "r1")
        runs.append([repr(c) for c in core.tick()])
    assert runs[0] == runs[1]


# ---- 가상 창고와 기록 재생 ----------------------------------------------------------------------


def feature_world(detour, record=None, at_s=42.0, duration_s=15.0):
    """특징 지도에서 pinky1이 서쪽행 레인에서 막힌다. 기본 시각은 막힌 레인 우회가 나는 것."""
    robots = {"pinky1": {"start": "P-3", "speed_mps": 0.20, "pose_noise_m": 0.03},
              "pinky2": {"start": "P-2", "speed_mps": 0.18, "pose_noise_m": 0.03},
              "pinky3": {"start": "P-1", "speed_mps": 0.20, "pose_noise_m": 0.03}}
    world = SimWorld(site=load_lane_map(FEATURE), robots=robots,
                     params=replace(PARAMS, detour_enabled=detour), arm_s_per_item=4.0,
                     swap_step_s=4.0, dock_s=3.0,
                     faults=[{"at_s": at_s, "robot": "pinky1", "type": "blocked",
                              "duration_s": duration_s}])
    if record is not None:
        world.core.record_to(record.append)
    world.run_until(world.done, timeout_s=900)
    return world


def test_in_the_warehouse_a_robot_goes_around_the_blocked_lane_and_it_replays(tmp_path):
    lines = []
    world = feature_world(True, lines)
    detours = [e for e in world.events if e.code == "DETOUR"]
    assert detours and "F1 to F2 blocked" in detours[0].detail
    assert not [e for e in world.events if e.level == "ERROR"]
    path = tmp_path / "run.jsonl"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    result = replay.replay(path)
    assert result.same and result.compared, result


def test_with_the_scenario_times_it_goes_around_the_robot_behind_the_block():
    # feature_blocked_lane.yaml의 시각(46초, 30초). 막힌 레인 뒤에서 기다리는 로봇을 피한다.
    # ROS 시나리오에서도 같은 우회가 났다. 막힌 레인 자체의 우회는 위 시험(42초, 15초)이 본다
    world = feature_world(True, at_s=46.0, duration_s=30.0)
    detours = [e for e in world.events if e.code == "DETOUR"]
    assert detours and "F1 held by pinky2" in detours[0].detail
    assert not [e for e in world.events if e.level == "ERROR"]


def test_in_the_warehouse_without_detours_it_waits_and_still_finishes():
    world = feature_world(False)
    assert not [e for e in world.events if e.code == "DETOUR"]
    assert not [e for e in world.events if e.level == "ERROR"]
