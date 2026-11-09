"""자유바닥 접근선 시간창(floor_dock_schedule): ETA로 차례를 정하고, 비울 때에 맞춰 미리 출발."""
import math
import sys
from dataclasses import replace
from pathlib import Path

import pytest
from fleet_core.common.params import load_params
from fleet_core.manager import FleetManager
from fleet_core.traffic.deadlock import find_cycle, wait_graph
from fleet_core.traffic.floor.cells import disk_cells
from fleet_core.traffic.floor.controller import REST_RELEASE_SAMPLES
from fleet_core.traffic.lane_map import load_lane_map
from fleet_core.traffic.mission import MOVE, _Goal

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
import compare_floor  # noqa: E402


def floor(names_xy, schedule=True):
    params = replace(load_params(), traffic_mode="freefloor", floor_dock_schedule=schedule)
    now = [0.0]
    core = FleetManager(load_lane_map(), params, list(names_xy), lambda: now[0])
    for _ in range(3):
        for name, (x, y) in names_xy.items():
            core.on_robot_state(name, x, y, 0.0, "IDLE")
    traffic = core.traffic
    traffic._plan_budget = 99

    def at(t):  # 교통은 매니저가 tick마다 굳힌 시각을 본다
        now[0] = core._time = t

    return traffic, at


def line_cells(traffic, line="work_in"):
    return traffic._line_ids[line]


def test_the_next_robot_waits_while_the_line_is_long_from_free(monkeypatch):
    traffic, _ = floor({"a": (1.0, 1.15), "b": (2.6, 0.8)})
    a, b = traffic._missions["a"], traffic._missions["b"]
    traffic._start_leg(a, "TO_WORK", None, "W")
    del traffic._legs["a"]  # a는 도크에 섰다: 가는 다리가 없어 빌 시각만 본다
    a.phase = "AT_STATION"
    monkeypatch.setattr(traffic, "line_free_in", lambda line, exclude=None: 30.0)
    traffic._start_leg(b, "TO_WORK", None, "W")
    assert traffic._legs["b"].waiting
    monkeypatch.setattr(traffic, "line_free_in", lambda line, exclude=None: 1.0)
    traffic._drive(b)
    assert not traffic._legs["b"].waiting


def test_the_next_robot_sets_off_when_the_line_frees_by_its_arrival(monkeypatch):
    traffic, _ = floor({"a": (1.0, 1.15), "b": (2.6, 0.8)})
    a, b = traffic._missions["a"], traffic._missions["b"]
    traffic._start_leg(a, "TO_WORK", None, "W")
    del traffic._legs["a"]  # a는 도크에 섰다(다리 없음, 선 칸을 쥠)
    a.phase = "AT_STATION"
    held = line_cells(traffic) & traffic.zones.held_set("a")
    assert held
    monkeypatch.setattr(traffic, "line_free_in", lambda line, exclude=None: 1.0)
    traffic._start_leg(b, "TO_WORK", None, "W")
    leg = traffic._legs["b"]
    assert not leg.waiting and leg.admitted
    assert [f[2] for f in leg.route.fixed] == ["work_in"]
    traffic._extend(b, leg)  # 선 칸은 a가 쥐고 있어 선 앞에서 멈춘다
    assert not line_cells(traffic) & traffic.zones.held_set("b")


@pytest.mark.parametrize("dx,dy", [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)])
def test_a_robot_resting_at_the_dock_still_holds_its_line(dx, dy):
    """도크에 서서 쉬는 로봇은 몸 밖 칸을 0.2 m 안이라도 놓는다(쉬는 로봇 규칙). 도착 허용만큼
    비켜 서서 몸 칸만 남아도 접근선 칸에 닿아, 선이 빈 것(None)으로 보이지 않는다."""
    traffic, at = floor({"a": (0.65, 1.15), "b": (2.6, 0.8)})
    core = traffic.station_left.__self__  # 매니저(보고 입구)
    a = traffic._missions["a"]
    traffic._start_leg(a, "TO_WORK", None, "W")
    tol = traffic.params.arrival_tolerance_m
    wx, wy = traffic.site.waypoint_pose("W")[:2]
    x, y = wx + dx * tol, wy + dy * tol
    del traffic._legs["a"]
    traffic._active["a"].clear()
    a.goal, a.phase = None, "AT_STATION"
    t = [0.0]

    def report(n):
        for _ in range(n):
            t[0] += 1.0
            at(t[0])
            core.on_robot_state("a", x, y, 0.0, "IDLE")
            traffic.tick_progress(t[0])

    report(traffic.params.release_confirm_samples + 1)  # 0.2 m 밖은 보통 규칙으로 놓음
    assert traffic._at_rest(a)
    # 몸 밖이지만 0.2 m 안의 칸: 쉬는 로봇 규칙만 놓는다
    body, d = traffic._body, traffic.params.release_distance_m
    band = {c for c in traffic._ids(disk_cells(traffic.floor, x, y, d + 0.05))
            if body + 0.01 < traffic.floor.box_distance(traffic.floor.cell_at(c), x, y) < d - 0.01}
    band -= traffic.zones.held_set("b")
    assert band and traffic.zones.acquire_all("a", band | traffic.zones.held_set("a"))
    report(REST_RELEASE_SAMPLES - 1)
    assert band <= traffic.zones.held_set("a")
    report(1)
    held = traffic.zones.held_set("a")
    assert not band & held
    assert held <= traffic._body_cells(x, y)
    assert held & line_cells(traffic)
    assert traffic.line_free_in("work_in") is not None


def _sent_early_at_the_line(monkeypatch, short=0.05, arrived=True):
    """a는 도크에 서 있고(선 칸을 쥠), b는 미리 출발해 설 자리 short m 앞까지 왔다.
    arrived: 설 자리로 가는 MoveTo가 끝났다."""
    traffic, at = floor({"a": (1.0, 1.15), "b": (2.6, 0.8)})
    a, b = traffic._missions["a"], traffic._missions["b"]
    traffic._start_leg(a, "TO_WORK", None, "W")
    del traffic._legs["a"]
    a.phase = "AT_STATION"
    monkeypatch.setattr(traffic, "line_free_in", lambda line, exclude=None: 1.0)
    traffic._start_leg(b, "TO_WORK", None, "W")
    leg = traffic._legs["b"]
    cap = traffic._line_cap(b, leg)
    for cell in list(traffic.zones.held_set("b")):
        traffic.zones.release("b", cell)
    leg.s = leg.s_sent = cap - short
    b.x, b.y = leg.route.at(leg.s)
    leg.s_lock = cap
    assert traffic.zones.acquire_all("b", traffic._body_cells(b.x, b.y)
                                     | traffic._swept(leg.route.between(leg.s, cap)))
    b.goal = None if arrived else _Goal(MOVE, 99, 0)  # 설 자리로 가는 MoveTo
    return traffic, at, leg


def _parking_cells(traffic):
    """주차하는 로봇이 설 몸 칸."""
    return set().union(*(traffic._body_cells(*traffic.site.waypoint_pose(p))
                         for p in traffic.site.parking))


def test_a_robot_sent_early_stops_off_other_lines_and_parking_spots(monkeypatch):
    """제 접근선에만 속한 칸(선 시작 쪽)에는 서도 된다. 출구선 등 다른 고정 선 칸을 쥐고 서면
    떠나는 로봇의 길을 막고, 주차 지점 칸을 쥐고 서면 그리로 가는 로봇이 매 tick 다시 출발한다."""
    traffic, _, leg = _sent_early_at_the_line(monkeypatch)
    s0 = leg.route.fixed[0][0]
    assert s0 - 0.6 < leg.s_lock <= s0
    standing = traffic._standing_cells(*leg.route.at(leg.s_lock))
    others = set().union(*(ids for line, ids in traffic._line_ids.items() if line != "work_in"))
    assert standing.isdisjoint(others)
    assert standing.isdisjoint(_parking_cells(traffic))
    start = traffic.floor.fixed_lines["work_in"].points[0]
    assert math.dist(leg.route.at(leg.s_lock), start) <= 1.2


def test_a_robot_sent_early_does_not_queue_on_the_line_on_its_way(monkeypatch):
    """가는 동안 줄을 서면 떠나는 로봇과 대기 순환이 된다(지나가며 출구선 칸을 쥐고 있을 때)."""
    traffic, _, leg = _sent_early_at_the_line(monkeypatch, short=0.4, arrived=False)
    cap = leg.s_lock
    traffic._extend(traffic._missions["b"], leg)
    assert leg.s_lock == pytest.approx(cap)
    assert not set(traffic.zones.waited_cells("b")) & line_cells(traffic)
    assert "b" not in wait_graph(traffic.zones)


def test_a_robot_sent_early_queues_on_the_line_once_it_stands_at_its_stop(monkeypatch):
    """서 있는 동안은 줄을 선다: 떠나는 로봇이 이 로봇의 칸을 기다리면 순환이 보이고 교착 처리가
    맡는다(보이지 않는 교착이 없다)."""
    traffic, _, leg = _sent_early_at_the_line(monkeypatch)
    cap = leg.s_lock
    traffic._extend(traffic._missions["b"], leg)
    assert leg.s_lock == pytest.approx(cap)
    assert set(traffic.zones.waited_cells("b")) & line_cells(traffic)
    assert "a" in wait_graph(traffic.zones)["b"]
    # a가 떠나며 b의 칸을 기다리면 순환이 보인다
    assert not traffic.zones.acquire_all("a", set(traffic.zones.held_set("b")))
    assert set(find_cycle(wait_graph(traffic.zones))) == {"a", "b"}


def test_a_robot_standing_at_its_stop_stays_queued_through_a_whole_drive_tick(monkeypatch):
    """_drive 한 번 전체에서도 줄이 남는다: 몇 cm 못 미쳐 섰다고 다시 보내는 MoveTo가
    대기 기록을 지우면 순환이 다시 안 보인다."""
    traffic, _, leg = _sent_early_at_the_line(monkeypatch)
    b = traffic._missions["b"]
    traffic._drive(b)
    assert set(traffic.zones.waited_cells("b")) & line_cells(traffic)
    assert b.goal is None
    assert not traffic.zones.acquire_all("a", set(traffic.zones.held_set("b")))
    assert set(find_cycle(wait_graph(traffic.zones))) == {"a", "b"}
    traffic.take_output()
    traffic._check_deadlock(1.0)  # 다음 tick의 교착 검사가 본다
    codes = [c.code for c in traffic.take_output() if type(c).__name__ == "EventOut"]
    assert "DEADLOCK_DETECTED" in codes


def test_a_robot_sent_early_does_not_queue_until_its_move_to_the_stop_ends(monkeypatch):
    """설 자리에 거의 왔어도 아직 달리는 중이면 줄을 서지 않는다(지나온 칸을 아직 쥘 수 있다)."""
    traffic, _, leg = _sent_early_at_the_line(monkeypatch, arrived=False)
    traffic._extend(traffic._missions["b"], leg)
    assert not set(traffic.zones.waited_cells("b")) & line_cells(traffic)


def test_a_robot_that_stopped_short_still_queues(monkeypatch):
    """조금 못 미쳐 서서 다른 선 칸에 닿아 있어도 줄을 선다: 안 서면 떠나는 로봇이 그 칸을
    기다릴 때 순환이 안 보인다."""
    traffic, _, leg = _sent_early_at_the_line(monkeypatch)
    only_out = traffic._line_ids["work_out"] - traffic._line_ids["work_in"]
    cell = next(c for c in sorted(only_out) if traffic.zones.holder(c) is None)
    assert traffic.zones.acquire_all("b", {cell})
    traffic._extend(traffic._missions["b"], leg)
    assert set(traffic.zones.waited_cells("b")) & line_cells(traffic)


def test_a_robot_sent_early_waits_at_the_line_start_without_replanning(monkeypatch):
    traffic, at, leg = _sent_early_at_the_line(monkeypatch)
    b = traffic._missions["b"]
    route = leg.route
    for k in range(1, 40):
        at(0.5 * k)
        traffic._drive(b)
    assert leg.route is route and not leg.waiting


def test_a_route_with_no_stop_near_the_line_waits_at_a_wait_spot_instead(monkeypatch):
    """선 시작 가까이 다른 선·주차 지점에 닿지 않고 설 곳이 없는 길: 미리 출발하지 않는다."""
    traffic, _ = floor({"a": (1.0, 1.15), "b": (2.6, 0.8)})
    a, b = traffic._missions["a"], traffic._missions["b"]
    traffic._start_leg(a, "TO_WORK", None, "W")
    del traffic._legs["a"]
    a.phase = "AT_STATION"
    free = [1.0]
    monkeypatch.setattr(traffic, "line_free_in", lambda line, exclude=None: free[0])
    monkeypatch.setattr(traffic, "_stop_before_line", lambda leg: None)
    traffic._start_leg(b, "TO_WORK", None, "W")
    leg = traffic._legs["b"]
    assert leg.waiting and not leg.admitted
    spot = traffic._standing_cells(*leg.route.points[-1])
    assert spot.isdisjoint(traffic._all_line_ids)
    traffic._drive(b)  # 다시 미리 출발시키지 않는다(매 tick 계획을 오가지 않게)
    assert traffic._legs["b"].waiting
    for cell in list(traffic.zones.held_set("a")):
        traffic.zones.release("a", cell)
    free[0] = None  # 선이 비면 보통처럼 들어간다
    traffic._drive(b)
    assert not traffic._legs["b"].waiting and traffic._legs["b"].admitted


def test_a_robot_sent_early_goes_to_a_wait_spot_when_the_holder_is_isolated(monkeypatch):
    """선을 쥔 로봇이 격리되면 언제 빌지 모른다: 선 앞에 서 있지 않고 대기 자리로."""
    traffic, _, leg = _sent_early_at_the_line(monkeypatch)
    monkeypatch.undo()
    traffic._missions["a"].fault = "ERROR"
    traffic._drive(traffic._missions["b"])
    leg = traffic._legs["b"]
    assert leg.waiting and not leg.admitted


def test_only_one_robot_holds_the_turn_while_its_plan_waits_for_budget():
    traffic, _ = floor({"a": (2.6, 0.8), "b": (1.6, 0.8)})
    a, b = traffic._missions["a"], traffic._missions["b"]
    traffic._start_leg(a, "TO_WORK", None, "W")
    assert traffic._legs["a"].admitted
    traffic._plan_budget = 0
    traffic._plan(a, traffic._legs["a"])  # 다음 tick에 계획: 그동안 waiting
    assert traffic._legs["a"].waiting
    traffic._plan_budget = 99
    traffic._start_leg(b, "TO_WORK", None, "W")
    assert traffic._legs["b"].waiting


def test_a_robot_sent_early_enters_the_line_once_it_frees(monkeypatch):
    traffic, _, leg = _sent_early_at_the_line(monkeypatch)
    b = traffic._missions["b"]
    traffic._drive(b)  # 서서 줄을 섰다
    for cell in list(traffic.zones.held_set("a")):
        traffic.zones.release("a", cell)
    traffic.take_output()
    traffic._drive(b)
    assert leg.s_lock == pytest.approx(leg.route.fixed[0][1])
    assert line_cells(traffic) <= traffic.zones.held_set("b")
    assert b.goal is not None  # 선 끝(도크)까지 MoveTo를 보냈다
    moves = [c for c in traffic.take_output() if type(c).__name__ == "MoveCmd"]
    assert moves and moves[-1].robot == "b"


def test_a_robot_on_the_line_that_may_stay_long_keeps_the_next_one_waiting(monkeypatch):
    traffic, _ = floor({"a": (1.0, 1.15), "b": (2.6, 0.8)})
    monkeypatch.setattr(traffic, "line_free_in", lambda line, exclude=None: math.inf)
    traffic._start_leg(traffic._missions["b"], "TO_WORK", None, "W")
    assert traffic._legs["b"].waiting


def test_an_admitted_robot_keeps_its_turn_when_the_line_frees_late(monkeypatch):
    traffic, _ = floor({"a": (1.0, 1.15), "b": (2.6, 0.8)})
    free = [1.0]
    monkeypatch.setattr(traffic, "line_free_in", lambda line, exclude=None: free[0])
    b = traffic._missions["b"]
    traffic._start_leg(b, "TO_WORK", None, "W")
    assert traffic._legs["b"].admitted
    free[0] = 20.0  # 앞 로봇이 늦어졌다
    traffic._plan(b, traffic._legs["b"])
    assert not traffic._legs["b"].waiting


def test_the_robot_that_reaches_the_line_first_goes_first(monkeypatch):
    traffic, at = floor({"a": (1.0, 1.15), "far": (2.6, 0.8), "near": (1.6, 0.8)})
    monkeypatch.setattr(traffic, "line_free_in",
                        lambda line, exclude=None: None if busy[0] is False else 30.0)
    busy = [True]
    traffic._start_leg(traffic._missions["far"], "TO_WORK", None, "W")
    at(1.0)
    traffic._start_leg(traffic._missions["near"], "TO_WORK", None, "W")
    assert traffic._legs["far"].waiting and traffic._legs["near"].waiting
    assert traffic.line_eta("near") < traffic.line_eta("far")
    busy[0] = False
    traffic._drive(traffic._missions["far"])
    traffic._drive(traffic._missions["near"])
    assert traffic._legs["far"].waiting
    assert not traffic._legs["near"].waiting


def test_a_robot_that_waited_long_goes_first_again(monkeypatch):
    traffic, at = floor({"a": (1.0, 1.15), "far": (2.6, 0.8), "near": (1.6, 0.8)})
    busy = [True]
    monkeypatch.setattr(traffic, "line_free_in",
                        lambda line, exclude=None: None if busy[0] is False else 30.0)
    traffic._start_leg(traffic._missions["far"], "TO_WORK", None, "W")
    at(traffic.params.floor_dock_aging_s)
    traffic._start_leg(traffic._missions["near"], "TO_WORK", None, "W")
    busy[0] = False
    traffic._drive(traffic._missions["near"])
    traffic._drive(traffic._missions["far"])
    assert traffic._legs["near"].waiting
    assert not traffic._legs["far"].waiting


def test_without_the_schedule_the_first_robot_to_ask_goes_first(monkeypatch):
    traffic, at = floor({"a": (1.0, 1.15), "far": (2.6, 0.8), "near": (1.6, 0.8)},
                         schedule=False)
    traffic._start_leg(traffic._missions["a"], "TO_WORK", None, "W")
    traffic._start_leg(traffic._missions["far"], "TO_WORK", None, "W")
    at(1.0)
    traffic._start_leg(traffic._missions["near"], "TO_WORK", None, "W")
    del traffic._legs["a"]
    for cell in list(traffic.zones.held_set("a")):
        traffic.zones.release("a", cell)
    traffic._drive(traffic._missions["near"])
    traffic._drive(traffic._missions["far"])
    assert traffic._legs["near"].waiting
    assert not traffic._legs["far"].waiting


@pytest.mark.parametrize("scenario,seed", [
    ("basic_orders", 1), ("basic_orders", 2), ("basic_orders", 3),
    ("order_burst", 1), ("blocked_lane", 1),
])
def test_the_lane_scenarios_finish_with_the_schedule(scenario, seed, monkeypatch):
    real = compare_floor.load_params
    monkeypatch.setattr(compare_floor, "load_params",
                        lambda: replace(real(), floor_dock_schedule=True))
    row, world = compare_floor.run("lane_map", scenario, seed, "freefloor")
    assert world.core.traffic.params.floor_dock_schedule
    assert row["finished"], row["error"]
    assert row["violations"] == 0 and row["isolations"] == 0
    assert row["min_gap_m"] >= 2 * world.core.traffic.params.robot_radius_m
