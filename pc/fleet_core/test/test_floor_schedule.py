"""자유바닥 접근선 시간창(floor_dock_schedule): ETA로 차례를 정하고, 비울 때에 맞춰 미리 출발."""
import math
import sys
from dataclasses import replace
from pathlib import Path

import pytest
from fleet_core.common.params import load_params
from fleet_core.manager import FleetManager
from fleet_core.traffic.deadlock import wait_graph
from fleet_core.traffic.lane_map import load_lane_map

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


def _sent_early_at_the_line(monkeypatch):
    """a는 도크에 서 있고(선 칸을 쥠), b는 미리 출발해 선 시작 바로 앞까지 왔다."""
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
    leg.s = leg.s_sent = cap - 0.05
    b.x, b.y = leg.route.at(leg.s)
    leg.s_lock = cap
    assert traffic.zones.acquire_all("b", traffic._body_cells(b.x, b.y)
                                     | traffic._swept(leg.route.between(leg.s, cap)))
    return traffic, at, leg


def test_a_robot_sent_early_stops_where_it_stands_off_every_fixed_line(monkeypatch):
    """대기 자리와 같은 규칙: 선 시작에서 선 칸을 쥐고 서면 떠나는 로봇의 길을 막을 수 있다."""
    traffic, _, leg = _sent_early_at_the_line(monkeypatch)
    s0 = leg.route.fixed[0][0]
    assert s0 - 0.6 < leg.s_lock < s0
    assert traffic._standing_cells(*leg.route.at(leg.s_lock)).isdisjoint(traffic._all_line_ids)


def test_a_robot_sent_early_does_not_queue_on_a_line_someone_holds(monkeypatch):
    """줄을 서면 떠나는 로봇과 대기 순환이 된다(지나가며 출구선 칸을 쥐고 있을 때)."""
    traffic, _, leg = _sent_early_at_the_line(monkeypatch)
    cap = leg.s_lock
    traffic._extend(traffic._missions["b"], leg)
    assert leg.s_lock == pytest.approx(cap)
    assert not set(traffic.zones.waited_cells("b")) & line_cells(traffic)
    assert "b" not in wait_graph(traffic.zones)


def test_a_robot_sent_early_waits_at_the_line_start_without_replanning(monkeypatch):
    traffic, at, leg = _sent_early_at_the_line(monkeypatch)
    b = traffic._missions["b"]
    route = leg.route
    for k in range(1, 40):
        at(0.5 * k)
        traffic._drive(b)
    assert leg.route is route and not leg.waiting


def test_a_robot_sent_early_does_not_stop_on_a_parking_spot(monkeypatch):
    """주차 지점 칸을 쥐고 서면 그 지점으로 가는 로봇이 매 tick 다시 출발한다(격리까지 감)."""
    traffic, _ = floor({"a": (1.0, 1.15), "b": (1.52, 2.06)})
    a, b = traffic._missions["a"], traffic._missions["b"]
    traffic._start_leg(a, "TO_WORK", None, "W")
    del traffic._legs["a"]
    a.phase = "AT_STATION"
    monkeypatch.setattr(traffic, "line_free_in", lambda line, exclude=None: 1.0)
    traffic._start_leg(b, "TO_WORK", None, "W")
    leg = traffic._legs["b"]
    assert leg.admitted
    cap = traffic._line_cap(b, leg)
    if cap is not None:
        spots = set().union(*(traffic._standing_cells(*traffic.site.waypoint_pose(p))
                              for p in traffic.site.parking))
        assert traffic._standing_cells(*leg.route.at(cap)).isdisjoint(spots)


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
    for cell in list(traffic.zones.held_set("a")):
        traffic.zones.release("a", cell)
    assert traffic._extend(traffic._missions["b"], leg)
    assert leg.s_lock == pytest.approx(leg.route.fixed[0][1])
    assert line_cells(traffic) <= traffic.zones.held_set("b")


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
