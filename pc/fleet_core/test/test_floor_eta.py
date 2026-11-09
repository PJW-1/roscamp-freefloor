"""자유바닥 ETA(관측): 경로 시간, 스테이션 작업 남은 시간, 접근선이 빌 때, 접근선까지 닿는 때."""
import math
import statistics
import sys
from dataclasses import replace
from pathlib import Path

import pytest
from fleet_core.common.params import load_params
from fleet_core.manager import FleetManager
from fleet_core.orders.orchestrator import FILL
from fleet_core.traffic.floor.controller import BLOCK_OWNER
from fleet_core.traffic.floor.eta import route_s
from fleet_core.traffic.floor.planner import Route
from fleet_core.traffic.lane_map import load_lane_map
from fleet_core.traffic.mission import AT_STATION, ERROR
from test_floor_multi import two_robot_floor

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
import compare_floor  # noqa: E402

P = load_params()


# ---- 경로 시간 ------------------------------------------------------------------------

def test_a_straight_route_takes_its_length_over_the_speed():
    assert route_s([(0.0, 0.0), (1.0, 0.0)], P) == pytest.approx(1.0 / P.eta_speed_mps)


def test_a_sharp_corner_adds_an_in_place_turn():
    t = route_s([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)], P)
    assert t == pytest.approx(2.0 / P.eta_speed_mps + math.pi / 2 * P.eta_turn_s_per_rad)


def test_a_rounded_corner_counts_its_whole_turn_once():
    """원호 점마다 꺾임이 기준보다 커도 한 모서리로 센다."""
    r = 0.15
    arc = [(1.0 - r + r * math.sin(a), r - r * math.cos(a))
           for a in [k * (math.pi / 2) / 5 for k in range(6)]]
    points = [(0.0, 0.0)] + arc + [(1.0, 1.0)]
    length = sum(math.dist(a, b) for a, b in zip(points, points[1:], strict=False))
    t = route_s(points, P)
    assert t == pytest.approx(length / P.eta_speed_mps + math.pi / 2 * P.eta_turn_s_per_rad,
                              abs=0.05)


def test_a_small_bend_is_driven_through():
    t = route_s([(0.0, 0.0), (1.0, 0.0), (2.0, 0.1)], P)
    length = 1.0 + math.hypot(1.0, 0.1)
    assert t == pytest.approx(length / P.eta_speed_mps)


def test_the_first_turn_is_from_the_robot_heading():
    t = route_s([(0.0, 0.0), (0.0, 1.0)], P, heading=0.0)
    assert t == pytest.approx(1.0 / P.eta_speed_mps + math.pi / 2 * P.eta_turn_s_per_rad)


def test_a_one_point_route_takes_no_time():
    assert route_s([(1.0, 1.0)], P) == 0.0


# ---- 접근선 ---------------------------------------------------------------------------

def test_a_free_line_with_nobody_on_the_way_has_no_free_time():
    traffic = two_robot_floor()
    assert traffic.line_free_in("outbound_in") is None


def test_a_robot_standing_on_a_line_holds_it_for_an_unknown_time():
    traffic = two_robot_floor()  # a는 work_in 칸에 닿은 채 서 있다
    assert traffic.line_free_in("work_in") == math.inf


def test_a_line_someone_is_driving_to_is_free_after_its_dock_work_and_leaving():
    traffic = two_robot_floor()
    traffic._start_leg(traffic._missions["a"], "TO_WORK", None, "W")
    leg = traffic._legs["a"]
    p = traffic.params
    drive = route_s(leg.route.points, p, heading=0.0)
    want = (drive + p.eta_dock_s + p.eta_fill_base_s
            + p.eta_fill_items_default * p.eta_fill_s_per_item + p.floor_leave_s)
    assert traffic.line_free_in("work_in") == pytest.approx(want, abs=0.5)


def test_a_waiting_robot_reaches_the_line_start_from_its_wait_spot():
    traffic = two_robot_floor()
    traffic._start_leg(traffic._missions["a"], "TO_WORK", None, "W")
    b = traffic._missions["b"]
    traffic._start_leg(b, "TO_WORK", None, "W")
    leg = traffic._legs["b"]
    assert leg.waiting
    start = traffic.floor.fixed_lines["work_in"].points[0]
    spot = leg.route.points[-1]
    least = (route_s(leg.route.points, traffic.params, heading=0.0)
             + math.dist(spot, start) / traffic.params.eta_speed_mps)
    eta = traffic.line_eta("b")
    assert least <= eta <= least + math.pi * traffic.params.eta_turn_s_per_rad + 1e-9


def _floor_at(names_xy):
    params = replace(load_params(), traffic_mode="freefloor")
    now = [0.0]
    core = FleetManager(load_lane_map(), params, list(names_xy), lambda: now[0])
    for _ in range(3):
        for name, (x, y) in names_xy.items():
            core.on_robot_state(name, x, y, 0.0, "IDLE")
    core.traffic._plan_budget = 99

    def at(t):  # 교통은 매니저가 tick마다 굳힌 시각을 본다
        now[0] = core._time = t

    return core, now, at


def test_a_line_held_by_an_isolated_robot_frees_at_an_unknown_time():
    traffic = two_robot_floor()
    traffic._missions["a"].fault = ERROR  # a는 work_in 칸을 쥔 채 격리됐다
    assert traffic.line_free_in("work_in") == math.inf


def test_a_line_with_cells_marked_blocked_frees_at_an_unknown_time():
    traffic = two_robot_floor()
    cell = sorted(traffic._line_ids["outbound_in"])[0]
    assert traffic.zones.acquire_all(BLOCK_OWNER, {cell})
    assert traffic.line_free_in("outbound_in") == math.inf


def test_a_robot_that_has_not_moved_off_the_dock_still_needs_the_whole_leave_time():
    """다리를 만든 시각은 출발이 아니다: 계획·잠금이 늦으면 도크에 그대로 있다."""
    core, _, at = _floor_at({"a": (1.05, 1.55), "b": (2.6, 0.8)})
    traffic = core.traffic
    a = traffic._missions["a"]
    traffic._start_leg(a, "TO_OUTBOUND", None, "O")
    assert traffic._legs["a"].out_line == "work_out"
    at(30.0)
    assert traffic.line_free_in("work_in") == pytest.approx(traffic.params.floor_leave_s)


def test_the_leave_time_shrinks_with_the_way_gone_along_the_exit_line():
    """다시 계획해 경로에서 출구선이 빠져도 도크에서 나간 만큼 줄어든다."""
    core, _, _ = _floor_at({"a": (1.05, 1.55), "b": (2.6, 0.8)})
    traffic = core.traffic
    a = traffic._missions["a"]
    traffic._start_leg(a, "TO_OUTBOUND", None, "O")
    line = traffic.floor.fixed_lines["work_out"]
    (x0, y0), (x1, y1) = line.points[0], line.points[-1]
    a.x, a.y = (x0 + x1) / 2, (y0 + y1) / 2
    leg = traffic._legs["a"]
    leg.route = Route([(a.x, a.y), traffic.site.waypoint_pose("O")])
    leg.s = 0.0
    assert traffic.line_free_in("work_in") == pytest.approx(traffic.params.floor_leave_s / 2,
                                                             abs=0.1)


def test_a_robot_pushed_off_the_exit_line_still_needs_the_whole_leave_time():
    """출구선 방향으로 간 거리만 센다: 옆으로 밀려 도크에서 멀어진 것은 나간 것이 아니다."""
    core, _, _ = _floor_at({"a": (1.05, 1.55), "b": (2.6, 0.8)})
    traffic = core.traffic
    a = traffic._missions["a"]
    traffic._start_leg(a, "TO_OUTBOUND", None, "O")
    (x0, y0), (x1, y1) = traffic.floor.fixed_lines["work_out"].points[:2]
    d = math.hypot(x1 - x0, y1 - y0)
    a.x, a.y = x0 - 0.3 * (y1 - y0) / d, y0 + 0.3 * (x1 - x0) / d  # 출구선에 수직으로 0.3 m
    assert traffic.line_free_in("work_in") == pytest.approx(traffic.params.floor_leave_s,
                                                             abs=0.05)


def test_an_exit_line_of_no_length_counts_the_whole_leave_time(monkeypatch):
    core, _, _ = _floor_at({"a": (1.05, 1.55), "b": (2.6, 0.8)})
    traffic = core.traffic
    traffic._start_leg(traffic._missions["a"], "TO_OUTBOUND", None, "O")
    import fleet_core.traffic.floor.controller as floor_controller
    monkeypatch.setattr(floor_controller, "_along", lambda points, xy: (0.0, 0.0))
    assert traffic.line_free_in("work_in") == pytest.approx(traffic.params.floor_leave_s)


def test_the_station_work_left_reads_the_tick_time_not_the_live_clock(monkeypatch):
    """교통과 같은 굳힌 시각: 기록을 다시 돌려도 같은 값."""
    core, now, at = _floor_at({"a": (1.05, 1.55)})
    monkeypatch.setattr(core.orchestrator, "station_job", lambda robot: (FILL, 2, 10.0))
    at(12.0)
    p = core.params
    want = p.eta_fill_base_s + 2 * p.eta_fill_s_per_item - 2.0
    assert core.station_left_s("a") == pytest.approx(want)
    now[0] = 15.0  # 다음 입력 전
    assert core.station_left_s("a") == pytest.approx(want)


def test_a_robot_with_no_approach_line_has_no_line_eta():
    traffic = two_robot_floor()
    traffic._start_leg(traffic._missions["a"], "TO_PARK", None, "B")
    assert traffic.line_eta("a") is None


# ---- 시뮬에서 맞는지 ------------------------------------------------------------------

def _watch(scenario, seed):
    """도크의 로봇 하나만 접근선을 쥐고 그 선으로 가는 로봇도 없을 때의 예측(시각 + 빌 시각)과
    그 로봇이 선 칸을 다 놓은 시각. 그 뒤 다른 로봇이 곧바로 잡을 수 있어 선 전체가 비는 시각은 아니다.
    seen: 스테이션에 선 횟수(visits), 예측을 맞춰 본 머묾 번호(stays), 쥔 로봇이 있는데 빈 선·모름으로
    본 횟수(wrong), 끝날 때 아직 안 놓은 머묾 번호(pending). 머묾 번호는 스테이션에 설 때마다 하나."""
    data = compare_floor.baseline.load_scenario(scenario)
    world = compare_floor.build(data, "lane_map", seed, "freefloor")
    traffic = world.core.traffic
    preds: dict[str, list] = {}
    errors, left = [], []
    seen = {"visits": 0, "stays": [], "wrong": 0, "pending": []}
    phase: dict[str, str] = {}
    visit: dict[str, int] = {}  # 로봇 -> 지금 머묾 번호
    step = world.step

    def watched():
        step()
        for n, m in traffic._missions.items():
            if m.phase == AT_STATION and phase.get(n) != AT_STATION:
                seen["visits"] += 1
                visit[n] = seen["visits"]
            phase[n] = m.phase
        for line, ids in traffic._line_ids.items():
            if not line.endswith("_in"):
                continue
            holders = {n for n in traffic._missions if ids & traffic.zones.held_set(n)}
            docked = [n for n in holders if traffic._missions[n].phase == AT_STATION]
            going = [n for n, leg in traffic._legs.items()
                     if leg.in_line == line and not leg.waiting and leg.route is not None]
            free_in = traffic.line_free_in(line)
            if len(docked) == 1 and holders == set(docked) and not going:
                if free_in is None or free_in == math.inf:
                    seen["wrong"] += 1
                else:
                    key = (line, docked[0], visit[docked[0]])
                    preds.setdefault(key, []).append(world.t + free_in)
            for n in docked:
                s = world.core.station_left_s(n)
                if s is not None:
                    left.append((n, world.t, s))
            for key in [k for k in preds if k[0] == line and k[1] not in holders]:
                seen["stays"].append(key[2])
                errors.extend(abs(at - world.t) for at in preds.pop(key))

    world.step = watched
    while world.t < 400 and not world.all_orders_final():
        world.step()
    seen["pending"] = [k[2] for k in preds]
    reports = [t for t, c in world.commands if type(c).__name__ == "ReportOrderCmd"]
    return (errors, left, max(reports, default=None),
            [(round(t, 3), repr(c)) for t, c in world.commands], seen)


@pytest.mark.parametrize("scenario,seed", [("basic_orders", 1), ("order_burst", 2)])
def test_the_line_free_time_is_predicted_within_a_few_seconds(scenario, seed):
    """도크의 로봇이 선 칸을 다 놓는 시각. 머묾마다 맞춰 보고, 빠진 머묾이 없다."""
    errors, *_, seen = _watch(scenario, seed)
    assert seen["wrong"] == 0
    assert len(seen["pending"]) <= 1  # 주문이 다 끝날 때 도크에 선 로봇
    # 머묾마다 한 번씩: 빠지거나 두 번 센 머묾이 없다
    assert sorted(seen["stays"] + seen["pending"]) == list(range(1, seen["visits"] + 1))
    assert seen["visits"] >= 10
    assert statistics.median(errors) < 2.0
    assert max(errors) < 6.0


def test_the_station_work_left_counts_down_during_a_stay():
    left = _watch("basic_orders", 1)[1]
    assert left
    stays, open_ = [], {}  # 로봇마다 지금 머무는 구간
    for name, t, s in left:
        k = open_.get(name)
        if k is not None and t - stays[k][-1][0] < 0.5:
            stays[k].append((t, s))
        else:
            stays.append([(t, s)])
            open_[name] = len(stays) - 1
    assert len(stays) >= 4
    for stay in stays:
        values = [s for _, s in stay]
        assert all(b <= a + 1e-6 for a, b in zip(values, values[1:], strict=False)), values
        # 시계만큼 줄고 1초에서 멈춘다
        took = stay[-1][0] - stay[0][0]
        assert len(stay) >= 2 and took >= 2.0
        assert values[0] - values[-1] == pytest.approx(min(took, values[0] - 1.0), abs=0.5)


def test_watching_the_predictions_does_not_change_the_run():
    """ETA는 보기만 한다: 매 단계 물어도 보낸 명령이 모두 같다."""
    watched = _watch("basic_orders", 1)[3]
    data = compare_floor.baseline.load_scenario("basic_orders")
    world = compare_floor.build(data, "lane_map", 1, "freefloor")
    while world.t < 400 and not world.all_orders_final():
        world.step()
    assert world.all_orders_final()
    assert [(round(t, 3), repr(c)) for t, c in world.commands] == watched
