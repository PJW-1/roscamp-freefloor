"""자유바닥 ETA(관측): 경로 시간, 스테이션 작업 남은 시간, 접근선이 빌 때, 접근선까지 닿는 때."""
import math
import statistics
import sys
from pathlib import Path

import pytest
from fleet_core.common.params import load_params
from fleet_core.traffic.floor.eta import route_s
from fleet_core.traffic.mission import AT_STATION
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


def test_a_robot_with_no_approach_line_has_no_line_eta():
    traffic = two_robot_floor()
    traffic._start_leg(traffic._missions["a"], "TO_PARK", None, "B")
    assert traffic.line_eta("a") is None


# ---- 시뮬에서 맞는지 ------------------------------------------------------------------

def _watch(scenario, seed):
    """접근선을 쥔 로봇이 스테이션에 서 있는 동안의 예측(시각, 빌 시각)과 실제로 빈 시각."""
    data = compare_floor.baseline.load_scenario(scenario)
    world = compare_floor.build(data, "lane_map", seed, "freefloor")
    traffic = world.core.traffic
    preds: dict[str, list] = {}
    errors, left = [], []
    step = world.step

    def watched():
        step()
        for line, ids in traffic._line_ids.items():
            if not line.endswith("_in"):
                continue
            holders = {n for n in traffic._missions if ids & traffic.zones.held_set(n)}
            docked = [n for n in holders if traffic._missions[n].phase == AT_STATION]
            free_in = traffic.line_free_in(line)
            if len(docked) == 1 and free_in is not None and free_in < math.inf:
                preds.setdefault((line, docked[0]), []).append(world.t + free_in)
            for n in docked:
                s = world.core.station_left_s(n)
                if s is not None:
                    left.append((n, world.t, s))
            for key in [k for k in preds if k[0] == line and k[1] not in holders]:
                errors.extend(abs(at - world.t) for at in preds.pop(key))

    world.step = watched
    while world.t < 400 and not world.all_orders_final():
        world.step()
    reports = [t for t, c in world.commands if type(c).__name__ == "ReportOrderCmd"]
    return errors, left, max(reports, default=None)


@pytest.mark.parametrize("scenario,seed", [("basic_orders", 1), ("order_burst", 2)])
def test_the_line_free_time_is_predicted_within_a_few_seconds(scenario, seed):
    errors, _, _ = _watch(scenario, seed)
    assert errors
    assert statistics.median(errors) < 2.0
    assert max(errors) < 6.0


def test_the_station_work_left_counts_down_during_a_stay():
    _, left, _ = _watch("basic_orders", 1)
    assert left
    stays, last = [], {}
    for name, t, s in left:
        if name in last and t - last[name][0] < 0.5:
            stays[last[name][1]].append(s)
        else:
            stays.append([s])
        last[name] = (t, len(stays) - 1)
    for stay in stays:
        assert stay[-1] <= stay[0]


def test_watching_the_predictions_does_not_change_the_run():
    """ETA는 보기만 한다: 매 단계 물어도 결과가 같다."""
    _, _, finish = _watch("basic_orders", 1)
    row, _ = compare_floor.run("lane_map", "basic_orders", 1, "freefloor")
    assert row["finished"]
    assert finish == pytest.approx(row["finish_s"])
