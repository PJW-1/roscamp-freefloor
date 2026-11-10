"""자유바닥 교차 양보: 길이 엇갈리는 두 로봇은 먼저 닿는 쪽이 지나가고, 다른 쪽은 엇갈리는 곳
앞에 서서 기다렸다가 제 길로 간다(돌아가지 않는다). 8자 경기장 가운데(X)에서 본다."""
import math
from dataclasses import replace
from pathlib import Path

import pytest
from fleet_core.common.commands import MoveCmd
from fleet_core.common.params import DEFAULT_PARAMS as DEFAULT_PARAMS_FILE
from fleet_core.common.params import load_params
from fleet_core.traffic.floor.floor_map import floor_map_for
from fleet_core.traffic.lane_map import load_lane_map
from floor_world import FloorWorld

REPO = Path(__file__).resolve().parents[3]
LANES = load_lane_map(REPO / "sim" / "fake_robots" / "maps" / "figure8_arena.yaml")
FLOOR = floor_map_for(LANES)
PARAMS = replace(load_params(DEFAULT_PARAMS_FILE.parent / "fleet_params_real_arena.yaml"),
                 lane_goal_mode="path")
SPEED = 0.15
X = LANES.waypoint_pose("X")
NORTH = 0.75  # 가운데 통로를 다 지난 높이


def crossing(starts, goals, head_start_s=0.0):
    """starts {로봇: 지점}에 선 로봇들이 goals {로봇: 지점}로 간다. 뒤 로봇들은 head_start_s 늦게."""
    world = FloorWorld(seed=1, orders=[], site=LANES, floor=FLOOR, params=PARAMS,
                       robots={n: {"start": w, "speed_mps": SPEED, "pose_noise_m": 0.0}
                               for n, w in starts.items()})
    traffic = world.core.traffic
    names = list(starts)
    world.step()  # 첫 보고로 자리를 잡는다
    started, world.arrived, world.trace = set(), {}, []
    for _ in range(600):
        for k, name in enumerate(names):
            if name not in started and world.t >= k * head_start_s:
                traffic._plan_budget = 99
                traffic._start_leg(traffic._missions[name], "TO_PARK", None, goals[name])
                started.add(name)
        world.step()
        world.trace.append((world.t, {n: (r.x, r.y) for n, r in world.virtual.items()}))
        for n in started:
            r = world.virtual[n]
            if n not in world.arrived and r.y > NORTH:
                world.arrived[n] = world.t  # 가운데를 지나 북쪽 통로에 닿았다
        if len(world.arrived) == len(names):
            break
    return world


def codes(world) -> list[str]:
    return [e.code for e in world.events]


WEST, EAST = {"start": "B1", "goal": "QO"}, {"start": "B3", "goal": "QW"}


def run_cross(first, second, head_start_s):
    """서쪽 로봇은 남서에서 북동으로, 동쪽 로봇은 남동에서 북서로: 가운데에서 엇갈린다."""
    ways = dict(zip((first, second), (WEST, EAST) if first == "pinky1" else (EAST, WEST),
                    strict=True))
    return crossing({n: w["start"] for n, w in ways.items()},
                    {n: w["goal"] for n, w in ways.items()}, head_start_s)


# 같이 닿으면 예전에는 한 대가 돌아갔고(교착 감지, 재계획), 1초 차이면 서로 칸을 물고 비켜 서기까지
# 갔다(이 시험 세계에서 60초 넘게 안 풀림). Gazebo 8자 경기장 104 s, 183 s(561d111)가 이 꼴이다.
@pytest.mark.parametrize("head_start_s", [0.0, 1.0, 2.0])
@pytest.mark.parametrize("first, second", [("pinky1", "pinky3"), ("pinky3", "pinky1")])
def test_robots_whose_ways_cross_take_turns_and_keep_their_routes(first, second, head_start_s):
    world = run_cross(first, second, head_start_s)
    metrics = world.core.traffic.metrics
    assert set(world.arrived) == {first, second}, (world.arrived, world.trace[-1])
    assert metrics["replans"] == 0 and metrics["sidesteps"] == 0  # 돌아가지 않았다
    assert "DEADLOCK_DETECTED" not in codes(world)  # 서로 칸을 물지 않았다
    assert world.cell_violations == 0
    # 곧장 가면 9초쯤 걸린다: 기다린 쪽도 상대가 지나갈 몇 초만 더 쓴다
    assert max(world.arrived.values()) <= head_start_s + 16.0


def test_the_robot_that_gives_way_stands_clear_of_the_other():
    world = run_cross("pinky1", "pinky3", 0.0)
    gap = min(math.dist(*pos.values()) for _, pos in world.trace)
    assert gap >= 0.20  # 몸 둘(0.16 m)보다 넓게 떨어져 선다
    order = sorted(world.arrived, key=world.arrived.get)
    late = order[1]
    # 늦은 쪽은 가운데 앞에서 섰다가 간다: 선 채로 1초 넘게 머문 곳이 X 남쪽이다
    still = [pos[late] for (_, pos), (_, nxt) in zip(world.trace, world.trace[10:], strict=False)
             if math.dist(pos[late], nxt[late]) < 0.01 and pos[late][1] > 0.2]
    assert still and all(y < X[1] for _, y in still)


# ---- 차례 정하기(_crossing)와 비켜 주기(_give_way): 시각표를 손으로 준다 ----------------------


def pair():
    return pair_clock()[0]


def pair_clock():
    """기본 바닥의 두 로봇 a, b와 다리, 그리고 시계를 dt만큼 돌리는 함수(관제 시계는 입력이 올 때
    간다). 시각표(_spans)는 시험이 넣는다."""
    from fleet_core.manager import FleetManager
    params = replace(load_params(), traffic_mode="freefloor")
    now = [0.0]
    core = FleetManager(load_lane_map(), params, ["a", "b"], lambda: now[0])
    traffic = core.traffic
    for _ in range(3):
        core.on_robot_state("a", 1.0, 1.15, 0.0, "IDLE")
        core.on_robot_state("b", 2.6, 0.8, 0.0, "IDLE")
    traffic._plan_budget = 99
    traffic._start_leg(traffic._missions["a"], "TO_PARK", None, "B")
    traffic._start_leg(traffic._missions["b"], "TO_PARK", None, "A")

    def advance(dt: float) -> float:
        now[0] += dt
        core.on_robot_state("a", 1.0, 1.15, 0.0, "IDLE")
        return now[0]

    return traffic, advance


def timetable(traffic, monkeypatch, a_at, b_at, shared=((1.85, 1.15),)):
    """shared 자리의 칸을 a는 a_at초, b는 b_at초에 1초 동안 쓴다."""
    floor = traffic.floor
    cells = [floor.cell_of(*xy) for xy in shared]
    tables = {"a": {c: [a_at, a_at + 1.0, 1.0, 0.0] for c in cells},
              "b": {c: [b_at, b_at + 1.0, 0.0, 1.0] for c in cells}}
    monkeypatch.setattr(traffic, "_spans", lambda name: tables[name])
    return {floor.cell_id(c) for c in cells}


def test_the_robot_that_reaches_the_crossing_first_goes_first(monkeypatch):
    traffic = pair()
    timetable(traffic, monkeypatch, a_at=3.0, b_at=3.8)
    assert traffic._crossing("a", "b")[0] == "a"
    assert traffic._crossing("b", "a")[0] == "a"  # 누가 물어도 같은 답
    timetable(traffic, monkeypatch, a_at=3.8, b_at=3.0)
    assert traffic._crossing("a", "b")[0] == "b"


def test_ways_that_use_a_cell_at_different_times_do_not_cross(monkeypatch):
    traffic = pair()
    timetable(traffic, monkeypatch, a_at=1.0, b_at=5.0)
    assert traffic._crossing("a", "b") is None


def test_a_near_tie_goes_to_the_robot_with_the_higher_priority(monkeypatch):
    traffic = pair()
    timetable(traffic, monkeypatch, a_at=3.2, b_at=3.0)  # b가 0.2초 먼저: 동률로 본다
    monkeypatch.setattr(traffic, "_priority", lambda name: 100 if name == "a" else 0)
    assert traffic._crossing("a", "b")[0] == "a"
    monkeypatch.setattr(traffic, "_priority", lambda name: 0)
    assert traffic._crossing("a", "b")[0] == "b"  # 우선순위도 같으면 이름이 뒤인 쪽(물러서는 쪽은 a)


def test_a_robot_on_a_dock_line_does_not_give_way(monkeypatch):
    """도크를 드나드는 선이 병목이다: 선 위의 로봇은 늦게 닿아도 서서 기다리지 않는다."""
    traffic = pair()
    timetable(traffic, monkeypatch, a_at=3.0, b_at=3.8)
    m, leg = traffic._missions["b"], traffic._legs["b"]
    assert set(traffic._give_way(m, leg)) == {"a"}
    leg.behind = ()
    leg.route.fixed = [(0.0, leg.route.length, "work_out")]  # b는 도크를 나가는 선 위다
    assert traffic._give_way(m, leg) == {}


def test_the_cells_to_keep_off_are_the_others_way_with_a_ring_round_it(monkeypatch):
    traffic = pair()
    floor = traffic.floor
    shared = timetable(traffic, monkeypatch, a_at=3.0, b_at=3.8)
    lead, keep_off = traffic._crossing("b", "a")
    i, j = floor.cell_of(1.85, 1.15)
    assert lead == "a" and shared <= keep_off
    assert floor.cell_id((i + 1, j + 1)) in keep_off and floor.cell_id((i + 2, j)) not in keep_off


def test_a_robot_gives_way_only_to_one_that_is_moving_and_goes_first(monkeypatch):
    traffic = pair()
    shared = timetable(traffic, monkeypatch, a_at=3.0, b_at=3.8)
    m, leg = traffic._missions["b"], traffic._legs["b"]
    assert set(traffic._give_way(m, leg)) == {"a"} and shared <= traffic._give_way(m, leg)["a"]
    assert traffic._give_way(traffic._missions["a"], traffic._legs["a"]) == {}  # 먼저 갈 쪽
    traffic._legs["a"].stuck_since = 0.0  # a가 막혀 섰다: 기다려도 오지 않는다
    assert traffic._give_way(m, leg) == {}


def test_a_robot_already_in_the_others_way_does_not_stand_and_wait(monkeypatch):
    traffic = pair()
    m, leg = traffic._missions["b"], traffic._legs["b"]
    timetable(traffic, monkeypatch, a_at=3.0, b_at=3.8, shared=((m.x, m.y),))
    assert traffic._give_way(m, leg) == {}  # 몸이 a의 길 위다: 서 있어도 a가 못 지나간다


def test_a_turn_once_given_is_kept_until_the_other_has_passed(monkeypatch):
    traffic = pair()
    m, leg = traffic._missions["b"], traffic._legs["b"]
    timetable(traffic, monkeypatch, a_at=3.0, b_at=3.8)
    assert set(traffic._give_way(m, leg)) == {"a"}
    timetable(traffic, monkeypatch, a_at=3.0, b_at=1.0 + 1.5)  # 이제 b가 먼저로 보인다
    assert set(traffic._give_way(m, leg)) == {"a"}  # 정한 차례를 지킨다
    assert traffic._give_way(traffic._missions["a"], traffic._legs["a"]) == {}  # 둘 다 서지 않는다
    timetable(traffic, monkeypatch, a_at=0.0, b_at=5.0)  # a가 지나갔다
    assert traffic._give_way(m, leg) == {}


def test_a_robot_that_waited_too_long_stops_giving_way(monkeypatch):
    traffic = pair()
    m, leg = traffic._missions["b"], traffic._legs["b"]
    timetable(traffic, monkeypatch, a_at=3.0, b_at=3.8)
    leg.no_yield = ("a",)
    assert traffic._give_way(m, leg) == {}


def test_a_waiting_robot_takes_no_cell_of_the_others_way_and_gives_up_after_a_while(monkeypatch):
    traffic, advance = pair_clock()
    m, leg = traffic._missions["b"], traffic._legs["b"]
    ahead = leg.route.at(0.6)  # b가 곧 잡을 자리(몸에서는 떨어져 있다)
    banned = timetable(traffic, monkeypatch, a_at=3.0, b_at=3.8, shared=(ahead,))
    while traffic._extend(m, leg):
        pass  # 그 앞까지는 잡는다
    assert leg.s_lock < 0.6 and leg.yields == ("a",)
    assert not banned & traffic.zones.held_set("b")
    assert not traffic.zones.waited_cells("b")  # 줄을 서지 않는다: 대기 순환으로 보이지 않는다
    stuck = traffic.metrics["replans"]
    leg.s = leg.s_lock  # 잠근 끝까지 와서 섰다
    leg.stuck_since = advance(0.0)
    traffic._replan_if_stuck(m, leg, False)
    later = advance(3.0)
    traffic._replan_if_stuck(m, leg, False)
    assert traffic.metrics["replans"] == stuck and leg.stuck_since == later  # 돌아가지 않는다
    advance(traffic.params.floor_pass_wait_s + traffic.params.floor_plan_window_s)
    assert not traffic._extend(m, leg)
    assert leg.no_yield == ("a",) and leg.yields == ()
    assert traffic._extend(m, leg)  # 더 기다리지 않고 잡는다(예전 방식으로 돌아간다)


# ---- Codex 리뷰(d099005) ------------------------------------------------------------------


def test_a_cell_used_by_two_robots_one_after_the_other_costs_the_wait_for_both(monkeypatch):
    """[1, 3]초와 [2.5, 5]초에 쓰는 칸에 1.5초에 닿으면 5초까지 기다린다(첫 로봇만 보지 않는다)."""
    traffic = pair()
    floor, v = traffic.floor, traffic.params.detour_speed_mps
    cell = floor.cell_of(1.85, 1.15)
    tables = {"a": {cell: [1.0, 3.0, 1.0, 0.0]}, "b": {cell: [2.5, 5.0, 1.0, 0.0]}}
    traffic._legs["c"] = traffic._legs["b"]  # 계획하는 셋째 로봇 자리(시각표만 본다)
    monkeypatch.setattr(traffic, "_spans", lambda name: tables.get(name, {}))
    lag = max(0.0, traffic.params.release_distance_m - traffic._body) / v
    held, flat = traffic._wait_for("c")(cell, 1.5 * v, (0, 1))
    assert flat == 0.0 and held == pytest.approx((5.0 + lag - 1.5) * v)


def test_a_cell_with_an_oncoming_and_a_crossing_robot_costs_both(monkeypatch):
    from fleet_core.traffic.floor.controller import RESERVE_PENALTY_M
    traffic = pair()
    floor, v = traffic.floor, traffic.params.detour_speed_mps
    cell = floor.cell_of(1.85, 1.15)
    tables = {"a": {cell: [1.0, 3.0, 0.0, -1.0]}, "b": {cell: [1.0, 3.0, 1.0, 0.0]}}
    traffic._legs["c"] = traffic._legs["b"]
    monkeypatch.setattr(traffic, "_spans", lambda name: tables.get(name, {}))
    held, flat = traffic._wait_for("c")(cell, 1.5 * v, (0, 1))  # a는 마주 온다, b는 가로지른다
    assert flat == RESERVE_PENALTY_M and held > 0.0


def test_a_robot_standing_at_its_line_stop_or_blocked_is_not_waited_for(monkeypatch):
    traffic = pair()
    timetable(traffic, monkeypatch, a_at=3.0, b_at=3.8)
    m, leg = traffic._missions["b"], traffic._legs["b"]
    assert set(traffic._give_way(m, leg)) == {"a"}
    leg.behind = ()
    traffic._missions["a"].robot_state = "BLOCKED"  # 물체에 막혀 섰다
    assert traffic._give_way(m, leg) == {}
    traffic._missions["a"].robot_state = "MOVING"
    monkeypatch.setattr(traffic, "_line_cap", lambda mission, its_leg: its_leg.s_lock)
    assert traffic._give_way(m, leg) == {}  # 접근선 앞 설 자리까지 잠갔다: 서 있는다


def test_waiting_and_queueing_in_turn_still_reaches_the_time_limit(monkeypatch):
    """비켜 주기와 줄 서기가 번갈아 와도 기다린 시간은 이어서 센다."""
    traffic, advance = pair_clock()
    m, leg = traffic._missions["b"], traffic._legs["b"]
    timetable(traffic, monkeypatch, a_at=3.0, b_at=3.8, shared=(leg.route.at(0.6),))
    while traffic._extend(m, leg):
        pass
    assert leg.yields == ("a",) and leg.yield_since is not None
    since = leg.yield_since
    real = traffic.zones.acquire_all
    monkeypatch.setattr(traffic.zones, "acquire_all", lambda name, cells: False)
    monkeypatch.setattr(traffic, "_give_way", lambda mission, its_leg: {"a": set()})  # 금지 칸에 안 닿는다
    assert not traffic._extend(m, leg)  # 이번에는 남이 쥔 칸에 줄을 섰다
    assert leg.yields == () and leg.yield_since == since
    monkeypatch.setattr(traffic.zones, "acquire_all", real)
    monkeypatch.undo()
    timetable(traffic, monkeypatch, a_at=3.0, b_at=3.8, shared=(leg.route.at(0.6),))
    advance(traffic.params.floor_pass_wait_s + traffic.params.floor_plan_window_s)
    assert not traffic._extend(m, leg)
    assert leg.no_yield == ("a",)


def test_a_waiting_robot_that_cannot_resend_its_locked_part_makes_no_request(monkeypatch):
    traffic = pair()
    m, leg = traffic._missions["b"], traffic._legs["b"]
    timetable(traffic, monkeypatch, a_at=3.0, b_at=3.8, shared=(leg.route.at(0.6),))
    while traffic._extend(m, leg):
        pass
    assert leg.yields == ("a",) and m.goal is None and leg.s_lock - leg.s > 0.01
    floor = traffic.floor
    near = floor.cell_id(floor.cell_of(*leg.route.at(0.05)))
    traffic.zones.release("b", near)
    assert traffic.zones.acquire_all("a", {near})  # 보낼 길의 한 칸을 남이 쥐었다
    traffic._send(m, leg)
    assert m.goal is None and not traffic.zones.waited_cells("b")


def test_a_new_route_starts_with_no_turn_given(monkeypatch):
    traffic, advance = pair_clock()
    m, leg = traffic._missions["b"], traffic._legs["b"]
    timetable(traffic, monkeypatch, a_at=3.0, b_at=3.8, shared=(leg.route.at(0.6),))
    while traffic._extend(m, leg):
        pass
    assert leg.behind == ("a",) and leg.yields == ("a",)
    leg.no_yield = ("a",)
    advance(traffic.params.floor_replan_s)
    traffic._plan(m, leg)
    assert (leg.behind, leg.yields, leg.yield_since) == ((), (), None)
    assert leg.no_yield == ("a",)  # 이 다리에서 더 비켜 주지 않기로 한 것은 남는다
