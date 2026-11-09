"""자유바닥 경로 고르기: 지나가는 로봇은 벽이 아니고, 서로 막은 두 로봇은 한 대만 돌아간다."""
import math
from dataclasses import replace

from fleet_core.common.params import load_params
from fleet_core.manager import FleetManager
from fleet_core.traffic.floor.cells import disk_cells
from fleet_core.traffic.floor.controller import _Leg
from fleet_core.traffic.floor.planner import Route
from fleet_core.traffic.lane_map import load_lane_map
from fleet_core.traffic.mission import MOVE, _Goal

GOAL = (1.0, 0.75)  # b가 가는 곳: 출발에서 서쪽으로 곧게


def floor(starts):
    params = replace(load_params(), traffic_mode="freefloor")
    now = [0.0]
    core = FleetManager(load_lane_map(), params, list(starts), lambda: now[0])
    for _ in range(3):
        for name, (x, y) in starts.items():
            core.on_robot_state(name, x, y, 0.0, "IDLE")
    traffic = core.traffic
    traffic._plan_budget = 99

    def at(t):  # 교통은 매니저가 tick마다 굳힌 시각을 본다
        now[0] = core._time = t

    return traffic, at


def driving(traffic, name, end, lock_m=0.5):
    """name이 end로 달리는 중: 경로, 앞 lock_m 잠금, 보낸 MoveTo가 있고 막히지 않았다."""
    m = traffic._missions[name]
    route = Route([(m.x, m.y), end])
    leg = _Leg(route, end)
    leg.s_lock = leg.s_sent = min(route.length, lock_m)
    traffic._legs[name] = leg
    cells = traffic._swept(route.between(0.0, leg.s_lock)) | traffic._body_cells(m.x, m.y)
    assert traffic.zones.acquire_all(name, cells)
    m.goal = _Goal(MOVE, 7, 0)
    return leg


def standing(traffic, name):
    """칸은 쥔 채 섰다: 다리도 goal도 없다."""
    del traffic._legs[name]
    traffic._missions[name].goal = None


def test_a_passing_robot_is_not_a_wall_to_the_plan():
    """b의 곧은 길을 a의 앞 잠금이 가로지른다. a는 달리는 중이라 곧 비킨다: 돌아가지 않는다."""
    traffic, _ = floor({"a": (1.8, 0.45), "b": (2.6, 0.8)})
    b = traffic._missions["b"]
    driving(traffic, "a", (1.8, 1.15))
    held = set(traffic.zones.held_set("a"))
    straight = traffic.planner.leg((b.x, b.y), GOAL)
    assert traffic._swept(straight.points) & held  # 곧은 길이 a의 칸에 걸친다
    route = traffic._route(b, (b.x, b.y), GOAL)
    assert route.length <= straight.length + 0.05


def test_a_standing_robot_is_still_a_wall_to_the_plan():
    traffic, _ = floor({"a": (1.8, 0.45), "b": (2.6, 0.8)})
    b = traffic._missions["b"]
    driving(traffic, "a", (1.8, 1.15))
    standing(traffic, "a")
    route = traffic._route(b, (b.x, b.y), GOAL)
    assert not traffic._swept(route.points) & traffic.zones.held_set("a")


def _stuck_behind(traffic, at, t0=10.0):
    """b가 a의 칸 바로 앞에서 잠금을 못 늘린다. b의 다리를 돌려준다."""
    b = traffic._missions["b"]
    leg = _Leg(Route([(b.x, b.y), GOAL]), GOAL)
    traffic._legs["b"] = leg
    at(t0)
    assert not traffic._extend(b, leg)
    assert set(traffic.zones.waited_cells("b")) <= traffic.zones.held_set("a")
    traffic._replan_if_stuck(b, leg, False)  # 막힌 때를 적는다
    assert leg.stuck_since == t0
    return b, leg


def _tick(traffic, at, b, leg, t):
    at(t)
    traffic._extend(b, leg)
    traffic._replan_if_stuck(b, leg, False)


def test_a_robot_stuck_behind_a_passing_robot_waits_before_it_replans():
    traffic, at = floor({"a": (2.3, 0.45), "b": (2.6, 0.8)})
    driving(traffic, "a", (2.3, 1.15))
    b, leg = _stuck_behind(traffic, at)
    old = leg.route
    wait = traffic.params.floor_pass_wait_s
    assert wait > 2 * traffic.params.floor_replan_s
    for t in (11.1, 12.2, 10.0 + wait - 0.1):
        _tick(traffic, at, b, leg, t)
        assert leg.route is old  # 곧 비킬 로봇: 서서 기다린다
    _tick(traffic, at, b, leg, 10.0 + wait + 0.1)
    assert leg.route is not old  # 너무 오래 막으면 그래도 다른 길
    assert not traffic._swept(leg.route.points) & traffic.zones.held_set("a")


def test_a_robot_stuck_behind_a_standing_robot_replans_as_before():
    traffic, at = floor({"a": (2.3, 0.45), "b": (2.6, 0.8)})
    driving(traffic, "a", (2.3, 1.15))
    standing(traffic, "a")
    b, leg = _stuck_behind(traffic, at)
    old = leg.route
    _tick(traffic, at, b, leg, 10.0 + traffic.params.floor_replan_s + 0.1)
    assert leg.route is not old
    assert not traffic._swept(leg.route.points) & traffic.zones.held_set("a")


def _head_on(traffic, at, t0=10.0):
    """a는 동쪽으로, b는 서쪽으로 마주 달리다 서로의 칸에 막혔다."""
    a, b = traffic._missions["a"], traffic._missions["b"]
    legs = {}
    for m, end in ((a, (2.9, 0.8)), (b, (1.0, 0.8))):
        legs[m.name] = leg = _Leg(Route([(m.x, m.y), end]), end)
        traffic._legs[m.name] = leg
        assert traffic.zones.acquire_all(m.name, traffic._body_cells(m.x, m.y))
        m.goal = _Goal(MOVE, 7, 0)
    at(t0)
    for m in (a, b):
        assert not traffic._extend(m, legs[m.name])
        traffic._replan_if_stuck(m, legs[m.name], False)
    assert set(traffic.zones.waited_cells("a")) & traffic.zones.held_set("b")
    assert set(traffic.zones.waited_cells("b")) & traffic.zones.held_set("a")
    return a, b, legs


def test_of_two_robots_blocking_each_other_only_one_goes_around():
    """둘 다 다시 계획하면 같은 쪽으로 돌다 또 만난다. 비켜 서기와 같은 순서(우선순위, 이름)로
    한 대만 돌고, 다른 한 대는 제 길을 지킨다."""
    traffic, at = floor({"a": (1.6, 0.8), "b": (1.95, 0.8)})
    a, b, legs = _head_on(traffic, at)
    old = {n: leg.route for n, leg in legs.items()}
    t = 10.0 + traffic.params.floor_replan_s + 0.1
    at(t)
    for m in (a, b):
        traffic._extend(m, legs[m.name])
        traffic._replan_if_stuck(m, legs[m.name], False)
    assert legs["b"].route is old["b"]  # 이름순으로 a가 양보한다(우선순위 같음)
    assert legs["a"].route is not old["a"]


def test_the_robot_that_goes_around_keeps_off_the_other_robots_way_ahead():
    """돌아가는 로봇이 상대가 곧 갈 길 위로 가면 다시 마주친다: 그 길을 비켜서 간다."""
    traffic, at = floor({"a": (1.6, 0.8), "b": (1.95, 0.8)})
    a, b, legs = _head_on(traffic, at)
    at(10.0 + traffic.params.floor_replan_s + 0.1)
    traffic._extend(a, legs["a"])
    traffic._replan_if_stuck(a, legs["a"], False)
    route = legs["a"].route
    ahead = traffic._swept(legs["b"].route.between(0.0, 0.6))  # b가 a 쪽으로 갈 길
    near = traffic._body_cells(a.x, a.y)  # a가 지금 선 자리에서 벗어나는 첫걸음은 봐준다
    s_out = 2 * traffic._body + traffic.floor.cell_m
    later = traffic._swept(route.between(s_out, route.length))
    assert not (later & ahead) - near
    assert math.dist(route.points[-1], (2.9, 0.8)) < 1e-6


def test_the_other_robot_goes_around_when_the_one_that_should_cannot():
    """양보할 로봇이 대기 자리로 가는 다리면 다른 길을 찾을 수 없다: 상대가 돈다."""
    traffic, at = floor({"a": (1.6, 0.8), "b": (1.95, 0.8)})
    a, b, legs = _head_on(traffic, at)
    legs["a"].waiting = True  # a는 접근선을 기다리러 대기 자리로 가는 중
    old = legs["b"].route
    at(10.0 + traffic.params.floor_replan_s + 0.1)
    traffic._extend(b, legs["b"])
    traffic._replan_if_stuck(b, legs["b"], False)
    assert legs["b"].route is not old


def test_the_robot_that_keeps_its_way_goes_around_after_the_pass_wait():
    """양보할 로봇이 길을 못 찾고 그대로 서 있으면, 지킨 로봇도 floor_pass_wait_s 뒤에는 돈다."""
    traffic, at = floor({"a": (1.6, 0.8), "b": (1.95, 0.8)})
    a, b, legs = _head_on(traffic, at)
    old = legs["b"].route
    wait = traffic.params.floor_pass_wait_s
    at(10.0 + wait - 0.1)
    traffic._extend(b, legs["b"])
    traffic._replan_if_stuck(b, legs["b"], False)
    assert legs["b"].route is old
    at(10.0 + wait + 0.1)
    traffic._extend(b, legs["b"])
    traffic._replan_if_stuck(b, legs["b"], False)
    assert legs["b"].route is not old


def test_a_robot_coming_head_on_is_still_a_wall_to_the_plan():
    """마주 오는 로봇은 곧 비키지 않는다(좁은 통로에서 서로 막는다): 예전처럼 벽으로 피한다."""
    traffic, _ = floor({"a": (1.4, 0.8), "b": (2.6, 0.8)})
    b = traffic._missions["b"]
    driving(traffic, "a", (2.9, 0.8))  # a: 동쪽으로, b가 갈 서쪽 길을 마주 온다
    assert traffic._passing("a")
    route = traffic._route(b, (b.x, b.y), GOAL)
    assert not traffic._swept(route.points) & traffic.zones.held_set("a")


def test_a_robot_waiting_for_cells_is_not_passing():
    """남의 칸을 기다리는 로봇은 곧 선다: 지나가는 로봇이 아니다."""
    traffic, at = floor({"a": (1.6, 0.8), "b": (1.95, 0.8)})
    a, b, legs = _head_on(traffic, at)
    legs["a"].stuck_since = legs["b"].stuck_since = None  # 아직 앞 잠금이 남아 달리는 중
    assert not traffic._passing("a") and not traffic._passing("b")


def test_a_wait_spot_route_still_keeps_off_passing_robots():
    """대기 자리로 가는 길은 급하지 않다: 지나가는 로봇도 벽으로 피한다(예전대로)."""
    traffic, _ = floor({"a": (1.8, 0.45), "b": (2.6, 0.8)})
    b = traffic._missions["b"]
    driving(traffic, "a", (1.8, 1.15))
    route = traffic._route(b, (b.x, b.y), GOAL, cautious=True)
    assert not traffic._swept(route.points) & traffic.zones.held_set("a")


def test_the_keeper_goes_around_at_once_when_the_yielder_found_no_way():
    """양보할 로봇이 다른 길을 못 찾았으면 지킨 로봇이 floor_pass_wait_s를 기다리지 않는다."""
    traffic, at = floor({"a": (1.6, 0.8), "b": (1.95, 0.8)})
    a, b, legs = _head_on(traffic, at)
    old = legs["b"].route
    at(10.0 + traffic.params.floor_replan_s + 0.1)
    legs["a"].no_way_since = 11.1  # a가 돌 길을 찾다 못 찾았다
    traffic._extend(b, legs["b"])
    traffic._replan_if_stuck(b, legs["b"], False)
    assert legs["b"].route is not old


def test_a_failed_way_around_is_remembered_until_the_lock_grows(monkeypatch):
    traffic, at = floor({"a": (1.6, 0.8), "b": (1.95, 0.8)})
    a, b, legs = _head_on(traffic, at)
    monkeypatch.setattr(traffic, "_route", lambda *args, **kw: None)  # 어디로도 못 돈다
    at(10.0 + traffic.params.floor_replan_s + 0.1)
    traffic._extend(a, legs["a"])
    traffic._replan_if_stuck(a, legs["a"], False)
    assert legs["a"].no_way_since == 10.0 + traffic.params.floor_replan_s + 0.1
    traffic._replan_if_stuck(a, legs["a"], True)  # 잠금이 늘었다
    assert legs["a"].no_way_since is None


def test_a_new_route_clears_the_failed_way_around():
    """다른 계기로 새 길을 받으면 예전에 길을 못 찾은 기록은 지운다(상대가 헛짚지 않게)."""
    traffic, _ = floor({"a": (1.6, 0.8)})
    a = traffic._missions["a"]
    leg = driving(traffic, "a", (2.6, 0.8))
    leg.no_way_since = 5.0
    traffic._plan(a, leg)
    assert leg.route is not None and leg.no_way_since is None


def test_headings_reach_the_end_of_a_short_last_piece():
    """꺾은선 끝까지 방향을 적는다(마지막 조각이 표본 간격의 배수가 아니어도)."""
    traffic, _ = floor({"a": (1.0, 0.8)})
    end = (1.18, 0.8)
    heads = traffic._headings([(1.0, 0.8), end], 5.0)
    assert set(disk_cells(traffic.floor, *end, traffic._body)) <= set(heads)


def test_headings_keep_every_way_a_cell_is_passed():
    """되돌아오는 길은 같은 칸을 두 방향으로 지난다: 두 방향을 다 적는다."""
    traffic, _ = floor({"a": (1.0, 0.8)})
    heads = traffic._headings([(1.0, 0.8), (1.5, 0.8), (1.2, 0.8)], 5.0)
    cell = traffic.floor.cell_of(1.35, 0.85)
    xs = [d[0] for d in heads[cell]]
    assert max(xs) > 0.9 and min(xs) < -0.9


def test_headings_reach_the_window_end():
    """창(limit) 끝 지점까지 방향을 적는다."""
    traffic, _ = floor({"a": (1.0, 0.8)})
    heads = traffic._headings([(1.0, 0.8), (3.0, 0.8)], 0.18)
    assert set(disk_cells(traffic.floor, 1.18, 0.8, traffic._body)) <= set(heads)


def test_a_route_that_turns_back_meets_a_robot_head_on():
    """내 길이 a와 같은 쪽으로 갔다가 되돌아온다: 되돌아오는 쪽에서 a와 마주 온다."""
    traffic, _ = floor({"a": (1.3, 0.62), "b": (1.0, 0.8)})
    driving(traffic, "a", (2.9, 0.62))
    assert traffic._passing("a")
    assert traffic._oncoming("b", Route([(1.0, 0.8), (2.0, 0.8), (1.5, 0.8)])) == {"a"}
    assert traffic._oncoming("b", Route([(1.0, 0.8), (2.0, 0.8)])) == set()


def test_a_waiting_leg_stuck_behind_a_passing_robot_waits_before_it_picks_another_spot(monkeypatch):
    """대기 자리로 가다 지나가는 로봇에 막혀도 floor_pass_wait_s 서서 기다린 뒤에 다른 자리를 고른다."""
    traffic, at = floor({"a": (2.3, 0.45), "b": (2.6, 0.8)})
    driving(traffic, "a", (2.3, 1.15))
    b, leg = _stuck_behind(traffic, at)
    leg.waiting = True
    picked = []
    monkeypatch.setattr(traffic, "_plan", lambda m, leg: picked.append(traffic._clock()))
    wait = traffic.params.floor_pass_wait_s
    for t in (11.1, 12.2, 10.0 + wait - 0.1):
        _tick(traffic, at, b, leg, t)
    assert picked == []
    _tick(traffic, at, b, leg, 10.0 + wait + 0.1)
    assert picked == [10.0 + wait + 0.1]


def test_a_waiting_leg_stuck_behind_a_standing_robot_picks_another_spot_as_before(monkeypatch):
    traffic, at = floor({"a": (2.3, 0.45), "b": (2.6, 0.8)})
    driving(traffic, "a", (2.3, 1.15))
    standing(traffic, "a")
    b, leg = _stuck_behind(traffic, at)
    leg.waiting = True
    picked = []
    monkeypatch.setattr(traffic, "_plan", lambda m, leg: picked.append(traffic._clock()))
    _tick(traffic, at, b, leg, 10.0 + traffic.params.floor_replan_s + 0.1)
    assert picked == [10.0 + traffic.params.floor_replan_s + 0.1]


def test_after_an_unresolved_deadlock_the_chosen_robot_takes_a_long_way_around(monkeypatch):
    """비켜 설 자리도 없는 막힘(못 풂)으로 고른 로봇은 격리보다 먼 길이 낫다: +3 m 제한 없이 돈다.
    다른 로봇은 예전 제한 그대로다."""
    traffic, at = floor({"a": (1.6, 0.8), "b": (1.95, 0.8)})
    a, b, legs = _head_on(traffic, at)
    far = {n: Route([(m.x, m.y), (m.x, 1.6), (m.x + 3.0, 1.6), (m.x + 3.0, 0.4), legs[n].goal_xy])
           for n, m in (("a", a), ("b", b))}
    for n in far:
        assert far[n].length > legs[n].route.length - legs[n].s + 3.0
    monkeypatch.setattr(traffic, "_route", lambda m, *args, **kw: far[m.name])
    t = 10.0 + traffic.params.floor_pass_wait_s + 0.1  # 둘 다 다시 계획할 때
    at(t)
    for m in (a, b):
        traffic._extend(m, legs[m.name])
        traffic._replan_if_stuck(m, legs[m.name], False)
    assert legs["a"].route is not far["a"] and legs["b"].route is not far["b"]
    traffic._unresolved_at, traffic._unresolved_robot = t, "b"
    at(t + traffic.params.floor_replan_s + 0.1)
    for m in (a, b):
        traffic._extend(m, legs[m.name])
        traffic._replan_if_stuck(m, legs[m.name], False)
    assert legs["b"].route is far["b"] and legs["a"].route is not far["a"]


def test_the_last_resort_ends_with_the_cycle():
    """순환이 사라지면 못 풂 표시도 지운다: 나중의 다른 막힘에서 먼 길을 타지 않는다."""
    traffic, at = floor({"a": (1.6, 0.8), "b": (1.95, 0.8)})
    traffic._deadlock_key, traffic._deadlock_seen = ("a", "b"), 5.0
    traffic._unresolved_at, traffic._unresolved_robot = 5.0, "b"
    traffic._check_deadlock(6.5)  # 순환 없이 1초가 지났다
    assert traffic._unresolved_at is None
