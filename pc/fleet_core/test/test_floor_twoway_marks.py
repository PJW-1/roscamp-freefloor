"""arena_v4_twoway의 방향 표시(관제).

통로마다 (방향, 쥔 로봇들). 잠금이 통로 칸에 들어가기 전에 표시를 잡는다: 비었거나 같은 방향이면 잡고,
반대 방향이면 입구 밖에서 선다. 몸이 통로 칸을 다 벗어나고 다리가 그 통로를 더 지나지 않으면 놓는다.
격리된 로봇은 칸과 함께 표시도 쥔다. 반대쪽에서 먼저 기다린 로봇이 다음 차례다(공평). 표시는 칸 잠금을
대신하지 않는다. 기존 arena_v4 실행은 사건과 명령이 한 글자도 바뀌지 않는다.
"""
import math
from dataclasses import replace
from pathlib import Path

import compare_floor
import pytest
from fleet_core.common.params import load_params
from fleet_core.manager import FleetManager
from fleet_core.traffic.lane_map import load_lane_map
from test_floor_arena_v4 import LOCKS as V4_LOCKS
from test_floor_arena_v4 import THREE, drive_on, scenario
from test_floor_retreat import digest, run

REPO = Path(__file__).resolve().parents[3]
LANES = load_lane_map(REPO / "sim" / "fake_robots" / "maps" / "arena_v4_twoway.yaml")
LOCKS = {"floor_lock_ahead_m": 0.15, "floor_extend_step_m": 0.05, "release_distance_m": 0.17}
WEST_N = (0.34, 0.80, 0.0)      # 서쪽 북쪽, 물체 3 북쪽 문 앞(몸이 통로 칸 밖)
WEST_N2 = (0.30, 0.55, 0.0)  # 서쪽, WEST_N 남쪽(몸 칸이 겹치지 않는다)
EAST_S = (1.55, 0.55, math.pi)  # 동쪽, 물체 4 북쪽 통로 동쪽 입구에서 떨어진 곳


def fleet(poses, **over):
    params = replace(load_params(), traffic_mode="freefloor", **{**LOCKS, **over})
    now = [0.0]
    core = FleetManager(LANES, params, list(poses), lambda: now[0])
    for _ in range(3):
        for name, (x, y, yaw) in poses.items():
            core.on_robot_state(name, x, y, yaw, "IDLE")
    traffic = core.traffic
    assert all(m.placed for m in traffic._missions.values())  # 시작 자리가 서로 겹치지 않는다
    traffic._plan_budget = 99
    traffic.station_left = lambda name: None

    def tick():
        # 한 턴에 0.1 s: 관제는 입력 때 읽은 시각을 쓰므로 그 시각도 민다(다시 계획하기가 돈다)
        now[0] += 0.1
        core._time = now[0]

    traffic.test_tick = tick
    return traffic


def go(traffic, name, phase, goal, blind=False):
    """name이 goal로 떠나 한 번 달린다. blind: 다른 로봇의 다리를 모르는 채 계획한다(먼저 계획한
    길 위로 나중에 다른 로봇이 온 것과 같다: 마주 오는 로봇을 피하는 계획이 보지 못한 경우)."""
    m = traffic._missions[name]
    if blind:
        seen = traffic._oncoming, traffic._wait_for
        traffic._oncoming, traffic._wait_for = (lambda *a: set()), (lambda *a: None)
    traffic._start_leg(m, phase, "OUTBOUND" if goal == "O" else None, goal)
    if blind:
        traffic._oncoming, traffic._wait_for = seen
    traffic._drive(m)
    return traffic._legs[name]


def step_all(traffic, names, turns=400, each=None):
    """이름들이 한 걸음씩 돌아가며 달린다. 모두 다리를 마치면 그 턴 수."""
    for k in range(turns):
        traffic.test_tick()
        for name in names:
            traffic._plan_budget = 99
            drive_on(traffic, name)
        traffic._release_marks()
        if each is not None:
            each(k)
        if all(name not in traffic._legs for name in names):
            return k
    return turns


def holders(traffic, passage):
    way, who = traffic._mark.get(passage, (0, set()))
    return way, set(who)


def body(traffic, name):
    return traffic._body_cells(*traffic._here(traffic._missions[name]))


def test_a_route_through_the_doors_names_the_passages_it_crosses_and_their_way():
    traffic = fleet({"a": WEST_N})
    a = go(traffic, "a", "TO_PARK", "P-1")
    (s_in, _, ways_a), = traffic._passes(a.route)
    assert dict(ways_a) == {"north3": 1, "north4": 1}  # 위 합류점의 mid_n은 스칠 뿐
    traffic = fleet({"b": EAST_S})
    b = go(traffic, "b", "TO_OUTBOUND", "O")
    (s_in, s_out, ways_b), = traffic._passes(b.route)
    assert dict(ways_b) == {"north3": -1, "north4": -1}
    assert 0.0 < s_in < s_out < b.route.length
    # 묶음 앞 설 자리에서는 몸(여유 포함)이 통로 칸에 닿지 않는다
    stop = traffic._ids(__import__("fleet_core.traffic.floor.cells", fromlist=["disk_cells"])
                        .disk_cells(traffic.floor, *b.route.at(s_in), traffic._body + 0.06))
    assert stop.isdisjoint(traffic._passage_ids["north4"] | traffic._passage_ids["north3"])


def test_a_robot_coming_the_other_way_is_kept_apart_by_the_mark_not_by_the_plan():
    """남쪽 통로로 도는 길은 작업대에서 나오는 선(work_out) 칸을 비켜 가야 해서 길다: 마주 오는 a를 보는
    계획도 북쪽 통로로 간다. 그래서 마주침은 계획이 아니라 표시가 막는다."""
    traffic = fleet({"a": WEST_N, "b": EAST_S})
    go(traffic, "a", "TO_PARK", "P-1")
    b = go(traffic, "b", "TO_OUTBOUND", "O")
    (_, _, ways_b), = traffic._passes(b.route)
    assert dict(ways_b) == {"north3": -1, "north4": -1}


def test_the_existing_floor_has_no_passage_and_no_mark():
    traffic = fleet({"a": (1.57, 0.87, 0.0)})
    v4 = FleetManager(load_lane_map(REPO / "sim" / "fake_robots" / "maps" / "arena_v4.yaml"),
                      replace(load_params(), traffic_mode="freefloor"), ["a"],
                      lambda: 0.0).traffic
    assert traffic._passage_ids and not v4._passage_ids
    assert "mark_waits" in traffic.metrics and "mark_waits" not in v4.metrics


def test_two_robots_meeting_in_a_single_file_passage_never_both_enter_and_both_get_through():
    traffic = fleet({"a": WEST_N, "b": EAST_S})
    go(traffic, "a", "TO_PARK", "P-1")
    step_all(traffic, ["a"], turns=8)
    assert holders(traffic, "north3") == (1, {"a"})
    b = go(traffic, "b", "TO_OUTBOUND", "O", blind=True)
    assert dict(traffic._passes(b.route)[0][2]) == {"north3": -1, "north4": -1}
    doors = traffic._passage_ids["north4"] | traffic._passage_ids["north3"]
    seen = []

    def each(_):
        way, who = holders(traffic, "north4")
        seen.append((way, who))
        if "a" in who:  # a가 쥔 동안 b의 칸은 통로 칸에 닿지 않는다
            assert traffic.zones.held_set("b").isdisjoint(doors)

    step_all(traffic, ["a", "b"], each=each)
    assert "a" not in traffic._legs and traffic._missions["a"].phase == "PARKED"
    assert traffic._missions["b"].phase == "DOCKING"
    assert (-1, {"b"}) in seen  # b가 a 다음에 반대로 지났다
    assert traffic.metrics["mark_waits"] >= 1
    assert traffic.metrics["mark_switches"] == 0  # a가 곧 나간다: 먼 길로 바꾸지 않았다


def test_robots_the_same_way_follow_each_other_into_a_passage():
    traffic = fleet({"a": WEST_N, "c": WEST_N2})
    go(traffic, "a", "TO_PARK", "P-2")  # 먼 자리부터: 먼저 선 로봇이 뒤 로봇의 길을 막지 않는다
    go(traffic, "c", "TO_PARK", "P-1")
    seen = []

    def each(_):
        seen.append(holders(traffic, "north4"))  # c는 물체 2·3 사이 통로로 와서 물체 4 북쪽 통로에 a를 따라 든다
        # 표시는 칸 잠금을 대신하지 않는다: 함께 쥐어도 칸은 따로다
        assert traffic.zones.held_set("a").isdisjoint(traffic.zones.held_set("c"))

    step_all(traffic, ["a", "c"], each=each)
    assert (1, {"a", "c"}) in seen
    assert traffic._missions["a"].phase == traffic._missions["c"].phase == "PARKED"


def test_a_robot_waiting_the_other_way_goes_before_a_later_one_the_same_way_as_the_holder():
    """공평: 반대쪽에서 기다리기 시작한 뒤에 온 같은 방향 로봇은 새로 들어가지 않는다."""
    traffic = fleet({"a": WEST_N, "b": EAST_S, "c": WEST_N2})
    go(traffic, "a", "TO_PARK", "P-2")
    step_all(traffic, ["a"], turns=8)
    go(traffic, "b", "TO_OUTBOUND", "O", blind=True)
    step_all(traffic, ["a", "b"], turns=2)
    assert "b" in traffic._mark_wait.get("north4", {})
    go(traffic, "c", "TO_PARK", "P-1", blind=True)
    first = {}

    def each(k):
        for name in ("b", "c"):
            if name in holders(traffic, "north4")[1]:
                first.setdefault(name, k)

    step_all(traffic, ["a", "b", "c"], turns=600, each=each)
    assert first["b"] < first["c"]
    assert traffic._missions["c"].phase == "PARKED"  # 굶지 않고 다음에 지나갔다


def test_a_mark_is_let_go_once_the_body_is_out_of_the_passage():
    traffic = fleet({"a": WEST_N})
    go(traffic, "a", "TO_PARK", "P-1")
    trace = []
    step_all(traffic, ["a"], each=lambda _: trace.append(
        ("a" in holders(traffic, "north3")[1],
         not body(traffic, "a").isdisjoint(traffic._passage_ids["north3"]))))
    took = [k for k, (mine, _) in enumerate(trace) if mine]
    assert took and any(inside for _, inside in trace)
    # 몸이 통로 칸에 닿는 동안은 늘 쥐었고, 놓은 뒤에는 몸이 통로 칸 밖이다
    assert all(mine for mine, inside in trace if inside)
    assert not trace[took[-1] + 1][1]
    assert holders(traffic, "north3") == (0, set())


def test_a_robot_whose_new_leg_does_not_cross_the_passage_lets_the_mark_go():
    traffic = fleet({"a": WEST_N})
    go(traffic, "a", "TO_PARK", "P-1")
    step_all(traffic, ["a"], turns=1)
    assert "a" in holders(traffic, "north3")[1]
    assert body(traffic, "a").isdisjoint(traffic._passage_ids["north3"])  # 아직 입구 밖
    traffic._start_leg(traffic._missions["a"], "TO_OUTBOUND", "OUTBOUND", "O")
    traffic._release_marks()
    assert "a" not in holders(traffic, "north3")[1]


def test_an_isolated_robot_keeps_its_mark_with_its_cells():
    traffic = fleet({"a": WEST_N, "b": EAST_S})
    go(traffic, "a", "TO_PARK", "P-1")
    leg = traffic._legs["a"]
    for _ in range(200):
        drive_on(traffic, "a")
        traffic._release_marks()
        span = leg.route.fixed_at(leg.s)
        if span is not None and span[2] == "north3":
            break
    assert leg.route.fixed_at(leg.s)[2] == "north3"  # 문 안
    traffic._fail(traffic._missions["a"], "TEST")
    for _ in range(20):
        traffic._release_marks()
    assert holders(traffic, "north3") == (1, {"a"})
    go(traffic, "b", "TO_OUTBOUND", "O", blind=True)
    step_all(traffic, ["b"], turns=60)
    assert "b" not in holders(traffic, "north3")[1]
    assert traffic.zones.held_set("b").isdisjoint(traffic._passage_ids["north3"])


def isolated_in_north3(**over):
    """b가 반대쪽에서 출고로 떠난 뒤(길이 물체 3 북쪽 통로를 지난다) a가 그 통로 안에서 격리된다."""
    traffic = fleet({"a": WEST_N, "b": EAST_S}, **over)
    go(traffic, "a", "TO_PARK", "P-1")
    b = go(traffic, "b", "TO_OUTBOUND", "O", blind=True)  # a가 아직 통로 밖일 때 계획했다
    assert "north3" in {p for _, _, ways in traffic._passes(b.route) for p, _ in ways}
    leg = traffic._legs["a"]
    for _ in range(200):
        drive_on(traffic, "a")
        traffic._release_marks()
        span = leg.route.fixed_at(leg.s)
        if span is not None and span[2] == "north3":
            break
    assert holders(traffic, "north3") == (1, {"a"})
    traffic._fail(traffic._missions["a"], "TEST")
    return traffic


def test_a_robot_waiting_for_a_mark_that_will_not_come_takes_the_other_way():
    """격리된 로봇이 쥔 표시는 풀리지 않는다(남은 시간이 끝이 없다): 다른 통로로 돌아간다."""
    traffic = isolated_in_north3()
    seen = []
    step_all(traffic, ["b"], turns=500, each=lambda _: seen.append(holders(traffic, "north3")))
    assert traffic.metrics["mark_switches"] >= 1
    assert traffic._missions["b"].phase == "DOCKING"
    assert all("b" not in who for _, who in seen)
    assert holders(traffic, "north3") == (1, {"a"})


@pytest.mark.parametrize("gain, switches", [(3.0, True), (10.0, False)])
def test_a_robot_switches_only_when_the_other_way_wins_by_the_switch_gain(gain, switches):
    """쥔 로봇이 2 m 뒤에 나간다고 보면(0.2 m/s로 10 s) 기다림의 도착은 10 s + 남은 길이다. 다른 길이
    그보다 이득(gain) 넘게 빠를 때만 바꾼다: 3 s면 바꾸고 10 s면 끝까지 기다린다(흔들리지 않는다)."""
    traffic = isolated_in_north3(floor_mark_switch_gain_s=gain)
    traffic._mark_left = lambda name, passage: 2.0
    step_all(traffic, ["b"], turns=120)
    assert (traffic.metrics["mark_switches"] >= 1) == switches
    if not switches:
        assert traffic._missions["b"].phase == "TO_OUTBOUND"
        assert traffic._at_mark(traffic._legs["b"])


def test_the_switch_gain_is_a_parameter_in_both_parameter_files():
    assert load_params().floor_mark_switch_gain_s == 3.0
    for name in ("fleet_params.yaml", "fleet_params_real_arena.yaml"):
        text = (REPO / "pc" / "fleet_core" / "fleet_core" / "data" / name).read_text(encoding="utf-8")
        assert "floor_mark_switch_gain_s: 3.0" in text, name


# ---- 기존 지도는 그대로 ----------------------------------------------------------------------

OBJECT = {"type": "object", "at_s": 20.0, "duration_s": 120.0, "x": 0.97, "y": 0.85,
          "radius_m": 0.04}
V4_RUNS = {  # 바꾸기 전(b2c4632)에 구한 사건과 명령 sha256의 앞 16자리
    "three6_off": ("aadf1e7cefa0efd0", 3, 6, 20.0, [], False),
    "three6_on": ("a7e6342e8280cd99", 3, 6, 20.0, [], True),
    "two4_off": ("3c6a4e2b213383af", 2, 4, 10.0, [], False),
    "three4_obj": ("c21aff224481d856", 3, 4, 20.0, [OBJECT], False),
}


@pytest.mark.parametrize("name", sorted(V4_RUNS))
def test_arena_v4_runs_exactly_as_before(name):
    want, robots, count, period, faults, shortcut = V4_RUNS[name]
    data = scenario(robots=THREE[:robots], count=count, period_s=period, timeout_s=600)
    data["faults"] = faults
    compare_floor.OVERRIDES = {**V4_LOCKS, "floor_shortcut": shortcut}
    try:
        world = compare_floor.build(data, "arena_v4", 1, "freefloor")
    finally:
        compare_floor.OVERRIDES = {}
    run(world, 400)
    assert digest(world)[:16] == want


# ---- 머리 없는 실행 ---------------------------------------------------------------------------


def test_three_pinkys_ship_six_orders_on_the_twoway_floor_and_use_the_doors():
    data = scenario(robots=THREE, count=6, timeout_s=600)
    compare_floor.OVERRIDES = dict(LOCKS)
    try:
        world = compare_floor.build(data, "arena_v4_twoway", 1, "freefloor")
    finally:
        compare_floor.OVERRIDES = {}
    traffic = world.core.traffic
    ways = set()
    while world.t < 600:
        world.step()
        ways |= {(p, way) for p, (way, who) in traffic._mark.items() if who}
        if world.all_orders_final() and world.all_parked():
            break
    assert sum(r.status == "SHIPPED" for r in world.reports) == 6
    assert world.cell_violations == 0
    assert not [e for e in world.events if e.code in ("ROBOT_ISOLATED", "DEADLOCK_UNRESOLVED")]
    assert {"gap23", "north3"} & {p for p, _ in ways}
