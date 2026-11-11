"""필요할 때만 우측통행(floor_keep_right, 제안, 기본 꺼짐), arena_v5에서.

두 줄이 서는 통로(빈 칸 여덟 줄: 남쪽 통로 전체, 북쪽 통로의 물체 3 옆)를 두 로봇이 마주 보고 같은
때에 지나게 되면 관제가 두 로봇을 각자 진행 방향의 오른쪽 끝 줄로 보내 나란히 지나가게 한다.
두 줄이 서지 않는 통로(북쪽 통로의 물체 1 옆 일곱 줄, 27 cm 목, 섬 사이)에서는 한 대가 그 통로
밖에서 기다린다. 혼자 지나가는 로봇은 예전처럼 가장 짧은 길로 간다. 끄면 예전과 같다."""
import collections
import math
import sys
from dataclasses import replace
from pathlib import Path

import pytest
import yaml
from fleet_core.common.commands import MoveCmd
from fleet_core.common.params import load_params
from fleet_core.traffic.floor.floor_map import floor_map_for
from fleet_core.traffic.floor.lanes import passages
from fleet_core.traffic.lane_map import load_lane_map

REPO = Path(__file__).resolve().parents[3]
LANES = load_lane_map(REPO / "sim" / "fake_robots" / "maps" / "arena_v5.yaml")
FLOOR = floor_map_for(LANES)
LOCKS = {"floor_lock_ahead_m": 0.15, "floor_extend_step_m": 0.05, "release_distance_m": 0.17}
THREE = (("pinky1", "P-3"), ("pinky2", "P-2"), ("pinky3", "P-1"))
KINDS = [{"A": 1, "C": 1}, {"B": 1}, {"D": 1, "A": 1}, {"C": 1}, {"B": 1, "D": 1}, {"A": 1}]
SOUTH = (0.52, 1.30, 0.42)  # 남쪽 통로: x0, x1, 이 y 아래
NORTH_1 = (0.52, 0.69, 0.58)  # 물체 1 북쪽: x0, x1, 이 y 위

sys.path.insert(0, str(REPO / "tools"))
import compare_floor  # noqa: E402
from floor_world import FloorWorld  # noqa: E402  (compare_floor가 테스트 폴더를 경로에 넣는다)

# 남쪽 통로를 마주 지난다: pinky1은 서쪽 구역에서 동쪽 끝(SE)으로, pinky2는 동쪽 마당에서 서쪽 끝(SW)으로
SOUTH_POSES = {"pinky1": (0.25, 0.20), "pinky2": (1.55, 0.45)}
SOUTH_GOALS = {"pinky1": "SE", "pinky2": "SW"}


def world(poses, keep_right=True, noise=0.0, seed=1, faults=()):
    robots = {n: {"start": s, "speed_mps": 0.15, "pose_noise_m": noise} for n, s in THREE}
    w = FloorWorld(seed=seed, orders=[], robots=robots, site=LANES, floor=FLOOR,
                   faults=[dict(f) for f in faults],
                   params=replace(load_params(), floor_keep_right=keep_right, **LOCKS))
    for name, (x, y) in poses.items():
        w.virtual[name].x, w.virtual[name].y = x, y
    traffic = w.core.traffic
    traffic._spot_taken = lambda m: False  # 아무 지점으로나 보낸다(주차 자리 규칙 없이)
    w._report_states()
    return w, traffic


def send(traffic, goals):
    traffic._plan_budget = 99
    for name, waypoint in goals.items():
        traffic._start_leg(traffic._missions[name], "TO_PARK", None, waypoint)


def in_south(r) -> bool:
    return SOUTH[0] <= r.x <= SOUTH[1] and r.y < SOUTH[2]


def in_north_1(r) -> bool:
    return NORTH_1[0] <= r.x <= NORTH_1[1] and r.y > NORTH_1[2]


def abreast(a, b) -> bool:
    """둘 다 남쪽 통로 안에서 서로 반대로 가며 x가 0.25 m 안이다(나란히 지난다)."""
    return (in_south(a) and in_south(b) and abs(a.x - b.x) < 0.25
            and math.cos(a.heading) * math.cos(b.heading) < -0.25)


def drive(w, traffic, goals, seconds=60.0, hook=None):
    """두 로봇이 goals에 닿을 때까지(한 번이라도 0.05 m 안) 돌리며 본다.
    돌려주는 것: 나란히 지난(abreast) 단계 수, 둘 다 물체 1 북쪽에 있던 단계 수,
    로봇마다 출발 뒤 닿기 전까지 가장 오래 서 있던 시간, 다 닿았나."""
    names = sorted(goals)
    target = {n: LANES.waypoint_pose(goals[n]) for n in names}
    reached = {n: False for n in names}
    moved = {n: False for n in names}
    still = collections.Counter()
    longest = collections.Counter()
    last = {n: (w.virtual[n].x, w.virtual[n].y) for n in names}
    side = north = 0
    end = w.t + seconds
    while w.t < end and not all(reached.values()):
        w.step()
        if hook is not None:
            hook(w)
        if len(names) == 2:
            a, b = (w.virtual[n] for n in names)
            if abreast(a, b):
                side += 1
            if in_north_1(a) and in_north_1(b):
                north += 1
        for n in names:
            r = w.virtual[n]
            here = (r.x, r.y)
            if math.dist(here, target[n]) <= 0.05:
                reached[n] = True
            step = math.dist(here, last[n])
            last[n] = here
            if step > 1e-4:
                moved[n] = True
                still[n] = 0.0
            elif moved[n] and not reached[n]:
                still[n] += 0.1
                longest[n] = max(longest[n], still[n])
    return side, north, dict(longest), all(reached.values())


def codes(w) -> list:
    return [e.code for e in w.events]


# ---- 지도에서 구한 통로 ----------------------------------------------------------------------


def test_the_passages_with_two_lanes_are_the_south_passage_and_the_north_one_beside_object_3():
    """빈 칸 여덟 줄인 곳만 두 줄이다. 물체 1 북쪽(일곱 줄)과 27 cm 목, 섬 사이(다섯 칸)는 아니다."""
    found = passages(FLOOR, 0.09, 10)
    pairs = [p for p in found if p.pair]
    xs = sorted((round(min(FLOOR.centre(c)[0] for c in p.pair), 3),
                 round(max(FLOOR.centre(c)[0] for c in p.pair), 3),
                 round(min(FLOOR.centre(c)[1] for c in p.pair), 3)) for p in pairs)
    assert xs == [(0.435, 0.765, 0.105), (0.945, 1.395, 0.105), (0.945, 1.395, 0.615)]
    for x, y in ((0.60, 0.80), (0.53, 0.72), (0.86, 0.50)):  # 물체 1 북쪽, 목, 섬 사이
        cell = FLOOR.cell_of(x, y)
        assert any(cell in p.cells for p in found), (x, y)
        assert not any(cell in p.pair for p in found), (x, y)
    south = next(p for p in pairs if (FLOOR.cell_of(0.60, 0.105) in p.pair))
    # 동쪽으로 가는 로봇의 줄은 남쪽 벽 쪽(y 0.105), 서쪽으로 가는 로봇의 줄은 물체 1 쪽(y 0.315)
    assert {round(FLOOR.centre(c)[1], 3) for c in south.lo} == {0.105}
    assert FLOOR.cell_of(0.60, 0.315) in south.hi


def test_the_switch_is_off_by_default():
    assert load_params().floor_keep_right is False
    _, traffic = world(SOUTH_POSES, keep_right=False)
    assert not traffic._keep_right and "keep_right" not in traffic.metrics


# ---- 1. 남쪽 통로에서 나란히 -------------------------------------------------------------------


@pytest.mark.parametrize("noise", [0.0, 0.02])
def test_two_robots_meeting_in_the_south_passage_pass_side_by_side(noise):
    w, traffic = world(SOUTH_POSES, noise=noise)
    send(traffic, SOUTH_GOALS)
    side, _, longest, reached = drive(w, traffic, SOUTH_GOALS)
    assert reached, longest
    assert side >= 5, side  # 0.5 s 넘게 나란히
    assert max(longest.values(), default=0.0) < 3.0, longest
    assert traffic.metrics["keep_right"] >= 1
    assert w.cell_violations == 0
    assert "DEADLOCK_UNRESOLVED" not in codes(w), codes(w)


def test_with_the_switch_off_the_same_two_robots_are_never_side_by_side_in_the_passage():
    w, traffic = world(SOUTH_POSES, keep_right=False)
    send(traffic, SOUTH_GOALS)
    side, _, _, reached = drive(w, traffic, SOUTH_GOALS)
    assert reached
    assert side == 0


def test_each_robot_keeps_to_the_lane_on_its_right():
    """나란히 지나는 동안 동쪽으로 가는 로봇은 남쪽 벽 쪽, 서쪽으로 가는 로봇은 물체 쪽에 있다."""
    w, traffic = world(SOUTH_POSES)
    send(traffic, SOUTH_GOALS)
    seen = []

    def look(world_now):
        a, b = world_now.virtual["pinky1"], world_now.virtual["pinky2"]
        if in_south(a) and in_south(b):
            seen.append((a.y, b.y))
    drive(w, traffic, SOUTH_GOALS, hook=look)
    assert seen
    assert all(ya < 0.15 and yb > 0.27 for ya, yb in seen), seen[:5]


# ---- 2. 물체 1 북쪽에서는 한 대가 기다린다 ------------------------------------------------------

# pinky1은 물체 1 북쪽 서쪽 입구에서 동쪽(NM)으로, pinky2는 물체 3 북쪽에서 서쪽(QO)으로 와 물체 1 북쪽에서
# 마주친다. 스위치를 끄면 이 둘은 통로 안에서 마주 서 DEADLOCK_UNRESOLVED가 난다
NORTH_POSES = {"pinky1": (0.36, 0.80), "pinky2": (1.25, 0.78)}
NORTH_GOALS = {"pinky1": "NM", "pinky2": "QO"}


def test_robots_meeting_north_of_object_1_do_not_pass_side_by_side_one_waits():
    w, traffic = world(NORTH_POSES)
    send(traffic, NORTH_GOALS)
    _, north, longest, reached = drive(w, traffic, NORTH_GOALS, seconds=90.0)
    assert reached, longest
    assert north == 0
    assert traffic.metrics["keep_right"] == 0
    assert traffic.metrics["narrow_waits"] >= 1
    assert max(longest.values()) >= 1.0, longest  # 한 대가 섰다
    assert w.cell_violations == 0
    assert "DEADLOCK_UNRESOLVED" not in codes(w), codes(w)


# 기다릴 pinky1이 이미 pinky2의 길 위에 서 있어 비켜 선다
SIDESTEP_POSES = {"pinky1": (0.20, 0.76), "pinky2": (0.80, 0.78)}
SIDESTEP_GOALS = {"pinky1": "NE", "pinky2": "QO"}


def test_a_waiter_that_steps_aside_never_sends_its_old_leg():
    w, traffic = world(SIDESTEP_POSES)
    stale = []
    for fn in ("_extend", "_send"):
        real = getattr(traffic, fn)

        def check(m, leg, *args, _real=real, **kwargs):
            if traffic._legs.get(m.name) is not leg:
                stale.append((round(w.t, 1), m.name))
            return _real(m, leg, *args, **kwargs)
        setattr(traffic, fn, check)
    send(traffic, SIDESTEP_GOALS)
    _, _, longest, reached = drive(w, traffic, SIDESTEP_GOALS, seconds=90.0)
    assert "FLOOR_SIDESTEP" in codes(w), codes(w)
    assert not stale, stale
    assert reached, longest
    assert w.cell_violations == 0
    assert "DEADLOCK_UNRESOLVED" not in codes(w), codes(w)


# ---- 3. 혼자면 예전과 같다 ---------------------------------------------------------------------


def moves(w) -> list:
    return [(c.robot, c.x, c.y, c.yaw, c.path) for _, c in w.commands if isinstance(c, MoveCmd)]


@pytest.mark.parametrize("start, goal", [((0.25, 0.20), "SE"), ((1.55, 0.45), "SW"),
                                         ((0.20, 0.76), "NE")])
def test_a_robot_alone_takes_the_same_way_with_the_switch_on(start, goal):
    runs = []
    for on in (False, True):
        w, traffic = world({"pinky1": start}, keep_right=on, noise=0.02)
        send(traffic, {"pinky1": goal})
        route = list(traffic._legs["pinky1"].route.points)
        _, _, _, reached = drive(w, traffic, {"pinky1": goal})
        assert reached
        runs.append((route, moves(w), codes(w)))
    assert runs[0] == runs[1]


# ---- 4. 지나가는 중 한 대가 막혀 선다 -----------------------------------------------------------


def both_inside(w) -> bool:
    return in_south(w.virtual["pinky1"]) and in_south(w.virtual["pinky2"])


@pytest.mark.parametrize("robot", ["pinky1", "pinky2"])
def test_a_robot_that_is_blocked_while_they_pass_leaves_no_wait_cycle_unresolved(robot):
    """나란히 지나기 시작한 뒤 한 대가 8 s 동안 물체 앞에서 선다(BLOCKED, 막힘 표시, 취소)."""
    w, traffic = world(SOUTH_POSES, noise=0.02)
    send(traffic, SOUTH_GOALS)
    assert w.run_until(lambda: both_inside(w), timeout_s=30.0)
    w.pending_faults.append({"at_s": w.t + 0.1, "type": "blocked", "robot": robot,
                             "duration_s": 8.0})
    _, _, _, reached = drive(w, traffic, SOUTH_GOALS, seconds=120.0)
    assert reached
    assert "FLOOR_BLOCKED" in codes(w)
    assert w.cell_violations == 0
    assert "DEADLOCK_UNRESOLVED" not in codes(w), codes(w)


def test_a_new_lane_whose_first_lock_would_end_inside_a_fixed_line_is_not_taken():
    """고정 선 구간 안에서 끝나는 첫 잠금은 잡지 않는다(그 선은 통째로 잡거나 그 앞에 선다)."""
    w, traffic = world(SOUTH_POSES)
    send(traffic, SOUTH_GOALS)
    from fleet_core.traffic.floor.controller import _Pass
    from fleet_core.traffic.floor.planner import Route
    routes = {}
    for n in SOUTH_GOALS:
        leg = traffic._legs[n]
        r = Route(list(leg.route.points), [(0.05, leg.route.length - 0.05, "X")])
        routes[n] = r
    p = _Pass(tuple(SOUTH_GOALS), {n: {} for n in SOUTH_GOALS})
    held = {n: set(traffic.zones.held_by(n)) for n in SOUTH_GOALS}
    before = {n: (traffic._legs[n].route, traffic._passes.get(n)) for n in SOUTH_GOALS}
    assert not traffic._apply_pass(p, routes)
    assert {n: set(traffic.zones.held_by(n)) for n in SOUTH_GOALS} == held
    assert {n: (traffic._legs[n].route, traffic._passes.get(n)) for n in SOUTH_GOALS} == before


def test_a_waiter_stops_waiting_when_the_robot_it_waits_for_is_blocked():
    """좁은 통로에서 기다리는 동안 먼저 가는 로봇이 오래 막히면(BLOCKED) 기다리기를 푼다."""
    w, traffic = world(NORTH_POSES)
    send(traffic, NORTH_GOALS)
    assert w.run_until(lambda: bool(traffic._yields), timeout_s=30.0)
    leader = next(iter(traffic._yields.values())).leader
    w.pending_faults.append({"at_s": w.t + 0.1, "type": "blocked", "robot": leader,
                             "duration_s": 40.0})
    assert w.run_until(lambda: traffic._missions[leader].robot_state == "BLOCKED",
                       timeout_s=10.0)
    assert w.run_until(lambda: not traffic._yields, timeout_s=2.0), traffic._yields
    assert w.cell_violations == 0


def test_an_object_in_one_lane_while_they_pass_leaves_no_wait_cycle_unresolved():
    """물체(지도에 없음)가 동쪽으로 가는 로봇의 줄 앞에 나타나 8 s 있다(핑키처럼 서고 자기 복구 뒤 BLOCKED)."""
    probe, traffic = world(SOUTH_POSES, noise=0.02)
    send(traffic, SOUTH_GOALS)
    assert probe.run_until(lambda: both_inside(probe), timeout_s=30.0)
    at, x = probe.t, probe.virtual["pinky1"].x
    fault = {"type": "object", "at_s": at, "duration_s": 8.0, "x": x + 0.2, "y": 0.105,
             "radius_m": 0.03}
    w, traffic = world(SOUTH_POSES, noise=0.02, faults=[fault])
    send(traffic, SOUTH_GOALS)
    _, _, _, reached = drive(w, traffic, SOUTH_GOALS, seconds=120.0)
    assert reached
    assert w.cell_violations == 0
    assert "DEADLOCK_UNRESOLVED" not in codes(w), codes(w)


# ---- 5. 주문 전체 주행 -------------------------------------------------------------------------


def scenario(count=8, period_s=20.0, noise_m=0.03, timeout_s=600):
    return {
        "map": "arena_v5", "seed": 1, "speed_factor": 5.0, "timeout_s": timeout_s,
        "stock": {"arm1": {"A": 40, "B": 40}, "arm2": {"C": 40, "D": 40}},
        "orders": [{"at_s": period_s * k, "id": f"R-{k + 1:03d}", "items": KINDS[k % 6]}
                   for k in range(count)],
        "robots": {name: {"start": spot, "speed_mps": 0.15, "pose_noise_m": noise_m}
                   for name, spot in THREE},
        "arm": {"seconds_per_item": 4.0, "swap_seconds": 8.0, "fail_rate": 0.0},
        "dock": {"duration_s": 3.0, "error_cm": 1.0}, "faults": [],
        "expect": {"orders_shipped": count},
    }


@pytest.mark.parametrize("seed", [1, 2, 3, 4])
def test_three_pinkys_ship_eight_orders_with_the_switch_on(tmp_path, monkeypatch, seed):
    """test_floor_arena_v5의 전체 주행과 같은 주문을 스위치를 켜고."""
    path = tmp_path / "arena_v5_keep_right.yaml"
    path.write_text(yaml.safe_dump(scenario()), encoding="utf-8")
    monkeypatch.setattr(compare_floor, "OVERRIDES", {**LOCKS, "floor_keep_right": True})
    row, world_ = compare_floor.run("arena_v5", str(path), seed, "freefloor")
    assert world_ is not None, row["error"]
    assert row["finished"], {k: row[k] for k in ("error", "end_s", "orders_shipped")}
    assert row["orders_shipped"] == 8
    assert row["unresolved"] == 0 and row["isolations"] == 0
    assert world_.cell_violations == 0
    assert row["min_gap_m"] >= 2 * load_params().robot_radius_m
