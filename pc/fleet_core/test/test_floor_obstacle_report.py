"""막힘 물체 자리 보고(floor_obstacle_report, RobotState obstacle_*, 제안, 팀 합의 전).

로봇이 BLOCKED(와 RECOVERING)에 본 물체의 자리와 반지름을 보내면 관제는 물체 둘레 칸만 막고
그 밖으로 다시 계획한다. 스위치를 끄면 보고된 자리를 무시하고 예전처럼 앞 0.3 m를 짐작해 막는다.
표시는 막힌 로봇의 몸 칸과 다른 로봇의 몸 칸을 가져가지 않는다(스위치와 관계없음).
시험 지도는 arena_v6(통로 폭 23-42 cm)이다.
"""
import hashlib
import math
import sys
from pathlib import Path

import pytest
from fleet_core.common.commands import DockCmd
from fleet_core.traffic.floor.cells import disk_cells
from fleet_core.traffic.floor.controller import BLOCK_OWNER, _Leg
from fleet_core.traffic.floor.planner import Route

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))
import compare_floor  # noqa: E402

LOCKS = {"floor_lock_ahead_m": 0.15, "floor_extend_step_m": 0.05, "release_distance_m": 0.17}
ON = {"floor_obstacle_report": True}
THREE = (("pinky1", "P-3"), ("pinky2", "P-2"), ("pinky3", "P-1"))
KINDS = [{"A": 1, "C": 1}, {"B": 1}, {"D": 1, "A": 1}, {"C": 1}, {"B": 1, "D": 1}, {"A": 1}]
RADIUS = 0.085
EAST = (1.56, 0.405)  # 긴 진열대 사이 통로의 동쪽 끝: 출고 도크를 나온 로봇이 본다
SOUTH_WEST = (0.79, 0.25)  # 넓은 남서쪽: 옆으로 지나갈 수 있다
AT_S, FOR_S = 30.0, 100.0  # 한 로봇 실행: 로봇이 출고 도크에 있을 때 나타나 100 s 머문다
# 물체 셋 일정(나타나는 시각, 머무는 시간, x, y): 같은 세 자리에 두 번씩 나타난다
SCHEDULE = [(30, 55, 0.79, 0.25), (60, 55, 1.24, 0.405), (105, 55, 1.29, 0.864),
            (150, 50, 1.24, 0.405), (185, 50, 0.79, 0.25)]


def obj(x, y, at_s=AT_S, dur=FOR_S, r=RADIUS):
    return {"type": "object", "at_s": at_s, "duration_s": dur, "x": x, "y": y, "radius_m": r}


def v6(faults, robots=1, count=1, seed=1, report=True, noise=0.03, **over):
    data = {"map": "arena_v6", "seed": 1, "speed_factor": 5.0, "timeout_s": 900,
            "stock": {"arm1": {"A": 40, "B": 40}, "arm2": {"C": 40, "D": 40}},
            "orders": [{"at_s": 20.0 * k, "id": f"R-{k + 1:03d}", "items": KINDS[k % 6]}
                       for k in range(count)],
            "robots": {n: {"start": s, "speed_mps": 0.15, "pose_noise_m": noise}
                       for n, s in THREE[:robots]},
            "arm": {"seconds_per_item": 4.0, "swap_seconds": 8.0, "fail_rate": 0.0},
            "dock": {"duration_s": 3.0, "error_cm": 1.0}, "faults": faults,
            "expect": {"orders_shipped": count}}
    compare_floor.OVERRIDES = {**LOCKS, **over}
    try:
        world = compare_floor.build(data, "arena_v6", seed, "freefloor")
    finally:
        compare_floor.OVERRIDES = {}
    world.report_obstacle = report
    return world


def run(world, limit, watch=None):
    while world.t < limit - 1e-9:
        world.step()
        if watch is not None:
            watch(world)
        if world.all_orders_final() and world.all_parked():
            break
    return world


def digest(world):
    h = hashlib.sha256()
    for t, ev in world.event_log:
        h.update(f"{t:.3f} {ev.level} {ev.code} {ev.robot} {ev.detail}\n".encode())
    for t, c in world.commands:
        h.update(f"{t:.3f} {c!r}\n".encode())
    return h.hexdigest()


def codes(world, *names):
    return [(t, e) for t, e in world.event_log if e.code in names]


def first_mark(world, limit=80.0) -> set:
    """처음 막힘 표시된 칸들(표시가 생긴 tick의 것)."""
    seen = []

    def watch(w):
        if not seen and w.core.traffic.blocked_cells():
            seen.append(set(w.core.traffic.blocked_cells()))
    run(world, limit, watch)
    assert seen, "nothing was marked"
    return seen[0]


def done_at(world, limit=300.0) -> float | None:
    """주문이 끝나고 로봇이 모두 주차한 시각. 그 전에 끝나지 않으면 None."""
    run(world, limit)
    return world.t if world.all_orders_final() and world.all_parked() else None


def track(world, limit):
    """로봇 pinky1의 실제 자리 기록 [(t, x, y)]."""
    out = []
    run(world, limit, lambda w: out.append((w.t, w.virtual["pinky1"].x, w.virtual["pinky1"].y)))
    return out


# ---- 1. 보고된 자리면 물체 둘레만 ----------------------------------------------------------


def test_a_reported_object_marks_only_the_cells_round_it():
    on = v6([obj(*EAST)], **ON)
    marked = first_mark(on)
    floor = on.floor
    reach = RADIUS + on.params.floor_obstacle_margin_m
    assert all(floor.box_distance(floor.cell_at(c), *EAST) <= reach + 1e-9 for c in marked)
    guessed = first_mark(v6([obj(*EAST)]))
    assert len(marked) < len(guessed)
    assert codes(on, "FLOOR_OBSTACLE") and not codes(on, "FLOOR_BLOCKED")


# ---- 2. 좁은 동쪽 끝: 켜면 남쪽으로 돌아간다 -------------------------------------------------


def test_east_end_with_the_switch_on_goes_round_by_the_south_without_waiting():
    world = v6([obj(*EAST)], **ON)
    path = track(world, 300.0)
    end = world.t
    assert world.all_orders_final() and world.all_parked()
    assert end < AT_S + FOR_S, "it waited for the object to go"
    # 물체 동쪽에서 남쪽 통로로 내려가 서쪽으로 간다(가운데 통로 y 0.405를 지나지 않는다)
    assert any(1.3 < x < 1.75 and y < 0.25 for _, x, y in path)
    assert not codes(world, "ROBOT_ISOLATED")


def test_east_end_with_the_switch_off_waits_for_the_object():
    world = v6([obj(*EAST)])
    run(world, AT_S + FOR_S - 5.0)
    assert not (world.all_orders_final() and world.all_parked())
    assert codes(world, "FLOOR_BLOCKED")


# ---- 3. 남서쪽: 기다리지 않고 간다 -------------------------------------------------------
# (0.79, 0.25)의 물체 옆 빈 폭은 북쪽(진열대 b까지)과 남쪽(벽까지) 모두 0.165 m다. 관제의 몸
# 모형(막은 칸을 몸 반지름 + 여유로 넓혀 피함, 칸 0.03 m)으로는 어느 쪽도 지나갈 수 없어 진열대
# b를 돌아간다. 바로 옆을 지나가게 하려면 계획의 벽 넓힘을 반폭으로 줄여야 한다(이 변경 밖).


def test_in_the_south_west_the_robot_goes_on_without_waiting_for_the_object():
    world = v6([obj(*SOUTH_WEST)], **ON)
    path = track(world, 300.0)
    assert world.all_orders_final() and world.all_parked()
    assert world.t < AT_S + FOR_S, "it waited for the object to go"
    marked_at = codes(world, "FLOOR_OBSTACLE")[0][0]
    radius = world.params.robot_radius_m
    gaps = [math.dist((x, y), SOUTH_WEST) - RADIUS - radius for t, x, y in path
            if marked_at < t < AT_S + FOR_S]
    assert min(gaps) >= 0.0
    assert not codes(world, "FLOOR_BLOCKED", "ROBOT_ISOLATED")


# ---- 4. 표시는 로봇이 선 칸을 가져가지 않는다(스위치와 관계없음) ----------------------------------


def _two_robots_noisy_reports(**over):
    """pinky1이 pinky2 쪽으로 가다 막힌 꼴. 두 로봇의 보고가 잡음만큼 평균에서 벗어난다."""
    world = v6([], robots=2, count=0, noise=0.0, **over)
    for _ in range(30):
        world.step()
    tr = world.core.traffic
    m1, m2 = tr._missions["pinky1"], tr._missions["pinky2"]
    a, b = (m1.x, m1.y), (m2.x, m2.y)
    tr._legs["pinky1"] = leg = _Leg(Route([a, b]), b)
    leg.s_lock = leg.s_sent = 0.0
    return world, tr, m1, m2, a, b


def _marked_on(tr, x, y, radius) -> set:
    cells = tr._ids(disk_cells(tr.floor, x, y, radius))
    return {c for c in cells if tr.zones.holder(c) == BLOCK_OWNER}


@pytest.mark.parametrize("over", [{}, ON])
def test_a_mark_never_takes_cells_under_a_robot(over):
    world, tr, m1, m2, a, b = _two_robots_noisy_reports(**over)
    noise = 0.03
    seen = (a[0] + 0.3, a[1], 0.05) if over else None
    # 두 로봇 모두 잡음 섞인 보고를 몇 번 보냈다. 막힌 로봇은 동북쪽, 앞 로봇은 서남쪽으로 튄
    # 보고를 마지막으로 보냈다
    for k in range(6):
        d = noise if k % 2 else -noise
        tr.on_robot_state("pinky2", b[0] + d, b[1] + d, 0.0, "IDLE")
        tr.on_robot_state("pinky1", a[0] - d, a[1] - d, 0.0, "BLOCKED", obstacle=seen)
    tr._mark_blocked(m1) if not over else tr._mark_object(m1)
    # 다음 보고는 반대쪽으로 튄다: 쥐고 있던 앞 칸이 이때 표시로 옮겨진다
    tr.on_robot_state("pinky1", a[0] - noise, a[1] - noise, 0.0, "BLOCKED", obstacle=seen)
    tr.tick_progress(world.t)
    body = tr.floor.robot_radius_m
    for x, y in (a, (a[0] + noise, a[1] + noise), (a[0] - noise, a[1] - noise)):
        assert not _marked_on(tr, x, y, body), "the mark took cells under the blocked robot"
    for x, y in (b, (b[0] - noise, b[1] - noise)):
        assert not _marked_on(tr, x, y, body), "the mark took cells under the robot ahead"


def test_progress_does_not_jump_past_a_fold_onto_the_way_back():
    """되접히는 길(출고 접근선 위 끝에서 돌아 내려옴)에서 튄 보고가 진행점을 꼭짓점 너머 돌아오는
    구간으로 넘기면 꼭짓점 둘레 칸을 놓는다(헤드리스 씨앗 3의 불변식 위반)."""
    world = v6([], noise=0.0)
    for _ in range(5):
        world.step()
    tr = world.core.traffic
    m = tr._missions["pinky1"]
    route = Route([(1.80, 0.40), (1.85, 0.67), (1.85, 0.50)])
    tr._legs["pinky1"] = leg = _Leg(route, (1.85, 0.50))
    leg.s, leg.s_lock = 0.2, route.length
    leg.s_sent = route.length
    tr.on_robot_state("pinky1", *route.at(0.2), 0.0, "MOVING")
    tr._follow(m)
    tr.on_robot_state("pinky1", 1.85, 0.60, 0.0, "MOVING")  # 돌아오는 구간 위로 튄 보고
    tr._follow(m)
    assert leg.s <= route.cum[1] + 1e-9


# ---- 5. arena_v6 물체 셋 일정, 세 로봇 ----------------------------------------------------


@pytest.mark.parametrize("seed", [1, 2, 3])
def test_three_object_schedule_runs_clean_with_the_switch_on(seed):
    world = v6([obj(x, y, a, d) for a, d, x, y in SCHEDULE], robots=3, count=8, seed=seed, **ON)
    run(world, 900.0)
    assert sum(r.status == "SHIPPED" for r in world.reports) == 8
    assert world.all_parked()
    assert sum(isinstance(c, DockCmd) for _, c in world.commands) == 16
    assert not codes(world, "ROBOT_ISOLATED")
    assert world.cell_violations == 0


# ---- 6. 옛 로봇(빈 필드)과 끈 스위치는 예전 그대로 ---------------------------------------------


def test_an_old_robot_without_the_fields_runs_as_before_with_the_switch_on():
    old_on = run(v6([obj(*EAST)], report=False, **ON), 120.0)
    old_off = run(v6([obj(*EAST)], report=False), 120.0)
    assert digest(old_on) == digest(old_off)
    assert codes(old_on, "FLOOR_BLOCKED") and not codes(old_on, "FLOOR_OBSTACLE")


def test_a_bad_obstacle_value_is_taken_as_none_and_an_iterator_is_read_once():
    world = v6([], noise=0.0, **ON)
    world.step()
    core, tr = world.core, world.core.traffic
    m = tr._missions["pinky1"]
    core.on_robot_state("pinky1", m.x, m.y, 0.0, "IDLE", obstacle=1)
    assert m.obstacle is None
    core.on_robot_state("pinky1", m.x, m.y, 0.0, "BLOCKED", obstacle=iter([1.0, 0.4, 0.05]))
    assert m.obstacle == (1.0, 0.4, 0.05)


@pytest.mark.parametrize("place", [EAST, SOUTH_WEST])
def test_with_the_switch_off_reported_places_change_nothing(place):
    told = run(v6([obj(*place)]), 120.0)
    silent = run(v6([obj(*place)], report=False), 120.0)
    assert digest(told) == digest(silent)
