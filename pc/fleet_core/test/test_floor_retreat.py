"""관제가 시키는 물러나기(floor_block_retreat)와 통로 비우기(floor_corridor_clear).

둘 다 기본으로 꺼져 있다. 꺼 두면 전과 똑같이 돈다(test_switches_off_*).
막힌 로봇이 먼저 스스로 벗어나 보고(RECOVERING) BLOCKED라고 알린 뒤에만 관제가 움직인다.
아레나 v4 시험은 로봇이 지름길을 스스로 고르지 않게 해 두었다(_take_shortcut 끔): 가운데 통로
(outbound_in)를 막아 두고 갈림길(지름길이 갈라지는 곳)까지 물러나는지 본다.
"""
import hashlib
import math
from dataclasses import replace

import compare_floor
import pytest
from fleet_core.common.commands import MoveCmd, RetreatCmd
from fleet_core.common.params import load_params
from fleet_core.traffic.floor.controller import _along
from fleet_core.traffic.floor.planner import Route
from floor_world import FloorWorld
from test_floor_arena_v4 import LOCKS, THREE, scenario
from test_sim_world_object import ONE, ORDER

RETREAT_CODES = ("FLOOR_RETREAT_START", "FLOOR_RETREAT_DONE", "FLOOR_RETREAT_FAILED",
                 "FLOOR_REROUTE", "FLOOR_CORRIDOR_CLEAR")
SOLO = {"floor_block_retreat": True}
CORRIDOR = {"floor_corridor_clear": True}
FORK = (0.97, 0.41)   # outbound_in에서 지름길(outbound_short)이 갈라지는 곳 근처
MIDDLE = (0.97, 0.85)   # 갈림길 위쪽 가운데 통로: 지름길로 돌아갈 수 있다
NO_FORK = (0.97, 0.70)  # 표시가 지름길 모퉁이까지 닿는다: 돌아갈 길이 없다


def obj(x, y, at_s, dur, r=0.04):
    return {"type": "object", "at_s": at_s, "duration_s": dur, "x": x, "y": y, "radius_m": r}


def run(world, limit):
    while world.t < limit:
        world.step()
        if world.all_orders_final() and world.all_parked():
            break
    return world


def lane(faults, limit=200.0, **over):
    params = replace(load_params(), **over)
    world = FloorWorld(seed=3, robots=dict(ONE), orders=list(ORDER), faults=faults,
                       params=params, report_heading=True)
    return world


def arena(faults, robots=1, count=1, **over):
    data = scenario(robots=THREE[:robots], count=count, timeout_s=600, noise_m=0.0)
    data["faults"] = faults
    compare_floor.OVERRIDES = {**LOCKS, **over}
    try:
        world = compare_floor.build(data, "arena_v4", 1, "freefloor")
    finally:
        compare_floor.OVERRIDES = {}
    world.core.traffic._take_shortcut = lambda m, leg: None
    return world


def events(world, *codes):
    return [(t, ev) for t, ev in world.event_log if not codes or ev.code in codes]


def retreats(world):
    return [(t, c) for t, c in world.commands if isinstance(c, RetreatCmd)]


def digest(world):
    h = hashlib.sha256()
    for t, ev in world.event_log:
        h.update(f"{t:.3f} {ev.level} {ev.code} {ev.robot} {ev.detail}\n".encode())
    for t, c in world.commands:
        h.update(f"{t:.3f} {c!r}\n".encode())
    return h.hexdigest()


def shipped(world):
    return sum(r.status == "SHIPPED" for r in world.reports)


# ---- 1. 금방 치워진 물건 -------------------------------------------------------------------


def test_an_object_gone_before_the_wait_changes_nothing():
    """물건이 표시 뒤 floor_retreat_after_s 안에 사라지면 켜도 끈 것과 똑같이 돈다."""
    faults = [obj(2.4, 1.15, 30.0, 25.0, 0.06)]  # 47.9 s에 표시, 55 s에 치워짐(57.9 s 전)
    off = run(lane(faults), 200.0)
    on = run(lane(faults, **SOLO, **CORRIDOR), 200.0)
    assert events(off, "FLOOR_BLOCKED"), "the object must block the robot at all"
    assert not events(on, *RETREAT_CODES)
    assert digest(on) == digest(off)
    assert on.all_orders_final() and on.all_parked()


# ---- 2. 물러나지 않고 다른 길 --------------------------------------------------------------


def test_a_blocked_robot_with_another_way_from_where_it_stands_takes_it_without_backing():
    faults = [obj(2.4, 1.15, 0.0, 200.0, 0.06)]
    off = run(lane(faults), 200.0)
    on = run(lane(faults, **SOLO), 200.0)
    assert not off.all_parked(), "switched off, the robot waits for the object"
    assert [ev.code for _, ev in events(on, *RETREAT_CODES)] == ["FLOOR_REROUTE"]
    assert not retreats(on)
    (t_mark, _), = events(on, "FLOOR_BLOCKED")[:1]
    (t_re, _), = events(on, "FLOOR_REROUTE")
    assert t_re >= t_mark + on.core.traffic.params.floor_retreat_after_s - 0.2
    assert on.all_parked() and on.min_object_gap > 0.0
    assert on.core.traffic.metrics["reroutes"] == 1


# ---- 3. 갈림길까지 물러나기 ----------------------------------------------------------------


def test_a_robot_with_no_other_way_backs_to_the_fork_and_goes_round(monkeypatch):
    world = arena([obj(*MIDDLE, 0.0, 65.0)], **SOLO)
    tr = world.core.traffic
    seen = []
    real = tr._retreat

    def spy(m, back, way, why):
        ok = real(m, back, way, why)
        if ok:
            seen.append((set(tr.zones.held_set(m.name)), set(tr._swept(way) | tr._body_now(m)),
                         m.name in tr._marks, m.phase))
        return ok

    monkeypatch.setattr(tr, "_retreat", spy)
    run(world, 400.0)
    codes = [ev.code for _, ev in events(world, *RETREAT_CODES)]
    assert codes[:3] == ["FLOOR_RETREAT_START", "FLOOR_RETREAT_DONE", "FLOOR_REROUTE"]
    (_, cmd), = retreats(world)[:1]
    assert cmd.distance_m <= tr.params.floor_retreat_max_m
    assert math.dist(cmd.path[-1][:2], FORK) < 0.1
    held, way, marked, phase = seen[0]
    assert held == way, "while backing it holds its body and the way back, nothing ahead"
    assert marked and phase == "TO_OUTBOUND", "the mark stays and the order does not change"
    assert shipped(world) == 1 and world.min_object_gap > 0.0


# ---- 4. 실패 --------------------------------------------------------------------------------


def test_with_no_fork_it_warns_at_once_and_errors_when_it_did_before():
    faults = [obj(*NO_FORK, 0.0, 300.0)]
    off = run(arena(faults), 150.0)
    on = run(arena(faults, **SOLO), 150.0)
    errors_off = [(t, ev.code) for t, ev in events(off) if ev.level == "ERROR"]
    errors_on = [(t, ev.code) for t, ev in events(on) if ev.level == "ERROR"]
    assert errors_off and errors_on == errors_off
    (t_mark, _), = events(on, "FLOOR_BLOCKED")[:1]
    fails = events(on, "FLOOR_RETREAT_FAILED")
    assert fails and "no other way" in fails[0][1].detail
    # 갈림길 찾기는 tick마다 0.05 m씩(계획 예산 한 번): 1 m를 다 보는 데 2 s쯤 걸린다
    assert fails[0][0] <= t_mark + on.core.traffic.params.floor_retreat_after_s + 3.0
    assert not retreats(on)


@pytest.mark.parametrize("reason", ["TIMEOUT", "FAILED", "REJECTED", "LOST"])
def test_a_retreat_that_ends_badly_warns_and_tries_again(monkeypatch, reason):
    world = arena([obj(*MIDDLE, 0.0, 65.0)], **SOLO)
    core = world.core
    sent = []

    def fail_first(cmd):
        if sent:
            return real(cmd)
        sent.append(cmd)
        if reason == "REJECTED":
            world._queued_results.append((cmd.robot, core.on_goal_rejected,
                                          (cmd.robot, cmd.goal_id)))
        elif reason == "LOST":
            world._queued_results.append((cmd.robot, core.on_goal_lost, (cmd.robot, cmd.goal_id)))
        else:
            world._queued_results.append((cmd.robot, core.on_retreat_result,
                                          (cmd.robot, cmd.goal_id, False, reason)))

    real = world._start_retreat
    monkeypatch.setattr(world, "_start_retreat", fail_first)
    run(world, 400.0)
    fails = events(world, "FLOOR_RETREAT_FAILED")
    assert fails and fails[0][1].detail == f"the retreat ended {reason}"
    starts = [t for t, _ in events(world, "FLOOR_RETREAT_START")]
    assert len(starts) >= 2
    assert starts[1] - starts[0] >= world.core.traffic.params.floor_retreat_after_s - 0.2
    assert shipped(world) == 1


def test_an_object_behind_ends_the_retreat_and_it_waits():
    faults = [obj(*MIDDLE, 0.0, 300.0), obj(0.97, 0.45, 40.0, 30.0)]
    world = run(arena(faults, **SOLO), 150.0)
    fails = [ev.detail for _, ev in events(world, "FLOOR_RETREAT_FAILED")]
    assert "the retreat ended OBSTACLE_BEHIND" in fails
    assert world.min_object_gap > 0.0


def test_cells_behind_held_by_another_robot_warn_and_send_nothing(monkeypatch):
    world = arena([obj(*MIDDLE, 0.0, 300.0)], **SOLO)
    tr = world.core.traffic
    real = tr._retreat

    def grab(m, back, way, why):
        free = sorted(c for c in tr._swept(way) - tr._body_now(m) if tr.zones.holder(c) is None)
        assert free and tr.zones.acquire_all("ghost", {free[-1]})
        return real(m, back, way, why)

    monkeypatch.setattr(tr, "_retreat", grab)
    run(world, 120.0)
    fails = [ev.detail for _, ev in events(world, "FLOOR_RETREAT_FAILED")]
    assert fails and fails[0].startswith("cannot back up: cells behind held by ['ghost']")
    assert not retreats(world)


# ---- 5. 한 방향 선을 거스르는 길 -------------------------------------------------------------


def test_a_way_against_a_one_way_line_needs_the_whole_line(monkeypatch):
    """한 방향 선을 거꾸로 지나는 다른 길은 그 선을 통째로 잡을 수 있을 때만 쓴다."""
    world = arena([obj(*MIDDLE, 0.0, 65.0)], **SOLO)
    tr = world.core.traffic
    taken = []
    real_against = tr._against

    def against(route):
        names, until = real_against(route)
        return names | {"work_in"}, until

    monkeypatch.setattr(tr, "_against", against)
    real = tr.zones.acquire_all

    def acquire_all(name, cells):
        ok = real(name, cells)
        if set(cells) >= tr._line_ids["work_in"]:
            taken.append(ok)
        return ok

    monkeypatch.setattr(tr.zones, "acquire_all", acquire_all)
    run(world, 400.0)
    assert taken, "the reroute asked for the whole one-way line"
    reroutes = events(world, "FLOOR_REROUTE")
    assert reroutes and all(taken)
    assert shipped(world) == 1


def test_a_way_against_a_held_one_way_line_is_not_taken(monkeypatch):
    world = arena([obj(*MIDDLE, 0.0, 300.0)], **SOLO)
    tr = world.core.traffic
    real = tr._route_from

    def route_from(m, leg, start):
        # 지나온 work_in의 칸 하나를 남이 쥔다. 그 선을 거스르는 길은 쓸 수 없다
        free = sorted(c for c in tr._line_ids["work_in"] if tr.zones.holder(c) is None)
        if not tr.zones.held_by("ghost"):
            assert tr.zones.acquire_all("ghost", {free[0]})
        return real(m, leg, start)

    monkeypatch.setattr(tr, "_route_from", route_from)
    monkeypatch.setattr(tr, "_against", lambda route: ({"work_in"}, 0.0))
    run(world, 120.0)
    assert tr.zones.held_by("ghost")
    assert not events(world, "FLOOR_REROUTE")


# ---- 6. 통로 비우기 --------------------------------------------------------------------------


def test_the_front_robot_alone_backs_out_of_the_corridor_at_once():
    world = run(arena([obj(*MIDDLE, 20.0, 60.0)], robots=2, count=2, **CORRIDOR), 400.0)
    (t_mark, _), = events(world, "FLOOR_BLOCKED")[:1]
    (t_cc, ev), = events(world, "FLOOR_CORRIDOR_CLEAR")
    assert t_cc - t_mark < 0.5, "a corridor does not wait floor_retreat_after_s"
    assert ev.level == "INFO" and "['pinky1']" in ev.detail
    (t_r, cmd), = retreats(world)[:1]
    assert cmd.robot == "pinky1" and cmd.distance_m <= world.core.traffic.params.floor_retreat_max_m
    cleared = [t for t, ev in events(world, "FLOOR_CLEARED") if "planning again" in ev.detail]
    assert cleared
    assert shipped(world) == 2 and world.min_gap > 0.2 and world.min_object_gap > 0.0


def test_two_robots_in_the_corridor_back_out_rear_first():
    world = run(arena([obj(*MIDDLE, 40.0, 60.0)], robots=2, count=2, **CORRIDOR), 400.0)
    (_, ev), = events(world, "FLOOR_CORRIDOR_CLEAR")
    assert "['pinky1', 'pinky2']" in ev.detail  # 뒤에서부터
    order = [c.robot for _, c in retreats(world)]
    assert order[:2] == ["pinky1", "pinky2"]
    starts = [t for t, ev in events(world, "FLOOR_RETREAT_START")]
    dones = [t for t, ev in events(world, "FLOOR_RETREAT_DONE")]
    assert dones[0] <= starts[1], "the front robot waits until the one behind has moved"
    assert shipped(world) == 2 and world.min_gap > 0.2 and world.min_object_gap > 0.0


@pytest.mark.parametrize("at_s", [20.0, 40.0])
def test_corridor_members_back_one_at_a_time_from_the_rear_and_end_together(monkeypatch, at_s):
    """물러나기를 낼 때: 취소한 이동 goal이 끝났고, 뒤(선 위 덜 깊이)의 멤버는 다 나왔다.
    표시를 지울 때: 멤버가 모두 나왔다."""
    world = arena([obj(*MIDDLE, at_s, 60.0)], robots=2, count=2, **CORRIDOR)
    tr = world.core.traffic
    starts, ends = [], []
    real_retreat, real_clear = tr._retreat, tr._clear_mark

    def depth(name, line):
        return _along(tr.floor.fixed_lines[line].points, tr._here(tr._missions[name]))[0]

    def retreat(m, back, way, why):
        group = {n: b for n, b in tr._back.items() if b.owner == back.owner}
        behind = [n for n, b in group.items() if n != m.name and b.state != "OUT"
                  and depth(n, back.line) < depth(m.name, back.line)]
        ok = real_retreat(m, back, way, why)
        if ok:
            starts.append((m.name, bool(tr._active[m.name]), behind))
        return ok

    def clear_mark(owner, why):
        ends.append({n: b.state for n, b in tr._back.items() if b.owner == owner})
        return real_clear(owner, why)

    monkeypatch.setattr(tr, "_retreat", retreat)
    monkeypatch.setattr(tr, "_clear_mark", clear_mark)
    run(world, 400.0)
    assert starts and ends
    assert all(not active and not behind for _, active, behind in starts), starts
    assert all(set(states.values()) == {"OUT"} for states in ends), ends


def test_a_dropped_retreat_goal_keeps_its_cells(monkeypatch):
    """Retreat goal을 결과 없이 놓쳐도(취소 등) 로봇이 아직 움직일 수 있다: 물러날 칸을 쥔다."""
    world = arena([obj(*MIDDLE, 0.0, 300.0)], **SOLO)
    tr = world.core.traffic
    while not retreats(world) and world.t < 120.0:
        world.step()
    m = tr._missions["pinky1"]
    way = set(tr._back["pinky1"].cells)
    assert way and m.goal is not None
    monkeypatch.setattr(world, "_cancel", lambda cmd: None)  # 취소 결과가 오지 않는다
    tr._cancel_goal(m)
    for _ in range(5):
        world.step()
    assert events(world, "FLOOR_RETREAT_FAILED")
    assert way <= tr.zones.held_set("pinky1")
    (_, cmd), = retreats(world)
    world.core.on_retreat_result("pinky1", cmd.goal_id, False, "CANCELED")  # 늦게 온 결과: 섰다
    assert not tr._back["pinky1"].cells


def test_with_both_switches_on_the_corridor_rule_decides_on_a_line():
    both = run(arena([obj(*MIDDLE, 20.0, 60.0)], robots=2, count=2, **CORRIDOR, **SOLO), 400.0)
    alone = run(arena([obj(*MIDDLE, 20.0, 60.0)], robots=2, count=2, **CORRIDOR), 400.0)
    assert events(both, "FLOOR_CORRIDOR_CLEAR")
    assert digest(both) == digest(alone)


# ---- 7. 꺼 두면 그대로 ----------------------------------------------------------------------


@pytest.mark.parametrize("faults", [[], [obj(*MIDDLE, 0.0, 65.0)], [obj(*NO_FORK, 0.0, 300.0)]])
def test_switches_off_run_exactly_as_before(faults):
    """끈 스위치는 아무 것도 바꾸지 않는다: 매개변수를 적지 않은 실행과 사건·명령이 같다."""
    plain = run(arena(faults, robots=2, count=2), 200.0)
    off = run(arena(faults, robots=2, count=2, floor_block_retreat=False,
                    floor_corridor_clear=False, floor_retreat_after_s=3.0), 200.0)
    assert digest(off) == digest(plain)
    assert not events(off, *RETREAT_CODES)
    assert "retreats" not in off.core.traffic.metrics


BASE_DIGESTS = {  # 4a46878(이 기능 전)에서 같은 도우미로 잰 값
    "none": "0ddd4c4cfbc66ee5", "middle": "989bb439a631ebc3", "no_fork": "39b8fe2815672dc7",
}


@pytest.mark.parametrize("name,faults", [("none", []), ("middle", [obj(*MIDDLE, 0.0, 65.0)]),
                                         ("no_fork", [obj(*NO_FORK, 0.0, 300.0)])])
def test_switches_off_match_the_commit_before_the_feature(name, faults):
    assert digest(run(arena(faults, robots=2, count=2), 200.0))[:16] == BASE_DIGESTS[name]


def test_against_finds_a_free_way_backwards_along_a_one_way_line():
    tr = arena([]).core.traffic
    pts = tr.floor.fixed_lines["outbound_out"].points
    a, b = max(zip(pts, pts[1:], strict=False), key=lambda ab: math.dist(*ab))  # 가장 긴 구간
    assert "outbound_out" in tr._against(Route([b, a]))[0]
    assert "outbound_out" not in tr._against(Route([a, b]))[0]


def test_switches_off_the_lane_map_runs_as_before():
    faults = [obj(2.4, 1.15, 0.0, 200.0, 0.06)]
    plain = run(lane(faults), 200.0)
    off = run(lane(faults, floor_retreat_after_s=1.0), 200.0)
    assert digest(off) == digest(plain)
    assert not any(isinstance(c, RetreatCmd) for _, c in off.commands)
    assert any(isinstance(c, MoveCmd) for _, c in off.commands)
