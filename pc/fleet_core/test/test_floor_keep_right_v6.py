"""우측통행(floor_keep_right, 제안)을 2 cm 칸 지도 arena_v6에서.

통로 판정은 길이(m)로 한다(PASS_MAX_M 0.30 m: 3 cm 칸 10칸, 2 cm 칸 15칸). 칸 수로 보던 때(10칸)는
긴 진열대 북쪽 통로(폭 0.42 m, 빈 칸 11~13줄)가 통로로 잡히지 않아 마주 오는 두 로봇이 그 안에서 마주
섰다(교착 감지, 비켜 서기).
이제 통로로 잡히지만 두 줄은 서지 않는다: 두 줄 끝(y 0.68, 0.88, 0.20 m 떨어짐)에 선 두 로봇의 몸 잠금
(반폭 + cell_margin_m = 0.09 m)이 가운데 칸 한 줄(y 0.78)을 같이 쥐어야 한다(0.20 = 2 x 0.09 + 칸 0.02).
그래서 한 대가 통로 밖에서 기다린다. 잠금 폭과 지도 여유를 바꾸지 않는 한 나란히 지나가기는 안 된다."""
import collections
import math
import sys
from dataclasses import replace
from pathlib import Path

import pytest
from fleet_core.common.params import load_params
from fleet_core.traffic.floor.cells import disk_cells
from fleet_core.traffic.floor.floor_map import floor_map_for
from fleet_core.traffic.lane_map import load_lane_map

REPO = Path(__file__).resolve().parents[3]
LANES = load_lane_map(REPO / "sim" / "fake_robots" / "maps" / "arena_v6.yaml")
FLOOR = floor_map_for(LANES)
LOCKS = {"floor_lock_ahead_m": 0.15, "floor_extend_step_m": 0.05, "release_distance_m": 0.17}
THREE = (("pinky1", "P-3"), ("pinky2", "P-2"), ("pinky3", "P-1"))
NORTH = (1.12, 1.72, 0.57)  # 긴 진열대 북쪽 통로: x0, x1, 이 y 위

sys.path.insert(0, str(REPO / "tools"))
from floor_world import FloorWorld  # noqa: E402

# pinky1은 가운데 통로 북쪽에서 동쪽(QO)으로, pinky2는 동쪽 띠 북쪽에서 서쪽(NC)으로 북쪽 통로를 마주 지난다
POSES = {"pinky1": (1.00, 0.80), "pinky2": (1.86, 0.88)}
GOALS = {"pinky1": "QO", "pinky2": "NC"}


def world(poses, keep_right=True):
    robots = {n: {"start": s, "speed_mps": 0.15, "pose_noise_m": 0.0} for n, s in THREE}
    w = FloorWorld(seed=1, orders=[], robots=robots, site=LANES, floor=FLOOR, faults=[],
                   params=replace(load_params(), floor_keep_right=keep_right, **LOCKS))
    for name, (x, y) in poses.items():
        w.virtual[name].x, w.virtual[name].y = x, y
    traffic = w.core.traffic
    traffic._spot_taken = lambda m: False  # 아무 지점으로나 보낸다
    w._report_states()
    return w, traffic


def send(traffic, goals):
    traffic._plan_budget = 99
    for name, waypoint in goals.items():
        traffic._start_leg(traffic._missions[name], "TO_PARK", None, waypoint)


def in_north(r) -> bool:
    return NORTH[0] <= r.x <= NORTH[1] and r.y > NORTH[2]


def abreast(a, b) -> bool:
    """둘 다 북쪽 통로 안에서 서로 반대로 가며 x가 0.25 m 안이다."""
    return (in_north(a) and in_north(b) and abs(a.x - b.x) < 0.25
            and math.cos(a.heading) * math.cos(b.heading) < -0.25)


def drive(w, goals, seconds=60.0):
    """둘 다 goals에 닿을 때까지(한 번이라도 0.05 m 안). (나란히 지난 단계 수, 다 닿았나, 줄 자리)."""
    names = sorted(goals)
    target = {n: LANES.waypoint_pose(goals[n]) for n in names}
    reached = {n: False for n in names}
    side, seen = 0, collections.defaultdict(list)
    end = w.t + seconds
    while w.t < end and not all(reached.values()):
        w.step()
        a, b = (w.virtual[n] for n in names)
        if abreast(a, b):
            side += 1
            for r in (a, b):
                seen[r.name].append(r.y)
        for n in names:
            r = w.virtual[n]
            if math.dist((r.x, r.y), target[n]) <= 0.05:
                reached[n] = True
    return side, all(reached.values()), seen


def north_passage(traffic):
    cell = FLOOR.cell_of(1.40, 0.78)
    return next((p for p in traffic._passages if cell in p.cells), None)


def test_the_north_passage_is_a_passage_on_the_2cm_map():
    """칸 수(10칸)로 보던 때는 통로가 아니었다(빈 칸 11~13줄 = 0.22~0.26 m)."""
    _, traffic = world(POSES)
    passage = north_passage(traffic)
    assert passage is not None
    assert passage.axis == 0


def test_the_middle_of_the_north_passage_has_no_two_lanes_because_the_body_locks_meet():
    _, traffic = world(POSES)
    passage = north_passage(traffic)
    lo, hi = FLOOR.centre(FLOOR.cell_of(1.40, 0.68)), FLOOR.centre(FLOOR.cell_of(1.40, 0.88))
    assert FLOOR.cell_of(*lo) in passage.cells and FLOOR.cell_of(*hi) in passage.cells
    assert disk_cells(FLOOR, *lo, traffic._near) & disk_cells(FLOOR, *hi, traffic._near)
    assert not any(NORTH[0] <= FLOOR.centre(c)[0] <= NORTH[1] for c in passage.pair)


def test_two_robots_meeting_in_the_north_passage_one_waits_and_both_get_through():
    w, traffic = world(POSES)
    send(traffic, GOALS)
    side, reached, _ = drive(w, GOALS, seconds=90.0)
    codes = [e.code for e in w.events]
    assert reached, codes
    assert "FLOOR_NARROW_WAIT" in codes, codes
    assert "FLOOR_KEEP_RIGHT" not in codes, codes
    assert "DEADLOCK_DETECTED" not in codes and "DEADLOCK_UNRESOLVED" not in codes, codes
    assert side == 0
    assert w.cell_violations == 0
    assert w.min_gap >= 2 * FLOOR.robot_radius_m


@pytest.mark.xfail(strict=True, reason="0.42 m 통로에서는 두 몸 잠금이 가운데 칸 한 줄을 같이 쥔다. "
                   "잠금 폭이나 지도 여유를 바꿔야 한다(정책 결정)")
def test_two_robots_meeting_in_the_north_passage_pass_side_by_side():
    w, traffic = world(POSES)
    send(traffic, GOALS)
    side, reached, seen = drive(w, GOALS)
    codes = [e.code for e in w.events]
    assert "FLOOR_KEEP_RIGHT" in codes, codes
    assert reached
    assert side >= 5, side
    # 동쪽으로 가는 pinky1은 진열대 쪽(남), 서쪽으로 가는 pinky2는 벽 쪽(북)
    assert max(seen["pinky1"]) < min(seen["pinky2"]), seen
    assert w.cell_violations == 0
    assert w.min_gap >= 2 * FLOOR.robot_radius_m


def test_with_the_switch_off_there_are_no_passages_and_no_keep_right_events():
    w, traffic = world(POSES, keep_right=False)
    assert traffic._passages == []
    send(traffic, GOALS)
    side, _, _ = drive(w, GOALS, seconds=90.0)
    codes = [e.code for e in w.events]
    assert side == 0
    assert "FLOOR_KEEP_RIGHT" not in codes and "FLOOR_NARROW_WAIT" not in codes
