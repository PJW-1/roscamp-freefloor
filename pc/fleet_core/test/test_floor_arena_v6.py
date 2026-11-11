"""얇은 진열대 시험 배치 arena_v6: 칸 0.02 m.

사용자가 로봇 둘레 잠금을 줄여 달라고 했다(2026-11-11). 위치 오차 여유(cell_margin_m 0.03)는 줄이지
못한다(0.02로 줄인 Gazebo 실행이 일찍 놓기 2번으로 불합격). 칸을 0.02 m로 줄여 칸 반올림만 줄였다."""
import sys
from pathlib import Path

import pytest
import yaml
from fleet_core.common.params import load_params
from fleet_core.traffic.floor.floor_map import floor_map_for
from fleet_core.traffic.lane_map import load_lane_map

REPO = Path(__file__).resolve().parents[3]
LANES = load_lane_map(REPO / "sim" / "fake_robots" / "maps" / "arena_v6.yaml")
FLOOR = floor_map_for(LANES)
V5 = floor_map_for(load_lane_map(REPO / "sim" / "fake_robots" / "maps" / "arena_v5.yaml"))
LOCKS = {"floor_lock_ahead_m": 0.15, "floor_extend_step_m": 0.05, "release_distance_m": 0.17}
THREE = (("pinky1", "P-3"), ("pinky2", "P-2"), ("pinky3", "P-1"))
KINDS = [{"A": 1, "C": 1}, {"B": 1}, {"D": 1, "A": 1}, {"C": 1}, {"B": 1, "D": 1}, {"A": 1}]
sys.path.insert(0, str(REPO / "tools"))
import compare_floor  # noqa: E402


def test_the_cells_and_margins_of_the_map():
    assert (FLOOR.cell_m, FLOOR.cell_margin_m, FLOOR.side_margin_m) == (0.02, 0.03, 0.02)
    assert (V5.cell_m, V5.cell_margin_m, V5.side_margin_m) == (0.03, 0.03, 0.02)


def test_a_dock_line_is_longer_than_the_arrival_tolerance_by_more_than_the_position_error():
    """핑키는 목표에서 arrival_tolerance_m 안이면 도착으로 본다. 선이 0.17~0.18 m면 위치가 3 cm 틀린
    핑키가 선 시작점에서 도착해 도킹이 실패했다(Gazebo, 2026-11-11)."""
    for name, line in FLOOR.fixed_lines.items():
        (ax, ay), (bx, by) = line.points[0], line.points[-1]
        length = ((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5
        assert length >= load_params().arrival_tolerance_m + 0.07, (name, length)


def scenario(count=8, period_s=20.0, noise_m=0.03, timeout_s=600):
    return {
        "map": "arena_v6", "seed": 1, "speed_factor": 5.0, "timeout_s": timeout_s,
        "stock": {"arm1": {"A": 40, "B": 40}, "arm2": {"C": 40, "D": 40}},
        "orders": [{"at_s": period_s * k, "id": f"R-{k + 1:03d}", "items": KINDS[k % 6]}
                   for k in range(count)],
        "robots": {name: {"start": spot, "speed_mps": 0.15, "pose_noise_m": noise_m}
                   for name, spot in THREE},
        "arm": {"seconds_per_item": 4.0, "swap_seconds": 8.0, "fail_rate": 0.0},
        "dock": {"duration_s": 3.0, "error_cm": 1.0}, "faults": [],
        "expect": {"orders_shipped": count},
    }


@pytest.mark.parametrize("seed", [1, 2])
def test_three_pinkys_ship_eight_orders_on_the_smaller_cells(tmp_path, monkeypatch, seed):
    path = tmp_path / "run.yaml"
    path.write_text(yaml.safe_dump(scenario()))
    monkeypatch.setattr(compare_floor, "OVERRIDES", dict(LOCKS))
    row, world = compare_floor.run("arena_v6", str(path), seed, "freefloor")
    assert world is not None, row["error"]
    assert row["finished"], {k: row[k] for k in ("error", "end_s", "orders_shipped")}
    assert row["orders_shipped"] == 8
    assert row["unresolved"] == 0 and row["isolations"] == 0
    assert world.cell_violations == 0  # 몸이 잠근 칸 밖으로 나가지 않는다(위치 잡음 0.03 m)
    assert row["min_gap_m"] >= 0.22
    assert max(world.tick_s) < 1.0  # 핑키는 관제 신호가 1.5 s 끊기면 선다


def test_wait_spot_search_checks_only_the_cells_it_needs(monkeypatch):
    """대기 자리를 찾을 때 후보 칸을 모두 미리 걸러 보지 않는다: 칸 0.02 m 지도에서 한 주기가 길어졌다
    (핑키는 관제 신호가 1.5 s 끊기면 선다). 쓰는 자리까지만 _standing_cells를 부른다."""
    from dataclasses import replace

    from fleet_core.manager import FleetManager
    from fleet_core.traffic.floor.controller import _Leg
    from fleet_core.traffic.floor.planner import Route

    params = replace(load_params(), traffic_mode="freefloor", **LOCKS)
    line = next(n for n in FLOOR.fixed_lines if n.endswith("_in"))
    x0, y0 = FLOOR.fixed_lines[line].points[-1]  # 선 끝(도크): 대기 자리 범위 밖이라 자리를 찾아야 한다
    core = FleetManager(LANES, params, ["a"], lambda: 0.0)
    for _ in range(3):
        core.on_robot_state("a", x0, y0, 0.0, "IDLE")
    traffic = core.traffic
    traffic._plan_budget = 99
    leg = _Leg(None, FLOOR.fixed_lines[line].points[-1], in_line=line)
    leg.waiting = True
    traffic._legs["a"] = leg
    calls = []
    real = traffic._standing_cells
    monkeypatch.setattr(traffic, "_standing_cells", lambda x, y: (calls.append(1), real(x, y))[1])
    monkeypatch.setattr(traffic, "_route", lambda m, start, goal, *a, **k: Route([start, goal]))
    traffic._plan_wait(traffic._missions["a"], leg, None)
    assert leg.route is not None
    assert len(calls) <= 300, len(calls)
