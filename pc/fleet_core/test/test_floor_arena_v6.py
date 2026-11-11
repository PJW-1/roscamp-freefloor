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


def test_the_dock_lines_are_shorter_than_on_v5_and_longer_than_the_release_distance():
    for name, line in FLOOR.fixed_lines.items():
        (ax, ay), (bx, by) = line.points[0], line.points[-1]
        length = ((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5
        assert LOCKS["release_distance_m"] < length < 0.23, (name, length)
        assert length > load_params().arrival_tolerance_m


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
