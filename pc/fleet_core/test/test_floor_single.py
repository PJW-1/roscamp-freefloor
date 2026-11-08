"""자유바닥 한 대(P2): 계획 -> 굴러가는 칸 잠금 -> MoveTo 교체 -> 도착·도킹.

FloorWorld가 매 단계 칸 불변식을 본다(몸이 쥔 칸 밖으로 나가지 않음, path는 쥔 칸 안).
"""
import math
import sys
from pathlib import Path

import pytest
from fleet_core.common.commands import DockCmd, MoveCmd
from fleet_core.traffic.floor.cells import swept_cells
from fleet_core.traffic.floor.controller import FloorController
from floor_world import FloorWorld

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from baseline import Meter  # noqa: E402

ONE = {"pinky1": {"start": "P-6", "speed_mps": 0.20, "pose_noise_m": 0.03}}
ORDER = [{"at_s": 0.0, "id": "O-001", "items": {"A": 1, "B": 1}}]


def run_one(seed=1, seconds=400.0, **kw):
    world = FloorWorld(seed=seed, robots=dict(ONE), orders=list(ORDER), gaussian_noise=True, **kw)
    meter = Meter(list(world.robots), 0.1)
    finished = False
    for _ in range(round(seconds / 0.1)):
        world.step()
        meter.observe(world)
        if world.done():
            finished = True
            break
    return world, meter, finished


@pytest.fixture(scope="module")
def run():
    return run_one()


def test_the_floor_controller_runs(run):
    world, _, _ = run
    assert isinstance(world.core.traffic, FloorController)


def test_one_order_is_shipped_and_the_robot_parks(run):
    world, _, finished = run
    assert finished, f"not done at {world.t}: {world.core.robot_view('pinky1')}"
    assert world.report_statuses() == {"O-001": "SHIPPED"}
    docks = [c.dock_id for c in world.commands_of(DockCmd)]
    assert docks[:2] == ["WORK", "OUTBOUND"]


def test_the_robot_never_stops_on_the_way(run):
    _, meter, _ = run
    assert meter.stops == 0


def test_moves_are_replaced_as_the_lock_rolls_forward(run):
    world, _, _ = run
    moves = world.commands_of(MoveCmd)
    finals = [m for m in moves if "~" not in m.target_id]
    assert len(finals) >= 3  # WORK, OUTBOUND, a parking spot
    assert len(moves) > 3 * len(finals)  # many short replacing goals, not one per leg
    # a goal reaches at most the lock-ahead distance, plus a whole fixed line taken at once
    longest_line = max(line.length() for line in world.floor.fixed_lines.values())
    for move in moves:
        ahead = sum(math.dist(a[:2], b[:2]) for a, b in zip(move.path, move.path[1:],
                                                            strict=False))
        assert ahead <= world.core.params.floor_lock_ahead_m + longest_line + 0.1


def test_a_robot_holds_only_cells_near_it(run):
    world, _, _ = run
    held = world.core.zones.held_by("pinky1")
    floor = world.floor
    r = world.virtual["pinky1"]
    for cell in held:
        assert floor.box_distance(floor.cell_at(cell), r.x, r.y) <= 0.3


def test_the_dock_line_is_locked_whole_before_the_robot_enters_it():
    world = FloorWorld(seed=2, robots=dict(ONE), orders=list(ORDER), gaussian_noise=True)
    floor = world.floor
    line = floor.fixed_lines["work_in"]
    need = {floor.cell_id(c) for c in swept_cells(floor, list(line.points), floor.robot_radius_m)}
    entered = False
    for _ in range(3000):
        world.step()
        r = world.virtual["pinky1"]
        if math.dist((r.x, r.y), line.start) < 0.02:
            entered = True
            assert need <= world.core.zones.held_set("pinky1")
            break
    assert entered


def test_the_last_move_of_a_dock_leg_ends_on_the_dock(run):
    world, _, _ = run
    work = [m for m in world.commands_of(MoveCmd) if m.target_id == "W"]
    assert work and (work[-1].x, work[-1].y) == world.floor.docks["WORK"]


@pytest.mark.parametrize("seed", [3, 4, 5])
def test_more_seeds_finish_without_breaking_a_cell_invariant(seed):
    world, meter, finished = run_one(seed=seed)
    assert finished and world.cell_violations == 0
    assert meter.stops == 0
