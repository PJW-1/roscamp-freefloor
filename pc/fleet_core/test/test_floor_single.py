"""자유바닥 한 대(P2): 계획 -> 굴러가는 칸 잠금 -> MoveTo 교체 -> 도착·도킹.

FloorWorld가 매 단계 칸 불변식을 본다(몸이 쥔 칸 밖으로 나가지 않음, path는 쥔 칸 안).
"""
import math
import sys
from pathlib import Path

import pytest
from fleet_core.common.commands import DockCmd, MoveCmd
from fleet_core.traffic.floor.cells import disk_cells, swept_cells
from fleet_core.traffic.floor.controller import REST_RELEASE_SAMPLES, FloorController
from fleet_core.traffic.floor.floor_map import load_floor_map
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


# ---- Codex P2 review --------------------------------------------------------------------


def bare_floor(robots=("a",)):
    from dataclasses import replace

    from fleet_core.common.params import load_params
    from fleet_core.manager import FleetManager
    from fleet_core.traffic.lane_map import load_lane_map

    now = [0.0]
    core = FleetManager(load_lane_map(), replace(load_params(), traffic_mode="freefloor"),
                        list(robots), lambda: now[0])
    core.on_order_board("RUN", [])
    core.traffic._plan_budget = 99  # these tests plan several legs outside a tick
    return core, now


def report(core, robot, x, y, state="IDLE", times=3):
    for _ in range(times):
        core.on_robot_state(robot, x, y, 0.0, state)


def moves_of(out):
    return [c for c in out if isinstance(c, MoveCmd)]


def drive_cells(core, path):
    floor = core.traffic.floor
    body = floor.robot_radius_m + floor.cell_margin_m
    return {floor.cell_id(c) for c in swept_cells(floor, [tuple(p[:2]) for p in path], body)}


def test_the_whole_sent_path_is_locked_even_from_a_pose_off_the_route():
    core, now = bare_floor()
    report(core, "a", 1.0, 1.15)
    traffic = core.traffic
    m = traffic._missions["a"]
    traffic._start_leg(m, "TO_PARK", None, "B")
    traffic.step("a")
    first = moves_of(traffic.take_output())
    assert first
    report(core, "a", 1.10, 1.24, state="MOVING")  # noisy report, 9 cm off the line
    traffic.on_move_result("a", first[-1].goal_id, True, "")
    traffic.step("a")
    second = moves_of(traffic.take_output())
    assert second
    assert drive_cells(core, second[-1].path) <= core.zones.held_set("a")


def test_an_old_goal_keeps_its_cells_until_its_result_comes():
    core, now = bare_floor()
    report(core, "a", 1.0, 1.15)
    traffic = core.traffic
    m = traffic._missions["a"]
    traffic._start_leg(m, "TO_PARK", None, "B")  # east
    traffic.step("a")
    east = moves_of(traffic.take_output())[-1]
    ahead = drive_cells(core, east.path)
    traffic._start_leg(m, "TO_PARK", None, "P-3")  # west, while the east goal still runs
    traffic.step("a")
    west = moves_of(traffic.take_output())[-1]
    traffic.on_goal_rejected("a", west.goal_id)  # the new goal never starts
    for _ in range(5):
        now[0] += 0.1
        report(core, "a", 1.0, 1.15, state="MOVING", times=1)
        traffic.tick_progress(now[0])
    assert ahead <= core.zones.held_set("a")


def test_a_report_on_cells_it_does_not_hold_takes_them_or_stops_the_robot():
    core, now = bare_floor()
    report(core, "a", 1.0, 1.15)
    traffic = core.traffic
    traffic._start_leg(traffic._missions["a"], "TO_PARK", None, "B")
    traffic.step("a")
    traffic.take_output()
    report(core, "a", 1.0, 1.45, state="MOVING")  # 30 cm off: none of these cells were locked
    out = traffic.take_output()
    floor = traffic.floor
    body = {floor.cell_id(c) for c in
            disk_cells(floor, 1.0, 1.45, floor.robot_radius_m + floor.cell_margin_m)}
    stopped = any(type(c).__name__ == "CancelCmd" for c in out)
    assert body <= core.zones.held_set("a") or stopped


def test_a_robot_that_loses_its_link_resumes_on_its_cells():
    faults = [{"at_s": 12.0, "type": "comm_loss", "robot": "pinky1", "duration_s": 5.0}]
    world, meter, finished = run_one(seed=1, faults=faults)
    assert finished and world.cell_violations == 0
    assert "COMM_LOST" in world.event_codes()


def test_rounded_corners_do_not_cut_into_fixed_lines():
    from fleet_core.traffic.floor.cells import line_cells
    from fleet_core.traffic.floor.planner import FloorPlanner

    floor = load_floor_map()
    planner = FloorPlanner(floor)
    starts = [(0.4 + 0.3 * i, 0.5 + 0.3 * j) for i in range(10) for j in range(6)]
    starts = [p for p in starts if floor.cell_of(*p) in floor.clear]
    for a in starts[::3]:
        for b in starts[1::4]:
            raw_cells = planner.astar(floor.cell_of(*a), floor.cell_of(*b), floor.clear)
            if raw_cells is None:
                continue
            raw = [a] + [floor.centre(c) for c in raw_cells[1:-1]] + [b]
            straight = planner.shortcut(raw, floor.clear | {floor.cell_of(*a), floor.cell_of(*b)})
            rounded = planner.plan(a, b)
            new = (line_cells(floor, rounded) - line_cells(floor, straight)) & planner.line_cells
            assert not new, (a, b, sorted(new))


# ---- Codex review of the P2 fixes -------------------------------------------------------


def test_a_recovered_robot_that_was_moved_replans_instead_of_arriving():
    core, now = bare_floor()
    report(core, "a", 1.05, 1.55)  # on the WORK dock
    traffic = core.traffic
    m = traffic._missions["a"]
    traffic._start_leg(m, "TO_PARK", None, "B")
    leg = traffic._legs["a"]
    assert leg.out_line == "work_out"
    leg.done = True  # its last goal succeeded while the link was down
    report(core, "a", 2.0, 0.9)  # carried away meanwhile
    traffic._finish_recovery(m)
    leg = traffic._legs["a"]
    assert not leg.done
    assert leg.route.points[0] == (2.0, 0.9)
    assert not any(name == "work_out" for _, _, name in leg.route.fixed)


def test_a_robot_docking_when_the_link_went_is_not_docked_away_from_the_dock():
    core, now = bare_floor()
    report(core, "a", 2.0, 0.9)
    traffic = core.traffic
    m = traffic._missions["a"]
    m.phase, m.dest = "DOCKING", "WORK"
    traffic._finish_recovery(m)
    assert m.phase != "DOCKING"


def test_a_robot_on_cells_of_another_is_stopped_after_the_confirm_count():
    core, now = bare_floor(("a", "b"))
    report(core, "a", 1.0, 1.15)
    report(core, "b", 1.0, 1.60)
    traffic = core.traffic
    traffic._start_leg(traffic._missions["a"], "TO_PARK", None, "B")
    traffic.step("a")
    traffic.take_output()
    core._out.clear()
    report(core, "a", 1.0, 1.42, state="MOVING", times=4)  # into b's body cells
    out = list(core._out)  # the manager passes the core output on at once
    assert any(type(c).__name__ == "CancelCmd" for c in out)
    assert "FLOOR_OFF_CELLS" in [c.code for c in out if type(c).__name__ == "EventOut"]


# ---- a robot at the end of its trip ----------------------------------------------------------
# Gazebo warehouse, 2026-11-09: a pinky stopped 4 cm short of P-3 kept the cells it came over
# (within release_distance_m) and they shut P-2, 0.35 m away, for the next robot.

def test_a_robot_standing_at_the_end_of_its_trip_keeps_only_its_body():
    core, now = bare_floor()
    traffic = core.traffic
    floor = traffic.floor
    report(core, "a", 1.0, 1.15)
    came_over = {floor.cell_id(c) for c in disk_cells(floor, 1.0, 1.15,
                                                      core.params.release_distance_m)}
    body = {floor.cell_id(c) for c in disk_cells(floor, 1.0, 1.15,
                                                 floor.robot_radius_m + floor.cell_margin_m)}
    assert came_over > body
    assert traffic.zones.acquire_all("a", came_over)
    for n in range(1, REST_RELEASE_SAMPLES + 2):
        now[0] += 0.1
        report(core, "a", 1.0, 1.15, times=1)
        traffic.tick_progress(now[0])
        if n == core.params.release_confirm_samples + 1:
            # noisy reports of a robot at rest need a longer run before it lets go
            assert traffic.zones.held_set("a") == came_over
    assert traffic.zones.held_set("a") == body


def test_a_robot_on_its_way_still_keeps_the_cells_within_the_release_distance():
    core, now = bare_floor()
    traffic = core.traffic
    floor = traffic.floor
    report(core, "a", 1.0, 1.15)
    traffic._start_leg(traffic._missions["a"], "TO_PARK", None, "B")
    traffic.step("a")
    assert moves_of(traffic.take_output())
    behind = {floor.cell_id(c) for c in disk_cells(floor, 1.0, 1.15,
                                                   core.params.release_distance_m)}
    traffic.zones.acquire_all("a", behind)
    for _ in range(core.params.release_confirm_samples + 1):
        now[0] += 0.1
        report(core, "a", 1.0, 1.15, state="MOVING", times=1)
        traffic.tick_progress(now[0])
    assert behind <= traffic.zones.held_set("a")
