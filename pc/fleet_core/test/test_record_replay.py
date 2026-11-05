"""FmsCore keeps a record of what came in and what came of it. tools/replay.py plays it back."""
import inspect
import itertools
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest
from fleet_core.commands import OrderRecord
from fleet_core.fms_core import FmsCore
from fleet_core.params import load_params
from fleet_core.site_map import load_site_map
from sim_world import SimWorld, six_orders

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))

import replay  # noqa: E402

FAULTS = [
    {"at_s": 30.0, "robot": "pinky2", "type": "comm_loss", "duration_s": 12.0},
    {"at_s": 50.0, "robot": "pinky1", "type": "nav_abort", "count": 1},
]


@pytest.fixture(scope="module")
def run(tmp_path_factory):
    """A whole run of the real map on the virtual warehouse, with faults in it."""
    world = SimWorld(seed=3, site=load_site_map(), orders=six_orders(), faults=FAULTS)
    lines = []
    world.core.record_to(lines.append)
    world.run_until(world.done)
    path = tmp_path_factory.mktemp("records") / "run.jsonl"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path, lines


def changed(path, tmp_path, change):
    """Write a copy of the record with one entry changed."""
    lines = path.read_text(encoding="utf-8").splitlines()
    for number, line in enumerate(lines[1:], start=1):
        entry = json.loads(line)
        if change(entry):
            lines[number] = json.dumps(entry)
            break
    else:
        raise AssertionError("nothing to change")
    copy = tmp_path / "changed.jsonl"
    copy.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return copy, number + 1


def test_a_run_plays_back_to_the_same_commands(run):
    path, lines = run
    result = replay.replay(path)
    assert result.same and result.compared, result
    assert result.inputs > 1000
    kinds = {json.loads(line).get("out") for line in lines}
    assert {"MoveCmd", "DockCmd", "FillCmd", "SwapCmd", "ReportOrderCmd", "EventOut"} <= kinds
    inside = {json.loads(line).get("inside") for line in lines}
    assert {"DispatchCmd", "DispatchResult", "ReleaseCmd"} <= inside
    assert result.commands == sum("out" in json.loads(line) for line in lines)


def test_every_way_into_the_core_is_recorded():
    """A new on_ method with no record would make playback quietly wrong."""
    calls = {
        "on_robot_state": ("r1", 1.0, 2.0, 0.5, "IDLE", 90.0),
        "on_order_board": ("RUN", [OrderRecord("O-1", {"A": 1})]),
        "on_move_result": ("r1", 7, True, ""),
        "on_goal_rejected": ("r1", 7),
        "on_dock_result": ("r1", 7, True, 1.0, 1.0, 0.5),
        "on_fill_result": (7, True, {"A": 1}, {}, ""),
        "on_swap_result": (7, True, True, True, ""),
        "confirm_clear": ("r1", ["P-6"], "lifted off"),
        "tick": (),
    }
    public = {name for name, _ in inspect.getmembers(FmsCore, inspect.isfunction)
              if name.startswith("on_")}
    assert public <= set(calls), public - set(calls)
    core = FmsCore(load_site_map(), load_params(), ["r1"], lambda: 1.5)
    lines = []
    core.record_to(lines.append)
    for name, args in calls.items():
        getattr(core, name)(*args)
    seen = [json.loads(line) for line in lines[1:]]
    assert [e["in"] for e in seen if "in" in e] == list(calls)
    assert all(e["t"] == 1.5 for e in seen)
    state = next(e for e in seen if e.get("in") == "on_robot_state")
    assert state["args"] == ["r1", 1.0, 2.0, 0.5, "IDLE", 90.0]


def test_one_call_sees_one_time_live_as_well():
    """Otherwise a run could turn out differently from its record by a few microseconds."""
    ticks = itertools.count(1.0, 0.25)
    site = load_site_map()
    core = FmsCore(site, load_params(), ["r1"], lambda: next(ticks))
    core.on_robot_state("r1", *site.cell_pose("P-6"), 0.0, "IDLE")
    acquired = core.zones.snapshot()["P-6"]["acquired_at"]
    assert acquired == core.traffic._missions["r1"].last_seen


def test_a_decision_that_changed_is_found(run, tmp_path):
    def fail_a_move(entry):
        if entry.get("in") == "on_move_result" and entry["args"][2]:
            entry["args"][2] = False
            return True
        return False

    copy, number = changed(run[0], tmp_path, fail_a_move)
    result = replay.replay(copy)
    assert not result.same
    assert result.line > number
    assert result.recorded and result.produced


def test_a_command_that_changed_is_found(run, tmp_path):
    def move_it(entry):
        if entry.get("out") == "MoveCmd":
            entry["x"] += 1.0
            return True
        return False

    copy, number = changed(run[0], tmp_path, move_it)
    result = replay.replay(copy)
    assert (result.same, result.line) == (False, number)


def test_a_line_left_out_is_found(run, tmp_path):
    lines = run[0].read_text(encoding="utf-8").splitlines()
    position = next(n for n, line in enumerate(lines) if '"out"' in line)
    del lines[position]
    copy = tmp_path / "gap.jsonl"
    copy.write_text("\n".join(lines) + "\n", encoding="utf-8")
    result = replay.replay(copy)
    assert (result.same, result.line) == (False, position + 1)


def test_a_record_that_just_ends_is_no_difference(run, tmp_path):
    """A process that was killed stops writing anywhere."""
    lines = run[0].read_text(encoding="utf-8").splitlines()
    position = next(n for n, line in enumerate(lines) if '"out"' in line)
    copy = tmp_path / "cut.jsonl"
    copy.write_text("\n".join(lines[:position]) + "\n", encoding="utf-8")
    assert replay.replay(copy).same


def test_a_record_of_inputs_alone_just_runs(run, tmp_path):
    """What a converted bag will be: the first line and the inputs, with nothing to compare."""
    lines = run[0].read_text(encoding="utf-8").splitlines()
    only = [lines[0]] + [line for line in lines[1:] if '"in"' in line]
    copy = tmp_path / "inputs.jsonl"
    copy.write_text("\n".join(only) + "\n", encoding="utf-8")
    result = replay.replay(copy)
    full = replay.replay(run[0])
    assert (result.compared, result.same) == (False, True)
    assert (result.inputs, result.inside, result.commands) == (
        full.inputs, full.inside, full.commands)


@pytest.mark.parametrize("seed", ["1", "2"])
def test_it_plays_back_the_same_in_another_process(run, seed):
    """Another process hashes strings differently, so an order taken from a set would show."""
    done = subprocess.run(
        [sys.executable, str(REPO / "tools" / "replay.py"), str(run[0])],
        env=dict(os.environ, PYTHONHASHSEED=seed, PYTHONDONTWRITEBYTECODE="1"),
        capture_output=True, text=True, timeout=120)
    assert done.returncode == 0, done.stdout + done.stderr
    assert "same as recorded" in done.stdout


def test_the_tool_says_where_it_differs(run, tmp_path):
    def move_it(entry):
        if entry.get("out") == "MoveCmd":
            entry["x"] += 1.0
            return True
        return False

    copy, number = changed(run[0], tmp_path, move_it)
    done = subprocess.run(
        [sys.executable, str(REPO / "tools" / "replay.py"), str(copy)],
        env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1"),
        capture_output=True, text=True, timeout=120)
    assert done.returncode == 1
    assert f"line {number} differs" in done.stdout
