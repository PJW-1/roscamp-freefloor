"""FleetManager keeps a record of what came in and what came of it. tools/replay.py replays it."""
import inspect
import itertools
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest
import yaml
from gg_fleet_core.common.commands import OrderRecord
from gg_fleet_core.common.params import load_params
from gg_fleet_core.manager import FleetManager
from gg_fleet_core.traffic.lane_map import LaneMap, load_lane_map
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
    world = SimWorld(seed=3, site=load_lane_map(), orders=six_orders(), faults=FAULTS)
    lines = []
    world.core.record_to(lines.append)
    world.run_until(world.done)
    path = tmp_path_factory.mktemp("records") / "run.jsonl"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path, lines


def run_tool(path):
    return subprocess.run([sys.executable, str(REPO / "tools" / "replay.py"), str(path)],
                          env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1"),
                          capture_output=True, text=True, timeout=120)


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
        "on_goal_not_sent": ("r1", 7),
        "on_dock_result": ("r1", 7, True, 1.0, 1.0, 0.5),
        "on_fill_result": (7, True, {"A": 1}, {}, ""),
        "on_swap_result": (7, True, True, True, ""),
        "confirm_clear": ("r1", ["P-6"], "lifted off"),
        "tick": (),
    }
    public = {name for name, _ in inspect.getmembers(FleetManager, inspect.isfunction)
              if name.startswith("on_")}
    assert public <= set(calls), public - set(calls)
    core = FleetManager(load_lane_map(), load_params(), ["r1"], lambda: 1.5)
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
    site = load_lane_map()
    core = FleetManager(site, load_params(), ["r1"], lambda: next(ticks))
    core.on_robot_state("r1", *site.waypoint_pose("P-6"), 0.0, "IDLE")
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
    """A process that was killed stops writing anywhere. Only what the last input caused can be
    past the end, and the tool says how much of that it could not compare."""
    lines = run[0].read_text(encoding="utf-8").splitlines()
    position = next(n for n, line in enumerate(lines) if '"out"' in line)
    copy = tmp_path / "cut.jsonl"
    copy.write_text("\n".join(lines[:position]) + "\n", encoding="utf-8")
    result = replay.replay(copy)
    assert result.same
    assert result.uncompared > 0
    done = run_tool(copy)
    assert done.returncode == 0
    assert f"{result.uncompared} lines past its end were not compared" in done.stdout


def test_a_whole_record_has_nothing_left_to_compare(run):
    assert replay.replay(run[0]).uncompared == 0


def test_half_a_line_at_the_end_is_left_out(run, tmp_path):
    """Killed in the middle of a write."""
    text = run[0].read_text(encoding="utf-8")
    copy = tmp_path / "half.jsonl"
    copy.write_text(text[:-5], encoding="utf-8")
    result = replay.replay(copy)
    assert result.same and result.cut_line
    out = run_tool(copy).stdout
    assert "the last line was cut off" in out
    assert "as far as it goes" in out


def test_a_last_line_with_no_line_end_still_counts(run, tmp_path):
    text = run[0].read_text(encoding="utf-8")
    copy = tmp_path / "whole.jsonl"
    copy.write_text(text.rstrip("\n"), encoding="utf-8")
    result = replay.replay(copy)
    assert result.same and not result.cut_line and result.uncompared == 0


@pytest.mark.parametrize("content", ["", '{"robots": ["pinky'])
def test_a_record_with_no_first_line_is_refused(tmp_path, content):
    copy = tmp_path / "nothing.jsonl"
    copy.write_text(content, encoding="utf-8")
    done = run_tool(copy)
    assert done.returncode == 2, done.stdout + done.stderr
    assert "first line" in done.stdout
    assert "Traceback" not in done.stderr


def test_a_record_with_no_inputs_proves_nothing(run, tmp_path):
    first = run[0].read_text(encoding="utf-8").splitlines()[0]
    copy = tmp_path / "head.jsonl"
    copy.write_text(first + "\n", encoding="utf-8")
    done = run_tool(copy)
    assert done.returncode == 2, done.stdout + done.stderr
    assert "no inputs" in done.stdout


def test_a_line_in_the_middle_that_is_not_json_is_refused(run, tmp_path):
    lines = run[0].read_text(encoding="utf-8").splitlines()
    lines[5] = lines[5][:10]
    copy = tmp_path / "broken.jsonl"
    copy.write_text("\n".join(lines) + "\n", encoding="utf-8")
    done = run_tool(copy)
    assert done.returncode == 2, done.stdout + done.stderr
    assert "line 6" in done.stdout
    assert "Traceback" not in done.stderr


def test_a_record_that_lost_its_commands_does_not_pass_as_inputs_alone(run, tmp_path):
    lines = run[0].read_text(encoding="utf-8").splitlines()
    only = [lines[0]] + [line for line in lines[1:] if '"in"' in line]
    copy = tmp_path / "lost.jsonl"
    copy.write_text("\n".join(only) + "\n", encoding="utf-8")
    result = replay.replay(copy)
    assert result.compared and not result.same


def test_commands_in_the_file_are_compared_whatever_the_first_line_says(run, tmp_path):
    lines = run[0].read_text(encoding="utf-8").splitlines()
    head = json.loads(lines[0])
    del head["recorded"]
    lines[0] = json.dumps(head)
    copy = tmp_path / "old.jsonl"
    copy.write_text("\n".join(lines) + "\n", encoding="utf-8")
    result = replay.replay(copy)
    assert result.compared and result.same, result

    position = next(n for n, line in enumerate(lines) if '"out": "MoveCmd"' in line)
    entry = json.loads(lines[position])
    entry["x"] += 1.0
    lines[position] = json.dumps(entry)
    copy.write_text("\n".join(lines) + "\n", encoding="utf-8")
    result = replay.replay(copy)
    assert (result.same, result.line) == (False, position + 1)


def test_a_file_that_is_not_there_is_refused(tmp_path):
    done = run_tool(tmp_path / "gone.jsonl")
    assert done.returncode == 2, done.stdout + done.stderr
    assert "Traceback" not in done.stderr


@pytest.mark.parametrize("head", [{}, {"robots": ["pinky1"], "map": "x"}])
def test_a_first_line_that_lacks_something_is_refused(tmp_path, head):
    copy = tmp_path / "head.jsonl"
    copy.write_text(json.dumps(head) + '\n{"t": 0.0, "in": "tick", "args": []}\n', encoding="utf-8")
    done = run_tool(copy)
    assert done.returncode == 2, done.stdout + done.stderr
    assert "first line" in done.stdout
    assert "Traceback" not in done.stderr


def test_parameters_this_code_does_not_know_are_refused(run, tmp_path):
    lines = run[0].read_text(encoding="utf-8").splitlines()
    head = json.loads(lines[0])
    head["params"]["from_another_version"] = 1
    lines[0] = json.dumps(head)
    copy = tmp_path / "version.jsonl"
    copy.write_text("\n".join(lines) + "\n", encoding="utf-8")
    done = run_tool(copy)
    assert done.returncode == 2, done.stdout + done.stderr
    assert "parameters" in done.stdout
    assert "Traceback" not in done.stderr


def test_a_record_has_to_start_before_the_first_input():
    """Nothing in the first line says what the core had been through by then."""
    core = FleetManager(load_lane_map(), load_params(), ["pinky1"], lambda: 0.0)
    core.tick()
    with pytest.raises(RuntimeError, match="first input"):
        core.record_to([].append)


def test_a_map_with_no_text_cannot_be_recorded():
    """A replay builds the map from the text in the first line."""
    site = LaneMap(yaml.safe_load(load_lane_map().source_text))
    core = FleetManager(site, load_params(), ["pinky1"], lambda: 0.0)
    with pytest.raises(ValueError, match="text"):
        core.record_to([].append)


@pytest.mark.parametrize("bundle", [True, False])
def test_a_two_way_setting_given_in_code_comes_back_in_the_replay(bundle):
    """The map file says one thing and the call another: the record has what was really used."""
    site = load_lane_map(bundle_two_way=bundle)
    lines = []
    FleetManager(site, load_params(), ["pinky1"], lambda: 0.0).record_to(lines.append)
    core = replay.start_core(json.loads(lines[0]), lambda: 0.0)
    assert core.site.zones == site.zones


def test_a_record_of_inputs_alone_just_runs(run, tmp_path):
    """What a converted bag will be: a first line that does not claim to be a record, and the
    inputs. There is nothing to compare."""
    lines = run[0].read_text(encoding="utf-8").splitlines()
    head = json.loads(lines[0])
    del head["recorded"]
    only = [json.dumps(head)] + [line for line in lines[1:] if '"in"' in line]
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
