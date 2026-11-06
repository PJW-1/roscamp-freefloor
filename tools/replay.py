#!/usr/bin/env python3
"""Play back a record of the FMS on the two cores, with no ROS, and compare what comes out.

    ros2 run fleet_manager fms_node --record run.jsonl      (or record:=run.jsonl on pc.launch.py)
    python3 tools/replay.py run.jsonl

The first line says what the core was started with: robots, parameters, the map itself and its
two-way setting. Every line after it has the time `t` and one of:

    {"t": 12.5, "in": "on_robot_state", "args": ["pinky1", 1.2, 2.0, 0.0, "MOVING", 90.0]}
    {"t": 12.6, "in": "tick", "args": []}
    {"t": 12.6, "inside": "DispatchCmd", "goal_id": 3, "station_id": "WORK", "robot_id": ""}
    {"t": 12.6, "out": "MoveCmd", "robot": "pinky1", "goal_id": 4, "target_id": "N", ...}

`in` is a method of FmsCore and `args` its arguments. A run is one `in` line after another, each
followed by what it caused. `inside` is what the two cores said to each other and `out` what went
to ROS. The clock is `t`: a core reads one time for the whole of one call, live as well.
What the node decides on its own never goes through a core and is not in the record: the
CANCEL_UNCONFIRMED and ORDER_REPORT_REFUSED events, and the replies to cancels and order
reports they rest on. Both events go out on /central/events, so a bag has them.

`"recorded": true` in the first line says the core itself wrote this, so what it caused is in the
file and has to match. A record of inputs alone, as a converted bag would be, leaves that out and
has no `inside` or `out` lines: it just runs and reports what came out. Such a file needs its own
`tick` lines, 10 a second by default on the real node, and its first line.

A record may end anywhere, even in the middle of a line, as one does when the process is killed.
What it has is compared. Lines after the end can only be what its last input caused, and the
tool says how many it could not compare.

Exit code 0 if everything is the same, 1 at the first difference, 2 if this cannot be played back.
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "pc" / "fleet_core"))

from fleet_core.commands import OrderRecord  # noqa: E402
from fleet_core.fms_core import FmsCore  # noqa: E402
from fleet_core.params import FleetParams  # noqa: E402
from fleet_core.site_map import SiteMap  # noqa: E402


@dataclass(frozen=True)
class Result:
    inputs: int
    inside: int
    commands: int
    compared: bool  # False for a record of inputs alone
    same: bool
    line: int = 0  # first line that differs, counting from 1
    recorded: str = ""
    produced: str = ""
    uncompared: int = 0  # lines that came out after the record had ended
    cut_line: bool = False  # the last line was only half written, and was left out


def _count(lines: list[str], key: str) -> int:
    return sum(key in json.loads(line) for line in lines[1:])


def _arguments(entry: dict) -> list:
    if entry["in"] == "on_order_board":
        mode, orders = entry["args"]
        return [mode, [OrderRecord(**order) for order in orders]]
    return entry["args"]


def _read(path) -> tuple[list[str], list[dict], bool]:
    """The lines, what they say, and whether half a line at the end was left out."""
    text = Path(path).read_text(encoding="utf-8")
    lines = text.splitlines()
    entries = []
    for number, line in enumerate(lines, start=1):
        try:
            entries.append(json.loads(line))
        except json.JSONDecodeError as error:
            if number == len(lines) and not text.endswith("\n"):
                return lines[:-1], entries, True
            raise ValueError(f"line {number} is not JSON: {error}") from error
    return lines, entries, False


def start_core(head: dict, clock) -> FmsCore:
    missing = {"robots", "params", "map"} - set(head)
    if missing:
        raise ValueError(f"the first line has no {sorted(missing)}")
    try:
        params = FleetParams(**head["params"])
    except TypeError as error:
        raise ValueError(f"these parameters do not fit this code: {error}") from error
    site = SiteMap(yaml.safe_load(head["map"]), head["map"], head.get("bundle_two_way"))
    return FmsCore(site, params, head["robots"], clock)


def replay(path) -> Result:
    lines, entries, cut_line = _read(path)
    if not entries:
        raise ValueError("no first line, so this is not a record")
    head, *rest = entries
    if not any("in" in entry for entry in rest):
        raise ValueError("no inputs, so there is nothing to play back")
    now = [0.0]
    core = start_core(head, lambda: now[0])
    produced: list[str] = []
    core.record_to(produced.append)
    for entry in rest:
        if "in" in entry:
            now[0] = entry["t"]
            getattr(core, entry["in"])(*_arguments(entry))
    counts = tuple(_count(produced, key) for key in ("in", "inside", "out"))
    if not head.get("recorded") and not any("inside" in e or "out" in e for e in rest):
        return Result(*counts, False, True, cut_line=cut_line)
    # Not the first line: the core was built from it, and it is no decision of the core.
    for number, (old, new) in enumerate(zip(lines[1:], produced[1:], strict=False), start=2):
        if old != new:
            return Result(*counts, True, False, number, old, new, cut_line=cut_line)
    if len(lines) > len(produced):
        return Result(*counts, True, False, len(produced) + 1, lines[len(produced)],
                      "(nothing came out)", cut_line=cut_line)
    return Result(*counts, True, True, uncompared=len(produced) - len(lines), cut_line=cut_line)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("record", help="the JSONL file fms_node wrote with --record")
    try:
        result = replay(parser.parse_args().record)
    except (ValueError, OSError) as error:
        print(f"cannot play this back: {error}")
        return 2
    print(f"{result.inputs} inputs, {result.inside} inside calls, {result.commands} commands")
    if result.cut_line:
        print("the last line was cut off and left out")
    if not result.compared:
        print("inputs only: nothing to compare with")
    elif result.same and result.uncompared:
        print(f"same as recorded, as far as it goes: {result.uncompared} lines past its end "
              "were not compared")
    elif result.same and result.cut_line:
        print("same as recorded, as far as it goes")
    elif result.same:
        print("same as recorded")
    else:
        print(f"line {result.line} differs\n recorded: {result.recorded}\n produced: "
              f"{result.produced}")
    return 0 if result.same else 1


if __name__ == "__main__":
    sys.exit(main())
