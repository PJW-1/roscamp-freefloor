#!/usr/bin/env python3
"""Play back a record of the FMS on the two cores, with no ROS, and compare what comes out.

    ros2 run fleet_manager fms_node --record run.jsonl      (or record:=run.jsonl on pc.launch.py)
    python3 tools/replay.py run.jsonl

The first line says what the core was started with: robots, parameters and the map itself. Every
line after it has the time `t` and one of:

    {"t": 12.5, "in": "on_robot_state", "args": ["pinky1", 1.2, 2.0, 0.0, "MOVING", 90.0]}
    {"t": 12.6, "in": "tick", "args": []}
    {"t": 12.6, "inside": "DispatchCmd", "goal_id": 3, "station_id": "WORK", "robot_id": ""}
    {"t": 12.6, "out": "MoveCmd", "robot": "pinky1", "goal_id": 4, "target_id": "N", ...}

`in` is a method of FmsCore and `args` its arguments. A run is one `in` line after another, each
followed by what it caused. `inside` is what the two cores said to each other and `out` what went
to ROS. The clock is `t`: a core reads one time for the whole of one call, live as well.

A record of inputs alone, as a converted bag would be, just runs and reports what came out. Such a
file needs its own `tick` lines, 10 a second by default on the real node, and its first line.
A record may end anywhere, as one does when the process is killed: what it has is compared.

Exit code 0 if everything is the same, 1 at the first difference.
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


def _count(lines: list[str], key: str) -> int:
    return sum(key in json.loads(line) for line in lines[1:])


def _arguments(entry: dict) -> list:
    if entry["in"] == "on_order_board":
        mode, orders = entry["args"]
        return [mode, [OrderRecord(**order) for order in orders]]
    return entry["args"]


def replay(path) -> Result:
    lines = Path(path).read_text(encoding="utf-8").splitlines()
    head = json.loads(lines[0])
    now = [0.0]
    core = FmsCore(SiteMap(yaml.safe_load(head["map"]), head["map"]), FleetParams(**head["params"]),
                   head["robots"], lambda: now[0])
    produced: list[str] = []
    core.record_to(produced.append)
    for line in lines[1:]:
        entry = json.loads(line)
        if "in" in entry:
            now[0] = entry["t"]
            getattr(core, entry["in"])(*_arguments(entry))
    inputs, inside, commands = (_count(produced, key) for key in ("in", "inside", "out"))
    if not _count(lines, "out") and not _count(lines, "inside"):
        return Result(inputs, inside, commands, False, True)
    for number, (old, new) in enumerate(zip(lines, produced, strict=False), start=1):
        if old != new:
            return Result(inputs, inside, commands, True, False, number, old, new)
    if len(lines) > len(produced):
        return Result(inputs, inside, commands, True, False, len(produced) + 1,
                      lines[len(produced)], "(nothing came out)")
    return Result(inputs, inside, commands, True, True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("record", help="the JSONL file fms_node wrote with --record")
    result = replay(parser.parse_args().record)
    print(f"{result.inputs} inputs, {result.inside} inside calls, {result.commands} commands")
    if not result.compared:
        print("inputs only: nothing to compare with")
    elif result.same:
        print("same as recorded")
    else:
        print(f"line {result.line} differs\n recorded: {result.recorded}\n produced: "
              f"{result.produced}")
    return 0 if result.same else 1


if __name__ == "__main__":
    sys.exit(main())
