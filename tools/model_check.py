#!/usr/bin/env python3
"""Exhaustive check of the real TrafficCore on a lane map, over every order of calls.

    python3 tools/model_check.py --map sim/maps/oneway_shortcut.yaml --robots 3 --bundle off

The state is, per robot, its mission phase, what is left of its path, its running goal and where
it stands, plus the zone locks and how many open calls there are. Orders are not in it. At every
step any one of these can happen, followed by one tick of the core:

    open call       dispatch(WORK) with no robot named, up to one per robot
    cancel call     dispatch_cancel of an open call
    send X          dispatch(OUTBOUND, X)
    cancel X        dispatch_cancel of X's dispatch
    release X       release(X)
    pass X          X reports it is past d along its lane
    arrive X        X finishes its move
    docked X        X finishes its dock
    report X        one more report from X (only with --confirm above 1)
    nothing         only the tick

A call that cannot change anything (a second dispatch, a release with nothing to release) is the
same step as `nothing`, so it is left out. test_model_check_tool.py compares the two on a small map.

Output is (states, deadlocks, starved), and one shortest way into a deadlock.
  deadlock: a state with a wait cycle. The check does not go on from there, because time stands
    still here and the retreat of the real core never comes. A later cancel could still undo one,
    and it counts anyway.
  starved: (state, robot) pairs where the robot still has somewhere to drive and no sequence of
    events other than a cancel ever gives it a move again. Every member of a deadlock counts. Zero
    means a way out always exists, not that every schedule takes it: an order of events that keeps
    passing a robot over is not counted. Against that there is the FIFO in zones.py.

What it does not cover: geometry beyond the lock rules, faults, timing and comm loss. A pose is
only "on the node" or "just past d on the lane".

By default a zone is freed on the first report past it, not the third. A robot that has passed a
zone and not freed it yet is in no queue, so it should not be part of a wait cycle through that
zone. That is an argument, and the default search does not prove it. --confirm 3 searches every
order of those reports too, at the cost of many more states. The worklog has both where both ran.
A `report` repeats the same pose. A real report back inside the zone would reset the count, which
delays a release and reaches no lock state the search has not seen: an argument again.
"""
from __future__ import annotations

import argparse
import json
import multiprocessing
import os
import sys
from array import array
from collections.abc import Iterable
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "pc" / "fleet_core"))

from fleet_core.deadlock import find_cycle, wait_graph  # noqa: E402
from fleet_core.params import load_params  # noqa: E402
from fleet_core.site_map import load_site_map  # noqa: E402
from fleet_core.traffic_core import DOCK, MOVE, TrafficCore, _Dispatch, _Goal  # noqa: E402

PAST = 0.01  # how far beyond d a robot reports when it passes
NAMED = 100  # goal ids of named dispatches start here, so they never meet an open call's
CHUNK = 500  # states per job when several processes work
ROBOT_FIELDS = 12  # the length of the part of a state that describes one robot


class Model:
    """One core, set to any stored state, so a step costs no copying of objects."""

    def __init__(self, site, robots: int, start=None, params=None, confirm: int = 1):
        self.confirm = confirm
        params = replace(params or load_params(), release_confirm_samples=confirm)
        self.site = site
        self.d = params.release_distance_m
        self.names = [f"r{i + 1}" for i in range(robots)]
        self.core = TrafficCore(site, params, self.names, lambda: 0.0)
        _remember_plans(site)
        self._group = self._conflict_groups()
        spots = start or list(site.parking[-robots:])[::-1]
        self.pose = dict(zip(self.names, spots, strict=True))
        for name, node in self.pose.items():
            self.core.on_robot_state(name, *site.cell_pose(node), 0.0, "IDLE")
        refused = [m.name for m in self.core._missions.values() if not m.placed]
        if refused:
            raise ValueError(f"{refused} cannot start on {spots}: the lock is taken")
        self.core.on_run_mode("RUN")
        self._tick()
        self.initial = self.capture()
        self.event_list = self._all_events()
        self.event_code = {event: code for code, event in enumerate(self.event_list)}

    def _all_events(self) -> list[tuple]:
        found = [("nothing",), ("open call",), ("cancel call",)]
        for name in self.names:
            found += [(kind, name) for kind in ("send", "cancel", "release", "pass", "arrive",
                                                "docked", "report")]
        return found

    def _conflict_groups(self) -> dict:
        """Order of requests matters only inside a queue and across a conflict pair."""
        group = {zone: zone for zone in self.core.zones.snapshot()}
        for zone in group:
            for other in self.core.zones._conflicts[zone]:
                old, new = group[other], group[zone]
                for key, value in group.items():
                    if value == old:
                        group[key] = new
        return group

    # ---- state ----------------------------------------------------------------------------

    def capture(self) -> tuple:
        core = self.core
        robots = []
        for name in self.names:
            m = core._missions[name]
            first = self._first_needed(m)
            goal = None
            if m.goal:
                target = m.goal.target_idx - first if m.goal.kind == MOVE else 0
                goal = (m.goal.kind, target, m.goal.step)
            settle = None if m.settle_idx is None else m.settle_idx - first
            counted = ()
            if self.confirm > 1:
                counted = tuple(sorted((z, min(n, self.confirm)) for z, n in m.outside.items()))
            robots.append((m.phase, tuple(m.path[first:]), m.idx - first, m.dest, goal, settle,
                           m.called, m.dispatch.station_id if m.dispatch else None, m.hold,
                           m.await_pick, self.pose[name], counted))
        zones = core.zones
        holders = tuple(sorted((z, h) for z, h in zones._holder.items() if h))
        requests = sorted(zones._arrived.items(), key=lambda item: item[1])
        requests = sorted((self._group[z], rank, z, r)
                          for rank, ((z, r), _) in enumerate(requests))
        return (*robots, (holders, tuple((z, r) for _, _, z, r in requests)),
                len(core._open_calls))

    def _first_needed(self, m) -> int:
        """What is behind the robot counts only as far back as a zone it still holds."""
        held = self.core.zones._held.get(m.name, ())
        for index in range(m.idx):
            if self.site.zone_of[m.path[index]] in held:
                return index
        return m.idx

    def restore(self, state: tuple) -> None:
        core = self.core
        for number, name in enumerate(self.names):
            m = core._missions[name]
            (m.phase, path, m.idx, m.dest, goal, m.settle_idx, m.called, station, m.hold,
             m.await_pick, self.pose[name], counted) = state[number]
            m.path = list(path)
            m.goal = _Goal(goal[0], 1, goal[1], goal[2]) if goal else None
            m.dispatch = _Dispatch(NAMED + number, station) if station else None
            m.x, m.y = self.xy(self.pose[name])
            m.outside = dict(counted)
            m.off_lane, m.off_lane_warned, m.wait_zone, m.attempts = 0, False, None, 0
        holders, queue = state[len(self.names)]
        zones = core.zones
        for zone in zones._holder:
            zones._holder[zone] = None
            zones._queue[zone] = []
        zones._held = {name: [] for name in self.names}
        for zone, holder in holders:
            zones._holder[zone] = holder
            zones._held[holder].append(zone)
        zones._arrived = {}
        for rank, (zone, robot) in enumerate(queue):
            zones._queue[zone].append(robot)
            zones._arrived[(zone, robot)] = rank
        zones._arrivals = len(queue)
        core._open_calls = list(range(state[-1]))
        core._deadlock_since, core._deadlock_acted, core._retreating = None, False, None
        core._out = []

    def xy(self, pose) -> tuple[float, float]:
        if isinstance(pose, tuple) and pose[0] == "lane":
            lane = self.site.lane(pose[1], pose[2])
            return lane.ax + lane.ux * (self.d + PAST), lane.ay + lane.uy * (self.d + PAST)
        return self.site.cell_pose(pose)

    def has_cycle(self) -> bool:
        return bool(find_cycle(wait_graph(self.core.zones)))

    # ---- events ---------------------------------------------------------------------------

    def _tick(self, reporter: str | None = None) -> None:
        """Everyone reports, or with several reports to count only the robot that moved."""
        core = self.core
        reporters = self.names if self.confirm == 1 else [reporter] if reporter else []
        for name in reporters:
            m = core._missions[name]
            core.on_robot_state(name, m.x, m.y, 0.0, "IDLE")
        core.tick()

    def events(self, state: tuple) -> list[tuple]:
        """Everything that can happen now. The state must be restored already."""
        core = self.core
        calls = state[-1]
        found = [("nothing",)]
        if calls < len(self.names):
            found.append(("open call",))
        if calls:
            found.append(("cancel call",))
        for name in self.names:
            m = core._missions[name]
            if m.dispatch:
                found.append(("cancel", name))
            else:
                found.append(("send", name))
            if m.hold:
                found.append(("release", name))
            if m.goal and m.goal.kind == MOVE:
                if m.idx < m.goal.target_idx and self.pose[name] != self._lane_pose(m):
                    found.append(("pass", name))
                found.append(("arrive", name))
            elif m.goal and m.goal.kind == DOCK:
                found.append(("docked", name))
            if self.confirm > 1:
                found.append(("report", name))
        return found

    @staticmethod
    def _lane_pose(m) -> tuple:
        return ("lane", m.path[m.idx], m.path[m.idx + 1])

    def apply(self, event: tuple) -> None:
        core = self.core
        kind = event[0]
        if kind == "open call":
            core.on_dispatch(len(core._open_calls), core._work)
        elif kind == "cancel call":
            core.on_dispatch_cancel(core._open_calls[-1])
        elif kind == "send":
            core.on_dispatch(NAMED + self.names.index(event[1]), core._outbound, event[1])
        elif kind == "cancel":
            core.on_dispatch_cancel(core._missions[event[1]].dispatch.goal_id)
        elif kind == "release":
            core.on_release(event[1])
        elif kind == "pass":
            m = core._missions[event[1]]
            self.pose[event[1]] = self._lane_pose(m)
            m.x, m.y = self.xy(self.pose[event[1]])
        elif kind == "arrive":
            m = core._missions[event[1]]
            self.pose[event[1]] = m.path[m.goal.target_idx]
            m.x, m.y = self.xy(self.pose[event[1]])
            core.on_move_result(event[1], m.goal.id, True, "")
        elif kind == "docked":
            core.on_dock_result(event[1], core._missions[event[1]].goal.id, True)
        self._tick(event[1] if kind in ("pass", "arrive", "report") else None)

    def expand(self, states: Iterable[tuple]) -> list:
        """For each state: None if it is a deadlock, otherwise (event code, successor) pairs."""
        answers = []
        for state in states:
            self.restore(state)
            if self.has_cycle():
                answers.append(None)
                continue
            seen = set()
            successors = []
            for event in self.events(state):
                self.restore(state)
                self.apply(event)
                successor = self.capture()
                key = (successor, _is_cancel(event))
                if successor != state and key not in seen:
                    seen.add(key)
                    successors.append((self.event_code[event], successor))
            answers.append(successors)
        return answers


def _remember_plans(site) -> None:
    """A plan depends only on the map, and the core asks for the same ones all the time."""
    if "plan" in vars(site):
        return
    plans: dict = {}
    real_plan = site.plan

    def plan(a, b):
        found = plans.get((a, b))
        if found is None:
            found = plans[(a, b)] = real_plan(a, b)
        return found

    site.plan = plan


def _is_cancel(event: tuple) -> bool:
    return event[0] in ("cancel", "cancel call")


_worker_model: Model | None = None


def _start_worker(model_class, site, robots, start, params, confirm) -> None:
    global _worker_model
    _worker_model = model_class(site, robots, start, params, confirm)


def _work(states: list) -> list:
    return _worker_model.expand(states)


@dataclass
class Result:
    states: int
    deadlocks: int
    starved: int  # -1 when the search stopped early
    example: list
    complete: bool
    local_states: int


class _Store:
    """Every known state, in about 150 bytes each: most parts recur a great many times."""

    def __init__(self, robots: int):
        self.robots = robots
        self.part_ids: dict = {}
        self.parts: list = []
        self.columns = [array("I") for _ in range(robots + 1)]
        self.calls = array("B")
        self.index: dict[int, int] = {}

    def _part(self, value) -> int:
        found = self.part_ids.get(value)
        if found is None:
            found = self.part_ids[value] = len(self.parts)
            self.parts.append(value)
        return found

    def add(self, state: tuple) -> tuple[int, bool]:
        """Returns the state's number, and whether it is new."""
        key = state[-1]
        ids = []
        for part in state[:-1]:
            part_id = self._part(part)
            ids.append(part_id)
            key = key << 32 | part_id
        found = self.index.get(key)
        if found is not None:
            return found, False
        number = len(self.index)
        self.index[key] = number
        for column, part_id in zip(self.columns, ids, strict=True):
            column.append(part_id)
        self.calls.append(state[-1])
        return number, True

    def state(self, number: int) -> tuple:
        parts = self.parts
        return (*(parts[column[number]] for column in self.columns), self.calls[number])

    def __len__(self) -> int:
        return len(self.index)


def check(site, robots: int = 3, start=None, params=None, max_states: int = 0, workers: int = 1,
          progress=None, model_class=Model, stop_at_first: bool = False,
          confirm: int = 1) -> Result:
    model = model_class(site, robots, start, params, confirm)
    store = _Store(robots)
    store.add(model.initial)
    parents = array("I", [0])
    parent_events = array("H", [0])
    edges_from, edges_to = array("I"), array("I")  # without cancels: for the starved count
    dead: list[int] = []
    cancels = [_is_cancel(event) for event in model.event_list]
    frontier = [0]
    complete = True
    pool = None
    if workers > 1:
        pool = multiprocessing.get_context("fork").Pool(
            workers, _start_worker, (model_class, site, robots, start, params, confirm))
    try:
        while frontier:
            if max_states and len(store) >= max_states:
                complete = False
                break
            if stop_at_first and dead:
                complete = False
                break
            chunks = [frontier[i:i + CHUNK] for i in range(0, len(frontier), CHUNK)]
            jobs = ([store.state(n) for n in chunk] for chunk in chunks)
            answers = pool.imap(_work, jobs) if pool else map(model.expand, jobs)
            next_frontier = []
            for chunk, answer in zip(chunks, answers, strict=True):
                for node, successors in zip(chunk, answer, strict=True):
                    if successors is None:
                        dead.append(node)
                        continue
                    for code, successor in successors:
                        found, new = store.add(successor)
                        if new:
                            parents.append(node)
                            parent_events.append(code)
                            next_frontier.append(found)
                        if found != node and not cancels[code]:
                            edges_from.append(node)
                            edges_to.append(found)
            frontier = next_frontier
            if progress:
                progress(len(store), len(frontier), len(dead))
    finally:
        if pool:
            pool.terminate()
    starved = _count_starved(store, robots, edges_from, edges_to) if complete else -1
    example = _way_to(dead, parents, parent_events, model.event_list)
    robot_parts = sum(1 for part in store.parts if len(part) == ROBOT_FIELDS)
    return Result(len(store), len(dead), starved, example, complete, robot_parts)


def _count_starved(store: _Store, robots: int, edges_from, edges_to) -> int:
    """For each robot: states that want a drive, less those from which a drive can be reached."""
    wants = np.zeros(len(store.parts), dtype=bool)
    drives = np.zeros(len(store.parts), dtype=bool)
    for number, part in enumerate(store.parts):
        if len(part) == ROBOT_FIELDS:
            phase, path, idx, _, goal, settle, *_ = part
            drives[number] = goal is not None
            wants[number] = phase.startswith("TO_") and (
                goal is not None or settle is not None or idx < len(path) - 1)
    sources = np.frombuffer(edges_from, dtype=np.uint32)
    targets = np.frombuffer(edges_to, dtype=np.uint32)
    total = 0
    for i in range(robots):
        column = np.frombuffer(store.columns[i], dtype=np.uint32)
        reaches = drives[column].copy()
        while True:
            step = reaches[targets] & ~reaches[sources]
            if not step.any():
                break
            reaches[sources[step]] = True
        total += int((wants[column] & ~reaches).sum())
    return total


def _way_to(dead: list[int], parents, parent_events, event_list) -> list[str]:
    if not dead:
        return []
    way = []
    node = dead[0]
    while node:
        way.append(" ".join(event_list[parent_events[node]]))
        node = parents[node]
    return way[::-1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("--map", default=None,
                        help="lane map YAML (default: the one in fleet_core)")
    parser.add_argument("--robots", type=int, default=3)
    parser.add_argument("--bundle", choices=("file", "on", "off"), default="file",
                        help="two-way bundling: as the map says, or forced")
    parser.add_argument("--start", default=None, help="comma-separated start nodes")
    parser.add_argument("--max-states", type=int, default=0, help="stop early and say so")
    parser.add_argument("--stop-at-first", action="store_true",
                        help="stop at the first level with a deadlock, for a map too big to finish")
    parser.add_argument("--workers", type=int, default=1, help="processes working on the states")
    parser.add_argument("--confirm", type=int, default=1,
                        help="reports past a zone that free it. The real core takes 3")
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    bundle = {"file": None, "on": True, "off": False}[args.bundle]
    site = load_site_map(args.map, bundle_two_way=bundle)
    start = args.start.split(",") if args.start else None

    def show(known, frontier, dead):
        print(f"  {known} known, {frontier} to do, {dead} deadlocks", file=sys.stderr, flush=True)

    result = check(site, args.robots, start, max_states=args.max_states, workers=args.workers,
                   progress=None if args.quiet else show, stop_at_first=args.stop_at_first,
                   confirm=args.confirm)
    if args.json:
        print(json.dumps(vars(result)))
        return 0
    print(f"({result.states}, {result.deadlocks}, {result.starved})"
          + ("" if result.complete else " stopped early, starved not counted"))
    if result.example:
        print("shortest way into a deadlock:")
        for step in result.example:
            print(f"  {step}")
    return 0


if __name__ == "__main__":
    os.environ.setdefault("PYTHONDONTWRITEBYTECODE", "1")
    sys.exit(main())
