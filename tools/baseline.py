#!/usr/bin/env python3
"""Baseline measurement (work order H6): run scenarios headless on the real fleet core and write
one CSV row per run plus a mean/min/max summary over seeds.

    python3 tools/baseline.py --out baseline.csv
    python3 tools/baseline.py --maps lane_map --scenarios blocked_lane --modes path --seeds 1 2

Runs are driven by SimWorld (pc/fleet_core/test/sim_world.py): the real FleetManager with fake
robots, a fake central server and fake arms on a 0.1 s fake clock. No ROS graph is started.
Pose noise is gaussian (like the ROS fake nodes) and seeded, so the same command gives the same
rows. A scenario file sets the orders, stock, robots, arm and dock times and faults; its own
seed, speed_factor and expectations are ignored. The file is first checked by the fake robots'
parser (sim/fake_robots/fake_robots/scenario.py), and missing values take its defaults (4 s per
item, 8 s swap, 3 s dock, 900 s timeout). Supported faults: `blocked`, and the operator
actions `operator_remove` (confirm_clear on every zone the robot holds, retried each second
until accepted; the robot leaves the floor only when accepted) and `operator_reinstate`
(reinstate, retried each second; the robot is put back on its start zone with the given basket
only when accepted). Other fault types and a non-zero arm fail_rate are refused.

Limits of SimWorld against the ROS fakes: SimWorld has one shelf per item kind, so the per-arm
stock is summed per kind. Which items can be placed stays the same (the fake workcell also gives
each item to an arm that has it), but the fake workcell picks with both arms at once while
SimWorld places one item after another, so fills that use both arms take longer here. SimWorld's
central keeps no separate `inventory` record, so a scenario whose `inventory` differs from the
summed stock (stock_shortage) is refused instead of measured differently.

Default plan (map: scenarios). Each metric needs a scenario where it can be non-zero:
    lane_map, two_lane_segments: basic_orders, blocked_lane, reinstate
    feature_map:                 feature_basic_orders, feature_blocked_lane
basic_orders gives the plain order time, waits, stops and detours; blocked_lane adds one object
(object time, detours); reinstate is the only one with operator actions. With --scenarios the
given scenarios run on every given map, except a scenario whose `map:` names another map.

Columns (seconds are fake-clock seconds):
    finished        before the scenario timeout every order got a final report, every operator
                    action was accepted and every robot still on the floor was PARKED; no
                    invariant broke. Only such runs have comparable waits and stops
    finish_s        tick of the last final order report ("6 orders" for the basic scenarios,
                    7 for reinstate); filled whenever the orders finished, even if the run did
                    not; 0 for a scenario without orders. An empty floor counts as all parked
    end_s           time the run stopped: all parked, or the timeout
    wait_<robot>_s  time that robot held a zone-wait ticket (it was in some zone's queue,
                    ZoneTable.waiting_for), sampled after every tick; wait_total_s is the sum
    object_still_s  robot-seconds lost to objects. Each robot that stands in front of an object
                    (the fake robot stops with its move goal held) opens its own window, which
                    closes when that robot moves again. In every step with at least one window
                    open, each robot that did not move adds one step, once, leaving out robots
                    docked or docking at a station and robots PARKED with nothing to do.
                    Empty when the scenario has no object
    object_hits     how many windows opened (an object that no robot ran into gives 0)
    interventions   accepted operator calls: confirm_clear plus reinstate
    operator_refused  operator calls the fleet refused (each is retried a second later)
    error_entries   times a robot's fleet state became ERROR; each one waits for an operator
    stops           a robot's true position changed in one 0.1 s step and not in the next, while
                    it was not at the last waypoint of the path the fleet planned for it (so an
                    arrival is not a stop). FakePinky has no deceleration, so slowdowns are not
                    counted. Standing still for a long time is one stop
    detours         DETOUR events
The run goes on after the last order until every robot still on the floor is PARKED, so waits
and stops cover the trip home. The summary (<out>_summary.csv and the table on stdout) gives
mean, min and max over seeds; `finished` over all runs, every other metric over finished runs
only (n says how many), because a run cut by the timeout keeps adding waits.
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from dataclasses import replace
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "pc" / "fleet_core"))
sys.path.insert(0, str(REPO / "pc" / "fleet_core" / "test"))
sys.path.insert(0, str(REPO / "sim" / "fake_robots"))

from fake_robots.scenario import expand_random, parse_scenario  # noqa: E402
from fleet_core.common.commands import SHIPPED, ReportOrderCmd  # noqa: E402
from fleet_core.common.params import load_params  # noqa: E402
from fleet_core.traffic.lane_map import load_lane_map  # noqa: E402
from sim_world import DT, SimWorld  # noqa: E402

SCENARIOS = REPO / "sim" / "fake_robots" / "scenarios"
MAPS = REPO / "sim" / "fake_robots" / "maps"
DEFAULT_MAP = "lane_map"
DEFAULT_PLAN = {
    "lane_map": ["basic_orders", "blocked_lane", "reinstate"],
    "feature_map": ["feature_basic_orders", "feature_blocked_lane"],
    "two_lane_segments": ["basic_orders", "blocked_lane", "reinstate"],
}
MODES = ("path", "point")
SEEDS = (1, 2, 3, 4, 5)
WORLD_FAULTS = ("blocked", "low_battery")
OPERATOR_FAULTS = ("operator_remove", "operator_reinstate")
MOVED_M = 1e-9  # a step with more displacement than this is a moving step
ARRIVED_M = 1e-6  # the fake robot snaps onto its goal, so arrival is exact
OPERATOR_RETRY_S = 1.0
COLUMNS = ["map", "scenario", "mode", "seed", "finished", "error", "orders_total",
           "orders_shipped", "finish_s", "end_s", "wait_total_s", "object_still_s",
           "object_hits", "interventions", "operator_refused", "error_entries", "stops",
           "detours"]
SUMMARY_COLUMNS = ["map", "scenario", "mode", "metric", "runs", "n", "mean", "min", "max"]
NOT_METRICS = {"map", "scenario", "mode", "seed", "error"}


class Meter:
    """Reads a world after each step. The world needs t, virtual, core and site as in SimWorld."""

    def __init__(self, robots, dt):
        self.dt = dt
        self.wait = {name: 0.0 for name in robots}
        self.stops = 0
        self.object_still = 0.0
        self.object_hits = 0
        self.error_entries = 0
        self._last: dict[str, tuple[float, float]] = {}
        self._moving: dict[str, bool] = {}
        self._state: dict[str, str] = {}
        self._windows: dict[str, float] = {}  # robot in front of an object -> its stopped_at
        self._closed: set[tuple] = set()  # (robot, stopped_at) of closed windows

    def observe(self, world) -> None:
        moved = {}
        for name, r in world.virtual.items():
            if r.removed:
                self._last.pop(name, None)
                self._moving[name] = False
                continue
            view = world.core.robot_view(name)
            if world.core.zones.waiting_for(name):
                self.wait[name] += self.dt
            state = view.get("state", "")
            if state == "ERROR" and self._state.get(name) != "ERROR":
                self.error_entries += 1
            self._state[name] = state
            last = self._last.get(name)
            moved[name] = last is not None and math.dist(last, (r.x, r.y)) > MOVED_M
            if self._moving.get(name) and not moved[name] and not self._arrived(world, r, view):
                self.stops += 1
            self._moving[name] = moved[name]
            self._last[name] = (r.x, r.y)
        self._watch_object(world, moved)

    @staticmethod
    def _arrived(world, r, view) -> bool:
        path = view.get("path") or []
        return not path or math.dist((r.x, r.y), world.site.waypoint_pose(path[-1])) <= ARRIVED_M

    def _watch_object(self, world, moved) -> None:
        for name, r in world.virtual.items():
            key = (name, r.stopped_at)
            if (not r.removed and r.stopped_at is not None and name not in self._windows
                    and key not in self._closed):
                self._windows[name] = r.stopped_at
                self.object_hits += 1
        if not self._windows:
            return
        for name, r in world.virtual.items():
            if r.removed or moved.get(name):
                continue
            at_station = bool(r.docked_at) or r.goal_kind == "dock"
            if not at_station and self._state.get(name) != "PARKED":
                self.object_still += self.dt
        for name, stopped_at in list(self._windows.items()):
            if moved.get(name) or world.virtual[name].removed:
                self._closed.add((name, stopped_at))
                del self._windows[name]


class Operator:
    """Plays the scenario's operator actions on the world and counts accepted and refused calls.

    The fake robot changes only when the fleet accepts: SimWorld.confirm_clear would take the
    robot off the floor even on a refusal, so the core is called directly.
    """

    def __init__(self, world, actions):
        self.world = world
        self.pending = sorted(actions, key=lambda a: a["at_s"])
        self.retry_at = 0.0
        self.accepted = 0
        self.refused = 0

    def act(self) -> None:
        world = self.world
        if not self.pending or world.t < max(self.pending[0]["at_s"], self.retry_at) - 1e-9:
            return
        action = self.pending[0]
        robot = action["robot"]
        if action["type"] == "operator_remove":
            zones = list(world.core.zones.held_by(robot))
            done = world.core.confirm_clear(robot, zones, "baseline operator") is not None
            if done:
                world.virtual[robot].removed = True
        else:
            # the fleet puts the robot on its map at the next report, which comes in the next
            # step, so the robot can still be moved after the answer
            done = world.core.reinstate(robot, action["basket"], "baseline operator")
            if done:
                self._put_back(robot, action["start"], action["basket"])
        if done:
            self.accepted += 1
            self.pending.pop(0)
        else:
            self.refused += 1
            self.retry_at = world.t + OPERATOR_RETRY_S

    def _put_back(self, robot, start, basket) -> None:
        world = self.world
        r = world.virtual[robot]
        r.x, r.y = world.site.waypoint_pose(min(world.site.zones[start]))
        r.removed = False
        r.goal_id, r.goal_kind, r.target, r.via = None, "", None, []
        r.docked_at = r.dock_station = ""
        r.obstacle_until = r.stopped_at = r.clear_at = None
        r.blocked = False
        r.has_basket = basket == "EMPTY"
        r.basket = {}


def load_map(name):
    if name == DEFAULT_MAP:
        return load_lane_map()
    path = Path(name) if name.endswith(".yaml") else MAPS / f"{name}.yaml"
    return load_lane_map(path)


def map_name(name) -> str:
    return Path(name).stem if name.endswith(".yaml") else name


def load_scenario(name) -> dict:
    path = Path(name) if name.endswith(".yaml") else SCENARIOS / f"{name}.yaml"
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def scenario_name(name) -> str:
    return Path(name).stem if name.endswith(".yaml") else name


def _faults(data) -> tuple[list, list]:
    world, operator = [], []
    for fault in data.get("faults") or []:
        if fault["type"] in WORLD_FAULTS:
            world.append(dict(fault))
        elif fault["type"] in OPERATOR_FAULTS:
            operator.append(dict(fault))
        else:
            raise ValueError(f"fault type {fault['type']!r} is not supported by the baseline")
    return world, operator


def build_world(data, map_name_, seed, mode) -> SimWorld:
    world_faults, _ = _faults(data)
    site = load_map(map_name_)
    parsed = parse_scenario(data, site)  # the fakes' checks and defaults
    if parsed.arm.fail_rate:
        raise ValueError("arm fail_rate is not supported by the baseline")
    stock: dict[str, int] = {}
    for items in parsed.stock.values():
        for kind, count in items.items():
            stock[kind] = stock.get(kind, 0) + count
    if {k: v for k, v in parsed.inventory.items() if v} != {k: v for k, v in stock.items() if v}:
        raise ValueError(f"inventory {parsed.inventory} differs from the summed stock {stock}; "
                         "SimWorld's central keeps no separate record")
    return SimWorld(seed=seed, orders=[dict(o) for o in data["orders"]], stock=stock,
                    robots={n: dict(spec) for n, spec in data["robots"].items()},
                    faults=world_faults, site=site,
                    params=replace(load_params(), lane_goal_mode=mode),
                    arm_s_per_item=parsed.arm.seconds_per_item,
                    swap_step_s=parsed.arm.swap_seconds / 2,  # take, then put
                    dock_s=parsed.dock.duration_s,
                    gaussian_noise=True)


def _blank_row(map_, scenario, mode, seed, robots, total) -> dict:
    row = {c: "" for c in COLUMNS}
    row.update(map=map_, scenario=scenario, mode=mode, seed=seed, finished=False,
               orders_total=total)
    row.update({f"wait_{r}_s": "" for r in robots})
    return row


def run_one(map_, scenario, seed, mode, timeout_s=None) -> dict:
    return run_with_world(map_, scenario, seed, mode, timeout_s)[0]


def run_with_world(map_, scenario, seed, mode, timeout_s=None) -> tuple[dict, SimWorld | None]:
    data = expand_random(load_scenario(scenario), seed)  # random 블록은 이 실행의 seed로
    row = _blank_row(map_name(map_), scenario_name(scenario), mode, seed, data["robots"],
                     len(data["orders"]))
    try:
        world = build_world(data, map_, seed, mode)
    except (ValueError, KeyError) as error:
        row["error"] = f"cannot build: {error!r}"
        return row, None
    _, actions = _faults(data)
    meter = Meter(list(world.robots), DT)
    operator = Operator(world, actions)
    limit = timeout_s or parse_scenario(data, world.site).timeout_s
    finish = None
    complete = False
    meter.observe(world)
    try:
        while world.t < limit - 1e-9:
            operator.act()
            world.step()
            meter.observe(world)
            if finish is None and world.all_orders_final():
                # SimWorld's central takes a report one step after the tick that sent it
                # (no orders, no report: nothing to wait for, so 0)
                finish = max((t for t, c in world.commands if isinstance(c, ReportOrderCmd)),
                             default=0.0)
            on_floor = [n for n, r in world.virtual.items() if not r.removed]
            # all_parked([]) would check every robot, removed ones too
            parked = not on_floor or world.all_parked(on_floor)
            if finish is not None and not operator.pending and parked:
                complete = True
                break
    except AssertionError as error:
        row["error"] = f"invariant at {world.t:.1f} s: {error}"
    row.update(
        finished=complete and not row["error"],
        orders_shipped=sum(r.status == SHIPPED for r in world.reports),
        finish_s=round(finish, 3) if finish is not None else "",
        end_s=round(world.t, 3),
        wait_total_s=round(sum(meter.wait.values()), 3),
        interventions=operator.accepted,
        operator_refused=operator.refused,
        error_entries=meter.error_entries,
        stops=meter.stops,
        detours=sum(e.code == "DETOUR" for e in world.events),
    )
    row.update({f"wait_{r}_s": round(s, 3) for r, s in meter.wait.items()})
    if any(f["type"] == "blocked" for f in data.get("faults") or []):
        row.update(object_still_s=round(meter.object_still, 3), object_hits=meter.object_hits)
    return row, world


def _columns(rows) -> list[str]:
    extra = sorted({k for row in rows for k in row} - set(COLUMNS))
    return [*COLUMNS, *extra]


def write_rows(path, rows) -> None:
    columns = _columns(rows)
    with Path(path).open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=columns, restval="")
        writer.writeheader()
        writer.writerows(rows)


def _number(value):
    if value in ("", None):
        return None
    if isinstance(value, bool) or value in ("True", "False"):
        return float(value in (True, "True"))
    return float(value)


def summarize(rows) -> list[dict]:
    """Mean, min and max per map, scenario and mode. `finished` is taken over every run, the
    other metrics over finished runs only: a run cut by the timeout keeps adding waits."""
    groups: dict[tuple, list] = {}
    for row in rows:
        groups.setdefault((row["map"], row["scenario"], row["mode"]), []).append(row)
    metrics = [c for c in _columns(rows) if c not in NOT_METRICS]
    out = []
    for (map_, scenario, mode), group in groups.items():
        finished = [r for r in group if _number(r["finished"])]
        for metric in metrics:
            source = group if metric == "finished" else finished
            values = [v for v in (_number(r.get(metric)) for r in source) if v is not None]
            out.append({"map": map_, "scenario": scenario, "mode": mode, "metric": metric,
                        "runs": len(group), "n": len(values),
                        "mean": sum(values) / len(values) if values else "",
                        "min": min(values) if values else "",
                        "max": max(values) if values else ""})
    return out


def _rounded(row) -> dict:
    return {k: round(v, 3) if isinstance(v, float) else v for k, v in row.items()}


def write_summary(path, summary) -> None:
    with Path(path).open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=SUMMARY_COLUMNS)
        writer.writeheader()
        writer.writerows(_rounded(s) for s in summary)


TABLE_METRICS = ("finished", "finish_s", "wait_total_s", "object_still_s", "interventions",
                 "error_entries", "stops", "detours")


def table(summary) -> str:
    """Markdown table: mean (min-max) over seeds; finished is the share of runs."""
    cells: dict[tuple, dict] = {}
    for s in summary:
        cells.setdefault((s["map"], s["scenario"], s["mode"]), {})[s["metric"]] = s
    lines = ["| map | scenario | mode | " + " | ".join(TABLE_METRICS) + " |",
             "|---" * (3 + len(TABLE_METRICS)) + "|"]
    for key, metrics in cells.items():
        shown = []
        for metric in TABLE_METRICS:
            s = metrics.get(metric)
            if s is None or s["n"] == 0:
                shown.append("")
            elif metric == "finished":
                shown.append(f"{round(s['mean'] * s['runs'])}/{s['runs']}")
            elif s["min"] == s["max"]:
                shown.append(f"{s['mean']:g}")
            else:
                shown.append(f"{s['mean']:.1f} ({s['min']:g}-{s['max']:g})")
        lines.append("| " + " | ".join([*key, *shown]) + " |")
    return "\n".join(lines)


def plan(maps, scenarios) -> list[tuple[str, str]]:
    pairs = []
    for map_ in maps:
        for scenario in scenarios or DEFAULT_PLAN.get(map_name(map_), []):
            named = load_scenario(scenario).get("map")
            if scenarios and named and named != map_name(map_):
                continue
            pairs.append((map_, scenario))
    return pairs


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0],
                                     epilog=__doc__.split("\n", 1)[1],
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--maps", nargs="+", default=list(DEFAULT_PLAN),
                        help="lane_map, a map in sim/fake_robots/maps, or a .yaml path")
    parser.add_argument("--scenarios", nargs="+",
                        help="scenario names or .yaml paths; default: the plan above")
    parser.add_argument("--modes", nargs="+", choices=MODES, default=list(MODES))
    parser.add_argument("--seeds", nargs="+", type=int, default=list(SEEDS))
    parser.add_argument("--out", type=Path, default=Path("baseline.csv"))
    parser.add_argument("--summary", type=Path, help="default: <out>_summary.csv")
    parser.add_argument("--timeout", type=float, help="fake seconds; default: the scenario's")
    args = parser.parse_args(argv)
    summary_path = args.summary or args.out.with_name(f"{args.out.stem}_summary.csv")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    started = time.monotonic()
    for map_, scenario in plan(args.maps, args.scenarios):
        for mode in args.modes:
            for seed in args.seeds:
                row = run_one(map_, scenario, seed, mode, args.timeout)
                rows.append(row)
                print(f"{row['map']} {row['scenario']} {mode} seed {seed}: "
                      f"finish {row['finish_s'] or '-'} {row['error']}", file=sys.stderr,
                      flush=True)
    write_rows(args.out, rows)
    summary = summarize(rows)
    write_summary(summary_path, summary)
    print(table(summary))
    print(f"{len(rows)} runs in {time.monotonic() - started:.0f} s wall; rows {args.out}, "
          f"summary {summary_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
