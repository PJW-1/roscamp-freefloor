#!/usr/bin/env python3
"""Lane vs free-floor comparison (free-floor experiment P3/P6): run the same scenarios, seeds and
map coordinates with traffic_mode lanes and freefloor on the headless SimWorld fake and write one
CSV row per run plus a summary over seeds.

    python3 tools/compare_floor.py --out docs/freefloor/compare.csv
    python3 tools/compare_floor.py --scenarios basic_orders --seeds 1 2 --modes freefloor

Runs reuse tools/baseline.py (scenario parsing, Meter, columns). lanes runs are exactly baseline
runs (SimWorld, lane_goal_mode path). freefloor runs use FloorWorld (pc/fleet_core/test/
floor_world.py): the same fake robots, central and arms, the FloorController, and cell
invariants instead of lane checks. Extra columns, filled for both modes:
    min_gap_m       smallest true distance between two robots on the floor over the run
    path_m          true distance driven by all robots
    tick_ms_mean    FleetManager.tick wall time (ms), mean and max over the run. Wall time on
    tick_ms_max     this machine, so only the two modes run back to back compare
    tick_ms_p99     99th percentile of the same (single slow ticks are often the OS or the GC)
    slowdowns       freefloor: times the locked part ahead of a driving robot got short (the
                    real robot slows down there; the fake one does not), lanes: empty
    replans         freefloor: routes replaced because the lock could not grow
    sidesteps       freefloor: robots sent aside to break a wait cycle
    deadlocks       DEADLOCK_DETECTED events; unresolved: DEADLOCK_UNRESOLVED events
    isolations      ROBOT_ISOLATED events
    violations      1 when an invariant broke (the run stops there), else 0
    placement_outside  freefloor: robots whose true body stuck out of their cells before their
                    first MoveTo (placed from a report off by more than the cell margin)
    line_idle_s     freefloor: 도크 접근선이 비어 있는데 그 선으로 가려는 로봇이 있던 시간(선 합)
    pre_admits      freefloor: 선이 비기 전에 차례를 준 횟수(floor_dock_schedule). 출발 성공 수는 아니다

    --set KEY=VALUE  freefloor 실행의 파라미터를 바꾼다(여러 번 가능). 예: --set floor_dock_schedule=true
"""
from __future__ import annotations

import argparse
import itertools
import math
import sys
import time
from dataclasses import replace
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO / "pc" / "fleet_core" / "test"))

import baseline  # noqa: E402
from fake_robots.scenario import expand_random, parse_scenario  # noqa: E402
from fleet_core.common.commands import SHIPPED, ReportOrderCmd  # noqa: E402
from fleet_core.common.params import load_params  # noqa: E402
from fleet_core.traffic.floor.floor_map import floor_map_for  # noqa: E402
from floor_world import FloorWorld  # noqa: E402

MODES = ("lanes", "freefloor")
OVERRIDES: dict = {}  # --set
EXTRA = ["line_idle_s", "pre_admits", "min_gap_m", "path_m", "tick_ms_mean", "tick_ms_max", "tick_ms_p99", "slowdowns",
         "replans", "sidesteps", "deadlocks", "unresolved", "isolations", "violations",
         "placement_outside"]


def build(data, map_, seed, traffic_mode):
    if traffic_mode == "lanes":
        return baseline.build_world(data, map_, seed, "path")
    world_faults, _ = baseline._faults(data)
    site = baseline.load_map(map_)
    floor = floor_map_for(site)  # the lane map names its floor map (FloorMapError if it fits none)
    parsed = parse_scenario(data, site)
    stock: dict[str, int] = {}
    for items in parsed.stock.values():
        for kind, count in items.items():
            stock[kind] = stock.get(kind, 0) + count
    return FloorWorld(seed=seed, orders=[dict(o) for o in data["orders"]], stock=stock,
                      robots={n: dict(spec) for n, spec in data["robots"].items()},
                      faults=world_faults, site=site,
                      params=replace(load_params(), lane_goal_mode="path", **OVERRIDES),
                      arm_s_per_item=parsed.arm.seconds_per_item,
                      swap_step_s=parsed.arm.swap_seconds / 2, dock_s=parsed.dock.duration_s,
                      gaussian_noise=True,
                      floor=floor)


def _line_idle(traffic) -> int:
    """접근선 중 비어 있는데 그 선으로 가려는 로봇이 있는 선의 수."""
    n = 0
    for line, ids in traffic._line_ids.items():
        if not line.endswith("_in"):
            continue
        if any(traffic.zones.holder(c) is not None for c in ids):
            continue  # 로봇이나 막힘 표시가 쥠
        if any(leg.in_line == line and traffic._in_service(r) for r, leg in traffic._legs.items()):
            n += 1
    return n


def _timed(world, store):
    tick = world.core.tick

    def timed():
        start = time.perf_counter()
        out = tick()
        store.append(time.perf_counter() - start)
        return out

    world.core.tick = timed


def run(map_, scenario, seed, traffic_mode, timeout_s=None, frame_every=0):
    """One run. Returns (row, world); world is None when the scenario could not be built.

    random 블록(주문 시각, 배터리 부족)은 이 실행의 seed로 푼다: seed마다 다른 시간표."""
    data = expand_random(baseline.load_scenario(scenario), seed)
    row = baseline._blank_row(baseline.map_name(map_), baseline.scenario_name(scenario),
                              traffic_mode, seed, data["robots"], len(data["orders"]))
    row.update({c: "" for c in EXTRA})
    try:
        world = build(data, map_, seed, traffic_mode)
    except (ValueError, KeyError) as error:
        row["error"] = f"cannot build: {error!r}"
        return row, None
    if frame_every and isinstance(world, FloorWorld):
        world.frame_every = frame_every
    ticks: list[float] = []
    _timed(world, ticks)
    _, actions = baseline._faults(data)
    meter = baseline.Meter(list(world.robots), baseline.DT)
    operator = baseline.Operator(world, actions)
    limit = timeout_s or parse_scenario(data, world.site).timeout_s
    finish, complete = None, False
    gap, driven, line_idle = math.inf, 0.0, 0.0
    last = {n: (r.x, r.y) for n, r in world.virtual.items()}
    meter.observe(world)
    try:
        while world.t < limit - 1e-9:
            operator.act()
            world.step()
            meter.observe(world)
            present = [r for r in world.virtual.values() if not r.removed]
            for a, b in itertools.combinations(present, 2):
                gap = min(gap, math.hypot(a.x - b.x, a.y - b.y))
            for r in present:
                driven += math.dist(last.get(r.name, (r.x, r.y)), (r.x, r.y))
                last[r.name] = (r.x, r.y)
            if isinstance(world, FloorWorld):
                line_idle += _line_idle(world.core.traffic) * baseline.DT
            if finish is None and world.all_orders_final():
                finish = max((t for t, c in world.commands if isinstance(c, ReportOrderCmd)),
                             default=0.0)
            on_floor = [n for n, r in world.virtual.items() if not r.removed]
            parked = not on_floor or world.all_parked(on_floor)
            if finish is not None and not operator.pending and parked:
                complete = True
                break
    except AssertionError as error:
        row["error"] = f"invariant at {world.t:.1f} s: {error}"
    codes = [e.code for e in world.events]
    metrics = getattr(world.core.traffic, "metrics", None)
    row.update(
        finished=complete and not row["error"],
        orders_shipped=sum(r.status == SHIPPED for r in world.reports),
        finish_s=round(finish, 3) if finish is not None else "",
        end_s=round(world.t, 3),
        wait_total_s=round(sum(meter.wait.values()), 3),
        interventions=operator.accepted, operator_refused=operator.refused,
        error_entries=meter.error_entries, stops=meter.stops, detours=codes.count("DETOUR"),
        min_gap_m=round(gap, 4) if gap < math.inf else "",
        path_m=round(driven, 3),
        tick_ms_mean=round(1000 * sum(ticks) / len(ticks), 3) if ticks else "",
        tick_ms_max=round(1000 * max(ticks), 3) if ticks else "",
        tick_ms_p99=round(1000 * sorted(ticks)[int(0.99 * (len(ticks) - 1))], 3) if ticks else "",
        line_idle_s=round(line_idle, 3) if metrics else "",
        pre_admits=metrics.get("pre_admits", 0) if metrics else "",
        slowdowns=metrics["slowdowns"] if metrics else "",
        replans=metrics["replans"] if metrics else "",
        sidesteps=metrics["sidesteps"] if metrics else "",
        deadlocks=codes.count("DEADLOCK_DETECTED"),
        unresolved=codes.count("DEADLOCK_UNRESOLVED"),
        isolations=codes.count("ROBOT_ISOLATED"),
        violations=1 if row["error"].startswith("invariant") else 0,
        placement_outside=len(getattr(world, "placement_outside", ())) if metrics else "",
    )
    row.update({f"wait_{r}_s": round(s, 3) for r, s in meter.wait.items()})
    if any(f["type"] == "blocked" for f in data.get("faults") or []):
        row.update(object_still_s=round(meter.object_still, 3), object_hits=meter.object_hits)
    return row, world


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0],
                                     epilog=__doc__.split("\n", 1)[1],
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--maps", nargs="+", default=["lane_map"])
    parser.add_argument("--scenarios", nargs="+",
                        default=["basic_orders", "order_burst", "blocked_lane"])
    parser.add_argument("--modes", nargs="+", choices=MODES, default=list(MODES))
    parser.add_argument("--seeds", nargs="+", type=int, default=list(range(1, 11)))
    parser.add_argument("--out", type=Path, default=Path("compare.csv"))
    parser.add_argument("--timeout", type=float)
    parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")
    args = parser.parse_args(argv)
    for item in args.set:
        key, _, value = item.partition("=")
        OVERRIDES[key] = yaml.safe_load(value)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    started = time.monotonic()
    for map_ in args.maps:
        for scenario in args.scenarios:
            for mode in args.modes:
                for seed in args.seeds:
                    row, _ = run(map_, scenario, seed, mode, args.timeout)
                    rows.append(row)
                    print(f"{row['map']} {row['scenario']} {mode} seed {seed}: "
                          f"finish {row['finish_s'] or '-'} {row['error']}", file=sys.stderr,
                          flush=True)
    baseline.write_rows(args.out, rows)
    summary = baseline.summarize(rows)
    baseline.write_summary(args.out.with_name(f"{args.out.stem}_summary.csv"), summary)
    print(baseline.table(summary))
    print(f"{len(rows)} runs in {time.monotonic() - started:.0f} s wall; rows {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
