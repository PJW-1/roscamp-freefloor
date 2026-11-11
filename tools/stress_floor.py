#!/usr/bin/env python3
"""자유바닥 교착 찾기: 교착이 날 만한 상황을 많이 만들어 머리 없는 시뮬(FloorWorld)로 돌린다.

    python3 tools/stress_floor.py --map arena_v6 --kind swap --out swap.csv
    python3 tools/stress_floor.py --map arena_v6 --kind cross --cases 200 --seed 3 --out cross.csv
    python3 tools/stress_floor.py --map arena_v6 --kind ring --out ring.csv --set floor_keep_right=true
    python3 tools/stress_floor.py --map arena_v6 --kind swap --shard 1/3 --out swap1.csv   # 셋으로 나눠 돌릴 때

상황(kind):
  swap   두 로봇이 서로의 자리로 간다(마주 온다). 지점의 모든 쌍.
  cross  두 로봇이 아무 두 지점에서 아무 두 지점으로 간다(--cases개, --seed로 뽑는다).
  ring   세 로봇이 서로의 자리로 돌아가며 간다(a는 b 자리로, b는 c 자리로, c는 a 자리로).
세 번째 로봇은 swap과 cross에서 주차 자리에 서 있다. 지점은 레인 지도의 웨이포인트 중 빈 바닥(clear)에
있는 것이다. 도크 지점(고정 선으로만 들어간다)과 주차 자리(관제가 줄을 당겨 세운다)는 뺀다. 목표에 한 번
닿으면 닿은 것으로 친다: 그 뒤 관제가 로봇을 주차 자리로 보낸다.

한 판은 모두 목표에 닿거나 --limit 초가 지나면 끝난다. 줄마다 결과:
  ok        모두 닿았다
  stuck     시간 안에 못 닿았다(누가 못 닿았는지 left에 적는다)
  broken    칸 불변식이 깨졌다(FloorWorld의 검사: 몸이 쥔 칸 밖, 두 로봇이 닿음 등). error에 내용
사건 수(교착 감지, 풀지 못한 교착, 비켜 서기, 길 없음, 격리)와 가장 오래 선 시간, 최소 간격도 적는다.
"""
from __future__ import annotations

import argparse
import collections
import csv
import itertools
import math
import random
import sys
from dataclasses import replace
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
for part in ("pc/fleet_core/test", "pc/fleet_core", "sim/fake_robots"):
    sys.path.insert(0, str(REPO / part))

from fleet_core.common.params import load_params  # noqa: E402
from fleet_core.traffic.floor.floor_map import floor_map_for  # noqa: E402
from fleet_core.traffic.lane_map import load_lane_map  # noqa: E402
from floor_world import FloorWorld  # noqa: E402

LOCKS = {"floor_lock_ahead_m": 0.15, "floor_extend_step_m": 0.05, "release_distance_m": 0.17}
ROBOTS = ("pinky1", "pinky2", "pinky3")
COUNTED = ("DEADLOCK_DETECTED", "DEADLOCK_UNRESOLVED", "FLOOR_SIDESTEP", "FLOOR_NO_PATH",
           "ROBOT_ISOLATED", "FLOOR_KEEP_RIGHT", "FLOOR_NARROW_WAIT", "FLOOR_OFF_CELLS")
FIELDS = ("kind", "case", "seed", "noise", "result", "t_s", "left", "longest_stop_s", "min_gap_m",
          *COUNTED, "error")


class Bench:
    def __init__(self, map_name: str, overrides: dict, noise: float):
        self.lanes = load_lane_map(REPO / "sim" / "fake_robots" / "maps" / f"{map_name}.yaml")
        self.floor = floor_map_for(self.lanes)
        self.params = replace(load_params(), traffic_mode="freefloor", **LOCKS, **overrides)
        self.noise = noise
        docks = {station.dock for station in self.lanes.stations.values()}
        self.parking = list(self.lanes.parking)
        # 주차 자리도 뺀다: 주차 줄은 관제가 따로 당겨 세워서, 닿은 로봇을 다른 자리로 옮긴다
        self.points = [w for w in sorted(self.lanes._pose, key=str)
                       if w not in docks and w not in self.parking
                       and self.floor.cell_of(*self.lanes.waypoint_pose(w)) in self.floor.clear]

    def pose(self, waypoint):
        return self.lanes.waypoint_pose(waypoint)

    def run(self, starts: dict, goals: dict, seed: int, limit_s: float) -> dict:
        """starts {로봇: 지점}, goals {로봇: 지점}. 목표가 없는 로봇은 제자리에 서 있다."""
        # 목표가 없는 로봇은 제 주차 자리에 선다(가장 먼 자리부터: 지도의 주차 목록 뒤쪽)
        free = list(reversed(self.parking))
        robots = {}
        for n in ROBOTS:
            start = starts[n] if n in starts else free.pop(0)
            robots[n] = {"start": start, "speed_mps": 0.15, "pose_noise_m": self.noise}
        row = {k: 0 for k in COUNTED}
        row.update(result="ok", left="", error="", longest_stop_s=0.0, min_gap_m="")
        world = None
        try:
            world = FloorWorld(seed=seed, orders=[], robots=robots, site=self.lanes,
                               floor=self.floor, params=self.params)
            traffic = world.core.traffic
            traffic._spot_taken = lambda m: False  # 아무 지점으로나 보낸다(주차 자리 규칙 없이)
            world._report_states()
            traffic._plan_budget = 99
            for name, waypoint in goals.items():
                traffic._start_leg(traffic._missions[name], "TO_PARK", None, waypoint)
            target = {n: self.pose(w) for n, w in goals.items()}
            reached = {n: False for n in goals}
            still = collections.Counter()
            last = {n: (world.virtual[n].x, world.virtual[n].y) for n in goals}
            while world.t < limit_s and not all(reached.values()):
                world.step()
                for n in goals:
                    r = world.virtual[n]
                    here = (r.x, r.y)
                    if math.dist(here, target[n]) <= 0.05:
                        reached[n] = True
                    if math.dist(here, last[n]) > 1e-4:
                        still[n] = 0.0
                    elif not reached[n]:
                        still[n] += 0.1
                        row["longest_stop_s"] = max(row["longest_stop_s"], round(still[n], 1))
                    last[n] = here
            left = sorted(n for n, ok in reached.items() if not ok)
            if left:
                row["result"], row["left"] = "stuck", " ".join(left)
        except AssertionError as error:
            row["result"], row["error"] = "broken", str(error).splitlines()[0][:200]
        except Exception as error:  # noqa: BLE001  (한 판의 오류로 전체를 멈추지 않는다)
            row["result"], row["error"] = "broken", f"{type(error).__name__}: {error}"[:200]
        if world is not None:
            counts = collections.Counter(e.code for e in world.events)
            for code in COUNTED:
                row[code] = counts.get(code, 0)
            row["t_s"] = round(world.t, 1)
            if math.isfinite(world.min_gap):
                row["min_gap_m"] = round(world.min_gap, 3)
        return row


def cases(bench: Bench, kind: str, count: int, seed: int):
    """(이름, starts, goals)를 낸다."""
    pts = bench.points
    if kind == "swap":
        for a, b in itertools.combinations(pts, 2):
            if math.dist(bench.pose(a), bench.pose(b)) < 0.25:
                continue  # 처음부터 몸이 닿는다
            yield f"{a}<>{b}", {"pinky1": a, "pinky2": b}, {"pinky1": b, "pinky2": a}
    elif kind == "cross":
        rng = random.Random(seed)
        made = 0
        while made < count:
            a, b, c, d = rng.sample(pts, 4)
            if math.dist(bench.pose(a), bench.pose(b)) < 0.25 or \
                    math.dist(bench.pose(c), bench.pose(d)) < 0.25:
                continue
            made += 1
            yield f"{a}>{c}+{b}>{d}", {"pinky1": a, "pinky2": b}, {"pinky1": c, "pinky2": d}
    elif kind == "ring":
        rng = random.Random(seed)
        triples = [t for t in itertools.combinations(pts, 3)
                   if all(math.dist(bench.pose(p), bench.pose(q)) >= 0.25
                          for p, q in itertools.combinations(t, 2))]
        rng.shuffle(triples)
        for a, b, c in triples[:count]:
            yield (f"{a}>{b}>{c}>{a}", {"pinky1": a, "pinky2": b, "pinky3": c},
                   {"pinky1": b, "pinky2": c, "pinky3": a})
    else:
        raise ValueError(f"unknown kind {kind!r}")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("--map", default="arena_v6")
    parser.add_argument("--kind", choices=("swap", "cross", "ring"), default="swap")
    parser.add_argument("--cases", type=int, default=120, help="cross, ring: 몇 판")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--noise", type=float, default=0.03, help="위치 보고 잡음 표준편차(m)")
    parser.add_argument("--limit", type=float, default=90.0, help="한 판의 시간 한도(s)")
    parser.add_argument("--shard", default="1/1", help="i/n: 판을 n으로 나눈 i번째만")
    parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                        help="관제 파라미터를 바꾼다(여러 번)")
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    overrides = {}
    for item in args.set:
        key, _, value = item.partition("=")
        overrides[key] = yaml.safe_load(value)
    index, of = (int(v) for v in args.shard.split("/"))
    bench = Bench(args.map, overrides, args.noise)
    tally = collections.Counter()
    with open(args.out, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        for k, (name, starts, goals) in enumerate(cases(bench, args.kind, args.cases, args.seed)):
            if k % of != index - 1:
                continue
            row = bench.run(starts, goals, args.seed, args.limit)
            row.update(kind=args.kind, case=name, seed=args.seed, noise=args.noise)
            writer.writerow({f: row.get(f, "") for f in FIELDS})
            handle.flush()
            tally[row["result"]] += 1
    print(f"{args.map} {args.kind} {dict(overrides)} shard {args.shard}: {dict(tally)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
