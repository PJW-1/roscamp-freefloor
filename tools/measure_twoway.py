#!/usr/bin/env python3
"""arena_v4(한 방향 선) 대 arena_v4_twoway(양방향 통로 + 방향 표시) 머리 없는 측정.

tools/compare_floor.py의 가짜 세계(FloorWorld)로 같은 주문, 씨앗, 파라미터를 두 지도에서 돌리고 실행마다
CSV 한 줄을 쓴다. 파라미터는 test_floor_arena_v4.py의 LOCKS(잠금 값)에 floor_shortcut 끔(기본값)이다.

    python3 tools/measure_twoway.py --out docs/freefloor/arena_v4_twoway/runs.csv --jobs 4
    python3 tools/measure_twoway.py --summary docs/freefloor/arena_v4_twoway/runs.csv

구성(지시서 2026-11-11 4단계):
    standard  2대 주문 10건(10 s마다) 씨앗 6개, 3대 6건(20 s마다) 씨앗 12개, 3대 16건(12 s마다) 씨앗 6개
    asym      3대 주문 8건, 주기 10 s와 30 s, 물건/교체 4/8, 4/30, 8/4 s, 씨앗 3개씩
    obstacle  3대 6건(20 s마다), 20 s에 나타나 120 s 있는 물체(반지름 0.04 m): 가운데 통로 (0.97, 0.85),
              물체 3 북쪽 통로 안 (0.78, 0.905). 씨앗 3개씩
더한 열:
    group, config     구성 이름
    pass_<통로>_fwd / _back  로봇 실제 자리로 센 통로 통과 수(양방향 지도의 통로 가운데 선 기준, 두 지도
                      모두 같은 선으로 센다). fwd는 지도에 적은 점 순서 방향
    mark_waits        방향 표시를 못 잡아 입구에서 선 횟수(양방향 지도만)
    mark_switches     방향 표시를 기다리다 다른 통로로 바꾼 횟수(양방향 지도만)
"""
from __future__ import annotations

import argparse
import csv
import math
import multiprocessing
import statistics
import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
for sub in (("tools",), ("pc", "fleet_core", "test"), ("pc", "fleet_core"), ("sim", "fake_robots")):
    sys.path.insert(0, str(REPO.joinpath(*sub)))

import baseline  # noqa: E402
import compare_floor  # noqa: E402

LOCKS = {"floor_lock_ahead_m": 0.15, "floor_extend_step_m": 0.05, "release_distance_m": 0.17,
         "floor_shortcut": False}
KINDS = [{"A": 1, "C": 1}, {"B": 1}, {"D": 1, "A": 1}, {"C": 1}, {"B": 1, "D": 1}, {"A": 1}]
ROBOTS = (("pinky1", "P-3"), ("pinky2", "P-2"), ("pinky3", "P-1"))
PASSAGES_YAML = (REPO / "pc" / "fleet_core" / "fleet_core" / "data"
                 / "floor_map_arena_v4_twoway.yaml")
TIMEOUT_S = 900
INSIDE_M = 0.07  # 가운데 선에서 이만큼 안이면 통로 안
END_M = 0.02  # 양 끝에서 이만큼 안쪽이어야 통로 안


def scenario(robots, count, period_s, item_s=4.0, swap_s=8.0, faults=()):
    return {
        "map": "arena_v4", "seed": 1, "speed_factor": 5.0, "timeout_s": TIMEOUT_S,
        "stock": {"arm1": {"A": 60, "B": 60}, "arm2": {"C": 60, "D": 60}},
        "orders": [{"at_s": period_s * k, "id": f"R-{k + 1:03d}", "items": KINDS[k % 6]}
                   for k in range(count)],
        "robots": {name: {"start": spot, "speed_mps": 0.15, "pose_noise_m": 0.03}
                   for name, spot in ROBOTS[:robots]},
        "arm": {"seconds_per_item": item_s, "swap_seconds": swap_s, "fail_rate": 0.0},
        "dock": {"duration_s": 3.0, "error_cm": 1.0}, "faults": list(faults),
        "expect": {"orders_shipped": count},
    }


def obstacle(x, y):
    return {"type": "object", "at_s": 20.0, "duration_s": 120.0, "x": x, "y": y,
            "radius_m": 0.04}


def configs(groups) -> list[tuple[str, str, dict, int]]:
    """(group, config, scenario, seed)."""
    out = []
    if "standard" in groups:
        for name, n, count, period, seeds in (("2x10", 2, 10, 10.0, 6), ("3x6", 3, 6, 20.0, 12),
                                              ("3x16", 3, 16, 12.0, 6)):
            out += [("standard", name, scenario(n, count, period), s) for s in range(1, seeds + 1)]
    if "asym" in groups:
        for period in (10.0, 30.0):
            for item, swap in ((4.0, 8.0), (4.0, 30.0), (8.0, 4.0)):
                name = f"p{period:g}_i{item:g}_s{swap:g}"
                out += [("asym", name, scenario(3, 8, period, item, swap), s) for s in (1, 2, 3)]
    if "obstacle" in groups:
        for name, xy in (("mid_0.97_0.85", (0.97, 0.85)), ("north3_0.78_0.905", (0.78, 0.905))):
            out += [("obstacle", name, scenario(3, 6, 20.0, faults=[obstacle(*xy)]), s)
                    for s in (1, 2, 3)]
    return out


def load_passages(path=PASSAGES_YAML) -> dict:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    return {name: [tuple(map(float, p)) for p in raw["points"]]
            for name, raw in data["passages"].items()}


def _along(points, xy):
    """(선에서 거리, 선 위 거리, 선 길이)."""
    best, base = (math.inf, 0.0), 0.0
    for a, b in zip(points, points[1:], strict=False):
        seg = math.dist(a, b)
        t = max(0.0, min(1.0, ((xy[0] - a[0]) * (b[0] - a[0]) + (xy[1] - a[1]) * (b[1] - a[1]))
                         / seg ** 2))
        q = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
        best = min(best, (math.dist(q, xy), base + t * seg))
        base += seg
    return best[0], best[1], base


class Tracker:
    """로봇 실제 자리로 통로 통과를 센다: 안에 들어온 자리와 나간 자리가 선 길이 절반 넘게 떨어지면 한 번."""

    def __init__(self, passages):
        self.passages = passages
        self.entry: dict = {}  # (로봇, 통로) -> 들어온 선 위 거리
        self.last: dict = {}
        self.count = {(name, way): 0 for name in passages for way in ("fwd", "back")}

    def observe(self, world):
        for robot in world.virtual.values():
            if robot.removed:
                continue
            for name, pts in self.passages.items():
                off, at, length = _along(pts, (robot.x, robot.y))
                inside = off <= INSIDE_M and END_M < at < length - END_M
                key = (robot.name, name)
                if inside and key not in self.entry:
                    self.entry[key] = at
                elif not inside and key in self.entry:
                    went = self.last[key] - self.entry.pop(key)
                    if abs(went) >= length / 2:
                        self.count[(name, "fwd" if went > 0 else "back")] += 1
                if inside:
                    self.last[key] = at


def one(job):
    group, config, data, seed, map_, overrides = job
    compare_floor.OVERRIDES = dict(overrides)
    tracker = Tracker(load_passages())
    original = compare_floor.build

    def build(*args, **kwargs):
        world = original(*args, **kwargs)
        step = world.step

        def stepped():
            step()
            tracker.observe(world)

        world.step = stepped
        return world

    compare_floor.build = build
    baseline.load_scenario = lambda name: data
    try:
        row, world = compare_floor.run(map_, f"{group}_{config}", seed, "freefloor",
                                       timeout_s=TIMEOUT_S)
    finally:
        compare_floor.build = original
    row["group"], row["config"] = group, config
    for (name, way), n in sorted(tracker.count.items()):
        row[f"pass_{name}_{way}"] = n
    metrics = getattr(world.core.traffic, "metrics", {}) if world is not None else {}
    row["mark_waits"] = metrics.get("mark_waits", "")
    row["mark_switches"] = metrics.get("mark_switches", "")
    for key in ("wait_pinky1_s", "wait_pinky2_s", "wait_pinky3_s"):
        row.setdefault(key, "")
    return row


def run_all(maps, groups, jobs, overrides) -> list[dict]:
    work = [(g, c, d, s, m, overrides) for m in maps for g, c, d, s in configs(groups)]
    if jobs <= 1:
        return [one(job) for job in work]
    with multiprocessing.get_context("fork").Pool(jobs) as pool:
        return pool.map(one, work, chunksize=1)


FIRST = ["map", "group", "config", "seed", "finished", "error", "orders_total", "orders_shipped",
         "finish_s", "end_s"]


def write(rows, out: Path) -> None:
    keys = FIRST + sorted({k for row in rows for k in row} - set(FIRST))
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for row in rows:
            w.writerow(row)


def _num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def summary(path: Path) -> str:
    """두 지도 비교 표(마크다운). 끝난 시각 평균은 두 지도 모두 끝낸 판만."""
    rows = list(csv.DictReader(path.open(encoding="utf-8")))
    maps = sorted({r["map"] for r in rows})
    by = {}
    for r in rows:
        by.setdefault((r["group"], r["config"]), {}).setdefault(r["map"], {})[r["seed"]] = r
    lines = ["| 구성 | 지도 | 판 | 완료 | 둘 다 완료 평균 끝(s) | 대기 순환 | 못 풂 | 격리 | 칸 위반 | 최소 간격(m) |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for (group, config), per in sorted(by.items()):
        seeds = sorted(set.intersection(*(set(per.get(m, {})) for m in maps)), key=int)
        both = [s for s in seeds if all(per[m][s]["finished"] == "True" for m in maps)]
        for m in maps:
            runs = [per[m][s] for s in seeds]
            done = sum(r["finished"] == "True" for r in runs)
            ends = [_num(per[m][s]["finish_s"]) for s in both]
            mean = f"{statistics.mean(ends):.1f}" if ends else "-"
            gaps = [_num(r["min_gap_m"]) for r in runs if _num(r["min_gap_m"]) is not None]
            gap = f"{min(gaps):.3f}" if gaps else "-"
            lines.append(
                f"| {group} {config} | {m} | {len(runs)} | {done} | {mean} (n={len(both)}) | "
                f"{sum(int(_num(r['deadlocks']) or 0) for r in runs)} | "
                f"{sum(int(_num(r['unresolved']) or 0) for r in runs)} | "
                f"{sum(int(_num(r['isolations']) or 0) for r in runs)} | "
                f"{sum(int(_num(r['violations']) or 0) for r in runs)} | {gap} |")
    passes = sorted(k for k in rows[0] if k.startswith("pass_"))
    lines += ["", "| 지도 | " + " | ".join(p[5:] for p in passes) + " |",
              "|---|" + "---|" * len(passes)]
    for m in maps:
        total = [sum(int(_num(r[p]) or 0) for r in rows if r["map"] == m) for p in passes]
        lines.append(f"| {m} | " + " | ".join(str(t) for t in total) + " |")
    return "\n".join(lines)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("--maps", nargs="+", default=["arena_v4", "arena_v4_twoway"])
    parser.add_argument("--groups", nargs="+", default=["standard", "asym", "obstacle"])
    parser.add_argument("--jobs", type=int, default=1)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")
    parser.add_argument("--summary", type=Path)
    args = parser.parse_args(argv)
    if args.summary:
        print(summary(args.summary))
        return 0
    overrides = dict(LOCKS)
    for item in args.set:
        key, value = item.split("=", 1)
        overrides[key] = yaml.safe_load(value)
    rows = run_all(args.maps, args.groups, args.jobs, overrides)
    rows.sort(key=lambda r: (r["map"], r["group"], r["config"], int(r["seed"])))
    write(rows, args.out)
    print(f"{len(rows)} runs -> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
