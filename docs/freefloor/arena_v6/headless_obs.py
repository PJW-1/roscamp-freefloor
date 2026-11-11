"""v5 후보를 헤드리스로 돌리고 궤적을 그린다. 사용: headless.py <repo> <map> <out.png> [seed ...]"""
import sys, math, json, tempfile, collections
from pathlib import Path
repo = Path(sys.argv[1]); name = sys.argv[2]; out = sys.argv[3]
seeds = [int(s) for s in sys.argv[4:]] or [1]
for p in ("tools", "pc/fleet_core/test", "pc/fleet_core", "sim/fake_robots"):
    sys.path.insert(0, str(repo / p))
import yaml
import compare_floor
from fleet_core.traffic.lane_map import load_lane_map
from fleet_core.traffic.floor.floor_map import floor_map_for

lanes = load_lane_map(repo / "sim/fake_robots/maps" / f"{name}.yaml")
floor = floor_map_for(lanes)
clear = floor.clear if hasattr(floor, "clear") else None
cells = sorted(clear)
imax = max(i for i, j in cells); jmax = max(j for i, j in cells)
print(f"clear cells: {len(cells)}")
for j in range(jmax + 1, -1, -1):
    print("".join("#" if (i, j) in clear else "." for i in range(imax + 2)))
import os
OBJ = [(30, 55, 0.79, 0.25), (60, 55, 1.24, 0.405), (105, 55, 1.29, 0.864), (150, 50, 1.24, 0.405), (185, 50, 0.79, 0.25)]
FAULTS = [] if os.environ.get("NO_OBJ") else [{"type": "object", "at_s": a, "duration_s": d, "x": x, "y": y, "radius_m": 0.085} for a, d, x, y in OBJ]
import floor_world
DOCKS = []
_old = floor_world.FloorWorld._start_dock
def _rec(self, cmd):
    r = self.virtual[cmd.robot]
    dock = self.floor.docks[cmd.dock_id]
    DOCKS.append((round(self.t, 1), cmd.robot, cmd.dock_id, round(math.degrees(r.heading)), round(math.dist((r.x, r.y), dock), 2), r.goal_kind))
    return _old(self, cmd)
floor_world.FloorWorld._start_dock = _rec
KINDS = [{"A": 1, "C": 1}, {"B": 1}, {"D": 1, "A": 1}, {"C": 1}, {"B": 1, "D": 1}, {"A": 1}]
def scenario(count=8, period=20.0, noise=0.03):
    return {"map": name, "seed": 1, "speed_factor": 5.0, "timeout_s": 900,
            "stock": {"arm1": {"A": 40, "B": 40}, "arm2": {"C": 40, "D": 40}},
            "orders": [{"at_s": period * k, "id": f"R-{k+1:03d}", "items": KINDS[k % 6]} for k in range(count)],
            "robots": {n: {"start": s, "speed_mps": 0.15, "pose_noise_m": noise}
                       for n, s in (("pinky1", "P-3"), ("pinky2", "P-2"), ("pinky3", "P-1"))},
            "arm": {"seconds_per_item": 4.0, "swap_seconds": 8.0, "fail_rate": 0.0},
            "dock": {"duration_s": 3.0, "error_cm": 1.0}, "faults": FAULTS, "expect": {"orders_shipped": count}}
compare_floor.OVERRIDES = {"floor_lock_ahead_m": 0.15, "floor_extend_step_m": 0.05, "release_distance_m": 0.17}
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
fig, axes = plt.subplots(len(seeds), 1, figsize=(10, 5.2 * len(seeds)), squeeze=False)
for ax, seed in zip(axes[:, 0], seeds):
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "run.yaml"; p.write_text(yaml.safe_dump(scenario()))
        row, world = compare_floor.run(name, str(p), seed, "freefloor", frame_every=2)
    keys = ("finished", "orders_shipped", "finish_s", "end_s", "wait_total_s", "min_gap_m", "path_m",
            "replans", "sidesteps", "deadlocks", "unresolved", "isolations", "violations", "stops", "error")
    print(seed, {k: row[k] for k in keys})
    codes = collections.Counter(e.code for e in world.events)
    print("   events:", dict(codes))
    bad = [d for d in DOCKS if abs(math.remainder(math.radians(d[3] - (90 if d[2] == "WORK" else -90)), math.tau)) > math.radians(30)]
    print("   docks:", len(DOCKS), "wrong heading:", bad)
    import collections as _c
    seen = _c.Counter()
    prev = {}
    for d in DOCKS:
        if d[1] in prev and prev[d[1]][2] == d[2]:
            print("     again:", prev[d[1]], "->", d)
        if d[4] > 0.15:
            print("     far:", d)
        prev[d[1]] = d
    for e in world.events:
        if e.code.startswith(("FLOOR_", "GOAL_", "DEADLOCK", "ROBOT_")):
            print("    ", [getattr(e, k, "") for k in ("t", "time_s", "source", "robot")], e.code, str(getattr(e, "detail", ""))[:90])
    DOCKS.clear()
    for a, d, x, y in ([] if os.environ.get("NO_OBJ") else OBJ):
        ax.add_patch(plt.Circle((x, y), 0.085, color="tab:orange", alpha=0.6)); ax.text(x, y, f"{a}-{a+d}s", fontsize=7, ha="center")
    for b in yaml.safe_load((repo / "sim/fleet_gz_sim/sites" / f"{name}.yaml").read_text())["boxes"]:
        ax.add_patch(plt.Rectangle((b["x0"], b["y0"]), b["x1"] - b["x0"], b["y1"] - b["y0"], color="#b9a27c"))
    tracks = collections.defaultdict(list)
    for f in world.frames:
        for r, (x, y) in f["robots"].items():
            tracks[r].append((x, y))
    for r, c in zip(sorted(tracks), ("tab:blue", "tab:green", "tab:red")):
        xs, ys = zip(*tracks[r]); ax.plot(xs, ys, color=c, lw=0.8, alpha=0.8, label=r)
    ax.set_xlim(0, 1.98); ax.set_ylim(0, 0.99); ax.set_aspect("equal"); ax.legend(loc="upper center", ncol=3)
    ax.set_title(f"{name} seed {seed}: finished {row['finished']} in {row['end_s']} s, min gap {row['min_gap_m']} m")
fig.savefig(out, dpi=110, bbox_inches="tight")
