"""잠금 넓이를 잰다. 사용: lockarea.py <repo> <map> <release_m> <seed> [noise]"""
import sys, math, tempfile, collections
from pathlib import Path
repo = Path(sys.argv[1]); name = sys.argv[2]; release = float(sys.argv[3]); seed = int(sys.argv[4])
noise = float(sys.argv[5]) if len(sys.argv) > 5 else 0.03
for p in ("tools", "pc/fleet_core/test", "pc/fleet_core", "sim/fake_robots"):
    sys.path.insert(0, str(repo / p))
import yaml
import compare_floor
import floor_world
_oldf = floor_world.FloorWorld.frame
def _frame(self):
    f = _oldf(self)
    tr = self.core.traffic
    f["mode"] = {}
    for n, m in tr._missions.items():
        leg = tr._legs.get(n)
        f["mode"][n] = "disc" if tr._reach_now(m) >= tr._body else "band"
    return f
floor_world.FloorWorld.frame = _frame
from fleet_core.traffic.lane_map import load_lane_map
from fleet_core.traffic.floor.floor_map import floor_map_for
floor = floor_map_for(load_lane_map(repo / "sim/fake_robots/maps" / f"{name}.yaml"))
KINDS = [{"A": 1, "C": 1}, {"B": 1}, {"D": 1, "A": 1}, {"C": 1}, {"B": 1, "D": 1}, {"A": 1}]
sc = {"map": name, "seed": 1, "speed_factor": 5.0, "timeout_s": 900,
      "stock": {"arm1": {"A": 40, "B": 40}, "arm2": {"C": 40, "D": 40}},
      "orders": [{"at_s": 20.0 * k, "id": f"R-{k+1:03d}", "items": KINDS[k % 6]} for k in range(8)],
      "robots": {n: {"start": s, "speed_mps": 0.15, "pose_noise_m": noise}
                 for n, s in (("pinky1", "P-3"), ("pinky2", "P-2"), ("pinky3", "P-1"))},
      "arm": {"seconds_per_item": 4.0, "swap_seconds": 8.0, "fail_rate": 0.0},
      "dock": {"duration_s": 3.0, "error_cm": 1.0}, "faults": [], "expect": {"orders_shipped": 8}}
import os
compare_floor.OVERRIDES = {"floor_lock_ahead_m": 0.15, "floor_extend_step_m": 0.05, "release_distance_m": release, "floor_tight_release": bool(os.environ.get("TIGHT"))}
with tempfile.TemporaryDirectory() as d:
    p = Path(d) / "run.yaml"; p.write_text(yaml.safe_dump(sc))
    row, world = compare_floor.run(name, str(p), seed, "freefloor", frame_every=2)
cell_cm2 = (floor.cell_m * 100) ** 2
mov, stand = [], []
prev = {}
for f in world.frames:
    n = collections.Counter(f["cells"].values())
    for r, (x, y) in f["robots"].items():
        if r in prev:
            (mov if math.dist(prev[r], (x, y)) > 0.005 else stand).append(n.get(r, 0) * cell_cm2)
        prev[r] = (x, y)
avg = lambda v: sum(v) / len(v) if v else 0
# 곧게 달릴 때 진행 방향에 수직인 잠금 폭 (로봇 자리의 칸 줄 수 x 칸 크기)
wid = []; modes = []
prev2 = {}
for f in world.frames:
    by = collections.defaultdict(list)
    for c, h in f["cells"].items():
        by[h].append(c)
    for r, (x, y) in f["robots"].items():
        if r in prev2:
            dx, dy = x - prev2[r][0], y - prev2[r][1]
            if math.hypot(dx, dy) > 0.02 and (abs(dx) < 0.002 or abs(dy) < 0.002):
                i0, j0 = floor.cell_of(x, y)
                ids = set(by.get(r, []))
                n = 0
                if abs(dy) < 0.002:  # x로 간다: 같은 열에서 세로 줄 수
                    n = sum(1 for j in range(j0 - 12, j0 + 13) if floor.inside((i0, j)) and floor.cell_id((i0, j)) in ids)
                else:
                    n = sum(1 for i in range(i0 - 12, i0 + 13) if floor.inside((i, j0)) and floor.cell_id((i, j0)) in ids)
                wid.append(n * floor.cell_m * 100); modes.append(f['mode'].get(r))
        prev2[r] = (x, y)
import statistics
big = sorted(range(len(world.tick_s)), key=lambda i: -world.tick_s[i])[:4]
print("   slowest ticks (index, ms):", [(i, round(1000*world.tick_s[i])) for i in big], "of", len(world.tick_s))
for k in ("band", "disc"):
    v = [w for w, mo in zip(wid, modes) if mo == k]
    if v: print(f"   {k} mean {sum(v)/len(v):.1f} cm", end=" |")
    print(f"   {k}: {len(v)} frames ({100*len(v)/max(1,len(wid)):.0f}%), lateral median {statistics.median(v) if v else 0:.0f} cm")
ts = sorted(world.tick_s)
print(f"   straight lateral lock width cm: median {statistics.median(wid) if wid else 0:.0f} min {min(wid) if wid else 0:.0f} max {max(wid) if wid else 0:.0f} (n {len(wid)}) | tick ms mean {1000*sum(ts)/len(ts):.1f} p99 {1000*ts[int(len(ts)*0.99)]:.1f} max {1000*ts[-1]:.1f}")
print(f"{name} cell {floor.cell_m} margin {floor.cell_margin_m} side {floor.side_margin_m} release {release} noise {noise} seed {seed}: "
      f"finished {row['finished']} end {row['end_s']} wait {row['wait_total_s']} gap {row['min_gap_m']} viol {row['violations']} "
      f"| moving {avg(mov):.0f} cm2 standing {avg(stand):.0f} cm2 | err {str(row['error'])[:110]}")
