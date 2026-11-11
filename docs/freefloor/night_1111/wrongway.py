"""머리 없는 시뮬(arena_v6, 주문 8건)에서 OUTBOUND로 가는 로봇이 긴 진열대 남쪽·사이 통로를 동쪽으로 가는 순간을 찾는다.
사용: wrongway.py <repo> <seed> [verbose]"""
import sys, yaml, dataclasses, tempfile, os
repo = sys.argv[1]; seed = int(sys.argv[2]); verbose = len(sys.argv) > 3
for part in ("tools", "pc/fleet_core/test", "pc/fleet_core", "sim/fake_robots"):
    sys.path.insert(0, f"{repo}/{part}")
import compare_floor
from floor_world import FloorWorld
from pathlib import Path
from fleet_core.common.params import load_params
base = load_params()
v6 = load_params(Path(f"{repo}/pc/fleet_core/fleet_core/data/fleet_params_arena_v6.yaml"))
compare_floor.OVERRIDES = {f.name: getattr(v6, f.name) for f in dataclasses.fields(v6)
                           if getattr(v6, f.name) != getattr(base, f.name) and f.name not in ("traffic_mode", "lane_goal_mode")}
data = yaml.safe_load(open(f"{repo}/sim/fleet_gz_sim/scenarios/gz_arena_v6.yaml"))
SP = float(os.environ.get("SP", 0.15)); DK = float(os.environ.get("DK", 3.0)); SW = float(os.environ.get("SW", 8.0))
for spec in data["robots"].values(): spec.update(speed_mps=SP, pose_noise_m=0.03)
data["dock"]["duration_s"] = DK; data["arm"]["swap_seconds"] = SW
data["timeout_s"] = 600
path = os.path.join(tempfile.mkdtemp(), "run.yaml"); open(path, "w").write(yaml.safe_dump(data))
prev = {}; hits = []; last_route = {}
step0 = FloorWorld.step
def step(world, *a, **k):
    out = step0(world, *a, **k)
    t = world.core.traffic
    for n, r in world.virtual.items():
        m = t._missions[n]
        if n in prev and r.x - prev[n] > 0.002 and 1.15 < r.x < 1.7 and r.y < 0.47 and m.phase == "TO_OUTBOUND":
            hits.append((round(world.t, 1), n, round(r.x, 2), round(r.y, 2)))
        prev[n] = r.x
        leg = t._legs.get(n)
        if verbose and leg is not None and leg.route is not None and leg.route is not last_route.get(n) and m.phase == "TO_OUTBOUND":
            last_route[n] = leg.route
            pts = leg.route.points
            if 180 < world.t < 200 or min(p[1] for p in pts) < 0.3:
                others = {o[-1]: (round(world.virtual[o].x, 2), round(world.virtual[o].y, 2), t._missions[o].phase, "wait" if (t._legs.get(o) and t._legs[o].waiting) else "") for o in world.virtual if o != n}
                print(round(world.t, 1), n, "new route by the south:", "waiting" if leg.waiting else "", "in_line" if leg.in_line else "", [tuple(round(v, 2) for v in p) for p in pts[::max(1, len(pts)//6)]], "end", tuple(round(v, 2) for v in pts[-1]), "admitted", leg.admitted, "others", others)
    return out
FloorWorld.step = step
row, world = compare_floor.run("arena_v6", path, seed, "freefloor")
print("SP", SP, "DK", DK, "SW", SW, "seed", seed, "finished", row.get("finished"), "end", row.get("end_s"), "shipped", row.get("orders_shipped"), "error", str(row.get("error"))[:80], "| wrong-way samples", len(hits), hits[:2])
