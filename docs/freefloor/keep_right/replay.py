"""남쪽 통로에서 마주 오는 두 대를 스위치 켬/끔으로 돌려 재생 기록과 궤적 그림을 만든다.
사용: replay.py <저장소> <출력 폴더>"""
import json, math, sys
from pathlib import Path
repo, out = Path(sys.argv[1]), Path(sys.argv[2])
sys.path.insert(0, str(repo / "pc/fleet_core/test")); sys.path.insert(0, str(repo / "pc/fleet_core")); sys.path.insert(0, str(repo / "tools"))
import test_floor_keep_right as T
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import yaml
out.mkdir(parents=True, exist_ok=True)
boxes = yaml.safe_load((repo / "sim/fleet_gz_sim/sites/arena_v5.yaml").read_text())["boxes"]
fig, axes = plt.subplots(2, 1, figsize=(10, 10.5))
for ax, on in zip(axes, (True, False)):
    w, traffic = T.world(T.SOUTH_POSES, keep_right=on)
    w.frame_every = 1
    T.send(traffic, T.SOUTH_GOALS)
    side, north, longest, reached = T.drive(w, traffic, T.SOUTH_GOALS)
    tag = "on" if on else "off"
    rec = out / f"south_passage_keep_right_{tag}.jsonl"
    with rec.open("w") as f:
        f.write(json.dumps({"floor": "floor_map_arena_v5"}) + "\n")
        for fr in w.frames:
            f.write(json.dumps(fr) + "\n")
    codes = [c for c in T.codes(w) if c.startswith(("FLOOR_", "DEADLOCK"))]
    print(tag, "abreast steps", side, "longest stop", {k: round(v, 1) for k, v in longest.items()}, "reached", reached, "t", round(w.t, 1), "events", codes)
    for b in boxes:
        ax.add_patch(plt.Rectangle((b["x0"], b["y0"]), b["x1"] - b["x0"], b["y1"] - b["y0"], color="#b9a27c"))
    for name, c in (("pinky1", "tab:blue"), ("pinky2", "tab:green")):
        xs = [fr["robots"][name][0] for fr in w.frames]; ys = [fr["robots"][name][1] for fr in w.frames]
        ax.plot(xs, ys, color=c, lw=1.6, label=name); ax.plot(xs[0], ys[0], "o", color=c)
    ax.set_xlim(0, 1.98); ax.set_ylim(0, 0.99); ax.set_aspect("equal"); ax.legend(loc="upper right")
    ax.set_title(f"floor_keep_right {tag}: side by side for {side} steps (0.1 s each), longest stop {max(longest.values() or [0]):.1f} s, both arrived at {w.t:.1f} s")
fig.savefig(out / "south_passage_tracks.png", dpi=110, bbox_inches="tight")
