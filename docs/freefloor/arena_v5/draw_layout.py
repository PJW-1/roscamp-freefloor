"""arena_v5 배치도: 물체를 놓을 자리를 벽에서 잰 치수(cm)와 함께 그린다."""
import sys, yaml
from pathlib import Path
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
plt.rcParams["font.family"] = "Noto Sans CJK JP"
plt.rcParams["axes.unicode_minus"] = False
repo = Path(sys.argv[1]); out = sys.argv[2]
site = yaml.safe_load((repo / "sim/fleet_gz_sim/sites/arena_v5.yaml").read_text())
fl = yaml.safe_load((repo / "pc/fleet_core/fleet_core/data/floor_map_arena_v5.yaml").read_text())
W, H = site["bounds_m"][2], site["bounds_m"][3]
fig, ax = plt.subplots(figsize=(15, 8.6))
ax.add_patch(plt.Rectangle((-0.03, -0.03), W + 0.06, H + 0.06, color="#2f3640"))
ax.add_patch(plt.Rectangle((0, 0), W, H, color="#fbfaf5"))
for gx in range(0, 20):
    ax.plot([gx / 10, gx / 10], [0, H], color="#e6e3d8", lw=0.6, zorder=1)
for gy in range(0, 10):
    ax.plot([0, W], [gy / 10, gy / 10], color="#e6e3d8", lw=0.6, zorder=1)
names = {"object1": "물체 1\n17 x 16", "object3": "물체 3\n27 x 13", "object4": "물체 4   51 x 14  (북서쪽 구석에 붙임)",
         "object2": "물체 2   29 x 17\n(남서쪽 구석에 붙임)"}
for b in site["boxes"]:
    ax.add_patch(plt.Rectangle((b["x0"], b["y0"]), b["x1"] - b["x0"], b["y1"] - b["y0"], fc="#d8c7a9", ec="#7a6648", lw=2, zorder=3))
    ax.text((b["x0"] + b["x1"]) / 2, (b["y0"] + b["y1"]) / 2, names[b["name"]], ha="center", va="center", fontsize=12, weight="bold", color="#4a3b25", zorder=4)
def dim(p, q, text, off=(0, 0), color="#c0392b", fs=11.5):
    ax.annotate("", xy=q, xytext=p, arrowprops=dict(arrowstyle="<->", color=color, lw=1.6), zorder=6)
    ax.text((p[0] + q[0]) / 2 + off[0], (p[1] + q[1]) / 2 + off[1], text, ha="center", va="center", fontsize=fs, color=color, weight="bold",
            bbox=dict(fc="white", ec="none", pad=1.2, alpha=0.9), zorder=7)
# 놓는 자리 치수 (벽에서)
dim((0, 0.495), (0.52, 0.495), "52", off=(0.08, 0.035))
dim((0.605, 0), (0.605, 0.42), "42", off=(-0.06, 0))
dim((0.69, 0.495), (1.03, 0.495), "34", off=(0, 0.035))
dim((1.165, 0), (1.165, 0.43), "43", off=(0.05, 0))
dim((1.165, 0.56), (1.165, 0.99), "43", off=(0.05, 0))
dim((0.605, 0.58), (0.605, 0.99), "41", off=(0.06, 0.06))
dim((0.515, 0.58), (0.515, 0.85), "27", off=(-0.055, 0), color="#d35400")
dim((1.30, 0.495), (1.98, 0.495), "68", off=(-0.1, 0.035))
dim((0, 0.33), (1.03, 0.33), "서쪽 벽에서 물체 3까지 103", off=(0.33, 0.035), color="#8e44ad")
# 도크와 대기 자리
docks = fl["docks"]
for name, (x, y), c, lab in (("WORK", docks["WORK"], "#e1a100", "작업 도크"), ("OUTBOUND", docks["OUTBOUND"], "#e67e22", "출고 도크")):
    ax.add_patch(plt.Circle((x, y), 0.083, fc=c, ec="#5d4500", alpha=0.35, lw=1.5, zorder=4))
    ax.text(x, y, lab, ha="center", va="center", fontsize=10.5, weight="bold", zorder=5)
for line in fl["fixed_lines"].values():
    (x0, y0), (x1, y1) = line
    ax.annotate("", xy=(x1, y1), xytext=(x0 + (x0 - x1) * 0.6, y0 + (y0 - y1) * 0.6), arrowprops=dict(arrowstyle="-|>", color="#d35400", lw=3), zorder=6)
for p in fl["parking"]:
    ax.add_patch(plt.Circle((p["x"], p["y"]), 0.083, fc="#7dbb5c", ec="#2e6b1e", alpha=0.45, lw=1.5, zorder=4))
    ax.text(p["x"], p["y"], p["id"], ha="center", va="center", fontsize=10.5, weight="bold", zorder=5)
ax.text(0.89, 0.88, "북쪽 통로 41-43 cm  (양쪽 방향)", ha="center", fontsize=12, color="#1f6f8b", weight="bold")
ax.text(0.89, 0.07, "남쪽 통로 42-43 cm  (양쪽 방향)", ha="center", fontsize=12, color="#1f6f8b", weight="bold")
ax.text(0.86, 0.66, "건너는 통로", ha="center", va="center", fontsize=11, color="#1f6f8b", weight="bold")
ax.text(0.30, 0.30, "출고 앞 마당", ha="center", fontsize=11, color="#555")
ax.text(1.55, 0.66, "동쪽 마당\n(대기 · 작업)", ha="center", va="center", fontsize=11, color="#555")
ax.text(-0.075, 0.52, "서쪽 벽\n(로봇팔 3)", ha="center", va="center", fontsize=10.5, rotation=90)
ax.text(1.78, -0.075, "남쪽 벽 (로봇팔 1 · 2)", ha="center", va="center", fontsize=10.5)
ax.set_title("arena_v5 배치 제안  ·  벽 안쪽 198 x 99 cm  ·  치수는 cm  ·  주황 화살표 = 도크에 들어가는 방향", fontsize=14, weight="bold", pad=12)
ax.set_xlim(-0.13, W + 0.07); ax.set_ylim(-0.12, H + 0.07); ax.set_aspect("equal"); ax.axis("off")
fig.savefig(out, dpi=120, bbox_inches="tight", facecolor="white")
