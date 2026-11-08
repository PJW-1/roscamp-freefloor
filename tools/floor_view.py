#!/usr/bin/env python3
"""Free-floor cell-lock viewer (free-floor experiment P1): one self-contained HTML page that
replays a record of cell owners, robot positions and sent paths over the floor map.

    python3 tools/floor_view.py run.jsonl --out run.html     # replay a record
    python3 tools/floor_view.py --map floor_map --out floor.html   # the bare floor

Record format (JSON lines): the first line is a header {"floor": <map name or yaml path>}, then
one frame per line: {"t": s, "cells": {cell_id: robot}, "robots": {robot: [x, y]},
"paths": {robot: [[x, y], ...]}}. Frames may leave out "paths". FloorWorld writes such records
(pc/fleet_core/test/floor_world.py). The page has no outside scripts: open it in any browser.
"""
from __future__ import annotations

import argparse
import html
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "pc" / "fleet_core"))

from fleet_core.traffic.floor.floor_map import DEFAULT_FLOOR, load_floor_map  # noqa: E402

PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>%(title)s</title>
<style>
:root { --bg: #fafaf7; --ink: #222; --wall: #555; --tight: #ddd; --grid: #eee; }
@media (prefers-color-scheme: dark) {
  :root { --bg: #1b1b1b; --ink: #ddd; --wall: #999; --tight: #333; --grid: #262626; }
}
body { margin: 0; padding: 12px 16px; background: var(--bg); color: var(--ink);
       font: 14px system-ui, sans-serif; }
canvas { max-width: 100%%; height: auto; border: 1px solid var(--grid); }
#bar { display: flex; gap: 12px; align-items: center; flex-wrap: wrap; margin: 8px 0; }
#t { font-variant-numeric: tabular-nums; min-width: 7em; }
input[type=range] { flex: 1; min-width: 160px; }
</style></head><body>
<h1 style="font-size:16px;margin:0 0 4px">%(title)s</h1>
<div id="bar"><button id="play">play</button><input id="slider" type="range" min="0" value="0">
<span id="t"></span><span id="legend"></span></div>
<canvas id="c"></canvas>
<script>
const DATA = %(data)s;
const PX = 18, cv = document.getElementById("c"), cx = cv.getContext("2d");
cv.width = DATA.cols * PX; cv.height = DATA.rows * PX;
const COLORS = ["#d1495b", "#2e86ab", "#3f8f3f", "#e0a100", "#8e44ad", "#16a085"];
const robots = [...new Set(DATA.frames.flatMap(f => Object.keys(f.robots || {})))].sort();
const color = r => COLORS[robots.indexOf(r) %% COLORS.length];
for (const r of robots) {
  const tag = document.createElement("span");
  tag.style.color = color(r); tag.style.marginRight = "8px";
  tag.textContent = "■ " + r;
  document.getElementById("legend").appendChild(tag);
}
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n);
const toPx = (x, y) => [(x - DATA.x0) / DATA.cell * PX, cv.height - (y - DATA.y0) / DATA.cell * PX];
function cellRect(id) {
  const [i, j] = id.slice(1).split("_").map(Number);
  return [i * PX, cv.height - (j + 1) * PX];
}
function seg(a, b, c, d) { cx.beginPath(); cx.moveTo(a, b); cx.lineTo(c, d); cx.stroke(); }
function polyline(pts) {
  cx.beginPath();
  pts.forEach(([x, y], n) => { const [u, v] = toPx(x, y); n ? cx.lineTo(u, v) : cx.moveTo(u, v); });
  cx.stroke();
}
function draw(k) {
  cx.fillStyle = css("--bg"); cx.fillRect(0, 0, cv.width, cv.height);
  cx.strokeStyle = css("--grid");
  for (let i = 0; i <= DATA.cols; i++) seg(i * PX, 0, i * PX, cv.height);
  for (let j = 0; j <= DATA.rows; j++) seg(0, j * PX, cv.width, j * PX);
  for (const id of DATA.blocked) {
    const [x, y] = cellRect(id);
    cx.fillStyle = DATA.walls.includes(id) ? css("--wall") : css("--tight");
    cx.fillRect(x, y, PX, PX);
  }
  cx.lineWidth = 2; cx.setLineDash([4, 3]); cx.strokeStyle = "#888";
  for (const pts of Object.values(DATA.lines)) polyline(pts);
  cx.setLineDash([]);
  const f = DATA.frames[k];
  if (!f) return;
  document.getElementById("t").textContent = `t = ${f.t.toFixed(1)} s`;
  for (const [id, r] of Object.entries(f.cells)) {
    const [x, y] = cellRect(id);
    cx.globalAlpha = 0.35; cx.fillStyle = color(r); cx.fillRect(x, y, PX, PX); cx.globalAlpha = 1;
  }
  for (const [r, pts] of Object.entries(f.paths || {})) {
    cx.strokeStyle = color(r); cx.lineWidth = 2; polyline(pts);
  }
  for (const [r, [x, y]] of Object.entries(f.robots || {})) {
    const [u, v] = toPx(x, y);
    cx.fillStyle = color(r); cx.beginPath();
    cx.arc(u, v, DATA.radius / DATA.cell * PX, 0, 7); cx.fill();
  }
}
const slider = document.getElementById("slider");
slider.max = Math.max(0, DATA.frames.length - 1);
slider.oninput = () => draw(+slider.value);
let timer = null;
document.getElementById("play").onclick = () => {
  if (timer) { clearInterval(timer); timer = null; return; }
  timer = setInterval(() => {
    if (+slider.value >= +slider.max) { clearInterval(timer); timer = null; return; }
    slider.value = +slider.value + 1; draw(+slider.value);
  }, 50);
};
draw(0);
</script></body></html>
"""


def render(floor, frames, title="floor") -> str:
    """The page as a string. Frames as in the record format."""
    blocked = sorted(floor.cell_id((i, j)) for i in range(floor.cols) for j in range(floor.rows)
                     if (i, j) not in floor.clear)
    walls = sorted(floor.cell_id(c) for c in
                   ((i, j) for i in range(floor.cols) for j in range(floor.rows))
                   if c not in floor.free)
    x0, y0, _, _ = floor.bounds()
    data = {
        "cols": floor.cols, "rows": floor.rows, "cell": floor.cell_m, "x0": x0, "y0": y0,
        "radius": floor.robot_radius_m, "blocked": blocked, "walls": walls,
        "lines": {name: [list(p) for p in line.points]
                  for name, line in floor.fixed_lines.items()},
        "frames": [{"t": f["t"], "cells": dict(sorted(f.get("cells", {}).items())),
                    "robots": f.get("robots", {}), "paths": f.get("paths", {})}
                   for f in frames],
    }
    # '<' never reaches the page raw, so no name can close the script element
    text = json.dumps(data, separators=(",", ":")).replace("<", "\\u003c")
    return PAGE % {"title": html.escape(title), "data": text}


def floor_path(name: str) -> Path:
    if name.endswith(".yaml"):
        return Path(name)
    return DEFAULT_FLOOR.parent / f"{name}.yaml"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("record", nargs="?", help="JSON-lines record (header line first)")
    parser.add_argument("--map", help="floor map name or yaml, for the bare floor")
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    frames = []
    name = args.map or "floor_map"
    if args.record:
        lines = Path(args.record).read_text(encoding="utf-8").splitlines()
        name = json.loads(lines[0]).get("floor", name)
        frames = [json.loads(line) for line in lines[1:] if line.strip()]
    floor = load_floor_map(floor_path(name))
    title = Path(args.record).stem if args.record else Path(name).stem
    Path(args.out).write_bytes(render(floor, frames, title).encode("utf-8"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
