"""tools/floor_view.py(P1): 칸 잠금 기록을 재생하는 html 한 장."""
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))

import floor_view  # noqa: E402
from fleet_core.traffic.floor.floor_map import load_floor_map  # noqa: E402

FRAMES = [
    {"t": 0.0, "cells": {"c12_14": "pinky1"}, "robots": {"pinky1": [1.05, 1.55]},
     "paths": {"pinky1": [[1.05, 1.55], [1.45, 1.15]]}},
    {"t": 0.1, "cells": {"c12_14": "pinky1", "c13_13": "pinky1"},
     "robots": {"pinky1": [1.07, 1.53]}, "paths": {}},
]


def embedded(html: str) -> dict:
    start = html.index("const DATA = ") + len("const DATA = ")
    end = html.index(";\n", start)
    return json.loads(html[start:end])


def test_the_page_carries_the_floor_and_every_frame():
    floor = load_floor_map()
    html = floor_view.render(floor, FRAMES, title="test")
    data = embedded(html)
    assert data["cols"] == floor.cols and data["rows"] == floor.rows
    assert len(data["frames"]) == 2
    assert data["frames"][1]["cells"] == {"c13_13": "pinky1", "c12_14": "pinky1"}
    assert set(data["lines"]) == set(floor.fixed_lines)
    assert len(data["blocked"]) == floor.cols * floor.rows - len(floor.clear)
    assert "<canvas" in html and "<title>test</title>" in html


def test_the_map_alone_renders_without_frames():
    html = floor_view.render(load_floor_map(), [], title="floor")
    assert embedded(html)["frames"] == []


def test_the_command_reads_a_jsonl_record(tmp_path):
    record = tmp_path / "run.jsonl"
    lines = [json.dumps({"floor": "floor_map"}), *(json.dumps(f) for f in FRAMES)]
    record.write_bytes(("\n".join(lines) + "\n").encode())
    out = tmp_path / "run.html"
    assert floor_view.main([str(record), "--out", str(out)]) == 0
    assert len(embedded(out.read_text(encoding="utf-8"))["frames"]) == 2


def test_the_command_draws_the_bare_map(tmp_path):
    out = tmp_path / "map.html"
    assert floor_view.main(["--map", "floor_map", "--out", str(out)]) == 0
    assert embedded(out.read_text(encoding="utf-8"))["frames"] == []


def test_a_robot_name_cannot_close_the_script():
    frames = [{"t": 0.0, "cells": {}, "robots": {"</script><b>x": [1.0, 1.0]}}]
    html = floor_view.render(load_floor_map(), frames, title="t")
    assert "</script><b>" not in html
    assert "innerHTML" not in html
