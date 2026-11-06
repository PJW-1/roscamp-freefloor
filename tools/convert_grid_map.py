#!/usr/bin/env python3
"""Turn the old cell map into a lane map, one waypoint per cell, its zones kept as lock units.

Used once, to make pc/fleet_core/test/data/grid_cells.yaml, which keeps the older tests on the
layout they were written for. The grid file is pc/fleet_core/fleet_core/data/site_map.yaml in
commit d511c57.

    python3 tools/convert_grid_map.py old_site_map.yaml > grid_cells.yaml
"""
import sys

import yaml


def main(path: str) -> None:
    old = yaml.safe_load(open(path, encoding="utf-8"))
    size = old["cell_size_m"]
    ox, oy = old["origin_m"]
    lanes = []

    def add(a, b):
        if [a, b] not in lanes:
            lanes.append([a, b])

    for route in old["routes"].values():
        for a, b in zip(route["cells"], route["cells"][1:], strict=False):
            add(a, b)
    for link in old["links"]:
        add(link["from"], link["to"])
    cells = sorted({tuple(c) for lane in lanes for c in lane})

    def flow(items):
        return yaml.safe_dump(items, default_flow_style=True, width=1000).strip()

    print("# The old 12 x 7 cell map as a lane map. One waypoint per cell, 0.33 m lanes, one-way,")
    print("# its zones kept as lock units. Made by tools/convert_grid_map.py for the older tests.")
    print(f"half_width_m: {size / 2}  # half a cell: a robot belongs to the nearer cell, as before")
    print(f"lane_entry_m: {size}  # a zone is its cells, not the lanes leading to them")
    print(f"origin_m: {flow([ox, oy])}")
    print("waypoints:")
    for c, r in cells:
        x, y = round(ox + (c + 0.5) * size, 4), round(oy + (r + 0.5) * size, 4)
        print(f"  - {{id: [{c},{r}], x: {x}, y: {y}}}")
    print("lanes:")
    for a, b in lanes:
        print(f"  - {{from: {flow(a)}, to: {flow(b)}}}")
    print("units:")
    for name, members in old["zones"].items():
        print(f"  {name}: {flow(members)}")
    print("stations:")
    for name, s in old["stations"].items():
        chain = [*reversed(s["queue"]), s["dock"]]  # the old list starts next to the dock
        queue = [[list(a), list(b)] for a, b in zip(chain, chain[1:], strict=False)]
        commit = {"WORK": old["zones"]["Q-WORK-2"][0]}.get(name, s["dock"])
        print(f"  {name}: {{dock: {flow(s['dock'])}, queue: {flow(queue)}, commit: {flow(commit)}, "
              f"server: {s['server']}, arms: {flow(s['arms'])}, action: {s['action']}}}")
    print(f"parking: {flow(old['parking'])}")


if __name__ == "__main__":
    main(sys.argv[1])
