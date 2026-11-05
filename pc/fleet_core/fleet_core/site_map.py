"""Course map: cells, one-way lanes, zones, stations and parking, loaded from YAML.

Cell coordinates, cell size and zone polygons are not agreed yet, so all of them live in
site_map.yaml. Nothing else in the code base hard-codes a coordinate.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import yaml

Cell = tuple[int, int]
DEFAULT_MAP = Path(__file__).parent / "data" / "site_map.yaml"


class SiteMapError(ValueError):
    """The map file is inconsistent, or a requested path does not exist."""


@dataclass(frozen=True)
class Station:
    id: str
    dock: Cell
    queue: tuple[Cell, ...]
    server: str  # who answers this station's action
    arms: tuple[str, ...]  # the arms working behind that server
    action: str  # fill or swap


@dataclass(frozen=True)
class Route:
    name: str
    flow: str
    cells: tuple[Cell, ...]


def _cell(raw) -> Cell:
    return (int(raw[0]), int(raw[1]))


class SiteMap:
    def __init__(self, data: dict, source_text: str = ""):
        self.source_text = source_text
        self.cols = int(data["grid"]["cols"])
        self.rows = int(data["grid"]["rows"])
        self.cell_size = float(data["cell_size_m"])
        self.origin = tuple(float(v) for v in data["origin_m"])
        self.obstacles = {_cell(c) for c in data["obstacles"]}
        self.arm_cells = {_cell(c) for c in data["arm_cells"]}
        self.routes = {
            name: Route(name, r["flow"], tuple(_cell(c) for c in r["cells"]))
            for name, r in data["routes"].items()
        }
        self.stations = {
            name: Station(name, _cell(s["dock"]), tuple(_cell(c) for c in s["queue"]),
                          s["server"], tuple(s["arms"]), s["action"])
            for name, s in data["stations"].items()
        }
        self.parking = tuple(_cell(c) for c in data["parking"])
        self.zones = {name: frozenset(_cell(c) for c in cells)
                      for name, cells in data["zones"].items()}
        self.docks = {s.dock for s in self.stations.values()}
        self._overrides = {
            tuple(int(v) for v in key.split(",")): (float(xy[0]), float(xy[1]))
            for key, xy in (data.get("cell_overrides") or {}).items()
        }
        self._polygons = {
            name: [(float(x), float(y)) for x, y in poly]
            for name, poly in (data.get("zone_polygons") or {}).items()
        }
        self.zone_of = self._build_zone_of()
        self._next = self._build_graph(data.get("links") or [])
        self._validate()

    def _build_zone_of(self) -> dict[Cell, str]:
        zone_of: dict[Cell, str] = {}
        for name, cells in self.zones.items():
            for cell in cells:
                if cell in zone_of:
                    raise SiteMapError(f"{cell} is in more than one zone")
                zone_of[cell] = name
        return zone_of

    def node(self, cell: Cell, flow: str) -> tuple[Cell, str]:
        """Docks carry no flow: a robot may leave a dock on any lane that starts there."""
        return (cell, "" if cell in self.docks else flow)

    def _build_graph(self, links: list[dict]) -> dict:
        graph: dict = {}
        for route in self.routes.values():
            for a, b in zip(route.cells, route.cells[1:], strict=False):
                if abs(a[0] - b[0]) + abs(a[1] - b[1]) != 1:
                    raise SiteMapError(f"route {route.name}: {a} and {b} are not adjacent")
                graph.setdefault(self.node(a, route.flow), []).append(self.node(b, route.flow))
        for link in links:
            a, b = _cell(link["from"]), _cell(link["to"])
            if abs(a[0] - b[0]) + abs(a[1] - b[1]) != 1:
                raise SiteMapError(f"link: {a} and {b} are not adjacent")
            graph.setdefault(self.node(a, link["from_flow"]), []).append(
                self.node(b, link["to_flow"]))
        return graph

    def _validate(self) -> None:
        route_cells = {c for r in self.routes.values() for c in r.cells}
        for cell in route_cells:
            if not (0 <= cell[0] < self.cols and 0 <= cell[1] < self.rows):
                raise SiteMapError(f"{cell} is outside the grid")
            if cell in self.obstacles:
                raise SiteMapError(f"{cell} is on a route and is an obstacle")
            if cell in self.arm_cells:
                raise SiteMapError(f"{cell} is on a route and is an arm cell")
            if cell not in self.zone_of:
                raise SiteMapError(f"{cell} is on a route but has no zone")
        extra = set(self.zone_of) - route_cells
        if extra:
            raise SiteMapError(f"zone cells are on no route: {sorted(extra)}")
        for station in self.stations.values():
            for cell in (station.dock, *station.queue):
                if cell not in route_cells:
                    raise SiteMapError(f"station {station.id}: {cell} is on no route")
        for cell in self.parking:
            if cell not in route_cells:
                raise SiteMapError(f"parking cell {cell} is on no route")

    def plan(self, start: Cell, flow: str, goal: Cell, goal_flow: str | None = None) -> list[Cell]:
        """Shortest path along the one-way lanes. Returned path includes both ends."""
        origin = self.node(start, flow)
        queue = deque([origin])
        prev = {origin: None}
        while queue:
            current = queue.popleft()
            at_goal = current[0] == goal and (goal_flow is None or current[1] == goal_flow)
            if at_goal:
                path = []
                node = current
                while node is not None:
                    path.append(node[0])
                    node = prev[node]
                return path[::-1]
            for nxt in self._next.get(current, []):
                if nxt not in prev:
                    prev[nxt] = current
                    queue.append(nxt)
        raise SiteMapError(f"no one-way path from {start} ({flow!r}) to {goal}")

    def flow_after(self, a: Cell, b: Cell) -> str:
        """Return the flow a robot is on after driving from a to b."""
        for node_a, targets in self._next.items():
            if node_a[0] != a:
                continue
            for node_b in targets:
                if node_b[0] == b:
                    return node_b[1]
        raise SiteMapError(f"no lane from {a} to {b}")

    def flows_at(self, cell: Cell) -> list[str]:
        """Flows that exist at a cell, used to place a robot that starts there."""
        flows = {n[1] for n in self._next if n[0] == cell}
        flows |= {t[1] for targets in self._next.values() for t in targets if t[0] == cell}
        return sorted(flows)

    def cell_pose(self, cell: Cell) -> tuple[float, float]:
        if cell in self._overrides:
            return self._overrides[cell]
        return (self.origin[0] + (cell[0] + 0.5) * self.cell_size,
                self.origin[1] + (cell[1] + 0.5) * self.cell_size)

    def cell_id(self, cell: Cell) -> str:
        return f"c{cell[0]}_{cell[1]}"

    def zone_cells(self, zone: str) -> frozenset:
        return self.zones[zone]

    def nearest_cell(self, x: float, y: float, cells=None) -> Cell:
        candidates = cells if cells is not None else self.zone_of
        return min(candidates, key=lambda c: math.dist((x, y), self.cell_pose(c)))

    def _distance_to_cell(self, x: float, y: float, cell: Cell) -> float:
        cx, cy = self.cell_pose(cell)
        half = self.cell_size / 2
        dx = max(abs(x - cx) - half, 0.0)
        dy = max(abs(y - cy) - half, 0.0)
        return math.hypot(dx, dy)

    def distance_to_zone(self, x: float, y: float, zone: str) -> float:
        """Distance from a pose to a zone. Zero when the pose is inside."""
        if zone in self._polygons:
            return _distance_to_polygon(x, y, self._polygons[zone])
        return min(self._distance_to_cell(x, y, c) for c in self.zones[zone])


def _distance_to_polygon(x: float, y: float, poly: list[tuple[float, float]]) -> float:
    inside = False
    best = math.inf
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        if (ay > y) != (by > y) and x < (bx - ax) * (y - ay) / (by - ay) + ax:
            inside = not inside
        vx, vy = bx - ax, by - ay
        denom = vx * vx + vy * vy
        t = max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / denom)) if denom else 0.0
        best = min(best, math.hypot(x - (ax + t * vx), y - (ay + t * vy)))
    return 0.0 if inside else best


def load_site_map(path=None) -> SiteMap:
    source = Path(path) if path else DEFAULT_MAP
    text = source.read_text(encoding="utf-8")
    return SiteMap(yaml.safe_load(text), text)
