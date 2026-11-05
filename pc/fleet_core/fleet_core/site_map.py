"""Course map: nodes, directed lanes, lock units, stations and parking, loaded from YAML.

A robot drives straight from node to node, in whatever direction the lane points. There is no
grid. The real map is not agreed yet, so coordinates live in the map file and nothing else in
the code base hard-codes one: a new map is new data.

A lock unit is what one robot holds at a time, and it is also what the rest of the code calls a
zone. Every node belongs to exactly one unit, and a lane belongs to the unit of the node it
ends at:
  - Lanes that end at one node could never be used together, so they share a lock, and that
    lock's queue is also the merge's FIFO. Where one lane reaches each node, one lane is one lock.
  - A two-way pair is one lock when bundled, so two robots cannot meet head-on on it.
  - Lanes of different units too close for two robot bodies are found by conflict_pairs().

`cell` in a method name is the old word for a node. The scenario judge still calls these
methods, so they keep the name.
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from pathlib import Path

import yaml

DEFAULT_MAP = Path(__file__).parent / "data" / "site_map.yaml"


class SiteMapError(ValueError):
    """The map file is inconsistent, or a requested path does not exist."""


@dataclass(frozen=True)
class Station:
    id: str
    dock: object  # node
    queue: tuple  # (from, to) lanes where robots line up for the dock
    commit: object  # node: a robot holding its unit can no longer be sent elsewhere
    server: str  # who answers this station's action
    arms: tuple[str, ...]  # the arms working behind that server
    action: str  # fill or swap


@dataclass(frozen=True)
class Lane:
    a: object  # node it starts at
    b: object  # node it ends at
    ax: float
    ay: float
    bx: float
    by: float
    ux: float  # unit vector from a to b
    uy: float
    length: float  # planning cost, the straight distance unless the file says otherwise
    span: float  # straight distance


def _node_id(raw):
    """A node is named by a string, or by [col, row] on a map converted from cells."""
    if isinstance(raw, (list, tuple)):
        return tuple(int(v) for v in raw)
    return str(raw)


def _point_segment(x, y, ax, ay, bx, by) -> float:
    vx, vy = bx - ax, by - ay
    denom = vx * vx + vy * vy
    t = max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / denom)) if denom else 0.0
    return math.hypot(x - (ax + t * vx), y - (ay + t * vy))


def _segment_distance(a1, a2, b1, b2) -> float:
    """Shortest distance between two segments, each a pair of (x, y)."""
    def side(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    if (side(a1, a2, b1) * side(a1, a2, b2) < 0
            and side(b1, b2, a1) * side(b1, b2, a2) < 0):
        return 0.0
    return min(_point_segment(*a1, *b1, *b2), _point_segment(*a2, *b1, *b2),
               _point_segment(*b1, *a1, *a2), _point_segment(*b2, *a1, *a2))


class SiteMap:
    def __init__(self, data: dict, source_text: str = "", bundle_two_way: bool | None = None):
        self.source_text = source_text
        self.origin = tuple(float(v) for v in data["origin_m"])
        self.half_width = float(data["half_width_m"])
        self.lane_entry = float(data["lane_entry_m"])
        self._pose = {_node_id(n["id"]): (float(n["x"]), float(n["y"])) for n in data["nodes"]}
        if bundle_two_way is None:
            bundle_two_way = bool(data.get("bundle_two_way", True))
        self.bundle_two_way = bundle_two_way
        two_way = {name: (_node_id(a), _node_id(b))
                   for name, (a, b) in (data.get("two_way") or {}).items()}
        self.lanes = self._build_lanes(data.get("lanes") or [], two_way)
        self._out: dict = {}
        self._in: dict = {}
        for lane in self.lanes.values():
            self._out.setdefault(lane.a, []).append(lane)
            self._in.setdefault(lane.b, []).append(lane)
        self.stations = {name: self._station(name, s)
                         for name, s in (data.get("stations") or {}).items()}
        self.parking = tuple(self._known(_node_id(c), "parking") for c in data.get("parking") or [])
        self.docks = {s.dock for s in self.stations.values()}
        self.zones = self._build_zones(data.get("units") or {}, two_way, bundle_two_way)
        self.zone_of = {node: name for name, members in self.zones.items() for node in members}
        self._segments = {zone: self._zone_segments(zone) for zone in self.zones}
        for node in self._pose:
            if node not in self._out and node not in self._in:
                raise SiteMapError(f"{node} has no lane")

    def _known(self, node, what: str):
        if node not in self._pose:
            raise SiteMapError(f"{what}: {node} is nowhere on the map")
        return node

    def _build_lanes(self, raw: list[dict], two_way: dict) -> dict:
        lanes: dict = {}

        def add(a, b, length=None):
            self._known(a, "lane")
            self._known(b, "lane")
            if a == b:
                raise SiteMapError(f"lane from {a} to itself")
            if (a, b) in lanes:
                raise SiteMapError(f"lane from {a} to {b} is written twice")
            (ax, ay), (bx, by) = self._pose[a], self._pose[b]
            span = math.hypot(bx - ax, by - ay)
            if span <= 0.0:
                raise SiteMapError(f"lane from {a} to {b} has no length")
            lanes[(a, b)] = Lane(a, b, ax, ay, bx, by, (bx - ax) / span, (by - ay) / span,
                                 float(length) if length is not None else span, span)

        for lane in raw:
            add(_node_id(lane["from"]), _node_id(lane["to"]), lane.get("length"))
        for a, b in lanes:
            if (b, a) in lanes:
                raise SiteMapError(f"lanes {a} to {b} and back must be one two_way entry")
        for name, (a, b) in two_way.items():
            self._known(a, f"two_way {name}")
            self._known(b, f"two_way {name}")
            add(a, b)
            add(b, a)
        return lanes

    def _station(self, name: str, raw: dict) -> Station:
        dock = self._known(_node_id(raw["dock"]), f"station {name}")
        queue = tuple((_node_id(a), _node_id(b)) for a, b in raw.get("queue") or [])
        for lane in queue:
            if lane not in self.lanes:
                raise SiteMapError(f"station {name}: no lane from {lane[0]} to {lane[1]}")
        commit = self._known(_node_id(raw.get("commit", raw["dock"])), f"station {name}")
        return Station(name, dock, queue, commit, raw["server"], tuple(raw["arms"]), raw["action"])

    def _build_zones(self, units: dict, two_way: dict, bundle: bool) -> dict:
        """Every node in one unit: as written, else merged by a bundle, else on its own."""
        name_of: dict = {}
        for name, raw in units.items():
            for member in raw:
                node = self._known(_node_id(member), f"unit {name}")
                if node in name_of:
                    raise SiteMapError(f"{node} is in more than one unit")
                name_of[node] = name
        for node in self._pose:
            name_of.setdefault(node, self.cell_id(node))
        if bundle:
            for name, (a, b) in two_way.items():
                old = {name_of[a], name_of[b]}
                for node, unit in name_of.items():
                    if unit in old:
                        name_of[node] = name
        zones: dict = {}
        for node, name in name_of.items():
            zones.setdefault(name, set()).add(node)
        return {name: frozenset(members) for name, members in zones.items()}

    def _zone_segments(self, zone: str) -> list[tuple]:
        """The floor of a zone: its nodes, and its lanes from the point a robot has entered."""
        members = self.zones[zone]
        segments = [(*self._pose[n], *self._pose[n]) for n in members]
        for node in members:
            for lane in self._in.get(node, ()):
                skip = 0.0 if lane.a in members else self.lane_entry
                if skip < lane.span:
                    segments.append((lane.ax + lane.ux * skip, lane.ay + lane.uy * skip,
                                     lane.bx, lane.by))
        return segments

    # ---- geometry ---------------------------------------------------------------------------

    def lane(self, a, b) -> Lane:
        return self.lanes[(a, b)]

    def lanes_of_zone(self, zone: str) -> set:
        return {key for key in self.lanes if key[1] in self.zones[zone]}

    def lane_progress(self, a, b, x: float, y: float) -> tuple[float, float]:
        """s = (p - p_a)·u, how far along the lane, and e = |(p - p_a) - s·u|, how far off it."""
        lane = self.lanes[(a, b)]
        rx, ry = x - lane.ax, y - lane.ay
        s = rx * lane.ux + ry * lane.uy
        return s, math.hypot(rx - s * lane.ux, ry - s * lane.uy)

    def cell_pose(self, cell) -> tuple[float, float]:
        return self._pose[cell]

    def cell_id(self, cell) -> str:
        return f"c{cell[0]}_{cell[1]}" if isinstance(cell, tuple) else cell

    def nearest_cell(self, x: float, y: float, cells=None, radius: float | None = None):
        """The node whose unit a pose belongs to.

        Standing on a node, that node: within `radius`, half_width by default. On a lane, the node
        it leads to, because driving on a lane needs that node's unit. A true pose can use the
        narrow default, a single noisy report cannot, so what decides from one report passes a
        wider radius. `cells` restricts the answer to the nearest of those nodes.
        """
        if cells is not None:
            return min(cells, key=lambda c: math.dist((x, y), self._pose[c]))
        node = min(self._pose, key=lambda c: math.dist((x, y), self._pose[c]))
        limit = self.half_width if radius is None else radius
        if math.dist((x, y), self._pose[node]) <= limit:
            return node
        lane = min(self.lanes.values(),
                   key=lambda one: _point_segment(x, y, one.ax, one.ay, one.bx, one.by))
        return lane.b

    def distance_to_zone(self, x: float, y: float, zone: str) -> float:
        """Distance from a pose to a zone's floor. Zero when the pose is inside."""
        nearest = min(_point_segment(x, y, *segment) for segment in self._segments[zone])
        return max(0.0, nearest - self.half_width)

    def shortest_exit_lane(self) -> float:
        """Length of the shortest lane leaving a unit, which a release has to fit on."""
        return min((lane.span for (a, b), lane in self.lanes.items()
                    if self.zone_of[a] != self.zone_of[b]), default=math.inf)

    # ---- conflict pairs -----------------------------------------------------------------------

    def conflict_pairs(self, clearance_m: float, release_m: float) -> set[frozenset]:
        """Pairs of units that must not be held together: their lanes pass closer than 2r.

        Lanes that meet at a node would always count, and then nobody could follow anybody. A
        robot gives up the unit behind it only after driving release_m along the next lane, so
        from the shared node that stretch cannot hold it while another robot holds the first
        unit, and only the rest of the lane counts. The two directions of a two-way pair are left
        out: holding them together is what the bundle is for.
        """
        pairs: set[frozenset] = set()
        lanes = list(self.lanes.values())
        for i, one in enumerate(lanes):
            for other in lanes[i + 1:]:
                zone_a, zone_b = self.zone_of[one.b], self.zone_of[other.b]
                if zone_a != zone_b and self._too_close(one, other, clearance_m, release_m):
                    pairs.add(frozenset({zone_a, zone_b}))
        return pairs

    def _too_close(self, one: Lane, other: Lane, clearance: float, release: float) -> bool:
        if frozenset({one.a, one.b}) == frozenset({other.a, other.b}):
            return False  # a two-way pair
        shared = {one.a, one.b} & {other.a, other.b}
        if not shared:
            return self._lane_distance(one, 0.0, other, 0.0) < clearance
        node = next(iter(shared))
        if one.a == node and other.a == node:  # a fork: either one can be the leader
            return (self._lane_distance(one, release, other, 0.0) < clearance
                    or self._lane_distance(one, 0.0, other, release) < clearance)
        if one.a == node:  # other leads into one
            return self._lane_distance(one, release, other, 0.0) < clearance
        return self._lane_distance(one, 0.0, other, release) < clearance

    @staticmethod
    def _lane_distance(one: Lane, skip_one: float, other: Lane, skip_other: float) -> float:
        """Distance between two lanes, leaving out the first metres of each."""
        if skip_one >= one.span or skip_other >= other.span:
            return math.inf
        return _segment_distance(
            (one.ax + one.ux * skip_one, one.ay + one.uy * skip_one), (one.bx, one.by),
            (other.ax + other.ux * skip_other, other.ay + other.uy * skip_other),
            (other.bx, other.by))

    # ---- planning -----------------------------------------------------------------------------

    def path_length(self, path: list) -> float:
        return sum(self.lanes[(a, b)].length for a, b in zip(path, path[1:], strict=False))

    def plan(self, start, goal) -> list:
        """Shortest path along the lanes, in metres. The returned path includes both ends."""
        best = {start: 0.0}
        prev: dict = {start: None}
        seen = 0  # breaks ties in the heap without comparing nodes
        heap = [(0.0, seen, start)]
        while heap:
            cost, _, node = heapq.heappop(heap)
            if node == goal:
                path = []
                while node is not None:
                    path.append(node)
                    node = prev[node]
                return path[::-1]
            if cost > best[node]:
                continue
            for lane in self._out.get(node, ()):
                total = cost + lane.length
                if total < best.get(lane.b, math.inf):
                    best[lane.b], prev[lane.b] = total, node
                    seen += 1
                    heapq.heappush(heap, (total, seen, lane.b))
        raise SiteMapError(f"no path along the lanes from {start} to {goal}")


def load_site_map(path=None, bundle_two_way: bool | None = None) -> SiteMap:
    source = Path(path) if path else DEFAULT_MAP
    text = source.read_text(encoding="utf-8")
    return SiteMap(yaml.safe_load(text), text, bundle_two_way)
