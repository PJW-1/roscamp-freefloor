"""Course map: waypoints, directed lanes, lock units, stations and parking, loaded from YAML.

A robot drives straight from waypoint to waypoint, in whatever direction the lane points. There is
no grid. The real map is not agreed yet, so coordinates live in the map file and nothing else in
the code base hard-codes one: a new map is new data.

A lock unit is what one robot holds at a time, and it is also what the rest of the code calls a
zone. Every waypoint belongs to exactly one unit, and a lane belongs to the unit of the waypoint it
ends at:
  - Lanes that end at one waypoint could never be used together, so they share a lock, and that
    lock's queue is also the merge's FIFO. Where one lane reaches each waypoint, one lane is one
    lock.
  - A two-way pair is one lock when bundled, so two robots cannot meet head-on on it.
  - Lanes of different units too close for two robot bodies are found by conflict_pairs().

A waypoint is a vertex of the lane graph. It is not called a node, to keep that word for ROS.
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from pathlib import Path

import yaml

DEFAULT_MAP = Path(__file__).parents[1] / "data" / "lane_map.yaml"


class LaneMapError(ValueError):
    """The map file is inconsistent, or a requested path does not exist."""


@dataclass(frozen=True)
class Station:
    id: str
    dock: object  # waypoint
    queue: tuple  # (from, to) lanes where robots line up for the dock
    commit: object  # waypoint: a robot holding its unit can no longer be sent elsewhere
    server: str  # who answers this station's action
    arms: tuple[str, ...]  # the arms working behind that server
    action: str  # fill or swap


@dataclass(frozen=True)
class Lane:
    a: object  # waypoint it starts at
    b: object  # waypoint it ends at
    ax: float
    ay: float
    bx: float
    by: float
    ux: float  # unit vector from a to b
    uy: float
    length: float  # planning cost, the straight distance unless the file says otherwise
    span: float  # straight distance


def _waypoint_id(raw):
    """A waypoint is named by a string, or by [col, row] on a map converted from waypoints."""
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


class LaneMap:
    def __init__(self, data: dict, source_text: str = "", bundle_two_way: bool | None = None):
        self.source_text = source_text
        self.origin = tuple(float(v) for v in data["origin_m"])
        self.half_width = float(data["half_width_m"])
        self.lane_entry = float(data["lane_entry_m"])
        if "waypoints" not in data and "nodes" in data:
            raise LaneMapError("the map lists `nodes:`; since 2026-11-06 the key is `waypoints:`")
        self._pose = {_waypoint_id(n["id"]): (float(n["x"]), float(n["y"]))
                      for n in data["waypoints"]}
        if bundle_two_way is None:
            bundle_two_way = bool(data.get("bundle_two_way", True))
        self.bundle_two_way = bundle_two_way
        two_way = {name: (_waypoint_id(a), _waypoint_id(b))
                   for name, (a, b) in (data.get("two_way") or {}).items()}
        self.lanes = self._build_lanes(data.get("lanes") or [], two_way)
        self._out: dict = {}
        self._in: dict = {}
        for lane in self.lanes.values():
            self._out.setdefault(lane.a, []).append(lane)
            self._in.setdefault(lane.b, []).append(lane)
        self.stations = {name: self._station(name, s)
                         for name, s in (data.get("stations") or {}).items()}
        self.parking = tuple(self._known(_waypoint_id(c), "parking")
                             for c in data.get("parking") or [])
        self.docks = {s.dock for s in self.stations.values()}
        self.zones = self._build_zones(data.get("units") or {}, two_way, bundle_two_way)
        self.zone_of = {waypoint: name for name, members in self.zones.items()
                        for waypoint in members}
        self._segments = {zone: self._zone_segments(zone) for zone in self.zones}
        for waypoint in self._pose:
            if waypoint not in self._out and waypoint not in self._in:
                raise LaneMapError(f"{waypoint} has no lane")

    def _known(self, waypoint, what: str):
        if waypoint not in self._pose:
            raise LaneMapError(f"{what}: {waypoint} is nowhere on the map")
        return waypoint

    def _build_lanes(self, raw: list[dict], two_way: dict) -> dict:
        lanes: dict = {}

        def add(a, b, length=None):
            self._known(a, "lane")
            self._known(b, "lane")
            if a == b:
                raise LaneMapError(f"lane from {a} to itself")
            if (a, b) in lanes:
                raise LaneMapError(f"lane from {a} to {b} is written twice")
            (ax, ay), (bx, by) = self._pose[a], self._pose[b]
            span = math.hypot(bx - ax, by - ay)
            if span <= 0.0:
                raise LaneMapError(f"lane from {a} to {b} has no length")
            lanes[(a, b)] = Lane(a, b, ax, ay, bx, by, (bx - ax) / span, (by - ay) / span,
                                 float(length) if length is not None else span, span)

        for lane in raw:
            add(_waypoint_id(lane["from"]), _waypoint_id(lane["to"]), lane.get("length"))
        for a, b in lanes:
            if (b, a) in lanes:
                raise LaneMapError(f"lanes {a} to {b} and back must be one two_way entry")
        for name, (a, b) in two_way.items():
            self._known(a, f"two_way {name}")
            self._known(b, f"two_way {name}")
            add(a, b)
            add(b, a)
        return lanes

    def _station(self, name: str, raw: dict) -> Station:
        dock = self._known(_waypoint_id(raw["dock"]), f"station {name}")
        queue = tuple((_waypoint_id(a), _waypoint_id(b)) for a, b in raw.get("queue") or [])
        for lane in queue:
            if lane not in self.lanes:
                raise LaneMapError(f"station {name}: no lane from {lane[0]} to {lane[1]}")
        commit = self._known(_waypoint_id(raw.get("commit", raw["dock"])), f"station {name}")
        return Station(name, dock, queue, commit, raw["server"], tuple(raw["arms"]), raw["action"])

    def _build_zones(self, units: dict, two_way: dict, bundle: bool) -> dict:
        """Every waypoint in one unit: as written, else merged by a bundle, else on its own."""
        name_of: dict = {}
        for name, raw in units.items():
            for member in raw:
                waypoint = self._known(_waypoint_id(member), f"unit {name}")
                if waypoint in name_of:
                    raise LaneMapError(f"{waypoint} is in more than one unit")
                name_of[waypoint] = name
        for waypoint in self._pose:
            name_of.setdefault(waypoint, self.waypoint_id(waypoint))
        if bundle:
            for name, (a, b) in two_way.items():
                old = {name_of[a], name_of[b]}
                for waypoint, unit in name_of.items():
                    if unit in old:
                        name_of[waypoint] = name
        zones: dict = {}
        for waypoint, name in name_of.items():
            zones.setdefault(name, set()).add(waypoint)
        return {name: frozenset(members) for name, members in zones.items()}

    def _zone_segments(self, zone: str) -> list[tuple]:
        """The floor of a zone: its waypoints, and its lanes from the point a robot has entered."""
        members = self.zones[zone]
        segments = [(*self._pose[n], *self._pose[n]) for n in members]
        for waypoint in members:
            for lane in self._in.get(waypoint, ()):
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

    def waypoint_pose(self, waypoint) -> tuple[float, float]:
        return self._pose[waypoint]

    def waypoint_id(self, waypoint) -> str:
        return f"c{waypoint[0]}_{waypoint[1]}" if isinstance(waypoint, tuple) else waypoint

    def nearest_waypoint(self, x: float, y: float, waypoints=None, radius: float | None = None):
        """The waypoint whose unit a pose belongs to.

        Standing on a waypoint, that waypoint: within `radius`, half_width by default. On a lane,
        the waypoint it leads to, because driving on a lane needs that waypoint's unit. A true pose
        can use the narrow default, a single noisy report cannot, so what decides from one report
        passes a wider radius. `waypoints` restricts the answer to the nearest of those waypoints.
        """
        if waypoints is not None:
            return min(waypoints, key=lambda c: math.dist((x, y), self._pose[c]))
        waypoint = min(self._pose, key=lambda c: math.dist((x, y), self._pose[c]))
        limit = self.half_width if radius is None else radius
        if math.dist((x, y), self._pose[waypoint]) <= limit:
            return waypoint
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

        Lanes that meet at a waypoint would always count, and then nobody could follow anybody. A
        robot gives up the unit behind it only after driving release_m along the next lane, so
        from the shared waypoint that stretch cannot hold it while another robot holds the first
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
        waypoint = next(iter(shared))
        if one.a == waypoint and other.a == waypoint:  # a fork: either one can be the leader
            return (self._lane_distance(one, release, other, 0.0) < clearance
                    or self._lane_distance(one, 0.0, other, release) < clearance)
        if one.a == waypoint:  # other leads into one
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
        seen = 0  # breaks ties in the heap without comparing waypoints
        heap = [(0.0, seen, start)]
        while heap:
            cost, _, waypoint = heapq.heappop(heap)
            if waypoint == goal:
                path = []
                while waypoint is not None:
                    path.append(waypoint)
                    waypoint = prev[waypoint]
                return path[::-1]
            if cost > best[waypoint]:
                continue
            for lane in self._out.get(waypoint, ()):
                total = cost + lane.length
                if total < best.get(lane.b, math.inf):
                    best[lane.b], prev[lane.b] = total, waypoint
                    seen += 1
                    heapq.heappush(heap, (total, seen, lane.b))
        raise LaneMapError(f"no path along the lanes from {start} to {goal}")


def load_lane_map(path=None, bundle_two_way: bool | None = None) -> LaneMap:
    source = Path(path) if path else DEFAULT_MAP
    text = source.read_text(encoding="utf-8")
    return LaneMap(yaml.safe_load(text), text, bundle_two_way)
