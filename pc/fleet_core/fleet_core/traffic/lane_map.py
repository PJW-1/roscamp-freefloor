"""코스 맵(지점, 방향 레인, 잠금 단위, 스테이션, 대기 지점)을 YAML에서 읽는다.

로봇은 레인 방향대로 지점에서 지점으로 곧게 달린다. 실제 맵이 아직 정해지지 않아
좌표는 맵 파일에만 둔다. 새 맵은 새 데이터일 뿐이다.

잠금 단위는 한 로봇이 한 번에 쥐는 것이고, 나머지 코드에서는 구역이라 부른다.
지점은 정확히 한 단위에 속하고, 레인은 끝 지점의 단위에 속한다.
  - 한 지점에서 끝나는 레인들은 함께 쓸 수 없어 잠금을 같이 쓰고, 그 대기열이 합류 FIFO다.
  - 양방향 쌍은 묶으면 잠금 하나라서 두 로봇이 마주 달릴 수 없다.
  - 다른 단위인데 두 로봇이 지나기에 너무 가까운 레인은 conflict_pairs()가 찾는다.

지점은 레인 그래프의 꼭짓점이다. 노드라는 말은 ROS용으로 남겨 둔다.
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from pathlib import Path

import yaml

DEFAULT_MAP = Path(__file__).parents[1] / "data" / "lane_map.yaml"


class LaneMapError(ValueError):
    """맵 파일이 맞지 않거나 요청한 경로가 없다."""


@dataclass(frozen=True)
class Station:
    id: str
    dock: object  # 지점
    queue: tuple  # 도크 앞에 줄 서는 (from, to) 레인
    commit: object  # 지점: 이 단위를 쥔 로봇은 더는 다른 곳으로 보낼 수 없다
    server: str  # 이 스테이션 액션의 서버
    arms: tuple[str, ...]  # 그 서버 뒤에서 일하는 팔
    action: str  # fill 또는 swap


@dataclass(frozen=True)
class Lane:
    a: object  # 시작 지점
    b: object  # 끝 지점
    ax: float
    ay: float
    bx: float
    by: float
    ux: float  # a에서 b로의 단위 벡터
    uy: float
    length: float  # 경로 계획 비용, 파일에 없으면 직선 거리
    span: float  # 직선 거리


def _waypoint_id(raw):
    """지점 이름은 문자열, 또는 변환된 맵에서는 [col, row]."""
    if isinstance(raw, (list, tuple)):
        return tuple(int(v) for v in raw)
    return str(raw)


def _point_segment(x, y, ax, ay, bx, by) -> float:
    vx, vy = bx - ax, by - ay
    denom = vx * vx + vy * vy
    t = max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / denom)) if denom else 0.0
    return math.hypot(x - (ax + t * vx), y - (ay + t * vy))


def _segment_distance(a1, a2, b1, b2) -> float:
    """두 선분 사이 최단 거리. 선분은 (x, y) 두 개."""
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
        """지점마다 단위 하나: 적힌 대로, 아니면 묶음으로 합치고, 아니면 혼자."""
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
        """구역의 바닥: 지점들과, 로봇이 들어선 것으로 치는 점부터의 레인."""
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
        """s = (p - p_a)·u는 레인을 따라 간 거리, e = |(p - p_a) - s·u|는 벗어난 거리."""
        lane = self.lanes[(a, b)]
        rx, ry = x - lane.ax, y - lane.ay
        s = rx * lane.ux + ry * lane.uy
        return s, math.hypot(rx - s * lane.ux, ry - s * lane.uy)

    def waypoint_pose(self, waypoint) -> tuple[float, float]:
        return self._pose[waypoint]

    def bounds(self) -> tuple[float, float, float, float]:
        """모든 웨이포인트의 min_x, min_y, max_x, max_y."""
        xs = [x for x, _ in self._pose.values()]
        ys = [y for _, y in self._pose.values()]
        return min(xs), min(ys), max(xs), max(ys)

    def waypoint_id(self, waypoint) -> str:
        return f"c{waypoint[0]}_{waypoint[1]}" if isinstance(waypoint, tuple) else waypoint

    def nearest_waypoint(self, x: float, y: float, waypoints=None, radius: float | None = None):
        """자세가 속한 단위의 지점.

        `radius`(기본 half_width) 안이면 그 지점이다. 레인 위면 레인의 끝 지점이다.
        레인을 달리려면 그 지점의 단위가 필요하기 때문이다. 보고 하나는 오차가 있으니
        보고 하나로 판단할 때는 더 넓은 반경을 넘긴다. `waypoints`를 주면 그중 가장 가까운 것.
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
        """자세에서 구역 바닥까지 거리. 안에 있으면 0."""
        nearest = min(_point_segment(x, y, *segment) for segment in self._segments[zone])
        return max(0.0, nearest - self.half_width)

    def shortest_exit_lane(self) -> float:
        """단위를 떠나는 가장 짧은 레인 길이. 해제 거리가 이 안에 들어가야 한다."""
        return min((lane.span for (a, b), lane in self.lanes.items()
                    if self.zone_of[a] != self.zone_of[b]), default=math.inf)

    # ---- conflict pairs -----------------------------------------------------------------------

    def conflict_pairs(self, clearance_m: float, release_m: float) -> set[frozenset]:
        """함께 쥐면 안 되는 단위 쌍: 레인이 2r보다 가깝게 지난다.

        지점에서 만나는 레인을 다 세면 아무도 앞 로봇을 따라갈 수 없다. 로봇은 다음 레인을
        release_m 달린 뒤에야 뒤 단위를 놓으므로, 공유 지점부터 그 구간은 빼고 나머지만 센다.
        양방향 쌍의 두 방향은 뺀다. 함께 쥐는 것은 묶음이 맡는다.
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
            return False  # 양방향 쌍
        shared = {one.a, one.b} & {other.a, other.b}
        if not shared:
            return self._lane_distance(one, 0.0, other, 0.0) < clearance
        waypoint = next(iter(shared))
        if one.a == waypoint and other.a == waypoint:  # 갈림길: 어느 쪽이든 앞설 수 있다
            return (self._lane_distance(one, release, other, 0.0) < clearance
                    or self._lane_distance(one, 0.0, other, release) < clearance)
        if one.a == waypoint:  # other가 one으로 이어진다
            return self._lane_distance(one, release, other, 0.0) < clearance
        return self._lane_distance(one, 0.0, other, release) < clearance

    @staticmethod
    def _lane_distance(one: Lane, skip_one: float, other: Lane, skip_other: float) -> float:
        """두 레인 사이 거리. 각 레인의 앞 몇 미터는 뺀다."""
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
        """레인을 따라가는 최단 경로(미터 기준). 양 끝 지점을 포함한다."""
        best = {start: 0.0}
        prev: dict = {start: None}
        seen = 0  # 힙에서 지점을 비교하지 않고 동점을 가른다
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
