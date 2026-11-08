"""관제 화면(시뮬레이션 전용): 관제가 보는 함대를 마커 명세로 바꾼다. rclpy를 쓰지 않는다.

레인과 지점, 로봇(몸, 방향, 상태 글), 쥔 구역(소유자 색 굵은 선), 막힌 레인(빨강)을 그린다.
구역 id는 맵의 잠금 단위 이름이다. 지점 하나(W, N, ...), 양방향 묶음(AVENUE),
또는 맵 `units:`의 이름이다. 구역의 바닥은 그 지점들과 그 지점에서 끝나는 레인이다.
교차 쌍은 따로 된 id가 없고 구역 두 개가 서로 잠글 뿐이라 각 구역으로 그린다.
"""
from __future__ import annotations

import re
from dataclasses import dataclass

STALE_S = 3.0  # 이만큼 자세가 없으면 회색

GRAY = (0.55, 0.55, 0.55, 1.0)
RED = (1.0, 0.1, 0.1, 1.0)
AMBER = (1.0, 0.75, 0.0, 1.0)
LABEL = (0.95, 0.95, 0.95, 1.0)
ROBOT_COLORS = {
    "pinky1": (0.2, 0.45, 1.0, 1.0),
    "pinky2": (0.1, 0.8, 0.25, 1.0),
    "pinky3": (1.0, 0.6, 0.0, 1.0),
}
_SPARE = ((0.8, 0.3, 0.9, 1.0), (0.0, 0.8, 0.8, 1.0), (0.9, 0.9, 0.2, 1.0))

_LANE_RE = re.compile(r"^(\(.*?\)|\S+) to (\(.*?\)|\S+)")


@dataclass(frozen=True)
class Spec:
    """마커 하나. kind는 visualization_msgs/Marker 형식 이름, pose는 (x, y, z, yaw)."""
    ns: str
    id: int
    kind: str
    pose: tuple = (0.0, 0.0, 0.0, 0.0)
    scale: tuple = (1.0, 1.0, 1.0)
    color: tuple = LABEL
    points: tuple = ()  # (x, y) 목록. 높이는 pose의 z
    text: str = ""


def robot_color(robot: str) -> tuple:
    if robot in ROBOT_COLORS:
        return ROBOT_COLORS[robot]
    return _SPARE[sum(map(ord, robot)) % len(_SPARE)]


def _faded(color: tuple, alpha: float) -> tuple:
    return (*color[:3], alpha)


def lane_specs(lane_map) -> list[Spec]:
    """레인은 얇은 회색 선(양방향 쌍은 하나), 지점은 점과 이름."""
    points = []
    seen = set()
    for (a, b), lane in lane_map.lanes.items():
        if frozenset((a, b)) in seen:
            continue
        seen.add(frozenset((a, b)))
        points += [(lane.ax, lane.ay), (lane.bx, lane.by)]
    waypoints = sorted(lane_map.zone_of, key=lane_map.waypoint_id)
    poses = [lane_map.waypoint_pose(w) for w in waypoints]
    specs = [
        Spec("lanes", 0, "LINE_LIST", (0.0, 0.0, 0.005, 0.0), (0.01, 0.0, 0.0),
             _faded(GRAY, 0.8), tuple(points)),
        Spec("waypoints", 0, "SPHERE_LIST", (0.0, 0.0, 0.01, 0.0), (0.04, 0.04, 0.04),
             GRAY, tuple(poses)),
    ]
    for i, (w, (x, y)) in enumerate(zip(waypoints, poses, strict=True)):
        specs.append(Spec("waypoint_labels", i, "TEXT", (x + 0.06, y + 0.06, 0.05, 0.0),
                          (0.0, 0.0, 0.06), LABEL, text=lane_map.waypoint_id(w)))
    return specs


def zone_geometry(lane_map, zone_id: str):
    """구역의 (지점 좌표 목록, 레인 선분 목록). 모르는 id는 None."""
    if zone_id not in lane_map.zones:
        return None
    members = sorted(lane_map.zones[zone_id], key=lane_map.waypoint_id)
    points = [lane_map.waypoint_pose(w) for w in members]
    keys = lane_map.lanes_of_zone(zone_id)
    lines = [((lane.ax, lane.ay), (lane.bx, lane.by))
             for key, lane in sorted(lane_map.lanes.items(),
                                     key=lambda item: tuple(map(lane_map.waypoint_id, item[0])))
             if key in keys]
    return points, lines


def robot_label(status, stale: bool) -> tuple[str, tuple]:
    """pinky1  MOVING  TO_WORK R-003 글과 색. 고장, BLOCKED는 빨강, RECOVERING은 호박색."""
    fault = getattr(status, "fault", "") or "NONE"
    state = getattr(status, "fsm_state", "")
    parts = [status.robot_id, state or "?",
             " ".join(p for p in (getattr(status, "task_type", ""),
                                  getattr(status, "order_id", "")) if p)]
    if fault != "NONE":
        reason = getattr(status, "fault_reason", "")
        parts.append(f"{fault}:{reason}" if reason else fault)
    if stale:
        parts.append("STALE")
    text = "  ".join(p for p in parts if p)
    if fault != "NONE" or state == "BLOCKED":
        return text, RED
    if state == "RECOVERING":
        return text, AMBER
    return text, GRAY if stale else robot_color(status.robot_id)


@dataclass
class _NoStatus:
    robot_id: str
    fsm_state: str = ""


class FleetView:
    """받은 상태를 모아 두고 그릴 때마다 명세 목록을 만든다. 시각은 초(호출하는 쪽 시계)."""

    def __init__(self, lane_map):
        self.lane_map = lane_map
        self._lanes = lane_specs(lane_map)
        self._by_name = {str(w): w for w in lane_map.zone_of}
        self._status: dict = {}
        self._pose: dict = {}  # 로봇 -> (x, y, yaw, 시각)
        self._zones: list = []
        self._blocked: dict = {}  # 로봇 -> 막힌 레인 (a, b)

    def set_statuses(self, statuses) -> None:
        self._status = {s.robot_id: s for s in statuses}

    def set_pose(self, robot: str, x: float, y: float, yaw: float, now: float) -> None:
        self._pose[robot] = (x, y, yaw, now)

    def set_zones(self, zones) -> None:
        self._zones = list(zones)

    def on_event(self, code: str, source: str, detail: str) -> None:
        """LANE_BLOCKED는 '{a} to {b} blocked ...', LANE_CLEARED는 '{a} to {b} ...'."""
        if code not in ("LANE_BLOCKED", "LANE_CLEARED"):
            return
        lane = self._parse_lane(detail)
        if lane is None:
            return
        if code == "LANE_BLOCKED":
            self._blocked[source] = lane
        elif self._blocked.get(source) == lane:
            del self._blocked[source]

    def _parse_lane(self, detail: str):
        found = _LANE_RE.match(detail or "")
        if not found:
            return None
        key = (self._by_name.get(found.group(1)), self._by_name.get(found.group(2)))
        return key if key in self.lane_map.lanes else None

    def blocked_lanes(self) -> set:
        return set(self._blocked.values())

    def specs(self, now: float) -> list[Spec]:
        return [*self._lanes, *self._zone_specs(), *self._blocked_specs(),
                *self._robot_specs(now)]

    def _zone_specs(self) -> list[Spec]:
        specs = []
        for i, zone in enumerate(self._zones):
            geometry = zone_geometry(self.lane_map, zone.zone_id)
            if geometry is None:
                continue
            points, lines = geometry
            if zone.holder:
                color = _faded(robot_color(zone.holder), 0.7)
                specs.append(Spec("zone_lanes", i, "LINE_LIST", (0.0, 0.0, 0.01, 0.0),
                                  (0.04, 0.0, 0.0), color,
                                  tuple(p for line in lines for p in line)))
                specs.append(Spec("zone_points", i, "SPHERE_LIST", (0.0, 0.0, 0.015, 0.0),
                                  (0.09, 0.09, 0.09), color, tuple(points)))
            if zone.queue:
                x, y = points[0]
                specs.append(Spec("zone_queue", i, "TEXT", (x, y - 0.08, 0.05, 0.0),
                                  (0.0, 0.0, 0.05), LABEL,
                                  text=f"{zone.zone_id} wait: {' '.join(zone.queue)}"))
        return specs

    def _blocked_specs(self) -> list[Spec]:
        points = []
        for a, b in sorted(self.blocked_lanes(), key=str):
            lane = self.lane_map.lanes[(a, b)]
            points += [(lane.ax, lane.ay), (lane.bx, lane.by)]
        if not points:
            return []
        return [Spec("blocked", 0, "LINE_LIST", (0.0, 0.0, 0.02, 0.0), (0.05, 0.0, 0.0),
                     RED, tuple(points))]

    def _robot_specs(self, now: float) -> list[Spec]:
        specs = []
        for i, robot in enumerate(sorted(self._pose)):
            x, y, yaw, seen = self._pose[robot]
            status = self._status.get(robot) or _NoStatus(robot)
            stale = (now - seen > STALE_S
                     or getattr(status, "fault", "") == "COMM_LOST")
            body = GRAY if stale else robot_color(robot)
            text, color = robot_label(status, stale)
            specs += [
                Spec("robot_body", i, "CYLINDER", (x, y, 0.04, yaw), (0.12, 0.12, 0.08), body),
                Spec("robot_heading", i, "ARROW", (x, y, 0.09, yaw), (0.15, 0.025, 0.025),
                     body),
                Spec("robot_label", i, "TEXT", (x, y, 0.3, 0.0), (0.0, 0.0, 0.07), color,
                     text=text),
            ]
        return specs
