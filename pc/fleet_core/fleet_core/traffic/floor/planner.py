"""자유바닥 경로 계획: 칸 위 A*(8방향) -> 시선 지름길(Theta* 방식) -> 꺾임을 작은 원호로.

로봇 중심은 clear 칸 안에서만 다닌다(clear 칸의 사각형 전체가 벽에서 반지름 + 여유 이상).
경로는 Route다: 점들과 그 위 고정 선 구간(s 범위). 컨트롤러는 고정 선 구간을 한꺼번에 잠근다.

penalty(cell, s)는 다른 로봇의 예약 벌점이다(P3). s는 출발점에서 그 칸까지의 경로 거리라
속도로 나누면 그 칸에 닿는 시각이 된다.
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

from fleet_core.traffic.floor.cells import line_cells, swept_cells

LINE_COST = 4.0  # 고정 선 칸을 자유 경로가 지날 때의 거리 배수: 도크 앞을 가로지르지 않게
ARC_RADIUS_M = 0.15  # 꺾임을 둥글리는 원호 반지름(이보다 작게 줄 수 있다)
ARC_STEP_M = 0.05
JOIN_M = 0.12  # 이만큼 안에 있으면 들어가는 고정 선 위에 선 것으로 보고 그 자리에서 잇는다
STRAIGHT = 1e-3  # 이보다 덜 꺾이면 곧은 것으로 친다(rad)
NEIGHBOURS = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]


def polyline_length(points) -> float:
    return sum(math.dist(a, b) for a, b in zip(points, points[1:], strict=False))


@dataclass
class Route:
    points: list  # [(x, y), ...]
    fixed: list = field(default_factory=list)  # [(s0, s1, line name)]

    def __post_init__(self):
        self.cum = [0.0]
        for a, b in zip(self.points, self.points[1:], strict=False):
            self.cum.append(self.cum[-1] + math.dist(a, b))

    @property
    def length(self) -> float:
        return self.cum[-1]

    def at(self, s: float) -> tuple[float, float]:
        s = min(max(s, 0.0), self.length)
        for k in range(1, len(self.points)):
            if self.cum[k] >= s - 1e-12:
                a, b = self.points[k - 1], self.points[k]
                span = self.cum[k] - self.cum[k - 1]
                f = (s - self.cum[k - 1]) / span if span > 0 else 1.0
                return (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)
        return self.points[-1]

    def between(self, s0: float, s1: float) -> list:
        """s0에서 s1까지의 꺾은선(양 끝 포함)."""
        s0, s1 = max(0.0, s0), min(self.length, s1)
        out = [self.at(s0)]
        for k, s in enumerate(self.cum):
            if s0 < s < s1:
                out.append(self.points[k])
        end = self.at(s1)
        if math.dist(out[-1], end) > 1e-9 or len(out) == 1:
            out.append(end)
        return out

    def project(self, x: float, y: float, s_from: float = 0.0, s_to: float | None = None):
        """(x, y)에 가장 가까운 경로 위 s를 [s_from, s_to] 안에서. (s, 거리)."""
        s_to = self.length if s_to is None else min(s_to, self.length)
        best = (s_from, math.dist((x, y), self.at(s_from)))
        for k in range(1, len(self.points)):
            c0, c1 = self.cum[k - 1], self.cum[k]
            if c1 < s_from or c0 > s_to or c1 <= c0:
                continue
            (ax, ay), (bx, by) = self.points[k - 1], self.points[k]
            dx, dy = bx - ax, by - ay
            f = ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)
            s = min(max(c0 + f * (c1 - c0), s_from, c0), s_to, c1)
            d = math.dist((x, y), self.at(s))
            if d < best[1] - 1e-12:
                best = (s, d)
        return best

    def fixed_at(self, s: float):
        """s가 든 고정 선 구간 (s0, s1, 이름), 없으면 None."""
        for span in self.fixed:
            if span[0] - 1e-9 <= s <= span[1] + 1e-9:
                return span
        return None


class FloorPlanner:
    def __init__(self, floor):
        self.floor = floor
        body = floor.robot_radius_m + floor.cell_margin_m
        self.line_cells = set()
        for line in floor.fixed_lines.values():
            self.line_cells |= swept_cells(floor, list(line.points), body) & floor.clear

    # ---- whole legs ---------------------------------------------------------------------

    def leg(self, start, goal, out_line=None, in_line=None, avoid=frozenset(), penalty=None):
        """출발 (x, y)에서 goal까지. out_line: 먼저 따를 고정 선(도크에서 나감),
        in_line: 마지막에 따를 고정 선(도크로 들어감). 길이 없으면 None."""
        lines = self.floor.fixed_lines
        head = [tuple(start)]
        fixed = []
        if out_line:
            pts = list(lines[out_line].points)
            head = [tuple(start), *pts[1:]] if math.dist(start, pts[0]) < 1e-6 else \
                [tuple(start), *pts]
            fixed.append((0.0, polyline_length(head), out_line))
        tail = [tuple(goal)]
        if in_line:
            tail = list(lines[in_line].points)
            rest = None if out_line else _rest_of_line(start, tail)
            if rest is not None and not line_cells(self.floor, [start, rest[0]]) <= \
                    self.floor.clear:
                rest = None  # the step onto the line would leave clear floor: plan as usual
            if rest is not None:
                # already standing on the approach line (it waited there): go on up the line
                route = Route([tuple(start), *rest])
                route.fixed = [(0.0, route.length, in_line)]
                return route
        middle = self.plan(head[-1], tail[0], avoid, penalty, polyline_length(head))
        if middle is None:
            return None
        points = head[:-1] + middle + tail[1:]
        points = [p for k, p in enumerate(points) if k == 0 or math.dist(p, points[k - 1]) > 1e-9]
        route = Route(points)
        if in_line:
            fixed.append((route.length - lines[in_line].length(), route.length, in_line))
        route.fixed = fixed
        return route

    # ---- free part ----------------------------------------------------------------------

    def plan(self, start, goal, avoid=frozenset(), penalty=None, s0=0.0):
        """자유 바닥 위 start에서 goal까지 둥글린 꺾은선, 없으면 None."""
        floor = self.floor
        a, b = floor.cell_of(*start), floor.cell_of(*goal)
        allowed = (floor.clear - avoid) | {a, b}
        if a not in floor.clear or b not in floor.clear:
            return None
        if math.dist(start, goal) < 1e-9:
            return [tuple(start)]
        cells = self.astar(a, b, allowed, penalty, s0)
        if cells is None:
            return None
        raw = [tuple(start)] + [floor.centre(c) for c in cells[1:-1]] + [tuple(goal)]
        short = self.shortcut(raw, allowed)
        return self.round_corners(short, allowed)

    def astar(self, a, b, allowed, penalty=None, s0=0.0):
        floor = self.floor
        step = floor.cell_m
        goal_xy = floor.centre(b)
        best = {a: 0.0}
        came = {}
        heap = [(math.dist(floor.centre(a), goal_xy), 0.0, 0.0, a)]
        while heap:
            _, g, dist, cell = heapq.heappop(heap)
            if cell == b:
                path = [cell]
                while cell in came:
                    cell = came[cell]
                    path.append(cell)
                return path[::-1]
            if g > best.get(cell, math.inf) + 1e-12:
                continue
            for di, dj in NEIGHBOURS:
                nxt = (cell[0] + di, cell[1] + dj)
                if nxt not in allowed:
                    continue
                if di and dj and ((cell[0] + di, cell[1]) not in allowed
                                  or (cell[0], cell[1] + dj) not in allowed):
                    continue  # no corner cutting
                d = step * (math.sqrt(2) if di and dj else 1.0)
                cost = g + (d * LINE_COST if nxt in self.line_cells and nxt not in (a, b) else d)
                if penalty is not None:
                    cost += penalty(nxt, s0 + dist + d)  # reached after driving dist + d
                if cost < best.get(nxt, math.inf) - 1e-12:
                    best[nxt] = cost
                    came[nxt] = cell
                    heapq.heappush(heap, (cost + math.dist(floor.centre(nxt), goal_xy), cost,
                                          dist + d, nxt))
        return None

    def visible(self, p, q, allowed) -> bool:
        """p에서 q까지 곧게 가도 중심이 지나는 칸이 모두 allowed인가(스치는 칸 포함)."""
        return line_cells(self.floor, [p, q]) <= allowed

    def shortcut(self, points, allowed) -> list:
        """앞에서부터 보이는 가장 먼 점으로 곧게 잇는다."""
        out = [points[0]]
        k = 0
        while k < len(points) - 1:
            far = k + 1
            for m in range(len(points) - 1, k + 1, -1):
                if self.visible(points[k], points[m], allowed) and not self._enters_lines(
                        points[k], points[m], points[k:m + 1]):
                    far = m
                    break
            out.append(points[far])
            k = far
        return out

    def _enters_lines(self, p, q, original, replacement=None) -> bool:
        """p-q(또는 replacement 꺾은선)가 원래 경로가 안 지나던 고정 선 칸을 지나는가."""
        if not self.line_cells:
            return False
        new = line_cells(self.floor, replacement or [p, q]) & self.line_cells
        return bool(new - line_cells(self.floor, original))

    def round_corners(self, points, allowed) -> list:
        """꺾인 점마다 두 선분에 접하는 원호로 바꾼다. 원호가 칸 밖으로 나가면 꺾인 채 둔다."""
        if len(points) < 3:
            return list(points)
        out = [points[0]]
        for k in range(1, len(points) - 1):
            a, b, c = out[-1], points[k], points[k + 1]
            arc = self._arc(a, b, c)
            if (arc and line_cells(self.floor, [a, *arc, c]) <= allowed
                    and not self._enters_lines(a, c, [a, b, c], [a, *arc, c])):
                out.extend(arc)
            else:
                out.append(b)
        out.append(points[-1])
        return out

    @staticmethod
    def _arc(a, b, c):
        ux, uy = b[0] - a[0], b[1] - a[1]
        vx, vy = c[0] - b[0], c[1] - b[1]
        lu, lv = math.hypot(ux, uy), math.hypot(vx, vy)
        if lu < 1e-9 or lv < 1e-9:
            return None
        ux, uy, vx, vy = ux / lu, uy / lu, vx / lv, vy / lv
        turn = math.atan2(ux * vy - uy * vx, ux * vx + uy * vy)
        if abs(turn) < STRAIGHT or abs(turn) > math.pi - 1e-3:
            return None
        half = math.tan(abs(turn) / 2)
        d = min(ARC_RADIUS_M * half, 0.45 * lu, 0.45 * lv)
        radius = d / half
        p0 = (b[0] - ux * d, b[1] - uy * d)
        side = 1.0 if turn > 0 else -1.0
        cx, cy = p0[0] - uy * radius * side, p0[1] + ux * radius * side
        start = math.atan2(p0[1] - cy, p0[0] - cx)
        n = max(2, math.ceil(radius * abs(turn) / ARC_STEP_M))
        return [(cx + radius * math.cos(start + side * abs(turn) * k / n),
                 cy + radius * math.sin(start + side * abs(turn) * k / n)) for k in range(n + 1)]


def _rest_of_line(point, line):
    """point가 line 첫 점을 지난 구간에서 JOIN_M 안이면 그 구간 끝부터의 점들, 아니면 None."""
    px, py = point
    for k, ((ax, ay), (bx, by)) in enumerate(zip(line, line[1:], strict=False)):
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy
        if length2 < 1e-12:
            continue
        u = ((px - ax) * dx + (py - ay) * dy) / length2
        if u <= 0.0 or u >= 1.0:
            continue
        if abs((px - ax) * dy - (py - ay) * dx) / math.sqrt(length2) <= JOIN_M:
            return [tuple(p) for p in line[k + 1:]]
    return None
