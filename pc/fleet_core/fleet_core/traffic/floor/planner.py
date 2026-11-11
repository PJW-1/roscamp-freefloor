"""자유바닥 경로 계획: 칸 위 A*(8방향) -> 시선 지름길(Theta* 방식) -> 꺾임을 작은 원호로.

로봇 중심은 clear 칸 안에서만 다닌다(clear 칸의 사각형 전체가 벽에서 반지름 + 여유 이상).
경로는 Route다: 점들과 그 위 고정 선 구간(s 범위). 컨트롤러는 고정 선 구간을 한꺼번에 잠근다.

penalty(cell, s)는 다른 로봇의 예약 벌점이다(P3). s는 출발점에서 그 칸까지의 경로 거리라
속도로 나누면 그 칸에 닿는 시각이 된다.
wait(cell, s, (di, dj))는 (기다림 m, 벌점 m)을 준다: 남이 쓰는 칸이 빌 때까지 서 있는 값이다.
기다림은 그 뒤 칸의 s에 더하고(그만큼 늦게 닿는다) 비용에는 WAIT_COST배로 더한다. 벌점은
비용에만 더한다.
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

from fleet_core.traffic.floor.cells import line_cells, swept_cells

WAIT_COST = 0.8  # 서서 기다린 값의 배수: 1보다 작아야 같은 시간을 빙 돌아 때우는 길보다 싸다
LINE_COST = 4.0  # 고정 선 칸을 자유 경로가 지날 때의 거리 배수: 도크 앞을 가로지르지 않게
ARC_RADIUS_M = 0.15  # 꺾임을 둥글리는 원호 반지름(이보다 작게 줄 수 있다)
ARC_STEP_M = 0.05
GAP_DEPTH_M = 0.10  # a start with no clear cell this near is inside the gap of a narrow line
MERGE_M = 0.05  # a report off a narrow line joins the line this far ahead of its foot
JOIN_M = 0.12  # 이만큼 안에 있으면 들어가는 고정 선 위에 선 것으로 보고 그 자리에서 잇는다
STEP_ONTO_CLEAR_M = 0.2  # a start stopped this close to clear floor steps onto it first
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

    def leg(self, start, goal, out_line=None, in_line=None, avoid=frozenset(), penalty=None,
            wait=None):
        """출발 (x, y)에서 goal까지. out_line: 먼저 따를 고정 선(도크에서 나감),
        in_line: 마지막에 따를 고정 선(도크로 들어감). 길이 없으면 None."""
        lines = self.floor.fixed_lines
        narrow = self.floor.narrow_lines
        if out_line in narrow and rest_of_exit(start, lines[out_line].points) is None:
            out_line = None  # off or past a narrow way out: never back into the gap along it
        if in_line in narrow and math.dist(start, lines[in_line].points[-1]) <= 1e-9:
            return Route([tuple(start)])  # on the dock of a narrow line already
        gap = None if out_line else self._gap_line(start)  # the narrow line start stands on
        head = [tuple(start)]
        fixed = []
        if out_line:
            pts = list(lines[out_line].points)
            rest = rest_of_exit(start, pts)  # 나가던 중이면 남은 선만(도크로 되돌아가지 않는다)
            if rest is not None and out_line in narrow:
                rest = self._ahead(start, out_line)  # no step square to the line in a gap
            head = [tuple(start)] + [p for p in (pts if rest is None else rest)
                                     if math.dist(p, start) > 1e-6]
            fixed.append((0.0, polyline_length(head), out_line))
        tail = [tuple(goal)]
        if in_line:
            tail = list(lines[in_line].points)
            rest = None if out_line else _rest_of_line(start, tail)
            on_line = self.floor.clear | self.floor.line_floor_of.get(in_line, frozenset())
            if (rest is not None and gap != in_line
                    and not line_cells(self.floor, [start, rest[0]]) <= on_line):
                rest = None  # the step onto the line would leave clear floor: plan as usual
            if rest is not None:
                # already standing on the approach line (it waited there): go on up the line
                if in_line in narrow:
                    rest = self._ahead(start, in_line)  # every point on the line
                route = Route([tuple(start), *rest])
                route.fixed = [(0.0, route.length, in_line)]
                return route
        if gap:
            # in a gap and not on the line it is to go in by: on along the lines to clear floor
            way = self._gap_exit(start, gap)
            if way is None:
                return None
            head, fixed = way
        middle = self.plan(head[-1], tail[0], avoid, penalty, polyline_length(head), wait)
        if middle is None:
            return None
        points = head[:-1] + middle + tail[1:]
        points = [p for k, p in enumerate(points) if k == 0 or math.dist(p, points[k - 1]) > 1e-9]
        route = Route(points)
        if in_line:
            fixed.append((route.length - lines[in_line].length(), route.length, in_line))
        route.fixed = fixed
        return route

    # ---- narrow lines -------------------------------------------------------------------

    def _gap_line(self, start):
        """Name of the narrow line in whose gap start stands: off clear floor, no clear cell
        within GAP_DEPTH_M (nearer than that it may be a robot passing the mouth with a report
        a few cm off, which steps back onto clear floor as anywhere else) and within JOIN_M of
        the line where it runs over its line floor (deep in a gap the robot is on the line).
        The line whose end it has passed does not count. Else None."""
        floor = self.floor
        a = floor.cell_of(*start)
        if not floor.narrow_lines or a in floor.clear:
            return None
        reach = math.ceil(GAP_DEPTH_M / floor.cell_m) + 1
        if any(cell in floor.clear and math.dist(start, floor.centre(cell)) <= GAP_DEPTH_M
               for cell in ((a[0] + di, a[1] + dj) for di in range(-reach, reach + 1)
                            for dj in range(-reach, reach + 1))):
            return None
        return self._near_line(start)

    def _near_line(self, start):
        """Name of the narrow line start is beside: off clear floor and within JOIN_M of the
        line where it runs over its line floor (the gap and its mouths). Else None."""
        floor = self.floor
        if not floor.narrow_lines or floor.cell_of(*start) in floor.clear:
            return None
        found = []
        for name in sorted(floor.narrow_lines):
            pts = floor.fixed_lines[name].points
            near = _nearest_on(start, pts)
            if near is None or near[0] > JOIN_M or (near[1] == len(pts) - 2 and near[2] >= 1.0):
                continue
            if floor.cell_of(*near[3]) in floor.line_floor_of[name]:
                found.append((near[0], name))
        return min(found)[1] if found else None

    def _ahead(self, start, name) -> list:
        """The way on along narrow line name from start: every point is on the line, so a report
        a few cm off never sends the robot off the centre of the gap. A start off the line joins
        it MERGE_M ahead of its foot (not square to the line: no turn on the spot in the gap)."""
        pts = self.floor.fixed_lines[name].points
        _, k, _, foot = _nearest_on(start, pts)
        rest = [tuple(p) for p in pts[k + 1:]]
        to_end = math.dist(foot, rest[0])
        if math.dist(start, foot) > 1e-9 and to_end > MERGE_M:  # also from before the line
            f = MERGE_M / to_end
            rest.insert(0, (foot[0] + (rest[0][0] - foot[0]) * f,
                            foot[1] + (rest[0][1] - foot[1]) * f))
        return rest

    def _gap_exit(self, start, name):
        """From start in the gap of narrow line name: on along it, and along the narrow lines
        that go on from its end, to the first line end on clear floor. (points from start,
        fixed spans) or None when the lines end off clear floor."""
        floor = self.floor
        head, spans, seen = [tuple(start)], [], set()
        while name is not None:
            seen.add(name)
            s0 = polyline_length(head)
            head = head + [p for p in self._ahead(head[-1], name)
                           if math.dist(p, head[-1]) > 1e-9]
            spans.append((s0, polyline_length(head), name))
            if floor.cell_of(*head[-1]) in floor.clear:
                return head, spans
            name = next((other for other in sorted(floor.narrow_lines - seen) if math.dist(
                floor.fixed_lines[other].points[0], head[-1]) <= 1e-6), None)
        return None

    # ---- free part ----------------------------------------------------------------------

    def plan(self, start, goal, avoid=frozenset(), penalty=None, s0=0.0, wait=None):
        """자유 바닥 위 start에서 goal까지 둥글린 꺾은선, 없으면 None."""
        floor = self.floor
        a, b = floor.cell_of(*start), floor.cell_of(*goal)
        if b not in floor.clear:
            return None
        if a not in floor.clear and self._gap_line(start):
            return None  # in the gap of a narrow line: only that line leads out (leg)
        if a not in floor.clear:
            # stopped a few cm into a wall margin: one straight step back onto clear floor first
            dead: set = set()  # 목표에 닿지 못한 걸음과 이어진 빈 칸
            for step in self._steps_onto_clear(start, avoid):
                cell = floor.cell_of(*step)
                if cell in dead:
                    continue  # 같은 바닥 조각에서 이미 찾았다: 다시 찾아도 길이 없다
                rest = self.plan(step, goal, avoid, penalty, s0 + math.dist(start, step), wait)
                if rest is not None:
                    return [tuple(start), *rest]
                dead |= self._piece(cell, (floor.clear - avoid) | {b})
            return None
        allowed = (floor.clear - avoid) | {a, b}
        if math.dist(start, goal) < 1e-9:
            return [tuple(start)]
        cells = self.astar(a, b, allowed, penalty, s0, wait)
        if cells is None:
            return None
        raw = [tuple(start)] + [floor.centre(c) for c in cells[1:-1]] + [tuple(goal)]
        short = self.shortcut(raw, allowed)
        return self.round_corners(short, allowed)

    @staticmethod
    def _piece(cell, allowed) -> set:
        """cell에서 allowed 칸만 밟아 닿는 칸들(astar와 같은 이웃 규칙: 모서리를 가로지르지 않는다)."""
        seen, todo = {cell}, [cell]
        while todo:
            i, j = todo.pop()
            for di, dj in NEIGHBOURS:
                nxt = (i + di, j + dj)
                if nxt in seen or nxt not in allowed:
                    continue
                if di and dj and ((i + di, j) not in allowed or (i, j + dj) not in allowed):
                    continue
                seen.add(nxt)
                todo.append(nxt)
        return seen

    def _steps_onto_clear(self, start, avoid=frozenset()) -> list:
        """Centres of the clear cells within STEP_ONTO_CLEAR_M that a straight step over free
        floor reaches without running over avoided cells, nearest first; none when the start
        is not on free floor. From the mouth of a gap only the clear cells its narrow line
        runs over (within a cell of the line)."""
        floor = self.floor
        a = floor.cell_of(*start)
        if a not in floor.free:
            return []
        reach = math.ceil(STEP_ONTO_CLEAR_M / floor.cell_m)
        walkable = (floor.free - avoid) | {a}  # the start cell may be held by a close robot
        mouth = self._near_line(start)  # in the mouth of a gap: only along the narrow line
        line = floor.fixed_lines[mouth].points if mouth else None
        steps = []
        for di in range(-reach, reach + 1):
            for dj in range(-reach, reach + 1):
                cell = (a[0] + di, a[1] + dj)
                if cell not in floor.clear or cell in avoid:
                    continue
                xy = floor.centre(cell)
                if line and _nearest_on(xy, line)[0] > floor.cell_m:
                    continue  # slantwise past the corner of what flanks the gap
                d = math.dist(start, xy)
                if d <= STEP_ONTO_CLEAR_M and line_cells(floor, [tuple(start), xy]) <= walkable:
                    steps.append((d, xy))
        return [xy for _, xy in sorted(steps)]

    def astar(self, a, b, allowed, penalty=None, s0=0.0, wait=None):
        floor = self.floor
        step = floor.cell_m
        goal_xy = floor.centre(b)
        best = {a: 0.0}
        came = {}
        heap = [(math.dist(floor.centre(a), goal_xy), 0.0, 0.0, 0.0, a)]
        while heap:
            _, g, dist, late, cell = heapq.heappop(heap)  # late: 여기까지 기다린 값(m)
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
                held = 0.0
                if wait is not None:
                    held, flat = wait(nxt, s0 + dist + late + d, (di, dj))
                    cost += held * WAIT_COST + flat
                if cost < best.get(nxt, math.inf) - 1e-12:
                    best[nxt] = cost
                    came[nxt] = cell
                    heapq.heappush(heap, (cost + math.dist(floor.centre(nxt), goal_xy), cost,
                                          dist + d, late + held, nxt))
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


def rest_of_exit(point, line):
    """point가 line(꺾은선)에서 JOIN_M 안이면 가장 가까운 구간 위 발(수직으로 내린 점)부터 남은 점들.
    멀거나 끝을 지났으면 None. 접히거나 겹친 선은 가장 가까운 구간을 고르므로 맞지 않는다."""
    best = _nearest_on(point, line)
    if best is None or best[0] > JOIN_M:
        return None
    _, k, u, foot = best
    if k == len(line) - 2 and u >= 1.0:
        return None  # 선 끝을 지났다
    rest = [tuple(p) for p in line[k + 1:]]
    if math.dist(foot, rest[0]) > 1e-9:
        rest.insert(0, foot)  # 수직으로 붙어 선 그대로(모서리를 안 자름)
    return rest


def _nearest_on(point, line):
    """(거리, 구간 번호, 구간 위 위치 u(자르지 않은 값), 발)로 point에서 가장 가까운 구간. 없으면 None."""
    best = None
    for k, ((ax, ay), (bx, by)) in enumerate(zip(line, line[1:], strict=False)):
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy
        if length2 < 1e-12:
            continue
        u = ((point[0] - ax) * dx + (point[1] - ay) * dy) / length2
        uc = min(1.0, max(0.0, u))
        d = math.dist(point, (ax + uc * dx, ay + uc * dy))
        if best is None or d < best[0] - 1e-12:
            best = (d, k, u, (ax + uc * dx, ay + uc * dy))
    return best


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
