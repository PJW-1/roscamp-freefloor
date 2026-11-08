"""몸이 쓰는 칸과 칸 잠금.

몸이 쓰는 칸 = 원(반지름 + 여유)이 닿는 칸. 경로를 따라 쓸고 지나가는 칸은 구간마다 칸 사각형에서
구간까지 거리가 반지름(+ 칸의 1/8) 이하인 칸이다. 처음에는 칸 1/4 간격 점마다 원을 놓았는데
(그 합과 같거나 더 넓다) tick이 20 ms를 넘어 한 번에 재도록 바꿨다.

CellLocks는 칸 이름 -> 주인 딕셔너리다. 늘릴 칸은 한꺼번에(acquire_all) 잡을 수 있을 때만 잡는다.
읽기 API는 ZoneLocks와 같아서 deadlock.wait_graph, 배차 화면, 측정 도구가 그대로 쓴다.
"""
from __future__ import annotations

import math
from functools import lru_cache


def disk_cells(floor, x: float, y: float, radius: float) -> set:
    """원이 닿는 지도 안의 칸 (i, j)."""
    # one cell wider than the box: a disk that just touches a cell edge takes that cell
    i0, j0 = floor.cell_of(x - radius, y - radius)
    i1, j1 = floor.cell_of(x + radius, y + radius)
    i0, j0, i1, j1 = i0 - 1, j0 - 1, i1 + 1, j1 + 1
    found = set()
    for i in range(i0, i1 + 1):
        for j in range(j0, j1 + 1):
            cell = (i, j)
            if floor.inside(cell) and floor.box_distance(cell, x, y) <= radius:
                found.add(cell)
    return found


def sample_line(points, step: float) -> list:
    """꺾은선 위 점들, 간격 step 이하. 끝점은 모두 든다."""
    if len(points) == 1:
        return [tuple(points[0])]
    samples = [tuple(points[0])]
    for a, b in zip(points, points[1:], strict=False):
        n = max(1, math.ceil(math.dist(a, b) / step))
        samples.extend((a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
                       for k in range(1, n + 1))
    return samples


def segment_cells(floor, a, b) -> set:
    """선분 a-b가 지나는 모든 칸(Amanatides-Woo 격자 걷기). 모서리를 스치는 칸도 든다."""
    gx0, gy0 = ((a[0] - floor.origin[0]) / floor.cell_m, (a[1] - floor.origin[1]) / floor.cell_m)
    gx1, gy1 = ((b[0] - floor.origin[0]) / floor.cell_m, (b[1] - floor.origin[1]) / floor.cell_m)
    i, j = math.floor(gx0), math.floor(gy0)
    end = (math.floor(gx1), math.floor(gy1))
    dx, dy = gx1 - gx0, gy1 - gy0
    step_i, step_j = (1 if dx > 0 else -1), (1 if dy > 0 else -1)
    t_dx = abs(1 / dx) if dx else math.inf
    t_dy = abs(1 / dy) if dy else math.inf
    t_x = ((i + 1 - gx0) if dx > 0 else (gx0 - i)) * t_dx if dx else math.inf
    t_y = ((j + 1 - gy0) if dy > 0 else (gy0 - j)) * t_dy if dy else math.inf
    cells = {(i, j)}
    for _ in range(4 * (abs(end[0] - i) + abs(end[1] - j)) + 4):
        if (i, j) == end or min(t_x, t_y) > 1:
            break
        if abs(t_x - t_y) < 1e-12:  # through a corner: the two side cells are touched too
            cells.add((i + step_i, j))
            cells.add((i, j + step_j))
            i, j = i + step_i, j + step_j
            t_x, t_y = t_x + t_dx, t_y + t_dy
        elif t_x < t_y:
            i, t_x = i + step_i, t_x + t_dx
        else:
            j, t_y = j + step_j, t_y + t_dy
        cells.add((i, j))
    cells.add(end)
    return cells


def line_cells(floor, points) -> set:
    """꺾은선이 지나는 모든 칸."""
    if len(points) == 1:
        return {floor.cell_of(*points[0])}
    cells = set()
    for a, b in zip(points, points[1:], strict=False):
        cells |= segment_cells(floor, a, b)
    return cells


def swept_cells(floor, points, radius: float) -> set:
    """꺾은선을 따라 원이 쓸고 지나가는 칸. 칸 1/4 간격 점마다 반지름 + 칸 1/8 원을 놓던 것과
    같거나 넓다(구간까지 거리가 반지름 + 칸 1/8 이하인 칸)."""
    if len(points) == 1:
        return disk_cells(floor, *points[0], radius)
    reach = radius + floor.cell_m / 8
    cells = set()
    for a, b in zip(points, points[1:], strict=False):
        cells |= _segment_sweep(floor, tuple(a), tuple(b), reach)
    return cells


@lru_cache(maxsize=65536)  # a route's inner segments are swept again every tick
def _segment_sweep(floor, a, b, reach: float) -> frozenset:
    i0, j0 = floor.cell_of(min(a[0], b[0]) - reach, min(a[1], b[1]) - reach)
    i1, j1 = floor.cell_of(max(a[0], b[0]) + reach, max(a[1], b[1]) + reach)
    m = floor.cell_m
    ox, oy = floor.origin
    found = set()
    for i in range(i0 - 1, i1 + 2):
        for j in range(j0 - 1, j1 + 2):
            cell = (i, j)
            if not floor.inside(cell):
                continue
            x0, y0 = ox + i * m, oy + j * m
            if _box_segment_distance(x0, y0, x0 + m, y0 + m, a, b) <= reach:
                found.add(cell)
    return frozenset(found)


def _box_segment_distance(x0, y0, x1, y1, a, b) -> float:
    """축 정렬 사각형과 선분 a-b 사이 거리(겹치면 0)."""
    (ax, ay), (bx, by) = a, b
    # Liang-Barsky clip: does the segment enter the box?
    t0, t1 = 0.0, 1.0
    dx, dy = bx - ax, by - ay
    for p, q in ((-dx, ax - x0), (dx, x1 - ax), (-dy, ay - y0), (dy, y1 - ay)):
        if p == 0:
            if q < 0:
                break
        else:
            t = q / p
            if p < 0:
                t0 = max(t0, t)
            else:
                t1 = min(t1, t)
            if t0 > t1:
                break
    else:
        return 0.0

    def point_box(px, py):
        return math.hypot(max(x0 - px, 0.0, px - x1), max(y0 - py, 0.0, py - y1))

    def point_segment(px, py):
        length2 = dx * dx + dy * dy
        u = 0.0 if length2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
        return math.hypot(ax + u * dx - px, ay + u * dy - py)

    return min(point_box(ax, ay), point_box(bx, by), point_segment(x0, y0), point_segment(x1, y0),
               point_segment(x0, y1), point_segment(x1, y1))


class CellLocks:
    def __init__(self, clock):
        self._clock = clock
        self._holder: dict[str, str] = {}
        self._acquired_at: dict[str, float] = {}
        self._held: dict[str, set[str]] = {}
        self._waiting: dict[str, tuple[str, ...]] = {}  # robot -> cells others hold, sorted

    # ---- writes -------------------------------------------------------------------------

    def acquire_all(self, robot: str, cells) -> bool:
        """모두 비었거나 제 것이면 모두 잡고 True. 하나라도 남의 것이면 하나도 안 잡고 False."""
        cells = set(cells)
        taken = sorted(c for c in cells if self._holder.get(c) not in (None, robot))
        if taken:
            self._waiting[robot] = tuple(taken)
            return False
        self._waiting.pop(robot, None)
        now = self._clock()
        mine = self._held.setdefault(robot, set())
        for cell in cells:
            if cell not in mine:
                self._holder[cell] = robot
                self._acquired_at[cell] = now
                mine.add(cell)
        return True

    def force_acquire(self, robot: str, cells) -> bool:
        """자리 잡기: 줄을 서지 않는다. 모두 비었거나 제 것이어야 한다."""
        cells = set(cells)
        if any(self._holder.get(c) not in (None, robot) for c in cells):
            return False
        waiting = self._waiting.get(robot)
        self.acquire_all(robot, cells)
        if waiting is not None:
            self._waiting[robot] = waiting  # placing does not end a wait
        return True

    def release(self, robot: str, cell: str) -> bool:
        if self._holder.get(cell) != robot:
            return False
        del self._holder[cell]
        del self._acquired_at[cell]
        self._held[robot].discard(cell)
        return True

    def request(self, robot: str, zone: str) -> bool:
        return self.acquire_all(robot, {zone})

    def cancel_request(self, robot: str, zone: str) -> bool:
        waiting = self._waiting.get(robot, ())
        if zone not in waiting:
            return False
        del self._waiting[robot]
        return True

    def cancel_all_requests(self, robot: str) -> None:
        self._waiting.pop(robot, None)

    # ---- reads (ZoneLocks API) ----------------------------------------------------------

    def exists(self, zone: str) -> bool:
        return isinstance(zone, str) and zone.startswith("c")

    def holder(self, zone: str) -> str | None:
        return self._holder.get(zone)

    def queue(self, zone: str) -> tuple[str, ...]:
        return tuple(sorted(r for r, cells in self._waiting.items() if zone in cells))

    def held_by(self, robot: str) -> list[str]:
        return sorted(self._held.get(robot, ()))

    def held_set(self, robot: str) -> set[str]:
        """held_by와 같지만 정렬하지 않은 집합(복사본이 아니다, 바꾸지 말 것)."""
        return self._held.get(robot, set())

    def held_by_others(self, robot: str) -> list[str]:
        return [c for c, holder in self._holder.items() if holder != robot]

    def waiting_for(self, robot: str) -> str | None:
        waiting = self._waiting.get(robot)
        return waiting[0] if waiting else None

    def waited_cells(self, robot: str) -> tuple[str, ...]:
        return self._waiting.get(robot, ())

    def blockers(self, robot: str, zone: str) -> set[str]:
        holder = self._holder.get(zone)
        return {holder} if holder not in (None, robot) else set()

    def holders_around(self, zone: str, robot: str) -> dict[str, str]:
        holder = self._holder.get(zone)
        return {zone: holder} if holder not in (None, robot) else {}

    def snapshot(self) -> dict[str, dict]:
        """잡혔거나 누가 기다리는 칸만. 빈 칸 수백 개를 매 tick 내보내지 않는다."""
        cells = set(self._holder)
        for waiting in self._waiting.values():
            cells.update(waiting)
        return {cell: {"holder": self._holder.get(cell), "queue": self.queue(cell),
                       "acquired_at": self._acquired_at.get(cell)} for cell in sorted(cells)}
