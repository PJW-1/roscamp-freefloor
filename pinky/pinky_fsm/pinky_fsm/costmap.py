"""local costmap(OccupancyGrid)에서 경로 앞 물체를 찾는다. ROS 없이 돈다.

Nav2 costmap은 0~255 비용을 OccupancyGrid의 -1~100으로 옮겨 발행한다. 99는 로봇 중심이
그 칸에 오면 몸이 장애물에 닿는다는 뜻(INSCRIBED), 100은 장애물 자체(LETHAL)다.
"""
from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

INSCRIBED = 99  # 이 값 이상이면 그 점을 지날 수 없다


@dataclass(frozen=True)
class Grid:
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    width: int
    height: int
    data: Sequence[int]

    def cost_at(self, x: float, y: float) -> int | None:
        """그 점의 값. 격자 밖이면 None."""
        dx, dy = x - self.origin_x, y - self.origin_y
        c, s = math.cos(-self.origin_yaw), math.sin(-self.origin_yaw)
        gx, gy = c * dx - s * dy, s * dx + c * dy
        i, j = math.floor(gx / self.resolution), math.floor(gy / self.resolution)
        if not (0 <= i < self.width and 0 <= j < self.height):
            return None
        return int(self.data[j * self.width + i])


def to_frame(points: Sequence[Sequence[float]],
             tf: tuple[float, float, float]) -> list[tuple[float, float]]:
    """점을 다른 틀로 옮긴다. tf는 새 틀에서 본 옛 틀의 자세 (x, y, yaw)."""
    tx, ty, yaw = tf
    c, s = math.cos(yaw), math.sin(yaw)
    return [(tx + c * px - s * py, ty + s * px + c * py) for px, py in points]


def obstacle_on(grid: Grid, points: Sequence[Sequence[float]],
                threshold: int = INSCRIBED) -> bool:
    """점들 중 하나라도 threshold 이상의 칸 위에 있으면 True. 모르는 칸(-1)은 물체가 아니다."""
    for x, y in points:
        cost = grid.cost_at(x, y)
        if cost is not None and cost >= threshold:
            return True
    return False


def invert_frame(tf: tuple[float, float, float]) -> tuple[float, float, float]:
    """to_frame에 쓰는 자세의 역. 옮긴 점을 원래 틀로 되돌릴 때 쓴다."""
    tx, ty, yaw = tf
    c, s = math.cos(-yaw), math.sin(-yaw)
    return (-(c * tx - s * ty), -(s * tx + c * ty), -yaw)


def _cell_of(grid: Grid, x: float, y: float) -> tuple[int, int] | None:
    dx, dy = x - grid.origin_x, y - grid.origin_y
    c, s = math.cos(-grid.origin_yaw), math.sin(-grid.origin_yaw)
    i = math.floor((c * dx - s * dy) / grid.resolution)
    j = math.floor((s * dx + c * dy) / grid.resolution)
    return (i, j) if 0 <= i < grid.width and 0 <= j < grid.height else None


def _centre_of(grid: Grid, i: int, j: int) -> tuple[float, float]:
    gx, gy = (i + 0.5) * grid.resolution, (j + 0.5) * grid.resolution
    c, s = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
    return grid.origin_x + c * gx - s * gy, grid.origin_y + s * gx + c * gy


def obstacle_location(grid: Grid, points: Sequence[Sequence[float]],
                      threshold: int = INSCRIBED, seed_reach: float = 0.15,
                      window: float = 0.6) -> tuple[float, float, float] | None:
    """경로 위 첫 막힌 점 가까이의 물체를 (x, y, 반지름)으로 돌려준다. 격자 틀 좌표.

    막힌 점 근처(seed_reach m)의 LETHAL 칸에서 8방향으로 이어진 칸을 window m 안에서 모아
    중심과 반지름(중심에서 칸 모서리까지 최대 거리)을 낸다. LETHAL 칸이 없으면(부풀림만 있으면)
    그 점과 반지름 0(함대 기본값)을 낸다. 막힌 점이 없으면 None.
    """
    hit = None
    for x, y in points:
        cost = grid.cost_at(x, y)
        if cost is not None and cost >= threshold:
            hit = (x, y)
            break
    if hit is None:
        return None
    cell = _cell_of(grid, *hit)
    reach = math.ceil(seed_reach / grid.resolution)
    seed, best = None, None
    for dj in range(-reach, reach + 1):
        for di in range(-reach, reach + 1):
            i, j = cell[0] + di, cell[1] + dj
            if not (0 <= i < grid.width and 0 <= j < grid.height):
                continue
            if int(grid.data[j * grid.width + i]) < 100:
                continue
            d = di * di + dj * dj
            if best is None or d < best:
                seed, best = (i, j), d
    if seed is None:
        return hit[0], hit[1], 0.0
    lim = math.ceil(window / grid.resolution)
    seen, todo = {seed}, [seed]
    while todo:
        ci, cj = todo.pop()
        for dj in (-1, 0, 1):
            for di in (-1, 0, 1):
                n = (ci + di, cj + dj)
                if n in seen or abs(n[0] - seed[0]) > lim or abs(n[1] - seed[1]) > lim:
                    continue
                if 0 <= n[0] < grid.width and 0 <= n[1] < grid.height \
                        and int(grid.data[n[1] * grid.width + n[0]]) >= 100:
                    seen.add(n)
                    todo.append(n)
    centres = [_centre_of(grid, i, j) for i, j in seen]
    mx = sum(p[0] for p in centres) / len(centres)
    my = sum(p[1] for p in centres) / len(centres)
    half = grid.resolution * math.sqrt(2) / 2
    return mx, my, max(math.hypot(px - mx, py - my) for px, py in centres) + half
