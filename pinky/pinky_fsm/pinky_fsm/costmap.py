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
