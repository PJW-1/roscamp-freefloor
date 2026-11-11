"""막힌 물체 보고(제안)의 위치 계산. ROS를 쓰지 않는다."""
from __future__ import annotations

import math

GAP_M = 0.02  # 로봇 몸 앞 끝에서 물체까지 작은 틈


def point_along(route, distance: float) -> tuple[float, float]:
    """꺾은선을 따라 distance만큼 간 점. 끝을 넘으면 끝점, 음수면 시작점."""
    x, y = route[0]
    left = max(0.0, distance)
    for a, b in zip(route, route[1:], strict=False):
        seg = math.dist(a, b)
        if left <= seg:
            if seg > 0.0:
                return (a[0] + (b[0] - a[0]) / seg * left, a[1] + (b[1] - a[1]) / seg * left)
            return a
        left -= seg
        x, y = b
    return (x, y)


def obstacle_fields(pos, route, stop_ahead_m: float, robot_radius_m: float,
                    gap_m: float = GAP_M, centre=None,
                    radius: float = 0.0) -> tuple[float, float, float]:
    """(x, y, 반지름). 물체 위치를 알면 그대로, 모르면 서는 자리 앞 robot_radius+gap 점.

    stop_ahead_m: 로봇에서 서는 자리까지 path를 따라 남은 거리. 반지름 0은 기본값을 쓰라는 뜻.
    """
    if centre is not None:
        return float(centre[0]), float(centre[1]), float(radius)
    x, y = point_along([tuple(pos), *route], stop_ahead_m + robot_radius_m + gap_m)
    return x, y, float(radius)
