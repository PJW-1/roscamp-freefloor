"""경로 기하. 레인 폴리라인을 Nav2 경로 점으로 펼치고, 로봇 위치를 선에 투영한다.

ROS 없이 돈다. 좌표는 지도(map) 틀의 m, 각도는 rad.
"""
from __future__ import annotations

import math
from collections.abc import Sequence
from itertools import pairwise

Point = tuple[float, float]
Pose = tuple[float, float, float]

_SAME_M = 1e-6  # 이보다 가까운 이웃 점은 같은 점으로 친다


def _dedup(points: Sequence[Point]) -> list[Point]:
    out: list[Point] = []
    for p in points:
        p = (float(p[0]), float(p[1]))
        if not out or math.dist(out[-1], p) > _SAME_M:
            out.append(p)
    return out


def poses_along(points: Sequence[Point], final_yaw: float, step: float = 0.05) -> list[Pose]:
    """폴리라인을 step 이하 간격의 (x, y, yaw)로 펼친다.

    각 점의 yaw는 그 점이 속한 선분의 방향이고, 마지막 점만 final_yaw다.
    """
    pts = _dedup(points)
    if len(pts) == 1:
        return [(pts[0][0], pts[0][1], float(final_yaw))]
    poses: list[Pose] = []
    for (ax, ay), (bx, by) in pairwise(pts):
        length = math.hypot(bx - ax, by - ay)
        yaw = math.atan2(by - ay, bx - ax)
        n = max(1, math.ceil(length / step - 1e-9))
        for i in range(n):
            f = i / n
            poses.append((ax + f * (bx - ax), ay + f * (by - ay), yaw))
    last = pts[-1]
    poses.append((last[0], last[1], float(final_yaw)))
    return poses


class Polyline:
    """누적 거리 s로 점을 찾는 폴리라인."""

    def __init__(self, points: Sequence[Point]):
        self.points = _dedup(points)
        if not self.points:
            raise ValueError("a polyline needs at least one point")
        self._s = [0.0]
        for a, b in pairwise(self.points):
            self._s.append(self._s[-1] + math.dist(a, b))
        self.length = self._s[-1]

    def project(self, p: Sequence[float], s_min: float = 0.0,
                s_max: float = math.inf) -> tuple[float, float]:
        """p에 가장 가까운 선 위의 점. (그 점의 s, p까지의 거리)

        s_min, s_max를 주면 그 구간 안에서만 찾는다(같은 곳을 두 번 지나는 선에서
        이미 지난 쪽이나 너무 먼 앞쪽으로 뛰지 않게).
        """
        px, py = float(p[0]), float(p[1])
        if len(self.points) == 1:
            return 0.0, math.hypot(px - self.points[0][0], py - self.points[0][1])
        best = (math.inf, 0.0)
        for i, ((ax, ay), (bx, by)) in enumerate(pairwise(self.points)):
            s0, s1 = self._s[i], self._s[i + 1]
            if s1 < s_min or s0 > s_max:
                continue
            seg = s1 - s0
            f = ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / (seg * seg)
            lo = max(0.0, (s_min - s0) / seg)
            hi = min(1.0, (s_max - s0) / seg)
            f = min(max(f, lo), hi)
            qx, qy = ax + f * (bx - ax), ay + f * (by - ay)
            d = math.hypot(px - qx, py - qy)
            if d < best[0] - 1e-12:
                best = (d, s0 + f * seg)
        return best[1], best[0]

    def point_at(self, s: float) -> Point:
        if s <= 0.0 or len(self.points) == 1:
            return self.points[0]
        if s >= self.length:
            return self.points[-1]
        for i in range(len(self.points) - 1):
            if self._s[i + 1] >= s:
                (ax, ay), (bx, by) = self.points[i], self.points[i + 1]
                f = (s - self._s[i]) / (self._s[i + 1] - self._s[i])
                return (ax + f * (bx - ax), ay + f * (by - ay))
        return self.points[-1]

    def rest(self, s: float) -> list[Point]:
        """s 지점부터 끝까지. s 뒤의 꼭짓점은 그대로 둔다."""
        if s >= self.length:
            return [self.points[-1]]
        start = self.point_at(s)
        later = [p for p, ps in zip(self.points, self._s, strict=True) if ps > s + _SAME_M]
        return _dedup([start, *later])

    def sample(self, s0: float, s1: float, step: float) -> list[Point]:
        """s0부터 s1(선 끝에서 자름)까지 step 이하 간격의 점."""
        s1 = min(s1, self.length)
        s0 = min(max(s0, 0.0), s1)
        n = max(1, math.ceil((s1 - s0) / step - 1e-9))
        return [self.point_at(s0 + (s1 - s0) * i / n) for i in range(n + 1)]


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
