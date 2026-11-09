"""자유바닥 ETA(버전 2 공식 ETA를 바닥 경로에): 길이 ÷ 속도 + 제자리 회전 시간."""
from __future__ import annotations

import math

JOIN_M = 0.1  # 이보다 짧은 선분을 사이에 둔 꺾임은 한 모서리(원호)


def turn_s(params, angle: float) -> float:
    """제자리 회전 시간. eta_turn_min_rad보다 작은 꺾임은 달리면서 지난다."""
    return angle * params.eta_turn_s_per_rad if angle >= params.eta_turn_min_rad else 0.0


def route_s(points, params, heading: float | None = None) -> float:
    """꺾은선을 따라가는 시간(초). heading은 출발 때 로봇 방향."""
    pts = [p for k, p in enumerate(points) if k == 0 or math.dist(p, points[k - 1]) > 1e-9]
    if len(pts) < 2:
        return 0.0
    segs = [(math.atan2(b[1] - a[1], b[0] - a[0]), math.dist(a, b))
            for a, b in zip(pts, pts[1:], strict=False)]
    t = sum(length for _, length in segs) / params.eta_speed_mps
    acc = 0.0
    if heading is not None:
        acc = math.remainder(segs[0][0] - heading, math.tau)
        if segs[0][1] >= JOIN_M:
            t += turn_s(params, abs(acc))
            acc = 0.0
    for (h0, _), (h1, length) in zip(segs, segs[1:], strict=False):
        acc += math.remainder(h1 - h0, math.tau)
        if length >= JOIN_M:  # 긴 선분이 이어지면 모서리 끝
            t += turn_s(params, abs(acc))
            acc = 0.0
    return t + turn_s(params, abs(acc))
