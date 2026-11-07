"""관제 흉내 주행의 판단(경로를 늘리는 때)과 안내 4절 지표 계산. ROS 없이 돈다."""
from __future__ import annotations

import math
from collections.abc import Sequence

Point = tuple[float, float]
PASSED_M = 0.10  # 지점에서 이만큼 가까워지면 지난 것으로 친다(RPP는 모서리를 조금 깎는다)


class Trip:
    """지점 목록을 따라가는 주행 하나. 관제처럼 경로를 조금씩 늘려 보낸다.

    관제는 잠금을 더 얻을 때마다 로봇이 마지막으로 지난 지점에서 시작하는 더 긴 경로를
    보낸다(안내 2.1, 2.2). 여기서는 로봇이 구간의 extend_at(비율)을 지나면 ahead+1개
    앞 지점까지 늘린다. 그래서 경로 끝은 늘 로봇보다 한 구간 반 이상 앞에 있다.
    """

    def __init__(self, points: Sequence[Point], ahead: int = 2, extend_at: float = 0.5):
        if len(points) < 2:
            raise ValueError("a trip needs at least two waypoints")
        self.points = [tuple(p) for p in points]
        self.ahead = ahead
        self.extend_at = extend_at
        self.segment = 0  # 로봇이 있는 구간(points[segment] → points[segment + 1])
        self.end = min(ahead, len(self.points) - 1)

    def first(self) -> list[int]:
        return list(range(0, self.end + 1))

    def granted_all(self) -> bool:
        return self.end == len(self.points) - 1

    def _fraction(self, x: float, y: float) -> float:
        (ax, ay), (bx, by) = self.points[self.segment], self.points[self.segment + 1]
        dx, dy = bx - ax, by - ay
        return ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)

    def update(self, x: float, y: float) -> list[int] | None:
        """로봇 위치를 받아, 경로를 늘릴 때면 새 경로(지점 번호)를 돌려준다."""
        last = len(self.points) - 1
        while self.segment < last - 1 and (
                self._fraction(x, y) >= 1.0
                or math.dist((x, y), self.points[self.segment + 1]) < PASSED_M):
            self.segment += 1
        if self.granted_all() or self._fraction(x, y) < self.extend_at:
            return None
        end = min(self.segment + self.ahead + 1, last)
        if end <= self.end:
            return None
        self.end = end
        return list(range(self.segment, end + 1))


def lap_stops(cycle: Sequence[str], start: str, laps: int) -> list[str]:
    """start에서 출발해 cycle을 laps바퀴 돌고 start로 돌아오는 정지 지점 목록."""
    i = list(cycle).index(start)
    order = [*cycle[i:], *cycle[:i]]
    return order * laps + [start]


def stopped_after(samples: Sequence[tuple[float, float]], t: float, before: float = 0.5,
                  after: float = 0.5, moving: float = 0.05, still: float = 0.01) -> bool | None:
    """t 앞 before초 동안 달리다가(평균 선속도 > moving) t 뒤 after초 안에 still 밑으로
    떨어졌나. 앞에서 달리지 않고 있었거나 t + after까지 기록이 없으면 None(판정하지 않음)."""
    pre = [v for s, v in samples if t - before <= s < t]
    post = [v for s, v in samples if t <= s <= t + after]
    if not pre or sum(pre) / len(pre) <= moving or samples[-1][0] < t + after:
        return None
    return any(abs(v) < still for v in post)


def full_stop_after(samples: Sequence[tuple[float, float, float]], t: float, before: float = 0.5,
                    after: float = 0.5, moving: float = 0.05, still: float = 0.01,
                    turn_still: float = 0.05, min_s: float = 0.1) -> bool | None:
    """stopped_after와 같되 (시각, 선속도, 각속도)를 받아 둘 다 0인 채 min_s초 이어질 때만
    정지로 친다.

    RPP는 모서리에서 선속도 0으로 제자리 회전하고(R12), 회전과 주행을 오갈 때 각속도가 0을
    한 표본 지난다. 취소 뒤 재전송처럼 controller가 속도 0을 내는 정지(안내 2.1)만 센다."""
    pre = [v for s, v, _ in samples if t - before <= s < t]
    if not pre or sum(pre) / len(pre) <= moving or samples[-1][0] < t + after:
        return None
    level = [(s, max(abs(v) / still, abs(w) / turn_still)) for s, v, w in samples]
    return still_episodes(level, t, t + after, still=1.0, min_s=min_s) > 0


def still_episodes(samples: Sequence[tuple[float, float]], t0: float, t1: float,
                   still: float = 0.01, min_s: float = 0.1) -> int:
    """[t0, t1] 안에서 값이 still 밑으로 min_s초 이상 이어진 구간 수."""
    count, start, last = 0, None, None
    for s, v in samples:
        if not t0 <= s <= t1:
            continue
        if abs(v) < still:
            start = s if start is None else start
            last = s
        else:
            if start is not None and last - start >= min_s - 1e-9:
                count += 1
            start = None
    if start is not None and last - start >= min_s - 1e-9:
        count += 1
    return count


def speeds(truth: Sequence[tuple[float, Point]], window: float = 0.2) -> list[tuple[float, float]]:
    """위치 기록에서 window초 앞 위치와 비교한 속도(m/s)."""
    out, j = [], 0
    for t, p in truth:
        while truth[j][0] < t - window:
            j += 1
        t0, p0 = truth[j]
        out.append((t, math.dist(p, p0) / (t - t0) if t > t0 else 0.0))
    return out


def turn_speeds(headings: Sequence[tuple[float, float]],
                window: float = 0.2) -> list[tuple[float, float]]:
    """방향(rad) 기록에서 window초 앞 방향과 비교한 회전 속도(rad/s, 크기). ±π를 넘어가도 된다."""
    out, j = [], 0
    for t, yaw in headings:
        while headings[j][0] < t - window:
            j += 1
        t0, yaw0 = headings[j]
        out.append((t, abs(math.remainder(yaw - yaw0, math.tau)) / (t - t0) if t > t0 else 0.0))
    return out


def time_to_stop(samples: Sequence[tuple[float, float]], t0: float,
                 still: float = 0.01, hold: float = 0.3) -> float | None:
    """t0 뒤 속도가 still 밑으로 떨어져 hold초 넘게 이어지기 시작한 때까지(초).

    감속 중 한 표본만 낮아졌다가 다시 움직이는 것은 정지가 아니다. 끝까지 못 봤으면 None."""
    start = None
    for s, v in samples:
        if s < t0:
            continue
        if abs(v) < still:
            start = s if start is None else start
            if s - start >= hold - 1e-9:
                return start - t0
        else:
            start = None
    return None


def settle_time(truth: Sequence[tuple[float, Point]], target: Point, t_result: float,
                radius: float = 0.15) -> float | None:
    """결과 시각 전 마지막으로 target의 radius 안에 들어온 때부터 결과까지(초)."""
    entered = None
    for s, p in truth:
        if s > t_result:
            break
        inside = math.dist(p, target) <= radius
        if inside and entered is None:
            entered = s
        elif not inside:
            entered = None
    return None if entered is None else t_result - entered
