"""시뮬 배터리 모형: low_battery 고장이면 낮아지고, 충전 자리에 서 있으면 다시 찬다. ROS를 쓰지 않는다.

충전 자리는 레인 지도의 주차 지점이다. 가짜 핑키, Gazebo의 sim_stand_in, 머리 없는 세계가 같이 쓴다.
"""
from __future__ import annotations

import math

FULL_PCT = 90.0  # 고장 전과 충전이 끝난 뒤
LOW_PCT = 10.0  # low_battery 시각부터
CHARGE_NEAR_M = 0.12  # 충전 자리에서 이 거리 안이면 자리 위


def _get(fault, key: str, default=None):
    """시나리오의 Fault와 고장 dict(머리 없는 세계) 둘 다 읽는다."""
    if isinstance(fault, dict):
        return fault.get(key, default)
    return getattr(fault, key, default)


class Battery:
    """로봇 한 대의 배터리. percent는 시각이 줄지 않는 차례로 부른다."""

    def __init__(self, faults, spots):
        events = [f for f in faults if _get(f, "type") == "low_battery"]
        # 안정 정렬: 같은 시각은 받은 순서
        self._events = sorted(((float(_get(f, "at_s")), float(_get(f, "duration_s")))
                               for f in events), key=lambda e: e[0])
        self._spots = [(float(x), float(y)) for x, y in spots]
        self._index = 0  # 지금(또는 다음) 사건
        self._charged_s = 0.0  # 지금 사건에서 충전한 시간
        self._last: float | None = None

    def _near(self, x: float, y: float) -> bool:
        return any(math.dist((x, y), spot) <= CHARGE_NEAR_M for spot in self._spots)

    def percent(self, now_s: float, x: float, y: float, standing: bool) -> float:
        last, self._last = self._last, now_s
        while self._index < len(self._events):
            at_s, duration_s = self._events[self._index]
            if now_s < at_s:
                return FULL_PCT
            if last is not None and standing and self._near(x, y):
                self._charged_s += max(0.0, now_s - max(last, at_s))
            if self._charged_s < duration_s:
                return LOW_PCT + (FULL_PCT - LOW_PCT) * self._charged_s / duration_s
            # 다 찼다. 시각이 지난 다음 사건은 지금부터 시작한다
            self._index += 1
            self._charged_s = 0.0
            last = now_s
        return FULL_PCT
