"""운영자의 치우기와 재투입 판단. ROS 없이 시험한다(scenario_monitor_node가 부른다).

치울 때 심판의 보유 스냅샷이 관제보다 늦으면 일부만 풀린다(ZONES_STILL_HELD).
실제 운영자처럼 남은 구역을 다시 치우고, 남은 구역이 없을 때만 재투입한다.
관제는 이미 풀린 구역도 받는다(recovery.py).
푼 구역은 다시 보내지 않는다.
"""
from __future__ import annotations

import ast
from dataclasses import dataclass, field

STILL_HELD = "ZONES_STILL_HELD"


@dataclass
class _Removal:
    clears: int = 0
    in_flight: bool = False
    accepted: bool = False
    released: set = field(default_factory=set)  # 이 치우기에서 푼 구역
    still: set = field(default_factory=set)  # 관제가 아직 쥐었다고 알린 구역
    gave_up: bool = False
    said: bool = False


def still_held_zones(detail: str) -> list[str]:
    """"['AVENUE'] stay locked"에서 구역 목록."""
    try:
        zones = ast.literal_eval(detail.split(" stay locked", 1)[0])
    except (ValueError, SyntaxError):
        return []
    return [z for z in zones if isinstance(z, str)] if isinstance(zones, list) else []


class OperatorClear:
    def __init__(self, max_clears: int = 5):
        self.max_clears = max_clears
        self._robots: dict[str, _Removal] = {}

    def start(self, robot: str) -> None:
        """치울 때가 됐고 로봇이 코스 밖이다."""
        self._robots[robot] = _Removal()

    def _left(self, r: _Removal, held: list[str]) -> list[str]:
        return sorted((r.still | set(held)) - r.released)

    def next_clear(self, robot: str, held: list[str]) -> list[str] | None:
        """지금 confirm_clear로 보낼 구역. 부르지 않으면 None. 첫 번째는 스냅샷 그대로."""
        r = self._robots.get(robot)
        if r is None or r.in_flight or r.gave_up:
            return None
        zones = sorted(held) if not r.clears else self._left(r, held)
        if r.clears and not zones:
            return None
        if r.clears >= self.max_clears:
            r.gave_up = True  # 판정은 재투입 없음으로 실패를 잡는다
            return None
        r.clears += 1
        r.in_flight = True
        return zones

    def on_cleared(self, robot: str, accepted: bool, released: list[str]) -> None:
        r = self._robots.get(robot)
        if r is None:
            return
        r.in_flight = False
        r.accepted = r.accepted or accepted
        r.released |= set(released)
        r.still -= set(released)

    def on_event(self, source: str, code: str, detail: str) -> None:
        r = self._robots.get(source)
        if r is not None and code == STILL_HELD:
            r.still |= set(still_held_zones(detail)) - r.released  # 늦게 온 알림은 무시

    def gave_up(self, robot: str) -> bool:
        """상한에 걸려 멈췄는가. 처음 한 번만 True."""
        r = self._robots.get(robot)
        if r is None or not r.gave_up or r.said:
            return False
        r.said = True
        return True

    def ready(self, robot: str, held: list[str]) -> bool:
        """재투입해도 되는가: 받아들여진 치우기가 있고 남은 구역이 없다."""
        r = self._robots.get(robot)
        return (r is not None and r.accepted and not r.in_flight and not r.gave_up
                and not self._left(r, held))

    def reinstated(self, robot: str) -> None:
        self._robots.pop(robot, None)
