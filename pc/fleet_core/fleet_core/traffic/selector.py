"""채우기 스테이션으로 부를 로봇을 고른다.

순수 함수다. TrafficController가 로봇마다 RobotInfo를 주고 결과를 적용한다.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RobotInfo:
    robot_id: str
    waypoint: object  # 거리를 재는 출발 지점
    called: bool = False
    parked: bool = False  # 대기 지점에 서 있음, 앞 대기 지점 뒤에 줄 선다
    eligible: bool = True  # 사용 가능, 스테이션에 잡혀 있지 않고 지정 배차도 없음
    available: bool = True  # 통신 끊김이나 오류 중이면 False


class RobotSelector:
    def __init__(self, site, params):
        self._site = site
        self._params = params
        self._dock = next(s.dock for s in site.stations.values() if s.action == "fill")

    def pick(self, calls: int, robots: list[RobotInfo]) -> set[str]:
        """부를 로봇들, 최대 `calls`대."""
        pool = [r for r in robots if r.eligible]
        chosen: set[str] = set()
        while len(chosen) < calls:
            free = [r for r in pool if self._may_pick(r, pool, robots)]
            if not free:
                break
            best = min(free, key=lambda r: (self._score(r), r.robot_id))
            chosen.add(best.robot_id)
            pool.remove(best)
        return chosen

    def _score(self, robot: RobotInfo) -> float:
        metres = self._site.path_length(self._site.plan(robot.waypoint, self._dock))
        score = self._params.score_distance_weight * metres
        return score + (self._params.score_same_call if robot.called else 0.0)

    def _may_pick(self, robot: RobotInfo, pool: list[RobotInfo], robots: list[RobotInfo]) -> bool:
        """대기 줄은 한 방향이다. 앞에서 기다리는 로봇을 앞지르지 않는다."""
        if not robot.parked:
            return True
        rank = self._site.parking.index(robot.waypoint)
        for other in robots:
            if not other.parked or self._site.parking.index(other.waypoint) <= rank:
                continue
            if other in pool or not other.available:
                return False
        return True
