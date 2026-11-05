"""Which robots are called to the fill station.

Pure: TrafficCore describes each robot with a RobotInfo and applies the result.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RobotInfo:
    robot_id: str
    cell: object  # node it counts from
    called: bool = False
    parked: bool = False  # standing in a waiting cell, in line behind the cells ahead of it
    eligible: bool = True  # available, not kept at a station and not sent somewhere by name
    available: bool = True  # False while comm is lost or in error


class Dispatcher:
    def __init__(self, site, params):
        self._site = site
        self._params = params
        self._dock = next(s.dock for s in site.stations.values() if s.action == "fill")

    def pick(self, calls: int, robots: list[RobotInfo]) -> set[str]:
        """Return the robots to call, at most `calls` of them."""
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
        metres = self._site.path_length(self._site.plan(robot.cell, self._dock))
        score = self._params.score_distance_weight * metres
        return score + (self._params.score_same_call if robot.called else 0.0)

    def _may_pick(self, robot: RobotInfo, pool: list[RobotInfo], robots: list[RobotInfo]) -> bool:
        """The waiting line is one-way: nobody passes the waiting robots ahead of it."""
        if not robot.parked:
            return True
        rank = self._site.parking.index(robot.cell)
        for other in robots:
            if not other.parked or self._site.parking.index(other.cell) <= rank:
                continue
            if other in pool or not other.available:
                return False
        return True
