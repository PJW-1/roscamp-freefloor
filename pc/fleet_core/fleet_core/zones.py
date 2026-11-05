"""Zone lock records: one holder per zone and a FIFO queue. Only this class changes them.

Two zones can also be a conflict pair: lanes too close for two robots, so one robot at a time
may hold either. First come, first served holds across a pair too, or a robot at a crossing
could wait for as long as the other road stays busy.
"""
from __future__ import annotations

import math


class ZoneManager:
    def __init__(self, zone_ids, clock, conflicts=()):
        self._clock = clock
        self._holder: dict[str, str | None] = {z: None for z in zone_ids}
        self._queue: dict[str, list[str]] = {z: [] for z in self._holder}
        self._acquired_at: dict[str, float | None] = {z: None for z in self._holder}
        self._held: dict[str, list[str]] = {}  # robot -> zones in acquire order
        self._conflicts: dict[str, set[str]] = {z: set() for z in self._holder}
        for pair in conflicts:
            one, other = tuple(pair)
            self._conflicts[one].add(other)
            self._conflicts[other].add(one)
        self._arrivals = 0
        self._arrived: dict[tuple[str, str], int] = {}  # (zone, robot) -> order of the requests

    def request(self, robot: str, zone: str) -> bool:
        """Ask for a zone. Granted only if nobody stands between the robot and the zone."""
        if self._holder[zone] == robot:
            return True
        queue = self._queue[zone]
        if robot not in queue:
            queue.append(robot)
            self._arrivals += 1
            self._arrived[(zone, robot)] = self._arrivals
        if self.blockers(robot, zone):
            return False
        self._grant(robot, zone)
        return True

    def blockers(self, robot: str, zone: str) -> set[str]:
        """Who has to get out of the way first: the holder, those ahead in line, and the same
        for the zones this one is a conflict pair with."""
        if self._holder[zone] == robot:
            return set()
        found = {self._holder[zone]} if self._holder[zone] else set()
        for waiter in self._queue[zone]:
            if waiter == robot:
                break
            found.add(waiter)
        mine = self._arrived.get((zone, robot), math.inf)
        for other in self._conflicts[zone]:
            if self._holder[other] not in (None, robot):
                found.add(self._holder[other])
            found.update(waiter for waiter in self._queue[other]
                         if waiter != robot and self._arrived[(other, waiter)] < mine)
        return found

    def release(self, robot: str, zone: str) -> bool:
        """Free a zone. Refused unless the robot holds it."""
        if self._holder[zone] != robot:
            return False
        self._holder[zone] = None
        self._acquired_at[zone] = None
        self._held[robot].remove(zone)
        return True

    def force_acquire(self, robot: str, zone: str) -> bool:
        """Initial placement: take a free zone past the queue.

        Refused if someone holds it or a zone it is a conflict pair with.
        """
        if self._holder[zone] == robot:
            return True
        if self.holders_around(zone, robot):
            return False
        self._grant(robot, zone)
        return True

    def holders_around(self, zone: str, robot: str) -> dict[str, str]:
        """Others holding this zone or a zone it is a conflict pair with: zone -> holder."""
        return {z: self._holder[z] for z in (zone, *sorted(self._conflicts[zone]))
                if self._holder[z] not in (None, robot)}

    def cancel_request(self, robot: str, zone: str) -> bool:
        queue = self._queue[zone]
        if robot not in queue:
            return False
        queue.remove(robot)
        del self._arrived[(zone, robot)]
        return True

    def cancel_all_requests(self, robot: str) -> None:
        for zone in self._queue:
            self.cancel_request(robot, zone)

    def exists(self, zone: str) -> bool:
        return zone in self._holder

    def holder(self, zone: str) -> str | None:
        return self._holder[zone]

    def queue(self, zone: str) -> tuple[str, ...]:
        return tuple(self._queue[zone])

    def held_by(self, robot: str) -> list[str]:
        return list(self._held.get(robot, ()))

    def waiting_for(self, robot: str) -> str | None:
        """The zone it queues for. A robot asks for its next zone only when it holds no other."""
        return next((zone for zone, who in self._arrived if who == robot), None)

    def snapshot(self) -> dict[str, dict]:
        return {
            zone: {
                "holder": self._holder[zone],
                "queue": tuple(self._queue[zone]),
                "acquired_at": self._acquired_at[zone],
            }
            for zone in self._holder
        }

    def _grant(self, robot: str, zone: str) -> None:
        self.cancel_request(robot, zone)
        self._holder[zone] = robot
        self._acquired_at[zone] = self._clock()
        self._held.setdefault(robot, []).append(zone)
