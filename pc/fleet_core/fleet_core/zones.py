"""Zone lock records: one holder per zone and a FIFO queue. Only this class changes them."""
from __future__ import annotations


class ZoneManager:
    def __init__(self, zone_ids, clock):
        self._clock = clock
        self._holder: dict[str, str | None] = {z: None for z in zone_ids}
        self._queue: dict[str, list[str]] = {z: [] for z in self._holder}
        self._acquired_at: dict[str, float | None] = {z: None for z in self._holder}
        self._held: dict[str, list[str]] = {}  # robot -> zones in acquire order

    def request(self, robot: str, zone: str) -> bool:
        """Ask for a zone. Granted only if it is free and the robot is first in line."""
        holder = self._holder[zone]
        if holder == robot:
            return True
        queue = self._queue[zone]
        if robot not in queue:
            queue.append(robot)
        if holder is None and queue[0] == robot:
            self._grant(robot, zone)
            return True
        return False

    def release(self, robot: str, zone: str) -> bool:
        """Free a zone. Refused unless the robot holds it."""
        if self._holder[zone] != robot:
            return False
        self._holder[zone] = None
        self._acquired_at[zone] = None
        self._held[robot].remove(zone)
        return True

    def force_acquire(self, robot: str, zone: str) -> bool:
        """Initial placement: take a free zone past the queue. Refused if someone holds it."""
        holder = self._holder[zone]
        if holder == robot:
            return True
        if holder is not None:
            return False
        self._grant(robot, zone)
        return True

    def cancel_request(self, robot: str, zone: str) -> bool:
        queue = self._queue[zone]
        if robot not in queue:
            return False
        queue.remove(robot)
        return True

    def cancel_all_requests(self, robot: str) -> None:
        for zone in self._queue:
            self.cancel_request(robot, zone)

    def holder(self, zone: str) -> str | None:
        return self._holder[zone]

    def queue(self, zone: str) -> tuple[str, ...]:
        return tuple(self._queue[zone])

    def held_by(self, robot: str) -> list[str]:
        return list(self._held.get(robot, ()))

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
        if robot in self._queue[zone]:
            self._queue[zone].remove(robot)
        self._holder[zone] = robot
        self._acquired_at[zone] = self._clock()
        self._held.setdefault(robot, []).append(zone)
