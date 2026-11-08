"""Driving check for the three-pinky Gazebo run. No ROS; the node is fleet_check_node.

Fed with the Gazebo true pose of every robot (sim time, speed, turn rate), the state each robot
reports, and the zone holders the fleet publishes. It keeps:
  - the closest approach between two robots, from samples at most sync_s apart
  - stops per robot: standing still (speed < STILL_MPS and turn < STILL_RADPS) for stop_min_s
    after moving faster than MOVING_MPS. Stops while the robot reports MOVING are also counted
    apart (a halt in the middle of a move_to)
  - pose samples outside every zone the fleet holds for that robot (farther than margin_m from
    the zone floor, fleet_core LaneMap.distance_to_zone), and apart from those the samples while
    the robot holds no zone at all. Counted only after the first zone message, so the time
    before the fleet starts does not count
  - sim and wall time spans and their ratio (real-time factor)
"""
from __future__ import annotations

import math
from collections import deque

STILL_MPS = 0.02
STILL_RADPS = 0.1
MOVING_MPS = 0.05
STOP_MIN_S = 0.5
SYNC_S = 0.1
KEEP_S = 1.0  # keep this much history so a sample handled late still meets its peers
MARGIN_M = 0.0
MOVING = "MOVING"


class _Robot:
    def __init__(self):
        self.recent: deque[tuple[float, float, float]] = deque()  # (sim t, x, y), newest last
        self.state = ""
        self.moved = False  # moved since the last counted stop
        self.still_since: float | None = None
        self.still_state = ""
        self.stops = 0
        self.stops_while_moving = 0
        self.zone_samples = 0
        self.outside = 0
        self.no_zone = 0
        self.worst_outside_m = 0.0


class FleetCheck:
    def __init__(self, site, robots, *, stop_min_s: float = STOP_MIN_S, sync_s: float = SYNC_S,
                 margin_m: float = MARGIN_M):
        self.site = site
        self.robots = {robot: _Robot() for robot in robots}
        self.stop_min_s = stop_min_s
        self.sync_s = sync_s
        self.margin_m = margin_m
        self.holders: dict[str, str] | None = None
        self.min_gap: tuple[float, float, str, str] | None = None  # (m, sim t, one, other)
        self.first: tuple[float, float] | None = None  # (sim t, wall t)
        self.last: tuple[float, float] | None = None

    def on_state(self, robot: str, state: str) -> None:
        if robot in self.robots:
            self.robots[robot].state = state

    def on_zones(self, holders: dict[str, str]) -> None:
        self.holders = dict(holders)

    def on_truth(self, robot: str, t: float, x: float, y: float, speed: float, turn: float,
                 wall: float | None = None) -> None:
        me = self.robots.get(robot)
        if me is None:
            return
        me.recent.append((t, x, y))
        while me.recent and me.recent[0][0] < me.recent[-1][0] - KEEP_S - self.sync_s:
            me.recent.popleft()
        if wall is not None:
            self.first = self.first or (t, wall)
            self.last = (t, wall)
        self._gaps(robot, t, x, y)
        self._stop(me, t, speed, turn)
        self._zones(robot, me, x, y)

    def _gaps(self, robot: str, t: float, x: float, y: float) -> None:
        """Every recent sample of the others within sync_s of this one, not only the latest."""
        for other, them in self.robots.items():
            if other == robot:
                continue
            for t_other, ox, oy in them.recent:
                if abs(t_other - t) > self.sync_s:
                    continue
                gap = math.dist((x, y), (ox, oy))
                if self.min_gap is None or gap < self.min_gap[0]:
                    self.min_gap = (gap, max(t, t_other), *sorted((robot, other)))

    def _stop(self, me: _Robot, t: float, speed: float, turn: float) -> None:
        if speed > MOVING_MPS:
            me.moved = True
        if speed >= STILL_MPS or abs(turn) >= STILL_RADPS:
            me.still_since = None
            return
        if me.still_since is None:
            me.still_since, me.still_state = t, me.state
        if me.moved and t - me.still_since >= self.stop_min_s - 1e-9:
            me.moved = False
            me.stops += 1
            if me.still_state == MOVING:
                me.stops_while_moving += 1

    def _zones(self, robot: str, me: _Robot, x: float, y: float) -> None:
        if self.holders is None:
            return
        me.zone_samples += 1
        held = [zone for zone, holder in self.holders.items() if holder == robot]
        if not held:
            me.no_zone += 1
            return
        away = min(self.site.distance_to_zone(x, y, zone) for zone in held)
        if away > self.margin_m:
            me.outside += 1
            me.worst_outside_m = max(me.worst_outside_m, away)

    def summary(self) -> dict:
        robots = {name: {"stops": r.stops, "stops_while_moving": r.stops_while_moving,
                         "outside_held_samples": r.outside, "no_zone_held_samples": r.no_zone,
                         "zone_samples": r.zone_samples,
                         "worst_outside_m": round(r.worst_outside_m, 3)}
                  for name, r in self.robots.items()}
        sim_span = wall_span = rtf = None
        if self.first and self.last:
            sim_span = self.last[0] - self.first[0]
            wall_span = self.last[1] - self.first[1]
            rtf = round(sim_span / wall_span, 3) if wall_span > 0 else None
        gap = self.min_gap
        return {
            "min_distance_m": None if gap is None else round(gap[0], 3),
            "min_distance_pair": None if gap is None else [gap[2], gap[3]],
            "min_distance_at_s": None if gap is None else round(gap[1], 2),
            "stops": sum(r.stops for r in self.robots.values()),
            "stops_while_moving": sum(r.stops_while_moving for r in self.robots.values()),
            "outside_held_samples": sum(r.outside for r in self.robots.values()),
            "no_zone_held_samples": sum(r.no_zone for r in self.robots.values()),
            "zone_samples": sum(r.zone_samples for r in self.robots.values()),
            "sim_span_s": None if sim_span is None else round(sim_span, 2),
            "wall_span_s": None if wall_span is None else round(wall_span, 2),
            "real_time_factor": rtf,
            "robots": robots,
        }
