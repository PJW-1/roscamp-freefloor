"""SafetyMeter: how close the robots really came, on the true poses. No ROS imports.

The same two numbers as the lane simulation (05_관제_시뮬레이션_레인.html). The judge stays as it
is and these go next to its summary, in safety.json:
  early: a zone was released with the body still within one radius of it.
  near: two robots came closer than two radii. One count per pair, each time it happens.
"""
from __future__ import annotations

import itertools
import math


class SafetyMeter:
    def __init__(self, site, radius_m: float, on_course=lambda robot: True):
        self.site = site
        self.radius_m = radius_m
        self.on_course = on_course
        self.poses: dict[str, tuple[float, float]] = {}
        self.holders: dict[str, str] = {}
        self.early = 0
        self.near = 0
        self.releases = 0
        self.min_gap_m = math.inf
        self.min_release_m = math.inf
        self.log: list[str] = []
        self._close: set[tuple[str, str]] = set()

    def on_pose(self, robot: str, x: float, y: float) -> None:
        self.poses[robot] = (x, y)

    def on_zones(self, t: float, holders: dict[str, str]) -> None:
        for zone, new in holders.items():
            old = self.holders.get(zone, "")
            self.holders[zone] = new
            if old and old != new and old in self.poses:
                self._on_release(t, zone, old)

    def _on_release(self, t: float, zone: str, robot: str) -> None:
        x, y = self.poses[robot]
        gap = self.site.distance_to_zone(x, y, zone) + self.site.half_width
        self.releases += 1
        self.min_release_m = min(self.min_release_m, gap)
        if gap < self.radius_m:
            self.early += 1
            self.log.append(f"t={t:.1f} {robot} released {zone} {gap:.3f} m from it")

    def tick(self, t: float) -> None:
        present = sorted(r for r in self.poses if self.on_course(r))
        for one, other in itertools.combinations(present, 2):
            gap = math.dist(self.poses[one], self.poses[other])
            self.min_gap_m = min(self.min_gap_m, gap)
            if gap >= 2 * self.radius_m:
                self._close.discard((one, other))
            elif (one, other) not in self._close:
                self._close.add((one, other))
                self.near += 1
                self.log.append(f"t={t:.1f} {one} and {other} are {gap:.3f} m apart")

    def summary(self) -> dict:
        def finite(value):
            return value if math.isfinite(value) else None

        return {"early": self.early, "near": self.near, "releases": self.releases,
                "min_gap_m": finite(self.min_gap_m), "min_release_m": finite(self.min_release_m),
                "radius_m": self.radius_m, "log": self.log}
