"""SimWorld의 자유바닥판: 같은 가짜 로봇·중앙·팔에 freefloor 교통 층을 돌린다.

레인 검사 대신 칸 불변식을 매 단계 본다.
  - 로봇의 실제 몸(반지름 r)이 닿는 칸은 모두 그 로봇이 쥔 칸이다(쥔 칸 밖으로 나가지 않음).
  - 받은 MoveTo path 위 어느 점에서도 몸이 닿는 칸은 그 로봇이 쥔 칸이다.
  - 로봇 사이 거리 >= 2r, 물건 수는 그대로(SimWorld와 같은 검사).
frames를 켜면 tools/floor_view.py가 읽는 기록을 모은다. tick 계산 시간도 잰다.
"""
from __future__ import annotations

import itertools
import math
import time
from dataclasses import dataclass, field, replace

from fleet_core.common.params import load_params
from fleet_core.traffic.floor.cells import disk_cells, sample_line
from fleet_core.traffic.floor.floor_map import floor_map_for
from fleet_core.traffic.lane_map import load_lane_map
from legacy_core import FleetCore
from sim_world import DT, LINE_TOLERANCE_M, SimWorld


@dataclass
class FloorWorld(SimWorld):
    floor: object = None
    frame_every: int = 0  # 0: no frames; n: one frame every n steps
    tick_s: list = field(default_factory=list)

    def __post_init__(self):
        self.site = self.site or load_lane_map()
        self.params = replace(self.params or load_params(), traffic_mode="freefloor")
        super().__post_init__()
        self.floor = self.floor or floor_map_for(self.site)
        self.core = FleetCore(self.site, self.params, list(self.robots), lambda: self.t,
                              floor=self.floor)
        self.frames: list[dict] = []
        self.cell_violations = 0
        # robots whose true body stuck out of their cells before their first MoveTo: the report
        # it was placed from was off by more than the cell margin (localisation noise)
        self.placement_outside: set[str] = set()
        self._moved: set[str] = set()
        self.min_gap = math.inf
        self._steps = 0

    # ---- one step -----------------------------------------------------------------------

    def step(self):
        self.t = round(self.t + DT, 6)
        self._fire_faults()
        self._watch_obstacles()
        self._deliver_queued_results()
        self._move_robots()
        self._run_arms()
        self._report_states()
        self._central()
        start = time.perf_counter()
        out = self.core.tick()
        self.tick_s.append(time.perf_counter() - start)
        for item in out:
            self._apply(item)
        self._check_invariants()
        self._steps += 1
        if self.frame_every and self._steps % self.frame_every == 0:
            self.frames.append(self.frame())

    def frame(self) -> dict:
        snap = self.core.zones.snapshot()
        return {"t": self.t,
                "cells": {c: e["holder"] for c, e in snap.items() if e["holder"]},
                "robots": {n: [round(r.x, 3), round(r.y, 3)] for n, r in self.virtual.items()
                           if not r.removed},
                "paths": {n: [list(map(lambda v: round(v, 3), p))
                              for p in [(r.x, r.y), *r.via, *([r.target] if r.target else [])]]
                          for n, r in self.virtual.items() if r.target and not r.removed}}

    # ---- commands -----------------------------------------------------------------------

    def _start_move(self, cmd):
        r = self.virtual[cmd.robot]
        if not r.comm_up or r.removed:
            return
        assert cmd.path, f"{cmd} has no path"
        self._moved.add(r.name)
        assert tuple(cmd.path[-1][:2]) == (cmd.x, cmd.y), f"{cmd} path ends off the target"
        held = self.core.zones.held_set(cmd.robot)
        body = self.params.robot_radius_m
        # from the true pose: the robot drives from where it is, not from the reported pose
        points = [(r.x, r.y)] + [tuple(p[:2]) for p in cmd.path[1:]]
        for x, y in sample_line(points, self.floor.cell_m / 8):
            cells = {self.floor.cell_id(c) for c in disk_cells(self.floor, x, y, body)}
            assert cells <= held, f"{cmd} drives over cells it does not hold: {cells - held}"
        if self._refuses_path(r, cmd):
            return
        via = [tuple(p[:2]) for p in cmd.path[1:-1]]
        while via:  # skip what is already behind (like FollowPath)
            a = via[0]
            b = via[1] if len(via) > 1 else (cmd.x, cmd.y)
            if not self._behind(r, a, b):
                break
            via.pop(0)
        self._preempt(r)
        r.docked_at = ""
        r.goal_id = cmd.goal_id
        r.goal_kind = "move"
        r.target = (cmd.x, cmd.y)
        r.via = via
        r.goal_elapsed = 0.0

    @staticmethod
    def _behind(r, a, b) -> bool:
        """a를 이미 지났다: 로봇이 a보다 b 쪽에 있고 a에 붙어 있지 않다."""
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        span = dx * dx + dy * dy
        if span < LINE_TOLERANCE_M:
            return True
        f = ((r.x - ax) * dx + (r.y - ay) * dy) / span
        return f > 0 and math.dist((r.x, r.y), a) > LINE_TOLERANCE_M and f <= 1 and \
            math.dist((r.x, r.y), (ax + f * dx, ay + f * dy)) < 0.02

    def _start_dock(self, cmd):
        r = self.virtual[cmd.robot]
        if not r.comm_up or r.removed:
            return
        dock = self.floor.docks[cmd.dock_id]
        assert math.dist((r.x, r.y), dock) <= self.params.arrival_tolerance_m, (
            f"{cmd} away from the dock")
        self._preempt(r)
        r.goal_id = cmd.goal_id
        r.goal_kind = "dock"
        r.dock_left = self.dock_s
        r.dock_station = cmd.dock_id

    # ---- invariants ---------------------------------------------------------------------

    def _check_releases(self, before):
        pass  # cells: the body check below covers every release

    def _check_invariants(self):
        zones = self.core.zones
        radius = self.params.robot_radius_m
        present = [r for r in self.virtual.values() if not r.removed]
        for r in present:
            if self.t < r.localizing_until:
                continue
            body = {self.floor.cell_id(c) for c in disk_cells(self.floor, r.x, r.y, radius)}
            if not self.core.traffic._missions[r.name].placed:
                # not on the map yet (its reported cells were taken): it gets no goal, and no
                # one else may hold the cells it stands on
                taken = {c for c in body if zones.holder(c) not in (None, r.name)}
                assert not taken, f"{r.name} unplaced on cells others hold at {self.t}: {taken}"
                assert not r.goal_kind, f"{r.name} unplaced but on a {r.goal_kind} goal"
                continue
            held = zones.held_set(r.name)
            outside = body - held
            if outside and r.name not in self._moved:
                # counted apart (it has not driven yet), but never on a cell another one holds
                taken = {c for c in outside if zones.holder(c) is not None}
                assert not taken, f"{r.name} stands on cells others hold at {self.t}: {taken}"
                self.placement_outside.add(r.name)
                continue
            if outside:
                self.cell_violations += 1
            assert not outside, f"{r.name} at ({r.x:.3f}, {r.y:.3f}) is on cells it does not " \
                f"hold at {self.t}: {sorted(outside)}"
        for a, b in itertools.combinations(present, 2):
            gap = math.hypot(a.x - b.x, a.y - b.y)
            self.min_gap = min(self.min_gap, gap)
            assert gap >= 2 * radius, f"{a.name} and {b.name} touch at {self.t}: {gap:.3f} m"
        in_baskets = sum(sum(r.basket.values()) for r in self.virtual.values())
        left = sum(self.stock.values())
        assert all(v >= 0 for v in self.stock.values())
        assert left + in_baskets + sum(self.shipped_items.values()) == self.total_items, "item lost"
        for r in present:
            at_outbound = r.goal_kind == "" and r.docked_at == self.core._outbound
            assert r.has_basket or at_outbound, f"{r.name} left the dock without a basket"
