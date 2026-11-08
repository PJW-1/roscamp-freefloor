"""자유바닥 교통 제어기. TrafficController의 바깥 API를 그대로 지키고 이동과 잠금만 바꾼다.

물려받는 것: 입력 처리, 배차, 임무 단계(TO_WORK, DOCKING, AT_STATION, ...), 도착 뒤 다음 일,
로봇 선택, 통신 점검, 격리. 바꾸는 것: 구역 대신 칸 잠금(CellLocks), 레인 대신 바닥 경로.

안전 규칙(각각 시험이 있다):
  - 로봇이 쓰는 칸 = 경로를 따라 몸(반지름 + 여유)이 쓸고 지나가는 칸.
  - 굴러가는 잠금: 앞쪽으로 floor_lock_ahead_m만큼 미리 잠근다. 늘릴 칸은 한꺼번에 잡을 수
    있을 때만 늘린다. 다 못 잡으면 floor_extend_step_m씩 짧게 다시 한꺼번에 시도한다.
  - 보내는 path는 잠근 끝까지만이다. 잠금이 늘면 MoveTo를 교체한다(서지 않는다).
  - 몸 아래 칸, 잠근 앞 칸, 아직 따라가는 MoveTo의 남은 칸은 쥔다. 나머지 칸은 연속 보고
    release_confirm_samples번이 칸에서 release_distance_m보다 멀 때 놓는다.
  - 고정 선(도킹 접근, 도크에서 나감)은 들어가기 전에 선 끝까지 한꺼번에 잡는다.
m.path는 [출발할 때 가까운 지점, 목적지 지점] 두 개다. 선택과 단계 처리가 그대로 쓴다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from fleet_core.common.commands import MoveCmd
from fleet_core.traffic.controller import TrafficController
from fleet_core.traffic.floor.cells import CellLocks, disk_cells, swept_cells
from fleet_core.traffic.floor.floor_map import load_floor_map
from fleet_core.traffic.floor.planner import FloorPlanner, Route
from fleet_core.traffic.mission import (
    ERROR,
    HEARTBEAT_LOST,
    IDLE,
    MOVE,
    NOT_PLACEABLE,
    ON_A_GOAL,
    PARKED,
    PLACEMENT_CONFLICT,
    RUN,
    TO_PARK,
    _Goal,
    _Mission,
)

AT_END_M = 1e-6


@dataclass
class _Leg:
    route: Route | None
    goal_xy: tuple
    out_line: str | None = None
    in_line: str | None = None
    s: float = 0.0  # progress along the route
    s_lock: float = 0.0  # the route is locked up to here (the centre may go this far)
    s_sent: float = 0.0  # end of the last MoveTo
    done: bool = False  # the MoveTo to the route end succeeded
    slowed: bool = False  # the locked part ahead is short (counted once per episode)
    planned_at: float = 0.0
    no_path_warned: bool = False


class FloorController(TrafficController):
    def __init__(self, site, params, robots, clock, next_goal_id=None, floor=None):
        super().__init__(site, params, robots, clock, next_goal_id)
        self.floor = floor or load_floor_map()
        if self.floor.robot_radius_m < params.robot_radius_m:
            raise ValueError(f"the floor map radius {self.floor.robot_radius_m} m is smaller "
                             f"than the fleet radius {params.robot_radius_m} m")
        self.zones = CellLocks(clock)
        self.planner = FloorPlanner(self.floor)
        self._body = self.floor.robot_radius_m + self.floor.cell_margin_m
        self._legs: dict[str, _Leg] = {}
        self._active: dict[str, Route] = {}  # path of the MoveTo the robot may still follow
        self._wp_cell: dict = {}
        self.metrics = {"slowdowns": 0, "replans": 0, "sidesteps": 0, "no_path": 0}

    # ---- cells --------------------------------------------------------------------------

    def _ids(self, cells) -> set[str]:
        return {self.floor.cell_id(c) for c in cells}

    def _body_cells(self, x: float, y: float) -> set[str]:
        return self._ids(disk_cells(self.floor, x, y, self._body))

    def _swept(self, points) -> set[str]:
        return self._ids(swept_cells(self.floor, points, self._body))

    def _zone(self, waypoint) -> str:
        """지점이 든 칸. 물려받은 대기 지점·commit 판단이 이 칸의 주인을 본다."""
        cell = self._wp_cell.get(waypoint)
        if cell is None:
            cell = self.floor.cell_id(self.floor.cell_of(*self.site.waypoint_pose(waypoint)))
            self._wp_cell[waypoint] = cell
        return cell

    # ---- reports ------------------------------------------------------------------------

    def _place(self, m: _Mission) -> None:
        """보고 자리의 몸 칸을 잡는다. 못 잡으면 맵 밖에 두고 다음 보고 때 다시 한다."""
        if m.robot_state in NOT_PLACEABLE:
            return
        cells = self._body_cells(m.x, m.y)
        if not self.zones.force_acquire(m.name, cells):
            in_the_way = sorted({self.zones.holder(c) for c in cells} - {None, m.name})
            if m.fault_reason != PLACEMENT_CONFLICT:
                m.fault = ERROR
                m.fault_reason = PLACEMENT_CONFLICT
                self._event("ERROR", PLACEMENT_CONFLICT,
                            f"{m.name} cannot take its cells, held by {in_the_way}", m.name)
            return
        m.placed = True
        self._joined(m)
        waypoint = self.site.nearest_waypoint(m.x, m.y)
        m.path = [waypoint]
        m.idx = 0
        near = math.dist((m.x, m.y), self.site.waypoint_pose(waypoint))
        m.phase = PARKED if (waypoint in self.site.parking
                             and near <= self.params.arrival_tolerance_m) else IDLE
        if m.fault_reason == PLACEMENT_CONFLICT:
            m.fault = ""
            m.fault_reason = ""

    def _count_reports_outside(self, m: _Mission) -> None:
        d = self.params.release_distance_m
        for cell_id in list(self.zones.held_set(m.name)):
            if self.floor.box_distance(self.floor.cell_at(cell_id), m.x, m.y) > d:
                m.outside[cell_id] = m.outside.get(cell_id, 0) + 1
            else:
                m.outside.pop(cell_id, None)

    def _watch_lane_error(self, m: _Mission) -> None:
        pass  # no lanes; the cell check keeps the body inside its cells

    def _note_block_report(self, m: _Mission) -> None:
        pass  # P3

    def _queue_rests(self, m: _Mission, lane=None) -> bool:
        m.queue_since = None
        return False

    # ---- tick ---------------------------------------------------------------------------

    def tick_progress(self, now: float) -> frozenset[str]:
        for m in self._missions.values():
            if m.available:
                self._follow(m)
                self._release_cells(m)
                self._watch_stall(m, now)
            else:
                m.stall_from = m.stall_mean = None
                m.stall_hold = False
        return frozenset()

    def _follow(self, m: _Mission) -> None:
        """경로 위 진행 s를 보고 자세로 넘긴다. 뒤로는 가지 않고 보낸 끝을 넘지 않는다."""
        if m.goal is None and m.robot_state not in ON_A_GOAL:
            self._active.pop(m.name, None)  # it stands: nothing old is followed any more
        leg = self._legs.get(m.name)
        if leg is None or leg.route is None:
            return
        s, _ = leg.route.project(m.x, m.y, leg.s, max(leg.s, leg.s_sent))
        leg.s = max(leg.s, s)
        short = (leg.s_lock < leg.route.length - AT_END_M
                 and leg.s_lock - leg.s < self.params.floor_lock_ahead_m / 2)
        if short and m.goal and not leg.slowed:
            self.metrics["slowdowns"] += 1
        leg.slowed = short and (leg.slowed or m.goal is not None)

    def _release_cells(self, m: _Mission) -> None:
        if m.fault == ERROR or m.block_cancel is not None:
            return  # an isolated robot keeps its cells until confirm_clear
        keep = self._body_cells(m.x, m.y)
        leg = self._legs.get(m.name)
        if leg and leg.route and leg.s_lock > leg.s:
            keep |= self._swept(leg.route.between(leg.s, leg.s_lock))
        active = self._active.get(m.name)
        if active:
            s, _ = active.project(m.x, m.y)
            keep |= self._swept(active.between(s, active.length))
        for cell_id in list(self.zones.held_set(m.name)):
            if (cell_id not in keep
                    and m.outside.get(cell_id, 0) >= self.params.release_confirm_samples):
                self.zones.release(m.name, cell_id)
                m.outside.pop(cell_id, None)

    # ---- legs ---------------------------------------------------------------------------

    def _line(self, station: str, way: str) -> str | None:
        name = f"{station.lower()}_{way}"
        return name if name in self.floor.fixed_lines else None

    def _dock_here(self, m: _Mission) -> str | None:
        for station, pose in self.floor.docks.items():
            if math.dist((m.x, m.y), pose) <= self.params.arrival_tolerance_m:
                return station
        return None

    def _start_leg(self, m: _Mission, phase: str, dest: str | None, goal_waypoint,
                   way=None) -> None:
        """지금 자리에서 goal_waypoint까지 새 바닥 경로. 따라가던 MoveTo는 교체될 때까지 쥔다."""
        self.zones.cancel_all_requests(m.name)
        here = self._dock_here(m)
        station = next((s for s, st in self.site.stations.items() if st.dock == goal_waypoint),
                       None)
        leg = _Leg(None, self.site.waypoint_pose(goal_waypoint),
                   out_line=self._line(here, "out") if here and here != station else None,
                   in_line=self._line(station, "in") if station else None)
        m.path = [self.site.nearest_waypoint(m.x, m.y), goal_waypoint]
        m.idx = 0
        m.settle_idx = None
        m.phase = phase
        m.dest = dest
        self._legs[m.name] = leg
        self._plan(m, leg)

    def _plan(self, m: _Mission, leg: _Leg) -> None:
        leg.planned_at = self._clock()
        start = (m.x, m.y)
        if math.dist(start, leg.goal_xy) <= self.params.arrival_tolerance_m and not leg.out_line:
            leg.route = Route([leg.goal_xy])
            leg.done = m.goal is None
            return
        route = self.planner.leg(start, leg.goal_xy, leg.out_line, leg.in_line)
        if route is None:
            self.metrics["no_path"] += 1
            if not leg.no_path_warned:
                leg.no_path_warned = True
                self._event("WARN", "FLOOR_NO_PATH",
                            f"no floor path from ({m.x:.2f}, {m.y:.2f}) to {m.path[-1]}", m.name)
            return
        leg.route = route
        leg.s = leg.s_lock = leg.s_sent = 0.0

    def _drive(self, m: _Mission) -> None:
        leg = self._legs.get(m.name)
        if leg is None:
            return
        if m.phase == TO_PARK and self._spot_taken(m):
            self._start_park_leg(m)  # someone stopped on the spot first: pick again
            leg = self._legs[m.name]
        if leg.route is None:
            if self._clock() - leg.planned_at >= self.params.floor_replan_s:
                self._plan(m, leg)
            return
        if leg.done and m.goal is None:
            del self._legs[m.name]
            m.idx = len(m.path) - 1
            self._arrive(m)
            return
        if m.block or m.block_cancel is not None:
            return
        if m.goal is None and self._clock() < m.retry_at:
            return
        self._extend(m, leg)
        self._send(m, leg)

    def _spot_taken(self, m: _Mission) -> bool:
        holder = self.zones.holder(self._zone(m.path[-1]))
        return holder is not None and holder != m.name

    def _candidates(self, leg: _Leg) -> list[float]:
        """Lock ends to try, best first. Inside a fixed line: its end, else stop before it."""
        route = leg.route
        want = min(route.length, leg.s + self.params.floor_lock_ahead_m)
        ends, e = [], want
        while e > leg.s_lock + 1e-9:
            ends.append(e)
            e -= self.params.floor_extend_step_m
        out: list[float] = []
        for e in ends:
            for s0, s1, _ in route.fixed:
                if s0 + 1e-9 < e < s1 - 1e-9 and leg.s_lock < s1 - 1e-9:
                    picks = [s1] + ([s0] if s0 > leg.s_lock + 1e-9 else [])
                    break
            else:
                picks = [e]
            out.extend(p for p in picks if p not in out)
        return out

    def _extend(self, m: _Mission, leg: _Leg) -> bool:
        for end in self._candidates(leg):
            cells = self._swept(leg.route.between(leg.s, end)) | self._body_cells(m.x, m.y)
            if self.zones.acquire_all(m.name, cells):
                leg.s_lock = end
                return True
        return False

    def _send(self, m: _Mission, leg: _Leg) -> None:
        route = leg.route
        end = leg.s_lock
        final = end >= route.length - AT_END_M
        if m.goal is None:
            if end - leg.s <= 0.01 and not final:
                return
            if final and math.dist((m.x, m.y), route.points[-1]) <= 0.01 and leg.s_sent >= end:
                leg.done = True
                return
        elif m.goal.kind != MOVE:
            return
        elif end - leg.s_sent < self.params.floor_extend_step_m - 1e-9 and not (
                final and leg.s_sent < end - AT_END_M):
            return
        ahead = route.between(min(leg.s + 0.05, end), end) if end - leg.s > 0.05 else [
            route.at(end)]
        points = [(m.x, m.y)] + [p for p in ahead if math.dist(p, (m.x, m.y)) > 1e-9]
        if len(points) == 1:
            points.append(route.at(end))
        poses = []
        for k, (x, y) in enumerate(points):
            a, b = (points[k], points[k + 1]) if k + 1 < len(points) else (points[k - 1],
                                                                            points[k])
            poses.append((x, y, math.atan2(b[1] - a[1], b[0] - a[0])))
        target = m.path[-1] if final else f"{m.path[-1]}~{end:.2f}"
        goal_id = self._next_goal_id()
        m.goal = _Goal(MOVE, goal_id, 1 if final else 0)
        m.stall_hold = False
        leg.s_sent = end
        self._active[m.name] = Route(points)
        x, y, yaw = poses[-1]
        self._out.append(MoveCmd(m.name, goal_id, target, x, y, yaw, tuple(poses)))

    def _step_parked(self, m: _Mission) -> None:
        """대기 입구가 비도록 빈 지점으로 계속 앞으로 간다."""
        if self._run_mode == RUN and self._park_goal(m, m.path[m.idx]):
            self._start_park_leg(m)

    # ---- goal results -------------------------------------------------------------------

    def on_move_result(self, robot, goal_id, success, reason) -> None:
        m = self._missions.get(robot)
        if m:
            self._cancel_finished(m, goal_id)
        goal = m.goal if m else None
        if goal is None or goal.id != goal_id or goal.kind != MOVE:
            return
        m.goal = None
        m.not_sent_warned = False
        m.last_result = "SUCCEEDED" if success else reason
        self._active.pop(robot, None)
        leg = self._legs.get(robot)
        if success:
            m.attempts = 0
            if leg and leg.route and leg.s_sent >= leg.route.length - AT_END_M:
                leg.done = True
        elif reason == HEARTBEAT_LOST:
            m.retry_at = self._clock() + self.params.move_retry_wait_s
        else:
            self._goal_failed(m, reason)

    def on_goal_rejected(self, robot, goal_id) -> None:
        super().on_goal_rejected(robot, goal_id)
        if robot in self._missions and self._missions[robot].goal is None:
            self._active.pop(robot, None)

    def on_goal_not_sent(self, robot, goal_id) -> None:
        super().on_goal_not_sent(robot, goal_id)
        if robot in self._missions and self._missions[robot].goal is None:
            self._active.pop(robot, None)
