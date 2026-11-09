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

여러 대(P3). 안전은 칸 잠금, 효율은 계획:
  - 계획은 남이 쥔 칸을 피하고, 남의 예약(경로를 계약 속도로 달릴 때 칸에 머무는 시간,
    floor_plan_window_s 앞까지)과 겹치는 칸에 벌점을 준다(시간 벌점 A*, 협조 A*의 가벼운 꼴).
  - 잠금이 늘지 않아 서게 되면 floor_replan_s마다 다시 계획한다(감속하는 동안).
  - 대기 그래프에 순환이 deadlock_grace_s 이어지면 우선순위가 낮은 로봇이 옆 빈 칸으로 비켜 선다.
    비켜 설 곳이 없으면 DEADLOCK_UNRESOLVED, 그 뒤 deadlock_unresolved_s 이어지면 격리한다.
  - 도크 접근선은 한 번에 한 대: 다른 로봇이 그 선으로 가는 중이거나 선 칸을 쥐고 있으면
    고정 선 칸에 닿지 않는 빈 자리에서 기다리고, 선이 비면 먼저 온 로봇부터 들어간다.
  - BLOCKED가 blocked_mark_s 이어지면 앞 칸을 막힘(주인 BLOCK_OWNER)으로 표시하고 목표를 거둔다.
    BLOCKED가 아닌 보고가 release_confirm_samples번 오면 표시를 지우고 다시 계획한다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from fleet_core.common.commands import MoveCmd
from fleet_core.traffic.controller import TrafficController
from fleet_core.traffic.deadlock import choose_retreat, find_cycle, wait_graph
from fleet_core.traffic.floor.cells import CellLocks, disk_cells, swept_cells
from fleet_core.traffic.floor.floor_map import load_floor_map
from fleet_core.traffic.floor.planner import FloorPlanner, Route
from fleet_core.traffic.mission import (
    AT_STATION,
    BLOCKED,
    DOCKING,
    ERROR,
    HEARTBEAT_LOST,
    IDLE,
    MOVE,
    NOT_PLACEABLE,
    ON_A_GOAL,
    PARKED,
    PLACEMENT_CONFLICT,
    RUN,
    TO_OUTBOUND,
    TO_PARK,
    TO_WORK,
    _Goal,
    _Mission,
)

AT_END_M = 1e-6
BLOCK_OWNER = "#blocked"  # pseudo holder of cells in front of an object
RESERVE_PENALTY_M = 0.5  # extra cost of a cell another robot plans to be in at that time
SIDESTEP_REACH_M = 1.2  # how far a robot looks for a free spot to step aside to
SIDESTEP_HOLD_S = 3.0  # how long it stays aside before planning its leg again
PLANS_PER_TICK = 1  # one plan can take ~10 ms: the other robots plan on the next ticks
WAIT_SPOT_M = (0.3, 1.2)  # a robot waiting for an approach line stands this far from its start


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
    stuck_since: float | None = None  # the lock could not grow since then
    resume: tuple | None = None  # sidestep leg: (phase, dest, goal waypoint) to go on with
    resume_at: float | None = None  # sidestep done: plan the real leg again from then
    waiting: bool = False  # another robot has the approach line: the route ends at a wait spot
    asked_at: float = 0.0  # when this leg asked for its approach line (first come first served)


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
        # robot -> goal id -> path of every MoveTo the robot may still follow (until its result)
        self._active: dict[str, dict[int, Route]] = {name: {} for name in robots}
        self._off_cells: dict[str, int] = {name: 0 for name in robots}
        self._wp_cell: dict = {}
        self.metrics = {"slowdowns": 0, "replans": 0, "sidesteps": 0, "no_path": 0}
        self._block_since: dict[str, float] = {}  # robot -> first BLOCKED report of this run
        self._clear_n: dict[str, int] = {}  # robot -> non-BLOCKED reports since its mark
        self._marks: dict[str, set[str]] = {}  # robot -> cells it marked as blocked
        self._ahead: dict[str, set[str]] = {}  # robot -> cells to mark once its goal has ended
        self._deadlock_key: tuple = ()
        # cell offsets whose box comes nearer than the body radius to a cell's box
        step = self.floor.cell_m
        n = math.ceil(self._body / step) + 1
        self._reach = [(di, dj) for di in range(-n, n + 1) for dj in range(-n, n + 1)
                       if math.hypot(max(0, abs(di) - 1) * step,
                                     max(0, abs(dj) - 1) * step) < self._body]
        self._line_ids = {name: self._swept(line.points)
                          for name, line in self.floor.fixed_lines.items()}
        self._all_line_ids = set().union(*self._line_ids.values()) if self._line_ids else set()
        self._deadlock_seen = 0.0
        self._unresolved_robot = ""
        self._plan_budget = PLANS_PER_TICK

    # ---- cells --------------------------------------------------------------------------

    def _ids(self, cells) -> set[str]:
        return {self.floor.cell_id(c) for c in cells}

    def _body_cells(self, x: float, y: float) -> set[str]:
        return self._ids(disk_cells(self.floor, x, y, self._body))

    def _standing_cells(self, x: float, y: float) -> set[str]:
        """로봇이 서 있는 동안 쥐는 칸: 몸 칸과 아직 release_distance_m 안이라 놓지 않는 칸."""
        return self._ids(disk_cells(self.floor, x, y,
                                    max(self._body, self.params.release_distance_m)))

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
        """보고 자리의 몸 칸을 쥐었나 본다. 아니면 한꺼번에 잡고, 못 잡는 보고가
        release_confirm_samples번 이어지면 세운다(레인 이탈 감시 대신)."""
        if not m.available:
            return
        need = self._body_cells(m.x, m.y)
        if need <= self.zones.held_set(m.name) or self.zones.acquire_all(m.name, need):
            self._off_cells[m.name] = 0
            return
        self._off_cells[m.name] += 1
        if self._off_cells[m.name] >= self.params.release_confirm_samples and m.goal:
            self._cancel_goal(m)
            self._event("WARN", "FLOOR_OFF_CELLS",
                        f"reported at ({m.x:.2f}, {m.y:.2f}) on cells held by others, stopped",
                        m.name)

    def _note_block_report(self, m: _Mission) -> None:
        """BLOCKED가 blocked_mark_s 이어지면 앞 칸을 막힘으로 표시한다. 풀리면 지운다."""
        now = self._clock()
        name = m.name
        if m.robot_state == "BLOCKED":
            self._clear_n[name] = 0
            since = self._block_since.setdefault(name, now)
            self._take_ahead(m)
            if name not in self._marks and now - since >= self.params.blocked_mark_s:
                self._mark_blocked(m)
            elif now - since >= self.params.blocked_timeout_s and not m.fault:
                self._fail(m, "BLOCKED")
            return
        self._block_since.pop(name, None)
        if name in self._marks:
            self._clear_n[name] = self._clear_n.get(name, 0) + 1
            if self._clear_n[name] >= self.params.release_confirm_samples:
                self._ahead.pop(name, None)
                for cell in self._marks.pop(name):
                    self.zones.release(BLOCK_OWNER, cell)
                self._event("INFO", "FLOOR_CLEARED", "the object is gone", name)
                leg = self._legs.get(name)
                if leg is not None and leg.route is not None:
                    self._plan(m, leg)

    def _mark_blocked(self, m: _Mission) -> None:
        leg = self._legs.get(m.name)
        ahead = set()
        if leg and leg.route:
            ahead = self._swept(leg.route.between(leg.s, leg.s + 0.3))
        ahead -= self._body_cells(m.x, m.y)
        if m.goal:
            self._cancel_goal(m)
        # cells the robot holds stay its own until the cancelled goal has ended (it may still
        # follow it until the result): _take_ahead moves them to the mark then
        free = {c for c in ahead if self.zones.holder(c) is None}
        self.zones.acquire_all(BLOCK_OWNER, free)
        self._marks[m.name] = free
        self._ahead[m.name] = ahead - free
        if leg and leg.route:
            leg.s_lock = leg.s_sent = leg.s
        self._event("WARN", "FLOOR_BLOCKED",
                    f"object ahead, {len(free)} cells marked blocked", m.name)

    def _take_ahead(self, m: _Mission) -> None:
        """표시할 앞 칸 중 로봇이 쥐고 있던 칸: goal이 모두 끝난 뒤에야 막힘 표시로 옮긴다."""
        ahead = self._ahead.get(m.name)
        if not ahead or m.goal is not None or self._active[m.name]:
            return
        body = self._body_cells(m.x, m.y)
        for cell in sorted(ahead):
            holder = self.zones.holder(cell)
            if cell in body or holder not in (None, m.name):
                continue
            if holder == m.name:
                self.zones.release(m.name, cell)
            if self.zones.acquire_all(BLOCK_OWNER, {cell}):
                self._marks[m.name].add(cell)
        self._ahead.pop(m.name)

    def blocked_cells(self) -> list[str]:
        return self.zones.held_by(BLOCK_OWNER)

    def _held_for_block(self, m: _Mission) -> bool:
        """막힌 동안과 표시를 지우기 전에는 새 goal을 내지 않는다."""
        return m.name in self._block_since or m.name in self._marks

    def _queue_rests(self, m: _Mission, lane=None) -> bool:
        m.queue_since = None
        return False

    # ---- tick ---------------------------------------------------------------------------

    def tick_progress(self, now: float) -> frozenset[str]:
        self._plan_budget = PLANS_PER_TICK
        for m in self._missions.values():
            if m.available:
                self._follow(m)
                self._release_cells(m)
                self._watch_stall(m, now)
            else:
                m.stall_from = m.stall_mean = None
                m.stall_hold = False
        return self._check_deadlock(now)

    def _follow(self, m: _Mission) -> None:
        """경로 위 진행 s를 보고 자세로 넘긴다. 뒤로는 가지 않고 보낸 끝을 넘지 않는다."""
        if m.goal is None and m.robot_state not in ON_A_GOAL + (BLOCKED,):
            # it stands: nothing old is followed any more (BLOCKED only pauses the goal)
            self._active[m.name].clear()
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
        for active in self._active[m.name].values():
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
        leg.asked_at = self._clock()
        self._legs[m.name] = leg
        self._plan(m, leg)

    def _plan(self, m: _Mission, leg: _Leg) -> None:
        leg.done = False
        if self._plan_budget <= 0:
            leg.route = None  # planned on the next tick
            leg.planned_at = self._clock() - self.params.floor_replan_s
            leg.waiting = bool(leg.in_line)  # no claim on an approach line before a plan
            if m.goal:
                self._cancel_goal(m)  # stop meanwhile, not on to the old target
            return
        self._plan_budget -= 1
        leg.planned_at = self._clock()
        start = (m.x, m.y)
        out_line = leg.out_line
        if out_line and math.dist(start, self.floor.fixed_lines[out_line].start) > \
                self.params.arrival_tolerance_m:
            out_line = None  # no longer on the dock (moved, or replanned on the way)
        leg.waiting = bool(leg.in_line) and not self._admitted(m, leg)
        if leg.waiting:
            self._plan_wait(m, leg, out_line)
            return
        if math.dist(start, leg.goal_xy) <= self.params.arrival_tolerance_m and not out_line:
            leg.route = Route([leg.goal_xy])
            leg.done = m.goal is None
            return
        route = self._route(m, start, leg.goal_xy, out_line, leg.in_line)
        if route is None:
            self.metrics["no_path"] += 1
            leg.route = None  # never drive on to the end of an old route (a wait spot)
            if not leg.no_path_warned:
                leg.no_path_warned = True
                self._event("WARN", "FLOOR_NO_PATH",
                            f"no floor path from ({m.x:.2f}, {m.y:.2f}) to {m.path[-1]}", m.name)
            return
        leg.route = route
        leg.s = leg.s_lock = leg.s_sent = 0.0
        leg.stuck_since = None

    # ---- one robot at a time on an approach line -------------------------------------------

    def _admitted(self, m: _Mission, leg: _Leg) -> bool:
        """leg가 접근선에 들어가도 되나: 다른 로봇이 그 선으로 가는 중이 아니고 선 칸을 쥐지 않았고
        먼저 기다리기 시작한 로봇이 없을 때."""
        line = leg.in_line
        cells = self._line_ids[line]
        for name in self._missions:
            if name == m.name:
                continue
            if not cells.isdisjoint(self.zones.held_set(name)):
                return False
            other = self._legs.get(name)
            if (other is not None and other.in_line == line and not other.waiting
                    and self._in_service(name)):
                return False
        mine = (leg.asked_at, m.name)
        return not any((other.asked_at, name) < mine for name, other in self._legs.items()
                       if name != m.name and other.in_line == line and other.waiting
                       and self._in_service(name))

    def _in_service(self, name: str) -> bool:
        """격리·제거되지 않은 로봇. 격리·제거된 로봇의 다리는 줄을 잡지 않는다."""
        other = self._missions[name]
        return other.live and other.fault != ERROR  # out of contact keeps its turn

    def _plan_wait(self, m: _Mission, leg: _Leg, out_line) -> None:
        """접근선 시작 근처, 고정 선 칸·남의 칸·다른 대기 자리에 닿지 않는 가장 가까운 빈 자리로."""
        start = (m.x, m.y)
        line_start = self.floor.fixed_lines[leg.in_line].points[0]
        banned = set(self._all_line_ids) | set(self.zones.held_by_others(m.name))
        for name, other in self._legs.items():
            if name != m.name and other.waiting and other.route is not None:
                banned |= self._standing_cells(*other.route.points[-1])
        near, far = WAIT_SPOT_M
        if not out_line and self._standing_cells(*start).isdisjoint(banned) and \
                near <= math.dist(start, line_start) <= far:
            leg.route = Route([start])  # already standing in a good place
            if m.goal:
                self._cancel_goal(m)  # stop here, not at the old leg's target
            leg.done = True
            leg.s = leg.s_lock = leg.s_sent = 0.0
            return
        spots = sorted((c for c in self.floor.clear
                        if near <= math.dist(line_start, self.floor.centre(c)) <= far),
                       key=lambda c: math.dist(line_start, self.floor.centre(c)))
        for cell in spots:
            spot = self.floor.centre(cell)
            if not self._standing_cells(*spot).isdisjoint(banned):
                continue
            route = self._route(m, start, spot, out_line)
            if route is None:
                continue
            leg.route = route
            leg.s = leg.s_lock = leg.s_sent = 0.0
            leg.stuck_since = None
            return
        leg.route = None  # nowhere to wait now: try again after floor_replan_s

    def _route(self, m: _Mission, start, goal, out_line=None, in_line=None, extra=frozenset()):
        """남이 쥔 칸을 피하고 남의 예약에 벌점을 준 경로. 없으면 피하지 않고 다시 찾는다."""
        avoid = self._inflate(self._others_cells(m.name)) | extra
        avoid -= {self.floor.cell_of(*start)}
        route = self.planner.leg(start, goal, out_line, in_line, frozenset(avoid),
                                 self._penalty_for(m.name))
        if route is None and not extra:
            route = self.planner.leg(start, goal, out_line, in_line)
        return route

    def _inflate(self, cells) -> set:
        """칸 (i, j)들 -> 로봇 중심이 그 칸 어디에 있어도 몸이 cells 중 하나에 닿는 칸까지 넓힌 것.
        계획기는 중심이 지나는 칸만 보므로 남의 칸을 피하려면 몸 반지름만큼 넓혀서 넘긴다."""
        out = set()
        for i, j in cells:
            out.update((i + di, j + dj) for di, dj in self._reach)
        return out

    def _others_cells(self, robot: str) -> set:
        """다른 로봇(과 막힘 표시)이 쥔 칸 (i, j)."""
        return {self.floor.cell_at(c) for c in self.zones.held_by_others(robot)}

    def _reservations(self, exclude: str | None = None) -> dict[str, list]:
        """칸 -> [(로봇, 들어가는 시각, 나가는 시각)]: 계획한 경로를 계약 속도로 달릴 때."""
        v = self.params.detour_speed_mps
        window = self.params.floor_plan_window_s
        hold = self._body / v
        table: dict[str, dict[str, list]] = {}
        for name, leg in self._legs.items():
            if name == exclude or leg.route is None or not self._missions[name].available:
                continue
            route = leg.route
            end = min(route.length, leg.s + v * window)
            n = max(1, int((end - leg.s) / (self.floor.cell_m / 2)))
            for k in range(n + 1):
                s = leg.s + (end - leg.s) * k / n
                t = (s - leg.s) / v
                for cell in self._body_cells(*route.at(s)):
                    span = table.setdefault(cell, {}).setdefault(name, [t - hold, t + hold])
                    span[0], span[1] = min(span[0], t - hold), max(span[1], t + hold)
        return {cell: [(name, a, b) for name, (a, b) in spans.items()]
                for cell, spans in table.items()}

    def _penalty_for(self, robot: str):
        """계획기 벌점: 다른 로봇이 그 칸에 있을 시각에 닿으면 RESERVE_PENALTY_M."""
        table = self._reservations(exclude=robot)
        v = self.params.detour_speed_mps
        ids = self.floor.cell_id

        def penalty(cell, s: float) -> float:
            t = s / v
            for _, t_in, t_out in table.get(ids(cell), ()):
                if t_in <= t <= t_out:
                    return RESERVE_PENALTY_M
            return 0.0

        return penalty

    def _drive(self, m: _Mission) -> None:
        leg = self._legs.get(m.name)
        if leg is None:
            return
        if m.phase == TO_PARK and self._spot_taken(m):
            self._start_park_leg(m)  # someone stopped on the spot first: pick again
            leg = self._legs[m.name]
        if leg.waiting and self._admitted(m, leg):
            if m.goal:
                self._cancel_goal(m)
            self.zones.cancel_all_requests(m.name)
            self._plan(m, leg)  # its turn on the approach line
        if leg.route is None:
            if self._clock() - leg.planned_at >= self.params.floor_replan_s:
                self._plan(m, leg)
            return
        if leg.done and m.goal is None:
            if leg.waiting:
                return  # at the wait spot until the line is free
            if leg.resume is not None:
                self._after_sidestep(m, leg)
                return
            del self._legs[m.name]
            m.idx = len(m.path) - 1
            self._arrive(m)
            return
        if m.goal is None and self._clock() < m.retry_at:
            return
        grew = self._extend(m, leg)
        self._send(m, leg)
        self._replan_if_stuck(m, leg, grew)

    def _replan_if_stuck(self, m: _Mission, leg: _Leg, grew: bool) -> None:
        """잠금이 floor_replan_s 동안 못 늘고 앞이 짧으면 남을 피해 다시 계획한다."""
        now = self._clock()
        route = leg.route
        short = leg.s_lock - leg.s < self.params.floor_extend_step_m
        if grew or not short or leg.s_lock >= route.length - AT_END_M:
            leg.stuck_since = None
            return
        if leg.stuck_since is None:
            leg.stuck_since = now
            return
        if (now - leg.stuck_since < self.params.floor_replan_s
                or now - leg.planned_at < self.params.floor_replan_s
                or route.fixed_at(leg.s) is not None):
            return  # inside a fixed line there is no other way
        if leg.waiting:
            if m.goal:
                self._cancel_goal(m)
            self.zones.cancel_all_requests(m.name)
            self._plan(m, leg)  # another wait spot (or the line, if it is free now)
            self.metrics["replans"] += 1
            return
        if self._plan_budget <= 0:
            return  # try on the next tick
        self._plan_budget -= 1
        leg.planned_at = now
        old_left = route.length - leg.s
        new = self._route(m, (m.x, m.y), leg.goal_xy, None, leg.in_line,
                          extra=frozenset(self._others_cells(m.name)))
        if new is None or new.length > old_left + 3.0:
            return
        if m.goal:
            self._cancel_goal(m)
        self.zones.cancel_all_requests(m.name)
        leg.route = new
        leg.s = leg.s_lock = leg.s_sent = 0.0
        leg.stuck_since = None
        self.metrics["replans"] += 1

    # ---- wait cycles ----------------------------------------------------------------------

    def _check_deadlock(self, now: float) -> frozenset[str]:
        """대기 순환을 찾는다. 아무도 세우지 않는다: 순환에 든 로봇도 다시 계획을 이어 간다.

        deadlock_grace_s 이어지면 choose_retreat가 고른 로봇이 비켜 선다. 비켜 설 곳이 없거나
        비켜 선 뒤에도 같은 순환이 deadlock_unresolved_s 이어지면 그 로봇을 격리한다.
        """
        graph = wait_graph(self.zones)
        graph.pop(BLOCK_OWNER, None)
        cycle = find_cycle(graph)
        if not cycle:
            # a robot that got a shorter extension stops waiting for a tick: the same cycle
            # coming back within a second is the same episode
            if self._deadlock_key and now - self._deadlock_seen > 1.0:
                self._deadlock_key = ()
            return frozenset()
        members, key = set(cycle), set(self._deadlock_key)
        same = now - self._deadlock_seen <= 1.0 and (
            members <= key or (members & key and not self._deadlock_acted))
        self._deadlock_seen = now
        if same:
            # robots that take turns getting a shorter lock change the cycle inside the same
            # group of robots: one episode over all of them
            self._deadlock_key = tuple(sorted(members | key))
        else:
            self._deadlock_key = tuple(sorted(cycle))
            self._deadlock_since = now
            self._deadlock_acted = False
            self._unresolved_at = None
            self._event("ERROR", "DEADLOCK_DETECTED", f"wait cycle {cycle}")
            return frozenset()
        if not self._deadlock_acted and now - self._deadlock_since >= \
                self.params.deadlock_grace_s:
            self._deadlock_acted = True
            # lowest priority first, as choose_retreat; one that cannot step aside (on a fixed
            # line, docking) passes the turn to the next
            turns = sorted(cycle, key=lambda name: (self._priority(name), name))
            if not any(self._sidestep(self._missions[name], cycle) for name in turns):
                robot = choose_retreat(cycle, self._priority)
                self._event("ERROR", "DEADLOCK_UNRESOLVED",
                            f"{robot} of {cycle} has no free spot to step aside to", robot)
                self._unresolved_at = now
                self._unresolved_robot = robot
        elif (self._deadlock_acted and self._unresolved_at is None
              and now - self._deadlock_since >= 2 * self.params.deadlock_grace_s + SIDESTEP_HOLD_S
              and not any(self._legs[name].resume for name in cycle if name in self._legs)):
            robot = choose_retreat(cycle, self._priority)
            self._event("ERROR", "DEADLOCK_UNRESOLVED",
                        f"{robot} of {cycle} stepped aside but the cycle remains", robot)
            self._unresolved_at = now
            self._unresolved_robot = robot
        elif (self._unresolved_at is not None
              and now - self._unresolved_at >= self.params.deadlock_unresolved_s):
            self._unresolved_at = None
            if self._unresolved_robot in cycle:
                self._fail(self._missions[self._unresolved_robot], "DEADLOCK_UNRESOLVED")
            else:
                # the robot found unresolved has left the cycle: judge the cycle as it is now
                self._deadlock_since = now
                self._deadlock_acted = False
        return frozenset()

    def _sidestep(self, m: _Mission, cycle) -> bool:
        """남의 칸과 남이 가려는 길에서 벗어난 가장 가까운 빈 자리로 비켜 선다."""
        if not m.available or m.phase not in (TO_WORK, TO_OUTBOUND, TO_PARK):
            return False
        leg = self._legs.get(m.name)
        if leg is None or leg.route is None or leg.route.fixed_at(leg.s) is not None:
            return False
        others = self._others_cells(m.name)
        wanted = set()
        for name in cycle:
            other = self._legs.get(name)
            if name != m.name and other is not None and other.route is not None:
                ahead = other.route.between(other.s, min(other.route.length, other.s + 1.2))
                wanted |= swept_cells(self.floor, ahead, self._body)
        blocked = others | wanted
        here = (m.x, m.y)
        avoid_ids = self._inflate(others) | wanted
        spots = sorted((c for c in self.floor.clear
                        if 0.25 <= math.dist(here, self.floor.centre(c)) <= SIDESTEP_REACH_M),
                       key=lambda c: math.dist(here, self.floor.centre(c)))
        for cell in spots:
            spot = self.floor.centre(cell)
            if disk_cells(self.floor, *spot,
                          max(self._body, self.params.release_distance_m)) & blocked:
                continue
            route = self.planner.leg(here, spot, avoid=frozenset(avoid_ids - {self.floor.cell_of(
                *here)}))
            if route is None:
                continue
            if m.goal:
                self._cancel_goal(m)
            self.zones.cancel_all_requests(m.name)
            goal = m.path[-1]
            side = _Leg(route, spot, resume=(m.phase, m.dest, goal))
            self._legs[m.name] = side
            self.metrics["sidesteps"] += 1
            self._event("WARN", "FLOOR_SIDESTEP",
                        f"steps aside to ({spot[0]:.2f}, {spot[1]:.2f}) to break {cycle}",
                        m.name)
            return True
        return False

    def _after_sidestep(self, m: _Mission, leg: _Leg) -> None:
        now = self._clock()
        if leg.resume_at is None:
            leg.resume_at = now + SIDESTEP_HOLD_S
        if now < leg.resume_at:
            return
        phase, dest, goal = leg.resume
        self._start_leg(m, phase, dest, goal)

    def _spot_taken(self, m: _Mission) -> bool:
        leg = self._legs.get(m.name)
        if leg is not None and leg.resume is not None:
            return False  # stepping aside: the spot is not where it goes now
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
            cells = (self._swept(leg.route.between(leg.s, end)) | self._body_cells(m.x, m.y)
                     | self._swept(self._drive_path(m, leg, end)))
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
        points = self._drive_path(m, leg, end)
        if not self.zones.acquire_all(m.name, self._swept(points) | self._body_cells(m.x, m.y)):
            return  # the first stretch from the reported pose is not free: wait
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
        self._active[m.name][goal_id] = Route(points)
        x, y, yaw = poses[-1]
        # final_goal: only the move that ends at the route's last point; every partial move
        # that ends at the rolling lock boundary says False (contract 0.8-draft proposal)
        self._out.append(MoveCmd(m.name, goal_id, target, x, y, yaw, tuple(poses),
                                 final or not self.params.mark_final_goal))

    @staticmethod
    def _drive_path(m: _Mission, leg: _Leg, end: float) -> list:
        """보낼 꺾은선: 보고 자리에서 경로 위 조금 앞(되돌아가지 않게)을 지나 end까지."""
        route = leg.route
        ahead = route.between(min(leg.s + 0.05, end), end) if end - leg.s > 0.05 else [
            route.at(end)]
        points = [(m.x, m.y)] + [p for p in ahead if math.dist(p, (m.x, m.y)) > 1e-9]
        if len(points) == 1:
            points.append(route.at(end))
        return points

    def _step_parked(self, m: _Mission) -> None:
        """대기 입구가 비도록 빈 지점으로 계속 앞으로 간다."""
        if self._run_mode == RUN and self._park_goal(m, m.path[m.idx]):
            self._start_park_leg(m)

    # ---- goal results -------------------------------------------------------------------

    def on_move_result(self, robot, goal_id, success, reason) -> None:
        m = self._missions.get(robot)
        if m:
            self._cancel_finished(m, goal_id)
            self._active[robot].pop(goal_id, None)  # any goal, old ones too: it has ended
        goal = m.goal if m else None
        if goal is None or goal.id != goal_id or goal.kind != MOVE:
            return
        m.goal = None
        m.not_sent_warned = False
        m.last_result = "SUCCEEDED" if success else reason
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
        if robot in self._active:
            self._active[robot].pop(goal_id, None)

    def on_goal_not_sent(self, robot, goal_id) -> None:
        super().on_goal_not_sent(robot, goal_id)
        if robot in self._active:
            self._active[robot].pop(goal_id, None)

    def on_goal_lost(self, robot, goal_id) -> None:
        super().on_goal_lost(robot, goal_id)
        if robot in self._active:
            self._active[robot].pop(goal_id, None)

    # ---- link recovery ------------------------------------------------------------------

    def _finish_recovery(self, m: _Mission) -> None:
        """통신이 돌아왔고 자세를 확인했다. 지금 몸 칸을 잡고 하던 다리를 그 자리에서 다시 잡는다.

        레인판은 지점 하나의 구역만 다시 잡고 쥔 구역을 레인 규칙으로 놓는다. 여기서는 칸을
        놓는 일을 보통 해제 규칙(_release_cells)에 맡긴다.
        """
        m.comm = ""
        need = self._body_cells(m.x, m.y)
        if not self.zones.acquire_all(m.name, need):
            in_the_way = sorted({self.zones.holder(c) for c in need} - {None, m.name})
            self._fail(m, "RECOVERY_CONFLICT")
            self._event("ERROR", "RECOVERY_CONFLICT",
                        f"{m.name} stands on cells held by {in_the_way}", m.name)
            return
        m.path = [self.site.nearest_waypoint(m.x, m.y), *m.path[-1:]]
        m.idx = 0
        m.settle_idx = None
        leg = self._legs.get(m.name)
        if leg is not None and m.phase in (TO_PARK, TO_WORK, TO_OUTBOUND):
            self._plan(m, leg)  # from where it stands now, same goal
            return
        here = self._dock_here(m)
        if m.phase == AT_STATION or (m.phase == DOCKING and here == m.dest):
            return  # still on its dock, as the lane recovery keeps it
        if m.dispatch:
            self._start_dispatch_leg(m)
            return
        m.path = [m.path[0]]
        near = math.dist((m.x, m.y), self.site.waypoint_pose(m.path[0]))
        m.phase = PARKED if (m.path[0] in self.site.parking
                             and near <= self.params.arrival_tolerance_m) else IDLE
        m.dest = None
