"""FleetCore: the mission logic of the fleet manager. No ROS and no wall clock in here.

Callers feed robot, arm and order board updates through the on_* methods and call tick()
periodically. tick() returns the commands to send and the events to publish.

Safety rules kept here, each backed by a test:
  - A robot holds its current zone and at most one next zone, and moves only inside them.
  - A zone is released only after consecutive reported poses are outside it by the margin,
    so one noisy pose cannot free a zone the robot's body is still next to.
  - Results of goals that are no longer the robot's active goal are ignored.
  - Nothing moves until every robot has reported, because a robot that has not reported may
    be standing in the zone another one would be given.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from fleet_core.commands import (
    HELD,
    PARTIAL,
    SHIPPED,
    AssignmentView,
    CancelCmd,
    DockCmd,
    EventOut,
    FillCmd,
    MoveCmd,
    ReportOrderCmd,
    SwapCmd,
    ZoneView,
)
from fleet_core.deadlock import choose_retreat, find_cycle, priority, wait_graph
from fleet_core.dispatcher import Dispatcher, RobotInfo
from fleet_core.zones import ZoneManager

# Mission phases.
IDLE = "IDLE"
PARKED = "PARKED"
TO_PARK = "TO_PARK"
TO_WORK = "TO_WORK"
TO_OUTBOUND = "TO_OUTBOUND"
DOCKING = "DOCKING"
FILLING = "FILLING"
SWAPPING = "SWAPPING"
# Fault states shown instead of the phase.
ERROR = "ERROR"
OUT_OF_SERVICE = "OUT_OF_SERVICE"
COMM_LOST = "COMM_LOST"
RECOVERING = "RECOVERING"

RUN = "RUN"
STOP = "STOP"

MOVE = "move"
DOCK = "dock"

# Station actions, as written in the site map.
FILL = "fill"
SWAP = "swap"

# What is on the tray.
EMPTY = "EMPTY"
FULL = "FULL"
NONE = "NONE"

CANCELED = "CANCELED"


def _merge(into: dict[str, int], more: dict[str, int]) -> None:
    for kind, count in more.items():
        if count > 0:
            into[kind] = into.get(kind, 0) + count


def _subtract(items: dict[str, int], *taken: dict[str, int]) -> dict[str, int]:
    left = {}
    for kind, count in items.items():
        rest = count - sum(t.get(kind, 0) for t in taken)
        if rest > 0:
            left[kind] = rest
    return left


@dataclass
class _Goal:
    kind: str
    id: int
    target_idx: int = 0  # move only: index into the mission path
    step: tuple[int, int] | None = None  # move only: grid direction of travel


@dataclass
class _ArmJob:
    server: str
    goal_id: int
    kind: str  # FILL or SWAP
    sent_at: float
    canceled_at: float | None = None
    timed_out: bool = False  # canceled because it ran too long, not because of STOP


@dataclass
class _BoundOrder:
    order_id: str
    items: dict[str, int]
    placed: dict[str, int] = field(default_factory=dict)
    missing: dict[str, int] = field(default_factory=dict)  # final: nobody will try these again


@dataclass
class _Mission:
    name: str
    placed: bool = False
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    robot_state: str = ""
    last_seen: float = 0.0
    phase: str = IDLE
    fault: str = ""  # ERROR or OUT_OF_SERVICE, both need the operator
    comm: str = ""  # COMM_LOST or RECOVERING
    recover_at: float = 0.0
    path: list = field(default_factory=list)
    idx: int = 0
    start_flow: str = ""  # lane at path[0]; later cells get theirs from the previous one
    dest: str | None = None  # station it is driving to or docked at
    goal: _Goal | None = None
    settle_idx: int | None = None  # drive here first, so a resumed goal stays on the lane
    attempts: int = 0
    basket: str = EMPTY
    order: _BoundOrder | None = None
    last_order_id: str = ""  # names the swap goal after the order was already reported
    called: bool = False
    arm: _ArmJob | None = None
    arm_retry_at: float = 0.0  # no new arm goal before this, after a failure or a rejection
    outside: dict = field(default_factory=dict)  # zone -> consecutive reports outside the margin
    wait_zone: str | None = None  # zone it has been queueing for since wait_since
    wait_since: float = 0.0
    wait_warned: bool = False

    @property
    def live(self) -> bool:
        return self.placed and not self.fault

    @property
    def available(self) -> bool:
        return self.live and not self.comm

    @property
    def state(self) -> str:
        return self.fault or self.comm or self.phase


class FleetCore:
    def __init__(self, site, params, robots, clock):
        if site.cell_size / 2 <= params.release_margin:
            raise ValueError(
                f"half a cell ({site.cell_size / 2} m) must exceed the release margin "
                f"({params.release_margin} m), or a robot stopped at a cell centre never "
                "releases the zone behind it")
        self.site = site
        self.params = params
        self.zones = ZoneManager(site.zones, clock)
        self.dispatcher = Dispatcher(site, params)
        self.shipped = 0
        self._clock = clock
        self._missions = {name: _Mission(name) for name in robots}
        self._out: list = []
        self._run_mode: str | None = None
        self._board: list = []
        self._reported: set[str] = set()  # the board may still list these for a moment
        self._goal_seq = 0
        self._work = next(s.id for s in site.stations.values() if s.action == FILL)
        self._outbound = next(s.id for s in site.stations.values() if s.action == SWAP)
        self._deadlock_since: float | None = None
        self._deadlock_acted = False
        self._retreating: tuple[str, list[str], float] | None = None  # robot, zones, release time
        self._startup_since: float | None = None
        self._missing_reported = False
        self._cleared_jobs: dict[str, tuple[int, float]] = {}  # server -> canceled goal, deadline
        self._no_robot_warned = False

    # ---- inputs -------------------------------------------------------------------------

    def on_robot_state(self, robot, x, y, yaw, state) -> None:
        m = self._missions[robot]
        m.x, m.y, m.yaw, m.robot_state = x, y, yaw, state
        m.last_seen = self._clock()
        if not m.placed:
            self._place(m)
        elif m.comm == COMM_LOST and not m.fault:
            self._begin_recovery(m)
        self._count_reports_outside(m)

    def on_order_board(self, run_mode, orders) -> None:
        if run_mode == STOP and self._run_mode != STOP:
            for m in self._missions.values():
                # A silent robot may not hear it, so recovery sends its cancel instead.
                if m.goal and m.comm != COMM_LOST:
                    self._cancel_goal(m)
                if m.arm and m.arm.canceled_at is None:
                    self._cancel_arm(m)
        self._run_mode = run_mode
        self._board = list(orders)
        self._reported &= {o.order_id for o in self._board}

    def on_move_result(self, robot, goal_id, success, reason) -> None:
        m = self._missions[robot]
        goal = m.goal
        if goal is None or goal.id != goal_id or goal.kind != MOVE:
            return
        m.goal = None
        if success:
            m.idx = max(m.idx, goal.target_idx)
            m.attempts = 0
        else:
            m.settle_idx = goal.target_idx
            self._goal_failed(m, reason)

    def on_goal_rejected(self, robot, goal_id) -> None:
        m = self._missions[robot]
        goal = m.goal
        if goal is None or goal.id != goal_id:
            return
        m.goal = None
        if goal.kind == MOVE:
            m.settle_idx = goal.target_idx
        self._goal_failed(m, "REJECTED")

    def on_dock_result(self, robot, goal_id, success, error_x_cm=0.0, error_y_cm=0.0,
                       error_yaw_deg=0.0) -> None:
        m = self._missions[robot]
        goal = m.goal
        if goal is None or goal.id != goal_id or goal.kind != DOCK:
            return
        m.goal = None
        if not success:
            self._goal_failed(m, "DOCK_FAILED")
            return
        m.attempts = 0
        fill = self.site.stations[m.dest].action == FILL
        m.phase = FILLING if fill else SWAPPING
        self._event("INFO", "DOCKED",
                    f"{m.dest} error x={error_x_cm} y={error_y_cm} yaw={error_yaw_deg}", robot)
        if fill:
            self._bind_next_order(m)

    def on_fill_result(self, goal_id, success, placed, missing, reason="") -> None:
        self._forget_cleared_job(goal_id)
        m = self._arm_job_owner(goal_id, FILL)
        if m is None:
            return
        job, m.arm = m.arm, None
        _merge(m.order.placed, placed)
        if success:
            # The server has finished trying: whatever is not in the basket stays out of it.
            m.order.missing = _subtract(m.order.items, m.order.placed)
            return
        self._arm_attempt_failed(m, job, reason, "FILL_RETRY", "FILL_FAILED")

    def on_swap_result(self, goal_id, success, took_full, put_empty_done, reason="") -> None:
        self._forget_cleared_job(goal_id)
        m = self._arm_job_owner(goal_id, SWAP)
        if m is None:
            return
        job, m.arm = m.arm, None
        if took_full and m.basket == FULL:
            m.basket = NONE
            self._report_shipped(m)
        if put_empty_done and m.basket == NONE:
            m.basket = EMPTY
            m.attempts = 0
            return
        self._arm_attempt_failed(m, job, reason, "SWAP_RETRY", "SWAP_FAILED")

    def confirm_clear(self, robot, zone_ids, note="") -> list[str] | None:
        """The operator cleared a stuck robot. The one release not judged by pose.

        Returns the zones released, or None if there is no such robot.
        """
        m = self._missions.get(robot)
        if m is None:
            return None
        released = [z for z in zone_ids if self.zones.release(robot, z)]
        m.outside.clear()
        self.zones.cancel_all_requests(robot)
        if m.goal:
            self._cancel_goal(m)
        if m.arm:
            # What the arm did for a cleared robot is for the operator to count. The server is
            # still busy stopping, and that must not count against the next robot.
            self._out.append(CancelCmd(m.arm.server, m.arm.goal_id))
            self._cleared_jobs[m.arm.server] = (
                m.arm.goal_id, self._clock() + self.params.arm_cancel_wait_s)
            m.arm = None
        m.fault = OUT_OF_SERVICE
        m.comm = ""
        m.called = False
        self._event("INFO", "ZONES_CLEARED", f"released {released}: {note}", robot)
        if m.order:
            # Unbound, it is back on the board at its registration position.
            self._event("INFO", "ORDER_REQUEUED",
                        f"{m.order.order_id} was bound, basket held {m.order.placed}", robot)
            m.order = None
        return released

    # ---- outputs ------------------------------------------------------------------------

    def tick(self) -> list:
        now = self._clock()
        everyone_here = self._everyone_reported(now)
        self._check_comm(now)
        self._finish_recoveries(now)
        self._check_arm_timeouts(now)
        for m in self._missions.values():
            if m.available:
                self._track_progress(m)
                self._release_passed_zones(m)
        halted = self._check_deadlock(now)
        if self._run_mode == RUN and not halted and everyone_here:
            self._dispatch()
            for m in self._missions.values():
                if m.available:
                    self._step(m)
        out, self._out = self._out, []
        return out

    def zones_view(self) -> list[ZoneView]:
        return [
            ZoneView(zone, entry["holder"] or "", entry["queue"], entry["acquired_at"])
            for zone, entry in self.zones.snapshot().items()
        ]

    def assignments_view(self) -> list[AssignmentView]:
        views = []
        for m in self._missions.values():
            if not m.placed or m.fault == OUT_OF_SERVICE:
                continue
            order_id = m.order.order_id if m.order else ""
            docking = m.dest if m.phase == DOCKING else None
            if m.phase == FILLING:
                views.append(AssignmentView(m.name, order_id, self._work, "FILL"))
            elif m.phase == SWAPPING:
                views.append(AssignmentView(m.name, order_id, self._outbound, "SWAP"))
            elif m.phase == TO_OUTBOUND or docking == self._outbound:
                views.append(AssignmentView(m.name, order_id, self._outbound, "TO_OUTBOUND"))
            elif docking == self._work or (m.phase == TO_WORK and m.called):
                views.append(AssignmentView(m.name, order_id, self._work, "TO_WORK"))
            else:
                views.append(AssignmentView(m.name, order_id, "", "WAIT"))
        return views

    def robot_view(self, robot) -> dict:
        m = self._missions[robot]
        return {
            "state": m.state,
            "phase": m.phase,
            "basket": m.basket,
            "order": m.order.order_id if m.order else None,
            "placed": dict(m.order.placed) if m.order else {},
            "called": m.called,
            "held": self.zones.held_by(robot),
            "path": list(m.path[m.idx:]),
            "cell": m.path[m.idx] if m.path else None,
            "flow": self._flow_at(m, m.idx) if m.path else "",
            "destination": m.dest,
            "goal_id": m.goal.id if m.goal else None,
            "attempts": m.attempts,
        }

    # ---- placement and helpers ----------------------------------------------------------

    def _everyone_reported(self, now: float) -> bool:
        missing = [m.name for m in self._missions.values()
                   if not m.placed and m.fault != OUT_OF_SERVICE]
        if not missing:
            return True
        if self._startup_since is None:
            self._startup_since = now
        if not self._missing_reported and now - self._startup_since >= self.params.startup_wait_s:
            self._missing_reported = True
            self._event("ERROR", "ROBOT_NOT_REPORTING",
                        f"nothing moves until {missing} report a position")
        return False

    def _event(self, level, code, detail, robot="") -> None:
        self._out.append(EventOut(level, code, detail, robot))

    def _next_goal_id(self) -> int:
        self._goal_seq += 1
        return self._goal_seq

    def _zone(self, cell) -> str:
        return self.site.zone_of[cell]

    def _place(self, m: _Mission) -> None:
        cell = self.site.nearest_cell(m.x, m.y)
        m.placed = True
        m.path = [cell]
        m.idx = 0
        m.start_flow = self.site.flows_at(cell)[0]
        m.phase = PARKED if cell in self.site.parking else IDLE
        zone = self._zone(cell)
        if not self.zones.force_acquire(m.name, zone):
            m.fault = ERROR
            self._event("ERROR", "PLACEMENT_CONFLICT",
                        f"{zone} is held by {self.zones.holder(zone)}", m.name)

    def _flow_at(self, m: _Mission, index: int) -> str:
        if index == 0:
            return m.start_flow
        return self.site.flow_after(m.path[index - 1], m.path[index])

    def _upcoming(self, m: _Mission) -> tuple[str, str | None]:
        """The robot's current zone and the next different zone on its path."""
        current = self._zone(m.path[m.idx])
        for cell in m.path[m.idx + 1:]:
            zone = self._zone(cell)
            if zone != current:
                return current, zone
        return current, None

    def _anchor(self, m: _Mission) -> int:
        """Path index the robot is committed to reach before anything can change its route."""
        if m.goal and m.goal.kind == MOVE:
            return m.goal.target_idx
        return m.settle_idx if m.settle_idx is not None else m.idx

    def _cancel_goal(self, m: _Mission) -> None:
        self._out.append(CancelCmd(m.name, m.goal.id))
        if m.goal.kind == MOVE:
            m.settle_idx = m.goal.target_idx
        m.goal = None

    def _fail(self, m: _Mission) -> None:
        """The operator has to look at it. It keeps its zones and any bound order."""
        m.fault = ERROR
        m.called = False
        self.zones.cancel_all_requests(m.name)

    def _goal_failed(self, m: _Mission, reason: str) -> None:
        m.attempts += 1
        if m.attempts > self.params.move_retry_limit:
            self._fail(m)
            self._event("ERROR", "GOAL_FAILED",
                        f"gave up after {m.attempts} attempts: {reason}", m.name)
        else:
            self._event("WARN", "GOAL_RETRY", f"attempt {m.attempts} failed: {reason}", m.name)

    # ---- progress and zone release ------------------------------------------------------

    def _track_progress(self, m: _Mission) -> None:
        """Move the path index forward to the cell under the pose, up to the running target."""
        if not m.goal or m.goal.kind != MOVE:
            return
        best = m.idx
        best_distance = math.inf
        for index in range(m.idx, m.goal.target_idx + 1):
            distance = math.dist((m.x, m.y), self.site.cell_pose(m.path[index]))
            if distance < best_distance:
                best, best_distance = index, distance
        m.idx = best

    def _count_reports_outside(self, m: _Mission) -> None:
        for zone in self.zones.held_by(m.name):
            if self.site.distance_to_zone(m.x, m.y, zone) >= self.params.release_margin:
                m.outside[zone] = m.outside.get(zone, 0) + 1
            else:
                m.outside.pop(zone, None)

    def _release_if_cleared(self, m: _Mission, zone: str) -> None:
        if m.outside.get(zone, 0) >= self.params.release_confirm_samples:
            self.zones.release(m.name, zone)
            m.outside.pop(zone, None)

    def _release_passed_zones(self, m: _Mission) -> None:
        """Release held zones the robot is outside of and does not need next.

        Only the current and next zone count as still needed. A zone that comes up again much
        later (a robot driving a whole loop crosses Z-X twice) must not stay held meanwhile.
        """
        keep = self._upcoming(m)
        for zone in self.zones.held_by(m.name):
            if zone not in keep:
                self._release_if_cleared(m, zone)

    # ---- comm loss and recovery ---------------------------------------------------------

    def _check_comm(self, now: float) -> None:
        for m in self._missions.values():
            if not m.live or m.comm == COMM_LOST:
                continue
            if now - m.last_seen > self.params.comm_timeout_s:
                m.comm = COMM_LOST
                m.called = False
                # A silent robot must not stop the zone passing to those queued behind it.
                self.zones.cancel_all_requests(m.name)
                self._event("WARN", "COMM_LOST", f"no state for {now - m.last_seen:.1f} s", m.name)

    def _begin_recovery(self, m: _Mission) -> None:
        m.comm = RECOVERING
        m.recover_at = self._clock() + self.params.recover_check_s
        if m.goal:
            self._cancel_goal(m)
        m.settle_idx = None
        self._event("INFO", "COMM_RESTORED", "checking the pose before resuming", m.name)

    def _finish_recoveries(self, now: float) -> None:
        for m in self._missions.values():
            if m.live and m.comm == RECOVERING and now >= m.recover_at:
                self._finish_recovery(m)

    def _recovery_position(self, m: _Mission) -> tuple[tuple[int, int], str]:
        """The cell under the pose and its lane. A robot moved by hand is placed like at start."""
        cell = self.site.nearest_cell(m.x, m.y)
        for index in range(max(m.idx - 1, 0), len(m.path)):
            if m.path[index] == cell:
                return cell, self._flow_at(m, index)
        return cell, self.site.flows_at(cell)[0]

    def _finish_recovery(self, m: _Mission) -> None:
        m.comm = ""
        cell, flow = self._recovery_position(m)
        zone = self._zone(cell)
        for held in self.zones.held_by(m.name):
            self._release_if_cleared(m, held)
        if zone not in self.zones.held_by(m.name) and not self.zones.request(m.name, zone):
            m.fault = ERROR
            self.zones.cancel_all_requests(m.name)
            self._event("ERROR", "RECOVERY_CONFLICT",
                        f"{m.name} is in {zone} held by {self.zones.holder(zone)}", m.name)
            return
        m.path = [cell]
        m.idx = 0
        m.start_flow = flow
        if m.phase in (FILLING, SWAPPING):
            return
        if m.phase == DOCKING and cell == self.site.stations[m.dest].dock:
            return
        m.settle_idx = 0
        if m.basket == FULL:
            self._start_leg(m, TO_OUTBOUND, self._outbound,
                            self.site.stations[self._outbound].dock)
        else:
            m.phase = PARKED if cell in self.site.parking else IDLE
            m.dest = None

    # ---- arms ---------------------------------------------------------------------------

    def _arm_job_owner(self, goal_id: int, kind: str) -> _Mission | None:
        for m in self._missions.values():
            if m.arm and m.arm.goal_id == goal_id and m.arm.kind == kind:
                return m
        return None

    def _forget_cleared_job(self, goal_id: int) -> None:
        for server, (cleared_goal, _) in list(self._cleared_jobs.items()):
            if cleared_goal == goal_id:
                del self._cleared_jobs[server]

    def _arm_goal_allowed(self, m: _Mission, server: str) -> bool:
        """Hold a new goal after a failure, and while the server stops a cleared robot's job."""
        now = self._clock()
        cleared = self._cleared_jobs.get(server)
        if cleared and now >= cleared[1]:
            del self._cleared_jobs[server]
            cleared = None
        return not cleared and now >= m.arm_retry_at

    def _cancel_arm(self, m: _Mission) -> None:
        m.arm.canceled_at = self._clock()
        self._out.append(CancelCmd(m.arm.server, m.arm.goal_id))

    def _check_arm_timeouts(self, now: float) -> None:
        for m in self._missions.values():
            job = m.arm
            if not job:
                continue
            limit = (self.params.fill_timeout_s if job.kind == FILL
                     else self.params.swap_timeout_s)
            if job.canceled_at is None and now - job.sent_at > limit:
                self._cancel_arm(m)
                job.timed_out = True
            elif job.canceled_at is not None and (
                    now - job.canceled_at >= self.params.arm_cancel_wait_s):
                # Nobody can say what is in the basket now, so the operator has to look.
                m.arm = None
                self._fail(m)
                self._event("ERROR", "ARM_NO_RESULT",
                            f"{job.server} goal {job.goal_id} never reported", m.name)

    def _arm_attempt_failed(self, m, job, reason, retry_code, failed_code) -> None:
        if reason == CANCELED and not job.timed_out:
            return  # STOP canceled it, nothing went wrong
        m.attempts += 1
        m.arm_retry_at = self._clock() + self.params.arm_retry_wait_s
        if m.attempts > self.params.move_retry_limit:
            self._fail(m)
            self._event("ERROR", failed_code, f"{job.server}: {reason}", m.name)
        else:
            self._event("WARN", retry_code,
                        f"{job.server} attempt {m.attempts} failed: {reason}", m.name)

    # ---- orders -------------------------------------------------------------------------

    def _pending(self) -> list:
        """Board orders nobody holds or finished, in registration order."""
        bound = {m.order.order_id for m in self._missions.values() if m.order}
        return [o for o in self._board
                if o.order_id not in bound and o.order_id not in self._reported]

    def _bind_next_order(self, m: _Mission) -> None:
        pending = self._pending()
        if not pending:
            return
        record = pending[0]
        m.order = _BoundOrder(record.order_id, dict(record.items))
        m.last_order_id = record.order_id
        self._event("INFO", "ORDER_BOUND", record.order_id, m.name)

    def _report(self, m: _Mission, status: str, detail: str = "") -> None:
        order, m.order = m.order, None
        self._reported.add(order.order_id)
        self._out.append(ReportOrderCmd(order.order_id, m.name, status, dict(order.placed),
                                        dict(order.missing), detail))

    def _report_shipped(self, m: _Mission) -> None:
        self.shipped += 1
        self._report(m, PARTIAL if m.order.missing else SHIPPED)

    def _finish_fill(self, m: _Mission) -> None:
        order = m.order
        m.attempts = 0
        order.missing = _subtract(order.items, order.placed)
        if not order.placed:
            self._report(m, HELD, f"nothing could be placed, missing {order.missing}")
        elif order.missing and not self.params.ship_partial:
            detail = f"{order.order_id} partial not allowed, basket holds {order.placed}"
            m.basket = FULL
            self._report(m, HELD, detail)
            self._fail(m)
            self._event("ERROR", "FILL_FAILED", detail, m.name)
        else:
            m.basket = FULL
            self._next_work(m)

    # ---- deadlock safety net ------------------------------------------------------------

    def _priority(self, robot: str) -> int:
        m = self._missions[robot]
        carrying = m.basket == FULL or bool(m.order and m.order.placed)
        return priority(self.params, m.phase in (FILLING, SWAPPING), carrying)

    def _check_deadlock(self, now: float) -> bool:
        """Return True while a wait cycle exists, which halts new moves and grants."""
        cycle = find_cycle(wait_graph(self.zones))
        if not cycle:
            self._deadlock_since = None
            self._deadlock_acted = False
            self._retreating = None
            return False
        if self._deadlock_since is None:
            self._deadlock_since = now
            self._event("ERROR", "DEADLOCK_DETECTED", f"wait cycle {cycle}")
        elif (now - self._deadlock_since >= self.params.deadlock_grace_s
              and not self._deadlock_acted):
            self._deadlock_acted = True
            self._retreat(choose_retreat(cycle, self._priority), cycle)
        elif self._retreating and now >= self._retreating[2]:
            self._finish_retreat(cycle)
        return True

    def _retreat(self, robot: str, cycle: list[str]) -> None:
        """Give up reservations of zones not yet entered. Physical reversing is not built."""
        m = self._missions[robot]
        given_up = [
            zone for zone in self.zones.held_by(robot)
            if self.site.distance_to_zone(m.x, m.y, zone) >= self.params.release_margin
        ]
        if not given_up:
            self._event("ERROR", "DEADLOCK_UNRESOLVED",
                        f"{robot} of {cycle} has no unentered zone to give up", robot)
            return
        if m.goal:
            self._cancel_goal(m)
        self._retreating = (robot, given_up, self._clock() + self.params.recover_check_s)
        self._event("ERROR", "DEADLOCK_RETREAT", f"{robot} gives up {given_up}", robot)

    def _finish_retreat(self, cycle: list[str]) -> None:
        """Release only what the stopped robot is still outside of."""
        robot, zones, _ = self._retreating
        m = self._missions[robot]
        for zone in zones:
            self._release_if_cleared(m, zone)
        nothing_released = set(zones) <= set(self.zones.held_by(robot))
        if nothing_released and any(
                self.site.distance_to_zone(m.x, m.y, zone) >= self.params.release_margin
                for zone in zones):
            # Outside but too few reports so far: the next ones can still confirm it.
            self._retreating = (robot, zones, self._clock() + self.params.recover_check_s)
            return
        self._retreating = None
        if nothing_released:
            self._event("ERROR", "DEADLOCK_UNRESOLVED",
                        f"{robot} of {cycle} entered {zones} before it stopped", robot)

    # ---- calling robots -----------------------------------------------------------------

    def _robot_info(self, m: _Mission) -> RobotInfo:
        # A robot driving inside the waiting cells counts from the cell it will stop at, one
        # driving out of them from the cell it still stands in.
        index = self._anchor(m)
        if m.path[index] not in self.site.parking:
            index = m.idx
        cell = m.path[index]
        return RobotInfo(
            robot_id=m.name,
            cell=cell,
            flow=self._flow_at(m, index),
            called=m.called,
            parked=cell in self.site.parking,
            eligible=m.available and m.basket == EMPTY and not m.order and not m.arm,
            available=m.available,
        )

    def _committed(self, m: _Mission) -> bool:
        """A robot heading to the fill dock cannot turn off once it has the commit zone."""
        if m.phase != TO_WORK:
            return False
        zone = self.params.commit_zone
        if zone in self.zones.held_by(m.name):
            return True
        return zone not in {self._zone(c) for c in m.path[m.idx:]}

    def _dispatch(self) -> None:
        # A cleared robot is off the floor, so unlike a failed one it blocks nobody.
        infos = [self._robot_info(m) for m in self._missions.values()
                 if m.placed and m.fault != OUT_OF_SERVICE]
        pending = self._pending()
        self._warn_if_no_robot(pending)
        called = self.dispatcher.pick(len(pending), infos)
        for m in self._missions.values():
            if m.available:
                m.called = m.name in called
                self._redirect(m)

    def _warn_if_no_robot(self, pending: list) -> None:
        """Orders wait and no robot can come until the operator acts. Say so, once per stall."""
        stalled = bool(pending) and not any(m.live for m in self._missions.values())
        if stalled and not self._no_robot_warned:
            ids = [o.order_id for o in pending]
            self._event("WARN", "NO_ROBOT_AVAILABLE", f"{ids} wait and no robot can take them")
        self._no_robot_warned = stalled

    def _redirect(self, m: _Mission) -> None:
        """Send an empty driving robot where its call now says."""
        if m.basket != EMPTY or m.phase not in (TO_WORK, TO_PARK) or self._committed(m):
            return
        if m.called:
            if m.phase != TO_WORK:
                self._start_leg(m, TO_WORK, self._work, self.site.stations[self._work].dock)
        elif m.phase == TO_WORK:
            self._start_park_leg(m)

    # ---- mission steps ------------------------------------------------------------------

    def _start_leg(self, m: _Mission, phase: str, dest: str | None, goal_cell) -> None:
        """Replace the path after the point the robot is committed to reach."""
        _, old_next = self._upcoming(m) if m.path else ("", None)
        anchor = self._anchor(m)
        tail = self.site.plan(m.path[anchor], self._flow_at(m, anchor), goal_cell)
        flow = self._flow_at(m, m.idx)
        m.path = m.path[m.idx:anchor + 1] + tail[1:]
        if m.goal and m.goal.kind == MOVE:
            m.goal.target_idx -= m.idx
        if m.settle_idx is not None:
            m.settle_idx = max(0, m.settle_idx - m.idx)
        m.idx = 0
        m.start_flow = flow
        m.phase = phase
        m.dest = dest
        _, new_next = self._upcoming(m)
        if old_next and old_next != new_next:
            self.zones.cancel_request(m.name, old_next)

    def _park_goal(self, m: _Mission, from_cell) -> tuple[int, int] | None:
        """Deepest parking cell it can drive to from here without passing another robot."""
        order = self.site.parking
        start = order.index(from_cell) + 1 if from_cell in order else 0
        goal = None
        for cell in order[start:]:
            zone = self._zone(cell)
            holder = self.zones.holder(zone)
            if (holder and holder != m.name) or set(self.zones.queue(zone)) - {m.name}:
                break
            goal = cell
        return goal

    def _start_park_leg(self, m: _Mission) -> None:
        anchor_cell = m.path[self._anchor(m)]
        goal = self._park_goal(m, anchor_cell)
        if goal is None:
            # Already parked with nothing free ahead: stay. Outside: queue for the entrance.
            goal = anchor_cell if anchor_cell in self.site.parking else self.site.parking[0]
        self._start_leg(m, TO_PARK, None, goal)

    def _next_work(self, m: _Mission) -> None:
        """Leave a dock or a parked spot for whatever comes next."""
        if m.basket == FULL:
            self._start_leg(m, TO_OUTBOUND, self._outbound,
                            self.site.stations[self._outbound].dock)
        elif m.called:
            self._start_leg(m, TO_WORK, self._work, self.site.stations[self._work].dock)
        else:
            self._start_park_leg(m)

    def _step(self, m: _Mission) -> None:
        """Act on the current phase, and straight away on any phase that leads to."""
        phase = None
        while m.phase != phase:
            phase = m.phase
            self._step_phase(m)

    def _step_phase(self, m: _Mission) -> None:
        if m.phase in (TO_WORK, TO_OUTBOUND, TO_PARK):
            self._drive(m)
        elif m.phase == DOCKING:
            if m.goal is None:
                goal_id = self._next_goal_id()
                m.goal = _Goal(DOCK, goal_id)
                self._out.append(DockCmd(m.name, goal_id, m.dest))
        elif m.phase == FILLING:
            self._step_filling(m)
        elif m.phase == SWAPPING:
            self._step_swapping(m)
        elif m.called or m.phase == IDLE:
            self._next_work(m)
        else:
            self._step_parked(m)

    def _step_parked(self, m: _Mission) -> None:
        """Keep advancing into free cells so the parking entrance stays clear."""
        if self._park_goal(m, m.path[m.idx]):
            self._start_park_leg(m)

    def _step_filling(self, m: _Mission) -> None:
        if m.arm:
            return
        if not m.order:
            # Docked with nothing bound, or the last order was held: stay only if called.
            if m.called:
                self._bind_next_order(m)
            if not m.order:
                self._next_work(m)
                return
        order = m.order
        if sum(order.items.values()) > self.params.basket_capacity:
            # Nobody looked on the shelf, so nothing is missing: the stock record stays as it is.
            self._report(m, HELD, f"{sum(order.items.values())} items exceed the basket "
                                  f"capacity of {self.params.basket_capacity}")
            return
        remaining = _subtract(order.items, order.placed, order.missing)
        if not remaining:
            self._finish_fill(m)
            return
        station = self.site.stations[m.dest]
        if not self._arm_goal_allowed(m, station.server):
            return
        goal_id = self._next_goal_id()
        m.arm = _ArmJob(station.server, goal_id, FILL, self._clock())
        self._out.append(FillCmd(goal_id, m.name, m.dest, order.order_id, remaining))

    def _step_swapping(self, m: _Mission) -> None:
        if m.arm:
            return
        if m.basket == EMPTY:
            self._next_work(m)
            return
        station = self.site.stations[m.dest]
        if not self._arm_goal_allowed(m, station.server):
            return
        goal_id = self._next_goal_id()
        m.arm = _ArmJob(station.server, goal_id, SWAP, self._clock())
        self._out.append(SwapCmd(station.server, goal_id, m.name, m.dest, m.last_order_id,
                                 take_full=m.basket == FULL, put_empty=True))

    def _drive(self, m: _Mission) -> None:
        if m.phase == TO_PARK:
            self._trim_parking_path(m)
        if m.goal is None and m.settle_idx is None and m.idx == len(m.path) - 1:
            self._arrive(m)
            return
        self._request_next_zone(m)
        self._issue_move(m)

    def _trim_parking_path(self, m: _Mission) -> None:
        """Re-aim inside the parking: stop behind whoever got there first, or go deeper."""
        anchor = self._anchor(m)
        cell = m.path[anchor]
        order = self.site.parking
        old_tail = m.path[anchor + 1:]
        if cell not in order and old_tail[:1] != [order[0]]:
            return
        goal = self._park_goal(m, cell)
        start = order.index(cell) + 1 if cell in order else 0
        new_tail = order[start:order.index(goal) + 1] if goal else []
        if cell not in order and not new_tail:
            return  # the first parking cell is taken: keep queueing for it
        if new_tail != old_tail:
            for removed in old_tail:
                if removed not in new_tail:
                    self.zones.cancel_request(m.name, self._zone(removed))
            m.path = m.path[:anchor + 1] + list(new_tail)

    def _arrive(self, m: _Mission) -> None:
        if m.phase == TO_PARK:
            m.phase = PARKED
        elif m.phase == TO_OUTBOUND:
            m.phase = DOCKING
        elif m.called and self._pending():
            m.phase = DOCKING
        else:
            m.dest = None
            self._next_work(m)

    def _request_next_zone(self, m: _Mission) -> None:
        """Ask for the next zone only once the robot holds nothing but its current one."""
        current, following = self._upcoming(m)
        if following is None:
            return
        held = self.zones.held_by(m.name)
        if following in held or any(zone != current for zone in held):
            m.wait_zone = None
            return
        if self.zones.request(m.name, following):
            m.wait_zone = None
        else:
            self._note_waiting(m, following)

    def _note_waiting(self, m: _Mission, zone: str) -> None:
        now = self._clock()
        if m.wait_zone != zone:
            m.wait_zone, m.wait_since, m.wait_warned = zone, now, False
        elif not m.wait_warned and now - m.wait_since >= self.params.zone_wait_warn_s:
            m.wait_warned = True
            self._event("WARN", "ZONE_WAIT_LONG",
                        f"waiting {now - m.wait_since:.0f} s for {zone} "
                        f"held by {self.zones.holder(zone) or 'nobody'}", m.name)

    def _issue_move(self, m: _Mission) -> None:
        held = self.zones.held_by(m.name)
        if m.settle_idx is not None and m.goal is None:
            settle = m.settle_idx
            while settle > m.idx and self._zone(m.path[settle]) not in held:
                settle -= 1
            m.settle_idx = None
            self._send_move(m, settle, self._step_into(m, settle))
            return
        last = len(m.path) - 1
        if m.goal:
            if m.goal.kind != MOVE:
                return
            anchor, step = m.goal.target_idx, m.goal.step
        else:
            anchor = m.idx
            step = self._step_between(m, anchor)
        reach = anchor
        while (reach < last and self._step_between(m, reach) == step
               and self._zone(m.path[reach + 1]) in held):
            reach += 1
        if reach != anchor:
            self._send_move(m, reach, step)

    def _step_between(self, m: _Mission, index: int) -> tuple[int, int]:
        a, b = m.path[index], m.path[index + 1]
        return (b[0] - a[0], b[1] - a[1])

    def _step_into(self, m: _Mission, index: int) -> tuple[int, int] | None:
        return self._step_between(m, index - 1) if index > 0 else None

    def _send_move(self, m: _Mission, target_idx: int, step) -> None:
        """A new goal replaces any running one. Its id is what later results must match."""
        cell = m.path[target_idx]
        x, y = self.site.cell_pose(cell)
        yaw = math.atan2(step[1], step[0]) if step else m.yaw
        goal_id = self._next_goal_id()
        m.goal = _Goal(MOVE, goal_id, target_idx, step)
        self._out.append(MoveCmd(m.name, goal_id, self.site.cell_id(cell), x, y, yaw))
