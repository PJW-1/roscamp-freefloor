"""TrafficCore: robot selection, zone locks and motion. No ROS, no wall clock, no orders.

Callers feed robot updates and dispatch requests through the on_* methods and call tick()
periodically. tick() returns the commands to send, the events to publish and the dispatch
feedback and results for whoever asked. What a robot does at a station is not known here: it
stands there (AT_STATION) until it is released or sent somewhere else.

Safety rules kept here, each backed by a test:
  - A robot holds its current zone and at most one next zone, and moves only inside them.
  - A zone is released only after consecutive reports are past it by the release distance,
    so one noisy pose cannot free a zone the robot's body is still next to.
  - Results of goals that are no longer the robot's active goal are ignored.
  - Nothing moves until every robot has reported, because a robot that has not reported may
    be standing in the zone another one would be given.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from fleet_core.commands import (
    NO_ROBOT,
    CancelCmd,
    DispatchFeedback,
    DispatchResult,
    DockCmd,
    EventOut,
    MoveCmd,
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
AT_STATION = "AT_STATION"
# Fault states shown instead of the phase.
ERROR = "ERROR"
OUT_OF_SERVICE = "OUT_OF_SERVICE"
COMM_LOST = "COMM_LOST"
RECOVERING = "RECOVERING"
PLACEMENT_CONFLICT = "PLACEMENT_CONFLICT"
NO_FAULT = "NONE"  # what /fleet/robots says when nothing is wrong
ON_A_GOAL = ("MOVING", "DOCKING")  # what a robot says in its state report
_COMM_REASON = {COMM_LOST: "NO_STATE", RECOVERING: "CHECKING_POSE"}

RUN = "RUN"
STOP = "STOP"

MOVE = "move"
DOCK = "dock"

# Station actions, as written in the site map.
FILL = "fill"
SWAP = "swap"


@dataclass
class _Goal:
    kind: str
    id: int
    target_idx: int = 0  # move only: index into the mission path
    step: tuple[float, float] | None = None  # move only: direction of travel, a unit vector


@dataclass
class _Dispatch:
    goal_id: int
    station_id: str


@dataclass
class _Mission:
    name: str
    placed: bool = False
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    robot_state: str = ""
    battery: float = 0.0
    last_seen: float = 0.0
    phase: str = IDLE
    fault: str = ""  # ERROR or OUT_OF_SERVICE, both need the operator
    fault_reason: str = ""
    comm: str = ""  # COMM_LOST or RECOVERING
    recover_at: float = 0.0
    path: list = field(default_factory=list)
    idx: int = 0  # the node whose zone it is in: reached, or less than d from the last one
    dest: str | None = None  # station it is driving to or docked at
    goal: _Goal | None = None
    settle_idx: int | None = None  # drive here first, so a resumed goal stays on the lane
    attempts: int = 0
    retry_at: float = 0.0  # no new move or dock before this, after a failed attempt
    called: bool = False
    dispatch: _Dispatch | None = None  # a dispatch that named this robot
    hold: bool = False  # kept at the station until it is released or sent on
    await_pick: bool = False  # at a station with no duty: acts after the next pick, not before
    outside: dict = field(default_factory=dict)  # zone -> consecutive reports clear of it
    off_lane: int = 0  # consecutive reports too far off the lane it is driving
    off_lane_warned: bool = False
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


class TrafficCore:
    def __init__(self, site, params, robots, clock, next_goal_id=None):
        if site.shortest_exit_lane() <= params.release_distance_m:
            raise ValueError(
                f"the shortest lane leaving a zone ({site.shortest_exit_lane()} m) must exceed "
                f"the release distance ({params.release_distance_m} m), or a robot stopped at "
                "its end never releases the zone behind it")
        if params.release_distance_m <= 2 * params.robot_radius_m:
            raise ValueError(
                f"the release distance ({params.release_distance_m} m) must exceed twice the "
                f"robot radius ({params.robot_radius_m} m), or all lanes meeting at a node become "
                "conflict pairs and no two robots can stand on neighbouring nodes")
        self.site = site
        self.params = params
        self.zones = ZoneManager(site.zones, clock, site.conflict_pairs(
            2 * params.robot_radius_m, params.release_distance_m))
        self.dispatcher = Dispatcher(site, params)
        self._clock = clock
        self._next_goal_id = next_goal_id or self._own_goal_id
        self._goal_seq = 0
        self._missions = {name: _Mission(name) for name in robots}
        self._out: list = []
        self._run_mode: str | None = None
        self._open_calls: list[int] = []  # dispatches to the fill station with no robot named
        self._work = next(s.id for s in site.stations.values() if s.action == FILL)
        self._outbound = next(s.id for s in site.stations.values() if s.action == SWAP)
        self._deadlock_since: float | None = None
        self._deadlock_acted = False
        self._retreating: tuple[str, list[str], float] | None = None  # robot, zones, release time
        self._startup_since: float | None = None
        self._missing_reported = False
        self._no_robot_warned = False

    # ---- inputs -------------------------------------------------------------------------

    def on_robot_state(self, robot, x, y, yaw, state, battery=0.0) -> None:
        m = self._missions[robot]
        m.x, m.y, m.yaw, m.robot_state, m.battery = x, y, yaw, state, battery
        m.last_seen = self._clock()
        if not m.placed and m.fault != OUT_OF_SERVICE:
            self._place(m)
        elif m.comm == COMM_LOST and not m.fault:
            self._begin_recovery(m)
        if m.placed:
            self._count_reports_outside(m)
            self._watch_lane_error(m)

    def on_run_mode(self, run_mode) -> None:
        if run_mode == STOP and self._run_mode != STOP:
            for m in self._missions.values():
                # A silent robot may not hear it, so recovery sends its cancel instead.
                if m.goal and m.comm != COMM_LOST:
                    self._cancel_goal(m)
        self._run_mode = run_mode

    def on_dispatch(self, goal_id, station_id, robot_id="") -> None:
        """Ask for a robot at a station. With no robot named it becomes an open call."""
        if not robot_id:
            if station_id != self._work:
                raise ValueError(f"only the fill station takes open calls, not {station_id}")
            self._open_calls.append(goal_id)
            return
        m = self._missions[robot_id]
        if not m.available:
            self._out.append(DispatchResult(goal_id, False, robot_id, "NOT_AVAILABLE"))
            return
        if m.dispatch:
            self._out.append(DispatchResult(goal_id, False, robot_id, "BUSY"))
            return
        m.dispatch = _Dispatch(goal_id, station_id)
        m.await_pick = False
        if m.phase == AT_STATION and m.dest == station_id:
            self._finish_dispatch(m)
            return
        m.hold = False
        self._start_dispatch_leg(m)

    def on_dispatch_cancel(self, goal_id) -> None:
        if goal_id in self._open_calls:
            self._open_calls.remove(goal_id)
            return
        for m in self._missions.values():
            if m.dispatch and m.dispatch.goal_id == goal_id:
                m.dispatch = None
                self._out.append(DispatchResult(goal_id, False, m.name, "CANCELED"))
                if m.goal and m.goal.kind == DOCK:
                    self._cancel_goal(m)
                if m.available:
                    self._start_park_leg(m)
                return

    def on_release(self, robot) -> None:
        m = self._missions[robot]
        m.hold = False
        m.await_pick = True

    def on_robot_fault(self, robot, reason="") -> None:
        self._fail(self._missions[robot], reason)

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
        m.phase = AT_STATION
        self._event("INFO", "DOCKED",
                    f"{m.dest} error x={error_x_cm} y={error_y_cm} yaw={error_yaw_deg}", robot)
        self._finish_dispatch(m, error_x_cm, error_y_cm, error_yaw_deg)

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
        m.fault = OUT_OF_SERVICE
        m.fault_reason = note
        m.comm = ""
        m.called = False
        m.hold = False
        self._event("INFO", "ZONES_CLEARED", f"released {released}: {note}", robot)
        if m.dispatch:
            self._out.append(DispatchResult(m.dispatch.goal_id, False, robot, "ROBOT_REMOVED"))
            m.dispatch = None
        return released

    # ---- outputs ------------------------------------------------------------------------

    @property
    def robots(self) -> list[str]:
        return list(self._missions)

    @property
    def everyone_placed(self) -> bool:
        return not self._not_placed()

    @property
    def running(self) -> bool:
        return self._run_mode == RUN

    def available(self, robot) -> bool:
        return self._missions[robot].available

    def tick(self) -> list:
        """One whole cycle, for a caller with nothing to do between its parts."""
        now = self._clock()
        everyone_here = self.tick_begin(now)
        halted = self.tick_progress(now)
        if self.running and not halted and everyone_here:
            self.tick_pick()
            for robot in self.robots:
                if self.available(robot):
                    self.step(robot)
        return self.take_output()

    def tick_begin(self, now: float) -> bool:
        """Startup and comm checks. Returns True once every robot has reported."""
        everyone_here = self._everyone_reported(now)
        self._check_comm(now)
        self._finish_recoveries(now)
        return everyone_here

    def tick_progress(self, now: float) -> bool:
        """Follow poses and free passed zones. Returns True while a wait cycle halts new moves."""
        for m in self._missions.values():
            if m.available:
                self._track_progress(m)
                self._release_passed_zones(m)
        return self._check_deadlock(now)

    def tick_pick(self) -> None:
        """Decide who answers the open calls, and redirect the robots that changed."""
        # A cleared robot is off the floor, so unlike a failed one it blocks nobody.
        infos = [self._robot_info(m) for m in self._missions.values()
                 if m.placed and m.fault != OUT_OF_SERVICE]
        self._warn_if_no_robot()
        called = self.dispatcher.pick(len(self._open_calls), infos)
        for m in self._missions.values():
            if m.available:
                m.called = m.name in called
                m.await_pick = False
                self._redirect(m)

    def step(self, robot) -> None:
        """Act on the current phase, and straight away on any phase that leads to."""
        m = self._missions[robot]
        phase = None
        while m.phase != phase:
            phase = m.phase
            self._step_phase(m)

    def take_output(self) -> list:
        out, self._out = self._out, []
        return out

    def zones_view(self) -> list[ZoneView]:
        return [
            ZoneView(zone, entry["holder"] or "", entry["queue"], entry["acquired_at"])
            for zone, entry in self.zones.snapshot().items()
        ]

    def robot_view(self, robot) -> dict:
        m = self._missions[robot]
        return {
            "state": m.state,
            "phase": m.phase,
            "on_map": m.placed,
            "fault": m.fault,
            "comm": m.comm,
            "called": m.called,
            "held": self.zones.held_by(robot),
            "path": list(m.path[m.idx:]),
            "cell": m.path[m.idx] if m.path else None,
            "destination": m.dest,
            "goal_id": m.goal.id if m.goal else None,
            "attempts": m.attempts,
        }

    def status_view(self, robot) -> dict:
        """What /fleet/robots shows that only traffic knows."""
        m = self._missions[robot]
        wait_zone = self.zones.waiting_for(robot) or ""
        in_the_way = self.zones.blockers(robot, wait_zone) if wait_zone else ()
        return {
            "fsm_state": m.robot_state,
            "fault": m.fault or m.comm or NO_FAULT,
            "fault_reason": m.fault_reason if m.fault else _COMM_REASON.get(m.comm, ""),
            "wait_zone": wait_zone,
            "wait_robots": tuple(sorted(in_the_way)),
            "battery": m.battery,
            "x": m.x,
            "y": m.y,
            "yaw": m.yaw,
        }

    # ---- placement and helpers ----------------------------------------------------------

    def _own_goal_id(self) -> int:
        self._goal_seq += 1
        return self._goal_seq

    def _not_placed(self) -> list[_Mission]:
        return [m for m in self._missions.values()
                if not m.placed and m.fault != OUT_OF_SERVICE]

    def _everyone_reported(self, now: float) -> bool:
        missing = self._not_placed()
        if not missing:
            return True
        if self._startup_since is None:
            self._startup_since = now
        silent = [m.name for m in missing if not m.fault]  # a refused robot has already reported
        if (silent and not self._missing_reported
                and now - self._startup_since >= self.params.startup_wait_s):
            self._missing_reported = True
            self._event("ERROR", "ROBOT_NOT_REPORTING",
                        f"nothing moves until {silent} report a position and stand still")
        return False

    def _event(self, level, code, detail, robot="") -> None:
        self._out.append(EventOut(level, code, detail, robot))

    def _zone(self, cell) -> str:
        return self.site.zone_of[cell]

    def _place(self, m: _Mission) -> None:
        """Take the zone it reports from, or stay off the map and try again at the next report.

        A refused robot stands there with no lock, so _everyone_reported keeps everybody still
        until it has moved or the operator has cleared it. The same goes for a robot on a goal
        this FMS did not give it.
        """
        if m.robot_state in ON_A_GOAL:
            return
        cell = self.site.nearest_cell(m.x, m.y, radius=self.params.release_distance_m)
        zone = self._zone(cell)
        in_the_way = self.zones.holders_around(zone, m.name)
        if not self.zones.force_acquire(m.name, zone):
            if m.fault_reason != PLACEMENT_CONFLICT:
                m.fault = ERROR
                m.fault_reason = PLACEMENT_CONFLICT
                self._event("ERROR", PLACEMENT_CONFLICT,
                            f"{m.name} cannot take {zone}, held: {in_the_way}", m.name)
            return
        m.placed = True
        m.path = [cell]
        m.idx = 0
        m.phase = PARKED if cell in self.site.parking else IDLE
        # Past d it is on the lane into that node, and a straight line from there is no lane.
        if math.dist((m.x, m.y), self.site.cell_pose(cell)) > self.params.release_distance_m:
            m.settle_idx = 0
        if m.fault_reason == PLACEMENT_CONFLICT:
            m.fault = ""
            m.fault_reason = ""

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

    def _fail(self, m: _Mission, reason: str = "") -> None:
        """The operator has to look at it. It keeps its zones."""
        m.fault = ERROR
        m.fault_reason = reason
        m.called = False
        self.zones.cancel_all_requests(m.name)
        if m.dispatch:
            self._out.append(DispatchResult(m.dispatch.goal_id, False, m.name, reason))
            m.dispatch = None

    def _goal_failed(self, m: _Mission, reason: str) -> None:
        m.attempts += 1
        m.retry_at = self._clock() + self.params.move_retry_wait_s
        if m.attempts > self.params.move_retry_limit:
            self._fail(m, reason)
            self._event("ERROR", "GOAL_FAILED",
                        f"gave up after {m.attempts} attempts: {reason}", m.name)
        else:
            self._event("WARN", "GOAL_RETRY", f"attempt {m.attempts} failed: {reason}", m.name)

    # ---- progress and zone release ------------------------------------------------------

    def _track_progress(self, m: _Mission) -> None:
        """Move the path index past every lane the pose is well along, up to the running target."""
        if not m.goal or m.goal.kind != MOVE:
            return
        while m.idx < m.goal.target_idx:
            s, _ = self.site.lane_progress(m.path[m.idx], m.path[m.idx + 1], m.x, m.y)
            if s <= self.params.release_distance_m:
                return
            m.idx += 1

    def _exit_lane(self, m: _Mission, zone: str) -> tuple | None:
        """The lane on which the path leaves a zone the robot is in or has been in."""
        last = m.idx
        while last + 1 < len(m.path) and self._zone(m.path[last + 1]) == self._zone(m.path[last]):
            last += 1
        for index in range(min(last, len(m.path) - 2), -1, -1):
            a, b = m.path[index], m.path[index + 1]
            if self._zone(a) == zone and self._zone(b) != zone:
                return a, b
        return None

    def _clear_of(self, m: _Mission, zone: str) -> bool:
        """Is the robot's body clear of a zone it holds?

        On its path: s > d along the lane leaving the zone. Why d is big enough is in
        fleet_params.yaml. Not on it (a zone ahead, or a robot moved by hand): at least d from the
        zone's centre lines, the same measure as s.

        s is a projection, so it says nothing about a robot far off its lane. Lanes of the zone
        close to the exit lane are a conflict pair already, and LANE_ERROR reports the rest.
        """
        lane = self._exit_lane(m, zone)
        if lane:
            s, _ = self.site.lane_progress(*lane, m.x, m.y)
            return s > self.params.release_distance_m
        return self._off_the_floor(m, zone)

    def _off_the_floor(self, m: _Mission, zone: str) -> bool:
        # The floor is half_width either side of the centre lines, so this is back to those.
        distance = self.site.distance_to_zone(m.x, m.y, zone) + self.site.half_width
        return distance >= self.params.release_distance_m

    def _count_reports_outside(self, m: _Mission) -> None:
        for zone in self.zones.held_by(m.name):
            if self._clear_of(m, zone):
                m.outside[zone] = m.outside.get(zone, 0) + 1
            else:
                m.outside.pop(zone, None)

    def _watch_lane_error(self, m: _Mission) -> None:
        """Locks assume a robot stays on its lane. Say so when one does not, once per goal."""
        if not (m.goal and m.goal.kind == MOVE and m.goal.target_idx > 0):
            return
        a, b = m.path[m.goal.target_idx - 1], m.path[m.goal.target_idx]
        s, e = self.site.lane_progress(a, b, m.x, m.y)
        m.off_lane = m.off_lane + 1 if e > self.params.lane_error_warn_m else 0
        if m.off_lane >= self.params.release_confirm_samples and not m.off_lane_warned:
            m.off_lane_warned = True
            self._event("WARN", "LANE_ERROR", f"{e:.2f} m off {a} to {b} at s={s:.2f}", m.name)

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

    def _finish_recovery(self, m: _Mission) -> None:
        m.comm = ""
        cell = self._resume_node(m)
        zone = self._zone(cell)
        for held in self.zones.held_by(m.name):
            self._release_if_cleared(m, held)
        if zone not in self.zones.held_by(m.name) and not self.zones.request(m.name, zone):
            m.fault = ERROR
            m.fault_reason = "RECOVERY_CONFLICT"
            self.zones.cancel_all_requests(m.name)
            self._event("ERROR", "RECOVERY_CONFLICT",
                        f"{m.name} is in {zone} held by {self.zones.holder(zone)}", m.name)
            return
        m.path = [cell]
        m.idx = 0
        if m.phase == AT_STATION:
            return
        if m.phase == DOCKING and cell == self.site.stations[m.dest].dock:
            return
        m.settle_idx = 0
        if m.dispatch:
            self._start_dispatch_leg(m)
        else:
            m.phase = PARKED if cell in self.site.parking else IDLE
            m.dest = None

    def _resume_node(self, m: _Mission):
        """Where a robot back from silence stands. One report can be off, so what it holds decides.

        Within d of a node it counts as on that node, as a driving robot does. Just past d the
        zone behind may be gone already, and a report short of d must not send it back there.
        """
        cell = self.site.nearest_cell(m.x, m.y, radius=self.params.release_distance_m)
        held = self.zones.held_by(m.name)
        if self._zone(cell) in held:
            return cell
        ahead = self.site.nearest_cell(m.x, m.y)
        return ahead if self._zone(ahead) in held else cell

    # ---- deadlock safety net ------------------------------------------------------------

    def _priority(self, robot: str) -> int:
        """Only what traffic knows: at a station, and asked by name to go somewhere."""
        m = self._missions[robot]
        return priority(self.params, m.phase == AT_STATION, m.dispatch is not None)

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
        given_up = [zone for zone in self.zones.held_by(robot) if self._off_the_floor(m, zone)]
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
        if nothing_released and any(self._off_the_floor(m, zone) for zone in zones):
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
            called=m.called,
            parked=cell in self.site.parking,
            eligible=m.available and not m.hold and m.dispatch is None,
            available=m.available,
        )

    def _committed(self, m: _Mission) -> bool:
        """A robot heading to the fill dock cannot turn off once it has the commit zone."""
        if m.phase != TO_WORK:
            return False
        zone = self._zone(self.site.stations[self._work].commit)
        if zone in self.zones.held_by(m.name):
            return True
        return zone not in {self._zone(c) for c in m.path[m.idx:]}

    def _warn_if_no_robot(self) -> None:
        """Calls wait and no robot can come until the operator acts. Say so, once per stall."""
        stalled = bool(self._open_calls) and not any(m.live for m in self._missions.values())
        if stalled and not self._no_robot_warned:
            self._out.append(DispatchFeedback(self._open_calls[0], "", NO_ROBOT))
        self._no_robot_warned = stalled

    def _redirect(self, m: _Mission) -> None:
        """Send an empty driving robot where its call now says."""
        if m.phase not in (TO_WORK, TO_PARK) or m.dispatch or self._committed(m):
            return
        if m.called:
            if m.phase != TO_WORK:
                self._start_leg(m, TO_WORK, self._work, self.site.stations[self._work].dock)
        elif m.phase == TO_WORK:
            self._start_park_leg(m)

    def _finish_dispatch(self, m: _Mission, error_x_cm=0.0, error_y_cm=0.0,
                         error_yaw_deg=0.0) -> None:
        """Standing at the dock: answer whoever asked, then wait to be told what comes next."""
        if m.dispatch:
            goal_id, m.dispatch = m.dispatch.goal_id, None
        elif m.dest == self._work and self._open_calls:
            goal_id = self._open_calls.pop(0)
        else:
            m.await_pick = True  # nobody is waiting for it any more
            return
        m.hold = True
        self._out.append(DispatchResult(goal_id, True, m.name, "", error_x_cm, error_y_cm,
                                        error_yaw_deg))

    # ---- mission steps ------------------------------------------------------------------

    def _start_leg(self, m: _Mission, phase: str, dest: str | None, goal_cell) -> None:
        """Replace the path after the point the robot is committed to reach."""
        _, old_next = self._upcoming(m) if m.path else ("", None)
        anchor = self._anchor(m)
        tail = self.site.plan(m.path[anchor], goal_cell)
        behind = max(m.idx - 1, 0)  # keep the lane just driven: its zone may not be released yet
        m.path = m.path[behind:anchor + 1] + tail[1:]
        if m.goal and m.goal.kind == MOVE:
            m.goal.target_idx -= behind
        if m.settle_idx is not None:
            m.settle_idx = max(0, m.settle_idx - behind)
        m.idx -= behind
        m.phase = phase
        m.dest = dest
        _, new_next = self._upcoming(m)
        if old_next and old_next != new_next:
            self.zones.cancel_request(m.name, old_next)

    def _start_dispatch_leg(self, m: _Mission) -> None:
        station = m.dispatch.station_id
        phase = TO_WORK if station == self._work else TO_OUTBOUND
        self._start_leg(m, phase, station, self.site.stations[station].dock)

    def _park_goal(self, m: _Mission, from_cell):
        """Deepest parking cell it can drive to from here without passing another robot."""
        spots = self.site.parking
        start = spots.index(from_cell) + 1 if from_cell in spots else 0
        goal = None
        for cell in spots[start:]:
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
        if m.dispatch:
            self._start_dispatch_leg(m)
        elif m.called:
            self._start_leg(m, TO_WORK, self._work, self.site.stations[self._work].dock)
        else:
            self._start_park_leg(m)

    def _step_phase(self, m: _Mission) -> None:
        if m.phase in (TO_WORK, TO_OUTBOUND, TO_PARK):
            self._drive(m)
        elif m.phase == DOCKING:
            if m.goal is None and self._clock() >= m.retry_at:
                goal_id = self._next_goal_id()
                m.goal = _Goal(DOCK, goal_id)
                self._out.append(DockCmd(m.name, goal_id, m.dest))
        elif m.phase == AT_STATION:
            self._step_at_station(m)
        elif m.called or m.phase == IDLE:
            self._next_work(m)
        else:
            self._step_parked(m)

    def _step_at_station(self, m: _Mission) -> None:
        if m.hold or m.await_pick:
            return
        if m.called and m.dest == self._work and self._open_calls:
            self._finish_dispatch(m)  # called while standing at the dock it was called to
            return
        self._next_work(m)

    def _step_parked(self, m: _Mission) -> None:
        """Keep advancing into free cells so the parking entrance stays clear."""
        if m.settle_idx is not None:  # found on the lane into this spot
            self._issue_move(m)
        elif self._park_goal(m, m.path[m.idx]):
            self._start_park_leg(m)

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
        spots = self.site.parking
        old_tail = m.path[anchor + 1:]
        if cell not in spots and old_tail[:1] != [spots[0]]:
            return
        goal = self._park_goal(m, cell)
        start = spots.index(cell) + 1 if cell in spots else 0
        new_tail = spots[start:spots.index(goal) + 1] if goal else []
        if cell not in spots and not new_tail:
            return  # the first parking cell is taken: keep queueing for it
        if new_tail != old_tail:
            for removed in old_tail:
                if removed not in new_tail:
                    self.zones.cancel_request(m.name, self._zone(removed))
            m.path = m.path[:anchor + 1] + list(new_tail)

    def _arrive(self, m: _Mission) -> None:
        if m.phase == TO_PARK:
            m.phase = PARKED
        elif m.phase == TO_OUTBOUND or m.dispatch:
            m.phase = DOCKING
        elif m.called and self._open_calls:
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
        if m.goal is None and self._clock() < m.retry_at:
            return
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

    def _step_between(self, m: _Mission, index: int) -> tuple[float, float]:
        """The direction of a lane, rounded so that lanes in one straight line compare equal."""
        lane = self.site.lane(m.path[index], m.path[index + 1])
        return (round(lane.ux, 6), round(lane.uy, 6))

    def _step_into(self, m: _Mission, index: int) -> tuple[float, float] | None:
        return self._step_between(m, index - 1) if index > 0 else None

    def _send_move(self, m: _Mission, target_idx: int, step) -> None:
        """A new goal replaces any running one. Its id is what later results must match."""
        cell = m.path[target_idx]
        x, y = self.site.cell_pose(cell)
        yaw = math.atan2(step[1], step[0]) if step else m.yaw
        goal_id = self._next_goal_id()
        m.goal = _Goal(MOVE, goal_id, target_idx, step)
        m.off_lane, m.off_lane_warned = 0, False
        self._out.append(MoveCmd(m.name, goal_id, self.site.cell_id(cell), x, y, yaw))
