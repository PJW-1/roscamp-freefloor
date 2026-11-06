"""Comm loss, recovery, deadlock retreat and the operator's confirm_clear.

TrafficRecovery is a part of TrafficController, split out to keep files short. Its methods use
the controller's state (self._missions, self.zones, self._clock, self._out) and are not meant to
be used alone. The public method here is confirm_clear; the rest are called from the tick.
"""
from __future__ import annotations

from gg_fleet_core.common.commands import DispatchResult
from gg_fleet_core.traffic.deadlock import choose_retreat, find_cycle, priority, wait_graph
from gg_fleet_core.traffic.mission import (
    AT_STATION,
    COMM_LOST,
    DOCKING,
    ERROR,
    IDLE,
    OUT_OF_SERVICE,
    PARKED,
    RECOVERING,
    _Mission,
)


class TrafficRecovery:
    def confirm_clear(self, robot, zone_ids, note="") -> list[str] | None:
        """The operator cleared a stuck robot. The one release not judged by pose.

        Returns the zones released, or None if it was refused and nothing changed. No zones is
        right for a robot that never got on the map. Only some of its zones is taken as meant,
        with a warning, and another call frees the rest.
        """
        refusal = self._clear_refusal(robot, zone_ids)
        if refusal:
            self._event("WARN", "CLEAR_REFUSED", f"{refusal}: {note}", robot)
            return None
        m = self._missions[robot]
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
        still_held = self.zones.held_by(robot)
        if still_held:
            # Nothing else frees them: this robot is no longer judged by its pose.
            self._event("WARN", "ZONES_STILL_HELD", f"{still_held} stay locked", robot)
        if m.dispatch:
            self._out.append(DispatchResult(m.dispatch.goal_id, False, robot, "ROBOT_REMOVED"))
            m.dispatch = None
        return released

    def _clear_refusal(self, robot, zone_ids) -> str:
        if robot not in self._missions:
            return "no such robot"
        unknown = [z for z in zone_ids if not self.zones.exists(z)]
        if unknown:
            return f"no such zone {unknown}"
        if not zone_ids and self.zones.held_by(robot):
            return f"no zones listed while it holds {self.zones.held_by(robot)}"
        return ""

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
        m.arrival_by = None
        self._event("INFO", "COMM_RESTORED", "checking the pose before resuming", m.name)

    def _finish_recoveries(self, now: float) -> None:
        for m in self._missions.values():
            if m.live and m.comm == RECOVERING and now >= m.recover_at:
                self._finish_recovery(m)

    def _finish_recovery(self, m: _Mission) -> None:
        m.comm = ""
        waypoint = self._resume_waypoint(m)
        zone = self._zone(waypoint)
        for held in self.zones.held_by(m.name):
            self._release_if_cleared(m, held)
        if zone not in self.zones.held_by(m.name) and not self.zones.request(m.name, zone):
            m.fault = ERROR
            m.fault_reason = "RECOVERY_CONFLICT"
            self.zones.cancel_all_requests(m.name)
            self._event("ERROR", "RECOVERY_CONFLICT",
                        f"{m.name} is in {zone} held by {self.zones.holder(zone)}", m.name)
            return
        m.path = [waypoint]
        m.idx = 0
        if m.phase == AT_STATION:
            return
        if m.phase == DOCKING and waypoint == self.site.stations[m.dest].dock:
            return
        m.settle_idx = 0
        if m.dispatch:
            self._start_dispatch_leg(m)
        else:
            m.phase = PARKED if waypoint in self.site.parking else IDLE
            m.dest = None

    def _resume_waypoint(self, m: _Mission):
        """Where a robot back from silence stands. One report can be off, so what it holds decides.

        Within d of a waypoint it counts as on that waypoint, as a driving robot does. Just past d
        the zone behind may be gone already, and a report short of d must not send it back there.
        """
        waypoint = self.site.nearest_waypoint(m.x, m.y, radius=self.params.release_distance_m)
        held = self.zones.held_by(m.name)
        if self._zone(waypoint) in held:
            return waypoint
        ahead = self.site.nearest_waypoint(m.x, m.y)
        return ahead if self._zone(ahead) in held else waypoint

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
