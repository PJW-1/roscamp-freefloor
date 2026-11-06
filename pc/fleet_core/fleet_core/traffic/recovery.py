"""통신 끊김, 복구, 교착 후퇴, 운영자의 confirm_clear.

파일을 짧게 하려고 TrafficController에서 떼어 낸 부분이다. 제어기 상태를 쓰므로 혼자 쓰지 않는다.
공개 메서드는 confirm_clear뿐이고 나머지는 tick에서 부른다.
"""
from __future__ import annotations

from fleet_core.common.commands import DispatchResult
from fleet_core.traffic.deadlock import choose_retreat, find_cycle, priority, wait_graph
from fleet_core.traffic.mission import (
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
        """운영자가 멈춘 로봇을 해제했다. 자세로 판단하지 않는 유일한 해제다.

        놓은 구역을 돌려주고, 거절하면 아무것도 안 바꾸고 None. 맵에 오른 적 없는 로봇은
        빈 목록이 맞다. 일부 구역만 주면 의도로 보고 경고하며, 나머지는 다시 불러 푼다.
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
            # 이 로봇은 더는 자세로 판단하지 않으니 다른 무엇도 이 구역을 풀지 않는다.
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
                # 말 없는 로봇이 뒤 대기자에게 구역이 넘어가는 것을 막으면 안 된다.
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
        """통신이 돌아온 로봇의 위치. 보고 하나는 틀릴 수 있어 쥔 구역으로 정한다.

        달리는 로봇처럼 지점에서 d 안이면 그 지점이다. d를 막 지나면 뒤 구역은 이미 놓았을 수
        있으니, d에 못 미친 보고 하나로 그리로 되돌리면 안 된다.
        """
        waypoint = self.site.nearest_waypoint(m.x, m.y, radius=self.params.release_distance_m)
        held = self.zones.held_by(m.name)
        if self._zone(waypoint) in held:
            return waypoint
        ahead = self.site.nearest_waypoint(m.x, m.y)
        return ahead if self._zone(ahead) in held else waypoint

    # ---- deadlock safety net ------------------------------------------------------------

    def _priority(self, robot: str) -> int:
        """교통이 아는 것만 본다: 스테이션에 있는지, 지정 배차가 있는지."""
        m = self._missions[robot]
        return priority(self.params, m.phase == AT_STATION, m.dispatch is not None)

    def _check_deadlock(self, now: float) -> bool:
        """대기 순환이 있는 동안 True. 그동안 새 이동과 구역 허가를 멈춘다."""
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
        """아직 들어가지 않은 구역 예약을 내놓는다. 실제 후진은 아직 없다."""
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
        """멈춘 로봇이 아직 밖에 있는 구역만 놓는다."""
        robot, zones, _ = self._retreating
        m = self._missions[robot]
        for zone in zones:
            self._release_if_cleared(m, zone)
        nothing_released = set(zones) <= set(self.zones.held_by(robot))
        if nothing_released and any(self._off_the_floor(m, zone) for zone in zones):
            # 밖에 있지만 보고가 아직 모자라다. 다음 보고로 확인될 수 있다.
            self._retreating = (robot, zones, self._clock() + self.params.recover_check_s)
            return
        self._retreating = None
        if nothing_released:
            self._event("ERROR", "DEADLOCK_UNRESOLVED",
                        f"{robot} of {cycle} entered {zones} before it stopped", robot)
