"""로봇 선택, 구역 잠금, 이동을 맡는다. ROS, 실제 시계, 주문은 모른다.

입력은 on_* 메서드로 받고 tick()을 주기적으로 부른다. tick()은 보낼 명령, 이벤트,
배차 피드백과 결과를 돌려준다. 스테이션에서 하는 일은 모른다. 놓아 주거나 다른 곳으로
보낼 때까지 그 자리에 서 있다(AT_STATION).

안전 규칙(각각 테스트가 있다):
  - 로봇은 현재 구역과 다음 구역 하나까지만 쥐고 그 안에서만 움직인다.
  - 구역은 연속 보고가 해제 거리만큼 지난 뒤에만 놓는다. 오차 섞인 보고 하나로
    로봇 몸이 아직 붙어 있는 구역을 풀지 않게 한다.
  - 지금 활성 goal이 아닌 goal의 결과는 무시한다.
  - 모든 로봇이 보고하기 전에는 아무것도 안 움직인다. 보고 안 한 로봇이 다른 로봇에게
    줄 구역에 서 있을 수 있기 때문이다.
"""
from __future__ import annotations

import math

from fleet_core.common.commands import (
    NO_ROBOT,
    CancelCmd,
    DispatchFeedback,
    DispatchResult,
    DockCmd,
    EventOut,
    MoveCmd,
    ZoneView,
)
from fleet_core.traffic.blocking import TrafficBlocking
from fleet_core.traffic.detour import TrafficDetour
from fleet_core.traffic.isolation import TrafficIsolation
from fleet_core.traffic.mission import (
    AT_STATION,
    BLOCKED,
    COMM_LOST,
    DOCK,
    DOCKING,
    ERROR,
    FILL,
    HEARTBEAT_LOST,
    IDLE,
    LOCALIZING,
    MOVE,
    NOT_AT_TARGET,
    NOT_PLACEABLE,
    OUT_OF_SERVICE,
    PARKED,
    PLACEMENT_CONFLICT,
    RECOVERING,
    RUN,
    STALLED,
    STOP,
    SWAP,
    TO_OUTBOUND,
    TO_PARK,
    TO_WORK,
    _Dispatch,
    _Goal,
    _Mission,
)
from fleet_core.traffic.off_lane import TrafficOffLane
from fleet_core.traffic.recovery import TrafficRecovery
from fleet_core.traffic.selector import RobotInfo, RobotSelector
from fleet_core.traffic.zones import ZoneLocks


class TrafficController(TrafficOffLane, TrafficRecovery, TrafficBlocking, TrafficDetour,
                        TrafficIsolation):
    def __init__(self, site, params, robots, clock, next_goal_id=None):
        if site.shortest_exit_lane() <= params.release_distance_m:
            raise ValueError(
                f"the shortest lane leaving a zone ({site.shortest_exit_lane()} m) must exceed "
                f"the release distance ({params.release_distance_m} m), or a robot stopped at "
                "its end never releases the zone behind it")
        if params.release_distance_m <= 2 * params.robot_radius_m:
            raise ValueError(
                f"the release distance ({params.release_distance_m} m) must exceed twice the "
                f"robot radius ({params.robot_radius_m} m), or all lanes meeting at a waypoint "
                "become conflict pairs and no two robots can stand on neighbouring waypoints")
        self.site = site
        self.params = params
        self.zones = ZoneLocks(site.zones, clock, site.conflict_pairs(
            2 * params.robot_radius_m, params.release_distance_m))
        self.selector = RobotSelector(site, params)
        self._clock = clock
        self._next_goal_id = next_goal_id or self._own_goal_id
        self._goal_seq = 0
        self._missions = {name: _Mission(name) for name in robots}
        self._out: list = []
        self._run_mode: str | None = None
        self._open_calls: list[int] = []  # 로봇 지정 없는 채우기 스테이션 배차
        self._work = next(s.id for s in site.stations.values() if s.action == FILL)
        self._outbound = next(s.id for s in site.stations.values() if s.action == SWAP)
        self._deadlock_since: float | None = None
        self._deadlock_acted = False
        self._retreating: tuple[str, list[str], float] | None = None  # 로봇, 구역, 해제 시각
        self._startup_since: float | None = None
        self._missing_reported = False
        self._no_robot_warned = False
        self.clear_reason = ""  # 마지막 confirm_clear 거부 이유, 받아들이면 빈 값

    # ---- inputs -------------------------------------------------------------------------

    # 공개 메서드는 모르는 로봇 이름을 무시한다. INPUT_REJECTED는 FleetManager가 낸다.

    def on_robot_state(self, robot, x, y, yaw, state, battery=0.0) -> None:
        m = self._missions.get(robot)
        if m is None:
            return
        if state != BLOCKED:
            m.queue_since = None  # 줄 서기 시계는 BLOCKED가 이어지는 동안만 잰다
        if state == LOCALIZING:
            # 자세를 믿지 않는다. 보고 시각도 두어 오래 가면 COMM_LOST가 된다.
            m.robot_state, m.battery = state, battery
            return
        if m.comm == RECOVERING and m.robot_state == LOCALIZING:
            # 위치를 다시 잡았다. 복구 확인을 이 보고부터 다시 잰다
            m.recover_at = self._clock() + self.params.recover_check_s
        m.x, m.y, m.yaw, m.robot_state, m.battery = x, y, yaw, state, battery
        m.last_seen = self._clock()
        if m.arrival_by is not None:
            m.heard_since_success = True
        if not m.placed and m.fault != OUT_OF_SERVICE:
            self._place(m)
        elif m.comm == COMM_LOST and not m.fault:
            self._begin_recovery(m)
        if m.placed:
            self._count_reports_outside(m)
            self._watch_lane_error(m)
            self._note_block_report(m)

    def on_run_mode(self, run_mode) -> None:
        if run_mode == STOP and self._run_mode != STOP:
            for m in self._missions.values():
                # 말 없는 로봇은 못 들을 수 있어 취소는 복구 때 보낸다.
                if m.goal and m.comm != COMM_LOST:
                    self._cancel_goal(m)
        if run_mode != self._run_mode:
            for m in self._missions.values():
                m.stall_from = None  # 모드가 바뀌면 멈춤 시계를 다시 잰다
        self._run_mode = run_mode

    def on_dispatch(self, goal_id, station_id, robot_id="") -> None:
        """스테이션으로 로봇을 부른다. 로봇을 지정하지 않으면 열린 호출이 된다."""
        if not robot_id:
            if station_id != self._work:
                raise ValueError(f"only the fill station takes open calls, not {station_id}")
            self._open_calls.append(goal_id)
            return
        m = self._missions.get(robot_id)
        if m is None:
            self._out.append(DispatchResult(goal_id, False, robot_id, "NO_SUCH_ROBOT"))
            return
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
        m = self._missions.get(robot)
        if m is None:
            return
        m.hold = False
        m.await_pick = True

    def on_robot_fault(self, robot, reason="") -> None:
        m = self._missions.get(robot)
        if m is not None:
            self._fail(m, reason)

    def on_move_result(self, robot, goal_id, success, reason) -> None:
        m = self._missions.get(robot)
        if m:
            self._cancel_finished(m, goal_id)
        goal = m.goal if m else None
        if goal is None or goal.id != goal_id or goal.kind != MOVE:
            return
        m.goal = None
        m.not_sent_warned = False
        if success and not m.comm and not self._at_target(m, goal.target_idx):
            m.settle_idx = goal.target_idx
            m.arrival_by = m.retry_at = self._clock() + self.params.arrival_settle_s
            m.heard_since_success = False
        elif success:
            m.idx = max(m.idx, goal.target_idx)
            m.attempts = 0
        elif reason == HEARTBEAT_LOST:
            self._heartbeat_stopped(m, goal)
        else:
            m.settle_idx = goal.target_idx
            self._goal_failed(m, reason)

    def on_goal_rejected(self, robot, goal_id) -> None:
        m = self._missions.get(robot)
        if m:
            self._cancel_finished(m, goal_id)
        goal = m.goal if m else None
        if goal is None or goal.id != goal_id:
            return
        m.goal = None
        m.not_sent_warned = False
        if goal.kind == MOVE:
            m.settle_idx = goal.target_idx
        self._goal_failed(m, "REJECTED")

    def on_goal_not_sent(self, robot, goal_id) -> None:
        """액션 서버가 없어 goal이 나가지 않았다. 시도 횟수로 치지 않는다."""
        m = self._missions.get(robot)
        if m:
            self._cancel_finished(m, goal_id)
        goal = m.goal if m else None
        if goal is None or goal.id != goal_id:
            return
        m.goal = None
        if goal.kind == MOVE:
            m.settle_idx = goal.target_idx
        m.retry_at = self._clock() + self.params.move_retry_wait_s
        if not m.not_sent_warned:
            m.not_sent_warned = True
            self._event("WARN", "GOAL_NOT_SENT", f"no {goal.kind} server, trying again", robot)

    def on_dock_result(self, robot, goal_id, success, error_x_cm=0.0, error_y_cm=0.0,
                       error_yaw_deg=0.0) -> None:
        m = self._missions.get(robot)
        goal = m.goal if m else None
        if goal is None or goal.id != goal_id or goal.kind != DOCK:
            return
        m.goal = None
        m.not_sent_warned = False
        if not success:
            self._goal_failed(m, "DOCK_FAILED")
            return
        m.attempts = 0
        m.phase = AT_STATION
        self._event("INFO", "DOCKED",
                    f"{m.dest} error x={error_x_cm} y={error_y_cm} yaw={error_yaw_deg}", robot)
        self._finish_dispatch(m, error_x_cm, error_y_cm, error_yaw_deg)

    # 운영자 입력 confirm_clear는 recovery.py에 있다.

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
        m = self._missions.get(robot)
        return m.available if m else False

    def tick(self) -> list:
        """한 주기 전체. 단계 사이에 할 일이 없는 호출자용."""
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
        """시작과 통신 점검. 모든 로봇이 보고하면 True."""
        everyone_here = self._everyone_reported(now)
        for m in self._missions.values():
            if m.goal:  # 나가지 못한 goal은 다음 tick 전에 보고된다
                m.not_sent_warned = False
        self._check_comm(now)
        self._finish_recoveries(now)
        return everyone_here

    def tick_progress(self, now: float) -> bool:
        """자세를 따라가며 지난 구역을 놓는다. 대기 순환으로 새 이동을 멈춘 동안 True."""
        for m in self._missions.values():
            self._watch_off_lane(m, now)
            if m.available:
                self._judge_arrival(m, now)
            if m.available:  # 방금 도착 판정으로 격리됐으면 이번 판은 건너뛴다
                self._track_progress(m)
                self._release_passed_zones(m)
                self._watch_stall(m, now)
            else:
                m.stall_from = m.stall_mean = None
                m.stall_hold = False
            if m.block:
                self._watch_block(m, now)
        return self._check_deadlock(now)

    def tick_pick(self) -> None:
        """열린 호출에 응할 로봇을 정하고, 바뀐 로봇의 경로를 다시 잡는다."""
        # 해제된 로봇은 코스 밖이라 고장 로봇과 달리 아무도 막지 않는다.
        infos = [self._robot_info(m) for m in self._missions.values()
                 if m.placed and m.fault != OUT_OF_SERVICE]
        self._warn_if_no_robot()
        called = self.selector.pick(len(self._open_calls), infos)
        for m in self._missions.values():
            if m.available:
                m.called = m.name in called
                m.await_pick = False
                self._redirect(m)

    def step(self, robot) -> None:
        """현재 단계를 처리하고, 이어지는 단계도 바로 처리한다."""
        m = self._missions.get(robot)
        if m is None:
            return
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
        """모르는 로봇이면 빈 dict."""
        m = self._missions.get(robot)
        if m is None:
            return {}
        return {
            "state": m.state,
            "phase": m.phase,
            "on_map": m.placed,
            "fault": m.fault,
            "comm": m.comm,
            "called": m.called,
            "held": self.zones.held_by(robot),
            "path": list(m.path[m.idx:]),
            "waypoint": m.path[m.idx] if m.path else None,
            "destination": m.dest,
            "goal_id": m.goal.id if m.goal else None,
            "attempts": m.attempts,
            "blocked_lane": list(m.block.lane) if m.block and m.block.marked else None,
        }

    def status_view(self, robot) -> dict:
        """/fleet/robots에 나가는 값 중 교통만 아는 것."""
        m = self._missions.get(robot)
        if m is None:
            return {}
        wait_zone = self.zones.waiting_for(robot) or ""
        in_the_way = self.zones.blockers(robot, wait_zone) if wait_zone else ()
        fault, reason = self._shown_fault(m)
        return {
            "fsm_state": m.robot_state,
            "fault": fault,
            "fault_reason": reason,
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
        silent = [m.name for m in missing if not m.fault]  # 거절된 로봇은 이미 보고했다
        if (silent and not self._missing_reported
                and now - self._startup_since >= self.params.startup_wait_s):
            self._missing_reported = True
            self._event("ERROR", "ROBOT_NOT_REPORTING",
                        f"nothing moves until {silent} report a position and stand still")
        return False

    def _event(self, level, code, detail, robot="") -> None:
        self._out.append(EventOut(level, code, detail, robot))

    def _zone(self, waypoint) -> str:
        return self.site.zone_of[waypoint]

    def _place(self, m: _Mission) -> None:
        """보고한 위치의 구역을 잡는다. 못 잡으면 맵 밖에 두고 다음 보고 때 다시 한다.

        거절된 로봇은 잠금 없이 서 있으므로, 움직이거나 운영자가 해제할 때까지
        _everyone_reported가 모두를 세워 둔다. 이 FMS가 주지 않은 goal로 달리는 로봇도 같다.
        """
        if m.robot_state in NOT_PLACEABLE:
            return
        waypoint = self.site.nearest_waypoint(m.x, m.y, radius=self.params.release_distance_m)
        zone = self._zone(waypoint)
        in_the_way = self.zones.holders_around(zone, m.name)
        if not self.zones.force_acquire(m.name, zone):
            if m.fault_reason != PLACEMENT_CONFLICT:
                m.fault = ERROR
                m.fault_reason = PLACEMENT_CONFLICT
                self._event("ERROR", PLACEMENT_CONFLICT,
                            f"{m.name} cannot take {zone}, held: {in_the_way}", m.name)
            return
        m.placed = True
        m.path = [waypoint]
        m.idx = 0
        m.phase = PARKED if waypoint in self.site.parking else IDLE
        # d를 지나면 그 지점으로 가는 레인 위라서, 거기서 곧장 가면 레인을 벗어난다.
        if (math.dist((m.x, m.y), self.site.waypoint_pose(waypoint))
                > self.params.release_distance_m):
            m.settle_idx = 0
        if m.fault_reason == PLACEMENT_CONFLICT:
            m.fault = ""
            m.fault_reason = ""

    def _upcoming(self, m: _Mission) -> tuple[str, str | None]:
        """현재 구역과 경로상 다음 다른 구역."""
        current = self._zone(m.path[m.idx])
        for waypoint in m.path[m.idx + 1:]:
            zone = self._zone(waypoint)
            if zone != current:
                return current, zone
        return current, None

    def _anchor(self, m: _Mission) -> int:
        """경로를 바꾸기 전에 반드시 도달해야 하는 경로 인덱스."""
        if m.goal and m.goal.kind == MOVE:
            return m.goal.target_idx
        return m.settle_idx if m.settle_idx is not None else m.idx

    def _cancel_goal(self, m: _Mission) -> None:
        self._out.append(CancelCmd(m.name, m.goal.id))
        if m.goal.kind == MOVE:
            m.settle_idx = m.goal.target_idx
        m.goal = None

    def _fail(self, m: _Mission, reason: str = "") -> None:
        """구역은 계속 쥔다. 처음 ERROR가 되면 격리한다(isolation.py)."""
        first = not m.fault
        m.fault = ERROR
        m.fault_reason = reason
        m.called = False
        self.zones.cancel_all_requests(m.name)
        if m.dispatch:
            self._out.append(DispatchResult(m.dispatch.goal_id, False, m.name, reason))
            m.dispatch = None
        if first:
            self._isolate(m, reason)

    def _at_target(self, m: _Mission, index: int) -> bool:
        target = self.site.waypoint_pose(m.path[index])
        return math.dist((m.x, m.y), target) <= self.params.arrival_tolerance_m

    def _judge_arrival(self, m: _Mission, now: float) -> None:
        """이동은 성공했는데 마지막 보고가 너무 멀다. 이후 보고로 판단한다.

        성공 뒤에 온 보고만 실패로 만들 수 있다. 없으면 자세가 옛것이라 기다리고,
        통신 점검이 먼저 알린다.
        """
        if m.arrival_by is None:
            return
        if self._at_target(m, m.settle_idx):
            m.idx = max(m.idx, m.settle_idx)
            m.settle_idx = None
            m.arrival_by = None
            m.retry_at = 0.0
            m.attempts = 0
        elif now >= m.arrival_by and m.heard_since_success:
            m.arrival_by = None
            self._goal_failed(m, NOT_AT_TARGET)

    def _goal_failed(self, m: _Mission, reason: str, announce: bool = True) -> None:
        m.attempts += 1
        m.retry_at = self._clock() + self.params.move_retry_wait_s
        if m.attempts > self.params.move_retry_limit:
            self._fail(m, reason)
            self._event("ERROR", "GOAL_FAILED",
                        f"gave up after {m.attempts} attempts: {reason}", m.name)
        elif announce:
            self._event("WARN", "GOAL_RETRY", f"attempt {m.attempts} failed: {reason}", m.name)

    def _watch_stall(self, m: _Mission, now: float) -> None:
        """목표가 열린 채 제자리면 취소하고 같은 목표를 다시 낸다. 시도 한 번으로 센다.

        목표 없이 구역을 기다리는 것과 RUN이 아닐 때는 재지 않는다. 목표를 바꿔도 시계는 그대로다.
        막힘과 줄 서기 동안은 쉰다. heartbeat로 선 goal은 다시 보낼 때까지 시계를 잇는다.
        """
        here = self._smoothed(m, now)
        idle = m.goal is None and not m.stall_hold
        if idle or self._run_mode != RUN or m.block or self._queue_rests(m):
            m.stall_from = None
            return
        if m.stall_from is None or math.dist(here, m.stall_from) >= self.params.stall_move_m:
            m.stall_from, m.stall_since = here, now
            return
        kind = m.goal.kind if m.goal else MOVE
        limit = self.params.dock_stall_s if kind == DOCK else self.params.stall_s
        if now - m.stall_since < limit:
            return
        m.stall_from = None
        m.stall_hold = False
        if m.goal:
            self._cancel_goal(m)
        self._event("WARN", "GOAL_STALLED",
                    f"no progress for {limit:.0f} s on a {kind}, attempt {m.attempts + 1}", m.name)
        self._goal_failed(m, STALLED, announce=False)

    def _smoothed(self, m: _Mission, now: float) -> tuple[float, float]:
        """보고 위치의 지수 평균. 잡음 하나로 멈춤 시계가 다시 시작되지 않게 한다."""
        if m.stall_mean is None:
            m.stall_mean = (m.x, m.y)
        else:
            tau = self.params.stall_smooth_s
            a = 1.0 - math.exp(-(now - m.stall_seen) / tau) if tau > 0 else 1.0
            mx, my = m.stall_mean
            m.stall_mean = (mx + a * (m.x - mx), my + a * (m.y - my))
        m.stall_seen = now
        return m.stall_mean

    # ---- progress and zone release ------------------------------------------------------

    def _track_progress(self, m: _Mission) -> None:
        """충분히 지난 레인마다 경로 인덱스를 넘긴다. 현재 목표까지만."""
        if not m.goal or m.goal.kind != MOVE:
            return
        while m.idx < m.goal.target_idx:
            s, _ = self.site.lane_progress(m.path[m.idx], m.path[m.idx + 1], m.x, m.y)
            if s <= self.params.release_distance_m:
                return
            m.idx += 1

    def _exit_lane(self, m: _Mission, zone: str) -> tuple | None:
        """로봇이 있거나 있던 구역을 경로가 떠나는 레인."""
        last = m.idx
        while last + 1 < len(m.path) and self._zone(m.path[last + 1]) == self._zone(m.path[last]):
            last += 1
        for index in range(min(last, len(m.path) - 2), -1, -1):
            a, b = m.path[index], m.path[index + 1]
            if self._zone(a) == zone and self._zone(b) != zone:
                return a, b
        return None

    def _clear_of(self, m: _Mission, zone: str) -> bool:
        """로봇 몸이 쥔 구역에서 벗어났는가?

        경로 위면 구역을 떠나는 레인에서 s > d. d가 충분한 이유는 fleet_params.yaml에 있다.
        경로 밖(앞 구역, 손으로 옮긴 로봇)이면 구역 중심선에서 d 이상, s와 같은 기준이다.

        s는 투영이라 레인에서 멀리 벗어난 로봇은 못 잡는다. 나가는 레인에 가까운 구역 레인은
        이미 충돌 쌍이고, 나머지는 이탈 감시가 세운다(off_lane.py).
        """
        lane = self._exit_lane(m, zone)
        if lane:
            s, _ = self.site.lane_progress(*lane, m.x, m.y)
            return s > self.params.release_distance_m
        return self._off_the_floor(m, zone)

    def _off_the_floor(self, m: _Mission, zone: str) -> bool:
        # 바닥은 중심선 양쪽 half_width까지라, 더해서 중심선 기준 거리로 되돌린다.
        distance = self.site.distance_to_zone(m.x, m.y, zone) + self.site.half_width
        return distance >= self.params.release_distance_m

    def _count_reports_outside(self, m: _Mission) -> None:
        for zone in self.zones.held_by(m.name):
            if self._clear_of(m, zone):
                m.outside[zone] = m.outside.get(zone, 0) + 1
            else:
                m.outside.pop(zone, None)

    # 레인 이탈 감시(_watch_lane_error)는 off_lane.py에 있다.

    def _release_if_cleared(self, m: _Mission, zone: str) -> None:
        if m.off_lane_since is not None:
            return  # 벗어난 로봇의 실제 위치는 모른다
        if m.fault == ERROR:
            return  # 격리된 로봇의 구역은 confirm_clear만 푼다
        if m.block_cancel is not None or (m.block and m.block.marked):
            # 막힘 표시 중에는 막힘 쪽만 놓고, 취소한 goal의 결과 전에는 아무것도 놓지 않는다
            return
        if m.outside.get(zone, 0) >= self.params.release_confirm_samples:
            self.zones.release(m.name, zone)
            m.outside.pop(zone, None)

    def _release_passed_zones(self, m: _Mission) -> None:
        """벗어났고 다음에 필요 없는 구역을 놓는다.

        필요한 구역은 현재와 다음 구역뿐이다. 한참 뒤 다시 지날 구역(한 바퀴 도는 로봇은
        Z-X를 두 번 지난다)을 그동안 쥐고 있으면 안 된다.
        """
        keep = self._upcoming(m)
        for zone in self.zones.held_by(m.name):
            if zone not in keep:
                self._release_if_cleared(m, zone)

    # ---- calling robots -----------------------------------------------------------------

    def _robot_info(self, m: _Mission) -> RobotInfo:
        # 대기 지점 안에서 달리는 로봇은 멈출 지점에서, 빠져나가는 로봇은 아직 선 지점에서 잰다.
        index = self._anchor(m)
        if m.path[index] not in self.site.parking:
            index = m.idx
        waypoint = m.path[index]
        return RobotInfo(
            robot_id=m.name,
            waypoint=waypoint,
            called=m.called,
            parked=waypoint in self.site.parking,
            # 막힘으로 선 로봇은 새 호출을 받지 않고, 대기 줄에서는 뒤 로봇이 앞지르지 않는다
            eligible=(m.available and not m.hold and m.dispatch is None
                      and (m.called or not self._held_for_block(m))),
            available=m.available and not self._held_for_block(m),
        )

    def _committed(self, m: _Mission) -> bool:
        """채우기 도크로 가는 로봇은 commit 구역을 잡으면 방향을 바꿀 수 없다."""
        if m.phase != TO_WORK:
            return False
        zone = self._zone(self.site.stations[self._work].commit)
        if zone in self.zones.held_by(m.name):
            return True
        return zone not in {self._zone(c) for c in m.path[m.idx:]}

    def _warn_if_no_robot(self) -> None:
        """호출이 남았는데 운영자가 손대기 전엔 올 로봇이 없다. 멈출 때마다 한 번 알린다."""
        stalled = bool(self._open_calls) and not any(m.live for m in self._missions.values())
        if stalled and not self._no_robot_warned:
            self._out.append(DispatchFeedback(self._open_calls[0], "", NO_ROBOT))
        self._no_robot_warned = stalled

    def _redirect(self, m: _Mission) -> None:
        """빈 채로 달리는 로봇을 지금 호출이 가리키는 곳으로 보낸다."""
        if m.phase not in (TO_WORK, TO_PARK) or m.dispatch or self._committed(m):
            return
        if m.called:
            if m.phase != TO_WORK:
                self._start_leg(m, TO_WORK, self._work, self.site.stations[self._work].dock)
        elif m.phase == TO_WORK:
            self._start_park_leg(m)

    def _finish_dispatch(self, m: _Mission, error_x_cm=0.0, error_y_cm=0.0,
                         error_yaw_deg=0.0) -> None:
        """도크에 섰다. 요청한 쪽에 답하고 다음 지시를 기다린다."""
        if m.dispatch:
            goal_id, m.dispatch = m.dispatch.goal_id, None
        elif m.dest == self._work and self._open_calls:
            goal_id = self._open_calls.pop(0)
        else:
            m.await_pick = True  # 이제 이 로봇을 기다리는 쪽이 없다
            return
        m.hold = True
        self._out.append(DispatchResult(goal_id, True, m.name, "", error_x_cm, error_y_cm,
                                        error_yaw_deg))

    # ---- mission steps ------------------------------------------------------------------

    def _start_leg(self, m: _Mission, phase: str, dest: str | None, goal_waypoint,
                   way=None) -> None:
        """반드시 도달할 지점 이후의 경로를 바꾼다. way는 우회가 고른 그 지점부터의 길."""
        _, old_next = self._upcoming(m) if m.path else ("", None)
        anchor = self._anchor(m)
        tail = way or self.site.plan(m.path[anchor], goal_waypoint)
        behind = max(m.idx - 1, 0)  # 방금 지난 레인은 남긴다. 그 구역을 아직 못 놓았을 수 있다
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

    def _park_goal(self, m: _Mission, from_waypoint):
        """다른 로봇을 앞지르지 않고 갈 수 있는 가장 깊은 대기 지점."""
        spots = self.site.parking
        start = spots.index(from_waypoint) + 1 if from_waypoint in spots else 0
        goal = None
        for waypoint in spots[start:]:
            zone = self._zone(waypoint)
            holder = self.zones.holder(zone)
            if (holder and holder != m.name) or set(self.zones.queue(zone)) - {m.name}:
                break
            goal = waypoint
        return goal

    def _start_park_leg(self, m: _Mission) -> None:
        anchor_waypoint = m.path[self._anchor(m)]
        goal = self._park_goal(m, anchor_waypoint)
        if goal is None:
            # 이미 대기 중이고 앞이 막혔으면 그대로, 밖이면 입구를 기다린다.
            goal = anchor_waypoint if anchor_waypoint in self.site.parking else self.site.parking[0]
        self._start_leg(m, TO_PARK, None, goal)

    def _next_work(self, m: _Mission) -> None:
        """다음 일을 위해 도크나 대기 지점을 떠난다."""
        if m.dispatch:
            self._start_dispatch_leg(m)
        elif m.called:
            self._start_leg(m, TO_WORK, self._work, self.site.stations[self._work].dock)
        else:
            self._start_park_leg(m)

    def _step_phase(self, m: _Mission) -> None:
        if self._held_for_block(m):
            # 막힌 레인 앞에 서서 풀리기와 취소 결과를 기다린다. 결과 전 새 goal은 추적을 흐린다
            return
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
            self._finish_dispatch(m)  # 불린 도크에 이미 서 있을 때
            return
        self._next_work(m)

    def _step_parked(self, m: _Mission) -> None:
        """대기 입구가 비도록 빈 지점으로 계속 앞으로 간다."""
        if m.settle_idx is not None:  # 이 자리로 오는 레인 위에서 발견됨
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
        if m.goal is None and m.settle_idx is None and m.idx == len(m.path) - 1:
            self._arrive(m)  # 우회한 길이 여기서 끝난다
            return
        self._issue_move(m)

    def _trim_parking_path(self, m: _Mission) -> None:
        """대기 지점 안에서 목표를 다시 잡는다. 먼저 온 로봇 뒤에 서거나 더 깊이 간다."""
        anchor = self._anchor(m)
        waypoint = m.path[anchor]
        spots = self.site.parking
        old_tail = m.path[anchor + 1:]
        if waypoint not in spots and old_tail[:1] != [spots[0]]:
            return
        goal = self._park_goal(m, waypoint)
        start = spots.index(waypoint) + 1 if waypoint in spots else 0
        new_tail = spots[start:spots.index(goal) + 1] if goal else []
        if waypoint not in spots and not new_tail:
            return  # 첫 대기 지점이 찼으니 계속 기다린다
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
        """현재 구역 말고 쥔 것이 없을 때만 다음 구역을 요청한다."""
        current, following = self._upcoming(m)
        if following is None:
            return
        held = self.zones.held_by(m.name)
        if following in held or any(zone != current for zone in held):
            m.wait_zone = None
            return
        if self._consider_detour(m, following):
            current, following = self._upcoming(m)
            if following is None:  # 새 길은 이 구역 안에서 끝난다
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
        if m.block or m.block_cancel is not None:
            return  # 막힌 동안과 취소 결과 전에는 목표를 늘리거나 새로 내지 않는다
        if m.goal is None and (self._clock() < m.retry_at or m.arrival_by is not None):
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
        """레인 방향. 한 직선 위 레인이 같게 나오도록 반올림한다."""
        lane = self.site.lane(m.path[index], m.path[index + 1])
        return (round(lane.ux, 6), round(lane.uy, 6))

    def _step_into(self, m: _Mission, index: int) -> tuple[float, float] | None:
        return self._step_between(m, index - 1) if index > 0 else None

    def _send_move(self, m: _Mission, target_idx: int, step) -> None:
        """새 goal은 진행 중인 goal을 대신한다. 이후 결과는 이 id와 맞아야 한다."""
        waypoint = m.path[target_idx]
        x, y = self.site.waypoint_pose(waypoint)
        yaw = math.atan2(step[1], step[0]) if step else m.yaw
        goal_id = self._next_goal_id()
        m.goal = _Goal(MOVE, goal_id, target_idx, step)
        m.stall_hold = False
        self._out.append(MoveCmd(m.name, goal_id, self.site.waypoint_id(waypoint), x, y, yaw))
