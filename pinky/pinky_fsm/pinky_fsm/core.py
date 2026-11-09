"""핑키 FSM의 판단. ROS를 모르고, 입력을 받아 명령 목록을 돌려준다.

규칙의 출처
- 안내 = Drive 핑키_FSM_Nav2_구현_안내_2026-11-07.md (2.1 교체 때 서지 않음, 2.2 연결 확인)
- 계약 = docs/핑키_계약.md (3절 지킬 것, 4절 BLOCKED)
- R번호 = Drive 핑키_제어파트_전달사항_2026-11-06 3절
- 자가 복구 = 핑키_계약 3절 7번(2026-11-09 제안): 막히면 쥔 구역 안에서 짧게 물러나고 지도
  장애물을 비운 뒤 같은 선을 다시 따른다. 두 번까지 하고, 그동안 RECOVERING으로 보고한다.

시각은 모두 벽시계 초다(heartbeat는 벽시계로 재야 하고, 나머지도 같은 축을 쓴다).
노드는 SendPath를 Nav2 FollowPath 목표로, CancelNav를 그 취소로, Finish를 액션 결과로 옮긴다.
"""
from __future__ import annotations

import math
from collections.abc import Callable, Hashable, Sequence
from dataclasses import dataclass, field

from pinky_fsm.backup import BackUp, CancelBackUp, backup_distance, keep_lateral
from pinky_fsm.geometry import Point, Polyline, Pose, poses_along

# RobotState.STATE_* 와 같은 값
IDLE = "IDLE"
MOVING = "MOVING"
DOCKING = "DOCKING"
DOCKED = "DOCKED"
BLOCKED = "BLOCKED"
LOCALIZING = "LOCALIZING"
RECOVERING = "RECOVERING"  # 2026-11-09 제안: 자가 복구 중

# MoveTo.Result.REASON_* 와 같은 값
NAV_ABORTED = "NAV_ABORTED"
CANCELED = "CANCELED"
TIMEOUT = "TIMEOUT"
HEARTBEAT_LOST = "HEARTBEAT_LOST"
PATH_NOT_CONNECTED = "PATH_NOT_CONNECTED"
DOCK_UNAVAILABLE = "DOCK_UNAVAILABLE"  # Dock 결과에는 reason이 없다. 로그에만 남는다

Obstacle = Callable[[list[Point]], bool]


@dataclass(frozen=True)
class FsmParams:
    arrival_tolerance_m: float = 0.15  # R4, 관제 arrival_tolerance_m
    connect_tolerance_m: float = 0.15  # 안내 2.2, 관제 lane_error_warn_m
    refresh_within_m: float = 0.15  # R13
    refresh_every_s: float = 0.5  # R13
    heartbeat_timeout_s: float = 1.5  # R10. 0이면 멈추지 않는다
    stopped_speed_mps: float = 0.02  # 계약 4절
    stopped_turn_rps: float = 0.1  # 제자리 회전은 서 있는 것이 아니다
    block_report_s: float = 1.0  # 계약 4절: 1초 서 있으면
    block_clear_s: float = 1.0  # 계약 4절: 1초 안 보이면 끈다
    block_lookahead_m: float = 0.5  # 계약 4절: 경로 위 0.5 m 안
    # Proposed. A goal blocked this long ends with TIMEOUT. The fleet manager cancels a blocked
    # goal after blocked_mark_s (5 s); this is the last safeguard for when it does not
    block_timeout_s: float = 90.0
    block_sample_m: float = 0.02  # costmap 해상도(0.025)보다 촘촘하게
    path_step_m: float = 0.05  # 전달사항 6.1 poses_along
    far_success_limit: int = 3  # 목표에서 먼 Nav2 성공을 몇 번까지 다시 보낼까
    track_back_m: float = 0.2  # 진행 거리를 찾을 때 뒤로 허용하는 거리
    track_ahead_m: float = 1.0  # 진행 거리를 찾을 때 앞으로 보는 거리
    # 자가 복구(핑키_계약 3절 7번, 2026-11-09 제안). 숫자는 추정이다
    recovery_attempts: int = 2  # 두 번 실패하면 BLOCKED(물체 앞)나 NAV_ABORTED로 넘긴다
    backup_max_m: float = 0.15
    # 관제 release_distance_m. 꼭짓점에서 이만큼 가면 뒤 레인이 풀려 뒤차가 꼭짓점에 설 수 있다.
    # 물러난 중심은 마지막으로 지난 경로 꼭짓점에서 이것 + backup_margin_m 앞에 남는다
    release_distance_m: float = 0.2
    backup_margin_m: float = 0.03  # 관제가 보는 위치 오차(eps)
    backup_min_m: float = 0.02  # 이보다 짧으면 물러나지 않고 지도만 비운다
    # 후진은 로봇 방향으로 곧게 가므로 옆으로 밀린다. 관제가 위치 오차로 보는 값(eps)까지만
    backup_lateral_limit_m: float = 0.03
    backup_speed_mps: float = 0.08
    backup_time_allowance_s: float = 5.0
    backup_result_grace_s: float = 2.0  # time allowance 뒤에도 결과가 없으면 멈추고 이어 간다
    recovery_pass_m: float = 0.05  # 막힌 곳을 이만큼 지나면 복구가 끝난다
    recovery_timeout_s: float = 10.0  # 다시 보낸 뒤 이만큼 못 지나면 다음 시도로 친다
    recovery_reset_m: float = 0.3  # 첫 시도 자리에서 이만큼 벗어나면 시도 수를 지운다
    # 잠금 경계에서 끝나는 목표(final_goal 거짓, 버전 2 4절 제안): 끝까지 남은 거리 d에서
    # √(2·a·d)로 제한한다. 숫자는 추정이다(실측 a 전). 한계 이상이면 제한하지 않는다
    horizon_decel_mps2: float = 0.25
    horizon_min_mps: float = 0.05
    horizon_cap_mps: float = 0.3


@dataclass(frozen=True)
class SendPath:
    """Nav2 FollowPath에 이 경로를 보낸다. 달리는 목표는 취소하지 않는다(안내 2.1)."""
    seq: int
    poses: list[Pose] = field(compare=False)
    # False when the path ends at a lock boundary, not at the mission end (MoveTo.final_goal).
    # The node may then use a controller without the approach slow-down
    final_goal: bool = True


@dataclass(frozen=True)
class CancelNav:
    """Nav2 FollowPath 목표를 취소한다. controller가 속도 0을 내고 선다."""


@dataclass(frozen=True)
class LimitSpeed:
    """Nav2 controller의 속도 제한(m/s). 0이면 제한이 없다(nav2_msgs/SpeedLimit)."""
    mps: float


@dataclass(frozen=True)
class ClearCostmap:
    """local costmap을 비운다. 다음 라이다 스캔이 지금 있는 물체를 다시 그린다."""


@dataclass(frozen=True)
class Finish:
    goal_id: Hashable
    success: bool
    reason: str


@dataclass
class _Move:
    goal_id: Hashable
    target_id: str
    target: Pose
    route: Polyline
    s: float = 0.0  # 로봇이 선 위에서 간 거리
    final_goal: bool = True  # 끝이 임무의 마지막 지점인가(계약 0.8-draft 제안)
    seq: int = 0  # 마지막으로 보낸 FollowPath 순번
    nav_running: bool = False
    last_send: float = 0.0
    far_successes: int = 0
    waiting_since: float | None = None  # when Nav2 stopped for an obstacle; resend when clear
    blocked_since: float | None = None  # when this goal became blocked (block_timeout_s)


@dataclass
class _Recovery:
    """자가 복구 한 번. 시도마다 막힌 곳(stuck_s, 선 위 거리)을 새로 잡는다."""
    stuck_s: float
    backup_seq: int | None = None  # 물러나는 중이면 그 BackUp 순번
    backup_since: float = 0.0
    follow_since: float = 0.0  # 남은 선을 다시 보낸 시각


@dataclass
class _Dock:
    goal_id: Hashable
    dock_id: str


class PinkyFsm:
    def __init__(self, params: FsmParams):
        self.p = params
        self._active: _Move | _Dock | None = None
        self._rest = IDLE
        self._localized = False
        self._pose: Sequence[float] | None = None  # 마지막으로 받은 자세
        self._seq = 0
        # A FollowPath may still be out there: one was sent since the last CancelNav. The latest
        # result does not clear it, since a replaced one may be accepted late.
        self._nav_running = False
        # 마지막으로 낸 LimitSpeed. 처음에는 모른다: 재시작 전에 건 제한을 첫 tick에 푼다
        self._speed_limit: float | None = None
        self._alive = -math.inf  # 마지막 heartbeat나 목표 수락
        self._stopped_since: float | None = None
        self._blocked = False
        self._clear_since: float | None = None
        self._blocked_points: list[Point] | None = None  # 목표가 끝난 뒤에도 지켜볼 앞 구간
        self._recovery: _Recovery | None = None
        self._attempts = 0  # 쓴 자가 복구 시도 수. 목표가 바뀌어도 남는다
        self._attempt_origin: Point | None = None  # 첫 시도를 시작한 자리
        self._bseq = 0
        self._backup_running = False  # 보낸 BackUp이 아직 끝나지 않았을 수 있다

    # -- 보고 -----------------------------------------------------------------------------

    def state(self) -> str:
        if not self._localized:
            return LOCALIZING
        if self._recovery is not None:
            return RECOVERING
        if self._blocked:
            return BLOCKED
        if isinstance(self._active, _Move):
            return MOVING
        if isinstance(self._active, _Dock):
            return DOCKING
        return self._rest

    def current_goal(self) -> str:
        if isinstance(self._active, _Move):
            return self._active.target_id
        if isinstance(self._active, _Dock):
            return self._active.dock_id
        return ""

    def distance_remaining(self, pose: Sequence[float] | None) -> float:
        g = self._active
        if not isinstance(g, _Move):
            return 0.0
        s = self._track(g, pose) if pose is not None else g.s
        return max(0.0, g.route.length - s)

    def is_running(self, goal_id: Hashable) -> bool:
        return self._active is not None and self._active.goal_id == goal_id

    @property
    def recovery_attempts_used(self) -> int:
        return self._attempts

    # -- 입력 -----------------------------------------------------------------------------

    def heartbeat(self, now: float) -> None:
        self._alive = max(self._alive, now)

    def move(self, goal_id: Hashable, now: float, target_id: str, target: Pose,
             path: Sequence[Point], pose: Sequence[float] | None,
             final_goal: bool = True) -> list:
        self._alive = max(self._alive, now)  # 목표 수락도 관제가 살아 있다는 증거(계약 3절 5번)
        if pose is None:
            return self._end_active(CANCELED, stop=True) + [Finish(goal_id, False, NAV_ABORTED)]
        here = (float(pose[0]), float(pose[1]))
        tx, ty, tyaw = (float(v) for v in target)
        if path:
            points = [(float(p[0]), float(p[1])) for p in path]
            # 받은 path로 판정한다. target까지 붙이는 선이 로봇을 지나도 연결된 것이 아니다
            _, off = Polyline(points).project(here)
            if off > self.p.connect_tolerance_m:
                # 안내 2.2: 따라가지 않고 실패. 이전 목표도 끝나므로 선다
                return (self._end_active(CANCELED, stop=True)
                        + [Finish(goal_id, False, PATH_NOT_CONNECTED)])
            if math.dist(points[-1], (tx, ty)) > 1e-6:
                points.append((tx, ty))
        else:
            points = [here, (tx, ty)]  # 점 목표: 지금 자리에서 곧게
        cmds = self._end_active(CANCELED, stop=False)  # 안내 2.1: 서지 않고 새 경로로
        g = _Move(goal_id, target_id, (tx, ty, tyaw), Polyline(points), final_goal=final_goal)
        g.s, _ = g.route.project(here)  # 가장 가까운 점부터(계약 2절)
        self._active = g
        self._stopped_since = None
        return cmds + self._send_rest(g, now)

    def dock(self, goal_id: Hashable, now: float, dock_id: str) -> list:
        """pinky_dock이 아직 없다. 도킹을 했다고 꾸미지 않고 실패로 끝낸다."""
        self._alive = max(self._alive, now)
        cmds = self._end_active(CANCELED, stop=True)
        return cmds + [Finish(goal_id, False, DOCK_UNAVAILABLE)]

    def cancel(self, goal_id: Hashable, now: float) -> list:
        if not self.is_running(goal_id):
            return []
        return self._end_active(CANCELED, stop=True)

    def nav_result(self, seq: int, outcome: str, now: float, pose: Sequence[float] | None,
                   obstacle: Obstacle) -> list:
        """FollowPath 결과. outcome은 succeeded, aborted, canceled, rejected 중 하나."""
        g = self._active
        if not isinstance(g, _Move) or seq != g.seq:
            return []  # 교체된 옛 목표의 결과(Nav2는 새 목표를 받으면 옛 것을 ABORTED로 끝낸다)
        g.nav_running = False
        if self._recovery is not None and self._recovery.backup_seq is not None:
            return []  # 물러나려고 취소한 FollowPath의 결과
        # Every end below stops Nav2: a replacement does not cancel the previous FollowPath
        # (guide 2.1), so an older one may still run or be accepted late.
        if outcome == "rejected":
            return self._end_active(NAV_ABORTED, stop=True)
        if outcome == "succeeded":
            if pose is not None and math.dist(pose[:2], g.target[:2]) <= self.p.arrival_tolerance_m:
                self._active = None
                self._rest = IDLE
                self._attempts, self._attempt_origin = 0, None
                return [Finish(g.goal_id, True, "")] + self._end_active("", stop=True)
            g.far_successes += 1
            if g.far_successes > self.p.far_success_limit:
                return self._end_active(NAV_ABORTED, stop=True)
            return self._send_rest(g, now, pose)
        if outcome == "aborted" and not self._blocked and self._can_recover(pose):
            return self._start_attempt(g, now, pose)  # RPP가 스스로 선 것도 막힌 것이다
        self._recovery = None  # 시도를 다 썼다. 아래는 자가 복구가 없던 때의 동작이다
        if pose is not None and obstacle(self._ahead(g, pose)):
            g.waiting_since = now  # 계약 4절: wait while blocked, up to block_timeout_s
            if g.blocked_since is None:
                g.blocked_since = now
            return []
        return self._end_active(NAV_ABORTED, stop=True)

    def backup_result(self, seq: int, outcome: str, now: float,
                      pose: Sequence[float] | None) -> list:
        """BackUp 결과. 어떤 결과든(뒤에 물체, 시간 초과 포함) 지도를 비우고 남은 선을
        다시 따른다."""
        r, g = self._recovery, self._active
        if r is None or r.backup_seq != seq or not isinstance(g, _Move):
            return []  # 멈춘 BackUp이나 끝난 목표의 결과
        self._backup_running = False
        return self._follow_again(g, r, now, pose)

    def tick(self, now: float, pose: Sequence[float] | None, speed: tuple[float, float],
             obstacle: Obstacle) -> list:
        """주기 판단. speed = (선속도 m/s, 각속도 rad/s)."""
        self._localized = pose is not None
        if pose is not None:
            self._pose = pose
        cmds: list = []
        g = self._active
        if (g is not None and self.p.heartbeat_timeout_s > 0
                and now - self._alive >= self.p.heartbeat_timeout_s):
            cmds += self._end_active(HEARTBEAT_LOST, stop=True)
            g = None
        if g is not None and pose is None:
            cmds += self._end_active(NAV_ABORTED, stop=True)  # never drive without a pose
            g = None
        if isinstance(g, _Move) and pose is not None:
            g.s = self._track(g, pose)
        cmds += self._watch_recovery(now, g, pose, obstacle)
        g = self._active
        cmds += self._watch_obstacle(now, g, pose, speed, obstacle)  # a clear comes first
        g = self._active
        if (isinstance(g, _Move) and g.blocked_since is not None
                and now - g.blocked_since >= self.p.block_timeout_s):
            cmds += self._end_active(TIMEOUT, stop=True)  # cannot go on; stop Nav2 if it runs
            g = None
        if (isinstance(g, _Move) and g.nav_running and pose is not None
                and g.route.length - g.s < self.p.refresh_within_m
                and now - g.last_send >= self.p.refresh_every_s):
            cmds += self._send_rest(g, now, pose)  # R13: 끝 근처 재전송
        return cmds + self._limit_speed()

    # -- 안쪽 -----------------------------------------------------------------------------

    def _watch_obstacle(self, now, g, pose, speed, obstacle: Obstacle) -> list:
        """계약 4절. 움직이는 목표 중 1초 서 있고 앞 0.5 m에 물체가 있으면 자가 복구를 하고,
        시도를 다 썼으면 BLOCKED."""
        if self._recovery is not None and self._recovery.backup_seq is not None:
            self._stopped_since = None  # 물러나는 중이다. 끝나고 다시 잰다
            return []
        lin, ang = speed
        stopped = abs(lin) < self.p.stopped_speed_mps and abs(ang) < self.p.stopped_turn_rps
        moving_goal = isinstance(g, _Move)
        if moving_goal and stopped:
            if self._stopped_since is None:
                self._stopped_since = now
        else:
            self._stopped_since = None
        if pose is None:
            return []
        if moving_goal:
            points = self._ahead(g, pose)
        elif self._blocked and self._blocked_points:
            points = self._blocked_points  # 목표가 끝난 뒤에도 그 경로 앞을 본다
        else:
            self._blocked, self._clear_since = False, None
            return []
        waiting = moving_goal and g.waiting_since is not None
        stood = self._stopped_since is not None and now - self._stopped_since >= \
            self.p.block_report_s
        if not (self._blocked or waiting or stood):
            return []
        if obstacle(points):
            self._clear_since = None
            if stood and moving_goal and not self._blocked and self._can_recover(pose):
                return self._start_attempt(g, now, pose)
            if stood:
                self._recovery = None  # 두 번 다 썼다. 계약 4절의 BLOCKED로 넘긴다
                self._blocked = True
            if moving_goal and self._blocked and g.blocked_since is None:
                g.blocked_since = now  # Nav2 may hold the goal open while the robot stands
            return []
        if self._clear_since is None:
            self._clear_since = now
        if now - self._clear_since < self.p.block_clear_s:
            return []
        self._blocked, self._clear_since, self._blocked_points = False, None, None
        if moving_goal:
            g.blocked_since = None
        if waiting:
            g.waiting_since = None
            return self._send_rest(g, now, pose)  # 물체가 비켰다. 남은 선을 다시 보낸다
        return []

    # -- 자가 복구 -------------------------------------------------------------------------

    def _can_recover(self, pose) -> bool:
        return pose is not None and self._attempts < self.p.recovery_attempts

    def _start_attempt(self, g: _Move, now: float, pose: Sequence[float]) -> list:
        """FollowPath를 멈추고(BackUp과 둘 다 cmd_vel을 낸다) 물러난다. 물러날 자리가 없으면
        바로 지도를 비우고 다시 보낸다."""
        self._attempts += 1
        if self._attempt_origin is None:
            self._attempt_origin = (float(pose[0]), float(pose[1]))
        g.s = self._track(g, pose)
        r = self._recovery = _Recovery(stuck_s=g.s)
        self._blocked, self._clear_since = False, None
        g.waiting_since = g.blocked_since = None
        cmds: list = []
        if self._nav_running:
            self._nav_running = g.nav_running = False
            cmds.append(CancelNav())
        d = self._backup_distance(g, pose)
        if d < self.p.backup_min_m:
            return cmds + self._follow_again(g, r, now, pose)
        self._bseq += 1
        r.backup_seq, r.backup_since = self._bseq, now
        self._backup_running = True
        return cmds + [BackUp(self._bseq, d, self.p.backup_speed_mps,
                              self.p.backup_time_allowance_s)]

    def _backup_distance(self, g: _Move, pose: Sequence[float]) -> float:
        """마지막으로 지난 경로 꼭짓점(경로 시작 포함)에서 온 거리로 정하고(backup_distance),
        레인 옆 거리가 커지지 않게 줄인다."""
        d = backup_distance(g.s - g.route.vertex_before(g.s), self.p.backup_max_m,
                            self.p.release_distance_m + self.p.backup_margin_m)
        qx, qy = g.route.point_at(g.s)
        phi = g.route.heading_at(g.s)
        lateral = -(pose[0] - qx) * math.sin(phi) + (pose[1] - qy) * math.cos(phi)
        err = math.atan2(math.sin(pose[2] - phi), math.cos(pose[2] - phi))
        return keep_lateral(d, lateral, err, self.p.backup_lateral_limit_m)

    def _follow_again(self, g: _Move, r: _Recovery, now: float, pose) -> list:
        r.backup_seq = None
        r.follow_since = now
        self._stopped_since = None
        return [ClearCostmap()] + self._send_rest(g, now, pose)

    def _watch_recovery(self, now: float, g, pose, obstacle: Obstacle) -> list:
        """복구가 끝났는지, 시간이 다 됐는지 본다. 복구 밖에서는 시도 수를 지울지 본다."""
        r = self._recovery
        if r is None:
            if (self._attempt_origin is not None and pose is not None
                    and math.dist(pose[:2], self._attempt_origin) >= self.p.recovery_reset_m):
                self._attempts, self._attempt_origin = 0, None
            return []
        if not isinstance(g, _Move) or pose is None:
            return []
        if r.backup_seq is not None:
            limit = self.p.backup_time_allowance_s + self.p.backup_result_grace_s
            if now - r.backup_since < limit:
                return []
            self._backup_running = False
            return [CancelBackUp()] + self._follow_again(g, r, now, pose)
        if g.s >= r.stuck_s + self.p.recovery_pass_m:
            self._recovery = None  # 막힌 곳을 지났다. 시도 수는 recovery_reset_m 뒤에 지운다
            return []
        if now - r.follow_since < self.p.recovery_timeout_s:
            return []
        if self._can_recover(pose):
            return self._start_attempt(g, now, pose)
        self._recovery = None
        if obstacle(self._ahead(g, pose)):
            self._blocked = True
            g.blocked_since = now
            return []
        return self._end_active(NAV_ABORTED, stop=True)

    def _track(self, g: _Move, pose: Sequence[float]) -> float:
        s, _ = g.route.project(pose[:2], s_min=g.s - self.p.track_back_m,
                               s_max=g.s + self.p.track_ahead_m)
        return s

    def _ahead(self, g: _Move, pose: Sequence[float]) -> list[Point]:
        s = self._track(g, pose)
        return g.route.sample(s, s + self.p.block_lookahead_m, self.p.block_sample_m)

    def _limit_speed(self) -> list:
        """잠금 경계에서 끝나는 목표만 설 수 있는 속도로 제한한다. 바뀔 때만 낸다."""
        g = self._active
        limit = 0.0
        if isinstance(g, _Move) and not g.final_goal:
            left = max(0.0, g.route.length - g.s)
            v = max(self.p.horizon_min_mps, math.sqrt(2.0 * self.p.horizon_decel_mps2 * left))
            # 내려서 자른다. 올려 반올림하면 √(2·a·d)를 넘는다
            limit = math.floor(v * 100.0) / 100.0 if v < self.p.horizon_cap_mps else 0.0
        if limit == self._speed_limit:
            return []
        self._speed_limit = limit
        return [LimitSpeed(limit)]

    def _send_rest(self, g: _Move, now: float, pose: Sequence[float] | None = None) -> list:
        if pose is not None:
            g.s = self._track(g, pose)
        self._seq += 1
        g.seq = self._seq
        g.nav_running = True
        g.last_send = now
        self._nav_running = True
        poses = poses_along(g.route.rest(g.s), g.target[2], self.p.path_step_m)
        return [SendPath(g.seq, poses, final_goal=g.final_goal)]

    def _end_active(self, reason: str, stop: bool) -> list:
        g = self._active
        cmds: list = []
        if g is not None:
            if isinstance(g, _Move) and self._blocked and self._pose is not None:
                # 계약 4절: 목표가 끝나도 물체가 그 경로 앞에 있는 동안은 BLOCKED다
                self._blocked_points = self._ahead(g, self._pose)
            self._active = None
            self._rest = IDLE
            cmds.append(Finish(g.goal_id, False, reason))
        if stop and self._nav_running:
            self._nav_running = False
            cmds.append(CancelNav())
        self._recovery = None
        if self._backup_running:  # 새 목표로 바뀔 때도 멈춘다: 둘 다 cmd_vel을 낸다
            self._backup_running = False
            cmds.append(CancelBackUp())
        return cmds
