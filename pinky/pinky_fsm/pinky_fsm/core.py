"""핑키 FSM의 판단. ROS를 모르고, 입력을 받아 명령 목록을 돌려준다.

규칙의 출처
- 안내 = Drive 핑키_FSM_Nav2_구현_안내_2026-11-07.md (2.1 교체 때 서지 않음, 2.2 연결 확인)
- 계약 = docs/핑키_계약.md (3절 지킬 것, 4절 BLOCKED)
- R번호 = Drive 핑키_제어파트_전달사항_2026-11-06 3절

시각은 모두 벽시계 초다(heartbeat는 벽시계로 재야 하고, 나머지도 같은 축을 쓴다).
노드는 SendPath를 Nav2 FollowPath 목표로, CancelNav를 그 취소로, Finish를 액션 결과로 옮긴다.
"""
from __future__ import annotations

import math
from collections.abc import Callable, Hashable, Sequence
from dataclasses import dataclass, field

from pinky_fsm.geometry import Point, Polyline, Pose, poses_along

# RobotState.STATE_* 와 같은 값
IDLE = "IDLE"
MOVING = "MOVING"
DOCKING = "DOCKING"
DOCKED = "DOCKED"
BLOCKED = "BLOCKED"
LOCALIZING = "LOCALIZING"

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
    # Proposed. After Nav2 stops for an obstacle, TIMEOUT after this long. Longer than the
    # fleet manager's blocked_timeout_s (60 s): a last safeguard for when the fleet is gone
    block_timeout_s: float = 90.0
    block_sample_m: float = 0.02  # costmap 해상도(0.025)보다 촘촘하게
    path_step_m: float = 0.05  # 전달사항 6.1 poses_along
    far_success_limit: int = 3  # 목표에서 먼 Nav2 성공을 몇 번까지 다시 보낼까
    track_back_m: float = 0.2  # 진행 거리를 찾을 때 뒤로 허용하는 거리
    track_ahead_m: float = 1.0  # 진행 거리를 찾을 때 앞으로 보는 거리


@dataclass(frozen=True)
class SendPath:
    """Nav2 FollowPath에 이 경로를 보낸다. 달리는 목표는 취소하지 않는다(안내 2.1)."""
    seq: int
    poses: list[Pose] = field(compare=False)


@dataclass(frozen=True)
class CancelNav:
    """Nav2 FollowPath 목표를 취소한다. controller가 속도 0을 내고 선다."""


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
    seq: int = 0  # 마지막으로 보낸 FollowPath 순번
    nav_running: bool = False
    last_send: float = 0.0
    far_successes: int = 0
    waiting_since: float | None = None  # when Nav2 stopped for an obstacle; resend when clear


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
        self._nav_running = False  # 끝나지 않은 FollowPath 목표가 있다
        self._alive = -math.inf  # 마지막 heartbeat나 목표 수락
        self._stopped_since: float | None = None
        self._blocked = False
        self._clear_since: float | None = None
        self._blocked_points: list[Point] | None = None  # 목표가 끝난 뒤에도 지켜볼 앞 구간

    # -- 보고 -----------------------------------------------------------------------------

    def state(self) -> str:
        if not self._localized:
            return LOCALIZING
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

    # -- 입력 -----------------------------------------------------------------------------

    def heartbeat(self, now: float) -> None:
        self._alive = max(self._alive, now)

    def move(self, goal_id: Hashable, now: float, target_id: str, target: Pose,
             path: Sequence[Point], pose: Sequence[float] | None) -> list:
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
        g = _Move(goal_id, target_id, (tx, ty, tyaw), Polyline(points))
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
        self._nav_running = False
        if outcome == "rejected":
            # A replacement does not cancel the previous FollowPath (guide 2.1), so it may still run
            return self._end_active(NAV_ABORTED, stop=False) + [CancelNav()]
        if outcome == "succeeded":
            if pose is not None and math.dist(pose[:2], g.target[:2]) <= self.p.arrival_tolerance_m:
                self._active = None
                self._rest = IDLE
                return [Finish(g.goal_id, True, "")]
            g.far_successes += 1
            if g.far_successes > self.p.far_success_limit:
                return self._end_active(NAV_ABORTED, stop=False)
            return self._send_rest(g, now, pose)
        if pose is not None and obstacle(self._ahead(g, pose)):
            g.waiting_since = now  # 계약 4절: wait while blocked, up to block_timeout_s
            return []
        return self._end_active(NAV_ABORTED, stop=False)

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
        if (isinstance(g, _Move) and g.waiting_since is not None
                and now - g.waiting_since >= self.p.block_timeout_s):
            cmds += self._end_active(TIMEOUT, stop=False)  # cannot go on: end with TIMEOUT
            g = None
        if isinstance(g, _Move) and pose is not None:
            g.s = self._track(g, pose)
        cmds += self._watch_obstacle(now, g, pose, speed, obstacle)
        g = self._active
        if (isinstance(g, _Move) and g.nav_running and pose is not None
                and g.route.length - g.s < self.p.refresh_within_m
                and now - g.last_send >= self.p.refresh_every_s):
            cmds += self._send_rest(g, now, pose)  # R13: 끝 근처 재전송
        return cmds

    # -- 안쪽 -----------------------------------------------------------------------------

    def _watch_obstacle(self, now, g, pose, speed, obstacle: Obstacle) -> list:
        """계약 4절. 움직이는 목표 중 1초 서 있고 앞 0.5 m에 물체가 있으면 BLOCKED."""
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
            if stood:
                self._blocked = True
            return []
        if self._clear_since is None:
            self._clear_since = now
        if now - self._clear_since < self.p.block_clear_s:
            return []
        self._blocked, self._clear_since, self._blocked_points = False, None, None
        if waiting:
            g.waiting_since = None
            return self._send_rest(g, now, pose)  # 물체가 비켰다. 남은 선을 다시 보낸다
        return []

    def _track(self, g: _Move, pose: Sequence[float]) -> float:
        s, _ = g.route.project(pose[:2], s_min=g.s - self.p.track_back_m,
                               s_max=g.s + self.p.track_ahead_m)
        return s

    def _ahead(self, g: _Move, pose: Sequence[float]) -> list[Point]:
        s = self._track(g, pose)
        return g.route.sample(s, s + self.p.block_lookahead_m, self.p.block_sample_m)

    def _send_rest(self, g: _Move, now: float, pose: Sequence[float] | None = None) -> list:
        if pose is not None:
            g.s = self._track(g, pose)
        self._seq += 1
        g.seq = self._seq
        g.nav_running = True
        g.last_send = now
        self._nav_running = True
        poses = poses_along(g.route.rest(g.s), g.target[2], self.p.path_step_m)
        return [SendPath(g.seq, poses)]

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
        return cmds
