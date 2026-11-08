"""앞 물체 앞에서 서고, 짧게 후진해 다시 가 보고, 그래도 막히면 BLOCKED. ROS를 쓰지 않는다.

자기 복구는 제안이다(2026-11-09 공유 명세 '로봇 쪽'). pinky_fsm과 같은 순서로 움직인다.
  - 물체 앞에 block_report_s 서 있으면 시도를 하나 쓰고 RECOVERING이다.
  - 후진 거리 d = min(backup_max_m, s - robot_radius_m - backup_margin_m), s는 이번 목표에서
    지나온 path 길이다. d < backup_min_m이면 후진하지 않는다. 뒤에 다른 로봇이나 물체가 있으면
    그 자리에서 후진을 멈춘다. 그다음 남은 path를 다시 간다(물체가 있으면 선 자리까지).
  - 시도를 다 쓰고 또 서면 BLOCKED(계약 4절: 목표를 쥐고 물체가 사라지면 이어 간다).
  - 시도를 시작한 자리에서 recovery_pass_m 더 가면 RECOVERING이 끝난다(MOVING). 시도 수는 남는다.
  - 처음 센 시도의 자리에서 recovery_reset_m 멀어지거나 목표가 성공하면 시도 수가 0이 된다.
물체는 시각으로만 있다(blocked 고장). 위치가 없으므로 처음 선 자리(path 위 거리)를 물체 앞으로
본다. 물체가 block_clear_s 동안 안 보여야 사라진 것으로 본다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from fake_robots.motion import path_gap


@dataclass(frozen=True)
class RecoveryParams:
    block_report_s: float = 1.0  # 물체 앞에서 이만큼 서 있으면 시도, 시도를 다 썼으면 BLOCKED
    block_clear_s: float = 1.0  # 물체가 이만큼 안 보이면 사라진 것이다
    recovery_attempts: int = 2
    backup_max_m: float = 0.15
    robot_radius_m: float = 0.08
    backup_margin_m: float = 0.03
    backup_min_m: float = 0.02
    backup_speed_mps: float = 0.08
    backup_timeout_s: float = 5.0
    recovery_pass_m: float = 0.05  # 시도를 시작한 자리에서 이만큼 더 가면 RECOVERING 끝
    recovery_reset_m: float = 0.3  # 처음 센 시도의 자리에서 이만큼 멀어지면 시도 수 0

    def backup_distance(self, s: float) -> float:
        """path를 s만큼 왔을 때의 후진 거리. 0이면 후진하지 않는다."""
        d = min(self.backup_max_m, s - self.robot_radius_m - self.backup_margin_m)
        return d if d >= self.backup_min_m - 1e-9 else 0.0


@dataclass
class _BackUp:
    left: float
    until: float
    fixed: list  # 후진을 시작할 때 뒤에 있던 물체 (x, y, 반지름)


class ObstacleRecovery:
    """한 로봇의 앞 물체 처리. 노드는 매 틱 watch()를, 이동 목표가 있으면 drive()를 부른다."""

    def __init__(self, params: RecoveryParams | None = None):
        self.p = params or RecoveryParams()
        self.blocked = False
        self.used = 0
        self._hold: float | None = None  # 물체 앞에 선 자리(path 위 거리)
        self._clear_at: float | None = None
        self._stood_at: float | None = None
        self._attempt_s: float | None = None  # 지금 시도를 시작한 자리(path 위 거리)
        self._first_at: tuple[float, float] | None = None
        self._backup: _BackUp | None = None

    @property
    def recovering(self) -> bool:
        return self._attempt_s is not None

    @property
    def backing_up(self) -> bool:
        return self._backup is not None

    def watch(self, t: float, ahead: bool, driving: bool, motion) -> None:
        """물체가 보이는지. 이동 중에 나타나면 그 자리가 물체 앞이다."""
        if ahead:
            self._clear_at = None
            if self._hold is None and driving:
                self._hold = motion.travelled
            return
        if self._hold is None:
            return
        if self._clear_at is None:
            self._clear_at = t
        if t - self._clear_at >= self.p.block_clear_s:
            self._hold = self._clear_at = self._stood_at = None
            self.blocked = False

    def drive(self, t: float, dt: float, motion, speed: float, others=(),
              wall_behind_m: float | None = None) -> None:
        """이동 목표 한 틱. others: 다른 로봇 (x, y, 반지름). wall_behind_m: 뒤 물체 거리."""
        if self._backup is not None:
            self._back_step(t, dt, motion, others)
            return
        limit = math.inf if self._hold is None else max(0.0, self._hold - motion.travelled)
        motion.step(dt, speed, limit)
        s = motion.travelled
        if self._attempt_s is not None and s >= self._attempt_s + self.p.recovery_pass_m - 1e-9:
            self._attempt_s = None
        if self._hold is not None and s >= self._hold - 1e-9:
            if self._stood_at is None:
                self._stood_at = t
            if not self.blocked and t - self._stood_at >= self.p.block_report_s - 1e-9:
                self._trigger(t, motion, wall_behind_m)
        else:
            self._stood_at = None
        if (self._attempt_s is None and self._first_at is not None and math.dist(
                self._first_at, (motion.x, motion.y)) >= self.p.recovery_reset_m - 1e-9):
            self.used, self._first_at = 0, None

    def _trigger(self, t: float, motion, wall_behind_m: float | None) -> None:
        if self.used >= self.p.recovery_attempts:
            self.blocked, self._attempt_s = True, None
            return
        self.used += 1
        if self._first_at is None:
            self._first_at = (motion.x, motion.y)
        self._attempt_s = motion.travelled
        self._stood_at = t  # 후진하지 않으면 여기서 다시 센다
        d = self.p.backup_distance(motion.travelled)
        if d <= 0.0:
            return
        fixed = []
        if wall_behind_m is not None:
            x, y = motion.way_back(wall_behind_m)[-1]
            fixed.append((x, y, 0.0))
        self._backup = _BackUp(d, t + self.p.backup_timeout_s, fixed)
        self._stood_at = None

    def _back_step(self, t: float, dt: float, motion, others) -> None:
        b = self._backup
        if (t >= b.until or b.left <= 1e-9
                or self._something_behind(motion, b.left, [*others, *b.fixed])):
            self._backup = None  # 그 자리에서 멈춘다. 시도는 이어 간다
            return
        step = min(self.p.backup_speed_mps * dt, b.left)
        motion.back(step)
        b.left -= step
        if b.left <= 1e-9:
            self._backup = None

    def _something_behind(self, motion, left: float, objects) -> bool:
        """남은 후진 길에서 몸(반지름)이 물체에 닿는가. 로봇 앞쪽의 물체는 보지 않는다."""
        way = motion.way_back(left)
        hx, hy = math.cos(motion.yaw), math.sin(motion.yaw)
        for x, y, radius in objects:
            if (x - motion.x) * hx + (y - motion.y) * hy > 0.0:
                continue
            if path_gap(x, y, way) <= self.p.robot_radius_m + radius:
                return True
        return False

    def restart(self) -> None:
        """목표가 끝났거나 바뀌었다. 후진과 RECOVERING을 멈추고 시도 수는 둔다.

        물체가 아직 보이면 새 목표는 지금 자리에서 서 있는다(path 위 거리 0).
        """
        self._backup = None
        self._attempt_s = None
        self._stood_at = None
        if self._hold is not None:
            self._hold = 0.0

    def succeeded(self) -> None:
        self.used, self._first_at = 0, None

    def forget(self) -> None:
        """로봇을 코스에서 들어냈다. 물체도 시도도 잊는다."""
        self.__init__(self.p)
