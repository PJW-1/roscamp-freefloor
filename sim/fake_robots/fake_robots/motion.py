"""직선과 선분 이동, 자세 잡음, 로봇별 고장 일정. ROS를 쓰지 않는다."""
from __future__ import annotations

import math
import random
from collections import deque


def finish_action(handle, success: bool) -> None:
    """액션 최종 상태를 알린다. 이미 성공한 목표는 성공으로 둔다."""
    if success:
        handle.succeed()
    elif handle.is_cancel_requested:
        handle.canceled()
    else:
        handle.abort()


def abort_at_shutdown(handle) -> None:
    """노드가 더는 처리할 수 없는 목표를 끝낸다. 결과를 읽을 쪽이 없다.

    SIGINT는 작업 스레드가 알아채기 전에 컨텍스트를 닫는다. abort()는 상태를 바꾼 뒤
    상태 발행에서 예외를 내는데, 여기서는 실패가 아니다.
    """
    try:
        handle.abort()
    except Exception:  # noqa: BLE001 - 예외 종류는 종료가 어디까지 진행됐는지에 달렸다
        pass


ON_SEGMENT_M = 0.05  # 이만큼 안이면 그 선분 위로 본다


class Motion:
    """목표 하나를 향해 직선으로 가는 점. 거칠 점(via)이 있으면 선분을 차례로 간다."""

    def __init__(self, x: float, y: float, yaw: float = 0.0):
        self.x, self.y, self.yaw = x, y, yaw
        self.target: tuple[float, float] | None = None
        self.via: list[tuple[float, float]] = []
        self.trail: list[tuple[float, float]] = [(x, y)]  # 이번 목표에서 지나온 점(시작, 모서리)
        self._has_start = False

    def set_target(self, x: float, y: float, via=(), start=None) -> None:
        """start: path 첫 점(path 모드). 없으면 로봇 자리가 첫 점이다. 둘 다 지나온 꼭짓점이다."""
        self.target = (x, y)
        self.via = [tuple(p) for p in via]
        self._has_start = start is not None
        self.trail = [tuple(start) if self._has_start else (self.x, self.y)]
        self._skip_passed()

    def _skip_passed(self) -> None:
        """이미 지나 다음 선분 위에 있으면 그 점을 뺀다. FollowPath처럼 가장 가까운 점부터 간다."""
        while self.via:
            a = self.via[0]
            b = self.via[1] if len(self.via) > 1 else self.target
            length = math.dist(a, b)
            if length == 0.0:
                self.via.pop(0)
                continue
            ux, uy = (b[0] - a[0]) / length, (b[1] - a[1]) / length
            s = (self.x - a[0]) * ux + (self.y - a[1]) * uy
            off = abs((self.x - a[0]) * uy - (self.y - a[1]) * ux)
            if not (1e-6 < s <= length and off <= ON_SEGMENT_M):
                return
            passed = self.via.pop(0)
            # 이미 지난 꼭짓점. 첫 점을 모르면 이 점부터 잰다
            self.trail = [*self.trail, passed] if self._has_start else [passed]

    def clear_target(self) -> None:
        self.target = None
        self.via = []
        self.trail = [(self.x, self.y)]

    @property
    def travelled(self) -> float:
        """이번 목표에서 path를 따라 지나온 길이. 후진하면 준다."""
        points = [*self.trail, (self.x, self.y)]
        return sum(math.dist(a, b) for a, b in zip(points, points[1:], strict=False))

    @property
    def since_vertex(self) -> float:
        """마지막으로 지난 path 꼭짓점(첫 점 포함)부터 온 거리."""
        return math.dist(self.trail[-1], (self.x, self.y))

    def way_back(self, distance: float) -> list[tuple[float, float]]:
        """지금 자리에서 지나온 선을 따라 distance만큼 뒤로 간 꺾은선(목표 시작점까지)."""
        out = [(self.x, self.y)]
        x, y = self.x, self.y
        for corner in reversed(self.trail):
            gap = math.dist((x, y), corner)
            if distance <= gap:
                if gap > 0.0:
                    out.append((x + (corner[0] - x) / gap * distance,
                                y + (corner[1] - y) / gap * distance))
                return out
            out.append(corner)
            distance -= gap
            x, y = corner
        return out

    def back(self, distance: float) -> None:
        """지나온 선을 따라 distance만큼 후진한다. 방향은 그대로, 목표 시작점에서 멈춘다.

        되짚은 모서리는 다시 거칠 점이 되어 앞으로 갈 때 같은 선을 탄다.
        """
        while distance > 0.0:
            last = self.trail[-1]
            gap = math.dist((self.x, self.y), last)
            if distance < gap:
                self.x += (last[0] - self.x) / gap * distance
                self.y += (last[1] - self.y) / gap * distance
                return
            self.x, self.y = last
            distance -= gap
            if len(self.trail) == 1:
                return
            self.via.insert(0, self.trail.pop())

    @property
    def moving(self) -> bool:
        return self.target is not None and self.remaining() > 0.0

    def remaining(self) -> float:
        """남은 선의 길이."""
        if self.target is None:
            return 0.0
        points = [(self.x, self.y), *self.via, self.target]
        return sum(math.dist(a, b) for a, b in zip(points, points[1:], strict=False))

    def step(self, dt: float, speed: float, limit: float = math.inf) -> None:
        """dt 동안 speed로 간다. limit보다 멀리는 가지 않는다(물체 앞에 선 자리까지)."""
        if not self.moving:
            return
        travel = min(speed * dt, limit)
        while travel > 0.0:
            aim = self.via[0] if self.via else self.target
            dist = math.dist((self.x, self.y), aim)
            if dist > 0.0:
                self.yaw = math.atan2(aim[1] - self.y, aim[0] - self.x)
            if travel < dist:
                self.x += (aim[0] - self.x) / dist * travel
                self.y += (aim[1] - self.y) / dist * travel
                return
            self.x, self.y = aim
            travel -= dist
            if not self.via:
                return
            self.trail.append(self.via.pop(0))


def noisy_pose(x: float, y: float, sigma: float, rng: random.Random) -> tuple[float, float]:
    if sigma <= 0.0:
        return x, y
    return x + rng.gauss(0.0, sigma), y + rng.gauss(0.0, sigma)


def path_gap(x: float, y: float, points) -> float:
    """(x, y)에서 꺾은선 points까지의 가장 짧은 거리. 점이 하나면 그 점까지(지시서 7.5)."""
    if len(points) == 1:
        return math.dist((x, y), points[0])
    return min(_segment_gap(x, y, a, b) for a, b in zip(points, points[1:], strict=False))


def _segment_gap(x: float, y: float, a, b) -> float:
    vx, vy = b[0] - a[0], b[1] - a[1]
    length2 = vx * vx + vy * vy
    t = max(0.0, min(1.0, ((x - a[0]) * vx + (y - a[1]) * vy) / length2)) if length2 else 0.0
    return math.hypot(x - (a[0] + t * vx), y - (a[1] + t * vy))


def drifted(x: float, y: float, yaw: float, offset: float) -> tuple[float, float]:
    """진행 방향 왼쪽으로 offset만큼 민 위치. 레인 옆으로 밀린 보고를 흉내 낸다."""
    if offset == 0.0:
        return x, y
    return x - math.sin(yaw) * offset, y + math.cos(yaw) * offset


class DelayLine:
    """넣은 것이 `delay` 뒤에 순서대로 나온다. 지연이 없으면 바로 나온다."""

    def __init__(self, delay: float):
        self._delay = delay
        self._waiting: deque = deque()

    def push(self, t: float, item) -> None:
        self._waiting.append((t + self._delay, item))

    def due(self, t: float) -> list:
        out = []
        while self._waiting and self._waiting[0][0] <= t:
            out.append(self._waiting.popleft()[1])
        return out

    def clear(self) -> None:
        self._waiting.clear()


class FaultSchedule:
    """한 로봇의 고장 여부를 시나리오 시각 기준으로 답한다."""

    def __init__(self, faults, robot: str):
        self._own = [f for f in faults if f.robot == robot]
        self._abort_left = {id(f): f.count for f in self._own if f.type == "nav_abort"}
        self.last_taken = None

    def comms_down(self, t: float) -> bool:
        return any(f.type == "comm_loss" and f.at_s <= t < f.at_s + f.duration_s
                   for f in self._own)

    def obstacle(self, t: float) -> bool:
        """경로 앞에 물체가 있다(blocked 고장). 보고와 통신은 정상이다.

        물체는 그 자리에 있으므로 로봇을 치우면 그 뒤로는 따라오지 않는다.
        """
        return any(f.type == "blocked" and f.at_s <= t < f.at_s + f.duration_s
                   and not any(g.type == "operator_remove" and f.at_s < g.at_s <= t
                               for g in self._own)
                   for f in self._own)

    def behind(self, t: float) -> float | None:
        """뒤에 물체가 있으면 로봇에서 그 물체까지 path를 따라 잰 거리(가장 가까운 것).

        후진할 때만 본다(자기 복구). 치운 로봇은 따라오지 않는다.
        """
        near = [f.distance_m for f in self._own
                if f.type == "obstacle_behind" and f.at_s <= t < f.at_s + f.duration_s
                and not any(g.type == "operator_remove" and f.at_s < g.at_s <= t
                            for g in self._own)]
        return min(near) if near else None

    def stuck(self, t: float) -> bool:
        """그 뒤 다시 투입되기 전까지 멈춘다. 운영자가 고쳐 넣은 것으로 본다."""
        return any(f.type == "stuck" and t >= f.at_s
                   and not any(g.type == "operator_reinstate" and f.at_s < g.at_s <= t
                               for g in self._own)
                   for f in self._own)

    def removed(self, t: float) -> bool:
        """치웠고 그 뒤 다시 투입하지 않았다."""
        last = self._last(t)
        return last is not None and last.type == "operator_remove"

    def put_back(self, t: float):
        """지금 다시 투입된 상태면 놓인 지점, 아니면 None."""
        last = self._last(t)
        return last.start if last is not None and last.type == "operator_reinstate" else None

    def _last(self, t: float):
        """t까지 일어난 마지막 치우기나 다시 투입. 같은 시각이면 파일 뒤쪽."""
        done = [f for f in self._own
                if f.type in ("operator_remove", "operator_reinstate") and f.at_s <= t]
        return sorted(done, key=lambda f: f.at_s)[-1] if done else None

    def drift(self, t: float) -> float:
        """이 시각 보고를 옆으로 미는 거리. 실제 위치는 그대로다."""
        return sum(f.offset_m for f in self._own
                   if f.type == "drift" and f.at_s <= t < f.at_s + f.duration_s)

    def take_nav_abort(self, t: float, target_id: str = "") -> bool:
        """새 이동 목표를 중단해야 하면 True. 고장 횟수를 하나 쓴다. target이면 그 지점만.

        쓴 일정은 last_taken에 남는다(돌려줄 때 그 일정에).
        """
        self.last_taken = None
        for f in self._own:
            if (f.type == "nav_abort" and t >= f.at_s and self._abort_left[id(f)] > 0
                    and f.target in ("", target_id)):
                self._abort_left[id(f)] -= 1
                self.last_taken = f
                return True
        return False

    def give_back_nav_abort(self, taken) -> None:
        """중단이 나기 전에 끝난 목표의 고장 횟수를 그 목표가 쓴 일정에 돌려준다."""
        if taken is not None and self._abort_left[id(taken)] < taken.count:
            self._abort_left[id(taken)] += 1


class ArmFaults:
    """팔의 고장 여부를 시나리오 시각 기준으로 답한다."""

    def __init__(self, faults):
        self._own = [f for f in faults if f.type in ("arm_fail", "swap_partial")]
        self._left = {id(f): f.count for f in self._own}

    def take_arm_fail(self, arm: str, t: float, units: int) -> int | None:
        """`units`개짜리 목표에서 이 팔이 실패하는 물건 순번, 없으면 None.

        이 목표에서 그 순번까지 갈 때만 고장 횟수를 하나 쓴다.
        """
        for f in self._own:
            if (f.type == "arm_fail" and f.arm == arm and t >= f.at_s
                    and self._left[id(f)] > 0 and f.after_items < units):
                self._left[id(f)] -= 1
                return f.after_items
        return None

    def take_swap_partial(self, t: float) -> bool:
        """빈 바구니 놓기를 실패시켜야 하면 True. 고장 횟수를 하나 쓴다."""
        for f in self._own:
            if f.type == "swap_partial" and t >= f.at_s and self._left[id(f)] > 0:
                self._left[id(f)] -= 1
                return True
        return False
