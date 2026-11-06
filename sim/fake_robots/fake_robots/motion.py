"""직선 이동, 자세 잡음, 로봇별 고장 일정. ROS를 쓰지 않는다."""
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


class Motion:
    """목표 하나를 향해 직선으로 가는 점."""

    def __init__(self, x: float, y: float, yaw: float = 0.0):
        self.x, self.y, self.yaw = x, y, yaw
        self.target: tuple[float, float] | None = None

    def set_target(self, x: float, y: float) -> None:
        self.target = (x, y)

    def clear_target(self) -> None:
        self.target = None

    @property
    def moving(self) -> bool:
        return self.target is not None and self.remaining() > 0.0

    def remaining(self) -> float:
        if self.target is None:
            return 0.0
        return math.dist((self.x, self.y), self.target)

    def step(self, dt: float, speed: float) -> None:
        if not self.moving:
            return
        dist = self.remaining()
        self.yaw = math.atan2(self.target[1] - self.y, self.target[0] - self.x)
        travel = speed * dt
        if travel >= dist:
            self.x, self.y = self.target
        else:
            self.x += (self.target[0] - self.x) / dist * travel
            self.y += (self.target[1] - self.y) / dist * travel


def noisy_pose(x: float, y: float, sigma: float, rng: random.Random) -> tuple[float, float]:
    if sigma <= 0.0:
        return x, y
    return x + rng.gauss(0.0, sigma), y + rng.gauss(0.0, sigma)


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

    def take_nav_abort(self, t: float) -> bool:
        """새 이동 목표를 중단해야 하면 True. 고장 횟수를 하나 쓴다."""
        for f in self._own:
            if f.type == "nav_abort" and t >= f.at_s and self._abort_left[id(f)] > 0:
                self._abort_left[id(f)] -= 1
                return True
        return False


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
