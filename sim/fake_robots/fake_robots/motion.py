"""Straight-line motion, pose noise and the per-robot fault schedule. No ROS imports."""
from __future__ import annotations

import math
import random
from collections import deque


def finish_action(handle, success: bool) -> None:
    """Report the final action status. A goal that already succeeded stays a success."""
    if success:
        handle.succeed()
    elif handle.is_cancel_requested:
        handle.canceled()
    else:
        handle.abort()


def abort_at_shutdown(handle) -> None:
    """End a goal the node can no longer serve. Nobody is left to read the result.

    SIGINT shuts the context down before the worker thread notices. abort() still changes the
    goal state and then raises while publishing the status, which is not a failure here.
    """
    try:
        handle.abort()
    except Exception:  # noqa: BLE001 - the error type depends on how far shutdown has got
        pass


class Motion:
    """A point that drives in a straight line toward one target."""

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
    """What goes in comes out `delay` later, in order. With no delay, straight through."""

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
    """Answers fault questions for one robot at a given scenario time."""

    def __init__(self, faults, robot: str):
        self._own = [f for f in faults if f.robot == robot]
        self._abort_left = {id(f): f.count for f in self._own if f.type == "nav_abort"}

    def comms_down(self, t: float) -> bool:
        return any(f.type == "comm_loss" and f.at_s <= t < f.at_s + f.duration_s
                   for f in self._own)

    def stuck(self, t: float) -> bool:
        return any(f.type == "stuck" and t >= f.at_s for f in self._own)

    def removed(self, t: float) -> bool:
        return any(f.type == "operator_remove" and t >= f.at_s for f in self._own)

    def take_nav_abort(self, t: float) -> bool:
        """True when this new move goal must abort. Consumes one of the fault's count."""
        for f in self._own:
            if f.type == "nav_abort" and t >= f.at_s and self._abort_left[id(f)] > 0:
                self._abort_left[id(f)] -= 1
                return True
        return False


class ArmFaults:
    """Answers fault questions for the arms at a given scenario time."""

    def __init__(self, faults):
        self._own = [f for f in faults if f.type in ("arm_fail", "swap_partial")]
        self._left = {id(f): f.count for f in self._own}

    def take_arm_fail(self, arm: str, t: float, units: int) -> int | None:
        """The item at which this arm fails in a goal of `units` items, or None.

        Consumes one of the fault's count, but only when the arm gets that far in this goal.
        """
        for f in self._own:
            if (f.type == "arm_fail" and f.arm == arm and t >= f.at_s
                    and self._left[id(f)] > 0 and f.after_items < units):
                self._left[id(f)] -= 1
                return f.after_items
        return None

    def take_swap_partial(self, t: float) -> bool:
        """True when placing the empty basket must fail. Consumes one of the fault's count."""
        for f in self._own:
            if f.type == "swap_partial" and t >= f.at_s and self._left[id(f)] > 0:
                self._left[id(f)] -= 1
                return True
        return False
