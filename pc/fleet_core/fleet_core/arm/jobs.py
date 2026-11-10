"""Sends the goals of arm work (fill, swap) and sees them through. Knows no order, basket or lane.

The orders ask once per attempt with ArmFillCmd or ArmSwapCmd; one goal goes out and its result
comes back. Goal ids, time-outs and cancels, the wait after a failure, the attempt count and its
limit live here. Whether an attempt failed is for the orders to say (ArmFailedCmd): they know the
basket.
"""
from __future__ import annotations

from dataclasses import dataclass

from fleet_core.common.commands import (
    FILL,
    SWAP,
    ArmDeclined,
    ArmDropCmd,
    ArmFailedCmd,
    ArmFillCmd,
    ArmFillResult,
    ArmLate,
    ArmLost,
    ArmResetCmd,
    ArmStopCmd,
    ArmSwapCmd,
    ArmSwapResult,
    CancelCmd,
    EventOut,
    FillCmd,
    RobotFaultCmd,
    SwapCmd,
)

STOP = "STOP"
CANCELED = "CANCELED"

_CODES = {FILL: ("FILL_RETRY", "FILL_FAILED"), SWAP: ("SWAP_RETRY", "SWAP_FAILED")}


@dataclass
class _Job:
    server: str
    goal_id: int
    kind: str  # FILL or SWAP
    sent_at: float
    canceled_at: float | None = None
    timed_out: bool = False  # canceled by the time-out, not by STOP


@dataclass
class _Robot:
    name: str
    job: _Job | None = None
    retry_at: float = 0.0  # no new goal before this time, after a failure
    attempts: int = 0
    last: tuple[_Job, str] | None = None  # the goal just answered and its reason, for ArmFailedCmd


class ArmJobs:
    def __init__(self, stations, params, robots, clock, next_goal_id=None):
        self.params = params
        self._stations = stations
        self._clock = clock
        self._next_goal_id = next_goal_id or self._own_goal_id
        self._goal_seq = 0
        self._robots = {name: _Robot(name) for name in robots}
        self._out: list = []
        self._run_mode: str | None = None
        self._cleared_jobs: dict[str, tuple[int, float]] = {}  # server -> dropped goal, until when

    # ---- inputs from outside ------------------------------------------------------------

    def on_run_mode(self, run_mode) -> None:
        if run_mode == STOP and self._run_mode != STOP:
            for r in self._robots.values():
                if r.job and r.job.canceled_at is None:
                    self._cancel(r)
        self._run_mode = run_mode

    def on_fill_result(self, goal_id, success, placed, missing, reason="") -> None:
        self._forget_cleared_job(goal_id)
        r = self._job_owner(goal_id, FILL)
        if r is None:
            self._out.append(ArmLate(goal_id, FILL, placed=placed))
            return
        r.last, r.job = (r.job, reason), None
        self._out.append(ArmFillResult(r.name, success, placed))

    def on_swap_result(self, goal_id, success, took_full, put_empty_done, reason="") -> None:
        self._forget_cleared_job(goal_id)
        r = self._job_owner(goal_id, SWAP)
        if r is None:
            self._out.append(ArmLate(goal_id, SWAP, took_full=took_full,
                                     put_empty_done=put_empty_done))
            return
        r.last, r.job = (r.job, reason), None
        self._out.append(ArmSwapResult(r.name, took_full, put_empty_done))

    def check_timeouts(self, now: float) -> None:
        for r in self._robots.values():
            job = r.job
            if not job:
                continue
            limit = (self.params.fill_timeout_s if job.kind == FILL
                     else self.params.swap_timeout_s)
            if job.canceled_at is None and now - job.sent_at > limit:
                self._cancel(r)
                job.timed_out = True
            elif job.canceled_at is not None and (
                    now - job.canceled_at >= self.params.arm_cancel_wait_s):
                # Nobody knows what is in the basket now: the operator has to look.
                r.job = None
                self._out.append(ArmLost(r.name, job.goal_id, job.server, job.kind))
                reason = f"{job.server} goal {job.goal_id} never reported"
                self._out.append(RobotFaultCmd(r.name, reason))
                self._event("ERROR", "ARM_NO_RESULT", reason, r.name)

    # ---- inputs from the orders ---------------------------------------------------------

    def on_command(self, command) -> None:
        r = self._robots.get(command.robot)
        if r is None:
            return
        if isinstance(command, ArmFillCmd):
            self._start(r, FILL, command)
        elif isinstance(command, ArmSwapCmd):
            self._start(r, SWAP, command)
        elif isinstance(command, ArmStopCmd):
            if r.job and r.job.canceled_at is None:
                self._cancel(r)
        elif isinstance(command, ArmDropCmd):
            self._drop(r, command)
        elif isinstance(command, ArmFailedCmd):
            self._attempt_failed(r)
        elif isinstance(command, ArmResetCmd):
            if command.forget:
                self._robots[r.name] = _Robot(r.name)
            else:
                r.attempts = 0

    # ---- outputs ------------------------------------------------------------------------

    def take_output(self) -> list:
        out, self._out = self._out, []
        return out

    def sent_at(self, robot) -> float | None:
        """When the running goal was sent, or None."""
        r = self._robots.get(robot)
        return r.job.sent_at if r and r.job else None

    def attempts(self, robot) -> int:
        r = self._robots.get(robot)
        return r.attempts if r else 0

    # ---- helpers ------------------------------------------------------------------------

    def _own_goal_id(self) -> int:
        self._goal_seq += 1
        return self._goal_seq

    def _event(self, level, code, detail, robot="") -> None:
        self._out.append(EventOut(level, code, detail, robot))

    def _start(self, r: _Robot, kind: str, command) -> None:
        server = self._stations[command.station_id].server
        if r.job or not self._goal_allowed(r, server):
            self._out.append(ArmDeclined(r.name))
            return
        goal_id = self._next_goal_id()
        r.job = _Job(server, goal_id, kind, self._clock())
        if kind == FILL:
            self._out.append(FillCmd(goal_id, r.name, command.station_id, command.order_id,
                                     command.items))
        else:
            self._out.append(SwapCmd(server, goal_id, r.name, command.station_id,
                                     command.order_id, take_full=command.take_full,
                                     put_empty=command.put_empty))

    def _goal_allowed(self, r: _Robot, server: str) -> bool:
        """No new goal right after a failure, nor while the server still stops a dropped job."""
        now = self._clock()
        cleared = self._cleared_jobs.get(server)
        if cleared and now >= cleared[1]:
            del self._cleared_jobs[server]
            cleared = None
        return not cleared and now >= r.retry_at

    def _job_owner(self, goal_id: int, kind: str) -> _Robot | None:
        for r in self._robots.values():
            if r.job and r.job.goal_id == goal_id and r.job.kind == kind:
                return r
        return None

    def _forget_cleared_job(self, goal_id: int) -> None:
        for server, (cleared_goal, _) in list(self._cleared_jobs.items()):
            if cleared_goal == goal_id:
                del self._cleared_jobs[server]

    def _cancel(self, r: _Robot) -> None:
        r.job.canceled_at = self._clock()
        self._out.append(CancelCmd(r.job.server, r.job.goal_id))

    def _drop(self, r: _Robot, command: ArmDropCmd) -> None:
        """The server may still be stopping: what it does then is not the next robot's job."""
        job = r.job
        if job is None:
            return
        self._out.append(CancelCmd(job.server, job.goal_id))
        self._cleared_jobs[job.server] = (
            job.goal_id, self._clock() + self.params.arm_cancel_wait_s)
        self._out.append(ArmLost(r.name, job.goal_id, job.server, job.kind, cleared=True,
                                 isolated=command.isolated, order_id=command.order_id,
                                 bound=command.bound))
        r.job = None

    def _attempt_failed(self, r: _Robot) -> None:
        if r.last is None:
            return
        (job, reason), r.last = r.last, None
        if reason == CANCELED and job.canceled_at is not None and not job.timed_out:
            return  # canceled by STOP, which is no failure
        retry_code, failed_code = _CODES[job.kind]
        r.attempts += 1
        r.retry_at = self._clock() + self.params.arm_retry_wait_s
        if r.attempts > self.params.move_retry_limit:
            self._out.append(RobotFaultCmd(r.name, f"{job.server}: {reason}"))
            self._event("ERROR", failed_code, f"{job.server}: {reason}", r.name)
        else:
            self._event("WARN", retry_code,
                        f"{job.server} attempt {r.attempts} failed: {reason}", r.name)
