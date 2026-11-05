"""OrchestratorCore: orders, baskets and arm jobs. No ROS, no wall clock, no lanes.

It asks for robots with DispatchCmd and lets go of them with ReleaseCmd, and hears back through
on_dispatch_result. Which robot comes and how it gets there is traffic's business. An order is
bound when a robot has finished docking at the fill station, so the dock arrival order is the
work order and a robot failing on its way holds nothing.

Open WORK dispatches always equal the unbound orders: every input that changes one fixes the
other before it returns.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from fleet_core.commands import (
    HELD,
    NO_ROBOT,
    PARTIAL,
    SHIPPED,
    CancelCmd,
    CancelDispatchCmd,
    DispatchCmd,
    EventOut,
    FillCmd,
    ReleaseCmd,
    ReportOrderCmd,
    RobotFaultCmd,
    SwapCmd,
)

STOP = "STOP"

# What the robot's stay at a station is for.
FILLING = "FILLING"
SWAPPING = "SWAPPING"

# Station actions, as written in the site map.
FILL = "fill"
SWAP = "swap"

# What is on the tray.
EMPTY = "EMPTY"
FULL = "FULL"
NONE = "NONE"

CANCELED = "CANCELED"


def _merge(into: dict[str, int], more: dict[str, int]) -> None:
    for kind, count in more.items():
        if count > 0:
            into[kind] = into.get(kind, 0) + count


def _subtract(items: dict[str, int], *taken: dict[str, int]) -> dict[str, int]:
    left = {}
    for kind, count in items.items():
        rest = count - sum(t.get(kind, 0) for t in taken)
        if rest > 0:
            left[kind] = rest
    return left


@dataclass
class _ArmJob:
    server: str
    goal_id: int
    kind: str  # FILL or SWAP
    sent_at: float
    canceled_at: float | None = None
    timed_out: bool = False  # canceled because it ran too long, not because of STOP


@dataclass
class _BoundOrder:
    order_id: str
    items: dict[str, int]
    placed: dict[str, int] = field(default_factory=dict)
    missing: dict[str, int] = field(default_factory=dict)  # final: nobody will try these again


@dataclass
class _Robot:
    name: str
    stage: str = ""  # FILLING or SWAPPING while the robot is kept at a station for an arm
    basket: str = EMPTY
    order: _BoundOrder | None = None
    last_order_id: str = ""  # names the swap goal after the order was already reported
    outbound_goal: int | None = None
    arm: _ArmJob | None = None
    arm_retry_at: float = 0.0  # no new arm goal before this, after a failure or a rejection
    attempts: int = 0


class OrchestratorCore:
    def __init__(self, stations, params, robots, clock, next_goal_id=None):
        self.params = params
        self.shipped = 0
        self._stations = stations
        self._clock = clock
        self._next_goal_id = next_goal_id or self._own_goal_id
        self._goal_seq = 0
        self._dispatch_seq = 0
        self._robots = {name: _Robot(name) for name in robots}
        self._out: list = []
        self._run_mode: str | None = None
        self._board: list = []
        self._reported: set[str] = set()  # the board may still list these for a moment
        self._open_calls: list[int] = []  # WORK dispatches sent and not answered yet
        self._work = next(s.id for s in stations.values() if s.action == FILL)
        self._outbound = next(s.id for s in stations.values() if s.action == SWAP)
        self._cleared_jobs: dict[str, tuple[int, float]] = {}  # server -> canceled goal, deadline

    # ---- inputs -------------------------------------------------------------------------

    def on_order_board(self, run_mode, orders) -> None:
        if run_mode == STOP and self._run_mode != STOP:
            for r in self._robots.values():
                if r.arm and r.arm.canceled_at is None:
                    self._cancel_arm(r)
        self._run_mode = run_mode
        self._board = list(orders)
        self._reported &= {o.order_id for o in self._board}
        self._sync_calls()

    def on_dispatch_feedback(self, goal_id, robot_id, phase) -> None:
        if phase == NO_ROBOT:
            ids = [o.order_id for o in self._pending()]
            self._event("WARN", "NO_ROBOT_AVAILABLE", f"{ids} wait and no robot can take them")

    def on_dispatch_result(self, goal_id, success, robot_id, reason="") -> None:
        if goal_id in self._open_calls:
            self._open_calls.remove(goal_id)
            if success:
                self._start_filling(self._robots[robot_id])
            self._sync_calls()
            return
        r = self._robots[robot_id]
        if r.outbound_goal == goal_id:
            r.outbound_goal = None
            if success:
                r.stage = SWAPPING

    def on_fill_result(self, goal_id, success, placed, missing, reason="") -> None:
        self._forget_cleared_job(goal_id)
        r = self._arm_job_owner(goal_id, FILL)
        if r is None:
            return
        job, r.arm = r.arm, None
        _merge(r.order.placed, placed)
        if success:
            # The server has finished trying: whatever is not in the basket stays out of it.
            r.order.missing = _subtract(r.order.items, r.order.placed)
            return
        self._arm_attempt_failed(r, job, reason, "FILL_RETRY", "FILL_FAILED")

    def on_swap_result(self, goal_id, success, took_full, put_empty_done, reason="") -> None:
        self._forget_cleared_job(goal_id)
        r = self._arm_job_owner(goal_id, SWAP)
        if r is None:
            return
        job, r.arm = r.arm, None
        if took_full and r.basket == FULL:
            r.basket = NONE
            self._report_shipped(r)
        if put_empty_done and r.basket == NONE:
            r.basket = EMPTY
            r.attempts = 0
            self._leave_station(r)
            return
        self._arm_attempt_failed(r, job, reason, "SWAP_RETRY", "SWAP_FAILED")

    def on_robot_removed(self, robot) -> None:
        """The operator took the robot away. Its order goes back to its registration position."""
        r = self._robots[robot]
        if r.arm:
            # What the arm did for a cleared robot is for the operator to count. The server is
            # still busy stopping, and that must not count against the next robot.
            self._out.append(CancelCmd(r.arm.server, r.arm.goal_id))
            self._cleared_jobs[r.arm.server] = (
                r.arm.goal_id, self._clock() + self.params.arm_cancel_wait_s)
            r.arm = None
        if r.order:
            self._event("INFO", "ORDER_REQUEUED",
                        f"{r.order.order_id} was bound, basket held {r.order.placed}", robot)
            r.order = None
        r.stage = ""
        r.outbound_goal = None
        self._sync_calls()

    # ---- outputs ------------------------------------------------------------------------

    def check_arm_timeouts(self, now: float) -> None:
        for r in self._robots.values():
            job = r.arm
            if not job:
                continue
            limit = (self.params.fill_timeout_s if job.kind == FILL
                     else self.params.swap_timeout_s)
            if job.canceled_at is None and now - job.sent_at > limit:
                self._cancel_arm(r)
                job.timed_out = True
            elif job.canceled_at is not None and (
                    now - job.canceled_at >= self.params.arm_cancel_wait_s):
                # Nobody can say what is in the basket now, so the operator has to look.
                r.arm = None
                reason = f"{job.server} goal {job.goal_id} never reported"
                self._fail(r, reason)
                self._event("ERROR", "ARM_NO_RESULT", reason, r.name)

    def step(self, robot) -> None:
        """Do what comes next for a robot traffic has made available."""
        r = self._robots[robot]
        if r.stage == FILLING:
            self._step_filling(r)
        elif r.stage == SWAPPING:
            self._step_swapping(r)

    def take_output(self) -> list:
        out, self._out = self._out, []
        return out

    def order_of(self, robot) -> str:
        order = self._robots[robot].order
        return order.order_id if order else ""

    def robot_view(self, robot) -> dict:
        r = self._robots[robot]
        return {
            "stage": r.stage,
            "basket": r.basket,
            "order": r.order.order_id if r.order else None,
            "placed": dict(r.order.placed) if r.order else {},
            "attempts": r.attempts,
        }

    # ---- helpers ------------------------------------------------------------------------

    def _own_goal_id(self) -> int:
        self._goal_seq += 1
        return self._goal_seq

    def _event(self, level, code, detail, robot="") -> None:
        self._out.append(EventOut(level, code, detail, robot))

    def _fail(self, r: _Robot, reason: str) -> None:
        """The operator has to look at it. It keeps any bound order."""
        self._out.append(RobotFaultCmd(r.name, reason))

    def _leave_station(self, r: _Robot) -> None:
        r.stage = ""
        self._out.append(ReleaseCmd(r.name))

    # ---- arms ---------------------------------------------------------------------------

    def _arm_job_owner(self, goal_id: int, kind: str) -> _Robot | None:
        for r in self._robots.values():
            if r.arm and r.arm.goal_id == goal_id and r.arm.kind == kind:
                return r
        return None

    def _forget_cleared_job(self, goal_id: int) -> None:
        for server, (cleared_goal, _) in list(self._cleared_jobs.items()):
            if cleared_goal == goal_id:
                del self._cleared_jobs[server]

    def _arm_goal_allowed(self, r: _Robot, server: str) -> bool:
        """Hold a new goal after a failure, and while the server stops a cleared robot's job."""
        now = self._clock()
        cleared = self._cleared_jobs.get(server)
        if cleared and now >= cleared[1]:
            del self._cleared_jobs[server]
            cleared = None
        return not cleared and now >= r.arm_retry_at

    def _cancel_arm(self, r: _Robot) -> None:
        r.arm.canceled_at = self._clock()
        self._out.append(CancelCmd(r.arm.server, r.arm.goal_id))

    def _arm_attempt_failed(self, r, job, reason, retry_code, failed_code) -> None:
        if reason == CANCELED and job.canceled_at is not None and not job.timed_out:
            return  # STOP canceled it, nothing went wrong
        r.attempts += 1
        r.arm_retry_at = self._clock() + self.params.arm_retry_wait_s
        if r.attempts > self.params.move_retry_limit:
            self._fail(r, f"{job.server}: {reason}")
            self._event("ERROR", failed_code, f"{job.server}: {reason}", r.name)
        else:
            self._event("WARN", retry_code,
                        f"{job.server} attempt {r.attempts} failed: {reason}", r.name)

    # ---- orders -------------------------------------------------------------------------

    def _pending(self) -> list:
        """Board orders nobody holds or finished, in registration order."""
        bound = {r.order.order_id for r in self._robots.values() if r.order}
        return [o for o in self._board
                if o.order_id not in bound and o.order_id not in self._reported]

    def _sync_calls(self) -> None:
        """Keep one open WORK dispatch per unbound order. The newest ones are canceled first."""
        wanted = len(self._pending())
        while len(self._open_calls) < wanted:
            goal_id = self._next_dispatch_id()
            self._open_calls.append(goal_id)
            self._out.append(DispatchCmd(goal_id, self._work))
        while len(self._open_calls) > wanted:
            self._out.append(CancelDispatchCmd(self._open_calls.pop()))

    def _next_dispatch_id(self) -> int:
        self._dispatch_seq += 1
        return self._dispatch_seq

    def _start_filling(self, r: _Robot) -> None:
        pending = self._pending()
        if not pending:
            self._out.append(ReleaseCmd(r.name))
            return
        record = pending[0]
        r.order = _BoundOrder(record.order_id, dict(record.items))
        r.last_order_id = record.order_id
        r.stage = FILLING
        self._event("INFO", "ORDER_BOUND", record.order_id, r.name)

    def _report(self, r: _Robot, status: str, detail: str = "") -> None:
        order, r.order = r.order, None
        self._reported.add(order.order_id)
        self._out.append(ReportOrderCmd(order.order_id, r.name, status, dict(order.placed),
                                        dict(order.missing), detail))

    def _report_shipped(self, r: _Robot) -> None:
        self.shipped += 1
        self._report(r, PARTIAL if r.order.missing else SHIPPED)

    def _finish_fill(self, r: _Robot) -> None:
        order = r.order
        r.attempts = 0
        order.missing = _subtract(order.items, order.placed)
        if not order.placed:
            self._report(r, HELD, f"nothing could be placed, missing {order.missing}")
            self._leave_station(r)
        elif order.missing and not self.params.ship_partial:
            detail = f"{order.order_id} partial not allowed, basket holds {order.placed}"
            r.basket = FULL
            r.stage = ""
            self._report(r, HELD, detail)
            self._fail(r, detail)
            self._event("ERROR", "FILL_FAILED", detail, r.name)
        else:
            r.basket = FULL
            r.stage = ""
            self._send_to_outbound(r)

    def _send_to_outbound(self, r: _Robot) -> None:
        r.outbound_goal = self._next_dispatch_id()
        self._out.append(DispatchCmd(r.outbound_goal, self._outbound, r.name))

    def _step_filling(self, r: _Robot) -> None:
        if r.arm:
            return
        order = r.order
        if sum(order.items.values()) > self.params.basket_capacity:
            # Nobody looked on the shelf, so nothing is missing: the stock record stays as it is.
            self._report(r, HELD, f"{sum(order.items.values())} items exceed the basket "
                                  f"capacity of {self.params.basket_capacity}")
            self._leave_station(r)
            return
        remaining = _subtract(order.items, order.placed, order.missing)
        if not remaining:
            self._finish_fill(r)
            return
        station = self._stations[self._work]
        if not self._arm_goal_allowed(r, station.server):
            return
        goal_id = self._next_goal_id()
        r.arm = _ArmJob(station.server, goal_id, FILL, self._clock())
        self._out.append(FillCmd(goal_id, r.name, station.id, order.order_id, remaining))

    def _step_swapping(self, r: _Robot) -> None:
        if r.arm:
            return
        if r.basket == EMPTY:
            self._leave_station(r)
            return
        station = self._stations[self._outbound]
        if not self._arm_goal_allowed(r, station.server):
            return
        goal_id = self._next_goal_id()
        r.arm = _ArmJob(station.server, goal_id, SWAP, self._clock())
        self._out.append(SwapCmd(station.server, goal_id, r.name, station.id, r.last_order_id,
                                 take_full=r.basket == FULL, put_empty=True))
