"""주문, 바구니, 팔 작업을 맡는다. ROS, 실제 시계, 레인은 모른다.

로봇은 DispatchCmd로 부르고 ReleaseCmd로 놓아 준다. 어느 로봇이 어떻게 오는지는 교통이 정한다.
주문은 채우기 스테이션 도킹이 끝나야 묶인다. 그래서 도착 순서가 작업 순서이고,
가다가 실패한 로봇은 주문을 쥐고 있지 않다.

열린 WORK 배차 수는 항상 묶이지 않은 주문 수와 같다.
한쪽을 바꾸는 입력은 반환 전에 다른 쪽도 맞춘다.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from fleet_core.common.commands import (
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

# 스테이션에 머무는 목적
FILLING = "FILLING"
SWAPPING = "SWAPPING"

# 맵에 적힌 스테이션 동작
FILL = "fill"
SWAP = "swap"

# 트레이 위 상태
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
    kind: str  # FILL 또는 SWAP
    sent_at: float
    canceled_at: float | None = None
    timed_out: bool = False  # STOP이 아니라 시간 초과로 취소됨


@dataclass
class _LostJob:
    """ARM_NO_RESULT로 포기했거나 해제로 취소한 작업. 결과가 늦게 올 수 있다."""

    robot: str
    server: str
    kind: str  # FILL 또는 SWAP
    order_id: str  # 포기할 때 묶여 있던 주문, 이미 보고했으면 빈 값
    cleared: bool = False  # 운영자가 로봇을 해제해서 취소됨


@dataclass
class _BoundOrder:
    order_id: str
    items: dict[str, int]
    placed: dict[str, int] = field(default_factory=dict)
    missing: dict[str, int] = field(default_factory=dict)  # 확정, 다시 시도하지 않음


@dataclass
class _Robot:
    name: str
    stage: str = ""  # 팔 작업으로 스테이션에 잡혀 있는 동안 FILLING 또는 SWAPPING
    basket: str = EMPTY
    order: _BoundOrder | None = None
    last_order_id: str = ""  # 주문 보고 뒤에도 교환 goal에 붙일 이름
    outbound_goal: int | None = None
    arm: _ArmJob | None = None
    arm_retry_at: float = 0.0  # 실패나 거절 뒤 이 시각 전에는 새 팔 goal 없음
    attempts: int = 0


class OrderOrchestrator:
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
        self._reported: set[str] = set()  # 보드에 잠시 남아 있을 수 있다
        self._open_calls: list[int] = []  # 보냈지만 아직 답이 없는 WORK 배차
        self._work = next(s.id for s in stations.values() if s.action == FILL)
        self._outbound = next(s.id for s in stations.values() if s.action == SWAP)
        self._cleared_jobs: dict[str, tuple[int, float]] = {}  # 서버 -> 취소한 goal, 기한
        self._lost_jobs: dict[int, _LostJob] = {}  # goal -> 작업, 늦은 결과가 올 때까지

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
            if success and robot_id in self._robots:
                self._start_filling(self._robots[robot_id])
            self._sync_calls()
            return
        r = self._robots.get(robot_id)
        if r is None:
            return
        if r.outbound_goal == goal_id:
            r.outbound_goal = None
            if success:
                r.stage = SWAPPING

    def on_fill_result(self, goal_id, success, placed, missing, reason="") -> None:
        self._forget_cleared_job(goal_id)
        r = self._arm_job_owner(goal_id, FILL)
        if r is None:
            self._late_fill(goal_id, placed)
            return
        job, r.arm = r.arm, None
        _merge(r.order.placed, placed)
        if success:
            # 서버가 시도를 마쳤으니 바구니에 없는 것은 빠진 것으로 확정한다.
            r.order.missing = _subtract(r.order.items, r.order.placed)
            return
        self._arm_attempt_failed(r, job, reason, "FILL_RETRY", "FILL_FAILED")

    def on_swap_result(self, goal_id, success, took_full, put_empty_done, reason="") -> None:
        self._forget_cleared_job(goal_id)
        r = self._arm_job_owner(goal_id, SWAP)
        if r is None:
            self._late_swap(goal_id, took_full, put_empty_done)
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
        """운영자가 로봇을 뺐다. 주문은 등록 순서 자리로 돌아간다."""
        r = self._robots.get(robot)
        if r is None:
            return
        if r.arm:
            # 해제된 로봇에 팔이 한 일은 운영자가 센다. 늦은 결과가 뭔가 했으면 알린다.
            # 서버가 아직 멈추는 중이어도 다음 로봇 탓으로 치지 않는다.
            self._out.append(CancelCmd(r.arm.server, r.arm.goal_id))
            self._cleared_jobs[r.arm.server] = (
                r.arm.goal_id, self._clock() + self.params.arm_cancel_wait_s)
            self._lost_jobs[r.arm.goal_id] = _LostJob(robot, r.arm.server, r.arm.kind,
                                                      self.order_of(robot), cleared=True)
            r.arm = None
        if r.order:
            self._event("INFO", "ORDER_REQUEUED",
                        f"{r.order.order_id} was bound, basket held {r.order.placed}", robot)
            r.order = None
        r.stage = ""
        r.outbound_goal = None
        self._sync_calls()

    def on_robot_reinstated(self, robot, basket) -> None:
        """운영자가 로봇을 다시 넣었다. 바구니는 운영자가 본 값, 주문 없음, 재시도 0."""
        if robot in self._robots:
            self._robots[robot] = _Robot(robot, basket=basket)

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
                # 바구니 내용을 알 수 없으니 운영자가 확인해야 한다.
                r.arm = None
                self._lost_jobs[job.goal_id] = _LostJob(r.name, job.server, job.kind,
                                                        self.order_of(r.name))
                reason = f"{job.server} goal {job.goal_id} never reported"
                self._fail(r, reason)
                self._event("ERROR", "ARM_NO_RESULT", reason, r.name)

    def step(self, robot) -> None:
        """교통이 넘겨준 로봇의 다음 일을 한다."""
        r = self._robots.get(robot)
        if r is None:
            return
        if r.stage == FILLING:
            self._step_filling(r)
        elif r.stage == SWAPPING:
            self._step_swapping(r)
        elif r.basket == NONE and not r.order and r.outbound_goal is None:
            # 바구니 없이 다시 투입됐다. 주문을 받기 전에 출고대에서 빈 바구니를 받는다.
            self._send_to_outbound(r)

    def take_output(self) -> list:
        out, self._out = self._out, []
        return out

    def order_of(self, robot) -> str:
        r = self._robots.get(robot)
        return r.order.order_id if r and r.order else ""

    def robot_view(self, robot) -> dict:
        """모르는 로봇이면 빈 dict."""
        r = self._robots.get(robot)
        if r is None:
            return {}
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
        """운영자가 봐야 한다. 묶인 주문은 그대로 둔다."""
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
        """실패 직후와, 서버가 해제된 로봇의 작업을 멈추는 동안은 새 goal을 보내지 않는다."""
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
            return  # STOP으로 취소한 것이라 실패가 아니다
        r.attempts += 1
        r.arm_retry_at = self._clock() + self.params.arm_retry_wait_s
        if r.attempts > self.params.move_retry_limit:
            self._fail(r, f"{job.server}: {reason}")
            self._event("ERROR", failed_code, f"{job.server}: {reason}", r.name)
        else:
            self._event("WARN", retry_code,
                        f"{job.server} attempt {r.attempts} failed: {reason}", r.name)

    def _late_fill(self, goal_id: int, placed: dict[str, int]) -> None:
        lost = self._take_lost_job(goal_id, FILL)
        if lost is None or (lost.cleared and not placed):
            return  # 취소된 작업이 한 일이 없으면 운영자가 셀 것도 없다
        self._late_event(goal_id, lost, f"placed {placed}")
        if self._still_bound(lost):
            _merge(self._robots[lost.robot].order.placed, placed)

    def _late_swap(self, goal_id: int, took_full: bool, put_empty_done: bool) -> None:
        lost = self._take_lost_job(goal_id, SWAP)
        if lost is None or (lost.cleared and not (took_full or put_empty_done)):
            return
        self._late_event(goal_id, lost, f"took_full={took_full} put_empty_done={put_empty_done}")
        r = self._robots[lost.robot]
        if self._still_bound(lost) and took_full and r.basket == FULL:
            r.basket = EMPTY if put_empty_done else NONE
            self._report_shipped(r)

    def _take_lost_job(self, goal_id: int, kind: str) -> _LostJob | None:
        lost = self._lost_jobs.get(goal_id)
        if lost is None or lost.kind != kind:
            return None
        return self._lost_jobs.pop(goal_id)

    def _still_bound(self, lost: _LostJob) -> bool:
        """고장 중에는 로봇 해제만이 주문을 떼어 간다. 해제 뒤 다시 묶인 같은 주문은 다른 일이다."""
        return (bool(lost.order_id) and not lost.cleared
                and self.order_of(lost.robot) == lost.order_id)

    def _late_event(self, goal_id: int, lost: _LostJob, reported: str) -> None:
        if not lost.order_id:
            then = "its order had already been reported"
        elif self._still_bound(lost):
            then = f"{lost.order_id} is still bound and {lost.robot} stays in fault"
        else:
            then = (f"{lost.robot} was cleared and {lost.order_id} requeued, so it may be filled "
                    "or shipped twice: check the basket")
        after = "the clear" if lost.cleared else "ARM_NO_RESULT"
        self._event("WARN", "ARM_LATE_RESULT",
                    f"{lost.server} goal {goal_id} reported {reported} after {after}, {then}",
                    lost.robot)

    # ---- orders -------------------------------------------------------------------------

    def _pending(self) -> list:
        """아무도 쥐지 않았고 끝나지도 않은 보드 주문, 등록 순서대로."""
        bound = {r.order.order_id for r in self._robots.values() if r.order}
        return [o for o in self._board
                if o.order_id not in bound and o.order_id not in self._reported]

    def _sync_calls(self) -> None:
        """묶이지 않은 주문마다 WORK 배차를 하나씩 둔다. 줄일 때는 최신 것부터 취소한다."""
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
            # 선반을 보지 않았으니 빠진 것도 없다. 재고 기록은 그대로 둔다.
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
