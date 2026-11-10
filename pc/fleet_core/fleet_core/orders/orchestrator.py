"""주문과 바구니를 맡는다. ROS, 실제 시계, 레인, 팔 goal은 모른다.

로봇은 DispatchCmd로 부르고 ReleaseCmd로 놓아 준다. 어느 로봇이 어떻게 오는지는 교통이 정한다.
Arm work is asked for one attempt at a time (ArmFillCmd, ArmSwapCmd). Goals, time-outs and the
wait before a retry are the arm side's (arm.jobs); only results come back. Whether an attempt
failed is decided here and told with ArmFailedCmd.
주문은 채우기 스테이션 도킹이 끝나야 묶인다. 그래서 도착 순서가 작업 순서이고,
가다가 실패한 로봇은 주문을 쥐고 있지 않다.

열린 WORK 배차 수는 항상 묶이지 않은 주문 수와 같다.
한쪽을 바꾸는 입력은 반환 전에 다른 쪽도 맞춘다.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from fleet_core.common.commands import (
    FILL,
    HELD,
    NO_ROBOT,
    PARTIAL,
    SHIPPED,
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
    CancelDispatchCmd,
    DispatchCmd,
    EventOut,
    ReleaseCmd,
    ReportOrderCmd,
    RobotFaultCmd,
)

STOP = "STOP"

# 스테이션에 머무는 목적
FILLING = "FILLING"
SWAPPING = "SWAPPING"

# 트레이 위 상태
EMPTY = "EMPTY"
FULL = "FULL"
NONE = "NONE"


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
class _LostJob:
    """ARM_NO_RESULT로 포기했거나 해제로 취소한 작업. 결과가 늦게 올 수 있다."""

    robot: str
    server: str
    kind: str  # FILL 또는 SWAP
    order_id: str  # 포기할 때 묶여 있던 주문, 이미 보고했으면 빈 값
    cleared: bool = False  # 운영자가 로봇을 해제해서 취소됨
    bound: int = 0  # 그때 주문을 묶은 차례. 같은 주문을 다시 묶으면 다른 값이다
    isolated: bool = False  # 자동 격리로 취소됨(cleared도 참)


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
    dispatch_failures: int = 0  # failed outbound dispatches in a row
    dispatch_retry_at: float | None = None  # send the outbound dispatch again from this time
    dispatch_reason: str = ""  # why the last outbound dispatch failed
    asked: str = ""  # arm work asked for and not answered yet: FILL, SWAP or ""
    bound: int = 0  # 지금 주문을 묶은 차례(_bind_seq)
    isolated: str = ""  # 격리 이유. 해제(confirm_clear)로 지운다


class OrderOrchestrator:
    def __init__(self, stations, params, robots, clock):
        self.params = params
        self.shipped = 0
        self._stations = stations
        self._clock = clock
        self._dispatch_seq = 0
        self._bind_seq = 0
        self._robots = {name: _Robot(name) for name in robots}
        self._out: list = []
        self._run_mode: str | None = None
        self._board: list = []
        self._reported: set[str] = set()  # 보드에 잠시 남아 있을 수 있다
        self._open_calls: list[int] = []  # 보냈지만 아직 답이 없는 WORK 배차
        self._work = next(s.id for s in stations.values() if s.action == FILL)
        self._outbound = next(s.id for s in stations.values() if s.action == SWAP)
        self._lost_jobs: dict[int, _LostJob] = {}  # goal -> 작업, 늦은 결과가 올 때까지

    # ---- inputs -------------------------------------------------------------------------

    def on_order_board(self, run_mode, orders) -> None:
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
                r.dispatch_failures = 0
            else:
                # step() decides once the wait is over. Traffic isolates a robot right after
                # failing its dispatch, and an isolated robot is neither sent again nor faulted.
                r.dispatch_failures += 1
                r.dispatch_retry_at = self._clock() + self.params.dispatch_retry_wait_s
                r.dispatch_reason = reason

    def on_arm_report(self, report) -> None:
        """What the arm side answers: a refusal, a result, a job given up, a late result."""
        if isinstance(report, ArmLate):
            if report.kind == FILL:
                self._late_fill(report.goal_id, report.placed)
            else:
                self._late_swap(report.goal_id, report.took_full, report.put_empty_done)
            return
        r = self._robots.get(report.robot)
        if r is None:
            return
        if isinstance(report, ArmDeclined):
            r.asked = ""
        elif isinstance(report, ArmFillResult):
            self._on_fill_result(r, report.success, report.placed)
        elif isinstance(report, ArmSwapResult):
            self._on_swap_result(r, report.took_full, report.put_empty_done)
        elif isinstance(report, ArmLost):
            self._on_arm_lost(r, report)

    def _on_fill_result(self, r: _Robot, success, placed) -> None:
        r.asked = ""
        _merge(r.order.placed, placed)
        if success:
            # 서버가 시도를 마쳤으니 바구니에 없는 것은 빠진 것으로 확정한다.
            r.order.missing = _subtract(r.order.items, r.order.placed)
            return
        self._out.append(ArmFailedCmd(r.name))

    def _on_arm_lost(self, r: _Robot, lost: ArmLost) -> None:
        """Keeps what was bound then, to judge a late result. A drop gave it; else it is now."""
        if lost.cleared:
            order_id, bound = lost.order_id, lost.bound
        else:
            r.asked = ""
            order_id, bound = self.order_of(r.name), r.bound
        self._lost_jobs[lost.goal_id] = _LostJob(r.name, lost.server, lost.kind, order_id,
                                                 cleared=lost.cleared, bound=bound,
                                                 isolated=lost.isolated)

    def _on_swap_result(self, r: _Robot, took_full, put_empty_done) -> None:
        r.asked = ""
        if took_full and r.basket == FULL:
            r.basket = NONE
            self._report_shipped(r)
        if r.isolated:
            self._requeue_isolated(r)  # 가져가지 못했으면 이제 되돌린다
            return
        if put_empty_done and r.basket == NONE:
            r.basket = EMPTY
            self._out.append(ArmResetCmd(r.name))
            self._leave_station(r)
            return
        self._out.append(ArmFailedCmd(r.name))

    def on_robot_removed(self, robot) -> None:
        """운영자가 로봇을 뺐다. 아직 묶인 주문은 등록 순서 자리로 돌아간다.

        격리된 로봇은 대개 주문을 이미 되돌렸다. 예외 둘은 여기서 되돌린다: 교환 취소 결과를
        기다리던 주문(그 뒤 took_full이 와도 ARM_LATE_RESULT만 낸다), ARM_NO_RESULT로 쥔 주문.
        """
        r = self._robots.get(robot)
        if r is None:
            return
        r.isolated = ""
        if r.asked:
            self._drop_arm(r)
        if r.order:
            self._event("INFO", "ORDER_REQUEUED",
                        f"{r.order.order_id} was bound, basket held {r.order.placed}", robot)
            r.order = None
        r.stage = ""
        r.outbound_goal = None
        self._forget_dispatch_failures(r)
        self._sync_calls()

    def on_robot_reinstated(self, robot, basket) -> None:
        """운영자가 로봇을 다시 넣었다. 바구니는 운영자가 본 값, 주문 없음, 재시도 0."""
        if robot in self._robots:
            self._robots[robot] = _Robot(robot, basket=basket)
            self._out.append(ArmResetCmd(robot, forget=True))

    def on_robot_isolated(self, robot, reason) -> None:
        """교통이 로봇을 격리했다(지시서 5.5). 주문은 사람 없이 대기로 돌아간다.

        자리는 등록 순서다. 묶을 때 늘 대기 맨 앞을 쓰므로, 되돌린 주문은 한 번도 묶이지 않은
        주문보다 앞서고 되돌린 주문끼리는 등록 순서를 지킨다.
        두 번 출고를 막는 예외 둘: 결과 없는 팔 작업(ARM_NO_RESULT)이 그 주문에 걸려 있으면
        confirm_clear까지, 교환 중이면 그 결과까지 주문을 둔다. 팔이 이미 바구니를 가져갔을 수 있다.
        """
        r = self._robots.get(robot)
        if r is None or self._result_still_due(r):
            return
        r.isolated = reason
        if r.asked == SWAP and r.order:
            self._out.append(ArmStopCmd(r.name))
            return  # the swap result or the ARM_NO_RESULT rule takes over
        if r.asked:
            self._drop_arm(r, isolated=True)  # 늦게 담긴 품목은 이 로봇 바구니에 남는다
        self._requeue_isolated(r)

    def _requeue_isolated(self, r: _Robot) -> None:
        if r.order:
            self._event("INFO", "ORDER_REQUEUED",
                        f"{r.order.order_id} was bound, basket held {r.order.placed}, "
                        f"robot isolated: {r.isolated}", r.name)
            r.order = None
        r.basket = NONE  # 바구니 안 물건은 로봇과 함께 남는다. 사람이 치운다
        r.stage = ""
        r.outbound_goal = None
        self._forget_dispatch_failures(r)
        self._sync_calls()

    # ---- outputs ------------------------------------------------------------------------

    def step(self, robot) -> None:
        """교통이 넘겨준 로봇의 다음 일을 한다."""
        r = self._robots.get(robot)
        if r is None:
            return
        if r.stage == FILLING:
            self._step_filling(r)
        elif r.stage == SWAPPING:
            self._step_swapping(r)
        elif r.isolated or r.outbound_goal is not None:
            return
        elif r.dispatch_retry_at is not None:
            self._retry_outbound(r)
        elif r.basket == NONE and not r.order:
            # 바구니 없이 다시 투입됐다. 주문을 받기 전에 출고대에서 빈 바구니를 받는다.
            self._send_to_outbound(r)

    def take_output(self) -> list:
        out, self._out = self._out, []
        return out

    def station_job(self, robot) -> tuple[str, int] | None:
        """(FILL 또는 SWAP, 안 놓은 수량). 팔 일 중이 아니면 None."""
        r = self._robots.get(robot)
        if r is None or r.stage not in (FILLING, SWAPPING):
            return None
        if r.stage == SWAPPING:
            return SWAP, 1
        left = _subtract(r.order.items, r.order.placed, r.order.missing) if r.order else {}
        return FILL, sum(left.values())

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
        }

    # ---- helpers ------------------------------------------------------------------------

    def _event(self, level, code, detail, robot="") -> None:
        self._out.append(EventOut(level, code, detail, robot))

    def _fail(self, r: _Robot, reason: str) -> None:
        """교통에 알린다. 교통이 격리하면 RobotIsolated로 주문을 되돌린다."""
        self._out.append(RobotFaultCmd(r.name, reason))

    def _leave_station(self, r: _Robot) -> None:
        r.stage = ""
        self._out.append(ReleaseCmd(r.name))

    # ---- arms ---------------------------------------------------------------------------

    def _drop_arm(self, r: _Robot, isolated: bool = False) -> None:
        """Drops the arm job on a clear or an isolation; the operator counts what the arm did there.

        The arm side cancels the goal and answers with ArmLost, which carries the order bound
        now: a late result that did something is then reported.
        """
        self._out.append(ArmDropCmd(r.name, isolated, self.order_of(r.name), r.bound))
        r.asked = ""

    def _result_still_due(self, r: _Robot) -> bool:
        """ARM_NO_RESULT로 포기한 작업이 지금 묶인 주문에 걸려 있다."""
        return any(lost.robot == r.name and not lost.cleared and self._still_bound(lost)
                   for lost in self._lost_jobs.values())

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
        """고장 중에는 해제나 격리만 주문을 떼어 간다. 해제 뒤 다시 묶인 같은 주문은 다른 일이다."""
        return (bool(lost.order_id) and self.order_of(lost.robot) == lost.order_id
                and self._robots[lost.robot].bound == lost.bound)

    def _late_event(self, goal_id: int, lost: _LostJob, reported: str) -> None:
        if not lost.order_id:
            then = "its order had already been reported"
        elif self._still_bound(lost):
            then = f"{lost.order_id} is still bound and {lost.robot} stays in fault"
        else:
            then = (f"{lost.order_id} was requeued from {lost.robot}, so it may be filled "
                    "or shipped twice: check the basket")
        after = ("the isolation" if lost.isolated else "the clear" if lost.cleared
                 else "ARM_NO_RESULT")
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
        self._bind_seq += 1
        r.bound = self._bind_seq
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
        self._out.append(ArmResetCmd(r.name))
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

    def _retry_outbound(self, r: _Robot) -> None:
        """Sends a failed outbound dispatch again, or past the limit fails the robot.

        Like the arm retry limit: traffic isolates the robot and the isolation puts the order
        back (no HELD report).
        """
        if self._clock() < r.dispatch_retry_at:
            return
        r.dispatch_retry_at = None
        detail = (f"dispatch to {self._outbound} failed {r.dispatch_failures} times: "
                  f"{r.dispatch_reason}")
        if r.dispatch_failures > self.params.dispatch_retry_limit:
            self._fail(r, detail)
            self._event("ERROR", "DISPATCH_FAILED", detail, r.name)
            return
        self._event("WARN", "DISPATCH_RETRY", f"{detail}, sending it again", r.name)
        self._send_to_outbound(r)

    @staticmethod
    def _forget_dispatch_failures(r: _Robot) -> None:
        r.dispatch_failures = 0
        r.dispatch_retry_at = None
        r.dispatch_reason = ""

    def _step_filling(self, r: _Robot) -> None:
        if r.asked:
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
        r.asked = FILL
        self._out.append(ArmFillCmd(r.name, self._work, order.order_id, remaining))

    def _step_swapping(self, r: _Robot) -> None:
        if r.asked:
            return
        if r.basket == EMPTY:
            self._leave_station(r)
            return
        r.asked = SWAP
        self._out.append(ArmSwapCmd(r.name, self._outbound, r.last_order_id,
                                    take_full=r.basket == FULL, put_empty=True))
