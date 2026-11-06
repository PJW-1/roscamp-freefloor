"""FakeWorkcell: the fill_basket server with arm1 and arm2 behind it.

Each requested item goes to an arm whose stock has that kind. Both arms pick at the same time,
but one lock lets only one of them into the basket space (PLACING) at once.
"""
from __future__ import annotations

import json
import random
import threading
from collections import Counter

import rclpy
from fleet_core.common.contract import to_ros_qos
from fleet_interfaces.action import FillBasket
from fleet_interfaces.msg import ArmState, OrderItem
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from fake_robots.common import (
    STOCK_TOPIC,
    ScenarioClock,
    load_contract_param,
    load_lane_map_param,
    load_scenario_param,
    station_with_action,
)
from fake_robots.motion import ArmFaults, abort_at_shutdown, finish_action

ERROR_HOLD_S = 2.0
REASON_ARM_FAULT = "ARM_FAULT"
REASON_CANCELED = "CANCELED"
LOCK_POLL_S = 0.01

STOCK_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)


def order_items(counts: Counter) -> list[OrderItem]:
    return [OrderItem(kind=kind, count=count) for kind, count in sorted(counts.items()) if count]


class _Run:
    """One fill goal while the arm threads work on it."""

    def __init__(self, handle):
        self.handle = handle
        self.placed: Counter = Counter()
        self.fault = threading.Event()
        self.lock = threading.Lock()


class FakeWorkcellNode(Node):
    def __init__(self, parameter_overrides=None, **kwargs):
        super().__init__("fake_workcell", parameter_overrides=parameter_overrides, **kwargs)
        contract = load_contract_param(self)
        site = load_lane_map_param(self)
        scenario = load_scenario_param(self, site)
        self._arms = list(station_with_action(site, "fill").arms)
        self._cfg = scenario.arm
        self._faults = ArmFaults(scenario.faults)
        self._rng = random.Random(f"{scenario.seed}-workcell")
        self._sc = ScenarioClock(self)
        self._stock = {arm: Counter(scenario.stock.get(arm, {})) for arm in self._arms}
        self._stock_lock = threading.Lock()
        self._busy_lock = threading.Lock()
        self._busy = False
        self._alive = True
        # The rule the real server has to keep: only the holder of this lock may be PLACING.
        self._basket_space = threading.Lock()
        self._state_lock = threading.Lock()
        self._state = {arm: (ArmState.STATE_IDLE, "") for arm in self._arms}
        self._error_until = dict.fromkeys(self._arms, 0.0)
        self._cancel_at: float | None = None

        self._state_pubs = {
            arm: self.create_publisher(ArmState, contract.name("arm_state", arm=arm),
                                       to_ros_qos(contract.qos("arm_state")))
            for arm in self._arms}
        self._stock_pubs = {
            arm: self.create_publisher(String, STOCK_TOPIC.format(arm=arm), STOCK_QOS)
            for arm in self._arms}
        self._server = ActionServer(
            self, FillBasket, contract.name("fill_basket"),
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=ReentrantCallbackGroup())
        self.create_timer(0.5, self._tick)
        for arm in self._arms:
            self._publish_state(arm)
            self._publish_stock(arm)

    def stop(self) -> None:
        self._alive = False

    def destroy_node(self):
        self.stop()
        self._server.destroy()
        return super().destroy_node()

    def _running(self) -> bool:
        return self._alive and rclpy.ok()

    # -- state and stock ----------------------------------------------------------------

    def _set_state(self, arm: str, state: str, detail: str = "") -> None:
        with self._state_lock:
            self._state[arm] = (state, detail)
            self._publish_state(arm, locked=True)

    def _publish_state(self, arm: str, locked: bool = False) -> None:
        if not locked:
            with self._state_lock:
                self._publish_state(arm, locked=True)
            return
        # Reading, stamping and publishing stay together so a stamp never goes back in time
        # relative to a state change: the judge compares these stamps between the arms.
        state, detail = self._state[arm]
        self._state_pubs[arm].publish(ArmState(
            arm_id=arm, state=state, detail=detail, stamp=self.get_clock().now().to_msg()))

    def _tick(self) -> None:
        t = self._sc.now()
        for arm in self._arms:
            if self._state[arm][0] == ArmState.STATE_ERROR and t >= self._error_until[arm]:
                self._set_state(arm, ArmState.STATE_IDLE)
            else:
                self._publish_state(arm)

    def _publish_stock(self, arm: str) -> None:
        with self._stock_lock:
            data = json.dumps({k: c for k, c in self._stock[arm].items() if c > 0})
        self._stock_pubs[arm].publish(String(data=data))

    def _change_stock(self, arm: str, kind: str, delta: int) -> None:
        with self._stock_lock:
            self._stock[arm][kind] += delta
        self._publish_stock(arm)

    # -- planning -----------------------------------------------------------------------

    def _plan(self, items) -> tuple[dict[str, list[str]], Counter]:
        """Give each requested item to the arm with most of that kind. Rest goes to missing."""
        with self._stock_lock:
            left = {arm: Counter(self._stock[arm]) for arm in self._arms}
        units: dict[str, list[str]] = {arm: [] for arm in self._arms}
        missing: Counter = Counter()
        for item in items:
            for _ in range(max(item.count, 0)):
                arm = max(self._arms, key=lambda a: left[a][item.kind])
                if left[arm][item.kind] <= 0:
                    missing[item.kind] += 1
                    continue
                left[arm][item.kind] -= 1
                units[arm].append(item.kind)
        return units, missing

    def _plan_failures(self, units: dict[str, list[str]]) -> dict[str, int]:
        """Which arm fails in this goal, and before which of its items."""
        t = self._sc.now()
        fail_at = {}
        for arm in self._arms:
            at = self._faults.take_arm_fail(arm, t, len(units[arm]))
            if at is not None:
                fail_at[arm] = at
        planned = [(arm, i) for arm in self._arms for i in range(len(units[arm]))]
        if planned and self._rng.random() < self._cfg.fail_rate:
            arm, i = self._rng.choice(planned)
            fail_at[arm] = min(i, fail_at.get(arm, i))
        return fail_at

    # -- goal ---------------------------------------------------------------------------

    def _on_goal(self, _request):
        with self._busy_lock:
            if self._busy:
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _on_cancel(self, _handle):
        if self._cancel_at is None:
            self._cancel_at = self._sc.now()
        return CancelResponse.ACCEPT

    def _execute(self, handle):
        self._cancel_at = None
        units, missing = self._plan(handle.request.items)
        fail_at = self._plan_failures(units)
        run = _Run(handle)
        workers = [threading.Thread(target=self._work,
                                    args=(run, arm, units[arm], fail_at.get(arm)), daemon=True)
                   for arm in self._arms if units[arm]]
        for worker in workers:
            worker.start()
        for worker in workers:
            worker.join()
        if handle.is_cancel_requested and self._cancel_at is not None:
            # A real arm takes time to stop, and the server stays busy until it has.
            self._sc.sleep_until(self._cancel_at + self._cfg.cancel_seconds, self._running)

        planned = sum(len(u) for u in units.values())
        complete = sum(run.placed.values()) == planned
        success = complete and not run.fault.is_set()
        reason = ""
        if run.fault.is_set():
            reason = REASON_ARM_FAULT
        elif not complete:
            reason = REASON_CANCELED
        # `missing` is only final when the server finished trying; a retry works it out again.
        result = FillBasket.Result(
            success=success, placed=order_items(run.placed),
            missing=order_items(missing) if success else [], reason=reason)
        with self._busy_lock:
            self._busy = False
        if self._running():
            finish_action(handle, success)
        else:
            abort_at_shutdown(handle)
        return result

    def _go_on(self, run: _Run) -> bool:
        return (self._running() and not run.fault.is_set()
                and not run.handle.is_cancel_requested)

    def _work(self, run: _Run, arm: str, kinds: list[str], fail_at: int | None) -> None:
        half = self._cfg.seconds_per_item / 2
        robot = run.handle.request.robot_id
        for i, kind in enumerate(kinds):
            if not self._go_on(run):
                break
            if fail_at == i:
                run.fault.set()
                self._error_until[arm] = self._sc.now() + ERROR_HOLD_S
                self._set_state(arm, ArmState.STATE_ERROR, robot)
                return
            self._set_state(arm, ArmState.STATE_PICKING, robot)
            if not self._sc.sleep_until(self._sc.now() + half, lambda: self._go_on(run)):
                break
            self._change_stock(arm, kind, -1)
            if not self._enter_basket_space(run):
                self._change_stock(arm, kind, +1)  # back on the shelf, it never reached the basket
                break
            try:
                self._set_state(arm, ArmState.STATE_PLACING, robot)
                placed = self._sc.sleep_until(self._sc.now() + half, self._running)
                # Published before the lock is released, so the other arm's PLACING comes later.
                self._set_state(arm, ArmState.STATE_IDLE)
            finally:
                self._basket_space.release()
            if not placed:
                return
            with run.lock:
                run.placed[kind] += 1
                total = sum(run.placed.values())
            run.handle.publish_feedback(FillBasket.Feedback(
                arm_id=arm, phase="PLACED", placed_total=total))
        self._set_state(arm, ArmState.STATE_IDLE)

    def _enter_basket_space(self, run: _Run) -> bool:
        while self._go_on(run):
            if self._basket_space.acquire(timeout=LOCK_POLL_S):
                return True
        return False


def main(args=None):
    rclpy.init(args=args)
    node = FakeWorkcellNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception:  # SIGINT can land inside a message take; a real error has ROS still up
        if rclpy.ok():
            raise
    finally:
        node.stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
