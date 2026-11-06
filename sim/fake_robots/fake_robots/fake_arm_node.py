"""FakeArm: the outbound arm. Takes the full basket off the tray and puts an empty one on."""
from __future__ import annotations

import random
import threading

import rclpy
from fleet_core.common.contract import to_ros_qos
from fleet_interfaces.action import SwapBasket
from fleet_interfaces.msg import ArmState
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from fake_robots.common import (
    ScenarioClock,
    load_contract_param,
    load_lane_map_param,
    load_scenario_param,
    node_name,
    station_with_action,
)
from fake_robots.motion import ArmFaults, abort_at_shutdown, finish_action

ERROR_HOLD_S = 2.0
REASON_ARM_FAULT = "ARM_FAULT"
REASON_PUT_EMPTY = "PUT_EMPTY_FAILED"
REASON_CANCELED = "CANCELED"


class FakeArmNode(Node):
    def __init__(self, parameter_overrides=None, **kwargs):
        super().__init__(node_name("fake_arm", "arm_id", parameter_overrides),
                         parameter_overrides=parameter_overrides, **kwargs)
        self.declare_parameter("arm_id", "arm3")
        self.arm_id = self.get_parameter("arm_id").value
        contract = load_contract_param(self)
        site = load_lane_map_param(self)
        scenario = load_scenario_param(self, site)
        if self.arm_id not in station_with_action(site, "swap").arms:
            raise ValueError(f"{self.arm_id} does not swap baskets, FakeWorkcell runs the others")
        self._cfg = scenario.arm
        self._faults = ArmFaults(scenario.faults)
        self._rng = random.Random(f"{scenario.seed}-{self.arm_id}")
        self._sc = ScenarioClock(self)
        self._lock = threading.Lock()
        self._alive = True
        self._busy = False
        self._state, self._detail, self._error_until = ArmState.STATE_IDLE, "", 0.0

        self._state_pub = self.create_publisher(
            ArmState, contract.name("arm_state", arm=self.arm_id),
            to_ros_qos(contract.qos("arm_state")))
        self._server = ActionServer(
            self, SwapBasket, contract.name("swap_basket", arm=self.arm_id),
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.create_timer(0.5, self._tick)
        self._publish_state()

    def stop(self) -> None:
        self._alive = False

    def destroy_node(self):
        self.stop()
        self._server.destroy()
        return super().destroy_node()

    def _running(self) -> bool:
        return self._alive and rclpy.ok()

    def _on_goal(self, _request):
        with self._lock:
            if self._busy:
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _set_state(self, state: str, detail: str = "") -> None:
        self._state, self._detail = state, detail
        self._publish_state()

    def _publish_state(self) -> None:
        self._state_pub.publish(ArmState(
            arm_id=self.arm_id, state=self._state, detail=self._detail,
            stamp=self.get_clock().now().to_msg()))

    def _tick(self) -> None:
        if self._state == ArmState.STATE_ERROR and self._sc.now() >= self._error_until:
            self._set_state(ArmState.STATE_IDLE)
        else:
            self._publish_state()

    def _failure_step(self, steps: list[str]) -> tuple[str | None, str]:
        """The step that fails in this goal and why. swap_partial always hits the empty one."""
        if "PUT_EMPTY" in steps and self._faults.take_swap_partial(self._sc.now()):
            return "PUT_EMPTY", REASON_PUT_EMPTY
        if steps and self._rng.random() < self._cfg.fail_rate:
            return self._rng.choice(steps), REASON_ARM_FAULT
        return None, ""

    def _execute(self, handle):
        req = handle.request
        steps = (["TAKE_FULL"] if req.take_full else []) + (["PUT_EMPTY"] if req.put_empty else [])
        failing, fail_reason = self._failure_step(steps)
        done, reason = set(), ""
        step_s = self._cfg.swap_seconds / 2

        def keep_going():
            return self._running() and not handle.is_cancel_requested

        for step in steps:
            if not keep_going():
                reason = REASON_CANCELED
                break
            self._set_state(ArmState.STATE_SWAPPING, req.robot_id)
            handle.publish_feedback(SwapBasket.Feedback(phase=step))
            if not self._sc.sleep_until(self._sc.now() + step_s, keep_going):
                reason = REASON_CANCELED
                break
            if step == failing:
                reason = fail_reason
                self._error_until = self._sc.now() + ERROR_HOLD_S
                self._set_state(ArmState.STATE_ERROR, req.robot_id)
                break
            done.add(step)
        if self._state != ArmState.STATE_ERROR:
            self._set_state(ArmState.STATE_IDLE)
        success = not reason
        result = SwapBasket.Result(
            success=success, took_full="TAKE_FULL" in done, put_empty_done="PUT_EMPTY" in done,
            reason=reason)
        with self._lock:
            self._busy = False
        if self._running():
            finish_action(handle, success)
        else:
            abort_at_shutdown(handle)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FakeArmNode()
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
