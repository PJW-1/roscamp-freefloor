"""OrderOrchestrator and ArmJobs joined the way FleetManager joins them, without traffic.

Tests written for the single orchestrator of before run on this unchanged: what the two say to
each other is passed on at once, in place, and everything else comes out of take_output().
"""
from fleet_core.arm.jobs import ArmJobs
from fleet_core.common.commands import ARM_COMMANDS, ARM_REPORTS
from fleet_core.orders.orchestrator import OrderOrchestrator

_ORDER_INPUTS = ("on_dispatch_feedback", "on_dispatch_result", "on_robot_removed",
                 "on_robot_reinstated", "on_robot_isolated", "step")


class OrdersAndArm:
    def __init__(self, stations, params, robots, clock):
        self.orders = OrderOrchestrator(stations, params, robots, clock)
        self.arm = ArmJobs(stations, params, robots, clock)
        self._out = []

    def __getattr__(self, name):
        found = getattr(self.orders, name)
        if name not in _ORDER_INPUTS:
            return found

        def call(*args, **kwargs):
            found(*args, **kwargs)
            self._settle()
        return call

    def on_order_board(self, run_mode, orders) -> None:
        self.arm.on_run_mode(run_mode)
        self.orders.on_order_board(run_mode, orders)
        self._settle()

    def on_fill_result(self, *args) -> None:
        self.arm.on_fill_result(*args)
        self._settle()

    def on_swap_result(self, *args) -> None:
        self.arm.on_swap_result(*args)
        self._settle()

    def check_arm_timeouts(self, now) -> None:
        self.arm.check_timeouts(now)
        self._settle()

    def take_output(self) -> list:
        out, self._out = self._out, []
        return out

    def robot_view(self, robot) -> dict:
        view = self.orders.robot_view(robot)
        return {**view, "attempts": self.arm.attempts(robot)} if view else view

    def _settle(self) -> None:
        for item in self.arm.take_output() + self.orders.take_output():
            self._route(item)

    def _route(self, item) -> None:
        if isinstance(item, ARM_COMMANDS):
            self.arm.on_command(item)
        elif isinstance(item, ARM_REPORTS):
            self.orders.on_arm_report(item)
        else:
            self._out.append(item)
            return
        self._settle()
