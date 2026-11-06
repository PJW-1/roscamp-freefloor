"""교통과 주문 코어를 직접 호출로 잇는다. 둘 사이 명령은 밖에 안 나가고 나머지는 tick()으로 나간다.

배차 API 밖으로 넘기는 것은 둘: run mode(STOP은 주문 없이 달리는 로봇도 세움)와 운영자의 로봇 제거.
외부 호출은 모두 _input을 거친다. 호출 동안 시각을 하나로 고정하고, 기록 중이면 남겨
ROS 없이 재생할 수 있다(형식은 tools/replay.py).
"""
from __future__ import annotations

import json
from dataclasses import asdict

from fleet_core.common.commands import (
    AssignmentView,
    CancelDispatchCmd,
    DispatchCmd,
    DispatchFeedback,
    DispatchResult,
    ReleaseCmd,
    RobotFaultCmd,
    RobotStatusView,
    ZoneView,
)
from fleet_core.orders.orchestrator import FILL, FILLING, SWAP, SWAPPING, OrderOrchestrator
from fleet_core.traffic.controller import TrafficController
from fleet_core.traffic.mission import (
    AT_STATION,
    DOCKING,
    OUT_OF_SERVICE,
    TO_OUTBOUND,
    TO_WORK,
)


class FleetManager:
    def __init__(self, site, params, robots, clock):
        self.site = site
        self.params = params
        self._clock = clock
        self._time = clock()
        self._write = None
        self._fed = False
        self._goal_seq = 0
        self.traffic = TrafficController(site, params, robots, self._frozen, self._next_goal_id)
        self.orchestrator = OrderOrchestrator(site.stations, params, robots, self._frozen,
                                             self._next_goal_id)
        self._work = next(s.id for s in site.stations.values() if s.action == FILL)
        self._outbound = next(s.id for s in site.stations.values() if s.action == SWAP)
        self._out: list = []

    @property
    def zones(self):
        return self.traffic.zones

    @property
    def shipped(self) -> int:
        return self.orchestrator.shipped

    def record_to(self, write) -> None:
        """모든 입력과 결과를 한 줄씩 JSON으로 `write`에 넘긴다.

        재생은 첫 줄에서 시작하므로 첫 입력보다 먼저 불러야 한다.
        """
        if self._fed:
            raise RuntimeError("a record has to start before the first input")
        if not self.site.source_text:
            raise ValueError("a record needs the text of the map, and this map has none")
        self._write = write
        write(json.dumps({"recorded": True, "robots": self.traffic.robots,
                          "params": asdict(self.params), "map": self.site.source_text,
                          "bundle_two_way": self.site.bundle_two_way}))

    # ---- inputs -------------------------------------------------------------------------

    def on_robot_state(self, robot, x, y, yaw, state, battery=0.0) -> None:
        self._input("on_robot_state", robot, x, y, yaw, state, battery)
        self.traffic.on_robot_state(robot, x, y, yaw, state, battery)
        self._pump()

    def on_order_board(self, run_mode, orders) -> None:
        self._input("on_order_board", run_mode, orders)
        self.traffic.on_run_mode(run_mode)
        self.orchestrator.on_order_board(run_mode, orders)
        self._pump()

    def on_move_result(self, robot, goal_id, success, reason) -> None:
        self._input("on_move_result", robot, goal_id, success, reason)
        self.traffic.on_move_result(robot, goal_id, success, reason)
        self._pump()

    def on_goal_rejected(self, robot, goal_id) -> None:
        self._input("on_goal_rejected", robot, goal_id)
        self.traffic.on_goal_rejected(robot, goal_id)
        self._pump()

    def on_goal_not_sent(self, robot, goal_id) -> None:
        self._input("on_goal_not_sent", robot, goal_id)
        self.traffic.on_goal_not_sent(robot, goal_id)
        self._pump()

    def on_dock_result(self, robot, goal_id, success, error_x_cm=0.0, error_y_cm=0.0,
                       error_yaw_deg=0.0) -> None:
        self._input("on_dock_result", robot, goal_id, success, error_x_cm, error_y_cm,
                    error_yaw_deg)
        self.traffic.on_dock_result(robot, goal_id, success, error_x_cm, error_y_cm, error_yaw_deg)
        self._pump()

    def on_fill_result(self, goal_id, success, placed, missing, reason="") -> None:
        self._input("on_fill_result", goal_id, success, placed, missing, reason)
        self.orchestrator.on_fill_result(goal_id, success, placed, missing, reason)
        self._pump()

    def on_swap_result(self, goal_id, success, took_full, put_empty_done, reason="") -> None:
        self._input("on_swap_result", goal_id, success, took_full, put_empty_done, reason)
        self.orchestrator.on_swap_result(goal_id, success, took_full, put_empty_done, reason)
        self._pump()

    def confirm_clear(self, robot, zone_ids, note="") -> list[str] | None:
        self._input("confirm_clear", robot, zone_ids, note)
        released = self.traffic.confirm_clear(robot, zone_ids, note)
        if released is not None:
            self.orchestrator.on_robot_removed(robot)
        self._pump()
        return released

    # ---- outputs ------------------------------------------------------------------------

    def tick(self) -> list:
        self._input("tick")
        traffic, orchestrator = self.traffic, self.orchestrator
        now = self._time
        everyone_here = traffic.tick_begin(now)
        self._pump()
        orchestrator.check_arm_timeouts(now)
        self._pump()
        halted = traffic.tick_progress(now)
        self._pump()
        if traffic.running and not halted and everyone_here:
            traffic.tick_pick()
            self._pump()
            for robot in traffic.robots:
                if traffic.available(robot):
                    self._step_robot(robot)
        out, self._out = self._out, []
        for item in out:
            self._note("out", item)
        return out

    def zones_view(self) -> list[ZoneView]:
        return self.traffic.zones_view()

    def assignments_view(self) -> list[AssignmentView]:
        views = []
        for robot in self.traffic.robots:
            t = self.traffic.robot_view(robot)
            if not t["on_map"] or t["fault"] == OUT_OF_SERVICE:
                continue
            order_id = self.orchestrator.order_of(robot)
            phase, dest = t["phase"], t["destination"]
            docking = dest if phase == DOCKING else None
            if phase == AT_STATION and dest == self._work:
                views.append(AssignmentView(robot, order_id, self._work, "FILL"))
            elif phase == AT_STATION:
                views.append(AssignmentView(robot, order_id, self._outbound, "SWAP"))
            elif phase == TO_OUTBOUND or docking == self._outbound:
                views.append(AssignmentView(robot, order_id, self._outbound, "TO_OUTBOUND"))
            elif docking == self._work or (phase == TO_WORK and t["called"]):
                views.append(AssignmentView(robot, order_id, self._work, "TO_WORK"))
            else:
                views.append(AssignmentView(robot, order_id, "", "WAIT"))
        return views

    def robots_view(self) -> list[RobotStatusView]:
        tasks = {view.robot_id: view for view in self.assignments_view()}
        views = []
        for robot in self.traffic.robots:
            task = tasks.get(robot)
            views.append(RobotStatusView(
                robot_id=robot,
                task_type=task.task_type if task else "",
                order_id=(task.order_id or "") if task else "",
                **self.traffic.status_view(robot)))
        return views

    def robot_view(self, robot) -> dict:
        t = self.traffic.robot_view(robot)
        o = self.orchestrator.robot_view(robot)
        phase = t["phase"]
        if phase == AT_STATION:
            phase = FILLING if t["destination"] == self._work else SWAPPING
        return {
            "state": t["fault"] or t["comm"] or phase,
            "phase": phase,
            "basket": o["basket"],
            "order": o["order"],
            "placed": o["placed"],
            "called": t["called"],
            "held": t["held"],
            "path": t["path"],
            "waypoint": t["waypoint"],
            "destination": t["destination"],
            "goal_id": t["goal_id"],
            "attempts": t["attempts"] + o["attempts"],
        }

    # ---- the join -----------------------------------------------------------------------

    def _frozen(self) -> float:
        return self._time

    def _input(self, method, *args) -> None:
        self._fed = True
        self._time = self._clock()
        if self._write:
            self._write(json.dumps({"t": self._time, "in": method, "args": args}, default=asdict))

    def _note(self, kind, item) -> None:
        if self._write:
            self._write(json.dumps({"t": self._time, kind: type(item).__name__, **asdict(item)}))

    def _next_goal_id(self) -> int:
        self._goal_seq += 1
        return self._goal_seq

    def _step_robot(self, robot) -> None:
        """주문, 교통 순으로 돌고, 방금 묶인 주문을 위해 주문 쪽을 한 번 더 돈다."""
        self.orchestrator.step(robot)
        self._pump()
        self.traffic.step(robot)
        self._pump()
        self.orchestrator.step(robot)
        self._pump()

    def _pump(self) -> None:
        """두 코어 모두 보낼 명령이 없을 때까지 내부 명령을 넘긴다."""
        while True:
            items = self.orchestrator.take_output() + self.traffic.take_output()
            if not items:
                return
            for item in items:
                self._route(item)

    def _route(self, item) -> None:
        if isinstance(item, DispatchCmd):
            self._note("inside", item)
            self.traffic.on_dispatch(item.goal_id, item.station_id, item.robot_id)
        elif isinstance(item, CancelDispatchCmd):
            self._note("inside", item)
            self.traffic.on_dispatch_cancel(item.goal_id)
        elif isinstance(item, ReleaseCmd):
            self._note("inside", item)
            self.traffic.on_release(item.robot)
        elif isinstance(item, RobotFaultCmd):
            self._note("inside", item)
            self.traffic.on_robot_fault(item.robot, item.reason)
        elif isinstance(item, DispatchFeedback):
            self._note("inside", item)
            self.orchestrator.on_dispatch_feedback(item.goal_id, item.robot_id, item.phase)
        elif isinstance(item, DispatchResult):
            self._note("inside", item)
            self.orchestrator.on_dispatch_result(item.goal_id, item.success, item.robot_id,
                                                 item.reason)
        else:
            self._out.append(item)
