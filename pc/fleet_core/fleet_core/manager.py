"""교통과 주문 코어를 직접 호출로 잇는다. 둘 사이 명령은 밖에 안 나가고 나머지는 tick()으로 나간다.

배차 API 밖으로 넘기는 것은 둘: run mode(STOP은 주문 없이 달리는 로봇도 세움)와 운영자의 로봇 제거.
외부 호출은 모두 _input을 거친다. 호출 동안 시각을 하나로 고정하고, 기록 중이면 남겨
ROS 없이 재생할 수 있다(형식은 tools/replay.py).

밖에서 온 잘못된 입력은 기록한 뒤, 코어에 닿기 전에 여기서 거른다(common/validate.py).
서비스는 거부, 토픽은 버리고 INPUT_REJECTED(WARN)를 낸다. 서비스는 호출마다 한 번,
토픽은 출처마다 한 번 내고 정상 메시지가 오면 다시 무장한다.
"""
from __future__ import annotations

import json
from dataclasses import asdict, is_dataclass

from fleet_core.common.commands import (
    AssignmentView,
    CancelDispatchCmd,
    DispatchCmd,
    DispatchFeedback,
    DispatchResult,
    EventOut,
    MoveCmd,
    OrderRecord,
    ReleaseCmd,
    RobotFaultCmd,
    RobotIsolated,
    RobotStatusView,
    ZoneView,
)
from fleet_core.common.validate import (
    checked,
    pose_problem,
    reported_id_problem,
    robot_problem,
    run_mode_problem,
    text,
)
from fleet_core.orders.orchestrator import FILL, FILLING, SWAP, SWAPPING, OrderOrchestrator
from fleet_core.orders.validate import orders_problem, reinstate_problem
from fleet_core.traffic.mission import (
    AT_STATION,
    DOCKING,
    LOCALIZING,
    OUT_OF_SERVICE,
    TO_OUTBOUND,
    TO_WORK,
)
from fleet_core.traffic.modes import LANES, build_traffic


def _recordable(value):
    """JSON에 못 쓰는 값은 repr로 남긴다. 기록 때문에 입력 처리가 멈추면 안 된다."""
    if is_dataclass(value) and not isinstance(value, type):
        return asdict(value)
    return repr(value)


def _recorded_orders(orders):
    """OrderRecord는 필드로, dict와 객체는 repr 문자열로 남긴다. 재생에서도 주문이 아니다."""
    if not isinstance(orders, (list, tuple)):
        return orders
    return [asdict(o) if isinstance(o, OrderRecord)
            else o if o is None or isinstance(o, (str, int, float, list))
            else repr(o) for o in orders]


class FleetManager:
    def __init__(self, site, params, robots, clock, floor=None):
        """floor는 traffic_mode가 freefloor일 때 쓰는 바닥 지도다(실험)."""
        self.site = site
        self.params = params
        self._clock = clock
        self._time = clock()
        self._write = None
        self._fed = False
        self._goal_seq = 0
        self.traffic = build_traffic(site, params, robots, self._frozen, self._next_goal_id,
                                     floor)
        self.orchestrator = OrderOrchestrator(site.stations, params, robots, self._frozen,
                                             self._next_goal_id)
        self._work = next(s.id for s in site.stations.values() if s.action == FILL)
        self._outbound = next(s.id for s in site.stations.values() if s.action == SWAP)
        self._out: list = []
        self._bounds = site.bounds()
        self._board: list = []  # 검사를 통과한 마지막 주문 목록
        self._rejecting: set = set()  # 거부를 이미 알린 토픽 출처
        self.last_clear_reason = ""
        self.last_reinstate_reason = ""

    @property
    def zones(self):
        return self.traffic.zones

    @property
    def shipped(self) -> int:
        return self.orchestrator.shipped

    def check_recordable(self) -> None:
        """기록할 수 없으면 ValueError. 기록 파일을 열기 전에 부른다(옛 파일을 지우지 않게)."""
        if self.params.traffic_mode != LANES:
            raise ValueError(f"replay knows only lane traffic, not {self.params.traffic_mode}")

    def record_to(self, write) -> None:
        """모든 입력과 결과를 한 줄씩 JSON으로 `write`에 넘긴다.

        재생은 첫 줄에서 시작하므로 첫 입력보다 먼저 불러야 한다.
        """
        self.check_recordable()
        if self._fed:
            raise RuntimeError("a record has to start before the first input")
        if not self.site.source_text:
            raise ValueError("a record needs the text of the map, and this map has none")
        self._write = write
        write(json.dumps({"recorded": True, "robots": self.traffic.robots,
                          "params": asdict(self.params), "map": self.site.source_text,
                          "bundle_two_way": self.site.bundle_two_way}))

    # ---- inputs -------------------------------------------------------------------------

    def on_robot_state(self, robot, x, y, yaw, state, battery=0.0, robot_id="") -> None:
        """robot은 토픽의 로봇, robot_id는 메시지에 적힌 이름(빈 값 가능)."""
        # 받은 그대로 기록한다. robot_id가 없던 기록도 똑같이 읽힌다.
        no_id = isinstance(robot_id, str) and text(robot_id) == ""
        self._input("on_robot_state", robot, x, y, yaw, state, battery,
                    *([] if no_id else [robot_id]))
        # 위치를 못 잡은 보고의 자세는 쓰지 않으니 검사하지 않는다.
        localizing = isinstance(state, str) and text(state) == LOCALIZING
        problem = checked(lambda: robot_problem(robot, self.traffic.robots)
                          or reported_id_problem(robot, robot_id)
                          or ("" if localizing else pose_problem(
                              x, y, yaw, self._bounds, self.params.input_pose_margin_m)))
        if self._topic_rejected(("robot_state", text(robot)), problem, "robot state dropped",
                                robot):
            return
        # state는 판단에 쓰므로 글자로 바꿔 둔다. 비교가 실패하는 값이 tick까지 가면 안 된다.
        self.traffic.on_robot_state(robot, x, y, yaw, text(state), battery)
        self._pump()

    def on_order_board(self, run_mode, orders) -> None:
        """run_mode가 틀리면 통째로 버린다. 주문이 틀리면 모드는 적용하고 주문은 전 것을 둔다.

        틀린 주문 하나가 STOP을 막으면 안 되고, 하나만 빼면 철회로 보이기 때문이다.
        """
        self._input("on_order_board", run_mode, orders)
        mode_problem = checked(run_mode_problem, run_mode)
        if mode_problem:
            self._topic_rejected(("order_board",), mode_problem, "order board dropped")
            return
        if not self._topic_rejected(("order_board",), checked(orders_problem, orders),
                                    f"run_mode {text(run_mode)} taken, orders kept as they were"):
            self._board = list(orders)
        self.traffic.on_run_mode(run_mode)
        self.orchestrator.on_order_board(run_mode, self._board)
        self._pump()

    def on_move_result(self, robot, goal_id, success, reason) -> None:
        self._input("on_move_result", robot, goal_id, success, reason)
        if not self._robot_refused("move result", robot):
            self.traffic.on_move_result(robot, goal_id, success, reason)
            self._pump()

    def on_goal_rejected(self, robot, goal_id) -> None:
        self._input("on_goal_rejected", robot, goal_id)
        if not self._robot_refused("goal rejection", robot):
            self.traffic.on_goal_rejected(robot, goal_id)
            self._pump()

    def on_goal_not_sent(self, robot, goal_id) -> None:
        self._input("on_goal_not_sent", robot, goal_id)
        if not self._robot_refused("goal not sent", robot):
            self.traffic.on_goal_not_sent(robot, goal_id)
            self._pump()

    def on_goal_lost(self, robot, goal_id) -> None:
        """노드가 취소한 goal의 결과를 잃었다(goal_tracker가 취소를 그만둠)."""
        self._input("on_goal_lost", robot, goal_id)
        if not self._robot_refused("goal lost", robot):
            self.traffic.on_goal_lost(robot, goal_id)
            self._pump()

    def on_dock_result(self, robot, goal_id, success, error_x_cm=0.0, error_y_cm=0.0,
                       error_yaw_deg=0.0) -> None:
        self._input("on_dock_result", robot, goal_id, success, error_x_cm, error_y_cm,
                    error_yaw_deg)
        if not self._robot_refused("dock result", robot):
            self.traffic.on_dock_result(robot, goal_id, success, error_x_cm, error_y_cm,
                                        error_yaw_deg)
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
        """놓은 구역, 거부면 None. 거부 이유는 last_clear_reason(받아들이면 빈 값).

        격리된 로봇이면 주문은 이미 돌아갔으니 구역만 푼다. 아니면 묶인 주문도 되돌린다.

        모르는 로봇·구역은 INPUT_REJECTED, 이름이 맞는 거부는 CLEAR_REFUSED. 판단은 traffic이 한다.
        """
        self._input("confirm_clear", robot, zone_ids, note)
        released = self.traffic.confirm_clear(robot, zone_ids, note)
        self.last_clear_reason = self.traffic.clear_reason
        if released is not None:
            self.orchestrator.on_robot_removed(robot)
        self._pump()
        return released

    def reinstate(self, robot, basket, note="") -> bool:
        """치운 로봇을 다시 넣는다. 거부면 False, 이유는 last_reinstate_reason(받아들이면 빈 값).

        모르는 로봇, OUT_OF_SERVICE가 아닌 로봇, 틀린 basket은 INPUT_REJECTED.
        맵에는 다음 위치 보고로 오른다. 그 전에는 일을 받지 않고 다른 로봇도 멈추지 않는다.
        """
        self._input("reinstate", robot, basket, note)
        problem = checked(lambda: reinstate_problem(robot, basket, self.traffic.robots)
                          or self.traffic.reinstate_refusal(text(robot)))
        self.last_reinstate_reason = problem
        if problem:
            self._reject(f"reinstate refused: {problem}: {text(note)}", robot)
            return False
        name, basket = text(robot), text(basket)
        self.traffic.reinstate(name)
        self.orchestrator.on_robot_reinstated(name, basket)
        self._out.append(EventOut("INFO", "ROBOT_REINSTATED",
                                  f"basket {basket}, on the map at its next report: {text(note)}",
                                  name))
        self._pump()
        return True

    # ---- outputs ------------------------------------------------------------------------

    def tick(self) -> list:
        self._input("tick")
        traffic, orchestrator = self.traffic, self.orchestrator
        now = self._time
        everyone_here = traffic.tick_begin(now)
        self._pump()
        orchestrator.check_arm_timeouts(now)
        self._pump()
        stopped = traffic.tick_progress(now)  # robots of a wait cycle stand still (H1)
        self._pump()
        if traffic.running and everyone_here:
            traffic.tick_pick(stopped)
            self._pump()
            for robot in traffic.robots:
                if traffic.available(robot) and robot not in stopped:
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
        """모르는 로봇이면 빈 dict."""
        t = self.traffic.robot_view(robot)
        o = self.orchestrator.robot_view(robot)
        if not t or not o:
            return {}
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
            "blocked_lane": t["blocked_lane"],
        }

    # ---- the join -----------------------------------------------------------------------

    def _frozen(self) -> float:
        return self._time

    def _input(self, method, *args) -> None:
        self._fed = True
        self._time = self._clock()
        if self._write:
            self._write(self._input_line(method, args))

    def _input_line(self, method, args) -> str:
        """기록 줄. 못 쓰는 입력도 처리는 계속하고, 재생할 수 없다는 표시만 남긴다."""
        try:
            if method == "on_order_board":
                args = (args[0], _recorded_orders(args[1]))
            return json.dumps({"t": self._time, "in": method, "args": args}, default=_recordable)
        except Exception as error:  # 순환 참조, 문자열이 아닌 키, repr 실패
            return json.dumps({"t": self._time, "in": method,
                               "unrecordable": type(error).__name__})

    def _reject(self, detail, robot="") -> None:
        name = text(robot) if isinstance(robot, str) else ""
        known = name if name in self.traffic.robots else ""
        self._out.append(EventOut("WARN", "INPUT_REJECTED", detail, known))

    def _topic_rejected(self, source, problem, what, robot="") -> bool:
        """problem이 있으면 True. 출처마다 한 번 알리고, 정상 메시지가 오면 다시 무장한다."""
        if not problem:
            self._rejecting.discard(source)
            return False
        if source not in self._rejecting:
            self._rejecting.add(source)
            self._reject(f"{what}: {problem}", robot)
        return True

    def _robot_refused(self, what, robot) -> bool:
        """모르는 로봇의 goal 응답. 잘못 부른 경우에만 생긴다."""
        problem = checked(robot_problem, robot, self.traffic.robots)
        if problem:
            self._reject(f"{what} refused: {problem}")
        return bool(problem)

    def _note(self, kind, item) -> None:
        if self._write:
            fields = asdict(item)
            if isinstance(item, MoveCmd) and not item.path:
                del fields["path"]  # point 모드 줄은 path가 없던 기록과 같게 쓴다
            self._write(json.dumps({"t": self._time, kind: type(item).__name__, **fields}))

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
        elif isinstance(item, RobotIsolated):
            self._note("inside", item)
            self.orchestrator.on_robot_isolated(item.robot, item.reason)
        elif isinstance(item, DispatchFeedback):
            self._note("inside", item)
            self.orchestrator.on_dispatch_feedback(item.goal_id, item.robot_id, item.phase)
        elif isinstance(item, DispatchResult):
            self._note("inside", item)
            self.orchestrator.on_dispatch_result(item.goal_id, item.success, item.robot_id,
                                                 item.reason)
        else:
            self._out.append(item)
