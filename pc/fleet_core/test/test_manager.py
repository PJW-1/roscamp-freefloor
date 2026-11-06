"""FleetManager는 두 코어를 잇는다: 내부 호출은 안에 머물고 옛 출력은 그대로 나온다."""
from core_helpers import cmds
from fleet_core.common.commands import (
    CancelDispatchCmd,
    DispatchCmd,
    DispatchFeedback,
    DispatchResult,
    OrderRecord,
    ReleaseCmd,
    RobotFaultCmd,
)
from fleet_core.common.params import load_params
from fleet_core.manager import FleetManager
from fleet_core.traffic.deadlock import wait_graph
from fleet_core.traffic.lane_map import load_lane_map
from grid_map import grid_site

INTERNAL = (DispatchCmd, CancelDispatchCmd, ReleaseCmd, RobotFaultCmd, DispatchFeedback,
            DispatchResult)
O1 = OrderRecord("O-001", {"A": 1})
O2 = OrderRecord("O-002", {"B": 1})


def make(cells, orders=()):
    now = [0.0]
    core = FleetManager(grid_site(), load_params(), list(cells), lambda: now[0])
    for robot, cell in cells.items():
        report(core, robot, cell)
    core.on_order_board("RUN", list(orders))
    return core, now


def report(core, robot, cell):
    x, y = core.site.waypoint_pose(cell)
    for _ in range(core.params.release_confirm_samples):
        core.on_robot_state(robot, x, y, 0.0, "IDLE")


def drive(core, robot, until, limit=300):
    seen = []
    for _ in range(limit):
        seen += core.tick()
        m = core.traffic._missions[robot]
        goal = m.goal
        if goal is not None and goal.kind == "move":
            report(core, robot, m.path[goal.target_idx])
            core.on_move_result(robot, goal.id, True, "")
        elif goal is not None:
            core.on_dock_result(robot, goal.id, True)
        report(core, robot, m.path[m.idx])
        if until():
            return seen + core.tick()
    raise AssertionError(f"not reached: {core.robot_view(robot)}")


def test_inside_calls_never_reach_the_ros_side():
    core, _ = make({"r1": (4, 3)}, [O1, O2])
    out = core.on_order_board("RUN", [O1, O2]) or core.tick()
    out += drive(core, "r1", lambda: core.robot_view("r1")["order"] == "O-001")
    assert out, "the walk produced commands"
    assert not [c for c in out if isinstance(c, INTERNAL)]


def test_an_order_goes_from_the_dock_to_the_fill_in_the_ticks_the_old_core_used():
    core, _ = make({"r1": (4, 3)}, [O1])
    out = drive(core, "r1", lambda: core.robot_view("r1")["order"] == "O-001")
    assert not [c for c in out if isinstance(c, INTERNAL)]
    assert cmds(out, "FillCmd")[0].items == {"A": 1}
    assert core.robot_view("r1")["phase"] == "FILLING"
    assert core.robot_view("r1")["state"] == "FILLING"


def test_the_view_shows_the_old_station_phases_not_the_traffic_one():
    core, _ = make({"r1": (4, 3)}, [O1])
    drive(core, "r1", lambda: core.traffic.robot_view("r1")["phase"] == "AT_STATION")
    assert core.robot_view("r1")["phase"] == "FILLING"
    assert [a.task_type for a in core.assignments_view()] == ["FILL"]


def test_goal_ids_of_moves_and_arm_jobs_share_one_sequence():
    core, _ = make({"r1": (4, 3)}, [O1])
    out = drive(core, "r1", lambda: core.robot_view("r1")["order"] == "O-001")
    ids = [c.goal_id for c in out if type(c).__name__ in ("MoveCmd", "DockCmd", "FillCmd")]
    assert ids == sorted(ids) and len(ids) == len(set(ids))


def test_confirm_clear_reaches_both_cores():
    core, _ = make({"r1": (4, 3)}, [O1])
    drive(core, "r1", lambda: core.robot_view("r1")["order"] == "O-001")
    released = core.confirm_clear("r1", core.zones.held_by("r1"), "lifted")
    assert released
    assert core.robot_view("r1")["state"] == "OUT_OF_SERVICE"
    assert core.robot_view("r1")["order"] is None
    assert core.confirm_clear("nobody", [], "typo") is None


def test_a_refused_confirm_clear_keeps_the_order_on_its_robot():
    core, _ = make({"r1": (4, 3)}, [O1])
    drive(core, "r1", lambda: core.robot_view("r1")["order"] == "O-001")
    assert core.confirm_clear("r1", ["NOPE"], "typo") is None
    assert core.confirm_clear("r1", [], "forgot the list") is None
    assert core.robot_view("r1")["order"] == "O-001"
    assert core.robot_view("r1")["state"] != "OUT_OF_SERVICE"


def test_clearing_a_robot_in_two_calls_puts_its_order_back_once():
    core, _ = make({"r1": (4, 3)}, [O1])
    drive(core, "r1", lambda: core.robot_view("r1")["order"] == "O-001")
    held = core.zones.held_by("r1")
    assert held
    core.confirm_clear("r1", ["Z-OUT"], "a zone it does not hold")
    first = [c.code for c in core.tick() if hasattr(c, "code")]
    core.confirm_clear("r1", held, "the one it does")
    second = [c.code for c in core.tick() if hasattr(c, "code")]
    assert first.count("ORDER_REQUEUED") == 1
    assert "ORDER_REQUEUED" not in second
    assert core.zones.held_by("r1") == []


def test_a_robot_faulted_by_the_arm_stops_being_available_to_traffic():
    core, now = make({"r1": (4, 3)}, [O1])
    out = drive(core, "r1", lambda: core.robot_view("r1")["order"] == "O-001")
    goal = cmds(out, "FillCmd")[0].goal_id
    for _ in range(core.params.move_retry_limit + 1):
        core.on_fill_result(goal, False, {}, {}, "GRIPPER")
        now[0] += core.params.arm_retry_wait_s
        report(core, "r1", (5, 3))
        fills = cmds(core.tick(), "FillCmd")
        if fills:
            goal = fills[0].goal_id
    assert core.robot_view("r1")["state"] == "ERROR"
    assert not core.traffic.available("r1")


def _filled_and_leaving(robot="r1"):
    """실제 결과로 채워지고 나가는 중인 바구니. 화이트박스 없음."""
    core, now = make({robot: (4, 3)}, [O1])
    out = drive(core, robot, lambda: core.robot_view(robot)["order"] == "O-001")
    core.on_fill_result(cmds(out, "FillCmd")[0].goal_id, True, {"A": 1}, {}, "")
    report(core, robot, (5, 3))
    core.tick()
    return core, now


def test_a_real_fill_opens_the_trip_out_by_name():
    core, _ = _filled_and_leaving()
    assert core.robot_view("r1")["basket"] == "FULL"
    assert core.robot_view("r1")["destination"] == "OUTBOUND"
    assert core.traffic._missions["r1"].dispatch is not None


def test_recovery_after_a_real_fill_still_heads_to_outbound():
    """test_fleet_unit은 legacy_core로 같은 상황을 강제하고, 여기서는 실제 입력을 쓴다."""
    core, now = _filled_and_leaving()
    now[0] += core.params.comm_timeout_s + 0.1
    core.tick()
    assert core.robot_view("r1")["state"] == "COMM_LOST"
    now[0] += 1.0
    report(core, "r1", (5, 3))
    core.tick()
    now[0] += core.params.recover_check_s
    report(core, "r1", (5, 3))
    core.tick()
    assert core.robot_view("r1")["state"] == "TO_OUTBOUND"
    assert core.robot_view("r1")["destination"] == "OUTBOUND"


def test_a_robot_at_a_station_is_never_in_a_wait_cycle():
    """후퇴 선택이 스테이션의 적재를 무시하는 이유: 요청이 없어 순환에 낄 수 없다."""
    core, _ = make({"r1": (4, 3), "r2": (3, 3)}, [O1])
    drive(core, "r1", lambda: core.traffic.robot_view("r1")["phase"] == "AT_STATION")
    assert "r1" not in [r for z in core.zones.snapshot().values() for r in z["queue"]]
    assert not wait_graph(core.zones).get("r1")


# ---- /fleet/robots ----------------------------------------------------------------------


def status(core, robot):
    return {view.robot_id: view for view in core.robots_view()}[robot]


def test_the_status_says_what_the_pinky_reported_and_where_it_is():
    core, _ = make({"r1": (4, 3)})
    x, y = core.site.waypoint_pose((4, 3))
    core.on_robot_state("r1", x, y, 0.5, "WAITING", 87.5)
    view = status(core, "r1")
    assert (view.fsm_state, view.x, view.y, view.yaw, view.battery) == ("WAITING", x, y, 0.5, 87.5)
    assert (view.fault, view.fault_reason) == ("NONE", "")
    assert (view.wait_zone, view.wait_robots) == ("", ())


def test_a_robot_nobody_has_heard_from_still_has_a_status():
    core = FleetManager(grid_site(), load_params(), ["r1"], lambda: 0.0)
    view = status(core, "r1")
    assert (view.fsm_state, view.task_type, view.order_id, view.fault) == ("", "", "", "NONE")


def test_comm_loss_shows_in_the_status_and_goes_away():
    core, now = make({"r1": (4, 3)})
    now[0] = core.params.comm_timeout_s + 0.1
    core.tick()
    assert (status(core, "r1").fault, status(core, "r1").fault_reason) == ("COMM_LOST", "NO_STATE")
    report(core, "r1", (4, 3))
    assert status(core, "r1").fault == "RECOVERING"
    now[0] += core.params.recover_check_s + 0.1
    report(core, "r1", (4, 3))
    core.tick()
    assert (status(core, "r1").fault, status(core, "r1").fault_reason) == ("NONE", "")


def test_an_error_shows_its_cause():
    core, _ = make({"r1": (4, 3)})
    core.traffic.on_robot_fault("r1", "NAV_ABORTED")
    assert (status(core, "r1").fault, status(core, "r1").fault_reason) == ("ERROR", "NAV_ABORTED")


def test_a_cleared_robot_is_out_of_service_with_no_task():
    core, _ = make({"r1": (4, 3)})
    core.confirm_clear("r1", core.zones.held_by("r1"), "lifted off")
    view = status(core, "r1")
    assert (view.fault, view.fault_reason, view.task_type) == ("OUT_OF_SERVICE", "lifted off", "")


def test_a_waiting_robot_shows_the_zone_and_who_is_in_the_way():
    now = [0.0]
    core = FleetManager(load_lane_map(), load_params(), ["a", "b"], lambda: now[0])
    report(core, "a", "P-6")
    report(core, "b", "P-5")
    core.on_order_board("RUN", [O1, O2])
    core.tick()
    assert (status(core, "b").wait_zone, status(core, "b").wait_robots) == ("P-6", ("a",))
    assert status(core, "a").wait_zone == ""
    lane = core.site.lane("P-6", "N")
    for _ in range(core.params.release_confirm_samples):
        core.on_robot_state("a", lane.ax + lane.ux * 0.3, lane.ay + lane.uy * 0.3, 0.0, "MOVING")
    core.tick()
    assert (status(core, "b").wait_zone, status(core, "b").wait_robots) == ("", ())


def test_sr17_every_robot_state_can_be_told_apart():
    """대기, 이동, 도킹, 담기 대기, 출고 대기, 오류 + 원인. 표는 worklog에 있다."""
    core, _ = make({"r1": (4, 3), "r2": (2, 0)})
    assert status(core, "r1").task_type == "WAIT"
    core.on_order_board("RUN", [O1])
    core.tick()
    assert status(core, "r1").task_type == "TO_WORK"
    out = drive(core, "r1", lambda: core.robot_view("r1")["order"] == "O-001")
    view = status(core, "r1")
    assert (view.task_type, view.order_id) == ("FILL", "O-001")
    core.on_fill_result(cmds(out, "FillCmd")[0].goal_id, True, {"A": 1}, {}, "")
    report(core, "r1", (5, 3))
    core.tick()
    assert status(core, "r1").task_type == "TO_OUTBOUND"
    drive(core, "r1", lambda: core.robot_view("r1")["phase"] == "SWAPPING")
    assert status(core, "r1").task_type == "SWAP"
    x, y = core.site.waypoint_pose((2, 0))
    core.on_robot_state("r2", x, y, 0.0, "DOCKING")
    assert status(core, "r2").fsm_state == "DOCKING"
    core.traffic.on_robot_fault("r2", "DOCK_FAILED")
    assert (status(core, "r2").fault, status(core, "r2").fault_reason) == ("ERROR", "DOCK_FAILED")
