"""FmsCore wires the two cores: inside calls stay inside, and the old outputs come out."""
from core_helpers import cmds
from fleet_core.commands import (
    CancelDispatchCmd,
    DispatchCmd,
    DispatchFeedback,
    DispatchResult,
    OrderRecord,
    ReleaseCmd,
    RobotFaultCmd,
)
from fleet_core.deadlock import wait_graph
from fleet_core.fms_core import FmsCore
from fleet_core.params import load_params
from grid_map import grid_site

INTERNAL = (DispatchCmd, CancelDispatchCmd, ReleaseCmd, RobotFaultCmd, DispatchFeedback,
            DispatchResult)
O1 = OrderRecord("O-001", {"A": 1})
O2 = OrderRecord("O-002", {"B": 1})


def make(cells, orders=()):
    now = [0.0]
    core = FmsCore(grid_site(), load_params(), list(cells), lambda: now[0])
    for robot, cell in cells.items():
        report(core, robot, cell)
    core.on_order_board("RUN", list(orders))
    return core, now


def report(core, robot, cell):
    x, y = core.site.cell_pose(cell)
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
    """A basket filled through the real results, on its way out, no white box."""
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
    """test_fleet_unit forces the same story through legacy_core; this one uses real inputs."""
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
    """Why the retreat choice ignores cargo at a station: no request, so no place in a cycle."""
    core, _ = make({"r1": (4, 3), "r2": (3, 3)}, [O1])
    drive(core, "r1", lambda: core.traffic.robot_view("r1")["phase"] == "AT_STATION")
    assert "r1" not in [r for z in core.zones.snapshot().values() for r in z["queue"]]
    assert not wait_graph(core.zones).get("r1")
