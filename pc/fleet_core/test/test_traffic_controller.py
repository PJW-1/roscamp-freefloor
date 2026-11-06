"""TrafficController alone, driven only through the internal dispatch API. No orders exist here."""
from core_helpers import cmds, codes
from fleet_core.common.commands import DispatchFeedback, DispatchResult
from fleet_core.common.params import load_params
from fleet_core.traffic.controller import TrafficController
from grid_map import grid_site

WORK = "WORK"
OUTBOUND = "OUTBOUND"
WORK_DOCK = (5, 3)
OUT_DOCK = (9, 3)


def make(cells, mode="RUN"):
    now = [0.0]
    core = TrafficController(grid_site(), load_params(), list(cells), lambda: now[0])
    for robot, cell in cells.items():
        report(core, robot, cell)
    core.on_run_mode(mode)
    return core, now


def report(core, robot, cell):
    x, y = core.site.waypoint_pose(cell)
    for _ in range(core.params.release_confirm_samples):
        core.on_robot_state(robot, x, y, 0.0, "IDLE")


def drive(core, robot, until, limit=300):
    """Tick, completing the robot's moves and docks at once, until `until()` holds."""
    seen = []
    for _ in range(limit):
        seen += core.tick()
        m = core._missions[robot]
        goal = m.goal
        if goal is not None and goal.kind == "move":
            report(core, robot, m.path[goal.target_idx])
            core.on_move_result(robot, goal.id, True, "")
        elif goal is not None:
            core.on_dock_result(robot, goal.id, True)
        report(core, robot, m.path[m.idx])
        if until():
            return seen + core.take_output()
    raise AssertionError(f"not reached: {core.robot_view(robot)}")


def results(out):
    return [c for c in out if isinstance(c, DispatchResult)]


def at_station(core, robot):
    return core.robot_view(robot)["phase"] == "AT_STATION"


def test_nothing_is_called_without_an_open_dispatch():
    core, _ = make({"r1": (3, 4)})
    core.tick()
    assert core.robot_view("r1")["called"] is False
    assert core.robot_view("r1")["state"] == "PARKED"


def test_an_open_dispatch_calls_the_robot_it_picks_and_ends_in_a_result_at_the_dock():
    core, _ = make({"r1": (3, 4), "r2": (3, 5)})
    core.on_dispatch(1, WORK)
    core.tick()
    assert core.robot_view("r1")["called"] is True
    assert core.robot_view("r2")["called"] is False
    out = drive(core, "r1", lambda: at_station(core, "r1"))
    done = results(out)
    assert [(r.goal_id, r.success, r.robot_id) for r in done] == [(1, True, "r1")]
    assert "DOCKED" in codes(out)


def test_a_robot_with_a_result_waits_at_the_dock_until_it_is_released():
    core, _ = make({"r1": (4, 3)})
    core.on_dispatch(1, WORK)
    drive(core, "r1", lambda: at_station(core, "r1"))
    for _ in range(5):
        out = core.tick()
        assert not cmds(out, "MoveCmd"), "held at the dock"
    assert at_station(core, "r1")
    core.on_release("r1")
    out = core.tick()
    assert core.robot_view("r1")["state"] != "AT_STATION", "released, nothing open: it leaves"
    assert cmds(out, "MoveCmd")


def test_a_released_robot_at_the_fill_dock_takes_an_open_call_where_it_stands():
    core, _ = make({"r1": (4, 3)})
    core.on_dispatch(1, WORK)
    drive(core, "r1", lambda: at_station(core, "r1"))
    core.on_dispatch(2, WORK)
    core.on_release("r1")
    out = core.tick()
    assert [(r.goal_id, r.robot_id) for r in results(out)] == [(2, "r1")]
    assert not cmds(out, "MoveCmd") and not cmds(out, "DockCmd"), "it never left"
    assert at_station(core, "r1")


def test_a_named_dispatch_sends_that_robot_and_nobody_else():
    core, _ = make({"r1": (4, 3), "r2": (3, 4)})
    core.on_dispatch(1, OUTBOUND, "r2")
    out = core.tick()
    assert core.robot_view("r2")["state"] == "TO_OUTBOUND"
    assert core.robot_view("r1")["called"] is False
    assert cmds(out, "MoveCmd")


def test_a_robot_on_a_named_dispatch_is_not_a_candidate_for_the_open_ones():
    core, _ = make({"r1": (3, 4)})
    core.on_dispatch(1, OUTBOUND, "r1")
    core.on_dispatch(2, WORK)
    core.tick()
    assert core.robot_view("r1")["called"] is False


def test_a_named_dispatch_reports_its_own_goal_at_the_dock():
    core, _ = make({"r1": (9, 4)})
    core.on_dispatch(7, OUTBOUND, "r1")
    out = drive(core, "r1", lambda: at_station(core, "r1"))
    assert [(r.goal_id, r.success, r.robot_id) for r in results(out)] == [(7, True, "r1")]
    assert core.robot_view("r1")["destination"] == OUTBOUND


def test_a_named_dispatch_to_a_robot_that_cannot_go_fails_at_once():
    core, now = make({"r1": (3, 4)})
    now[0] = core.params.comm_timeout_s + 0.1
    core.tick()
    core.on_dispatch(1, OUTBOUND, "r1")
    done = results(core.tick())
    assert [(r.goal_id, r.success, r.reason) for r in done] == [(1, False, "NOT_AVAILABLE")]


def test_canceling_the_only_open_dispatch_before_the_commit_zone_sends_the_robot_back():
    core, _ = make({"r1": (3, 6)})
    core.on_dispatch(1, WORK)
    core.tick()
    assert core.robot_view("r1")["state"] == "TO_WORK"
    core.on_dispatch_cancel(1)
    core.tick()
    assert core.robot_view("r1")["called"] is False
    assert core.robot_view("r1")["state"] == "TO_PARK"


def test_canceling_after_the_commit_zone_goes_around_without_docking():
    core, _ = make({"r1": (4, 3)})
    core.on_dispatch(1, WORK)
    core.tick()
    assert "Q-WORK-2" not in core.robot_view("r1")["held"]
    assert core.robot_view("r1")["state"] == "TO_WORK", "past the commit zone"
    core.on_dispatch_cancel(1)
    out = drive(core, "r1", lambda: core.robot_view("r1")["state"] == "TO_PARK")
    assert not cmds(out, "DockCmd")
    assert not results(out)


def test_canceling_a_named_dispatch_fails_it_and_returns_the_robot_to_the_pool():
    core, _ = make({"r1": (6, 5)})
    core.on_dispatch(1, OUTBOUND, "r1")
    core.tick()
    core.on_dispatch_cancel(1)
    done = results(core.take_output())
    assert [(r.goal_id, r.success, r.reason) for r in done] == [(1, False, "CANCELED")]
    assert core.robot_view("r1")["state"] == "TO_PARK"


def test_no_robot_left_is_told_once_per_stall():
    core, _ = make({"r1": (3, 4)})
    core.tick()
    core.confirm_clear("r1", core.zones.held_by("r1"), "gone")
    core.on_dispatch(5, WORK)
    feedback = [c for c in core.tick() if isinstance(c, DispatchFeedback)]
    assert [(f.goal_id, f.phase) for f in feedback] == [(5, "NO_ROBOT")]
    assert not [c for c in core.tick() if isinstance(c, DispatchFeedback)]
    core.on_dispatch_cancel(5)
    core.tick()
    core.on_dispatch(6, WORK)
    assert [c for c in core.tick() if isinstance(c, DispatchFeedback)], "a new stall"


def test_confirm_clear_fails_a_named_dispatch_with_the_reason_the_orchestrator_expects():
    core, _ = make({"r1": (6, 5)})
    core.on_dispatch(1, OUTBOUND, "r1")
    core.tick()
    core.confirm_clear("r1", core.zones.held_by("r1"), "gone")
    done = results(core.take_output())
    assert [(r.goal_id, r.success, r.reason) for r in done] == [(1, False, "ROBOT_REMOVED")]
    assert core.robot_view("r1")["state"] == "OUT_OF_SERVICE"


def test_a_robot_giving_up_on_its_move_fails_its_named_dispatch():
    core, now = make({"r1": (6, 5)})
    core.on_dispatch(1, OUTBOUND, "r1")
    move = cmds(core.tick(), "MoveCmd")[0]
    goal_id = move.goal_id
    seen = []
    for _ in range(core.params.move_retry_limit + 1):
        core.on_move_result("r1", goal_id, False, "NAV_ABORTED")
        now[0] += core.params.move_retry_wait_s
        report(core, "r1", (6, 5))
        out = core.tick()
        seen += out
        moves = cmds(out, "MoveCmd")
        if moves:
            goal_id = moves[0].goal_id
    done = results(seen)
    assert [(r.goal_id, r.success) for r in done] == [(1, False)]
    assert "NAV_ABORTED" in done[0].reason


def test_on_robot_fault_stops_the_robot_keeps_its_zones_and_drops_its_requests():
    core, _ = make({"r1": (3, 4), "r2": (4, 3)})
    core.tick()
    held = core.zones.held_by("r1")
    core.on_robot_fault("r1", "arm said so")
    assert core.robot_view("r1")["state"] == "ERROR"
    assert core.zones.held_by("r1") == held
    assert not core.available("r1")
    core.on_dispatch(1, WORK)
    core.tick()
    assert core.robot_view("r1")["called"] is False


def test_priority_uses_only_the_station_and_the_named_dispatch():
    core, _ = make({"r1": (6, 5), "r2": (3, 4)})
    params = core.params
    assert core._priority("r1") == 0
    core.on_dispatch(1, OUTBOUND, "r1")
    assert core._priority("r1") == params.priority_has_cargo
    assert core._priority("r2") == 0


def test_a_second_named_dispatch_is_refused_while_one_is_open():
    """The first one's answer would otherwise never come, and its asker would wait for ever."""
    core, _ = make({"r1": (5, 3)})
    core.on_dispatch(7, "OUTBOUND", "r1")
    core.on_dispatch(8, "OUTBOUND", "r1")
    results = [r for r in core.take_output() if isinstance(r, DispatchResult)]
    assert [(r.goal_id, r.success, r.reason) for r in results] == [(8, False, "BUSY")]
    assert core._missions["r1"].dispatch.goal_id == 7
