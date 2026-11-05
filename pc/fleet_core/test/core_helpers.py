"""Small builders shared by the FleetCore unit tests."""
from fleet_core.params import load_params
from grid_map import grid_site
from legacy_core import FleetCore


def make_core(robots, params=None):
    """Return a core and a one-element list holding the fake time."""
    now = [0.0]
    core = FleetCore(grid_site(), params or load_params(), list(robots), lambda: now[0])
    return core, now


def place(core, robot, cell, yaw=0.0, state="IDLE"):
    """Report a pose at the centre of a cell, often enough for a release to count."""
    x, y = core.site.cell_pose(cell)
    for _ in range(core.params.release_confirm_samples):
        core.on_robot_state(robot, x, y, yaw, state)


def cmds(out, name):
    """Filter tick output by class name, so tests read as what they check."""
    return [c for c in out if type(c).__name__ == name]


def codes(out):
    return [e.code for e in cmds(out, "EventOut")]


def serve(core, robots, until, limit=300):
    """Tick, finishing every running move and dock at once, until `until()` holds.

    Arm commands are left unanswered, so a robot stops at them. Time does not advance.
    Goals are read from the missions, so one sent by a tick before this call counts too.
    Returns everything the ticks put out.
    """
    seen = []
    for _ in range(limit):
        seen += core.tick()
        for robot in robots:
            mission = core._missions[robot]
            goal = mission.goal
            if goal is not None and goal.kind == "move":
                place(core, robot, mission.path[goal.target_idx])
                core.on_move_result(robot, goal.id, True, "")
            elif goal is not None:
                core.on_dock_result(robot, goal.id, True)
            place(core, robot, mission.path[mission.idx])
        if until():
            return seen
    raise AssertionError(f"not reached: {[core.robot_view(r) for r in robots]}")
