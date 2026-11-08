"""No robot is sent into a waiting queue whose exit an isolated robot holds (H3 item 2).

The feature map's queue P-1..P-3 is one-way and leaves only through QW. A robot that never moves
on its own (isolated or OUT_OF_SERVICE) holding QW traps every robot that goes in.
"""
from dataclasses import replace
from pathlib import Path

from fleet_core.common.params import load_params
from fleet_core.traffic.controller import TrafficController
from fleet_core.traffic.lane_map import load_lane_map
from fleet_core.traffic.mission import AT_STATION, OUT_OF_SERVICE, TO_PARK
from sim_world import SimWorld

FEATURE = Path(__file__).resolve().parents[3] / "sim/fake_robots/maps/feature_map.yaml"
PARAMS = load_params()
QUEUE = ("P-1", "P-2", "P-3")


def make(starts, faults=(), held=("r2",)):
    """Places the robots while paused, then runs. `held` robots stand still as if at a station."""
    now = [0.0]
    site = load_lane_map(FEATURE)
    core = TrafficController(site, PARAMS, list(starts), lambda: now[0])
    for robot, waypoint in starts.items():
        x, y = site.waypoint_pose(waypoint)
        for _ in range(PARAMS.release_confirm_samples):
            core.on_robot_state(robot, x, y, 0.0, "IDLE")
    core.on_run_mode("PAUSE")
    core.tick()
    for robot in held:
        m = core._missions[robot]
        m.phase, m.dest, m.hold = AT_STATION, m.path[0], True
    for robot in faults:
        core.on_robot_fault(robot, "GOAL_FAILED")
    core.on_run_mode("RUN")
    core.tick()
    return core


def released_at_outbound(core, robot="r2"):
    """robot has swapped at O and the orders side lets it go with no call."""
    m = core._missions[robot]
    assert m.phase == AT_STATION and m.dest == "O"
    core.on_release(robot)
    core.tick()  # tick_pick clears await_pick, then the robot looks for a parking spot
    core.tick()
    return m


def heading_into_queue(core, robot):
    m = core._missions[robot]
    return [w for w in m.path[m.idx:] if w in QUEUE]


def test_a_free_queue_still_takes_the_robot():
    core = make({"r1": "E0", "r2": "O"})
    m = released_at_outbound(core)
    assert m.phase == TO_PARK and heading_into_queue(core, "r2")


def test_a_robot_does_not_go_into_a_queue_whose_exit_an_isolated_robot_holds():
    core = make({"r1": "QW", "r2": "O"}, faults=["r1"])
    assert core.zones.held_by("r1") == [core.site.zone_of["QW"]]
    m = released_at_outbound(core)
    assert not heading_into_queue(core, "r2")
    assert m.phase == TO_PARK and m.path[-1] == "F1", "off the dock, it waits at the queue door"


def test_out_of_service_counts_as_an_indefinite_holder():
    core = make({"r1": "QW", "r2": "O"}, faults=["r1"])
    core._missions["r1"].fault = OUT_OF_SERVICE  # cleared with QW still held
    released_at_outbound(core)
    assert not heading_into_queue(core, "r2")


def test_a_robot_outside_a_dock_waits_where_it_is():
    core = make({"r1": "QW", "r2": "F0"}, faults=["r1"], held=())
    m = core._missions["r2"]
    assert m.path[m.idx:] == ["F0"]


def test_a_robot_on_its_way_stops_before_the_queue_when_the_exit_gets_held():
    core = make({"r1": "QW", "r2": "O"}, held=("r1", "r2"))
    m = released_at_outbound(core)
    assert heading_into_queue(core, "r2")
    core.on_robot_fault("r1", "GOAL_FAILED")
    core.tick()
    assert not heading_into_queue(core, "r2")
    assert m.path[-1] == m.path[core._anchor(m)] == "F0", "it stops where it must get to"


def test_once_the_exit_is_cleared_the_waiting_robot_goes_in():
    core = make({"r1": "QW", "r2": "F0"}, faults=["r1"], held=())
    assert not heading_into_queue(core, "r2")
    core.confirm_clear("r1", core.zones.held_by("r1"), "lifted off")
    core.tick()
    assert heading_into_queue(core, "r2")


def test_a_robot_already_in_the_queue_stays_put():
    core = make({"r1": "QW", "r2": "P-1"}, faults=["r1"], held=())
    m = core._missions["r2"]
    assert m.path[m.idx:] == ["P-1"], "it does not move up to the blocked exit"


# ---- the virtual warehouse (the review case) ---------------------------------------------------


def isolated_at_qw_world():
    """pinky1 stands on QW and is isolated before anyone moves. Two batches of orders."""
    robots = {"pinky1": {"start": "QW", "speed_mps": 0.20, "pose_noise_m": 0.03},
              "pinky2": {"start": "F1", "speed_mps": 0.18, "pose_noise_m": 0.03},
              "pinky3": {"start": "E1", "speed_mps": 0.20, "pose_noise_m": 0.03}}
    orders = [{"at_s": at, "id": f"O-{i:03d}", "items": {"AB"[i % 2]: 1}}
              for i, at in enumerate([1.0, 1.0, 1.0, 200.0, 200.0, 200.0], start=1)]
    world = SimWorld(site=load_lane_map(FEATURE), robots=robots, orders=orders,
                     params=replace(PARAMS, detour_enabled=True), run_mode="PAUSE",
                     faults=[{"at_s": 0.5, "type": "run_mode", "mode": "RUN"}])
    world.run(0.3)
    world.core.traffic.on_robot_fault("pinky1", "GOAL_FAILED")
    world.core._pump()
    return world


def test_in_the_warehouse_no_robot_enters_the_trapped_queue_and_both_batches_ship():
    world = isolated_at_qw_world()
    site = world.site
    assert world.core.zones.held_by("pinky1") == [site.zone_of["QW"]]
    queue_zones = {site.zone_of[w] for w in QUEUE}
    entered = []

    def finished():
        for robot in ("pinky2", "pinky3"):
            inside = queue_zones & set(world.core.zones.held_by(robot))
            if inside:
                entered.append((world.t, robot, sorted(inside)))
        return world.all_orders_final()

    assert world.run_until(finished, timeout_s=600), world.report_statuses()
    assert not entered, entered[:3]
    assert world.report_statuses() == {f"O-{i:03d}": "SHIPPED" for i in range(1, 7)}
    assert world.event_codes("ERROR").count("ROBOT_ISOLATED") == 1
