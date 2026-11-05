"""Traffic on the default map: diagonal lanes, a two-way avenue, and a crossing with no node."""
from dataclasses import replace

import pytest
from core_helpers import cmds, codes
from fleet_core.commands import EventOut
from fleet_core.params import load_params
from fleet_core.site_map import load_site_map
from fleet_core.traffic_core import TrafficCore


def make(where):
    now = [0.0]
    core = TrafficCore(load_site_map(), load_params(), list(where), lambda: now[0])
    for robot, node in where.items():
        place(core, robot, node)
    core.on_run_mode("RUN")
    return core, now


def place(core, robot, node, along=0.0, to=None):
    """Report a pose on a node, or `along` metres down the lane to the node `to`."""
    x, y = core.site.cell_pose(node)
    if to is not None:
        lane = core.site.lane(node, to)
        x, y = x + lane.ux * along, y + lane.uy * along
    for _ in range(core.params.release_confirm_samples):
        core.on_robot_state(robot, x, y, 0.0, "IDLE")


def send(core, robot, station):
    core.on_dispatch(sum(map(ord, robot + station)), station, robot)


def held(core, robot):
    return core.robot_view(robot)["held"]


def move_of(out, robot):
    return [c for c in cmds(out, "MoveCmd") if c.robot == robot]


def test_a_move_goes_straight_to_the_end_of_a_diagonal_lane():
    core, _ = make({"r1": "W"})
    send(core, "r1", "OUTBOUND")
    move = move_of(core.tick(), "r1")[0]
    assert move.target_id == "A"
    assert (move.x, move.y) == core.site.cell_pose("A")
    assert held(core, "r1") == ["W", "AVENUE"]


def test_one_move_per_lane_when_the_direction_changes():
    """Lane compliance belongs to traffic: Nav2 only ever gets the end of the next straight."""
    core, _ = make({"r1": "W"})
    send(core, "r1", "OUTBOUND")
    move = move_of(core.tick(), "r1")[0]
    place(core, "r1", "A")
    core.on_move_result("r1", move.goal_id, True, "")
    move = move_of(core.tick(), "r1")[0]
    assert move.target_id == "B", "the avenue is one straight lane"
    assert move.yaw == pytest.approx(0.0)


def test_the_zone_behind_is_released_by_progress_along_the_diagonal_lane():
    core, _ = make({"r1": "W"})
    send(core, "r1", "OUTBOUND")
    core.tick()
    d = core.params.release_distance_m
    place(core, "r1", "W", d - 0.01, "A")
    core.tick()
    assert "W" in held(core, "r1"), "just under d along the lane"
    place(core, "r1", "W", d + 0.01, "A")
    core.tick()
    assert "W" not in held(core, "r1")
    assert "AVENUE" in held(core, "r1")


def test_being_off_the_lane_does_not_change_the_progress_that_releases():
    core, _ = make({"r1": "W"})
    send(core, "r1", "OUTBOUND")
    core.tick()
    lane = core.site.lane("W", "A")
    x, y = core.site.cell_pose("W")
    s, off = core.params.release_distance_m + 0.01, 0.1
    for _ in range(core.params.release_confirm_samples):
        core.on_robot_state("r1", x + lane.ux * s - lane.uy * off, y + lane.uy * s + lane.ux * off,
                            0.0, "MOVING")
    core.tick()
    assert "W" not in held(core, "r1")


def test_one_report_past_the_release_distance_frees_nothing():
    core, _ = make({"r1": "W"})
    send(core, "r1", "OUTBOUND")
    core.tick()
    x, y = core.site.cell_pose("W")
    lane = core.site.lane("W", "A")
    s = core.params.release_distance_m + 0.05
    core.on_robot_state("r1", x + lane.ux * s, y + lane.uy * s, 0.0, "MOVING")
    core.tick()
    core.on_robot_state("r1", *core.site.cell_pose("W"), 0.0, "MOVING")
    core.tick()
    assert "W" in held(core, "r1")


def test_a_robot_far_off_its_lane_is_reported_once():
    core, _ = make({"r1": "W"})
    send(core, "r1", "OUTBOUND")
    core.tick()
    x, y = core.site.cell_pose("W")
    lane = core.site.lane("W", "A")
    off = core.params.lane_error_warn_m + 0.05
    for _ in range(3 * core.params.release_confirm_samples):
        core.on_robot_state("r1", x - lane.uy * off, y + lane.ux * off, 0.0, "MOVING")
    assert codes(core.tick()).count("LANE_ERROR") == 1


def test_a_robot_off_its_lane_on_the_next_goal_is_reported_again():
    """The next goal can go out with no report in between, so nothing else marks the new goal."""
    core, _ = make({"r1": "W"})
    send(core, "r1", "OUTBOUND")
    first = move_of(core.tick(), "r1")[-1]
    samples = core.params.release_confirm_samples
    off = core.params.lane_error_warn_m + 0.05

    def report_beside(a, b):
        x, y = core.site.cell_pose(a)
        lane = core.site.lane(a, b)
        for _ in range(samples):
            core.on_robot_state("r1", x - lane.uy * off, y + lane.ux * off, 0.0, "MOVING")

    report_beside("W", first.target_id)
    assert codes(core.tick()).count("LANE_ERROR") == 1

    # Past the node it left, so the next zone can be asked for and the goal after this one sent.
    place(core, "r1", "W", core.params.release_distance_m + 0.05, first.target_id)
    core.tick()
    place(core, "r1", first.target_id)
    core.on_move_result("r1", first.goal_id, True, "")
    second = move_of(core.tick(), "r1")[-1]
    assert second.goal_id != first.goal_id

    report_beside(first.target_id, second.target_id)
    assert codes(core.tick()).count("LANE_ERROR") == 1


def test_a_robot_on_its_lane_is_not_reported_around_a_corner():
    core, _ = make({"r1": "W"})
    send(core, "r1", "OUTBOUND")
    core.tick()
    d = core.params.release_distance_m
    out = []
    for along in (d + 0.05, d + 0.1, d + 0.2):
        place(core, "r1", "W", along, "A")
        out += core.tick()
    assert "LANE_ERROR" not in codes(out)


def test_the_avenue_is_one_lock_for_both_directions():
    core, _ = make({"r1": "W", "r2": "O"})
    send(core, "r1", "OUTBOUND")
    send(core, "r2", "WORK")
    out = core.tick()
    assert [m.target_id for m in move_of(out, "r1")] == ["A"]
    assert not move_of(out, "r2"), "the other end has to wait"
    assert core.zones.holder("AVENUE") == "r1"
    assert core.zones.queue("AVENUE") == ("r2",)


def test_the_other_direction_goes_once_the_avenue_is_clear():
    core, _ = make({"r1": "B", "r2": "O"})
    send(core, "r1", "OUTBOUND")
    send(core, "r2", "WORK")
    move = move_of(core.tick(), "r1")[0]
    assert move.target_id == "R"
    assert core.zones.queue("AVENUE") == ("r2",)
    place(core, "r1", "B", core.params.release_distance_m + 0.01, "R")
    out = core.tick()
    assert core.zones.holder("AVENUE") == "r2"
    assert [m.target_id for m in move_of(out, "r2")] == ["B"]


def test_the_crossing_road_and_the_avenue_lock_each_other():
    core, _ = make({"r1": "N", "r2": "W"})
    assert core.site.conflict_pairs(0.16, 0.2) == {frozenset({"AVENUE", "S"})}
    core.on_dispatch(1, "WORK", "r1")
    core.tick()
    assert core.zones.holder("S") == "r1"
    core.on_dispatch(2, "OUTBOUND", "r2")
    out = core.tick()
    assert not move_of(out, "r2"), "the avenue is free, but a robot is on the road crossing it"
    assert core.zones.holder("AVENUE") is None
    assert core.zones.blockers("r2", "AVENUE") == {"r1"}


def test_at_the_crossing_who_asked_first_goes_first():
    core, _ = make({"r1": "R", "r2": "N", "r3": "W"})
    core.zones.force_acquire("r1", "AVENUE")
    core.on_dispatch(1, "WORK", "r2")
    core.tick()
    assert core.zones.queue("S") == ("r2",)
    core.on_dispatch(2, "OUTBOUND", "r3")
    core.tick()
    assert core.zones.queue("AVENUE") == ("r3",)
    core.zones.release("r1", "AVENUE")
    core.tick()
    assert core.zones.holder("S") == "r2"
    assert core.zones.holder("AVENUE") is None, "r3 asked after r2 and waits"


def test_the_commit_node_comes_from_the_map():
    core, _ = make({"r1": "Q"})
    assert core.site.stations["WORK"].commit == "W"
    core.on_dispatch(1, "WORK")
    core.tick()
    core.tick()
    assert core.robot_view("r1")["phase"] == "TO_WORK"
    assert core.zones.holder("W") == "r1"
    core.on_dispatch_cancel(1)
    core.tick()
    assert core.robot_view("r1")["phase"] == "TO_WORK", "it holds the commit unit, too late to turn"


def test_before_the_commit_unit_a_robot_can_still_turn_off():
    core, _ = make({"r1": "L"})
    core.on_dispatch(1, "WORK")
    core.tick()
    assert core.robot_view("r1")["phase"] == "TO_WORK"
    assert "W" not in held(core, "r1")
    core.on_dispatch_cancel(1)
    core.tick()
    assert core.robot_view("r1")["phase"] == "TO_PARK"


def test_a_noisy_first_report_does_not_put_a_robot_on_the_next_node():
    """7 cm off a node is noise. Taking it for the lane ahead hands out locks nobody stands in."""
    site = load_site_map()
    core = TrafficCore(site, load_params(), ["a", "b"], lambda: 0.0)
    x, y = site.cell_pose("P-5")
    core.on_robot_state("a", x + 0.07, y - 0.02, 0.0, "IDLE")
    x, y = site.cell_pose("P-6")
    core.on_robot_state("b", x + 0.01, y - 0.06, 0.0, "IDLE")
    assert core.zones.held_by("a") == ["P-5"]
    assert core.zones.held_by("b") == ["P-6"]
    assert not core.take_output()


def test_a_robot_placed_well_along_a_lane_belongs_to_the_node_it_leads_to():
    site = load_site_map()
    core = TrafficCore(site, load_params(), ["a"], lambda: 0.0)
    lane = site.lane("P-6", "N")
    core.on_robot_state("a", lane.ax + lane.ux * 0.3, lane.ay + lane.uy * 0.3, 0.0, "IDLE")
    assert core.zones.held_by("a") == ["N"]


def test_recovery_trusts_a_noisy_report_only_as_far_as_what_the_robot_holds():
    """Stopped just past d with the zone behind already freed: a noisy report may look short of
    d, and that must not send it back into a zone another robot may have by now."""
    now = [0.0]
    site = load_site_map()
    params = load_params()
    core = TrafficCore(site, params, ["a", "b"], lambda: now[0])
    x, y = site.cell_pose("P-4")
    core.on_robot_state("a", x, y, 0.0, "IDLE")
    x, y = site.cell_pose("P-6")
    core.on_robot_state("b", x, y, 0.0, "IDLE")
    core.zones.release("a", "P-4")
    core.zones.force_acquire("b", "P-4")
    core.zones.force_acquire("a", "P-5")
    m = core._missions["a"]
    m.comm = "RECOVERING"
    m.recover_at = 0.0
    lane = site.lane("P-4", "P-5")
    m.x, m.y = lane.ax + lane.ux * 0.19, lane.ay + lane.uy * 0.19
    core._finish_recovery(m)
    assert not m.fault
    assert m.path == ["P-5"]


def test_recovery_on_a_node_is_not_thrown_by_a_noisy_report():
    site = load_site_map()
    core = TrafficCore(site, load_params(), ["a", "b"], lambda: 0.0)
    x, y = site.cell_pose("P-5")
    core.on_robot_state("a", x, y, 0.0, "IDLE")
    x, y = site.cell_pose("P-6")
    core.on_robot_state("b", x, y, 0.0, "IDLE")
    m = core._missions["a"]
    m.comm = "RECOVERING"
    m.x, m.y = site.cell_pose("P-5")
    m.x += 0.07
    core._finish_recovery(m)
    assert not m.fault
    assert m.path == ["P-5"]


def test_a_robot_placed_across_a_conflict_pair_from_another_is_refused():
    """Holding AVENUE and S together would let two robots drive through the crossing at once."""
    site = load_site_map()
    core = TrafficCore(site, load_params(), ["a", "b"], lambda: 0.0)
    x, y = site.cell_pose("A")
    core.on_robot_state("a", x, y, 0.0, "IDLE")
    lane = site.lane("N", "S")
    core.on_robot_state("b", lane.ax + lane.ux * 0.3, lane.ay + lane.uy * 0.3, 0.0, "IDLE")
    assert core.zones.held_by("a") == ["AVENUE"]
    assert core.zones.held_by("b") == []
    assert core.robot_view("b")["fault"] == "ERROR"
    events = [e for e in core.take_output() if e.code == "PLACEMENT_CONFLICT"]
    assert len(events) == 1
    assert "AVENUE" in events[0].detail and "a" in events[0].detail


def refused_on_the_crossing():
    """a holds AVENUE, b stands on the lane that crosses it. Nobody has been told to move yet."""
    now = [0.0]
    core = TrafficCore(load_site_map(), load_params(), ["a", "b"], lambda: now[0])
    place(core, "a", "A")
    place(core, "b", "N", 0.3, "S")
    core.on_run_mode("RUN")
    core.take_output()
    return core, now


def test_nothing_moves_while_a_placement_is_refused():
    """The robot holds no lock and is still standing there, so nobody may be sent past it."""
    core, _ = refused_on_the_crossing()
    send(core, "a", "OUTBOUND")
    for _ in range(3):
        assert not cmds(core.tick(), "MoveCmd")
    assert core.robot_view("b")["on_map"] is False


def test_everyone_is_placed_only_when_nobody_is_left_to_report():
    core = TrafficCore(load_site_map(), load_params(), ["a", "b"], lambda: 0.0)
    assert not core.everyone_placed
    place(core, "a", "A")
    assert not core.everyone_placed
    place(core, "b", "N", 0.3, "S")
    assert not core.everyone_placed, "refused"
    core.confirm_clear("b", [], "lifted off")
    assert core.everyone_placed


def test_a_refused_placement_is_said_once_however_often_it_reports():
    core, _ = refused_on_the_crossing()
    for _ in range(5):
        place(core, "b", "N", 0.3, "S")
        assert "PLACEMENT_CONFLICT" not in codes(core.tick())


def test_moving_the_refused_robot_away_settles_it_without_the_operator():
    core, _ = refused_on_the_crossing()
    place(core, "b", "P-5")
    assert core.robot_view("b")["fault"] == ""
    assert (core.robot_view("b")["on_map"], held(core, "b")) == (True, ["P-5"])
    send(core, "a", "OUTBOUND")
    assert move_of(core.tick(), "a")


def test_clearing_a_refused_robot_frees_the_others_and_keeps_it_off_the_map():
    core, _ = refused_on_the_crossing()
    assert core.confirm_clear("b", [], "lifted off") == []
    place(core, "b", "P-5")
    assert core.robot_view("b")["fault"] == "OUT_OF_SERVICE"
    assert (core.robot_view("b")["on_map"], held(core, "b")) == (False, [])
    send(core, "a", "OUTBOUND")
    assert move_of(core.tick(), "a")


def found_along_a_lane(robot, node, to, along):
    core = TrafficCore(load_site_map(), load_params(), [robot], lambda: 0.0)
    place(core, robot, node, along, to)
    core.on_run_mode("RUN")
    return core


def test_a_robot_found_along_a_lane_drives_to_its_end_first():
    """A new FMS finds robots wherever they stopped. A straight line from there to the next
    node is no lane."""
    core = found_along_a_lane("r1", "W", "A", 0.28)
    send(core, "r1", "OUTBOUND")
    move = move_of(core.tick(), "r1")[0]
    assert move.target_id == "A"
    assert core.robot_view("r1")["cell"] == "A"
    place(core, "r1", "A")
    core.on_move_result("r1", move.goal_id, True, "")
    assert move_of(core.tick(), "r1")[0].target_id == "B"


def test_a_robot_found_along_a_lane_with_nothing_to_do_still_gets_off_it():
    core = found_along_a_lane("r1", "W", "A", 0.28)
    assert move_of(core.tick(), "r1")[0].target_id == "A"


def test_a_robot_found_within_d_of_a_node_is_on_that_node():
    core = found_along_a_lane("r1", "W", "A", 0.1)
    send(core, "r1", "OUTBOUND")
    assert move_of(core.tick(), "r1")[0].target_id == "A"
    assert held(core, "r1") == ["W", "AVENUE"]


def test_a_robot_found_on_the_lane_into_a_parking_spot_finishes_that_lane():
    core = TrafficCore(load_site_map(), load_params(), ["a", "b"], lambda: 0.0)
    place(core, "b", "P-4")
    place(core, "a", "Q", 0.26, "P-3")
    core.on_run_mode("RUN")
    move = move_of(core.tick(), "a")[0]
    assert move.target_id == "P-3"
    place(core, "a", "P-3")
    core.on_move_result("a", move.goal_id, True, "")
    assert move_of(core.tick(), "a") == []


@pytest.mark.parametrize("state", ["MOVING", "DOCKING"])
def test_a_robot_still_on_somebodys_goal_goes_on_the_map_when_it_has_stopped(state):
    """A goal of the FMS before this one, one that came late or a robot still braking: the new FMS
    knows nothing of it, so it waits, and so does everybody else."""
    site = load_site_map()
    core = TrafficCore(site, load_params(), ["r1"], lambda: 0.0)
    x, y = site.cell_pose("W")
    core.on_robot_state("r1", x, y, 0.0, state)
    assert not core.robot_view("r1")["on_map"]
    assert not core.everyone_placed
    core.on_robot_state("r1", x, y, 0.0, "IDLE")
    assert core.robot_view("r1")["on_map"]


def test_a_robot_that_keeps_moving_is_called_out_like_a_silent_one():
    site = load_site_map()
    now = [0.0]
    core = TrafficCore(site, load_params(), ["r1"], lambda: now[0])
    x, y = site.cell_pose("W")
    core.on_robot_state("r1", x, y, 0.0, "MOVING")
    core.on_run_mode("RUN")
    core.tick()
    now[0] = load_params().startup_wait_s
    core.on_robot_state("r1", x, y, 0.0, "MOVING")
    events = [e for e in core.tick() if isinstance(e, EventOut)]
    assert [e.code for e in events] == ["ROBOT_NOT_REPORTING"]
    assert "stand still" in events[0].detail


@pytest.mark.parametrize("d", [0.1, 0.16])
def test_a_release_distance_within_two_radii_is_refused(d):
    """Every pair of lanes meeting at a node turns into a conflict pair, and all that shows is
    robots refused at the start: the waiting line cannot hold them next to each other."""
    with pytest.raises(ValueError, match="twice the robot radius"):
        TrafficCore(load_site_map(), replace(load_params(), release_distance_m=d), ["r1"],
                    lambda: 0.0)


def test_the_conflict_pairs_stay_as_they_are_just_past_two_radii():
    core = TrafficCore(load_site_map(), replace(load_params(), release_distance_m=0.17), ["r1"],
                       lambda: 0.0)
    assert core.site.conflict_pairs(0.16, 0.17) == {frozenset({"AVENUE", "S"})}
