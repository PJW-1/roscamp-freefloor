"""기본 지도의 교통: 대각선 레인, 양방향 대로, 지점 없는 교차."""
from dataclasses import replace

import pytest
from core_helpers import cmds, codes
from fleet_core.common.commands import EventOut
from fleet_core.common.params import load_params as _load_params
from fleet_core.traffic.controller import TrafficController
from fleet_core.traffic.lane_map import load_lane_map


def load_params():
    """이 파일은 point 모드(같은 방향 레인만 묶음)를 본다. path 모드는 test_lane_path.py."""
    return replace(_load_params(), lane_goal_mode="point")


def make(where):
    now = [0.0]
    core = TrafficController(load_lane_map(), load_params(), list(where), lambda: now[0])
    for robot, node in where.items():
        place(core, robot, node)
    core.on_run_mode("RUN")
    return core, now


def place(core, robot, node, along=0.0, to=None):
    """지점 위 자세를 보고하거나, 지점 `to`로 가는 레인을 `along` m 간 자세를 보고한다."""
    x, y = core.site.waypoint_pose(node)
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
    assert (move.x, move.y) == core.site.waypoint_pose("A")
    assert held(core, "r1") == ["W", "AVENUE"]


def test_one_move_per_lane_when_the_direction_changes():
    """레인 준수는 traffic의 몫이다. Nav2는 다음 직선의 끝만 받는다."""
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
    x, y = core.site.waypoint_pose("W")
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
    x, y = core.site.waypoint_pose("W")
    lane = core.site.lane("W", "A")
    s = core.params.release_distance_m + 0.05
    core.on_robot_state("r1", x + lane.ux * s, y + lane.uy * s, 0.0, "MOVING")
    core.tick()
    core.on_robot_state("r1", *core.site.waypoint_pose("W"), 0.0, "MOVING")
    core.tick()
    assert "W" in held(core, "r1")


def test_a_robot_far_off_its_lane_is_reported_once():
    core, _ = make({"r1": "W"})
    send(core, "r1", "OUTBOUND")
    core.tick()
    x, y = core.site.waypoint_pose("W")
    lane = core.site.lane("W", "A")
    off = core.params.lane_error_warn_m + 0.05
    for _ in range(3 * core.params.release_confirm_samples):
        core.on_robot_state("r1", x - lane.uy * off, y + lane.ux * off, 0.0, "MOVING")
    assert codes(core.tick()).count("LANE_ERROR") == 1


def test_a_robot_off_its_lane_on_the_next_goal_is_reported_again():
    """사이에 보고 없이 다음 목표가 나갈 수 있어, 새 목표를 알리는 것은 이것뿐이다."""
    core, _ = make({"r1": "W"})
    send(core, "r1", "OUTBOUND")
    first = move_of(core.tick(), "r1")[-1]
    samples = core.params.release_confirm_samples
    off = core.params.lane_error_warn_m + 0.05

    def report_beside(a, b):
        x, y = core.site.waypoint_pose(a)
        lane = core.site.lane(a, b)
        for _ in range(samples):
            core.on_robot_state("r1", x - lane.uy * off, y + lane.ux * off, 0.0, "MOVING")

    report_beside("W", first.target_id)
    assert codes(core.tick()).count("LANE_ERROR") == 1

    # 떠난 지점을 지났으므로 다음 구역을 요청하고 그다음 목표를 보낼 수 있다.
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
    """지점에서 7cm 벗어난 것은 잡음이다. 이를 앞 레인으로 보면 아무도 없는 잠금을 내준다."""
    site = load_lane_map()
    core = TrafficController(site, load_params(), ["a", "b"], lambda: 0.0)
    x, y = site.waypoint_pose("P-5")
    core.on_robot_state("a", x + 0.07, y - 0.02, 0.0, "IDLE")
    x, y = site.waypoint_pose("P-6")
    core.on_robot_state("b", x + 0.01, y - 0.06, 0.0, "IDLE")
    assert core.zones.held_by("a") == ["P-5"]
    assert core.zones.held_by("b") == ["P-6"]
    assert not core.take_output()


def test_a_robot_placed_well_along_a_lane_belongs_to_the_node_it_leads_to():
    site = load_lane_map()
    core = TrafficController(site, load_params(), ["a"], lambda: 0.0)
    lane = site.lane("P-6", "N")
    core.on_robot_state("a", lane.ax + lane.ux * 0.3, lane.ay + lane.uy * 0.3, 0.0, "IDLE")
    assert core.zones.held_by("a") == ["N"]


def test_recovery_trusts_a_noisy_report_only_as_far_as_what_the_robot_holds():
    """d를 막 지났고 구역은 이미 풀렸다. 잡음 보고가 d보다 짧아 보여도
    그 사이 다른 로봇이 얻었을 구역으로 돌려보내면 안 된다.
    """
    now = [0.0]
    site = load_lane_map()
    params = load_params()
    core = TrafficController(site, params, ["a", "b"], lambda: now[0])
    x, y = site.waypoint_pose("P-4")
    core.on_robot_state("a", x, y, 0.0, "IDLE")
    x, y = site.waypoint_pose("P-6")
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
    site = load_lane_map()
    core = TrafficController(site, load_params(), ["a", "b"], lambda: 0.0)
    x, y = site.waypoint_pose("P-5")
    core.on_robot_state("a", x, y, 0.0, "IDLE")
    x, y = site.waypoint_pose("P-6")
    core.on_robot_state("b", x, y, 0.0, "IDLE")
    m = core._missions["a"]
    m.comm = "RECOVERING"
    m.x, m.y = site.waypoint_pose("P-5")
    m.x += 0.07
    core._finish_recovery(m)
    assert not m.fault
    assert m.path == ["P-5"]


def test_a_robot_placed_across_a_conflict_pair_from_another_is_refused():
    """AVENUE와 S를 함께 쥐면 두 로봇이 교차로를 동시에 지나게 된다."""
    site = load_lane_map()
    core = TrafficController(site, load_params(), ["a", "b"], lambda: 0.0)
    x, y = site.waypoint_pose("A")
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
    """a는 AVENUE를 쥐고 b는 그것과 교차하는 레인에 서 있다. 아직 이동 지시는 없다."""
    now = [0.0]
    core = TrafficController(load_lane_map(), load_params(), ["a", "b"], lambda: now[0])
    place(core, "a", "A")
    place(core, "b", "N", 0.3, "S")
    core.on_run_mode("RUN")
    core.take_output()
    return core, now


def test_nothing_moves_while_a_placement_is_refused():
    """로봇은 잠금이 없고 아직 서 있으므로 아무도 그 앞을 지나게 하면 안 된다."""
    core, _ = refused_on_the_crossing()
    send(core, "a", "OUTBOUND")
    for _ in range(3):
        assert not cmds(core.tick(), "MoveCmd")
    assert core.robot_view("b")["on_map"] is False


def test_everyone_is_placed_only_when_nobody_is_left_to_report():
    core = TrafficController(load_lane_map(), load_params(), ["a", "b"], lambda: 0.0)
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
    core = TrafficController(load_lane_map(), load_params(), [robot], lambda: 0.0)
    place(core, robot, node, along, to)
    core.on_run_mode("RUN")
    return core


def test_a_robot_found_along_a_lane_drives_to_its_end_first():
    """새 FMS는 로봇을 멈춘 곳에서 찾는다. 거기서 다음 지점까지의 직선은 레인이 아니다."""
    core = found_along_a_lane("r1", "W", "A", 0.28)
    send(core, "r1", "OUTBOUND")
    move = move_of(core.tick(), "r1")[0]
    assert move.target_id == "A"
    assert core.robot_view("r1")["waypoint"] == "A"
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
    core = TrafficController(load_lane_map(), load_params(), ["a", "b"], lambda: 0.0)
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
    """이전 FMS의 목표(늦게 온 것, 아직 제동 중인 로봇)를 새 FMS는 모르므로
    그 로봇도 다른 모두도 기다린다.
    """
    site = load_lane_map()
    core = TrafficController(site, load_params(), ["r1"], lambda: 0.0)
    x, y = site.waypoint_pose("W")
    core.on_robot_state("r1", x, y, 0.0, state)
    assert not core.robot_view("r1")["on_map"]
    assert not core.everyone_placed
    core.on_robot_state("r1", x, y, 0.0, "IDLE")
    assert core.robot_view("r1")["on_map"]


def test_a_robot_that_keeps_moving_is_called_out_like_a_silent_one():
    site = load_lane_map()
    now = [0.0]
    core = TrafficController(site, load_params(), ["r1"], lambda: now[0])
    x, y = site.waypoint_pose("W")
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
    """한 지점에서 만나는 레인 쌍은 모두 충돌 쌍이 되어, 시작에서 거절되는 로봇만 드러난다.
    대기줄이 서로 붙은 로봇을 담지 못한다.
    """
    with pytest.raises(ValueError, match="twice the robot radius"):
        TrafficController(load_lane_map(), replace(load_params(), release_distance_m=d), ["r1"],
                    lambda: 0.0)


def test_the_conflict_pairs_stay_as_they_are_just_past_two_radii():
    core = TrafficController(load_lane_map(), replace(load_params(), release_distance_m=0.17),
                             ["r1"], lambda: 0.0)
    assert core.site.conflict_pairs(0.16, 0.17) == {frozenset({"AVENUE", "S"})}
