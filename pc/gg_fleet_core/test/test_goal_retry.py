"""A failed move or dock waits before it goes again, and a success has to be where it says.

One robot on the default map, one tick per 0.1 s, and the robot's answers given as a function:
what comes back for a goal, right after the tick that sent it, as the node passes them on.
"""
import sys
from pathlib import Path

import pytest
import yaml
from core_helpers import cmds
from gg_fleet_core.common.commands import OrderRecord
from gg_fleet_core.common.params import load_params
from gg_fleet_core.manager import FleetManager
from gg_fleet_core.traffic.controller import TrafficController
from gg_fleet_core.traffic.lane_map import LaneMap, load_lane_map

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))

import replay  # noqa: E402

ROBOT = "pinky1"
ORDER = OrderRecord("O-001", {"A": 1})
GOALS = ("MoveCmd", "DockCmd")
WAIT_S = load_params().move_retry_wait_s
TOLERANCE_M = load_params().arrival_tolerance_m
SETTLE_S = load_params().arrival_settle_s


class Run:
    def __init__(self, start="P-6", write=None):
        self.now = [0.0]
        self.core = FleetManager(load_lane_map(), load_params(), [ROBOT], lambda: self.now[0])
        if write:
            self.core.record_to(write)
        self.pose = self.core.site.waypoint_pose(start)
        self.sent = []  # (time, command)
        self.events = []
        self.report()
        self.core.on_order_board("RUN", [ORDER])

    def report(self):
        self.core.on_robot_state(ROBOT, *self.pose, 0.0, "IDLE")

    def tick(self, answer=None):
        self.report()
        out = self.core.tick()
        self.events += cmds(out, "EventOut")
        for command in out:
            if type(command).__name__ in GOALS:
                self.sent.append((self.now[0], command))
                if answer:
                    answer(self, command)
        self.now[0] = round(self.now[0] + 0.1, 6)

    def run(self, seconds, answer=None):
        for _ in range(round(seconds / 0.1)):
            self.tick(answer)

    def run_until(self, done, seconds, answer=None):
        for _ in range(round(seconds / 0.1)):
            self.tick(answer)
            if done():
                return
        raise AssertionError(f"not reached in {seconds} s: {self.view()}")

    def view(self):
        return self.core.robot_view(ROBOT)

    def codes(self):
        return [e.code for e in self.events]

    def times(self, kind="MoveCmd"):
        return [t for t, c in self.sent if type(c).__name__ == kind]

    def gaps(self, kind="MoveCmd"):
        times = self.times(kind)
        return [round(b - a, 6) for a, b in zip(times, times[1:], strict=False)]


def is_move(command):
    return type(command).__name__ == "MoveCmd"


def arrive(run, command):
    """The robot drives there, reports it, and says so."""
    run.pose = (command.x, command.y)
    run.report()
    run.core.on_move_result(command.robot, command.goal_id, True, "")


def honest(run, command):
    if is_move(command):
        arrive(run, command)
    else:
        run.core.on_dock_result(command.robot, command.goal_id, True)


def nav_aborts(run, command):
    if is_move(command):
        run.core.on_move_result(command.robot, command.goal_id, False, "NAV_ABORTED")
    else:
        run.core.on_dock_result(command.robot, command.goal_id, False)


def rejects(run, command):
    run.core.on_goal_rejected(command.robot, command.goal_id)


def not_sent(run, command):
    run.core.on_goal_not_sent(command.robot, command.goal_id)


def says_success_without_moving(run, command):
    if is_move(command):
        run.core.on_move_result(command.robot, command.goal_id, True, "")


def to_the_dock(answer_dock):
    """Drive to the fill dock for real, and answer docks with `answer_dock`."""
    def answer(run, command):
        if is_move(command):
            arrive(run, command)
        else:
            answer_dock(run, command)
    return answer


# ---- the policy value ---------------------------------------------------------------------


def test_three_attempts_outlast_the_comm_timeout():
    params = load_params()
    assert params.move_retry_limit * params.move_retry_wait_s > params.comm_timeout_s


# ---- defect 1: retries wait ---------------------------------------------------------------


def test_a_failure_of_half_a_second_ends_with_the_robot_working():
    def fails_at_first(run, command):
        if run.now[0] < 0.5:
            nav_aborts(run, command)
        else:
            honest(run, command)

    run = Run()
    run.run(30.0, fails_at_first)
    assert run.view()["state"] == "FILLING"
    assert "GOAL_FAILED" not in run.codes()
    assert run.codes().count("GOAL_RETRY") == 1
    assert run.gaps()[0] >= WAIT_S


def test_a_permanent_failure_still_ends_in_error_after_three_attempts_later_than_before():
    run = Run()
    run.run_until(lambda: run.view()["state"] == "ERROR", 30.0, nav_aborts)
    assert len(run.times()) == 3
    assert all(gap >= WAIT_S for gap in run.gaps())
    assert run.now[0] >= 2 * WAIT_S
    assert run.view()["attempts"] == 3
    run.run(1.0, nav_aborts)
    assert run.codes().count("GOAL_FAILED") == 1
    assert run.codes().count("GOAL_RETRY") == 2


@pytest.mark.parametrize("answer", [nav_aborts, rejects], ids=["failed", "rejected"])
def test_nothing_goes_again_before_the_wait_is_over(answer):
    run = Run()
    run.tick(answer)
    first = run.times()[0]
    run.run(WAIT_S - 0.2, answer)
    assert len(run.times()) == 1, "held back"
    run.run(0.3)
    assert len(run.times()) == 2
    assert run.times()[1] == pytest.approx(first + WAIT_S)
    assert run.sent[0][1].target_id == run.sent[1][1].target_id


def test_a_canceled_result_for_the_current_goal_waits_as_well():
    def canceled(run, command):
        run.core.on_move_result(command.robot, command.goal_id, False, "CANCELED")

    run = Run()
    run.tick(canceled)
    run.run(WAIT_S - 0.2)
    assert len(run.times()) == 1
    assert run.view()["attempts"] == 1
    run.run(0.3)
    assert len(run.times()) == 2


def test_a_result_for_a_goal_it_no_longer_runs_does_not_start_a_wait():
    run = Run()
    run.tick()
    goal = run.sent[0][1]
    run.core.on_move_result(ROBOT, goal.goal_id + 100, False, "CANCELED")
    arrive(run, goal)
    run.run(1.0)  # long enough to leave the zone behind, well short of the wait
    assert len(run.times()) == 2
    assert run.view()["attempts"] == 0


@pytest.mark.parametrize("answer", [nav_aborts, rejects], ids=["failed", "rejected"])
def test_a_dock_waits_too_and_gives_up_after_three_attempts(answer):
    run = Run()
    run.run_until(lambda: run.view()["state"] == "ERROR", 60.0, to_the_dock(answer))
    run.tick()
    assert len(run.times("DockCmd")) == 3
    assert all(gap >= WAIT_S for gap in run.gaps("DockCmd"))
    assert run.codes().count("GOAL_FAILED") == 1


def test_a_dock_that_fails_once_docks_after_the_wait():
    answers = iter([nav_aborts])

    def once(run, command):
        next(answers, honest)(run, command)

    run = Run()
    run.run_until(lambda: run.view()["state"] == "FILLING", 60.0, to_the_dock(once))
    assert run.gaps("DockCmd")[0] >= WAIT_S


STRAIGHT_MAP = """
half_width_m: 0.05
lane_entry_m: 0.25
origin_m: [0.0, 0.0]
bundle_two_way: false
waypoints:
  - {id: W, x: 1.0, y: 1.0}
  - {id: A, x: 2.0, y: 1.0}
  - {id: O, x: 3.0, y: 1.0}
  - {id: E, x: 2.0, y: 2.0}
  - {id: P-1, x: 1.0, y: 2.0}
lanes:
  - {from: W, to: A}
  - {from: A, to: O}
  - {from: O, to: E}
  - {from: E, to: P-1}
  - {from: P-1, to: W}
stations:
  WORK: {dock: W, queue: [[P-1, W]], commit: W, server: workcell, arms: [arm1, arm2], action: fill}
  OUTBOUND: {dock: O, queue: [[A, O]], commit: O, server: arm3, arms: [arm3], action: swap}
parking: [P-1]
"""


def test_extending_a_goal_that_is_still_running_does_not_wait():
    now = [0.0]
    site = LaneMap(yaml.safe_load(STRAIGHT_MAP), STRAIGHT_MAP)
    core = TrafficController(site, load_params(), ["r1"], lambda: now[0])
    x, y = site.waypoint_pose("W")
    core.on_robot_state("r1", x, y, 0.0, "IDLE")
    core.on_run_mode("RUN")
    core.on_dispatch(1, "OUTBOUND", "r1")
    assert [c.target_id for c in cmds(core.tick(), "MoveCmd")] == ["A"]
    core._missions["r1"].retry_at = now[0] + 100.0  # white box: only a goal not running waits
    for _ in range(core.params.release_confirm_samples):
        core.on_robot_state("r1", x + 0.5, y, 0.0, "MOVING")
    assert [c.target_id for c in cmds(core.tick(), "MoveCmd")] == ["O"]


# ---- a server that is not there -----------------------------------------------------------


def test_a_server_missing_for_30_s_does_not_fault_the_robot():
    run = Run()
    run.run(30.0, not_sent)
    assert run.view()["state"] == "TO_WORK"
    assert run.view()["attempts"] == 0
    assert run.codes().count("GOAL_NOT_SENT") == 1
    assert next(e for e in run.events if e.code == "GOAL_NOT_SENT").level == "WARN"
    assert "GOAL_RETRY" not in run.codes()
    assert all(gap >= WAIT_S for gap in run.gaps())
    assert len(run.times()) > 10, "it keeps trying"
    run.run_until(lambda: run.view()["state"] == "FILLING", 60.0, honest)


def test_a_dock_server_missing_for_30_s_does_not_fault_the_robot():
    run = Run()
    run.run_until(lambda: run.times("DockCmd"), 60.0, to_the_dock(not_sent))
    run.run(30.0, to_the_dock(not_sent))
    assert run.view()["state"] == "DOCKING"
    assert run.view()["attempts"] == 0
    assert run.codes().count("GOAL_NOT_SENT") == 1
    assert all(gap >= WAIT_S for gap in run.gaps("DockCmd"))
    run.run_until(lambda: run.view()["state"] == "FILLING", 10.0, honest)


def test_not_sent_keeps_the_settle_index_as_a_rejection_does_and_keeps_the_zones():
    run = Run()
    run.tick(not_sent)
    mission = run.core.traffic._missions[ROBOT]
    assert mission.goal is None
    assert mission.settle_idx is not None
    held = run.view()["held"]
    run.run(WAIT_S)
    assert run.view()["held"] == held
    assert run.sent[1][1].target_id == run.sent[0][1].target_id


def test_a_goal_that_is_not_the_current_one_is_ignored_when_not_sent():
    run = Run()
    run.tick()
    goal = run.sent[0][1]
    run.core.on_goal_not_sent(ROBOT, goal.goal_id + 100)
    assert run.core.traffic._missions[ROBOT].goal.id == goal.goal_id
    run.tick()
    assert "GOAL_NOT_SENT" not in run.codes()


def test_the_warning_comes_again_after_a_goal_reached_the_server():
    run = Run()
    run.tick(not_sent)
    run.run(WAIT_S)  # this one went out and nothing came back for it, so it is running
    run.tick()
    goal = run.core.traffic._missions[ROBOT].goal
    assert goal is not None
    nav_aborts(run, run.sent[-1][1])
    run.run_until(lambda: run.codes().count("GOAL_NOT_SENT") == 2, WAIT_S + 1.0, not_sent)


def test_the_warning_comes_again_after_a_goal_answered_before_the_next_tick():
    run = Run()
    run.tick(not_sent)
    run.run(WAIT_S, rejects)
    assert len(run.times()) == 2
    run.run(WAIT_S, not_sent)
    run.tick()
    assert run.codes().count("GOAL_NOT_SENT") == 2


def test_a_record_with_a_missing_server_plays_back(tmp_path):
    lines = []
    run = Run(write=lines.append)
    run.run(5.0, not_sent)
    run.run_until(lambda: run.view()["state"] == "FILLING", 60.0, honest)
    path = tmp_path / "run.jsonl"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    assert sum('"in": "on_goal_not_sent"' in line for line in lines) >= 2
    result = replay.replay(path)
    assert result.compared and result.same, (result.line, result.recorded, result.produced)


# ---- defect 2: a success has to be where it says -------------------------------------------


def test_success_on_the_first_lane_with_a_pose_that_never_moved_is_a_failed_attempt():
    run = Run()
    run.run_until(lambda: run.view()["state"] == "ERROR", 30.0, says_success_without_moving)
    run.tick()
    assert run.core.traffic.status_view(ROBOT)["fault_reason"] == "NOT_AT_TARGET"
    assert run.view()["waypoint"] == "P-6", "it never left"
    assert len(run.times()) == 3
    assert all(gap >= WAIT_S for gap in run.gaps())
    assert run.codes().count("GOAL_RETRY") == 2
    assert run.codes().count("GOAL_FAILED") == 1
    assert "NOT_AT_TARGET" in next(e for e in run.events if e.code == "GOAL_RETRY").detail


def test_no_dock_goes_out_away_from_the_dock_node():
    run = Run(start="Q")
    run.run(30.0, says_success_without_moving)
    assert run.sent[0][1].target_id == "W"
    assert not run.times("DockCmd")
    assert run.view()["state"] == "ERROR"


def test_an_honest_arrival_within_the_tolerance_advances():
    def stops_short(run, command):
        if is_move(command):
            run.pose = (command.x - 0.9 * TOLERANCE_M, command.y)
            run.report()
            run.core.on_move_result(command.robot, command.goal_id, True, "")
        else:
            run.core.on_dock_result(command.robot, command.goal_id, True)

    run = Run()
    run.run_until(lambda: run.view()["state"] == "FILLING", 60.0, stops_short)
    assert "GOAL_RETRY" not in run.codes()
    assert run.sent[0][1].target_id == "N"


def test_a_stop_past_the_tolerance_goes_again_to_the_same_node_and_then_on():
    first = []

    def stops_short_once(run, command):
        if is_move(command) and not first:
            first.append(command)
            run.pose = (command.x - 1.1 * TOLERANCE_M, command.y)
            run.report()
            run.core.on_move_result(command.robot, command.goal_id, True, "")
        else:
            honest(run, command)

    run = Run()
    run.run_until(lambda: run.view()["state"] == "FILLING", 60.0, stops_short_once)
    assert run.sent[0][1].target_id == run.sent[1][1].target_id
    assert run.gaps()[0] >= WAIT_S
    assert run.codes().count("GOAL_RETRY") == 1


def test_a_report_that_lags_behind_the_result_is_not_a_failed_attempt():
    """The result can overtake the report that shows the arrival."""
    def result_first(run, command):
        if is_move(command):
            run.core.on_move_result(command.robot, command.goal_id, True, "")
            run.pose = (command.x, command.y)  # reported on the next tick
        else:
            run.core.on_dock_result(command.robot, command.goal_id, True)

    run = Run()
    run.run_until(lambda: run.view()["state"] == "FILLING", 60.0, result_first)
    assert "GOAL_RETRY" not in run.codes()
    assert all(gap < WAIT_S for gap in run.gaps())


def test_a_pose_that_stays_away_is_a_failed_attempt_only_when_the_settle_time_is_over():
    run = Run()
    run.tick(says_success_without_moving)
    run.run(SETTLE_S - 0.2)
    assert run.view()["attempts"] == 0
    assert "GOAL_RETRY" not in run.codes()
    assert len(run.times()) == 1, "nothing goes out while the reports can still catch up"
    run.run(0.3)
    assert run.view()["attempts"] == 1
    assert "NOT_AT_TARGET" in next(e for e in run.events if e.code == "GOAL_RETRY").detail
    assert len(run.times()) == 1, "and then it waits like any failed attempt"


def test_losing_comm_while_the_pose_is_awaited_leaves_the_judgement_to_recovery():
    run = Run()
    run.tick(says_success_without_moving)
    run.now[0] += run.core.params.comm_timeout_s + 0.1
    run.events += cmds(run.core.tick(), "EventOut")
    assert run.view()["state"] == "COMM_LOST"
    run.run_until(lambda: run.view()["state"] == "FILLING", 60.0, honest)
    assert "GOAL_RETRY" not in run.codes()


def test_with_no_report_after_the_success_the_old_pose_costs_no_attempt():
    """Reports that stop as a move ends say nothing about where it ended. Two real failures
    before it, so an attempt counted on the old pose would end in GOAL_FAILED, not COMM_LOST."""
    run = Run()
    run.run_until(lambda: run.view()["attempts"] == 2, 20.0, nav_aborts)
    run.run_until(lambda: len(run.sent) == 3, 20.0)
    goal = run.sent[-1][1]
    run.core.on_move_result(goal.robot, goal.goal_id, True, "")  # and no report from now on
    for _ in range(round((run.core.params.comm_timeout_s + 0.5) / 0.1)):
        run.now[0] = round(run.now[0] + 0.1, 6)
        out = run.core.tick()
        run.events += cmds(out, "EventOut")
        assert not cmds(out, "MoveCmd"), "nothing goes out while the pose is awaited"
    assert run.view()["state"] == "COMM_LOST"
    assert run.view()["attempts"] == 2
    assert "GOAL_FAILED" not in run.codes()


def test_a_report_after_the_success_still_judges_the_arrival_when_the_window_is_over():
    """Waiting for a report must not make a robot that stopped short look arrived."""
    run = Run()
    run.tick(says_success_without_moving)
    run.run(SETTLE_S + 0.1)  # reports keep coming from the old place
    assert run.view()["attempts"] == 1
    assert "NOT_AT_TARGET" in [e.detail.split(": ")[-1] for e in run.events]


def test_on_the_default_map_a_robot_counted_as_arrived_can_release_the_zone_behind_it():
    """A whole tolerance short on the shortest lane still has to leave s > d."""
    margin = load_lane_map().shortest_exit_lane() - load_params().release_distance_m
    assert TOLERANCE_M < margin


def test_the_attempts_start_over_after_a_real_arrival():
    run = Run()
    run.tick(says_success_without_moving)
    run.run(SETTLE_S + 0.1)
    assert run.view()["attempts"] == 1
    run.run(WAIT_S)
    arrive(run, run.sent[-1][1])
    assert run.view()["attempts"] == 0


def test_a_success_while_the_robot_is_silent_is_not_checked_against_the_old_pose():
    """The pose is stale then, and recovery judges the position anyway."""
    run = Run()
    run.tick()
    goal = run.sent[0][1]
    run.now[0] += run.core.params.comm_timeout_s + 0.1
    run.core.tick()
    assert run.view()["state"] == "COMM_LOST"
    run.core.on_move_result(ROBOT, goal.goal_id, True, "")
    assert run.view()["attempts"] == 0
    assert run.core.traffic._missions[ROBOT].goal is None
    assert "NOT_AT_TARGET" not in str(run.core.tick())


def test_a_dock_result_is_not_checked_against_the_pose():
    run = Run(start="Q")
    run.run_until(lambda: run.times("DockCmd"), 30.0, to_the_dock(lambda *_: None))
    x, y = run.core.site.waypoint_pose("W")
    run.pose = (x + 0.5, y)
    run.report()
    run.core.on_dock_result(ROBOT, run.sent[-1][1].goal_id, True)
    run.tick()
    assert "GOAL_RETRY" not in run.codes()
    assert run.view()["state"] == "FILLING"
