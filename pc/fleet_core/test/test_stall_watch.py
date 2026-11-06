"""멈춤 감시: 목표가 열린 채 위치가 그대로면 취소하고 같은 목표를 다시 보낸다(지시서 2번).

기본 지도에서 로봇 하나, 0.1초 틱. test_goal_retry.py와 같은 방식이다.
"""
import dataclasses
import math
import random
import sys
from pathlib import Path

import pytest
import yaml
from core_helpers import cmds
from fleet_core.common.commands import OrderRecord
from fleet_core.common.params import load_params
from fleet_core.manager import FleetManager
from fleet_core.traffic.controller import TrafficController
from fleet_core.traffic.lane_map import LaneMap, load_lane_map

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))

import replay  # noqa: E402

ROBOT = "pinky1"
ORDER = OrderRecord("O-001", {"A": 1})
PARAMS = load_params()
STALL_S = PARAMS.stall_s
DOCK_STALL_S = PARAMS.dock_stall_s
WAIT_S = PARAMS.move_retry_wait_s


class Run:
    def __init__(self, start="P-6", write=None):
        self.now = [0.0]
        self.core = FleetManager(load_lane_map(), load_params(), [ROBOT], lambda: self.now[0])
        if write:
            self.core.record_to(write)
        self.pose = self.core.site.waypoint_pose(start)
        self.silent = False
        self.motion = None  # 틱마다 위치를 바꾸는 함수
        self.out = []  # (시간, 명령)
        self.answered = set()  # 결과를 돌려준 goal 번호
        self.report()
        self.core.on_order_board("RUN", [ORDER])

    def report(self):
        if not self.silent:
            self.core.on_robot_state(ROBOT, *self.pose, 0.0, "IDLE")

    def tick(self, answer=None):
        if self.motion:
            self.motion(self)
        self.report()
        for command in self.core.tick():
            self.out.append((self.now[0], command))
            if answer and type(command).__name__ in ("MoveCmd", "DockCmd"):
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

    def sent(self, kind):
        return [(t, c) for t, c in self.out if type(c).__name__ == kind]

    def events(self, code):
        return [(t, c) for t, c in self.out
                if type(c).__name__ == "EventOut" and c.code == code]


def arrive(run, command):
    if type(command).__name__ == "MoveCmd":
        run.pose = (command.x, command.y)
        run.report()
        run.core.on_move_result(command.robot, command.goal_id, True, "")


def honest(run, command):
    if type(command).__name__ == "MoveCmd":
        arrive(run, command)
    else:
        run.core.on_dock_result(command.robot, command.goal_id, True)


# ---- 값 ------------------------------------------------------------------------------------


def test_the_values():
    # stall_move_m은 지시서의 0.02가 아니라 0.05다. 위치 잡음 때문이다(아래 잡음 시험).
    assert (STALL_S, PARAMS.stall_move_m, PARAMS.stall_smooth_s, DOCK_STALL_S) == (
        6.0, 0.05, 1.0, 15.0)


def test_a_silent_robot_shows_comm_lost_before_a_stall():
    assert PARAMS.comm_timeout_s < STALL_S < DOCK_STALL_S


# ---- 멈춤 → 취소 → 같은 목표 다시 ---------------------------------------------------------


def test_a_goal_with_no_movement_is_canceled_and_sent_again_to_the_same_waypoint():
    run = Run()
    run.run(STALL_S + WAIT_S + 1.0)
    first_t, first = run.sent("MoveCmd")[0]
    stalls = run.events("GOAL_STALLED")
    assert len(stalls) == 1
    stall_t, event = stalls[0]
    assert first_t + STALL_S <= stall_t <= first_t + STALL_S + 0.2
    assert event.level == "WARN" and event.robot == ROBOT
    cancels = run.sent("CancelCmd")
    assert [(t, c.goal_id) for t, c in cancels] == [(stall_t, first.goal_id)]
    again_t, again = run.sent("MoveCmd")[1]
    assert again.target_id == first.target_id
    assert again_t >= stall_t + WAIT_S
    assert run.view()["attempts"] == 1
    assert not run.events("GOAL_RETRY")


def test_three_stalls_end_in_error_and_the_robot_keeps_its_zones():
    run = Run()
    run.run_until(lambda: run.view()["state"] == "ERROR", 40.0)
    run.run(STALL_S + WAIT_S)
    assert len(run.events("GOAL_STALLED")) == 3
    assert len(run.events("GOAL_FAILED")) == 1
    assert run.events("GOAL_FAILED")[0][1].level == "ERROR"
    assert len(run.sent("MoveCmd")) == 3
    assert len(run.sent("CancelCmd")) == 3
    assert run.core.traffic.status_view(ROBOT)["fault_reason"] == "STALLED"
    assert run.core.traffic.zones.held_by(ROBOT)


def test_a_late_canceled_result_of_the_stalled_goal_costs_nothing_more():
    run = Run()
    run.run(STALL_S + 0.3)
    stalled = run.sent("MoveCmd")[0][1]
    run.core.on_move_result(ROBOT, stalled.goal_id, False, "CANCELED")
    run.tick()
    assert run.view()["attempts"] == 1


def test_a_robot_that_moves_again_after_the_resend_starts_over():
    run = Run()
    run.run_until(lambda: len(run.sent("MoveCmd")) == 2, STALL_S + WAIT_S + 1.0)
    arrive(run, run.sent("MoveCmd")[-1][1])
    run.run_until(lambda: run.view()["state"] == "FILLING", 60.0, honest)
    assert len(run.events("GOAL_STALLED")) == 1
    assert not run.events("GOAL_FAILED")


# ---- 멈춤이 아닌 것 ------------------------------------------------------------------------


def _creep(speed_mps):
    """목표 쪽으로 천천히 간다."""
    def motion(run):
        goals = run.sent("MoveCmd")
        if not goals:
            return
        goal = goals[-1][1]
        dx, dy = goal.x - run.pose[0], goal.y - run.pose[1]
        dist = math.hypot(dx, dy)
        step = min(dist, speed_mps * 0.1)
        if dist > 0:
            run.pose = (run.pose[0] + dx / dist * step, run.pose[1] + dy / dist * step)
    return motion


def test_slow_but_steady_movement_is_not_a_stall():
    run = Run()
    run.motion = _creep(0.02)  # 6초에 0.12 m
    run.run(30.0)
    assert not run.events("GOAL_STALLED")


def test_jitter_in_place_is_a_stall():
    run = Run()
    x, y = run.pose
    def jitter(run):
        sign = 1 if round(run.now[0] * 10) % 2 else -1
        run.pose = (x + sign * 0.009, y)

    run.motion = jitter
    run.run(STALL_S + 0.5)
    assert len(run.events("GOAL_STALLED")) == 1


# ---- 위치 잡음 -----------------------------------------------------------------------------

NOISE_M = 0.03  # 시나리오의 pose_noise_m과 같은 σ
SEEDS = range(20)


def _noisy(sigma, seed, motion=None):
    """보고 위치에 가우시안 잡음을 더한다. motion은 참 위치를 옮긴다."""
    rng = random.Random(seed)

    def noisy(run):
        if motion:
            run.true = motion(run, run.true)
        run.pose = (run.true[0] + rng.gauss(0.0, sigma), run.true[1] + rng.gauss(0.0, sigma))
    return noisy


def _toward_goal(speed_mps):
    def motion(run, true):
        goals = run.sent("MoveCmd")
        if not goals:
            return true
        goal = goals[-1][1]
        dx, dy = goal.x - true[0], goal.y - true[1]
        dist = math.hypot(dx, dy)
        if dist == 0:  # 닿으면 성공을 돌려준다
            if goal.goal_id not in run.answered:
                run.answered.add(goal.goal_id)
                run.core.on_move_result(ROBOT, goal.goal_id, True, "")
            return true
        step = min(dist, speed_mps * 0.1)
        return (true[0] + dx / dist * step, true[1] + dy / dist * step)
    return motion


def _dock_ok(run, command):
    if type(command).__name__ == "DockCmd":
        run.core.on_dock_result(command.robot, command.goal_id, True)


@pytest.mark.parametrize("seed", SEEDS)
def test_a_robot_standing_still_with_pose_noise_is_still_a_stall(seed):
    run = Run()
    run.true = run.pose
    run.motion = _noisy(NOISE_M, seed)
    run.run_until(lambda: run.view()["state"] == "ERROR", 3 * (STALL_S + WAIT_S) + 6.0)
    assert len(run.events("GOAL_STALLED")) == 3


@pytest.mark.parametrize("seed", SEEDS)
def test_a_moving_robot_with_pose_noise_is_never_a_stall(seed):
    run = Run()
    run.true = run.pose
    run.motion = _noisy(NOISE_M, seed, _toward_goal(0.2))
    run.run(30.0, _dock_ok)
    assert not run.events("GOAL_STALLED")


@pytest.mark.parametrize("seed", SEEDS)
def test_a_slow_robot_with_pose_noise_is_not_a_stall(seed):
    run = Run()
    run.true = run.pose
    run.motion = _noisy(NOISE_M, seed, _toward_goal(0.03))
    run.run(30.0, _dock_ok)
    assert not run.events("GOAL_STALLED")


def test_standing_with_no_goal_while_waiting_for_a_zone_is_not_a_stall():
    now = [0.0]
    site = LaneMap(yaml.safe_load(STRAIGHT_MAP), STRAIGHT_MAP)
    core = TrafficController(site, load_params(), ["r1", "r2"], lambda: now[0])
    for robot, waypoint in (("r1", "W"), ("r2", "A")):
        core.on_robot_state(robot, *site.waypoint_pose(waypoint), 0.0, "IDLE")
    core.on_run_mode("RUN")
    core._missions["r2"].fault = "ERROR"  # 화이트박스: r2가 A를 쥔 채 서 있다
    core.on_dispatch(1, "OUTBOUND", "r1")
    out = []
    for _ in range(300):
        for robot, waypoint in (("r1", "W"), ("r2", "A")):
            core.on_robot_state(robot, *site.waypoint_pose(waypoint), 0.0, "IDLE")
        out += core.tick()
        now[0] = round(now[0] + 0.1, 6)
    assert core.zones.holder("A") == "r2"
    assert not cmds(out, "MoveCmd")
    assert "GOAL_STALLED" not in [e.code for e in cmds(out, "EventOut")]


def test_a_robot_at_the_station_with_no_goal_is_not_a_stall():
    run = Run()
    run.run_until(lambda: run.view()["state"] == "FILLING", 60.0, honest)
    run.run(30.0)
    assert not run.events("GOAL_STALLED")


def test_pause_is_not_a_stall_and_the_clock_starts_again_on_run():
    run = Run()
    run.run(STALL_S - 1.0)
    run.core.on_order_board("PAUSE", [ORDER])
    run.run(3 * STALL_S)
    assert not run.events("GOAL_STALLED")
    run.core.on_order_board("RUN", [ORDER])
    resumed = run.now[0]
    run.run(STALL_S + 0.5)
    stalls = run.events("GOAL_STALLED")
    assert len(stalls) == 1
    assert stalls[0][0] >= resumed + STALL_S


def test_pause_and_run_between_two_ticks_still_restart_the_clock():
    run = Run()
    run.run(STALL_S - 1.0)
    run.core.on_order_board("PAUSE", [ORDER])
    run.core.on_order_board("RUN", [ORDER])
    resumed = run.now[0]
    run.run(STALL_S + 0.5)
    stalls = run.events("GOAL_STALLED")
    assert len(stalls) == 1
    assert stalls[0][0] >= resumed + STALL_S


def test_no_smoothing_does_not_divide_by_zero():
    run = Run()
    run.core.traffic.params = dataclasses.replace(run.core.traffic.params, stall_smooth_s=0.0)
    run.run(STALL_S + 0.5)
    assert len(run.events("GOAL_STALLED")) == 1


def test_a_silent_robot_is_left_to_the_comm_check():
    run = Run()
    run.tick()
    run.silent = True
    run.run(3 * STALL_S)
    assert run.events("COMM_LOST")
    assert not run.events("GOAL_STALLED")


# ---- 도킹 ----------------------------------------------------------------------------------


def test_a_dock_gets_its_longer_time():
    run = Run()
    run.run_until(lambda: run.sent("DockCmd"), 60.0, arrive)
    dock_t, dock = run.sent("DockCmd")[0]
    run.run(STALL_S + 1.0, arrive)
    assert not run.events("GOAL_STALLED"), "도킹은 6초로 재지 않는다"
    # arrive는 로봇을 순간 이동시켜 매끄럽게 한 위치가 따라오는 몇 초 뒤부터 잰다.
    settle = 4 * PARAMS.stall_smooth_s
    run.run(DOCK_STALL_S - STALL_S + WAIT_S + settle, arrive)
    stalls = run.events("GOAL_STALLED")
    assert len(stalls) == 1
    assert dock_t + DOCK_STALL_S <= stalls[0][0] <= dock_t + DOCK_STALL_S + settle
    assert [c.goal_id for _, c in run.sent("CancelCmd")] == [dock.goal_id]
    assert len(run.sent("DockCmd")) == 2


# ---- 기록과 재생 ---------------------------------------------------------------------------


def test_a_record_with_a_stall_plays_back(tmp_path):
    lines = []
    run = Run(write=lines.append)
    run.run_until(lambda: len(run.sent("MoveCmd")) == 2, STALL_S + WAIT_S + 1.0)
    arrive(run, run.sent("MoveCmd")[-1][1])
    run.run_until(lambda: run.view()["state"] == "FILLING", 60.0, honest)
    assert run.events("GOAL_STALLED")
    path = tmp_path / "run.jsonl"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    result = replay.replay(path)
    assert result.compared and result.same, (result.line, result.recorded, result.produced)


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
