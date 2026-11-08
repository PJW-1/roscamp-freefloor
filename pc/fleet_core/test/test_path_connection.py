"""경로 연결 확인(지시서 7.5), 관제 쪽.

핑키는 받은 path가 자기 자리에서 lane_error_warn_m보다 멀면 따라가지 않고 PATH_NOT_CONNECTED로
실패한다. 관제는 다른 이동 실패처럼 한 번으로 세고, move_retry_wait_s 뒤 지금 보고 위치에서
path를 다시 만든다. path의 첫 점이 로봇의 보고 위치라서 정상 운행에서는 이 실패가 나지 않는다.
SimWorld의 로봇도 같은 검사를 한다.
"""
import math
from dataclasses import replace
from pathlib import Path

import pytest
from core_helpers import cmds
from fleet_core.common.commands import MoveCmd, OrderRecord
from fleet_core.common.params import load_params
from fleet_core.manager import FleetManager
from fleet_core.traffic.lane_map import load_lane_map
from sim_world import SimWorld, path_gap

REPO = Path(__file__).resolve().parents[3]
FEATURE = REPO / "sim/fake_robots/maps/feature_map.yaml"
PARAMS = load_params()
WAIT_S = PARAMS.move_retry_wait_s
NOT_CONNECTED = "PATH_NOT_CONNECTED"
ROBOT = "pinky1"


class OneRobot:
    """로봇 하나, 0.1초 틱. 받은 이동 목표에 그 틱 안에 답한다."""

    def __init__(self, answer):
        self.now = [0.0]
        self.core = FleetManager(load_lane_map(), PARAMS, [ROBOT], lambda: self.now[0])
        self.pose = self.core.site.waypoint_pose("P-6")
        self.answer = answer
        self.moves = []  # (시각, MoveCmd, 그 틱에 보고한 자리)
        self.docks = []
        self.events = []
        self.report()
        self.core.on_order_board("RUN", [OrderRecord("O-001", {"A": 1})])

    def report(self):
        self.core.on_robot_state(ROBOT, *self.pose, 0.0, "IDLE")

    def run(self, seconds):
        for _ in range(round(seconds / 0.1)):
            self.report()
            out = self.core.tick()
            self.events += cmds(out, "EventOut")
            self.docks += cmds(out, "DockCmd")
            for move in cmds(out, "MoveCmd"):
                self.moves.append((self.now[0], move, self.pose))
                self.answer(self, move)
            self.now[0] = round(self.now[0] + 0.1, 6)


def refuse_the_first_then_arrive(run, move):
    """첫 목표는 연결이 안 됐다고 거절한다. 그 사이 로봇은 첫 레인을 따라 5 cm 움직였다."""
    if len(run.moves) == 1:
        (x0, y0), (x1, y1) = move.path[0][:2], move.path[1][:2]
        d = math.dist((x0, y0), (x1, y1))
        run.pose = (x0 + 0.05 * (x1 - x0) / d, y0 + 0.05 * (y1 - y0) / d)
        run.core.on_move_result(ROBOT, move.goal_id, False, NOT_CONNECTED)
    else:
        run.pose = (move.x, move.y)
        run.core.on_move_result(ROBOT, move.goal_id, True, "")


def test_a_path_not_connected_is_one_failed_attempt_and_the_retry_starts_where_the_robot_is():
    run = OneRobot(refuse_the_first_then_arrive)
    run.run(30.0)
    (t1, first, _), (t2, second, reported) = run.moves[:2]
    assert first.path, "the default lane_goal_mode is path"
    assert WAIT_S <= round(t2 - t1, 6) <= WAIT_S + 0.2, "it waits move_retry_wait_s, no more"
    assert (second.target_id, second.x, second.y) == (first.target_id, first.x, first.y)
    assert second.path[0][:2] == reported, "the new path starts at the reported position"
    assert reported != first.path[0][:2], "and that is where the robot is now"
    retries = [e for e in run.events if e.code == "GOAL_RETRY"]
    assert [e.detail for e in retries] == [f"attempt 1 failed: {NOT_CONNECTED}"]
    assert not [e for e in run.events if e.level == "ERROR"]
    assert run.docks, "the retried path got the robot on to the dock"


def bad_path_world(**kw):
    return SimWorld(seed=5, faults=[
        {"at_s": 3.0, "type": "bad_path", "robot": ROBOT, "offset_m": 0.4}], **kw)


@pytest.mark.parametrize("site", [None, "lane_map"])
def test_a_path_that_misses_the_robot_is_refused_once_and_every_order_still_ships(site):
    """일부러 떨어진 path를 한 번 받게 한다: PATH_NOT_CONNECTED, 재시도, 정상 path로 도착."""
    world = bad_path_world(site=load_lane_map() if site else None)
    world.run_until(world.done)
    assert [robot for _, robot, _ in world.path_refusals] == [ROBOT]
    refused_at = world.path_refusals[0][0]
    retries = [e for t, e in world.event_log if e.code == "GOAL_RETRY" and e.robot == ROBOT
               and NOT_CONNECTED in e.detail]
    assert len(retries) == 1
    later = [t for t, c in world.commands
             if isinstance(c, MoveCmd) and c.robot == ROBOT and t > refused_at]
    assert later and later[0] - refused_at >= WAIT_S - 0.1 - 1e-9
    assert not world.event_codes("ERROR")
    assert set(world.report_statuses().values()) == {"SHIPPED"}


SITES = {"grid": lambda: None, "lane_map": load_lane_map,
         "feature": lambda: load_lane_map(FEATURE)}
FEATURE_ROBOTS = {"pinky1": {"start": "P-3", "speed_mps": 0.20, "pose_noise_m": 0.03},
                  "pinky2": {"start": "P-2", "speed_mps": 0.18, "pose_noise_m": 0.03},
                  "pinky3": {"start": "P-1", "speed_mps": 0.20, "pose_noise_m": 0.03}}


@pytest.mark.parametrize("seed", [1, 2])
@pytest.mark.parametrize("mode", ["path", "point"])
@pytest.mark.parametrize("site", sorted(SITES))
def test_a_normal_run_never_sends_a_path_that_misses_the_robot(site, mode, seed):
    extra = {"robots": FEATURE_ROBOTS} if site == "feature" else {}
    world = SimWorld(seed=seed, site=SITES[site](), params=replace(PARAMS, lane_goal_mode=mode),
                     **extra)
    world.run_until(world.done, timeout_s=900)
    assert world.path_refusals == []
    assert not [e for _, e in world.event_log if NOT_CONNECTED in e.detail]
    sent = [c for _, c in world.commands if isinstance(c, MoveCmd)]
    assert sent and all(bool(c.path) == (mode == "path") for c in sent)


def test_a_fleet_path_that_misses_the_robot_breaks_the_world_invariant():
    """SimWorld는 보고가 바로 가는 세계라 첫 점(보고 위치)은 로봇에서 잡음만큼만 떨어진다.
    주입하지 않은 path를 거절하면 관제 결함이므로 재시도로 가리지 않고 멈춘다(Codex)."""
    world = SimWorld(seed=1)
    r = world.virtual[ROBOT]
    far = MoveCmd(ROBOT, 99, "x", r.x + 0.9, r.y + 0.5, 0.0,
                  path=((r.x + 0.5, r.y + 0.5, 0.0), (r.x + 0.9, r.y + 0.5, 0.0)))
    with pytest.raises(AssertionError, match="does not pass the robot"):
        world._refuses_path(r, far)
    assert world.path_refusals == []


def test_an_injected_bad_path_is_a_refusal_not_an_invariant_break():
    world = SimWorld(seed=1)
    r = world.virtual[ROBOT]
    r.bad_paths, r.bad_path_offset = 1, 0.4
    near = MoveCmd(ROBOT, 99, "x", r.x + 0.3, r.y, 0.0,
                   path=((r.x, r.y, 0.0), (r.x + 0.3, r.y, 0.0)))
    assert world._refuses_path(r, near) is True
    assert [(robot, goal) for _, robot, goal in world.path_refusals] == [(ROBOT, 99)]
    assert r.bad_paths == 0 and world._refuses_path(r, near) is False


def test_path_gap_measures_the_robot_against_the_path_it_got():
    assert path_gap(0.5, 0.1, [(0.0, 0.0), (1.0, 0.0)]) == pytest.approx(0.1)
    assert path_gap(1.0, 0.4, [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]) == pytest.approx(0.0)
    assert path_gap(0.0, 0.3, [(0.0, 0.0)]) == pytest.approx(0.3)
