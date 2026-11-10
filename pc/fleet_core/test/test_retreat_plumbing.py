"""Retreat(물러나기) 배선: 교통 층이 낸 RetreatCmd는 MoveCmd처럼 나가고, 결과는 교통 층에 간다.

아무도 RetreatCmd를 내지 않으면 어느 모드도 전과 같다. 물러나기 판단은 교통 층의 일이다.
"""
from dataclasses import asdict, replace

import pytest
import yaml
from fleet_core.common.commands import EventOut, RetreatCmd
from fleet_core.common.params import DEFAULT_PARAMS, FleetParams, load_params
from fleet_core.manager import FleetManager
from fleet_core.traffic.controller import TrafficController
from fleet_core.traffic.lane_map import load_lane_map
from fleet_core.traffic.mission import RETREAT, _Goal

ROBOTS = ["pinky1", "pinky2", "pinky3"]
MODES = ["lanes", "freefloor"]


def manager(mode):
    params = replace(load_params(), traffic_mode=mode)
    core = FleetManager(load_lane_map(), params, ROBOTS, lambda: 0.0)
    for robot, zone in zip(ROBOTS, ("P-6", "P-5", "P-4"), strict=True):
        x, y = core.site.waypoint_pose(min(core.site.zones[zone]))
        for _ in range(params.release_confirm_samples):
            core.on_robot_state(robot, x, y, 0.0, "IDLE")
    core.tick()
    return core


@pytest.mark.parametrize("mode", MODES)
def test_a_retreat_the_traffic_layer_emits_goes_out_like_a_move(mode):
    core = manager(mode)
    cmd = RetreatCmd("pinky1", 77, 0.3, 0.05, ((1.0, 0.5, 0.0), (0.8, 0.5, 0.0)))
    core.traffic._out.append(cmd)
    assert cmd in core.tick()


@pytest.mark.parametrize("mode", MODES)
def test_a_retreat_result_reaches_the_traffic_layer(mode, monkeypatch):
    core = manager(mode)
    seen = []
    monkeypatch.setattr(core.traffic, "on_retreat_result", lambda *a: seen.append(a))
    core.on_retreat_result("pinky1", 77, False, "OBSTACLE_BEHIND")
    assert seen == [("pinky1", 77, False, "OBSTACLE_BEHIND")]


def test_a_retreat_result_of_an_unknown_robot_is_refused():
    core = manager("lanes")
    core.on_retreat_result("ghost", 77, True, "")
    out = core.tick()
    assert [e.code for e in out if isinstance(e, EventOut)] == ["INPUT_REJECTED"]


@pytest.mark.parametrize("mode", MODES)
def test_the_default_handler_does_nothing(mode):
    core = manager(mode)
    before = core.robot_view("pinky1")
    core.on_retreat_result("pinky1", 77, True, "")
    assert core.tick() == []
    assert core.robot_view("pinky1") == before


def test_the_base_controller_has_the_default():
    assert "on_retreat_result" in vars(TrafficController)


def retreating(core, robot="pinky1", goal_id=900):
    m = core.traffic._missions[robot]
    m.goal = _Goal(RETREAT, goal_id)
    return m


@pytest.mark.parametrize("mode", MODES)
def test_the_move_a_retreat_replaced_ends_canceled_without_a_failure(mode):
    """Retreat는 달리던 이동을 먼저 세운다. 그 MoveTo의 CANCELED는 실패로 세지 않는다."""
    core = manager(mode)
    m = retreating(core)
    core.on_move_result("pinky1", 5, False, "CANCELED")  # the move the retreat replaced
    out = core.tick()
    assert m.goal == _Goal(RETREAT, 900)
    assert m.attempts == 0
    assert not [e for e in out if isinstance(e, EventOut) and e.level != "INFO"]


@pytest.mark.parametrize("mode", MODES)
@pytest.mark.parametrize(("call", "reason"), [("on_goal_rejected", "REJECTED"),
                                               ("on_goal_not_sent", "NOT_SENT"),
                                               ("on_goal_lost", "LOST")])
def test_a_retreat_goal_that_never_ran_ends_as_a_retreat_result(mode, call, reason, monkeypatch):
    """거절, 미전송, 잃은 Retreat goal은 이동 재시도로 세지 않고 on_retreat_result로 간다."""
    core = manager(mode)
    m = retreating(core)
    seen = []
    monkeypatch.setattr(core.traffic, "on_retreat_result", lambda *a: seen.append(a))
    getattr(core, call)("pinky1", 900)
    assert seen == [("pinky1", 900, False, reason)]
    assert m.attempts == 0
    assert "GOAL_RETRY" not in [e.code for e in core.tick() if isinstance(e, EventOut)]


@pytest.mark.parametrize("mode", MODES)
def test_the_stall_watch_rests_while_a_robot_retreats(mode):
    """물러나는 로봇은 느리게(0.05 m/s) 뒤로 간다. 제자리 감시가 같은 MoveTo를 다시 내면 안 된다."""
    core = manager(mode)
    m = retreating(core)
    traffic = core.traffic
    traffic.on_run_mode("RUN")
    m.stall_from, m.stall_since = (m.x, m.y), -100.0
    traffic._watch_stall(m, 0.0)
    assert m.stall_from is None
    assert m.goal == _Goal(RETREAT, 900)


RETREAT_PARAMS = {"floor_block_retreat": False, "floor_corridor_clear": False,
                  "floor_retreat_after_s": 10.0, "floor_retreat_speed_mps": 0.05,
                  "floor_retreat_max_m": 1.0}


@pytest.mark.parametrize("name", ["fleet_params.yaml", "fleet_params_real_arena.yaml"])
def test_the_shipped_params_keep_retreat_off(name):
    """제안값. 둘 다 꺼져 있어 아무것도 바뀌지 않는다(arena_v4도 fleet_params_real_arena를 쓴다)."""
    shipped = yaml.safe_load((DEFAULT_PARAMS.parent / name).read_text(encoding="utf-8"))
    assert {k: shipped.get(k) for k in RETREAT_PARAMS} == RETREAT_PARAMS
    params = load_params(DEFAULT_PARAMS.parent / name)
    assert {k: getattr(params, k) for k in RETREAT_PARAMS} == RETREAT_PARAMS


def test_a_record_without_the_retreat_params_replays_with_them_off():
    fields = {k: v for k, v in asdict(load_params()).items() if k not in RETREAT_PARAMS}
    params = FleetParams(**fields)
    assert {k: getattr(params, k) for k in RETREAT_PARAMS} == RETREAT_PARAMS
