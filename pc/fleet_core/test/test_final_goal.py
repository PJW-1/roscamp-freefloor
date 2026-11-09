"""버전 2 4절 horizon 최소판(제안, 팀 합의 전, 계약 0.8-draft MoveTo.final_goal).

관제는 이동 목표마다 그 끝이 임무의 마지막 지점인지 알린다. 잠금 경계에서 끝나는 목표는
final_goal이 거짓이라 로봇이 끝에서 정밀 접근 대신 설 수 있는 속도로만 줄인다.
"""
import json
from dataclasses import replace

from fleet_core.common.commands import MoveCmd
from test_lane_path import make, moves, place, send


def test_a_move_cmd_is_final_unless_told_otherwise():
    assert MoveCmd("r1", 1, "A", 1.0, 2.0, 0.5).final_goal is True


def test_only_the_move_to_the_last_point_of_the_mission_is_final():
    core, now = make({"r1": "W"})
    send(core, "r1", "OUTBOUND")
    m = core._missions["r1"]
    seen = []
    for _ in range(40):
        out = core.tick()
        now[0] += 0.1
        for move in moves(out):
            seen.append((move.target_id, move.final_goal))
            place(core, "r1", m.path[m.goal.target_idx], state="IDLE")
            core.on_move_result("r1", move.goal_id, True, "")
        if seen and seen[-1][1]:
            break
    last = core.site.waypoint_id(m.path[-1])
    assert seen[-1] == (last, True)
    assert seen[0][1] is False, "잠금 경계에서 끝나는 첫 목표는 마지막이 아니다"
    assert all(final is (target == last) for target, final in seen)


def test_a_record_without_the_flag_replays_with_every_move_final():
    """horizon 1 Codex 검수 4: 이 값이 없던 기록은 모든 이동이 마지막 표시(옛 동작)다."""
    core, now = make({"r1": "W"})
    core.params = replace(core.params, mark_final_goal=False)
    send(core, "r1", "OUTBOUND")
    assert all(move.final_goal for move in moves(core.tick()))


def test_a_final_move_is_written_like_a_move_from_before_the_flag():
    from fleet_core.manager import FleetManager
    lines = []
    manager = FleetManager.__new__(FleetManager)
    manager._write, manager._time = lines.append, 1.0
    manager._note("out", MoveCmd("r1", 1, "A", 1.0, 2.0, 0.5))
    manager._note("out", MoveCmd("r1", 2, "A", 1.0, 2.0, 0.5, (), False))
    assert "final_goal" not in json.loads(lines[0])
    assert json.loads(lines[1])["final_goal"] is False
