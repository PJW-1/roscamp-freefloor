"""로봇의 자가 복구 상태 RECOVERING(제안, docs/핑키_계약.md 4절).

관제는 MOVING처럼 본다. 다만 막힘 시계와 멈춤 감시는 그동안 쉬고, 레인 이탈 감시는 그대로다.
이 RECOVERING은 로봇 상태다. 관제가 로봇을 확인하는 fault RECOVERING과 다르다.
"""
from core_helpers import cmds
from fleet_core.common.params import load_params
from fleet_core.manager import FleetManager
from fleet_core.traffic.lane_map import load_lane_map
from fleet_core.traffic.mission import SELF_RECOVERING
from test_lane_blocked import CONFIRM, MARK_S, PARAMS, REPO, Pilot


def on_the_way(pilot):
    """pinky1을 P-6→N 레인 0.4 m에 두어 N, S를 쥐게 한다. MOVING으로 보고 중이다."""
    pilot.run(0.3)
    assert pilot.sent("MoveCmd")[0][1].target_id == "N"
    pilot.put("pinky1", "P-6", "N", 0.4)
    pilot.run(1.0)
    assert pilot.held() == {"N", "S"}
    return pilot.mission().goal.id


def test_the_state_value_is_the_message_constant():
    msg = (REPO / "interfaces/fleet_interfaces/msg/RobotState.msg").read_text()
    assert SELF_RECOVERING == "RECOVERING"
    assert 'string STATE_RECOVERING="RECOVERING"' in msg


# ---- 막힘 시계 ---------------------------------------------------------------------------


def test_recovering_adds_no_block_time_and_does_not_release_the_block():
    pilot = Pilot()
    on_the_way(pilot)
    pilot.state["pinky1"] = "BLOCKED"
    pilot.run(2.0)
    total = pilot.mission().block.total
    pilot.state["pinky1"] = SELF_RECOVERING
    pilot.run(5.0)
    block = pilot.mission().block
    assert block is not None, "RECOVERING은 풀림 확인 보고가 아니다"
    assert block.total == total
    assert not pilot.events("LANE_CLEARED")
    pilot.state["pinky1"] = "BLOCKED"
    start = pilot.now[0]
    pilot.run(MARK_S)
    marks = pilot.events("LANE_BLOCKED")
    assert len(marks) == 1
    assert marks[0][0] <= start + MARK_S - total + 0.2  # 앞의 BLOCKED 시간에 이어 센다


def test_three_moving_reports_after_recovering_still_release_the_block():
    pilot = Pilot()
    on_the_way(pilot)
    pilot.state["pinky1"] = "BLOCKED"
    pilot.run(1.0)
    pilot.state["pinky1"] = SELF_RECOVERING
    pilot.run(1.0)
    pilot.state["pinky1"] = "MOVING"
    pilot.run(0.1 * CONFIRM)
    assert pilot.mission().block is None


def test_a_long_recovering_never_marks_cancels_stalls_or_fails():
    pilot = Pilot()
    goal = on_the_way(pilot)
    start = pilot.now[0]
    pilot.state["pinky1"] = SELF_RECOVERING  # 그 자리에 서 있다
    pilot.run(PARAMS.blocked_timeout_s + 10.0)
    for code in ("LANE_BLOCKED", "BLOCKED_TOO_LONG", "GOAL_STALLED", "GOAL_RETRY"):
        assert not pilot.events(code), code
    assert not [t for t, _ in pilot.sent("CancelCmd") if t >= start]
    assert pilot.mission().goal.id == goal
    assert pilot.mission().block is None
    assert pilot.status()["fault"] == "NONE"


# ---- 멈춤 감시 ---------------------------------------------------------------------------


def test_the_stall_window_starts_again_when_the_robot_leaves_recovering():
    pilot = Pilot()
    on_the_way(pilot)
    pilot.run(PARAMS.stall_s - 1.0)  # MOVING인데 제자리. 멈춤 시계가 돈다
    pilot.state["pinky1"] = SELF_RECOVERING
    pilot.run(4.0)
    assert not pilot.events("GOAL_STALLED")
    pilot.state["pinky1"] = "MOVING"
    left = pilot.now[0]
    pilot.run(PARAMS.stall_s + 1.0)
    stalls = pilot.events("GOAL_STALLED")
    assert len(stalls) == 1
    assert stalls[0][0] >= left + PARAMS.stall_s - 0.2


# ---- 레인 이탈은 그대로 ----------------------------------------------------------------------


def test_the_lane_error_watch_still_runs_while_recovering():
    pilot = Pilot()
    on_the_way(pilot)
    lane = pilot.site.lane("P-6", "N")
    e = PARAMS.lane_error_warn_m + 0.05
    x, y = pilot.pose["pinky1"]
    pilot.pose["pinky1"] = (x - e * lane.uy, y + e * lane.ux)
    pilot.state["pinky1"] = SELF_RECOVERING
    pilot.run(1.0)
    assert len(pilot.events("LANE_ERROR")) == 1
    assert pilot.status()["fault"] == "RECOVERING"  # 관제가 확인하는 fault, 로봇 상태와 다르다
    assert pilot.status()["fault_reason"] == "OFF_LANE"
    assert pilot.status()["fsm_state"] == SELF_RECOVERING


# ---- 이벤트와 /fleet/robots --------------------------------------------------------------------


def test_entering_and_leaving_recovering_are_said_once_each_with_time_and_next_state():
    pilot = Pilot()
    on_the_way(pilot)
    pilot.state["pinky1"] = SELF_RECOVERING
    entered = pilot.now[0]
    pilot.run(3.0)
    starts = pilot.events("SELF_RECOVERY")
    assert len(starts) == 1
    assert starts[0][0] == entered
    assert starts[0][1].level == "INFO" and starts[0][1].robot == "pinky1"
    assert not pilot.events("SELF_RECOVERY_END")
    status = pilot.status()
    assert status["fsm_state"] == SELF_RECOVERING
    assert (status["fault"], status["fault_reason"]) == ("NONE", "")
    pilot.state["pinky1"] = "BLOCKED"
    pilot.run(1.0)
    ends = pilot.events("SELF_RECOVERY_END")
    assert len(ends) == 1
    assert ends[0][1].level == "INFO" and ends[0][1].robot == "pinky1"
    assert "3.0 s" in ends[0][1].detail and "BLOCKED" in ends[0][1].detail
    assert len(pilot.events("SELF_RECOVERY")) == 1


def test_a_robot_that_leaves_recovering_for_localizing_says_so():
    pilot = Pilot()
    on_the_way(pilot)
    pilot.state["pinky1"] = SELF_RECOVERING
    pilot.run(1.0)
    pilot.state["pinky1"] = "LOCALIZING"
    pilot.run(0.5)
    ends = pilot.events("SELF_RECOVERY_END")
    assert len(ends) == 1 and "LOCALIZING" in ends[0][1].detail


# ---- 입력 ------------------------------------------------------------------------------------


def test_recovering_is_taken_as_input_but_a_first_report_of_it_does_not_place_the_robot():
    now = [0.0]
    core = FleetManager(load_lane_map(), load_params(), ["pinky1"], lambda: now[0])
    x, y = core.site.waypoint_pose("P-6")
    core.on_robot_state("pinky1", x, y, 0.0, SELF_RECOVERING)
    out = core.tick()
    assert not [e for e in cmds(out, "EventOut") if e.code == "INPUT_REJECTED"]
    assert core.traffic.status_view("pinky1")["fsm_state"] == SELF_RECOVERING
    assert core.traffic.robot_view("pinky1")["on_map"] is False
