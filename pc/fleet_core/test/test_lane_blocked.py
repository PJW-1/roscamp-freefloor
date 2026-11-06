"""레인 막힘 1단계: 서고 알리기(지시서 4번, docs/핑키_계약.md 4절).

Pilot은 로봇을 손으로 몰아 시각을 정확히 본다. SimWorld는 두 대가 함께 달리는 흐름을 본다.
LOCALIZING과 HEARTBEAT_LOST(계약 3절 5·6번)도 여기서 본다.
"""
import sys
from pathlib import Path

import pytest
from fleet_core.common.commands import OrderRecord
from fleet_core.common.params import load_params
from fleet_core.manager import FleetManager
from fleet_core.traffic.lane_map import load_lane_map
from fleet_core.traffic.mission import MOVE, _Goal
from sim_world import SimWorld

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))

import replay  # noqa: E402

PARAMS = load_params()
ORDER = OrderRecord("O-001", {"A": 1})
MARK_S = PARAMS.blocked_mark_s if hasattr(PARAMS, "blocked_mark_s") else 5.0
CONFIRM = PARAMS.release_confirm_samples


class Pilot:
    """로봇을 손으로 몬다. 보고 위치와 상태는 시험이 정하고, 나온 명령은 기록만 한다."""

    def __init__(self, starts=None, orders=(ORDER,), write=None):
        self.starts = starts or {"pinky1": "P-6"}
        self.now = [0.0]
        self.core = FleetManager(load_lane_map(), load_params(), list(self.starts),
                                 lambda: self.now[0])
        if write:
            self.core.record_to(write)
        self.site = self.core.site
        self.pose = {r: self.site.waypoint_pose(w) for r, w in self.starts.items()}
        self.state = {r: "IDLE" for r in self.starts}
        self.silent = set()
        self.out = []
        for _ in range(CONFIRM):
            self.report()
        self.core.on_order_board("RUN", list(orders))

    def report(self):
        for robot in self.starts:
            if robot not in self.silent:
                self.core.on_robot_state(robot, *self.pose[robot], 0.0, self.state[robot])

    def tick(self):
        self.report()
        for command in self.core.tick():
            self.out.append((self.now[0], command))
        self.now[0] = round(self.now[0] + 0.1, 6)

    def run(self, seconds):
        for _ in range(round(seconds / 0.1)):
            self.tick()

    def put(self, robot, a, b, s, state="MOVING"):
        lane = self.site.lane(a, b)
        self.pose[robot] = (lane.ax + s * lane.ux, lane.ay + s * lane.uy)
        self.state[robot] = state

    def sent(self, kind, robot="pinky1"):
        return [(t, c) for t, c in self.out
                if type(c).__name__ == kind and c.robot == robot]

    def events(self, code):
        return [(t, c) for t, c in self.out
                if type(c).__name__ == "EventOut" and c.code == code]

    def mission(self, robot="pinky1"):
        return self.core.traffic._missions[robot]

    def held(self, robot="pinky1"):
        return set(self.core.zones.held_by(robot))

    def status(self, robot="pinky1"):
        return self.core.traffic.status_view(robot)


def blocked_on_the_way(pilot):
    """pinky1을 P-6→N 레인 0.4 m에 두어 N, S를 쥐게 한 뒤 BLOCKED를 보고하게 한다. 시작 시각."""
    pilot.run(0.3)
    assert pilot.sent("MoveCmd")[0][1].target_id == "N"
    pilot.put("pinky1", "P-6", "N", 0.4)
    pilot.run(1.0)
    assert pilot.held() == {"N", "S"}
    pilot.state["pinky1"] = "BLOCKED"
    return pilot.now[0]


def first_goal(pilot):
    return pilot.sent("MoveCmd")[0][1].goal_id


# ---- 값 ------------------------------------------------------------------------------------


def test_the_values():
    assert (PARAMS.blocked_mark_s, PARAMS.blocked_timeout_s, PARAMS.blocked_rejoin_s,
            PARAMS.queue_gap_m) == (5.0, 60.0, 10.0, 0.6)


def test_the_state_values_are_in_the_message_files():
    msg = (REPO / "interfaces/fleet_interfaces/msg/RobotState.msg").read_text()
    action = (REPO / "interfaces/fleet_interfaces/action/MoveTo.action").read_text()
    assert 'string STATE_BLOCKED="BLOCKED"' in msg
    assert 'string STATE_LOCALIZING="LOCALIZING"' in msg
    assert 'string REASON_HEARTBEAT_LOST="HEARTBEAT_LOST"' in action


# ---- 5초 전: 기다린다 -------------------------------------------------------------------------


def test_a_short_block_keeps_the_goal_and_sends_nothing():
    pilot = Pilot()
    start = blocked_on_the_way(pilot)
    goal = first_goal(pilot)
    pilot.run(3.0)
    assert pilot.status()["fault_reason"] == "BLOCKED"
    assert pilot.status()["fault"] == "NONE"
    pilot.state["pinky1"] = "MOVING"
    pilot.run(1.0)
    assert pilot.status()["fault_reason"] == ""
    assert not pilot.events("LANE_BLOCKED") and not pilot.events("LANE_CLEARED")
    assert not [t for t, _ in pilot.sent("CancelCmd") if t >= start]
    assert not [t for t, _ in pilot.sent("MoveCmd") if t >= start]
    assert pilot.mission().goal.id == goal
    assert pilot.held() == {"N", "S"}


def test_the_stall_clock_rests_while_blocked_and_starts_from_zero_after():
    """서 있은 지 6초가 넘어도 BLOCKED 동안은 멈춤이 아니다. 풀린 뒤 6초를 새로 잰다."""
    pilot = Pilot()
    blocked_on_the_way(pilot)
    pilot.run(4.0)
    pilot.state["pinky1"] = "MOVING"  # 풀렸지만 그 자리에 그대로 있다
    pilot.run(0.3)
    cleared = pilot.now[0]
    pilot.run(PARAMS.stall_s + 2.0)
    stalls = pilot.events("GOAL_STALLED")
    assert len(stalls) == 1
    assert stalls[0][0] >= cleared + PARAMS.stall_s - 0.2


def test_blocked_off_every_lane_is_still_watched_for_a_stall():
    """달리던 레인을 못 찾으면 막힘을 열 수 없다. 그래도 영원히 서 있게 두지 않는다."""
    pilot = Pilot(orders=())
    x, y = pilot.site.waypoint_pose("P-6")
    pilot.pose["pinky1"] = (x, y + 0.3)
    pilot.run(3.0)  # 매끄럽게 한 위치가 자리를 잡는다
    m = pilot.mission()
    m.goal = _Goal(MOVE, 99, 0)
    pilot.state["pinky1"] = "BLOCKED"
    pilot.run(PARAMS.stall_s + 0.5)
    assert m.block is None
    assert len(pilot.events("GOAL_STALLED")) == 1


def test_a_move_goal_is_needed_to_count_as_a_block():
    pilot = Pilot(orders=())
    pilot.run(0.3)
    pilot.state["pinky1"] = "BLOCKED"
    pilot.run(MARK_S + 2.0)
    assert not pilot.events("LANE_BLOCKED")
    assert pilot.mission().block is None
    assert pilot.status()["fault_reason"] == ""
    assert pilot.status()["fsm_state"] == "BLOCKED"


# ---- 5초: 표시하고 취소, 앞 구역은 취소 결과와 정지 뒤 놓는다 ----------------------------


def test_after_five_seconds_the_lane_is_marked_once_and_the_goal_canceled():
    pilot = Pilot()
    start = blocked_on_the_way(pilot)
    goal = first_goal(pilot)
    pilot.run(MARK_S + 1.0)
    marks = pilot.events("LANE_BLOCKED")
    assert len(marks) == 1
    when, event = marks[0]
    assert start + MARK_S <= when <= start + MARK_S + 0.2
    assert event.level == "WARN" and event.robot == "pinky1"
    assert "P-6" in event.detail and "N" in event.detail
    assert [(t, c.goal_id) for t, c in pilot.sent("CancelCmd")] == [(when, goal)]
    assert pilot.core.traffic.blocked_lanes() == [("pinky1", "P-6", "N")]
    assert pilot.core.traffic.robot_view("pinky1")["blocked_lane"] == ["P-6", "N"]


def test_the_zone_ahead_is_freed_only_after_the_cancel_result_and_a_stop():
    pilot = Pilot()
    blocked_on_the_way(pilot)
    goal = first_goal(pilot)
    pilot.run(MARK_S + 2.0)
    assert pilot.held() == {"N", "S"}  # 취소 결과가 아직 없다
    pilot.core.on_move_result("pinky1", goal, False, "CANCELED")
    pilot.run(0.1 * (CONFIRM - 1))
    assert pilot.held() == {"N", "S"}  # 정지 보고가 아직 모자라다
    pilot.run(0.2)
    assert pilot.held() == {"N"}  # 몸이 있는 N은 쥐고 S는 놓았다


def test_a_robot_still_moving_after_the_cancel_keeps_the_zone_ahead():
    pilot = Pilot()
    blocked_on_the_way(pilot)
    goal = first_goal(pilot)
    pilot.run(MARK_S + 0.5)
    pilot.core.on_move_result("pinky1", goal, False, "CANCELED")
    pilot.state["pinky1"] = "MOVING"
    pilot.run(2.0)
    assert pilot.held() == {"N", "S"}


def test_nothing_is_requested_or_sent_while_marked():
    pilot = Pilot()
    blocked_on_the_way(pilot)
    goal = first_goal(pilot)
    pilot.run(MARK_S + 0.2)
    pilot.core.on_move_result("pinky1", goal, False, "CANCELED")
    pilot.run(10.0)
    marked_at = pilot.events("LANE_BLOCKED")[0][0]
    assert not [t for t, _ in pilot.sent("MoveCmd") if t > marked_at]
    assert pilot.held() == {"N"}
    assert pilot.core.zones.waiting_for("pinky1") is None
    assert not pilot.events("GOAL_STALLED")


# ---- 풀림 -----------------------------------------------------------------------------------


def test_after_a_mark_three_clear_reports_resend_the_same_goal_without_counting():
    pilot = Pilot()
    blocked_on_the_way(pilot)
    goal = first_goal(pilot)
    pilot.run(MARK_S + 0.2)
    pilot.core.on_move_result("pinky1", goal, False, "CANCELED")
    pilot.run(2.0)
    pilot.state["pinky1"] = "IDLE"  # 물체가 비켰고 goal은 취소되어 있다
    pilot.run(0.1 * (CONFIRM - 1))
    assert not pilot.events("LANE_CLEARED")
    pilot.run(0.5)
    cleared = pilot.events("LANE_CLEARED")
    assert len(cleared) == 1 and cleared[0][1].level == "INFO"
    again = [(t, c) for t, c in pilot.sent("MoveCmd") if t >= cleared[0][0]]
    assert again and again[0][1].target_id == "N"
    assert pilot.mission().attempts == 0
    assert pilot.core.traffic.blocked_lanes() == []
    assert not pilot.events("GOAL_RETRY")


def test_a_clear_before_the_cancel_result_sends_again_only_after_the_result():
    """풀림이 취소 결과보다 먼저 오면 결과가 올 때까지 새 goal을 내지 않는다.

    그 사이 새 goal이 다시 막혀 표시되면 앞 취소의 추적을 잃기 때문이다.
    """
    pilot = Pilot()
    blocked_on_the_way(pilot)
    goal = first_goal(pilot)
    pilot.run(MARK_S + 0.2)
    pilot.state["pinky1"] = "MOVING"
    pilot.run(0.1 * CONFIRM + 0.3)
    cleared = pilot.events("LANE_CLEARED")
    assert len(cleared) == 1
    assert pilot.mission().goal is None
    assert not [c for t, c in pilot.sent("MoveCmd") if t >= cleared[0][0]]
    pilot.core.on_move_result("pinky1", goal, False, "CANCELED")
    pilot.run(0.3)
    resent = pilot.mission().goal
    assert resent is not None and resent.id != goal and resent.kind == MOVE
    assert pilot.mission().attempts == 0
    assert not pilot.events("GOAL_RETRY")


def test_a_robot_held_for_a_block_keeps_its_call_but_takes_no_new_one():
    """표시 중이거나 취소 결과를 기다리는 로봇은 맡은 호출은 지키고 새 호출은 받지 않는다."""
    pilot = Pilot()
    blocked_on_the_way(pilot)
    pilot.run(MARK_S + 0.2)
    m, traffic = pilot.mission(), pilot.core.traffic
    assert m.block.marked and m.called
    assert traffic._robot_info(m).eligible
    m.called = False
    assert not traffic._robot_info(m).eligible
    m.called = True
    pilot.state["pinky1"] = "MOVING"
    pilot.run(0.1 * CONFIRM + 0.3)
    assert m.block is None and m.block_cancel is not None
    m.called = False
    assert not traffic._robot_info(m).eligible
    pilot.core.on_move_result("pinky1", first_goal(pilot), False, "CANCELED")
    assert traffic._robot_info(m).eligible


def test_a_lost_link_neither_adds_block_time_nor_clears_it():
    """두절 중에는 막힌 시간을 더하지 않는다. 복구 확인이 끝나야 다시 센다."""
    pilot = Pilot()
    start = blocked_on_the_way(pilot)
    pilot.run(1.0)
    pilot.silent.add("pinky1")
    pilot.run(PARAMS.comm_timeout_s + 2.0)
    assert pilot.events("COMM_LOST")
    pilot.silent.clear()
    pilot.run(MARK_S + PARAMS.recover_check_s + 1.0)
    marks = pilot.events("LANE_BLOCKED")
    assert len(marks) == 1
    lost = pilot.events("COMM_LOST")[0][0]
    restored = pilot.events("COMM_RESTORED")[0][0]
    frozen = restored + PARAMS.recover_check_s - lost
    assert marks[0][0] >= start + MARK_S + frozen - 0.3


def test_a_recovery_keeps_the_zone_ahead_until_the_cancel_result():
    """표시 뒤 두절이 끝나도 취소 결과가 오기 전에는 앞 구역을 놓지 않는다."""
    pilot = Pilot()
    blocked_on_the_way(pilot)
    goal = first_goal(pilot)
    pilot.run(MARK_S + 0.2)
    assert pilot.events("LANE_BLOCKED")
    pilot.silent.add("pinky1")
    pilot.run(PARAMS.comm_timeout_s + 0.5)
    pilot.silent.clear()
    pilot.run(PARAMS.recover_check_s + 1.0)
    assert pilot.events("COMM_RESTORED") and not pilot.mission().comm
    assert pilot.held() == {"N", "S"}
    pilot.core.on_move_result("pinky1", goal, False, "CANCELED")
    pilot.run(0.1)
    assert pilot.held() == {"N", "S"}  # 결과 뒤 정지 보고가 아직 모자라다
    pilot.run(0.1 * CONFIRM)
    assert pilot.held() == {"N"}


def test_no_zone_is_let_go_before_the_cancel_result_even_after_the_clear():
    """풀림이 취소 결과보다 먼저 와도 결과가 올 때까지 자동으로 놓지 않는다."""
    pilot = Pilot()
    blocked_on_the_way(pilot)
    goal = first_goal(pilot)
    pilot.run(MARK_S + 0.2)
    pilot.state["pinky1"] = "MOVING"
    pilot.run(0.1 * CONFIRM + 0.3)
    m = pilot.mission()
    assert m.block is None and pilot.events("LANE_CLEARED")
    traffic = pilot.core.traffic
    m.outside["S"] = CONFIRM
    traffic._release_if_cleared(m, "S")
    assert "S" in pilot.held()
    pilot.core.on_move_result("pinky1", goal, False, "CANCELED")
    traffic._release_if_cleared(m, "S")
    assert "S" not in pilot.held()


def test_clear_reports_before_a_lost_link_do_not_count_after_it():
    """두절과 복구 확인 중의 BLOCKED는 못 볼 수 있어 풀림 보고를 처음부터 다시 센다."""
    pilot = Pilot()
    blocked_on_the_way(pilot)
    goal = first_goal(pilot)
    pilot.run(MARK_S + 0.2)
    pilot.core.on_move_result("pinky1", goal, False, "CANCELED")
    pilot.state["pinky1"] = "IDLE"
    pilot.run(0.1 * (CONFIRM - 1))
    pilot.silent.add("pinky1")
    pilot.run(PARAMS.comm_timeout_s + 0.5)
    pilot.silent.clear()
    pilot.state["pinky1"] = "BLOCKED"
    for _ in range(round((PARAMS.recover_check_s + 1.0) / 0.1)):
        if not pilot.mission().comm:
            break
        pilot.tick()
    assert not pilot.mission().comm
    pilot.state["pinky1"] = "IDLE"
    pilot.tick()
    assert not pilot.events("LANE_CLEARED")
    pilot.run(0.1 * (CONFIRM - 1))
    assert len(pilot.events("LANE_CLEARED")) == 1


# ---- 60초 -----------------------------------------------------------------------------------


def test_sixty_seconds_of_block_is_an_error_and_the_body_zone_stays():
    pilot = Pilot()
    start = blocked_on_the_way(pilot)
    goal = first_goal(pilot)
    pilot.run(MARK_S + 0.2)
    pilot.core.on_move_result("pinky1", goal, False, "CANCELED")
    pilot.run(PARAMS.blocked_timeout_s - MARK_S + 0.5)
    errors = pilot.events("BLOCKED_TOO_LONG")
    assert len(errors) == 1 and errors[0][1].level == "ERROR"
    assert start + PARAMS.blocked_timeout_s - 0.2 <= errors[0][0] <= start + 60.3
    assert pilot.status()["fault"] == "ERROR"
    assert pilot.status()["fault_reason"] == "BLOCKED"
    assert "N" in pilot.held()
    pilot.state["pinky1"] = "IDLE"
    pilot.run(1.0)
    assert len(pilot.events("LANE_CLEARED")) == 1  # 레인은 풀렸지만 ERROR는 운영자 몫
    assert pilot.status()["fault"] == "ERROR"


def test_a_block_again_on_the_same_lane_within_ten_seconds_adds_up():
    pilot = Pilot()
    blocked_on_the_way(pilot)
    pilot.run(3.0)
    pilot.put("pinky1", "P-6", "N", 0.45)  # 풀리고 조금 갔다
    pilot.run(1.0)
    again = pilot.now[0]
    pilot.state["pinky1"] = "BLOCKED"
    pilot.run(MARK_S)
    marks = pilot.events("LANE_BLOCKED")
    assert len(marks) == 1
    assert marks[0][0] <= again + MARK_S - 2.5  # 앞의 3초에 이어 센다


def test_a_block_after_ten_seconds_starts_from_zero():
    pilot = Pilot()
    blocked_on_the_way(pilot)
    pilot.run(3.0)
    pilot.state["pinky1"] = "MOVING"
    pilot.run(0.5)
    rejoin = pilot.mission().block_rejoin
    assert rejoin is not None
    lane, cleared_at, total = rejoin
    pilot.mission().block_rejoin = (lane, cleared_at - PARAMS.blocked_rejoin_s - 1.0, total)
    again = pilot.now[0]
    pilot.state["pinky1"] = "BLOCKED"
    pilot.run(MARK_S + 0.3)
    marks = pilot.events("LANE_BLOCKED")
    assert len(marks) == 1 and marks[0][0] >= again + MARK_S - 0.1


def test_a_block_on_another_lane_starts_from_zero():
    pilot = Pilot()
    blocked_on_the_way(pilot)
    pilot.run(3.0)
    pilot.put("pinky1", "P-6", "N", 0.873, state="MOVING")
    pilot.run(0.5)
    goal = pilot.mission().goal
    pilot.core.on_move_result("pinky1", goal.id, True, "")
    pilot.run(0.5)
    assert pilot.mission().goal.target_idx == pilot.mission().path.index("S")
    pilot.put("pinky1", "N", "S", 0.3)
    pilot.run(0.5)
    pilot.state["pinky1"] = "BLOCKED"
    pilot.run(MARK_S - 1.0)
    assert not pilot.events("LANE_BLOCKED")  # 같은 레인이면 3초에 이어 세어 표시했다


# ---- 줄 서기 -------------------------------------------------------------------------------------


def queue(front=None, gap=0.5):
    """pinky2가 P-6→N 끝으로 가고, pinky1은 N→S 앞쪽에 있다. 둘 사이 경로 거리는 gap."""
    pilot = Pilot(starts={"pinky1": "N", "pinky2": "P-6"}, orders=())
    traffic = pilot.core.traffic
    rear, ahead = traffic._missions["pinky2"], traffic._missions["pinky1"]
    span = pilot.site.lane("P-6", "N").span
    rear.path, rear.idx, rear.goal = ["P-6", "N", "S"], 1, _Goal(MOVE, 91, 1)
    s_rear = span - 0.25
    pilot.put("pinky2", "P-6", "N", s_rear)
    rear.x, rear.y = pilot.pose["pinky2"]
    ahead.path, ahead.idx, ahead.goal = ["N", "S"], 1, _Goal(MOVE, 92, 1)
    pilot.put("pinky1", "N", "S", gap - 0.25)
    ahead.x, ahead.y = pilot.pose["pinky1"]
    ahead.robot_state = "MOVING"
    if front:
        front(ahead)
    return pilot, traffic, rear


def faulted(m):
    m.fault = "ERROR"


def silent(m):
    m.comm = "COMM_LOST"


def blocked(m):
    m.robot_state = "BLOCKED"


def localizing(m):
    m.robot_state = "LOCALIZING"


def test_a_healthy_robot_close_ahead_is_a_queue():
    pilot, traffic, rear = queue()
    assert traffic._queued_behind(rear)


@pytest.mark.parametrize("front", [faulted, silent, blocked, localizing])
def test_a_robot_ahead_that_is_not_healthy_is_no_queue(front):
    pilot, traffic, rear = queue(front)
    assert not traffic._queued_behind(rear)


def test_a_robot_further_ahead_is_no_queue():
    pilot, traffic, rear = queue(gap=PARAMS.queue_gap_m + 0.1)
    assert not traffic._queued_behind(rear)


def test_a_queued_robot_reporting_blocked_opens_no_block():
    pilot, traffic, rear = queue()
    for _ in range(CONFIRM):
        traffic.on_robot_state("pinky2", rear.x, rear.y, 0.0, "BLOCKED")
    assert rear.block is None
    pilot, traffic, rear = queue(faulted)
    traffic.on_robot_state("pinky2", rear.x, rear.y, 0.0, "BLOCKED")
    assert rear.block is not None and rear.block.lane == ("P-6", "N")


def test_the_block_clock_rests_while_a_healthy_robot_stands_close_ahead():
    """막힘을 연 뒤 앞에 정상 로봇이 서면 줄 서기다. 막힌 시간을 더하지 않는다."""
    pilot, traffic, rear = queue(faulted)
    traffic.on_robot_state("pinky2", rear.x, rear.y, 0.0, "BLOCKED")
    assert rear.block is not None
    traffic._missions["pinky1"].fault = ""
    for step in range(1, round((MARK_S + 1.0) / 0.1)):
        traffic._watch_block(rear, step * 0.1)
    assert rear.block.total == 0.0 and not rear.block.marked


def test_a_queue_that_lasts_past_the_timeout_is_a_block():
    """앞 정상 로봇 뒤 줄 서기도 blocked_timeout_s를 넘기면 막힘 규칙을 쓴다."""
    pilot, traffic, rear = queue()
    for step in range(round((PARAMS.blocked_timeout_s + 1.0) / 0.5)):
        pilot.now[0] = step * 0.5
        pilot.core.on_robot_state("pinky2", rear.x, rear.y, 0.0, "BLOCKED")
        if step == 2:
            assert rear.block is None
    assert rear.block is not None and rear.block.lane == ("P-6", "N")


def test_a_queue_past_the_timeout_marks_and_fails_on_the_block_clock():
    """줄 서기가 시간 제한을 넘으면 그때부터 막힘 시계로 5초에 표시하고 60초에 ERROR다."""
    pilot, traffic, rear = queue()
    marked_at = failed_at = None
    for step in range(round((2 * PARAMS.blocked_timeout_s + 1.0) / 0.5)):
        t = step * 0.5
        pilot.now[0] = t
        pilot.core.on_robot_state("pinky2", rear.x, rear.y, 0.0, "BLOCKED")
        if rear.block:
            traffic._watch_block(rear, t)
        if marked_at is None and rear.block and rear.block.marked:
            marked_at = t
        if failed_at is None and rear.fault:
            failed_at = t
    assert marked_at == pytest.approx(PARAMS.blocked_timeout_s + MARK_S)
    assert failed_at == pytest.approx(2 * PARAMS.blocked_timeout_s)


def test_a_report_that_is_not_blocked_starts_the_queue_clock_again():
    """막힘을 연 채 줄을 서다가 BLOCKED가 아닌 보고가 오면 줄 서기 시계를 처음부터 잰다."""
    pilot, traffic, rear = queue(faulted)
    pilot.core.on_robot_state("pinky2", rear.x, rear.y, 0.0, "BLOCKED")
    assert rear.block is not None
    traffic._missions["pinky1"].fault = ""
    moved = PARAMS.blocked_timeout_s - 10.0
    for step in range(1, round((PARAMS.blocked_timeout_s + MARK_S + 5.0) / 0.5)):
        t = step * 0.5
        pilot.now[0] = t
        state = "MOVING" if moved <= t < moved + 0.5 * (CONFIRM - 1) else "BLOCKED"
        pilot.core.on_robot_state("pinky2", rear.x, rear.y, 0.0, state)
        traffic._watch_block(rear, t)
    assert rear.block is not None and not rear.block.marked


def test_a_lost_link_starts_the_queue_clock_again():
    """두절 동안 줄을 섰는지 모르니 줄 서기 시계를 처음부터 잰다."""
    pilot, traffic, rear = queue()
    pilot.core.on_robot_state("pinky2", rear.x, rear.y, 0.0, "BLOCKED")
    assert rear.queue_since == 0.0
    traffic._check_comm(PARAMS.comm_timeout_s + 1.0)
    assert rear.comm == "COMM_LOST" and rear.queue_since is None


def test_a_lane_no_longer_on_the_path_has_no_next_lane():
    """경로가 바뀌어 막힌 레인이 지금 자리에 없으면 그 다음 레인으로 줄 서기를 보지 않는다."""
    pilot, traffic, rear = queue()
    rear.path, rear.idx = ["S", "L", "Q"], 0
    pilot.put("pinky1", "L", "Q", 0.1)
    front = traffic._missions["pinky1"]
    front.x, front.y = pilot.pose["pinky1"]
    assert not traffic._queued_behind(rear, ("P-6", "N"))


# ---- 운영자 ----------------------------------------------------------------------------------


def test_confirm_clear_drops_the_mark():
    pilot = Pilot()
    blocked_on_the_way(pilot)
    pilot.run(MARK_S + 0.5)
    released = pilot.core.confirm_clear("pinky1", sorted(pilot.held()), "lifted")
    assert released is not None
    pilot.run(0.1)
    assert pilot.core.traffic.blocked_lanes() == []
    assert len(pilot.events("LANE_CLEARED")) == 1


def test_stop_during_a_block_keeps_the_zones_and_the_mark():
    pilot = Pilot()
    blocked_on_the_way(pilot)
    pilot.run(1.0)
    pilot.core.on_order_board("STOP", [ORDER])
    pilot.run(MARK_S + 1.0)
    assert len(pilot.events("LANE_BLOCKED")) == 1
    assert "N" in pilot.held()


# ---- 기록 재생 -------------------------------------------------------------------------------


def test_a_block_plays_back_the_same(tmp_path):
    lines = []
    pilot = Pilot(write=lines.append)
    blocked_on_the_way(pilot)
    goal = first_goal(pilot)
    pilot.run(MARK_S + 0.5)
    pilot.core.on_move_result("pinky1", goal, False, "CANCELED")
    pilot.run(1.0)
    pilot.state["pinky1"] = "IDLE"
    pilot.run(1.0)
    assert pilot.events("LANE_CLEARED")
    path = tmp_path / "block.jsonl"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    result = replay.replay(path)
    assert result.compared and result.same, result


# ---- LOCALIZING -----------------------------------------------------------------------------


def test_a_localizing_robot_is_not_placed_and_its_pose_is_not_rejected():
    now = [0.0]
    core = FleetManager(load_lane_map(), load_params(), ["pinky1"], lambda: now[0])
    for _ in range(CONFIRM):
        core.on_robot_state("pinky1", -5.0, -5.0, 0.0, "LOCALIZING")
    out = core.tick()
    assert not [e for e in out if type(e).__name__ == "EventOut" and e.code == "INPUT_REJECTED"]
    assert core.traffic.robot_view("pinky1")["on_map"] is False
    x, y = core.site.waypoint_pose("P-6")
    for _ in range(CONFIRM):
        core.on_robot_state("pinky1", x, y, 0.0, "IDLE")
    core.tick()
    assert core.traffic.robot_view("pinky1")["on_map"] is True


def test_a_first_report_of_blocked_does_not_place_the_robot():
    now = [0.0]
    core = FleetManager(load_lane_map(), load_params(), ["pinky1"], lambda: now[0])
    x, y = core.site.waypoint_pose("P-6")
    core.on_robot_state("pinky1", x, y, 0.0, "BLOCKED")
    assert core.traffic.robot_view("pinky1")["on_map"] is False


def test_a_placed_robot_that_reports_localizing_keeps_its_zones_and_pose():
    pilot = Pilot(orders=())
    pilot.run(0.3)
    held = pilot.held()
    pose = (pilot.status()["x"], pilot.status()["y"])
    pilot.pose["pinky1"] = (-5.0, -5.0)
    pilot.state["pinky1"] = "LOCALIZING"
    pilot.run(PARAMS.comm_timeout_s + 1.0)
    lost = pilot.events("COMM_LOST")
    assert len(lost) == 1 and "LOCALIZING" in lost[0][1].detail
    assert (pilot.status()["x"], pilot.status()["y"]) == pose
    assert pilot.held() == held
    assert not pilot.events("INPUT_REJECTED")


def test_a_robot_that_says_localizing_during_the_recovery_check_is_not_resumed():
    """복구 확인 중 LOCALIZING이 오면 위치를 다시 잡을 때까지 복구를 끝내지 않는다."""
    pilot = Pilot(orders=())
    pilot.run(0.3)
    pilot.silent.add("pinky1")
    pilot.run(PARAMS.comm_timeout_s + 0.5)
    pilot.silent.clear()
    pilot.tick()
    assert pilot.mission().comm == "RECOVERING"
    pilot.state["pinky1"] = "LOCALIZING"
    pilot.run(PARAMS.recover_check_s + 0.5)
    assert pilot.mission().comm == "RECOVERING"
    pilot.run(PARAMS.comm_timeout_s)
    assert len(pilot.events("COMM_LOST")) == 2
    pilot.state["pinky1"] = "IDLE"
    pilot.run(PARAMS.recover_check_s + 0.5)
    assert pilot.mission().comm == ""


def test_a_recovery_waits_when_the_last_report_said_localizing():
    """LOCALIZING 하나 뒤 보고가 끊겨도 옛 자세로 복구를 끝내지 않는다. 위치가 오면 다시 잰다."""
    pilot = Pilot(orders=())
    pilot.run(0.3)
    pilot.silent.add("pinky1")
    pilot.run(PARAMS.comm_timeout_s + 0.5)
    pilot.silent.clear()
    pilot.tick()
    assert pilot.mission().comm == "RECOVERING"
    pilot.state["pinky1"] = "LOCALIZING"
    pilot.tick()
    pilot.silent.add("pinky1")
    pilot.run(PARAMS.recover_check_s + 0.5)
    assert pilot.mission().comm == "RECOVERING"
    pilot.silent.clear()
    pilot.state["pinky1"] = "IDLE"
    pilot.run(PARAMS.recover_check_s - 0.2)
    assert pilot.mission().comm == "RECOVERING"
    pilot.run(0.4)
    assert pilot.mission().comm == ""


# ---- HEARTBEAT_LOST -------------------------------------------------------------------------


def test_a_goal_stopped_for_the_heartbeat_is_sent_again_without_counting():
    pilot = Pilot()
    pilot.run(0.3)
    goal = first_goal(pilot)
    pilot.core.on_move_result("pinky1", goal, False, "HEARTBEAT_LOST")
    pilot.run(PARAMS.move_retry_wait_s + 0.3)
    moves = pilot.sent("MoveCmd")
    assert len(moves) == 2 and moves[1][1].target_id == "N"
    assert pilot.mission().attempts == 0
    warned = pilot.events("HEARTBEAT_LOST")
    assert len(warned) == 1 and warned[0][1].level == "WARN"
    assert not pilot.events("GOAL_RETRY")


def test_heartbeat_stops_over_and_over_without_progress_become_a_stall():
    pilot = Pilot()
    answered = set()
    for _ in range(round((PARAMS.stall_s + 3 * PARAMS.move_retry_wait_s) / 0.1)):
        pilot.tick()
        for _, move in pilot.sent("MoveCmd"):
            if move.goal_id not in answered:
                answered.add(move.goal_id)
                pilot.core.on_move_result("pinky1", move.goal_id, False, "HEARTBEAT_LOST")
    assert len(pilot.events("GOAL_STALLED")) == 1
    assert pilot.mission().attempts == 1


# ---- 두 대가 함께 (SimWorld, 기본 지도, 잡음 없음) ---------------------------------------

TWO = {
    "pinky1": {"start": "P-6", "speed_mps": 0.20, "pose_noise_m": 0.0},
    "pinky2": {"start": "P-5", "speed_mps": 0.18, "pose_noise_m": 0.0},
}
BLOCK_AT = 7.0  # pinky1이 N→S 레인 가운데쯤


def world(duration_s, robots=None):
    faults = [{"at_s": BLOCK_AT, "robot": "pinky1", "type": "blocked", "duration_s": duration_s}]
    return SimWorld(site=load_lane_map(), robots=dict(robots or TWO), faults=faults)


def codes(w, code):
    return [(t, e) for t, e in w.event_log if e.code == code]


def test_the_robot_behind_waits_for_the_blocked_one_and_everything_ships():
    w = world(15.0)
    w.run(20.0)
    assert w.core.traffic.blocked_lanes() == [("pinky1", "N", "S")]
    assert set(w.core.zones.held_by("pinky1")) == {"S"}  # 앞 구역 L은 놓았다
    assert "pinky1" in w.core.traffic.status_view("pinky2")["wait_robots"]
    assert w.run_until(w.done)
    assert len(codes(w, "LANE_BLOCKED")) == 1 and len(codes(w, "LANE_CLEARED")) == 1
    for code in ("GOAL_STALLED", "GOAL_RETRY", "GOAL_FAILED", "DEADLOCK_DETECTED"):
        assert not codes(w, code), code


def test_a_short_block_ships_everything_without_a_mark():
    w = world(3.0)
    assert w.run_until(w.done)
    assert not codes(w, "LANE_BLOCKED") and not codes(w, "GOAL_STALLED")


def test_a_robot_that_starts_localizing_joins_when_it_reports_a_pose():
    robots = dict(TWO)
    robots["pinky2"] = dict(TWO["pinky2"], localizing_s=2.0)
    w = SimWorld(site=load_lane_map(), robots=robots)
    assert w.run_until(w.done)
    assert not codes(w, "INPUT_REJECTED")
    first_move = min(t for t, c in w.commands if type(c).__name__ == "MoveCmd")
    assert first_move >= 2.0  # 모두 지도에 오르기 전에는 아무도 움직이지 않는다
