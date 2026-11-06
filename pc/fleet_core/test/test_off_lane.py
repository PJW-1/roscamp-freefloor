"""레인 이탈: 많이 벗어나면 세우고 구역을 쥔 채 해제 판단을 멈춘다(11-06 지시서 3번).

기본 지도에서 W를 떠나 A로 가는 로봇 하나로 잰다. 0.1초 틱.
"""
import math
from pathlib import Path

from core_helpers import cmds, codes
from fleet_core.common.commands import OrderRecord
from fleet_core.common.params import load_params
from fleet_core.manager import FleetManager
from fleet_core.traffic.controller import TrafficController
from fleet_core.traffic.lane_map import load_lane_map
from fleet_core.traffic.mission import MOVE, _Goal
from grid_map import grid_site

PARAMS = load_params()
N = PARAMS.release_confirm_samples
OFF = PARAMS.lane_error_warn_m + 0.05


def make(where, site=None):
    now = [0.0]
    core = TrafficController(site or load_lane_map(), load_params(), list(where),
                             lambda: now[0])
    for robot, node in where.items():
        x, y = core.site.waypoint_pose(node)
        for _ in range(N):
            core.on_robot_state(robot, x, y, 0.0, "IDLE")
    core.on_run_mode("RUN")
    return core, now


def beside(core, a, b, along, off, times=N, robot="r1"):
    """a에서 b로 가는 레인을 along만큼 가고 왼쪽으로 off만큼 벗어난 자세를 보고한다."""
    x, y = core.site.waypoint_pose(a)
    lane = core.site.lane(a, b)
    for _ in range(times):
        core.on_robot_state(robot, x + lane.ux * along - lane.uy * off,
                            y + lane.uy * along + lane.ux * off, 0.0, "MOVING")


def start(core):
    core.on_dispatch(1, "OUTBOUND", "r1")
    return cmds(core.tick(), "MoveCmd")[0]


def status(core):
    view = core.status_view("r1")
    return view["fault"], view["fault_reason"]


def events(out, code):
    return [e for e in cmds(out, "EventOut") if e.code == code]


def stopped():
    """W에서 A로 가다 레인을 벗어나 선 로봇."""
    core, now = make({"r1": "W"})
    first = start(core)
    beside(core, "W", "A", 0.1, OFF)
    out = core.tick()
    return core, now, first, out


def test_the_values():
    assert PARAMS.off_lane_timeout_s == 20.0
    assert load_lane_map().half_width < PARAMS.lane_error_warn_m


def test_a_robot_far_off_its_lane_is_stopped_and_keeps_its_zones():
    core, _ = make({"r1": "W"})
    first = start(core)
    held = core.robot_view("r1")["held"]
    beside(core, "W", "A", 0.1, OFF, N - 1)
    out = core.tick()
    assert not cmds(out, "CancelCmd"), "one report short"
    beside(core, "W", "A", 0.1, OFF, 1)
    out = core.tick()
    warned = events(out, "LANE_ERROR")
    assert [e.level for e in warned] == ["WARN"]
    assert [c.goal_id for c in cmds(out, "CancelCmd")] == [first.goal_id]
    assert core.robot_view("r1")["goal_id"] is None
    assert core.robot_view("r1")["held"] == held
    assert status(core) == ("RECOVERING", "OFF_LANE")


def test_nothing_is_released_or_sent_while_off_the_lane():
    core, now, _, _ = stopped()
    out = []
    d = PARAMS.release_distance_m
    for _ in range(round(2 * PARAMS.stall_s / 0.1)):
        now[0] = round(now[0] + 0.1, 6)
        beside(core, "W", "A", d + 0.1, OFF, 1)  # 이탈 전이면 W를 놓을 진행
        out += core.tick()
    assert "W" in core.robot_view("r1")["held"]
    assert not cmds(out, "MoveCmd")
    assert "GOAL_STALLED" not in codes(out), "no goal, so the stall watch rests"
    assert codes(out).count("LANE_ERROR") == 0, "once per stop"
    assert status(core) == ("RECOVERING", "OFF_LANE")


def test_back_on_the_lane_sends_the_same_goal_again_and_clears_the_fault():
    core, _, first, _ = stopped()
    half = core.site.half_width
    beside(core, "W", "A", 0.1, half - 0.01)
    out = core.tick()
    again = cmds(out, "MoveCmd")
    assert [m.target_id for m in again] == [first.target_id]
    assert again[0].goal_id != first.goal_id
    assert [e.level for e in events(out, "BACK_ON_LANE")] == ["INFO"]
    assert status(core) == ("NONE", "")


def test_reports_back_on_the_lane_must_follow_each_other():
    core, _, _, _ = stopped()
    beside(core, "W", "A", 0.1, 0.0, N - 1)
    beside(core, "W", "A", 0.1, OFF, 1)
    beside(core, "W", "A", 0.1, 0.0, N - 1)
    assert not cmds(core.tick(), "MoveCmd")
    beside(core, "W", "A", 0.1, 0.0, 1)
    assert cmds(core.tick(), "MoveCmd")


def test_on_the_line_of_the_lane_but_past_its_end_is_not_back():
    """레인 직선 위라도 구간 밖이면 돌아온 것이 아니다(Codex 리뷰)."""
    core, _, _, _ = stopped()
    half = core.site.half_width
    span = core.site.lane("W", "A").span
    beside(core, "W", "A", span + 3 * half, 0.0)
    beside(core, "W", "A", -3 * half, 0.0, 1)
    out = core.tick()
    assert not cmds(out, "MoveCmd")
    assert status(core) == ("RECOVERING", "OFF_LANE")
    beside(core, "W", "A", span, 0.0)  # 끝 지점까지는 레인이다
    assert cmds(core.tick(), "MoveCmd")


def test_coming_back_after_the_timeout_is_still_an_error():
    """시간을 넘긴 뒤 들어온 복귀 보고는 시간 초과를 덮지 않는다(Codex 리뷰)."""
    core, now, _, _ = stopped()
    now[0] += PARAMS.off_lane_timeout_s
    beside(core, "W", "A", 0.1, 0.0)
    out = core.tick()
    assert not cmds(out, "MoveCmd")
    assert status(core) == ("ERROR", "OFF_LANE")


def test_a_robot_that_does_not_come_back_ends_in_error_and_keeps_its_zones():
    core, now, _, _ = stopped()
    held = core.robot_view("r1")["held"]
    since = now[0]
    now[0] = since + PARAMS.off_lane_timeout_s - 0.1
    beside(core, "W", "A", 0.1, OFF, 1)
    assert not events(core.tick(), "OFF_LANE")
    now[0] = since + PARAMS.off_lane_timeout_s
    beside(core, "W", "A", 0.1, OFF, 1)
    out = core.tick()
    assert [e.level for e in events(out, "OFF_LANE")] == ["ERROR"]
    assert status(core) == ("ERROR", "OFF_LANE")
    assert core.robot_view("r1")["held"] == held
    beside(core, "W", "A", 0.1, 0.0)
    out = core.tick()
    assert not cmds(out, "MoveCmd"), "the operator decides now"
    assert status(core) == ("ERROR", "OFF_LANE")
    assert core.confirm_clear("r1", held, "lifted") == held
    assert status(core)[0] == "OUT_OF_SERVICE"


def test_the_count_survives_a_goal_sent_again_on_the_same_lane():
    """예전에는 새 목표마다 횟수가 0이 되어 연장이 잦으면 영영 못 잡았다."""
    core, now = make({"r1": "W"})
    first = start(core)
    beside(core, "W", "A", 0.1, OFF, N - 1)
    core.on_goal_not_sent("r1", first.goal_id)
    now[0] += PARAMS.move_retry_wait_s
    again = cmds(core.tick(), "MoveCmd")
    assert [m.target_id for m in again] == [first.target_id]
    beside(core, "W", "A", 0.1, OFF, 1)
    out = core.tick()
    assert len(events(out, "LANE_ERROR")) == 1
    assert status(core) == ("RECOVERING", "OFF_LANE")


def test_the_count_starts_again_on_the_next_lane_of_a_long_goal():
    """곧은 목표 하나가 레인 둘에 걸친다. 레인이 바뀌면 처음부터 센다."""
    core, _ = make({"r1": (4, 3)}, grid_site())
    start(core)
    m = core._missions["r1"]
    a, b, c = m.path[0], m.path[1], m.path[2]
    assert m.goal.target_idx == 2
    beside(core, a, b, 0.5 * core.site.lane(a, b).length, OFF, N - 1)
    beside(core, b, c, 0.05, OFF, 1)
    out = core.tick()
    assert not events(out, "LANE_ERROR"), "one report on the new lane"
    beside(core, b, c, 0.1, OFF, N - 1)
    assert len(events(core.tick(), "LANE_ERROR")) == 1


def test_comm_loss_while_off_the_lane_keeps_the_zones_and_waits_for_the_lane():
    core, now, _, _ = stopped()
    held = core.robot_view("r1")["held"]
    now[0] += PARAMS.comm_timeout_s + 0.5
    out = core.tick()
    assert "COMM_LOST" in codes(out)
    assert status(core) == ("COMM_LOST", "NO_STATE")
    beside(core, "W", "A", 0.1, OFF, 1)
    out = core.tick()
    assert "COMM_RESTORED" in codes(out)
    for _ in range(round(3 * PARAMS.recover_check_s / 0.1)):
        now[0] = round(now[0] + 0.1, 6)
        beside(core, "W", "A", 0.1, OFF, 1)
        out += core.tick()
    assert not cmds(out, "MoveCmd"), "still off the lane"
    assert "RECOVERY_CONFLICT" not in codes(out)
    assert core.robot_view("r1")["held"] == held
    assert status(core)[0] == "RECOVERING"
    beside(core, "W", "A", 0.1, 0.0)
    out = core.tick() + core.tick()
    assert cmds(out, "MoveCmd")
    assert "BACK_ON_LANE" in codes(out)
    assert status(core) == ("NONE", "")
    assert core.robot_view("r1")["held"] == held


def test_back_before_the_deadline_is_not_an_error_while_the_comm_check_runs_past_it():
    """마감 전에 돌아왔으면 통신 복구 확인이 마감을 넘겨도 격리하지 않는다(Codex 리뷰)."""
    core, now, _, _ = stopped()
    since = now[0]
    now[0] += PARAMS.comm_timeout_s + 0.5
    assert "COMM_LOST" in codes(core.tick())
    now[0] = since + PARAMS.off_lane_timeout_s - 0.5
    beside(core, "W", "A", 0.1, 0.0)
    out = core.tick()
    assert "COMM_RESTORED" in codes(out)
    for _ in range(round(3 * PARAMS.recover_check_s / 0.1)):
        now[0] = round(now[0] + 0.1, 6)
        beside(core, "W", "A", 0.1, 0.0, 1)
        out += core.tick()
    assert not events(out, "OFF_LANE") and "ROBOT_ISOLATED" not in codes(out)
    assert "BACK_ON_LANE" in codes(out)
    assert cmds(out, "MoveCmd")
    assert status(core) == ("NONE", "")


def test_off_again_after_coming_back_brings_the_deadline_back():
    """돌아온 뒤 다시 벗어나 횟수가 0이 되면 마감이 다시 걸린다."""
    core, now, _, _ = stopped()
    since = now[0]
    now[0] += PARAMS.comm_timeout_s + 0.5
    assert "COMM_LOST" in codes(core.tick())
    now[0] = since + PARAMS.off_lane_timeout_s - 0.5
    beside(core, "W", "A", 0.1, 0.0)
    core.tick()
    beside(core, "W", "A", 0.1, OFF, 1)
    now[0] = since + PARAMS.off_lane_timeout_s
    out = core.tick()
    assert [e.level for e in events(out, "OFF_LANE")] == ["ERROR"]
    assert status(core) == ("ERROR", "OFF_LANE")


# ---- 레인 막힘(파트 4)과 함께 ---------------------------------------------------------------


def blocked_beside(core, along, off, times=N):
    x, y = core.site.waypoint_pose("W")
    lane = core.site.lane("W", "A")
    for _ in range(times):
        core.on_robot_state("r1", x + lane.ux * along - lane.uy * off,
                            y + lane.uy * along + lane.ux * off, 0.0, "BLOCKED")


def test_off_the_lane_while_a_block_is_open_stops_as_off_lane_and_closes_the_block():
    """막힘 표시 전에 이탈하면 이탈이 세운다. 막힘 시계는 닫고 구역은 쥔다."""
    core, now = make({"r1": "W"})
    start(core)
    held = core.robot_view("r1")["held"]
    blocked_beside(core, 0.1, 0.0, 1)
    core.tick()
    assert core._missions["r1"].block is not None
    blocked_beside(core, 0.1, OFF)
    out = core.tick()
    assert len(events(out, "LANE_ERROR")) == 1
    assert core._missions["r1"].block is None
    for _ in range(round(2 * PARAMS.blocked_mark_s / 0.1)):
        now[0] = round(now[0] + 0.1, 6)
        blocked_beside(core, 0.1, OFF, 1)
        out += core.tick()
    assert not events(out, "LANE_BLOCKED"), "no goal, so no block opens"
    assert status(core) == ("RECOVERING", "OFF_LANE")
    assert core.robot_view("r1")["held"] == held


def test_blocked_again_after_coming_back_opens_the_block_again():
    core, _ = make({"r1": "W"})
    first = start(core)
    blocked_beside(core, 0.1, OFF)
    core.tick()
    beside(core, "W", "A", 0.1, 0.0)
    again = cmds(core.tick(), "MoveCmd")
    assert [m.target_id for m in again] == [first.target_id]
    blocked_beside(core, 0.1, 0.0, 1)
    core.tick()
    assert core._missions["r1"].block is not None
    assert status(core) == ("NONE", "BLOCKED")


def test_blocked_again_within_the_rejoin_time_marks_at_the_remaining_time():
    """이탈로 닫은 막힘의 시간은 돌아와 같은 레인에서 다시 막히면 이어 센다(rejoin)."""
    core, now = make({"r1": "W"})
    start(core)
    before = 2.0
    assert before < PARAMS.blocked_mark_s

    def step(off, state="BLOCKED"):
        now[0] = round(now[0] + 0.1, 6)
        if state == "BLOCKED":
            blocked_beside(core, 0.1, off, 1)
        else:
            beside(core, "W", "A", 0.1, off, 1)
        return core.tick()

    for _ in range(round(before / 0.1)):
        step(0.0)
    for _ in range(N):
        step(OFF)  # 이탈로 서며 막힘을 닫는다
    assert core._missions["r1"].block is None
    for _ in range(N):
        out = step(0.0, "MOVING")
    assert events(out, "BACK_ON_LANE")
    again_at = now[0]
    out, marked_at = [], None
    for _ in range(round(2 * PARAMS.blocked_mark_s / 0.1)):
        out = step(0.0)
        if events(out, "LANE_BLOCKED"):
            marked_at = now[0]
            break
    assert marked_at is not None
    waited = marked_at - again_at
    assert waited < PARAMS.blocked_mark_s - 1.0, "the clock went on, not from zero"
    assert abs(waited - (PARAMS.blocked_mark_s - before)) <= 0.5


def test_a_marked_block_has_no_goal_so_lane_error_does_not_watch():
    """표시 뒤에는 목표가 취소되어 이탈을 재지 않는다. 그동안 놓는 것은 막힘 쪽 규칙만 한다."""
    core, now = make({"r1": "W"})
    start(core)
    out = []
    for _ in range(round((PARAMS.blocked_mark_s + 0.5) / 0.1)):
        now[0] = round(now[0] + 0.1, 6)
        blocked_beside(core, 0.1, 0.0, 1)
        out += core.tick()
    assert len(events(out, "LANE_BLOCKED")) == 1
    blocked_beside(core, 0.1, OFF)
    out = core.tick()
    assert not events(out, "LANE_ERROR")
    assert status(core) == ("NONE", "BLOCKED")


# ---- 우회(파트 5)와 함께 --------------------------------------------------------------------

FEATURE = Path(__file__).resolve().parents[3] / "sim/fake_robots/maps/feature_map.yaml"


def feature(starts):
    now = [0.0]
    site = load_lane_map(FEATURE)
    core = TrafficController(site, load_params(), list(starts), lambda: now[0])
    for robot, waypoint in starts.items():
        core.on_robot_state(robot, *site.waypoint_pose(waypoint), 0.0, "IDLE")
    core.on_run_mode("RUN")
    return core, now


def beside_on(core, robot, a, b, along, off, times=N):
    x, y = core.site.waypoint_pose(a)
    lane = core.site.lane(a, b)
    for _ in range(times):
        core.on_robot_state(robot, x + lane.ux * along - lane.uy * off,
                            y + lane.uy * along + lane.ux * off, 0.0, "MOVING")


def stop_off_lane_on(core, robot, path):
    """robot이 path[0]에서 path[1]로 가다 레인을 벗어나 서게 한다."""
    m = core._missions[robot]
    m.phase, m.dest, m.path, m.idx = "TO_WORK", "WORK", list(path), 0
    for waypoint in path[:2]:
        core.zones.force_acquire(robot, core.site.zone_of[waypoint])
    m.goal = _Goal(MOVE, 99, 1, core._step_between(m, 0))
    beside_on(core, robot, path[0], path[1], 0.05, OFF)
    core.take_output()
    assert core.status_view(robot)["fault_reason"] == "OFF_LANE"


def test_a_robot_off_its_lane_stands_for_good_so_the_one_behind_goes_around():
    core, _ = feature({"r1": "F0", "r2": "F1"})
    stop_off_lane_on(core, "r2", ["F1", "F2", "W"])
    assert core._standing_wait_s(core._missions["r2"]) == math.inf
    core.on_dispatch(1, "WORK", "r1")
    out = core.tick()
    assert len(events(out, "DETOUR")) == 1
    m = core._missions["r1"]
    assert m.path[m.idx:] == ["F0", "T2", "T1", "QW", "W"]


def test_with_no_other_way_the_one_behind_waits_for_the_off_lane_robot():
    core, _ = feature({"r1": "O", "r2": "F0"})
    stop_off_lane_on(core, "r2", ["F0", "F1", "F2", "W"])
    core.on_dispatch(1, "WORK", "r1")
    out = core.tick()
    assert not events(out, "DETOUR")
    assert core.zones.waiting_for("r1") == "F0"


def test_a_robot_on_a_detour_is_watched_and_resumed_on_its_new_lane():
    core, _ = feature({"r1": "F0", "r2": "F1"})
    stop_off_lane_on(core, "r2", ["F1", "F2", "W"])
    core.on_dispatch(1, "WORK", "r1")
    first = [c for c in core.tick() if type(c).__name__ == "MoveCmd" and c.robot == "r1"]
    # path 모드(파트 7)는 꺾여도 쥔 데까지 묶어 T2를 지나 T1까지 간다
    target = "T1" if core.params.lane_goal_mode == "path" else "T2"
    assert [c.target_id for c in first] == [target]
    # 동쪽으로 밀면 들어오는 레인 O→F0 위라 F0로 오는 중으로 본다. 서쪽으로 민다
    beside_on(core, "r1", "F0", "T2", 0.05, -OFF)
    out = core.tick()
    (warned,) = [e for e in events(out, "LANE_ERROR") if e.robot == "r1"]
    assert "F0 to T2" in warned.detail
    beside_on(core, "r1", "F0", "T2", 0.05, 0.0)
    out = core.tick()
    again = [c for c in cmds(out, "MoveCmd") if c.robot == "r1"]
    assert [c.target_id for c in again] == [target]
    assert [e.robot for e in events(out, "BACK_ON_LANE")] == ["r1"]


# ---- 자동 격리와의 결합(파트 5.5) ---------------------------------------------------------


def test_an_off_lane_timeout_isolates_the_robot_once():
    core, now, _, _ = stopped()
    held = core.zones.held_by("r1")
    now[0] += PARAMS.off_lane_timeout_s
    beside(core, "W", "A", 0.1, OFF, 1)
    out = core.tick()
    assert [(c.robot, c.reason) for c in cmds(out, "RobotIsolated")] == [("r1", "OFF_LANE")]
    assert len(events(out, "ROBOT_ISOLATED")) == 1
    assert core.zones.held_by("r1") == held
    now[0] += 1.0
    beside(core, "W", "A", 0.1, 0.0)
    again = core.tick()
    assert not cmds(again, "RobotIsolated") and not events(again, "ROBOT_ISOLATED")


def test_an_off_lane_timeout_puts_the_order_back_through_the_manager():
    now = [0.0]
    core = FleetManager(grid_site(), load_params(), ["r1"], lambda: now[0])

    def report(cell):
        x, y = core.site.waypoint_pose(cell)
        for _ in range(N):
            core.on_robot_state("r1", x, y, 0.0, "IDLE")

    report((4, 3))
    core.on_order_board("RUN", [OrderRecord("O-001", {"A": 1})])
    m = core.traffic._missions["r1"]
    for _ in range(300):
        out = core.tick()
        for fill in cmds(out, "FillCmd"):
            core.on_fill_result(fill.goal_id, True, {"A": 1}, {})
        bound = core.robot_view("r1")["order"] == "O-001"
        goal = m.goal
        if bound and goal is not None and goal.kind == MOVE and goal.target_idx > m.idx:
            break  # 주문을 싣고 레인을 달리는 중
        if goal is not None and goal.kind == MOVE:
            report(m.path[goal.target_idx])
            core.on_move_result("r1", goal.id, True, "")
        elif goal is not None:
            core.on_dock_result("r1", goal.id, True)
        report(m.path[m.idx])
    else:
        raise AssertionError(f"no move goal with the order: {core.robot_view('r1')}")
    a = core.site.waypoint_pose(m.path[m.idx])
    b = core.site.waypoint_pose(m.path[m.idx + 1])
    ux, uy = (b[0] - a[0]) / math.dist(a, b), (b[1] - a[1]) / math.dist(a, b)
    x, y = a[0] + ux * 0.1 - uy * OFF, a[1] + uy * 0.1 + ux * OFF
    for _ in range(N):
        core.on_robot_state("r1", x, y, 0.0, "MOVING")
    seen = core.tick()
    assert "LANE_ERROR" in codes(seen)
    now[0] += PARAMS.off_lane_timeout_s
    core.on_robot_state("r1", x, y, 0.0, "MOVING")
    seen += core.tick() + core.tick()
    assert "ROBOT_ISOLATED" in codes(seen)
    requeued = [e for e in cmds(seen, "EventOut") if e.code == "ORDER_REQUEUED"]
    assert len(requeued) == 1 and requeued[0].detail.startswith("O-001")
    view = core.robot_view("r1")
    assert (view["state"], view["order"]) == ("ERROR", None)
    assert core.traffic.status_view("r1")["fault_reason"] == "OFF_LANE"


# ---- 재투입과 함께(파트 6 병합) ------------------------------------------------------------


def test_a_reinstated_robot_starts_with_no_off_lane_state():
    core, now, _, _ = stopped()
    assert core.confirm_clear("r1", core.robot_view("r1")["held"], "lifted") is not None
    core.tick()
    assert core.reinstate("r1")
    m = core._missions["r1"]
    assert (m.off_lane_since, m.off_lane_at, m.back_on_lane) == (None, None, 0)
    assert (m.off_lane, m.off_lane_warned, m.lane_ref) == (0, False, None)
    x, y = core.site.waypoint_pose("P-3")
    for _ in range(N):
        core.on_robot_state("r1", x, y, 0.0, "IDLE")
    core.tick()
    assert core.robot_view("r1")["on_map"] and status(core) == ("NONE", "")


def test_a_holder_back_on_its_lane_stays_put_while_a_reinstate_holds_it():
    """이탈 중에 재투입 충돌로 세운 주인은 돌아와도 풀릴 때까지 목표를 받지 않는다."""
    core, now = make({"r1": "W", "r2": "P-6"})
    assert core.confirm_clear("r2", core.robot_view("r2")["held"], "lifted") is not None
    first = start(core)
    beside(core, "W", "A", 0.1, OFF)
    core.tick()
    assert status(core) == ("RECOVERING", "OFF_LANE")
    assert core.reinstate("r2")
    x, y = core.site.waypoint_pose("W")  # r1이 쥔 W에 놓았다
    for _ in range(N):
        core.on_robot_state("r2", x, y, 0.0, "IDLE")
    out = core.tick()
    assert events(out, "PLACEMENT_CONFLICT") and not core.available("r1")
    half = core.site.half_width
    beside(core, "W", "A", 0.1, half - 0.01)
    out = core.tick()
    assert events(out, "BACK_ON_LANE")
    assert not cmds(out, "MoveCmd"), "held for the placement"
    x, y = core.site.waypoint_pose("P-3")  # r2를 빈 자리로 옮겼다
    for _ in range(N):
        core.on_robot_state("r2", x, y, 0.0, "IDLE")
    beside(core, "W", "A", 0.1, half - 0.01, 1)
    out = core.tick()
    assert [c.target_id for c in cmds(out, "MoveCmd") if c.robot == "r1"] == [first.target_id]
