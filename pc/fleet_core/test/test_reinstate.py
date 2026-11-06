"""치운 로봇의 재투입(11-06 6번).

OUT_OF_SERVICE를 풀고 맵 밖에 둔다. 다음 위치 보고로 시작 때처럼 올라온다.
올라오기 전에도 다른 로봇은 멈추지 않고, 그 로봇에게는 일을 주지 않는다.
"""
import json

from core_helpers import cmds, codes
from fleet_core.common.commands import OrderRecord
from fleet_core.common.params import load_params
from fleet_core.manager import FleetManager
from fleet_core.traffic.lane_map import load_lane_map

O1 = OrderRecord("O-001", {"A": 1})
O2 = OrderRecord("O-002", {"B": 1})
SPOTS = {"pinky1": "P-6", "pinky2": "P-5"}


def make(robots=SPOTS):
    now = [0.0]
    core = FleetManager(load_lane_map(), load_params(), list(robots), lambda: now[0])
    for robot, waypoint in robots.items():
        report(core, robot, waypoint)
    core.on_order_board("RUN", [])
    core.tick()
    return core, now


def report(core, robot, waypoint, state="IDLE"):
    x, y = core.site.waypoint_pose(waypoint)
    for _ in range(core.params.release_confirm_samples):
        core.on_robot_state(robot, x, y, 0.0, state, 90.0)


def cleared(robots=SPOTS, robot="pinky2"):
    core, now = make(robots)
    assert core.confirm_clear(robot, core.zones.held_by(robot), "lifted") is not None
    core.tick()
    return core, now


def drive(core, robots, until, limit=400, swap=None):
    """이동과 도킹은 바로 끝내고 보고한다. 교환 명령은 swap(cmd)에 넘긴다."""
    seen = []
    for _ in range(limit):
        out = core.tick()
        seen += out
        for cmd in cmds(out, "SwapCmd"):
            if swap:
                swap(cmd)
        for robot in robots:
            m = core.traffic._missions[robot]
            goal = m.goal
            if goal is not None and goal.kind == "move":
                report(core, robot, m.path[goal.target_idx])
                core.on_move_result(robot, goal.id, True, "")
            elif goal is not None:
                core.on_dock_result(robot, goal.id, True)
            if m.placed:
                report(core, robot, m.path[m.idx])
        if until():
            return seen
    raise AssertionError(f"not reached: {[core.robot_view(r) for r in robots]}")


def moves_of(out, robot):
    return [c for c in cmds(out, "MoveCmd") if c.robot == robot]


# ---- 받아들임 -----------------------------------------------------------------------------


def test_a_cleared_robot_is_taken_back_and_waits_off_the_map_for_a_report():
    core, _ = cleared()
    assert core.reinstate("pinky2", "EMPTY", "back on the floor") is True
    assert core.last_reinstate_reason == ""
    view = core.traffic.robot_view("pinky2")
    assert view["fault"] == "" and not view["on_map"]
    out = core.tick()
    events = [e for e in cmds(out, "EventOut") if e.code == "ROBOT_REINSTATED"]
    assert len(events) == 1
    assert events[0].level == "INFO" and events[0].robot == "pinky2"
    assert "EMPTY" in events[0].detail and "back on the floor" in events[0].detail
    assert "INPUT_REJECTED" not in codes(out)


def test_the_robot_comes_back_on_its_first_report_like_at_start():
    core, _ = cleared()
    core.reinstate("pinky2", "EMPTY", "back")
    report(core, "pinky2", "P-3")
    view = core.traffic.robot_view("pinky2")
    assert view["on_map"] and view["fault"] == ""
    assert view["held"] == ["P-3"]
    assert core.robot_view("pinky2")["state"] == "PARKED"


def test_the_orders_side_takes_the_basket_given_with_no_order_and_no_attempts():
    core, _ = cleared()
    core.reinstate("pinky2", "NONE", "tray empty")
    view = core.orchestrator.robot_view("pinky2")
    assert view["basket"] == "NONE" and view["order"] is None
    assert view["attempts"] == 0 and view["stage"] == ""


def test_the_others_keep_working_before_the_robot_reports():
    """시작 때와 달리 모두를 세우지 않는다. 이미 운행 중이기 때문이다."""
    core, now = cleared()
    core.reinstate("pinky2", "EMPTY", "back")
    assert core.traffic.everyone_placed  # 노드가 /fleet/zones와 heartbeat를 계속 낸다
    core.on_order_board("RUN", [O1])
    out = []
    for _ in range(5):
        now[0] += 1.0
        out += core.tick()
    assert moves_of(out, "pinky1"), "pinky1 should go to work"
    assert not moves_of(out, "pinky2")
    now[0] += core.params.startup_wait_s + 1.0
    out = core.tick()
    assert "ROBOT_NOT_REPORTING" not in codes(out)


def test_the_robot_gets_no_work_before_it_reports():
    core, now = cleared(robots={"pinky1": "P-6", "pinky2": "P-5"}, robot="pinky1")
    core.confirm_clear("pinky2", core.zones.held_by("pinky2"), "lifted too")
    core.reinstate("pinky1", "EMPTY", "back")
    core.on_order_board("RUN", [O1, O2])
    out = []
    for _ in range(10):
        now[0] += 1.0
        out += core.tick()
    assert not moves_of(out, "pinky1")
    assert not core.traffic.robot_view("pinky1")["called"]
    report(core, "pinky1", "P-6")
    out = core.tick()
    assert moves_of(out, "pinky1"), "once on the map it takes the waiting order"


def test_a_report_while_moving_does_not_place_it():
    core, _ = cleared()
    core.reinstate("pinky2", "EMPTY", "back")
    report(core, "pinky2", "P-3", state="MOVING")
    assert not core.traffic.robot_view("pinky2")["on_map"]
    report(core, "pinky2", "P-3")
    assert core.traffic.robot_view("pinky2")["on_map"]


THREE = {"pinky1": "P-6", "pinky2": "P-5", "pinky3": "P-4"}


def ticks(core, now, n=10):
    """올라 있는 로봇은 제자리에서 보고하며 시간을 보낸다. 목표는 끝내지 않는다."""
    out = []
    for _ in range(n):
        now[0] += 0.5
        for robot, m in core.traffic._missions.items():
            if m.placed:
                report(core, robot, m.path[m.idx])
        out += core.tick()
    return out


def test_a_placement_conflict_stops_the_holder_until_the_robot_is_placed():
    """잠금 없이 남의 구역에 선 로봇이 있으니 그 구역 주인을 세운다(리뷰 1번)."""
    core, now = cleared(robots=THREE, robot="pinky3")
    core.reinstate("pinky3", "EMPTY", "back")
    report(core, "pinky3", "P-5")  # pinky2가 서 있는 자리
    out = core.tick()
    events = [e for e in cmds(out, "EventOut") if e.code == "PLACEMENT_CONFLICT"]
    assert len(events) == 1 and events[0].level == "ERROR"
    assert "pinky2" in events[0].detail
    assert core.traffic.robot_view("pinky3")["fault"] == "ERROR"
    assert not core.traffic.robot_view("pinky3")["on_map"]
    assert not core.traffic.available("pinky2")
    core.on_order_board("RUN", [O1])
    out = ticks(core, now)
    assert not moves_of(out, "pinky2"), "the holder stays put"
    assert moves_of(out, "pinky1"), "a robot not in the way takes the order"
    report(core, "pinky3", "P-3")
    view = core.traffic.robot_view("pinky3")
    assert view["on_map"] and view["fault"] == ""
    assert core.traffic.available("pinky2")


def test_a_placement_conflict_cancels_the_goal_of_the_holder():
    core, now = cleared()
    core.on_order_board("RUN", [O1])
    for _ in range(20):
        now[0] += 0.5
        core.tick()
        if core.traffic._missions["pinky1"].goal is not None:
            break
    m = core.traffic._missions["pinky1"]
    assert m.goal is not None, "pinky1 should be driving"
    goal_id = m.goal.id
    core.reinstate("pinky2", "EMPTY", "back")
    report(core, "pinky2", m.path[m.idx])
    out = core.tick()
    assert any(c.robot == "pinky1" and c.goal_id == goal_id for c in cmds(out, "CancelCmd"))
    out = ticks(core, now)
    assert not moves_of(out, "pinky1")
    report(core, "pinky2", "P-3")
    out = ticks(core, now)
    assert moves_of(out, "pinky1"), "it drives on once the robot is placed"


def test_clearing_the_conflicting_robot_again_frees_the_holder():
    core, now = cleared()
    core.reinstate("pinky2", "EMPTY", "back")
    report(core, "pinky2", "P-6")
    core.tick()
    assert not core.traffic.available("pinky1")
    core.confirm_clear("pinky2", [], "lifted again")
    assert core.traffic.available("pinky1")


def test_a_held_holder_gives_up_its_zone_requests_so_those_behind_are_not_blocked():
    """2차 리뷰 1번: FIFO 대기열의 요청이 남으면 뒤 로봇까지 막는다."""
    core, now = cleared(robots=THREE, robot="pinky3")
    zones = core.zones
    zones.force_acquire("pinky1", "N")  # pinky1이 N을 쥐고
    assert not zones.request("pinky2", "N")  # pinky2가 먼저 줄을 서고
    core.traffic._missions["pinky2"].wait_zone = "N"
    core.reinstate("pinky3", "EMPTY", "back")
    report(core, "pinky3", "P-5")  # pinky2 자리
    assert not core.traffic.available("pinky2")
    assert "pinky2" not in zones.queue("N")
    assert core.traffic._missions["pinky2"].wait_zone is None
    assert core.traffic.status_view("pinky2")["wait_zone"] == ""
    zones.release("pinky1", "N")
    assert zones.holder("N") is None
    assert not zones.blockers("pinky1", "N"), "nobody left in line ahead"


def test_a_held_robot_takes_no_new_named_dispatch():
    """2차 리뷰 2번: 세운 동안 새 지정 배차는 NOT_AVAILABLE."""
    core, now = cleared()
    core.reinstate("pinky2", "EMPTY", "back")
    report(core, "pinky2", "P-6")  # pinky1 자리
    core.tick()
    core.traffic.on_dispatch(901, "OUTBOUND", "pinky1")
    out = core.traffic.take_output()
    results = [c for c in out if type(c).__name__ == "DispatchResult" and c.goal_id == 901]
    assert len(results) == 1
    assert not results[0].success and results[0].reason == "NOT_AVAILABLE"
    assert core.traffic._missions["pinky1"].dispatch is None


def test_a_held_robot_drops_an_open_call_but_keeps_its_named_dispatch():
    core, now = cleared(robots=THREE, robot="pinky3")
    core.traffic.on_dispatch(902, "OUTBOUND", "pinky2")
    core.on_order_board("RUN", [O1])
    core.tick()
    core.reinstate("pinky3", "EMPTY", "back")
    report(core, "pinky3", "P-6")  # pinky1 자리
    m1, m2 = core.traffic._missions["pinky1"], core.traffic._missions["pinky2"]
    assert not m1.called, "an open call does not stay on a held robot"
    for _ in range(5):
        now[0] += 0.5
        core.tick()
    assert not core.traffic._missions["pinky1"].called
    report(core, "pinky3", "P-5")  # pinky2 자리로 옮겨 다시 보고
    assert core.traffic._missions["pinky2"].dispatch is not None, "named dispatch kept"
    assert not core.traffic.available("pinky2")
    report(core, "pinky3", "P-3")
    assert core.traffic.available("pinky2")
    assert m2 is core.traffic._missions["pinky2"]
    assert m2.dispatch is not None and m2.dispatch.goal_id == 902


def test_a_robot_with_no_basket_fetches_one_at_the_outbound_before_any_order():
    core, _ = cleared(robots={"pinky1": "P-6"}, robot="pinky1")
    core.reinstate("pinky1", "NONE", "tray empty")
    report(core, "pinky1", "P-6")
    core.on_order_board("RUN", [O1])
    swaps = []

    def swap(cmd):
        swaps.append(cmd)
        assert core.robot_view("pinky1")["order"] is None, "no order before a basket"
        core.on_swap_result(cmd.goal_id, True, False, True, "")

    drive(core, ["pinky1"], lambda: core.robot_view("pinky1")["order"] == "O-001", swap=swap)
    assert len(swaps) == 1
    assert swaps[0].take_full is False and swaps[0].put_empty is True
    assert core.robot_view("pinky1")["basket"] == "EMPTY"


def test_a_reinstated_robot_takes_orders_again_to_the_end():
    core, _ = cleared(robots={"pinky1": "P-6"}, robot="pinky1")
    core.reinstate("pinky1", "EMPTY", "back")
    report(core, "pinky1", "P-6")
    core.on_order_board("RUN", [O1])
    out = drive(core, ["pinky1"], lambda: core.robot_view("pinky1")["order"] == "O-001")
    out += core.tick()
    assert "ORDER_BOUND" in codes(out)
    fills = cmds(out, "FillCmd")
    assert fills and fills[0].robot == "pinky1"


def test_a_late_arm_result_of_the_cleared_job_does_not_count_for_the_same_order_again():
    """해제로 취소한 작업의 늦은 결과는 같은 로봇이 같은 주문을 다시 묶어도 넣지 않는다."""
    core, _ = make({"pinky1": "P-6"})
    core.on_order_board("RUN", [O1])
    out = drive(core, ["pinky1"], lambda: core.orchestrator.robot_view("pinky1")["stage"]
                == "FILLING" and core.orchestrator._robots["pinky1"].arm is not None)
    old_fill = cmds(out, "FillCmd")[-1]
    core.confirm_clear("pinky1", core.zones.held_by("pinky1"), "lifted")
    core.tick()
    core.reinstate("pinky1", "EMPTY", "back")
    report(core, "pinky1", "P-6")
    drive(core, ["pinky1"], lambda: core.robot_view("pinky1")["order"] == "O-001")
    core.on_fill_result(old_fill.goal_id, False, {"A": 1}, {}, "CANCELED")
    out = core.tick()
    late = [e for e in cmds(out, "EventOut") if e.code == "ARM_LATE_RESULT"]
    assert late and "check the basket" in late[0].detail
    assert core.robot_view("pinky1")["placed"] == {}


def test_a_late_result_of_a_job_given_up_before_the_clear_does_not_count_again():
    """ARM_NO_RESULT로 포기한 작업은 해제 전부터 잃은 작업이다(리뷰 2번)."""
    core, now = make({"pinky1": "P-6"})
    core.on_order_board("RUN", [O1])
    out = drive(core, ["pinky1"], lambda: core.orchestrator._robots["pinky1"].arm is not None)
    old_fill = cmds(out + core.tick(), "FillCmd")[-1]
    now[0] += core.params.fill_timeout_s + 1.0
    core.tick()
    now[0] += core.params.arm_cancel_wait_s + 1.0
    assert "ARM_NO_RESULT" in codes(core.tick())
    core.confirm_clear("pinky1", core.zones.held_by("pinky1"), "lifted")
    core.tick()
    core.reinstate("pinky1", "EMPTY", "back")
    report(core, "pinky1", "P-6")
    drive(core, ["pinky1"], lambda: core.robot_view("pinky1")["order"] == "O-001")
    core.on_fill_result(old_fill.goal_id, True, {"A": 1}, {}, "")
    out = core.tick()
    late = [e for e in cmds(out, "EventOut") if e.code == "ARM_LATE_RESULT"]
    assert late and "check the basket" in late[0].detail
    assert core.robot_view("pinky1")["placed"] == {}


def test_a_clear_and_a_reinstate_play_back_the_same(tmp_path):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
    import replay

    now = [0.0]
    core = FleetManager(load_lane_map(), load_params(), list(SPOTS), lambda: now[0])
    lines = []
    core.record_to(lines.append)
    for robot, waypoint in SPOTS.items():
        report(core, robot, waypoint)
    core.on_order_board("RUN", [O1, O2])
    core.tick()
    core.confirm_clear("pinky2", core.zones.held_by("pinky2"), "lifted")
    core.reinstate("pinky2", "FULL", "wrong basket")
    core.reinstate("pinky2", "EMPTY", "back")
    core.tick()
    report(core, "pinky2", "P-3")
    drive(core, list(SPOTS), lambda: core.robot_view("pinky1")["order"]
          and core.robot_view("pinky2")["called"])
    path = tmp_path / "run.jsonl"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    entries = [json.loads(line) for line in lines[1:]]
    assert [e["args"] for e in entries if e.get("in") == "reinstate"] == [
        ["pinky2", "FULL", "wrong basket"], ["pinky2", "EMPTY", "back"]]
    result = replay.replay(path)
    assert result.same and result.compared, result


# ---- 막힘과 함께(병합 리뷰 1번) ----------------------------------------------------------------


def blocked_holder_held_by_a_reinstate():
    """pinky1이 막힘(표시 전) 중에 재투입 충돌로 선다. pilot과 취소한 goal을 돌려준다."""
    from test_lane_blocked import Pilot, blocked_on_the_way, first_goal

    pilot = Pilot(starts={"pinky1": "P-6", "pinky2": "P-5"})
    core = pilot.core
    assert core.confirm_clear("pinky2", core.zones.held_by("pinky2"), "lifted") is not None
    pilot.silent.add("pinky2")
    blocked_on_the_way(pilot)
    goal = first_goal(pilot)
    pilot.run(1.0)
    m = pilot.mission()
    assert m.block and not m.block.marked
    assert core.reinstate("pinky2", "EMPTY", "back")
    pilot.pose["pinky2"] = pilot.site.waypoint_pose("N")  # pinky1이 쥔 N에 놓는다
    pilot.state["pinky2"] = "IDLE"
    pilot.silent.discard("pinky2")
    pilot.run(0.2)
    assert pilot.events("PLACEMENT_CONFLICT")
    assert [c.goal_id for _, c in pilot.sent("CancelCmd")] == [goal]
    return pilot, goal


def test_a_reinstate_hold_on_a_blocked_holder_waits_for_the_cancel_result():
    """표시 때 goal이 없어도 재투입 충돌의 취소를 막힘이 기다린다."""
    pilot, goal = blocked_holder_held_by_a_reinstate()
    m = pilot.mission()
    assert m.block_cancel == goal
    pilot.run(core_params().blocked_mark_s)
    assert pilot.events("LANE_BLOCKED") and m.block.marked
    canceled_at = pilot.sent("CancelCmd")[0][0]
    assert pilot.held() == {"N", "S"}, "nothing is freed before the cancel result"
    assert not [t for t, _ in pilot.sent("MoveCmd") if t > canceled_at]
    pilot.core.on_move_result("pinky1", goal, False, "CANCELED")
    assert m.block_cancel is None
    pilot.run(0.1 * core_params().release_confirm_samples + 0.2)
    assert pilot.held() == {"N"}, "the usual block rule frees the zone ahead after the result"


def test_a_clear_and_a_placed_robot_before_the_result_send_the_goal_only_after_it():
    pilot, goal = blocked_holder_held_by_a_reinstate()
    m = pilot.mission()
    pilot.state["pinky1"] = "MOVING"  # 물체가 비켰다
    pilot.pose["pinky2"] = pilot.site.waypoint_pose("P-3")  # 빈 자리로 옮겨 올렸다
    pilot.run(0.1 * core_params().release_confirm_samples + 0.5)
    assert m.block is None
    assert pilot.core.traffic.robot_view("pinky2")["on_map"]
    canceled_at = pilot.sent("CancelCmd")[0][0]
    assert not [t for t, _ in pilot.sent("MoveCmd") if t > canceled_at]
    pilot.core.on_move_result("pinky1", goal, False, "CANCELED")
    pilot.run(0.3)
    resent = [c for t, c in pilot.sent("MoveCmd") if t > canceled_at]
    assert resent and resent[0].target_id == "N" and resent[0].goal_id != goal


def test_a_holder_with_no_block_keeps_no_cancel_to_wait_for():
    """막힘이 없는 주인의 취소는 파트 4 전처럼 기다리지 않는다."""
    core, now = cleared()
    core.on_order_board("RUN", [O1])
    for _ in range(20):
        now[0] += 0.5
        core.tick()
        if core.traffic._missions["pinky1"].goal is not None:
            break
    m = core.traffic._missions["pinky1"]
    assert m.goal is not None and m.block is None
    core.reinstate("pinky2", "EMPTY", "back")
    report(core, "pinky2", m.path[m.idx])
    out = core.tick()
    assert any(c.robot == "pinky1" for c in cmds(out, "CancelCmd"))
    assert m.block_cancel is None


def test_a_result_of_another_goal_does_not_end_the_wait():
    pilot, goal = blocked_holder_held_by_a_reinstate()
    m = pilot.mission()
    pilot.core.on_move_result("pinky1", goal + 1000, False, "CANCELED")
    assert m.block_cancel == goal


def test_a_rejected_canceled_goal_ends_the_wait():
    pilot, goal = blocked_holder_held_by_a_reinstate()
    pilot.core.on_goal_rejected("pinky1", goal)
    assert pilot.mission().block_cancel is None


def test_a_canceled_goal_that_was_never_sent_ends_the_wait():
    pilot, goal = blocked_holder_held_by_a_reinstate()
    pilot.core.on_goal_not_sent("pinky1", goal)
    assert pilot.mission().block_cancel is None


def test_a_clear_while_waiting_for_the_result_drops_the_wait():
    pilot, goal = blocked_holder_held_by_a_reinstate()
    core = pilot.core
    m = pilot.mission()
    pilot.run(core_params().blocked_mark_s)
    assert m.block.marked and core.traffic.blocked_lanes() and m.block_cancel == goal
    assert core.confirm_clear("pinky1", core.zones.held_by("pinky1"), "lifted") is not None
    assert m.block_cancel is None and m.block is None
    assert core.traffic.blocked_lanes() == []


def core_params():
    return load_params()


# ---- 자동 격리, 우회와 함께(b84f4d0 병합) ----------------------------------------------


def test_an_isolated_robot_cleared_and_reinstated_takes_its_requeued_order_again():
    """격리(5.5) → confirm_clear → reinstate. 되돌린 주문을 다시 맡고 바구니는 관리자 값이다."""
    from grid_map import grid_site
    from test_auto_isolation import drive as drive_one
    from test_auto_isolation import report as report_one

    now = [0.0]
    core = FleetManager(grid_site(), load_params(), ["r1"], lambda: now[0])
    report_one(core, "r1", (4, 3))
    core.on_order_board("RUN", [O1, O2])
    out = drive_one(core, "r1", lambda: core.robot_view("r1")["order"] == "O-001")
    fill = cmds(out, "FillCmd")[0]
    core.on_fill_result(fill.goal_id, True, {"A": 1}, {})
    seen = []
    for _ in range(100):  # 출고하러 가다 이동 실패 세 번으로 ERROR
        seen += core.tick()
        m = core.traffic._missions["r1"]
        if m.fault:
            break
        if m.goal is not None and m.goal.kind == "move":
            core.on_move_result("r1", m.goal.id, False, "ABORTED")
        now[0] += core.params.move_retry_wait_s + 0.1
        report_one(core, "r1", m.path[m.idx])
    seen += core.tick()
    assert "ROBOT_ISOLATED" in codes(seen)
    assert [e.detail.split()[0] for e in cmds(seen, "EventOut")
            if e.code == "ORDER_REQUEUED"] == ["O-001"]
    assert (core.robot_view("r1")["order"], core.robot_view("r1")["basket"]) == (None, "NONE")
    core.confirm_clear("r1", core.zones.held_by("r1"), "lifted off")
    seen += core.tick()
    assert core.reinstate("r1", "EMPTY", "new basket")
    seen += core.tick()
    view = core.robot_view("r1")
    assert (view["order"], view["basket"]) == (None, "EMPTY")
    report_one(core, "r1", (2, 4))
    seen += drive_one(core, "r1", lambda: core.robot_view("r1")["order"] == "O-001")
    assert codes(seen).count("ORDER_REQUEUED") == 1, "requeued once, by the isolation"
    assert core.robot_view("r1")["basket"] == "EMPTY"


def test_a_holder_stopped_for_a_placement_is_a_standing_robot_with_no_end():
    """재투입 충돌로 세운 주인은 저절로 풀리지 않는다. 우회 판단에서 못 비키는(inf) 로봇이다."""
    import math

    core, now = cleared()
    core.on_order_board("RUN", [O1])
    for _ in range(20):
        now[0] += 0.5
        core.tick()
        if core.traffic._missions["pinky1"].goal is not None:
            break
    m = core.traffic._missions["pinky1"]
    assert m.goal is not None
    core.reinstate("pinky2", "EMPTY", "back")
    report(core, "pinky2", m.path[m.idx])
    core.tick()
    assert not core.traffic.available("pinky1") and m.goal is None
    assert core.traffic._standing_wait_s(m) == math.inf
    report(core, "pinky2", "P-3")  # 빈 자리로 옮겨 올렸다
    assert core.traffic._standing_wait_s(m) != math.inf
