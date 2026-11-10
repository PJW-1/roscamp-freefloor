"""ScenarioJudge: 물리 구역 검사, 주문 수, 작업 순서, 팔, 최종 판정."""
import pytest
from conftest import GRID_MAP
from fake_robots.judge import ScenarioJudge, order_id_of
from fake_robots.scenario import parse_scenario
from fleet_core.traffic.lane_map import load_lane_map

SITE = load_lane_map(GRID_MAP)  # 이 테스트는 작업셀 배치(격자 지도)에서 심판을 시험한다
CELL = 2 * SITE.half_width
NOISE = 0.03
SAME_ZONE = ((9, 1), (9, 0))  # 둘 다 Z-R1
ZONE = "Z-R1"
OTHER = (4, 0)  # Z-R2
OTHER_ZONE = "Z-R2"
WAITING = ((3, 4), (3, 5))  # P-6, P-5
OUT = (9, 3)  # Z-OUT 도크
WORK = (5, 3)  # Z-WORK 도크
LANE = (9, 1)  # 복귀 레인, Z-OUT과 대기 루프 사이


def scenario(faults=(), orders=("O-1", "O-2", "O-3"), **expect):
    return parse_scenario({
        "seed": 1, "speed_factor": 1.0, "timeout_s": 100,
        "stock": {"arm1": {"A": 9}, "arm2": {"C": 9}},
        "orders": [{"at_s": i, "id": o, "items": {"A": 1}} for i, o in enumerate(orders)],
        "robots": {
            "pinky1": {"start": "P-5", "speed_mps": 0.2, "pose_noise_m": NOISE},
            "pinky2": {"start": "P-6", "speed_mps": 0.2, "pose_noise_m": NOISE},
        },
        "faults": list(faults),
        "expect": expect,
    }, SITE)


def judge(**expect):
    return ScenarioJudge(SITE, scenario(**expect), margin_m=0.13, grace_s=1.0)


def at(cell):
    return SITE.waypoint_pose(cell)


def put(j, t, robot, cell, state="MOVING"):
    x, y = at(cell)
    j.on_robot_state(t, robot, x, y, state)


def check(j, name, t=100.0):
    return j.summary(t)["checks"][name]


def event(j, code, detail, t=1.0, source="fleet"):
    j.on_event(t, "INFO", source, code, detail)


def bind(j, *orders):
    for o in orders:
        event(j, "ORDER_BOUND", o, source="pinky1")


def ship(j, *orders):
    for o in orders:
        event(j, "ORDER_SHIPPED", f"{o} robot=pinky1 placed=A:1", source="central")


def finish_everything(j):
    """깨끗한 실행의 이벤트에, 정확 검사가 성립할 만큼의 존재 기록을 더한다."""
    bind(j, "O-1", "O-2", "O-3")
    ship(j, "O-1", "O-2", "O-3")
    put(j, 1.0, "pinky1", WAITING[0], "IDLE")
    put(j, 1.0, "pinky2", WAITING[1], "IDLE")
    j.on_zones(1.0, {"P-6": "pinky1", "P-5": "pinky2"})
    j.tick(1.0)


# -- physical zone checks -----------------------------------------------------------------


def test_two_robots_in_one_zone_break_zone_holder_max():
    j = judge()
    put(j, 1.0, "pinky1", SAME_ZONE[0])
    put(j, 1.0, "pinky2", SAME_ZONE[1])
    j.tick(1.0)
    s = j.summary(1.0)
    assert s["checks"]["zone_holder_max"]["value"] == 2
    assert s["checks"]["zone_holder_max"]["ok"] is False and s["pass"] is False


def test_robots_in_different_zones_are_fine():
    j = judge()
    put(j, 1.0, "pinky1", SAME_ZONE[0])
    put(j, 1.0, "pinky2", OTHER)
    j.on_zones(1.0, {ZONE: "pinky1", OTHER_ZONE: "pinky2"})
    j.tick(2.5)
    assert check(j, "zone_holder_max", 2.5)["value"] == 1


def test_release_while_robot_still_inside_is_counted():
    j = judge()
    put(j, 1.0, "pinky1", SAME_ZONE[0])
    j.on_zones(1.0, {ZONE: "pinky1"})
    j.on_zones(2.0, {ZONE: ""})
    assert check(j, "released_without_position_check", 2.0)["value"] == 1
    assert check(j, "released_without_position_check", 2.0)["ok"] is False


def test_release_after_robot_left_with_noise_allowance_is_fine():
    j = judge()
    put(j, 1.0, "pinky1", SAME_ZONE[0])
    j.on_zones(1.0, {ZONE: "pinky1"})
    # 구역 가장자리에서 0.10 m 밖: margin(0.13) - 2*0.03 = 0.07보다 큼
    x, y = at(SAME_ZONE[0])
    j.on_robot_state(2.0, "pinky1", x + CELL / 2 + 0.10, y, "MOVING")
    j.on_zones(2.0, {ZONE: ""})
    assert check(j, "released_without_position_check", 2.0)["value"] == 0


def test_handover_to_another_robot_also_checks_release():
    j = judge()
    put(j, 1.0, "pinky1", SAME_ZONE[0])
    j.on_zones(1.0, {ZONE: "pinky1"})
    j.on_zones(2.0, {ZONE: "pinky2"})
    assert check(j, "released_without_position_check", 2.0)["value"] == 1


def test_pose_in_unheld_zone_counts_after_grace_only():
    j = judge()
    put(j, 1.0, "pinky1", OTHER)
    j.tick(1.5)
    assert check(j, "unheld_zone_pose", 1.5)["value"] == 0
    j.tick(3.0)
    assert check(j, "unheld_zone_pose", 3.0)["value"] == 1
    assert check(j, "unheld_zone_pose", 3.0)["ok"] is False
    j.tick(4.0)  # 같은 사건이라 다시 세지 않는다
    assert check(j, "unheld_zone_pose", 4.0)["value"] == 1


def test_a_robot_that_has_not_left_its_start_cell_is_not_counted():
    j = judge()
    put(j, 1.0, "pinky1", WAITING[1])  # pinky1은 P-5에서 시작
    j.tick(1.0)
    j.tick(9.0)
    assert check(j, "unheld_zone_pose", 9.0)["value"] == 0


def test_the_start_cell_exemption_ends_when_the_robot_first_moves():
    j = judge()
    put(j, 1.0, "pinky1", WAITING[1])
    j.tick(1.0)
    put(j, 2.0, "pinky1", OTHER)
    j.tick(2.0)
    j.tick(4.0)
    assert check(j, "unheld_zone_pose", 4.0)["value"] == 1
    put(j, 5.0, "pinky1", WAITING[1])  # 시작 칸으로 돌아오면 다른 칸과 똑같이 센다
    j.tick(5.0)
    j.tick(7.0)
    assert check(j, "unheld_zone_pose", 7.0)["value"] == 2


def test_pose_in_held_zone_is_fine():
    j = judge()
    put(j, 1.0, "pinky1", OTHER)
    j.on_zones(1.0, {OTHER_ZONE: "pinky1"})
    j.tick(5.0)
    assert check(j, "unheld_zone_pose", 5.0)["value"] == 0


def test_release_inside_the_zone_is_a_violation_even_with_large_noise():
    noisy = parse_scenario({
        "seed": 1, "speed_factor": 1.0, "timeout_s": 100,
        "stock": {"arm1": {"A": 1}},
        "robots": {"pinky1": {"start": "P-5", "speed_mps": 0.2, "pose_noise_m": 0.10}},
    }, SITE)
    j = ScenarioJudge(SITE, noisy, margin_m=0.13, grace_s=1.0)
    x, y = at(SAME_ZONE[1])
    j.on_robot_state(1.0, "pinky1", x, y, "MOVING")
    j.on_zones(1.0, {ZONE: "pinky1"})
    j.on_zones(2.0, {ZONE: ""})
    assert check(j, "released_without_position_check", 2.0)["value"] == 1


def test_robot_outside_every_zone_is_not_counted_as_occupying_one():
    j = judge()
    x, y = at(SAME_ZONE[0])
    # 칸 바로 왼쪽: 그 칸이 여전히 가장 가깝지만 자세는 구역 밖이다.
    j.on_robot_state(1.0, "pinky1", x - CELL / 2 - 0.02, y, "MOVING")
    put(j, 1.0, "pinky2", SAME_ZONE[0])
    j.tick(1.0)
    assert check(j, "zone_holder_max", 1.0)["value"] == 1


def beyond(cell, gap):
    """`cell`을 가진 구역의 가장자리에서 `gap` m 바깥의 자세."""
    x, y = at(cell)
    return x + CELL / 2 + gap, y


def test_one_noisy_report_cannot_fail_a_release_when_the_robot_really_left():
    """시드 3이 이렇게 실패했다: 로봇이 이미 벗어났는데 보고는 0.05 m 안쪽."""
    j = judge()
    put(j, 1.0, "pinky1", SAME_ZONE[0])
    j.on_zones(1.0, {ZONE: "pinky1"})
    j.on_true_pose("pinky1", *beyond(SAME_ZONE[0], 0.14))
    j.on_robot_state(2.0, "pinky1", *beyond(SAME_ZONE[0], 0.05), "MOVING")
    j.on_zones(2.0, {ZONE: ""})
    assert check(j, "released_without_position_check", 2.0)["value"] == 0


def test_a_good_looking_report_cannot_hide_a_release_while_really_inside():
    j = judge()
    put(j, 1.0, "pinky1", SAME_ZONE[0])
    j.on_zones(1.0, {ZONE: "pinky1"})
    j.on_true_pose("pinky1", *at(SAME_ZONE[0]))
    j.on_robot_state(2.0, "pinky1", *beyond(SAME_ZONE[0], 0.30), "MOVING")
    j.on_zones(2.0, {ZONE: ""})
    assert check(j, "released_without_position_check", 2.0)["value"] == 1


def test_a_robot_with_no_true_pose_is_judged_on_its_reports():
    """실제 로봇에는 /sim/truth 토픽이 없다."""
    j = judge()
    put(j, 1.0, "pinky1", SAME_ZONE[0])
    j.on_zones(1.0, {ZONE: "pinky1"})
    j.on_zones(2.0, {ZONE: ""})
    assert check(j, "released_without_position_check", 2.0)["value"] == 1


def test_occupancy_follows_the_true_position_even_while_reports_are_silent():
    """통신 두절 중에는 실제 위치만 움직인다."""
    j = judge()
    put(j, 1.0, "pinky1", SAME_ZONE[0])
    put(j, 1.0, "pinky2", OTHER)
    j.on_true_pose("pinky1", *at(SAME_ZONE[0]))
    j.on_true_pose("pinky2", *at(SAME_ZONE[1]))  # 같은 구역인데 보고는 아직 OTHER
    j.tick(1.0)
    assert check(j, "zone_holder_max", 1.0)["value"] == 2


def test_summary_and_rows_record_how_far_each_release_really_was():
    j = judge()
    put(j, 1.0, "pinky1", SAME_ZONE[0])
    j.on_zones(1.0, {ZONE: "pinky1"})
    j.on_true_pose("pinky1", *beyond(SAME_ZONE[0], 0.14))
    j.on_robot_state(2.0, "pinky1", *beyond(SAME_ZONE[0], 0.05), "MOVING")
    j.on_zones(2.0, {ZONE: ""})
    t, zone, robot, true_m, reported_m = j.release_rows[0]
    assert (t, zone, robot) == (2.0, ZONE, "pinky1")
    assert true_m == pytest.approx(0.14) and reported_m == pytest.approx(0.05)
    assert j.summary(2.0)["min_release_distance_m"] == pytest.approx(0.14)


def test_summary_has_no_release_distance_when_nothing_was_released():
    assert judge().summary(1.0)["min_release_distance_m"] is None


@pytest.mark.parametrize("gap, violations", [(0.075, 1), (0.09, 0)])
def test_a_true_position_is_held_to_the_robot_body(gap, violations):
    """여유 거리는 보고 잡음을 흡수하려는 것이므로 실제 한계는 반지름이다."""
    j = ScenarioJudge(SITE, scenario(), margin_m=0.13, radius_m=0.08, grace_s=1.0)
    put(j, 1.0, "pinky1", SAME_ZONE[0])
    j.on_zones(1.0, {ZONE: "pinky1"})
    j.on_true_pose("pinky1", *beyond(SAME_ZONE[0], gap))
    j.on_zones(2.0, {ZONE: ""})
    assert check(j, "released_without_position_check", 2.0)["value"] == violations


def test_rows_for_csv_outputs():
    j = judge()
    for i in range(10):
        put(j, i * 0.1, "pinky1", SAME_ZONE[0])
    assert len(j.robot_rows) == 2  # 약 1초 동안 2 Hz로 줄임
    j.on_zones(1.0, {ZONE: "pinky1"})
    j.on_zones(1.2, {ZONE: "pinky1"})
    j.on_zones(2.0, {ZONE: ""})
    assert [(r[1], r[2], r[3]) for r in j.zone_rows] == [
        (ZONE, "", "pinky1"), (ZONE, "pinky1", "")]
    j.on_event(1.0, "INFO", "fleet", "X", "d")
    assert j.event_rows[0][3] == "X"


# -- order counts -------------------------------------------------------------------------


@pytest.mark.parametrize("detail, order", [
    ("O-001", "O-001"), ("O-001 robot=pinky1", "O-001"), ("O-001: back", "O-001"),
    ("  O-U002, x", "O-U002"), ("", ""),
])
def test_order_id_comes_from_the_start_of_the_detail(detail, order):
    assert order_id_of(detail) == order


def test_a_clean_run_passes():
    j = judge(orders_shipped=3, orders_partial=0, orders_held=0, orders_shipped_min=1)
    finish_everything(j)
    s = j.summary(1.0)
    assert s["pass"] is True, s
    assert s["timed_out"] is False
    assert (s["orders_total"], s["orders_shipped"], s["orders_partial"], s["orders_held"]) == (
        3, 3, 0, 0)


@pytest.mark.parametrize("expected", [2, 4])
def test_orders_shipped_must_match_exactly(expected):
    j = judge(orders_shipped=expected)
    finish_everything(j)
    assert check(j, "orders_shipped")["ok"] is False and j.summary(1.0)["pass"] is False


@pytest.mark.parametrize("key, code, expected", [
    ("orders_partial", "ORDER_PARTIAL", 0), ("orders_partial", "ORDER_PARTIAL", 2),
    ("orders_held", "ORDER_HELD", 0), ("orders_held", "ORDER_HELD", 2),
])
def test_orders_partial_and_held_must_match_exactly(key, code, expected):
    j = judge(**{key: expected})
    event(j, code, "O-1 robot=pinky1")
    assert check(j, key)["value"] == 1 and check(j, key)["ok"] is False
    assert j.summary(1.0)["pass"] is False


def test_orders_partial_and_held_pass_when_they_match():
    j = judge(orders_partial=1, orders_held=1)
    event(j, "ORDER_PARTIAL", "O-1 robot=pinky1")
    event(j, "ORDER_HELD", "O-2 robot=pinky1")
    assert check(j, "orders_partial")["ok"] and check(j, "orders_held")["ok"]


def test_a_partial_does_not_count_as_shipped():
    j = judge(orders_shipped=0)
    event(j, "ORDER_PARTIAL", "O-1")
    assert check(j, "orders_shipped")["ok"] is True


def test_orders_shipped_min_fails_below_and_passes_at_the_minimum():
    j = judge(orders_shipped_min=2, all_orders_finished=False)
    ship(j, "O-1")
    assert check(j, "orders_shipped_min")["ok"] is False and j.summary(1.0)["pass"] is False
    ship(j, "O-2")
    assert check(j, "orders_shipped_min")["ok"] is True


def test_held_orders_do_not_satisfy_orders_shipped_min():
    j = judge(orders_shipped_min=1)
    event(j, "ORDER_HELD", "O-1")
    assert check(j, "orders_shipped_min")["ok"] is False


def test_zero_shipped_fails_a_failure_scenario_that_expects_at_least_one():
    faulty = scenario(faults=[{"at_s": 1, "robot": "pinky1", "type": "stuck"}],
                      all_orders_finished=False, orders_shipped_min=1,
                      events_include=["COMM_LOST"])
    j = ScenarioJudge(SITE, faulty, margin_m=0.13, grace_s=1.0)
    put(j, 1.0, "pinky2", WAITING[1], "IDLE")
    j.on_zones(1.0, {"P-5": "pinky2"})
    j.tick(1.0)
    event(j, "COMM_LOST", "no state")
    s = j.summary(100.0)
    assert s["checks"]["orders_shipped_min"]["ok"] is False and s["pass"] is False


def test_all_orders_finished_true_fails_while_one_is_open():
    j = judge()
    ship(j, "O-1", "O-2")
    assert check(j, "all_orders_finished")["ok"] is False


def test_all_orders_finished_false_fails_when_all_finished():
    j = judge(all_orders_finished=False)
    event(j, "ORDER_HELD", "O-1")
    ship(j, "O-2", "O-3")
    assert check(j, "all_orders_finished")["ok"] is False


def test_held_and_partial_count_as_finished():
    j = judge()
    event(j, "ORDER_HELD", "O-1")
    event(j, "ORDER_PARTIAL", "O-2")
    ship(j, "O-3")
    assert check(j, "all_orders_finished")["ok"] is True


def test_a_submitted_order_also_has_to_finish():
    j = judge()
    ship(j, "O-1", "O-2", "O-3")
    event(j, "ORDER_ACCEPTED", "O-U001", source="central")
    assert check(j, "all_orders_finished")["ok"] is False
    ship(j, "O-U001")
    assert check(j, "all_orders_finished")["ok"] is True


def test_events_include_fails_when_a_code_is_missing():
    j = judge(events_include=["COMM_LOST", "ZONE_WAIT_LONG"])
    event(j, "COMM_LOST", "x")
    assert check(j, "events_include") == {"missing": ["ZONE_WAIT_LONG"], "ok": False}
    assert j.summary(1.0)["pass"] is False
    event(j, "ZONE_WAIT_LONG", "x")
    assert check(j, "events_include")["ok"] is True


@pytest.mark.parametrize("seen, ok", [(1, False), (2, True), (3, False)])
def test_event_counts_must_match_exactly(seen, ok):
    j = judge(event_counts={"STOCK_MISMATCH": 2})
    for _ in range(seen):
        event(j, "STOCK_MISMATCH", "O-1 A missing 1, record 1", source="central")
    assert check(j, "event_counts")["ok"] is ok
    assert check(j, "event_counts")["value"] == {"STOCK_MISMATCH": seen}


def test_event_counts_fail_when_the_code_never_appeared():
    assert check(judge(event_counts={"OUT_OF_STOCK": 1}), "event_counts")["ok"] is False


def test_exact_means_exact_for_the_zero_limits_too():
    j = judge(zone_holder_max=2)
    finish_everything(j)  # 이 실행은 구역마다 로봇이 하나였다
    assert check(j, "zone_holder_max")["ok"] is False


# -- fifo_violations ----------------------------------------------------------------------


def test_fifo_clean_run_has_no_violation():
    j = judge()
    bind(j, "O-1", "O-2", "O-3")
    ship(j, "O-1", "O-2", "O-3")
    assert check(j, "fifo_violations")["value"] == 0 and check(j, "fifo_violations")["ok"]


def test_binding_against_registration_order_is_a_violation():
    j = judge()
    bind(j, "O-2", "O-1", "O-3")
    assert check(j, "fifo_violations")["value"] == 1
    assert check(j, "fifo_violations")["ok"] is False and j.summary(1.0)["pass"] is False
    assert any("O-1" in v and "registration" in v for v in j.summary(1.0)["violations"])


def test_shipping_against_bound_order_is_a_violation():
    j = judge()
    bind(j, "O-1", "O-2", "O-3")
    ship(j, "O-2", "O-1", "O-3")
    assert check(j, "fifo_violations")["value"] == 1 and check(j, "fifo_violations")["ok"] is False
    assert any("O-1" in v and "bound order" in v for v in j.summary(1.0)["violations"])


def test_a_partial_counts_in_the_shipping_order():
    j = judge()
    bind(j, "O-1", "O-2")
    event(j, "ORDER_PARTIAL", "O-2 robot=pinky1")
    ship(j, "O-1")
    assert check(j, "fifo_violations")["value"] == 1


def test_a_held_order_does_not_count_in_the_shipping_order():
    j = judge()
    bind(j, "O-1", "O-2", "O-3")
    event(j, "ORDER_HELD", "O-1 robot=pinky1")
    ship(j, "O-2", "O-3")
    assert check(j, "fifo_violations")["value"] == 0


def test_shipping_an_order_that_was_never_bound_is_a_violation():
    j = judge()
    bind(j, "O-1")
    ship(j, "O-1", "O-2")
    assert check(j, "fifo_violations")["value"] == 1


def test_binding_an_order_nobody_registered_is_a_violation():
    j = judge()
    bind(j, "O-999")
    assert check(j, "fifo_violations")["value"] == 1


def test_a_requeued_order_bound_next_is_no_violation():
    j = judge()
    bind(j, "O-1", "O-2")
    event(j, "ORDER_REQUEUED", "O-1 placed: A:1")
    ship(j, "O-2")
    bind(j, "O-1", "O-3")
    ship(j, "O-1", "O-3")
    assert check(j, "fifo_violations")["value"] == 0


def test_only_the_requeued_order_is_excused():
    j = judge()
    bind(j, "O-1", "O-3", "O-2")
    event(j, "ORDER_REQUEUED", "O-1")
    assert check(j, "fifo_violations")["value"] == 1


def test_a_requeued_order_must_be_the_next_one_bound():
    j = judge(orders_shipped=3)
    bind(j, "O-1")
    event(j, "ORDER_REQUEUED", "O-1 was bound")
    bind(j, "O-2", "O-3", "O-1")
    ship(j, "O-2", "O-3", "O-1")
    put(j, 1.0, "pinky1", WAITING[0], "IDLE")
    put(j, 1.0, "pinky2", WAITING[1], "IDLE")
    j.on_zones(1.0, {"P-6": "pinky1", "P-5": "pinky2"})
    j.tick(1.0)
    s = j.summary(5.0)
    assert s["checks"]["fifo_violations"]["value"] == 2, s["violations"]  # O-2, O-3
    assert s["pass"] is False
    assert any("ahead of requeued" in v for v in s["violations"])


def test_requeued_twice_and_bound_next_each_time_is_no_violation():
    j = judge()
    bind(j, "O-1")
    event(j, "ORDER_REQUEUED", "O-1")
    bind(j, "O-1")
    event(j, "ORDER_REQUEUED", "O-1")
    bind(j, "O-1", "O-2", "O-3")
    ship(j, "O-1", "O-2", "O-3")
    assert check(j, "fifo_violations")["value"] == 0


def test_two_requeued_orders_come_back_in_registration_order():
    j = judge()
    bind(j, "O-1", "O-2")
    event(j, "ORDER_REQUEUED", "O-2")
    event(j, "ORDER_REQUEUED", "O-1")
    bind(j, "O-2", "O-1")
    assert check(j, "fifo_violations")["value"] == 1


def test_binding_an_order_twice_without_a_requeue_is_a_violation():
    j = judge(orders_shipped=3)
    bind(j, "O-1", "O-1", "O-2", "O-3")
    ship(j, "O-1", "O-2", "O-3")
    put(j, 1.0, "pinky1", WAITING[0], "IDLE")
    put(j, 1.0, "pinky2", WAITING[1], "IDLE")
    j.on_zones(1.0, {"P-6": "pinky1", "P-5": "pinky2"})
    j.tick(1.0)
    s = j.summary(5.0)
    assert s["checks"]["fifo_violations"]["value"] == 1 and s["pass"] is False
    assert any("without a requeue" in v for v in s["violations"])


def test_binding_a_finished_order_again_is_a_violation():
    j = judge()
    bind(j, "O-1")
    ship(j, "O-1")
    bind(j, "O-1")
    assert check(j, "fifo_violations")["value"] == 1


def test_a_submitted_order_ranks_after_the_scenario_orders():
    j = judge()
    event(j, "ORDER_ACCEPTED", "O-U001", source="central")
    bind(j, "O-U001", "O-1")
    assert check(j, "fifo_violations")["value"] == 1


# -- dual_arm_overlaps --------------------------------------------------------------------


def place(j, arm, start, end):
    j.on_arm_state(start, arm, "PLACING")
    j.on_arm_state(end, arm, "IDLE")


def test_both_arms_placing_at_once_is_an_overlap():
    j = judge()
    place(j, "arm1", 1.0, 3.0)
    place(j, "arm2", 2.0, 4.0)
    assert check(j, "dual_arm_overlaps")["value"] == 1
    assert check(j, "dual_arm_overlaps")["ok"] is False and j.summary(1.0)["pass"] is False
    assert any("PLACING" in v for v in j.summary(1.0)["violations"])


def test_one_arm_after_the_other_is_no_overlap():
    j = judge()
    place(j, "arm1", 1.0, 2.0)
    place(j, "arm2", 2.0, 3.0)  # 같은 스탬프에서 인계
    place(j, "arm1", 3.0, 4.0)
    assert check(j, "dual_arm_overlaps")["value"] == 0


def test_delivery_order_between_the_arms_does_not_matter():
    """arm2의 시작이 arm1의 끝보다 나중에 발행됐어도 먼저 도착할 수 있다."""
    j = judge()
    j.on_arm_state(1.0, "arm1", "PLACING")
    j.on_arm_state(2.0, "arm2", "PLACING")  # 먼저 도착
    j.on_arm_state(2.0, "arm1", "IDLE")
    j.on_arm_state(3.0, "arm2", "IDLE")
    assert check(j, "dual_arm_overlaps")["value"] == 0


def test_an_arm_that_never_left_the_basket_space_still_overlaps():
    j = judge()
    j.on_arm_state(1.0, "arm1", "PLACING")
    j.on_arm_state(2.0, "arm2", "PLACING")
    j.on_arm_state(3.0, "arm2", "PLACING")  # 하트비트이고 arm1은 더 보고하지 않는다
    assert check(j, "dual_arm_overlaps")["value"] == 1


def test_a_heartbeat_does_not_start_a_second_span():
    j = judge()
    j.on_arm_state(1.0, "arm1", "PLACING")
    j.on_arm_state(1.5, "arm1", "PLACING")
    j.on_arm_state(2.0, "arm1", "IDLE")
    place(j, "arm2", 3.0, 4.0)
    assert check(j, "dual_arm_overlaps")["value"] == 0


def test_the_outbound_arm_is_not_part_of_the_overlap():
    j = judge()
    place(j, "arm1", 1.0, 3.0)
    place(j, "arm3", 1.0, 3.0)
    assert check(j, "dual_arm_overlaps")["value"] == 0


# -- laps ---------------------------------------------------------------------------------


def lap(j, robot, cells, start=1.0):
    """주변에 아무도 없이 로봇 하나를 칸마다 한 틱씩 지나가게 한다."""
    t = start
    for cell in cells:
        put(j, t, robot, cell)
        j.tick(t)
        t += 1.0


def test_direct_return_is_counted():
    j = judge(direct_returns_min=1)
    assert check(j, "direct_returns_min")["ok"] is False
    lap(j, "pinky1", [OUT, LANE, WORK])
    assert check(j, "direct_returns_min")["value"] == 1
    assert check(j, "direct_returns_min")["ok"] is True


def test_a_lap_through_the_waiting_loop_is_not_a_direct_return():
    j = judge(direct_returns_min=1)
    lap(j, "pinky1", [OUT, LANE, WAITING[0], WORK])
    assert check(j, "direct_returns_min")["value"] == 0
    assert check(j, "direct_returns_min")["ok"] is False


def test_leaving_the_waiting_loop_for_work_is_not_a_direct_return():
    j = judge(direct_returns_min=1)
    lap(j, "pinky1", [WAITING[0], WORK])
    assert check(j, "direct_returns_min")["ok"] is False


def test_entering_the_waiting_loop_with_a_backlog_is_a_violation():
    j = judge()
    j.on_board(["O-1", "O-2"])  # 미배정 주문 둘, 로봇 둘: 둘 다 호출됐어야 한다
    lap(j, "pinky1", [OUT, LANE, WAITING[0]])
    assert check(j, "parked_while_backlog")["value"] == 1
    assert check(j, "parked_while_backlog")["ok"] is False and j.summary(9.0)["pass"] is False


def test_entering_the_waiting_loop_with_fewer_orders_than_robots_is_allowed():
    j = judge()
    j.on_board(["O-1"])  # 다른 로봇이 유일한 호출을 쥐고 있을 수 있다
    lap(j, "pinky1", [OUT, LANE, WAITING[0]])
    assert check(j, "parked_while_backlog")["value"] == 0


def test_bound_orders_are_not_a_backlog():
    j = judge()
    bind(j, "O-1", "O-2")
    j.on_board(["O-1", "O-2"])
    lap(j, "pinky1", [OUT, LANE, WAITING[0]])
    assert check(j, "parked_while_backlog")["value"] == 0


def test_a_requeued_order_is_a_backlog_again():
    j = judge()
    bind(j, "O-1", "O-2")
    event(j, "ORDER_REQUEUED", "O-1")
    event(j, "ORDER_REQUEUED", "O-2")
    j.on_board(["O-1", "O-2"])
    lap(j, "pinky1", [OUT, LANE, WAITING[0]])
    assert check(j, "parked_while_backlog")["value"] == 1


def test_a_robot_that_starts_in_the_waiting_loop_is_no_violation():
    j = judge()
    j.on_board(["O-1", "O-2"])
    lap(j, "pinky1", [WAITING[0], WAITING[1]])
    assert check(j, "parked_while_backlog")["value"] == 0


# -- outcome ------------------------------------------------------------------------------


def test_finish_needs_all_orders_final_and_robots_parked():
    j = judge()
    put(j, 1.0, "pinky1", WAITING[0], "IDLE")
    put(j, 1.0, "pinky2", WAITING[1], "IDLE")
    ship(j, "O-1", "O-2")
    assert not j.finished(1.0)
    ship(j, "O-3")
    assert j.finished(2.0)
    put(j, 3.0, "pinky2", OTHER, "MOVING")
    assert not j.finished(3.0)


def test_a_docked_robot_is_not_parked():
    j = judge()
    put(j, 1.0, "pinky1", WAITING[0], "IDLE")
    put(j, 1.0, "pinky2", OUT, "IDLE")
    ship(j, "O-1", "O-2", "O-3")
    assert not j.finished(2.0)


def test_timeout_finishes_and_fails_when_finishing_was_expected():
    j = judge()
    assert j.finished(100.0)
    s = j.summary(100.0)
    assert s["pass"] is False and s["timed_out"] is True and s["orders_shipped"] == 0


@pytest.mark.parametrize("kind", ["stuck", "operator_remove"])
def test_finish_does_not_wait_for_a_robot_that_is_gone(kind):
    gone = scenario(faults=[{"at_s": 1, "robot": "pinky1", "type": kind}],
                    orders_shipped_min=1)
    j = ScenarioJudge(SITE, gone, margin_m=0.13, grace_s=1.0)
    put(j, 1.0, "pinky2", WAITING[1], "IDLE")
    ship(j, "O-1", "O-2", "O-3")
    assert j.finished(2.0)


REINSTATED = [{"at_s": 1, "robot": "pinky1", "type": "operator_remove"},
              {"at_s": 5, "robot": "pinky1", "type": "operator_reinstate", "start": "P-5"}]


def test_finish_waits_for_a_robot_that_was_put_back():
    j = ScenarioJudge(SITE, scenario(faults=REINSTATED, orders_shipped_min=1),
                      margin_m=0.13, grace_s=1.0)
    put(j, 1.0, "pinky2", WAITING[1], "IDLE")
    ship(j, "O-1", "O-2", "O-3")
    assert not j.finished(2.0)
    put(j, 2.0, "pinky1", WAITING[0], "IDLE")
    assert j.finished(2.0)


def _put_back_off_its_start(t_back):
    j = ScenarioJudge(SITE, scenario(faults=REINSTATED, orders_shipped_min=1),
                      margin_m=0.13, grace_s=1.0)
    j.on_true_pose("pinky1", *at(OTHER))  # 출발 칸을 떠났다
    j.tick(0.0)
    j.on_zones(0.0, {OTHER_ZONE: "pinky1"})
    j.on_true_pose("pinky1", *at(WAITING[0]))
    j.on_zones(t_back, {OTHER_ZONE: ""})
    j.put_back(t_back, "pinky1", WAITING[0])
    return j


def test_a_robot_put_back_has_a_short_while_to_be_placed():
    """매니저가 다음 보고를 들어야 잡는다. 그동안만 면제한다."""
    j = _put_back_off_its_start(1.0)
    j.tick(1.0)
    j.tick(1.9)
    j.on_zones(1.9, {"P-6": "pinky1"})
    for t in (3.0, 5.0, 9.0):
        j.tick(t)
    assert check(j, "unheld_zone_pose")["value"] == 0


def test_a_robot_put_back_that_is_never_placed_is_counted():
    """서비스 거부나 PLACEMENT_CONFLICT로 잠금을 못 얻으면 놓치지 않는다(리뷰 4번)."""
    j = _put_back_off_its_start(1.0)
    for t in (1.0, 2.0, 3.0, 4.0, 5.0):
        j.tick(t)
    assert check(j, "unheld_zone_pose")["value"] == 1


def test_a_robot_put_back_unplaced_is_counted_when_its_grace_ends():
    """2차 리뷰 4번: 유예가 끝난 뒤 다시 grace_s를 기다리지 않는다."""
    j = _put_back_off_its_start(1.0)
    for t in (1.0, 1.5, 1.99):
        j.tick(t)
    assert check(j, "unheld_zone_pose")["value"] == 0
    j.tick(2.0)
    assert check(j, "unheld_zone_pose")["value"] == 1


def test_a_robot_put_back_on_its_own_start_cell_gets_no_start_exemption():
    """출발 칸을 떠나는 것을 못 본 채 치웠다가 같은 칸에 다시 놓아도 면제는 유예뿐이다."""
    j = ScenarioJudge(SITE, scenario(faults=REINSTATED, orders_shipped_min=1),
                      margin_m=0.13, grace_s=1.0)
    j.on_true_pose("pinky1", *at(WAITING[1]))  # 시작 칸 P-5
    j.tick(0.0)
    j.put_back(1.0, "pinky1", WAITING[1])
    for t in (1.0, 2.0, 3.0, 9.0):
        j.tick(t)
    assert check(j, "unheld_zone_pose")["value"] == 1


def test_orders_bound_by_a_robot_after_its_reinstate_are_counted():
    j = ScenarioJudge(SITE, scenario(faults=REINSTATED, orders_shipped_min=1,
                                     bound_after_reinstate_min=1), margin_m=0.13, grace_s=1.0)
    event(j, "ORDER_BOUND", "O-1", t=1.0, source="pinky1")
    event(j, "ROBOT_REINSTATED", "basket EMPTY", t=2.0, source="pinky2")
    event(j, "ORDER_BOUND", "O-2", t=3.0, source="pinky1")
    assert check(j, "bound_after_reinstate_min") == {"value": 0, "minimum": 1, "ok": False}
    event(j, "ROBOT_REINSTATED", "basket EMPTY", t=4.0, source="pinky1")
    event(j, "ORDER_BOUND", "O-3", t=5.0, source="pinky1")
    assert check(j, "bound_after_reinstate_min") == {"value": 1, "minimum": 1, "ok": True}


def reinstated_judge():
    j = ScenarioJudge(SITE, scenario(faults=REINSTATED, orders_shipped_min=1,
                                     shipped_after_reinstate_min=1), margin_m=0.13, grace_s=1.0)
    event(j, "ROBOT_REINSTATED", "basket EMPTY", t=2.0, source="pinky2")
    return j


def test_an_order_bound_after_a_reinstate_but_requeued_and_shipped_by_another_does_not_count():
    j = reinstated_judge()
    event(j, "ORDER_BOUND", "O-1", t=3.0, source="pinky2")
    event(j, "ORDER_REQUEUED", "O-1 was bound, basket held {}", t=4.0, source="pinky2")
    event(j, "ORDER_BOUND", "O-1", t=5.0, source="pinky1")
    event(j, "ORDER_SHIPPED", "O-1", t=6.0)
    assert check(j, "shipped_after_reinstate_min") == {"value": 0, "minimum": 1, "ok": False}
    assert check(j, "bound_after_reinstate_min")["value"] == 1  # 기존 판정은 그대로


def test_an_order_bound_after_a_reinstate_and_shipped_counts():
    j = reinstated_judge()
    event(j, "ORDER_BOUND", "O-1", t=3.0, source="pinky2")
    event(j, "ORDER_SHIPPED", "O-1", t=6.0)
    assert check(j, "shipped_after_reinstate_min") == {"value": 1, "minimum": 1, "ok": True}


def test_an_order_bound_before_the_clear_does_not_count():
    j = ScenarioJudge(SITE, scenario(faults=REINSTATED, orders_shipped_min=1,
                                     shipped_after_reinstate_min=1), margin_m=0.13, grace_s=1.0)
    event(j, "ORDER_BOUND", "O-1", t=0.5, source="pinky2")
    event(j, "ROBOT_REINSTATED", "basket EMPTY", t=2.0, source="pinky2")
    event(j, "ORDER_SHIPPED", "O-1", t=6.0)
    assert check(j, "shipped_after_reinstate_min") == {"value": 0, "minimum": 1, "ok": False}


def test_a_held_order_bound_after_a_reinstate_does_not_count():
    j = reinstated_judge()
    event(j, "ORDER_BOUND", "O-1", t=3.0, source="pinky2")
    event(j, "ORDER_HELD", "O-1", t=6.0)
    assert check(j, "shipped_after_reinstate_min")["value"] == 0


def test_an_order_rebound_by_another_reinstated_robot_counts_for_neither_twice():
    """두 대 모두 재투입. A가 묶은 주문을 B가 되돌림 없이 다시 묶으면 A 몫이 아니다."""
    j = reinstated_judge()
    event(j, "ROBOT_REINSTATED", "basket EMPTY", t=2.5, source="pinky1")
    event(j, "ORDER_BOUND", "O-1", t=3.0, source="pinky2")
    event(j, "ORDER_BOUND", "O-1", t=4.0, source="pinky1")
    event(j, "ORDER_SHIPPED", "O-1", t=6.0)
    assert check(j, "shipped_after_reinstate_min")["value"] == 1  # 끝낸 pinky1 몫 하나


def test_a_final_event_twice_counts_once():
    j = reinstated_judge()
    event(j, "ORDER_BOUND", "O-1", t=3.0, source="pinky2")
    event(j, "ORDER_HELD", "O-1", t=5.0)
    event(j, "ORDER_SHIPPED", "O-1", t=6.0)
    event(j, "ORDER_SHIPPED", "O-1", t=7.0)
    assert check(j, "shipped_after_reinstate_min")["value"] == 0


def test_a_partial_order_bound_after_a_reinstate_counts():
    j = reinstated_judge()
    event(j, "ORDER_BOUND", "O-1", t=3.0, source="pinky2")
    event(j, "ORDER_PARTIAL", "O-1", t=6.0)
    event(j, "ORDER_SHIPPED", "O-1", t=7.0)
    assert check(j, "shipped_after_reinstate_min")["value"] == 1


def test_an_order_bound_between_a_second_clear_and_reinstate_does_not_count():
    j = reinstated_judge()
    event(j, "ZONES_CLEARED", "lifted", t=3.0, source="pinky2")
    event(j, "ORDER_BOUND", "O-1", t=3.5, source="pinky2")
    event(j, "ORDER_SHIPPED", "O-1", t=6.0)
    assert check(j, "shipped_after_reinstate_min")["value"] == 0
    assert check(j, "bound_after_reinstate_min")["value"] == 1  # 기존 판정은 그대로


def test_finish_waits_for_a_robot_after_a_comm_loss():
    lost = scenario(faults=[{"at_s": 1, "robot": "pinky1", "type": "comm_loss", "duration_s": 5}],
                    orders_shipped_min=1)
    j = ScenarioJudge(SITE, lost, margin_m=0.13, grace_s=1.0)
    put(j, 1.0, "pinky2", WAITING[1], "IDLE")
    ship(j, "O-1", "O-2", "O-3")
    assert not j.finished(2.0)


def test_summary_reports_duration():
    assert judge().summary(42.0)["duration_s"] == pytest.approx(42.0)


def test_off_course_needs_a_known_pose_clear_of_every_zone():
    j = judge()
    assert j.off_course("pinky1") is False
    put(j, 1.0, "pinky1", SAME_ZONE[0])
    assert j.off_course("pinky1") is False
    j.on_true_pose("pinky1", -1.0, -1.0)
    assert j.off_course("pinky1") is True


def test_held_by_lists_only_the_robots_own_zones():
    j = judge()
    j.on_zones(1.0, {ZONE: "pinky1", OTHER_ZONE: "pinky1", "P-6": "pinky2", "P-5": ""})
    assert j.held_by("pinky1") == sorted([ZONE, OTHER_ZONE])
    assert j.held_by("pinky3") == []


# -- timed_out, unexpected_errors, order_results ------------------------------------------


def clean_run(j):
    finish_everything(j)
    j.tick(1.0)
    return j


def test_a_timeout_fails_even_with_every_other_check_good():
    j = clean_run(judge())
    put(j, 1.0, "pinky1", LANE, "MOVING")  # 모두 출고됐지만 이 로봇은 돌아오지 못했다
    s = j.summary(100.0)
    failed = [k for k, c in s["checks"].items() if not c["ok"]]
    assert failed == ["timed_out"] and s["timed_out"] is True and s["pass"] is False


def test_a_declared_timeout_passes_and_a_missing_one_fails():
    j = clean_run(judge(timed_out=True))
    put(j, 1.0, "pinky1", LANE, "MOVING")
    assert check(j, "timed_out")["ok"] is True
    j = clean_run(judge(timed_out=True))
    assert check(j, "timed_out")["ok"] is False, "it was meant to run out of time and did not"


def test_an_unexpected_error_event_fails_the_run():
    j = clean_run(judge())
    assert j.summary(5.0)["pass"] is True
    j.on_event(2.0, "ERROR", "fleet_manager", "DEADLOCK_DETECTED", "wait cycle")
    s = j.summary(5.0)
    assert s["checks"]["unexpected_errors"] == {
        "value": 1, "expected": 0, "codes": ["DEADLOCK_DETECTED"], "ok": False}
    assert s["pass"] is False
    assert any("DEADLOCK_DETECTED" in v for v in s["violations"])


@pytest.mark.parametrize("expect", [{"events_include": ["FILL_FAILED"]},
                                    {"event_counts": {"FILL_FAILED": 1}}])
def test_an_error_the_scenario_names_is_not_unexpected(expect):
    j = clean_run(judge(**expect))
    j.on_event(2.0, "ERROR", "pinky1", "FILL_FAILED", "workcell: ARM_FAULT")
    assert check(j, "unexpected_errors")["ok"] is True
    assert j.summary(5.0)["pass"] is True


def test_warnings_are_not_unexpected_errors():
    j = clean_run(judge())
    j.on_event(2.0, "WARN", "pinky1", "GOAL_RETRY", "attempt 1")
    assert check(j, "unexpected_errors")["value"] == 0


def test_right_counts_on_the_wrong_orders_fail():
    j = judge(orders_shipped=2, orders_held=1,
              order_results={"O-1": "HELD", "O-2": "SHIPPED", "O-3": "SHIPPED"})
    bind(j, "O-1", "O-2", "O-3")
    ship(j, "O-1", "O-2")
    event(j, "ORDER_HELD", "O-3 robot=pinky1", source="central")
    put(j, 1.0, "pinky1", WAITING[0], "IDLE")
    put(j, 1.0, "pinky2", WAITING[1], "IDLE")
    j.on_zones(1.0, {"P-6": "pinky1", "P-5": "pinky2"})
    j.tick(1.0)
    s = j.summary(5.0)
    assert [k for k, c in s["checks"].items() if not c["ok"]] == ["order_results"]
    assert s["pass"] is False


def test_order_results_that_match_pass():
    j = clean_run(judge(order_results={"O-1": "SHIPPED", "O-2": "SHIPPED", "O-3": "SHIPPED"}))
    assert check(j, "order_results")["ok"] is True and j.summary(5.0)["pass"] is True


def test_an_order_results_entry_that_never_finished_fails():
    j = judge(order_results={"O-1": "SHIPPED"})
    assert check(j, "order_results")["ok"] is False


# -- no arm data --------------------------------------------------------------------------


def both_arm_judge():
    sc = parse_scenario({
        "seed": 1, "speed_factor": 1.0, "timeout_s": 100,
        "stock": {"arm1": {"A": 9}, "arm2": {"C": 9}},
        "orders": [{"at_s": 0, "id": "O-1", "items": {"A": 1, "C": 1}}],
        "robots": {"pinky1": {"start": "P-5"}, "pinky2": {"start": "P-6"}},
    }, SITE)
    j = ScenarioJudge(SITE, sc, margin_m=0.13, grace_s=1.0)
    bind(j, "O-1")
    ship(j, "O-1")
    put(j, 1.0, "pinky1", WAITING[0], "IDLE")
    put(j, 1.0, "pinky2", WAITING[1], "IDLE")
    j.on_zones(1.0, {"P-6": "pinky1", "P-5": "pinky2"})
    j.tick(1.0)
    return j


def placed(j, arm, start):
    j.on_arm_state(start, arm, "PLACING")
    j.on_arm_state(start + 1.0, arm, "IDLE")


def test_no_arm_data_fails_when_the_shipped_orders_needed_both_arms():
    j = both_arm_judge()
    s = j.summary(5.0)
    c = s["checks"]["dual_arm_overlaps"]
    assert c["value"] == 0 and c["arms_never_placing"] == ["arm1", "arm2"] and c["ok"] is False
    assert s["pass"] is False


def test_one_arm_never_placing_fails_too():
    j = both_arm_judge()
    placed(j, "arm1", 1.0)
    assert check(j, "dual_arm_overlaps")["arms_never_placing"] == ["arm2"]
    assert j.summary(5.0)["pass"] is False


def test_both_arms_seen_placing_without_overlap_passes():
    j = both_arm_judge()
    placed(j, "arm1", 1.0)
    placed(j, "arm2", 3.0)
    assert check(j, "dual_arm_overlaps")["ok"] is True and j.summary(5.0)["pass"] is True


def test_orders_for_one_arm_do_not_need_the_other():
    j = clean_run(judge())  # 이 시나리오의 주문은 모두 arm1의 물건 A
    assert check(j, "dual_arm_overlaps")["arms_never_placing"] == []


# -- 레인 이탈 중 해제 ---------------------------------------------------------------------


def drifting(**expect):
    sc = scenario(faults=[{"at_s": 10, "robot": "pinky1", "type": "drift", "duration_s": 5,
                           "offset_m": 0.3}], orders_shipped_min=1, **expect)
    return ScenarioJudge(SITE, sc, margin_m=0.13, grace_s=1.0)


def release_far(j, t, robot="pinky1"):
    """몸이 확실히 벗어난 뒤의 해제. 다른 검사에는 걸리지 않는다."""
    j.on_true_pose(robot, *at(OTHER))
    j.on_zones(t - 0.5, {ZONE: robot})
    j.on_zones(t, {ZONE: ""})


@pytest.mark.parametrize("t, robot, counted", [
    (10.5, "pinky1", 0),  # 아직 FMS가 알아챌 수 없다
    (12.0, "pinky1", 1),
    (14.9, "pinky1", 1),
    (15.5, "pinky1", 0),  # 이탈이 끝났다
    (12.0, "pinky2", 0),
])
def test_a_release_from_a_drifting_robot_is_counted(t, robot, counted):
    j = drifting()
    release_far(j, t, robot)
    result = check(j, "released_while_off_lane", t)
    assert result["value"] == counted
    assert result["ok"] is (counted == 0)
    assert check(j, "released_without_position_check", t)["value"] == 0
# ---- 자동 격리(지시서 5.5) -------------------------------------------------------------------


def test_shipped_before_operator_counts_from_the_isolation_to_the_first_clear():
    j = judge(shipped_before_operator_min=1, all_orders_finished=False)
    ship(j, "O-1")  # 격리 전
    assert check(j, "shipped_before_operator")["value"] == 0
    j.on_event(2.0, "ERROR", "fleet", "ROBOT_ISOLATED", "GRIPPER: stopped where it is")
    assert check(j, "shipped_before_operator")["ok"] is False
    event(j, "ORDER_PARTIAL", "O-2", t=3.0)
    event(j, "ZONES_CLEARED", "released ['Z-WORK']: lifted", t=4.0)
    ship(j, "O-3")  # 사람이 온 뒤
    result = check(j, "shipped_before_operator")
    assert (result["value"], result["minimum"], result["ok"]) == (1, 1, True)


def test_clearing_a_robot_that_is_not_isolated_does_not_end_the_count():
    j = judge(shipped_before_operator_min=2, all_orders_finished=False)
    j.on_event(1.0, "ERROR", "pinky1", "ROBOT_ISOLATED", "GRIPPER: stopped where it is")
    j.on_event(2.0, "INFO", "pinky2", "ZONES_CLEARED", "released ['Z-R1']: lifted")
    ship(j, "O-1")
    j.on_event(3.0, "INFO", "pinky1", "ZONES_CLEARED", "released ['Z-WORK']: lifted")
    ship(j, "O-2")
    assert check(j, "shipped_before_operator")["value"] == 1


def test_shipped_before_operator_is_not_checked_by_default():
    j = judge()
    assert check(j, "shipped_before_operator") == {"value": 0, "minimum": 0, "ok": True}


def test_detours_around_isolated_count_only_detours_naming_an_isolated_robot():
    j = judge(detours_around_isolated_min=1, all_orders_finished=False)
    detour = "avoids F1 held by pinky1, standing inf s; new path F0 T2 T1 QW W"
    j.on_event(1.0, "INFO", "pinky2", "DETOUR", detour)  # 격리 전
    j.on_event(2.0, "INFO", "pinky2", "DETOUR", "avoids lane F1 to F2 blocked; new path F0 T2")
    assert check(j, "detours_around_isolated")["ok"] is False
    j.on_event(3.0, "ERROR", "pinky1", "ROBOT_ISOLATED", "NAV_ABORTED: stopped where it is")
    j.on_event(4.0, "INFO", "pinky2", "DETOUR", "avoids F1 held by pinky3, standing 16 s; new path")
    assert check(j, "detours_around_isolated")["value"] == 0
    j.on_event(5.0, "INFO", "pinky2", "DETOUR", detour)
    j.on_event(6.0, "INFO", "pinky1", "ZONES_CLEARED", "released ['F1', 'F2']: lifted")
    j.on_event(7.0, "INFO", "pinky3", "DETOUR", detour)  # 치운 뒤
    result = check(j, "detours_around_isolated")
    assert (result["value"], result["minimum"], result["ok"]) == (1, 1, True)


def test_detours_around_isolated_is_not_checked_by_default():
    j = judge()
    assert check(j, "detours_around_isolated") == {"value": 0, "minimum": 0, "ok": True}


def test_detours_around_isolated_see_every_holder_once_per_event():
    j = judge(detours_around_isolated_min=1, all_orders_finished=False)
    j.on_event(1.0, "ERROR", "pinky1", "ROBOT_ISOLATED", "NAV_ABORTED: stopped where it is")
    j.on_event(2.0, "INFO", "pinky2", "DETOUR",
               "avoids F1 held by pinky3, standing 16 s; "
               "F2 held by pinky1, standing inf s; new path")
    assert check(j, "detours_around_isolated")["value"] == 1
    j.on_event(3.0, "ERROR", "pinky3", "ROBOT_ISOLATED", "NAV_ABORTED: stopped where it is")
    j.on_event(4.0, "INFO", "pinky2", "DETOUR",
               "avoids F1 held by pinky3, standing inf s; "
               "F2 held by pinky1, standing inf s; new path")
    assert check(j, "detours_around_isolated")["value"] == 2



# ---- 경로 연결 확인(지시서 7.5) ----------------------------------------------------------


def test_a_path_not_connected_failure_is_counted_and_none_is_expected_by_default():
    j = judge()
    assert check(j, "path_not_connected") == {"value": 0, "expected": 0, "ok": True}
    j.on_event(2.0, "WARN", "pinky1", "GOAL_RETRY", "attempt 1 failed: PATH_NOT_CONNECTED")
    j.on_event(3.0, "WARN", "pinky1", "GOAL_RETRY", "attempt 1 failed: NAV_ABORTED")
    j.on_event(9.0, "ERROR", "pinky2", "GOAL_FAILED",
               "gave up after 4 attempts: PATH_NOT_CONNECTED")
    result = check(j, "path_not_connected")
    assert (result["value"], result["ok"]) == (2, False)


def test_robot_idle_is_counted_and_none_is_expected_by_default():
    """1.1 지시서 H2: 정상 운행에서 ROBOT_IDLE이 나면 오탐이다. 고장 시나리오는 null로 안 본다."""
    j = judge()
    assert check(j, "robot_idle") == {"value": 0, "expected": 0, "ok": True}
    j.on_event(2.0, "WARN", "pinky1", "ROBOT_IDLE", "TO_PARK at F0 for 10 s")
    assert check(j, "robot_idle") == {"value": 1, "expected": 0, "ok": False}
    j = judge(robot_idle=None)
    j.on_event(2.0, "WARN", "pinky1", "ROBOT_IDLE", "TO_PARK at F0 for 10 s")
    assert check(j, "robot_idle")["ok"] is True


def test_parked_while_backlog_null_is_not_checked():
    """충전하러 가는 로봇은 주문이 밀려도 WAIT로 갈 수 있다. null이면 안 본다(robot_idle처럼)."""
    j = judge(parked_while_backlog=None)
    j.on_board(["O-1", "O-2"])
    lap(j, "pinky1", [OUT, LANE, WAITING[0]])
    result = check(j, "parked_while_backlog")
    assert (result["value"], result["expected"], result["ok"]) == (1, None, True)


def test_a_scenario_can_expect_path_not_connected_failures():
    j = judge(path_not_connected=1)
    j.on_event(2.0, "WARN", "pinky1", "GOAL_RETRY", "attempt 1 failed: PATH_NOT_CONNECTED")
    assert check(j, "path_not_connected")["ok"] is True
