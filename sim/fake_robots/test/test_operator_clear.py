"""운영자 치우기·재투입 판단(scenario_monitor_node). ROS 없이 돈다.

치울 때의 심판 스냅샷과 관제의 실제 보유가 어긋나면(경쟁) 남은 구역을 다시 치운다.
"""
from fake_robots.operator_clear import OperatorClear


def removed(held=("W",), max_clears=5):
    op = OperatorClear(max_clears=max_clears)
    op.start("pinky2")
    assert op.next_clear("pinky2", list(held)) == sorted(held)
    return op


def test_one_clear_with_the_snapshot_then_reinstate_when_nothing_is_left():
    op = removed()
    assert not op.ready("pinky2", ["W"])
    op.on_cleared("pinky2", True, ["W"])
    assert op.next_clear("pinky2", []) is None
    assert op.ready("pinky2", [])


def test_a_zone_still_held_by_the_event_is_cleared_again():
    op = removed()
    op.on_cleared("pinky2", True, ["W"])
    op.on_event("pinky2", "ZONES_STILL_HELD", "['AVENUE'] stay locked")
    assert not op.ready("pinky2", [])
    assert op.next_clear("pinky2", []) == ["AVENUE"]
    op.on_cleared("pinky2", True, ["AVENUE"])
    assert op.next_clear("pinky2", []) is None
    assert op.ready("pinky2", [])


def test_a_zone_the_judge_still_shows_held_is_cleared_again():
    op = removed()
    op.on_cleared("pinky2", True, ["W"])
    assert not op.ready("pinky2", ["AVENUE"])
    assert op.next_clear("pinky2", ["AVENUE"]) == ["AVENUE"]
    op.on_cleared("pinky2", True, ["AVENUE"])
    assert op.ready("pinky2", [])


def test_no_second_clear_while_one_is_in_flight():
    op = removed()
    assert op.next_clear("pinky2", ["W", "AVENUE"]) is None
    assert not op.ready("pinky2", [])


def test_a_released_zone_the_judge_has_not_caught_up_with_is_not_sent_again():
    """관제는 이미 풀린 구역도 받아들이고 released에서 뺀다(recovery.py). 보낼 까닭이 없다."""
    op = removed()
    op.on_cleared("pinky2", True, ["W"])
    assert op.next_clear("pinky2", ["W"]) is None
    assert op.ready("pinky2", ["W"])


def test_a_late_still_held_event_for_a_zone_already_cleared_again_is_ignored():
    op = removed()
    op.on_cleared("pinky2", True, ["W"])
    assert op.next_clear("pinky2", ["AVENUE"]) == ["AVENUE"]
    op.on_cleared("pinky2", True, ["AVENUE"])
    op.on_event("pinky2", "ZONES_STILL_HELD", "['AVENUE'] stay locked")
    assert op.next_clear("pinky2", []) is None
    assert op.ready("pinky2", [])


def test_another_robots_event_does_not_count():
    op = removed()
    op.on_cleared("pinky2", True, ["W"])
    op.on_event("pinky1", "ZONES_STILL_HELD", "['AVENUE'] stay locked")
    assert op.ready("pinky2", [])


def test_a_refused_clear_is_tried_again_with_what_the_judge_shows():
    """빈 목록은 쥔 구역이 있으면 거절된다. 심판이 보이면 그 구역으로 다시 부른다."""
    op = removed(held=())
    op.on_cleared("pinky2", False, [])
    assert not op.ready("pinky2", [])
    assert op.next_clear("pinky2", ["W"]) == ["W"]
    op.on_cleared("pinky2", True, ["W"])
    assert op.ready("pinky2", [])


def test_clearing_again_stops_at_the_limit_and_never_reinstates():
    """받아들여진 치우기의 구역은 끝난 것이다. 거절이 이어지면 상한에서 멈춘다."""
    op = removed(held=("AVENUE",), max_clears=3)
    for _ in range(2):
        op.on_cleared("pinky2", False, [])
        assert op.next_clear("pinky2", ["AVENUE"]) == ["AVENUE"]
    op.on_cleared("pinky2", False, [])
    assert not op.gave_up("pinky2")
    assert op.next_clear("pinky2", ["AVENUE"]) is None
    assert op.gave_up("pinky2")
    assert not op.gave_up("pinky2"), "said once"
    assert not op.ready("pinky2", ["AVENUE"])


def test_a_reinstate_ends_the_removal():
    op = removed()
    op.on_cleared("pinky2", True, ["W"])
    op.reinstated("pinky2")
    assert not op.ready("pinky2", [])
    assert op.next_clear("pinky2", ["AVENUE"]) is None


def test_no_reinstate_until_a_late_still_held_event_has_had_time_to_come():
    """응답이 ZONES_STILL_HELD보다 먼저 와도 잠시 기다린 뒤에만 재투입한다(Codex)."""
    op = removed()
    op.on_cleared("pinky2", True, ["W"], t=40.0)
    assert not op.ready("pinky2", [], t=40.1)
    op.on_event("pinky2", "ZONES_STILL_HELD", "['AVENUE'] stay locked")
    assert op.next_clear("pinky2", []) == ["AVENUE"]
    op.on_cleared("pinky2", True, ["AVENUE"], t=40.3)
    assert not op.ready("pinky2", [], t=40.3 + op.settle_s - 0.01)
    assert op.ready("pinky2", [], t=40.3 + op.settle_s)


def test_a_zone_sent_in_an_accepted_clear_but_already_free_is_not_sent_again():
    """관제는 이미 풀린 구역을 받아들이되 released에 넣지 않는다. 늦은 심판 스냅샷에 남아도 끝."""
    op = removed(held=("W", "AVENUE"))
    op.on_cleared("pinky2", True, ["W"])
    assert op.next_clear("pinky2", ["AVENUE"]) is None
    assert op.ready("pinky2", ["AVENUE"])


def test_a_second_removal_while_a_clear_is_in_flight_keeps_waiting_for_it():
    op = removed()
    op.start("pinky2")
    assert op.next_clear("pinky2", ["W"]) is None, "the first clear is still in flight"
    op.on_cleared("pinky2", True, ["W"])
    assert op.ready("pinky2", [])
