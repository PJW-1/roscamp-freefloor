"""운영자 치우기·재투입 판단(scenario_monitor_node). ROS 없이 돈다.

치울 때의 심판 스냅샷과 관제의 실제 보유가 어긋나면(경쟁) 남은 구역을 다시 치운다.
"""
from fake_robots.operator_clear import OperatorClear


def removed(held=("W",), max_clears=5):
    op = OperatorClear(max_clears=max_clears)
    op.start("pinky2")
    assert op.next_clear("pinky2", list(held)) == list(held)
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
    op = removed(max_clears=3)
    for _ in range(2):
        op.on_cleared("pinky2", True, [])
        assert op.next_clear("pinky2", ["AVENUE"]) == ["AVENUE"]
    op.on_cleared("pinky2", True, [])
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
