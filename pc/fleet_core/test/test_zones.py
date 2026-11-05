"""Zone lock records: one holder per zone, FIFO queue, nothing else may change them."""
import pytest
from fleet_core.zones import ZoneManager


@pytest.fixture
def now():
    return [0.0]


@pytest.fixture
def zones(now):
    return ZoneManager(["A", "B"], clock=lambda: now[0])


def test_t04_two_robots_requesting_one_zone(zones):
    """T-04: the first requester holds the zone and the second waits."""
    assert zones.request("r1", "A") is True
    assert zones.request("r2", "A") is False
    assert zones.holder("A") == "r1"
    assert zones.queue("A") == ("r2",)


def test_t04_fifo_order_on_release(zones):
    """T-04: a freed zone goes to the head of the queue, not to whoever asks first."""
    zones.request("r1", "A")
    zones.request("r2", "A")
    zones.request("r3", "A")
    assert zones.release("r1", "A") is True
    assert zones.request("r3", "A") is False, "r3 is behind r2"
    assert zones.holder("A") is None
    assert zones.request("r2", "A") is True
    assert zones.holder("A") == "r2"
    assert zones.queue("A") == ("r3",)


def test_t04_request_is_idempotent(zones):
    """T-04: repeating a request neither duplicates the queue entry nor loses the place."""
    assert zones.request("r1", "A") is True
    assert zones.request("r1", "A") is True
    zones.request("r2", "A")
    zones.request("r2", "A")
    assert zones.queue("A") == ("r2",)


def test_t04_release_by_non_holder_is_refused(zones):
    """T-04: only the holder can release."""
    zones.request("r1", "A")
    assert zones.release("r2", "A") is False
    assert zones.holder("A") == "r1"
    assert zones.release("r1", "B") is False


def test_force_acquire_places_a_robot_and_refuses_a_held_zone(zones):
    assert zones.force_acquire("r1", "A") is True
    assert zones.force_acquire("r1", "A") is True
    assert zones.force_acquire("r2", "A") is False
    assert zones.holder("A") == "r1"


def test_force_acquire_is_refused_across_a_conflict_pair():
    zones = ZoneManager(["A", "B"], clock=lambda: 0.0, conflicts=[frozenset({"A", "B"})])
    assert zones.force_acquire("r1", "A") is True
    assert zones.force_acquire("r2", "B") is False
    assert zones.holder("B") is None
    assert zones.force_acquire("r1", "B") is True, "one robot may hold both"


def test_force_acquire_takes_the_robot_out_of_the_queue(zones):
    zones.request("r1", "A")
    zones.request("r2", "A")
    zones.release("r1", "A")
    assert zones.force_acquire("r2", "A") is True
    assert zones.queue("A") == ()


def test_waiting_for_names_the_zone_a_robot_is_queueing_for(zones):
    zones.request("r1", "A")
    assert zones.waiting_for("r1") is None
    zones.request("r2", "A")
    assert zones.waiting_for("r2") == "A"
    zones.cancel_request("r2", "A")
    assert zones.waiting_for("r2") is None


def test_held_by_lists_zones_in_acquire_order(zones):
    zones.request("r1", "B")
    zones.request("r1", "A")
    assert zones.held_by("r1") == ["B", "A"]
    assert zones.held_by("r2") == []


def test_cancel_request_leaves_the_queue_and_keeps_held_zones(zones):
    zones.request("r1", "A")
    zones.request("r2", "A")
    zones.request("r3", "A")
    assert zones.cancel_request("r2", "A") is True
    assert zones.cancel_request("r2", "A") is False
    assert zones.queue("A") == ("r3",)
    assert zones.cancel_request("r1", "A") is False
    assert zones.holder("A") == "r1"


def test_snapshot_reports_holder_queue_and_acquire_time(zones, now):
    now[0] = 12.5
    zones.request("r1", "A")
    zones.request("r2", "A")
    snap = zones.snapshot()
    assert snap["A"] == {"holder": "r1", "queue": ("r2",), "acquired_at": 12.5}
    assert snap["B"] == {"holder": None, "queue": (), "acquired_at": None}
    zones.release("r1", "A")
    assert zones.snapshot()["A"]["acquired_at"] is None


def test_unknown_zone_is_an_error(zones):
    with pytest.raises(KeyError):
        zones.request("r1", "NOPE")


# ---- conflict pairs ---------------------------------------------------------------------------


def paired(*zones):
    return ZoneManager(zones or ("A", "B"), clock=lambda: 0.0, conflicts=[frozenset({"A", "B"})])


def test_a_held_zone_blocks_the_zone_it_is_a_conflict_pair_with():
    zones = paired()
    assert zones.request("r1", "A")
    assert not zones.request("r2", "B")
    assert zones.holder("B") is None, "free, and still not to be had"
    assert zones.blockers("r2", "B") == {"r1"}
    assert zones.release("r1", "A")
    assert zones.request("r2", "B")


def test_one_robot_may_hold_both_zones_of_a_pair():
    zones = paired()
    assert zones.request("r1", "A")
    assert zones.request("r1", "B"), "the pair keeps robots apart, not a robot from itself"


def test_first_come_first_served_holds_across_a_pair():
    """Otherwise a robot at a crossing waits for as long as the other road stays busy."""
    zones = paired()
    assert zones.request("r1", "A")
    assert not zones.request("r2", "B"), "first in line for the crossing"
    assert not zones.request("r3", "A"), "came later, to the other road"
    assert zones.release("r1", "A")
    assert not zones.request("r3", "A"), "A is free, but r2 asked first"
    assert zones.blockers("r3", "A") == {"r2"}
    assert zones.request("r2", "B")
    assert not zones.request("r3", "A"), "r2 now holds the other half of the pair"
    assert zones.release("r2", "B")
    assert zones.request("r3", "A")


def test_giving_up_the_place_in_line_lets_the_next_one_through():
    zones = paired()
    assert zones.request("r1", "A")
    assert not zones.request("r2", "B")
    assert not zones.request("r3", "A")
    zones.release("r1", "A")
    assert zones.cancel_request("r2", "B")
    assert zones.request("r3", "A")


def test_a_request_made_again_keeps_its_place():
    zones = paired()
    assert zones.request("r1", "A")
    assert not zones.request("r2", "B")
    assert not zones.request("r3", "A")
    assert not zones.request("r2", "B"), "asking every tick is still one place in line"
    zones.release("r1", "A")
    assert zones.request("r2", "B")


def test_blockers_are_the_holder_and_those_ahead_in_line():
    zones = ZoneManager(["A"], clock=lambda: 0.0)
    zones.request("r1", "A")
    zones.request("r2", "A")
    zones.request("r3", "A")
    assert zones.blockers("r2", "A") == {"r1"}
    assert zones.blockers("r3", "A") == {"r1", "r2"}
    assert zones.blockers("r1", "A") == set()
