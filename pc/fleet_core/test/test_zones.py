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


def test_force_acquire_takes_the_robot_out_of_the_queue(zones):
    zones.request("r1", "A")
    zones.request("r2", "A")
    zones.release("r1", "A")
    assert zones.force_acquire("r2", "A") is True
    assert zones.queue("A") == ()


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
