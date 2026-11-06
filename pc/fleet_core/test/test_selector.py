"""RobotSelector: who is called to the fill dock. Pure, driven by plain robot records."""
import pytest
from fleet_core.common.params import load_params
from fleet_core.traffic.selector import RobotInfo, RobotSelector
from grid_map import grid_site

P6, P5, P4 = (3, 4), (3, 5), (3, 6)


@pytest.fixture(scope="module")
def selector():
    return RobotSelector(grid_site(), load_params())


def lane(name, cell, **kw):
    return RobotInfo(name, cell, **kw)


def parked(name, cell, **kw):
    return RobotInfo(name, cell, parked=True, **kw)


def test_no_orders_no_calls(selector):
    assert selector.pick(0, [parked("a", P6)]) == set()


def test_calls_follow_the_number_of_unbound_orders(selector):
    robots = [parked("a", P6), parked("b", P5), parked("c", P4)]
    assert selector.pick(1, robots) == {"a"}
    assert selector.pick(2, robots) == {"a", "b"}
    assert selector.pick(5, robots) == {"a", "b", "c"}, "no more calls than robots"


def test_shortest_way_along_the_lanes_wins_not_the_straight_line(selector):
    """(6,3) is one cell from the dock on the grid, but it must drive the whole loop."""
    robots = [lane("near_by_grid", (6, 3)), lane("near_by_lane", (2, 0))]
    assert selector.pick(1, robots) == {"near_by_lane"}


def test_only_eligible_robots_are_called(selector):
    robots = [lane("busy", (4, 3), eligible=False), lane("free", (9, 2))]
    assert selector.pick(1, robots) == {"free"}


def test_a_called_robot_keeps_the_call_unless_another_is_clearly_closer(selector):
    far = lane("far", (9, 2), called=True)  # 15 cells from the dock, worth 4.5 minus 4.0
    assert selector.pick(1, [far, lane("closer", (5, 0))]) == {"far"}, "9 cells, worth 2.7"
    assert selector.pick(1, [far, lane("much_closer", (4, 3))]) == {"much_closer"}, "1 cell"


def test_name_breaks_a_tie_so_a_run_can_be_replayed(selector):
    assert selector.pick(1, [lane("b", (2, 2)), lane("a", (2, 2))]) == {"a"}


def test_waiting_robot_cannot_pass_the_one_ahead(selector):
    """b is closer than a lane robot only if a goes first, so a is what gets called."""
    robots = [parked("b", P5), parked("a", P6)]
    assert selector.pick(1, robots) == {"a"}


def test_failed_waiting_robot_blocks_those_behind_it(selector):
    robots = [parked("a", P6, eligible=False, available=False), parked("b", P5), parked("c", P4)]
    assert selector.pick(2, robots) == set()


def test_failed_waiting_robot_does_not_block_those_ahead_of_it(selector):
    robots = [parked("a", P6), parked("b", P5, eligible=False, available=False), parked("c", P4)]
    assert selector.pick(2, robots) == {"a"}


def test_robot_on_the_lane_is_not_blocked_by_a_failed_waiting_robot(selector):
    robots = [parked("a", P6, eligible=False, available=False), parked("b", P5), lane("c", (9, 2))]
    assert selector.pick(2, robots) == {"c"}


def test_a_robot_already_gone_from_the_waiting_cells_blocks_nobody(selector):
    robots = [lane("a", (3, 3), eligible=False), parked("b", P5)]
    assert selector.pick(1, robots) == {"b"}
