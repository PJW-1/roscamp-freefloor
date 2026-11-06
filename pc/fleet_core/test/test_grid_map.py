"""The cell layout, converted to lanes, must stay the layout the older tests were written for."""
import pytest
from fleet_core.traffic.lane_map import LaneMapError
from grid_map import grid_site


@pytest.fixture(scope="module")
def site():
    return grid_site()


def test_map_matches_the_design_document(site):
    assert len(site.zones) == 16
    assert len(set(site.zone_of)) == 30
    work, outbound = site.stations["WORK"], site.stations["OUTBOUND"]
    assert (work.dock, work.queue, work.commit, work.server, work.arms, work.action) == (
        (5, 3), (((3, 3), (4, 3)), ((4, 3), (5, 3))), (3, 3), "workcell", ("arm1", "arm2"), "fill")
    assert (outbound.dock, outbound.queue, outbound.server, outbound.arms, outbound.action) == (
        (9, 3), (((9, 5), (9, 4)), ((9, 4), (9, 3))), "arm3", ("arm3",), "swap")


def test_every_node_belongs_to_exactly_one_zone(site):
    nodes = {node for lane in site.lanes for node in lane}
    assert nodes == set(site.zone_of)
    assert sum(len(cells) for cells in site.zones.values()) == len(site.zone_of)


def test_round_trip_lengths_match_the_design_document(site):
    assert len(site.plan((5, 3), (9, 3))) == 9
    assert len(site.plan((9, 3), (5, 3))) == 17
    assert len(site.parking) == 6


def test_every_lane_is_one_cell_long(site):
    assert {round(lane.length, 6) for lane in site.lanes.values()} == {0.33}


def test_plan_follows_the_one_way_lanes(site):
    path = site.plan((9, 3), (2, 0))
    assert path[:3] == [(9, 3), (9, 2), (9, 1)]
    assert path[-3:] == [(4, 0), (3, 0), (2, 0)]
    assert (9, 4) not in path, "must not reverse into the outbound queue"


def test_plan_from_outbound_dock_to_work_passes_outside_the_waiting_cells(site):
    path = site.plan((9, 3), (5, 3))
    assert len(path) == 17
    assert not set(path) & set(site.parking)


def test_plan_into_the_waiting_cells_uses_the_link(site):
    path = site.plan((9, 3), (3, 4))
    assert path[-7:] == [(2, 3), (2, 4), (2, 5), (2, 6), (3, 6), (3, 5), (3, 4)]


def test_plan_from_a_waiting_cell_rejoins_before_the_work_queue(site):
    assert site.plan((3, 5), (5, 3)) == [(3, 5), (3, 4), (3, 3), (4, 3), (5, 3)]


def test_plan_from_past_the_commit_zone_to_waiting_laps_through_both_docks(site):
    path = site.plan((3, 3), (2, 4))
    assert path[:3] == [(3, 3), (4, 3), (5, 3)]
    assert (9, 3) in path
    assert path[-2:] == [(2, 3), (2, 4)]


def test_plan_to_a_node_with_no_way_in_raises(site):
    with pytest.raises(LaneMapError):
        site.plan((9, 3), (5, 1))


def test_cell_pose_is_the_centre_of_the_cell(site):
    x, y = site.waypoint_pose((2, 0))
    assert x == pytest.approx(2.5 * 0.33)
    assert y == pytest.approx(0.5 * 0.33)


def test_distance_to_zone_is_zero_inside_and_grows_outside(site):
    x, y = site.waypoint_pose((9, 4))
    assert site.distance_to_zone(x, y, "Q-OUT-1") == 0.0
    # Centre of (9,3) is half a cell beyond the edge of (9,4).
    x, y = site.waypoint_pose((9, 3))
    assert site.distance_to_zone(x, y, "Q-OUT-1") == pytest.approx(0.33 / 2)


def test_no_lane_of_the_cell_layout_conflicts_with_another(site):
    assert site.conflict_pairs(0.16, 0.2) == set()
