"""The lane map: free-direction segments, lock units, conflict pairs and the two-way bundle."""
import math

import pytest
from fleet_core.site_map import SiteMap, SiteMapError, load_site_map

CLEARANCE = 0.16  # 2r
RELEASE = 0.2  # d


def nodes(**where):
    return [{"id": name, "x": xy[0], "y": xy[1]} for name, xy in where.items()]


def lanes(*pairs):
    return [{"from": a, "to": b} for a, b in pairs]


def build(where, pairs, **extra):
    return SiteMap({"half_width_m": 0.05, "lane_entry_m": 0.2, "origin_m": [0, 0],
                    "nodes": nodes(**where), "lanes": lanes(*pairs), "stations": {},
                    "parking": [], **extra})


STRAIGHT = {"a": (0, 0), "b": (1, 0), "c": (2, 0)}


# ---- geometry ---------------------------------------------------------------------------


def test_a_lane_can_point_in_any_direction():
    site = build({"a": (0, 0), "b": (0.3, 0.4)}, [("a", "b")])
    assert site.lane("a", "b").length == pytest.approx(0.5)
    assert site.lane("a", "b").ux == pytest.approx(0.6)
    assert site.lane("a", "b").uy == pytest.approx(0.8)


def test_a_written_length_is_the_planning_cost_and_the_geometry_stays_straight():
    site = SiteMap({"half_width_m": 0.05, "lane_entry_m": 0.2, "origin_m": [0, 0],
                    "nodes": nodes(a=(0, 0), b=(1, 0)),
                    "lanes": [{"from": "a", "to": "b", "length": 2.5}],
                    "stations": {}, "parking": []})
    assert site.lane("a", "b").length == 2.5
    assert site.lane_progress("a", "b", 0.5, 0.0) == pytest.approx((0.5, 0.0))


def test_progress_along_a_lane_is_the_projection_and_the_error_is_the_rest():
    site = build({"a": (0, 0), "b": (3, 4)}, [("a", "b")])
    s, e = site.lane_progress("a", "b", 3.0, 4.0)
    assert s == pytest.approx(5.0) and e == pytest.approx(0.0)
    s, e = site.lane_progress("a", "b", -0.8, 0.6)
    assert s == pytest.approx(0.0) and e == pytest.approx(1.0)
    s, _ = site.lane_progress("a", "b", -3.0, -4.0)
    assert s == pytest.approx(-5.0), "not clamped: behind the lane start is negative"


def test_the_pose_of_a_node_is_what_the_map_says():
    site = build({"a": (0.25, 1.5), "b": (1, 1)}, [("a", "b")])
    assert site.cell_pose("a") == (0.25, 1.5)
    assert site.cell_id("a") == "a"


def test_a_grid_cell_can_still_be_a_node_id():
    site = SiteMap({"half_width_m": 0.05, "lane_entry_m": 0.2, "origin_m": [0, 0],
                    "nodes": [{"id": [5, 3], "x": 1.0, "y": 1.0},
                              {"id": [6, 3], "x": 2.0, "y": 1.0}],
                    "lanes": [{"from": [5, 3], "to": [6, 3]}], "stations": {}, "parking": []})
    assert site.zone_of[(5, 3)] == "c5_3"
    assert site.cell_id((5, 3)) == "c5_3"
    assert site.plan((5, 3), (6, 3)) == [(5, 3), (6, 3)]


# ---- which unit a pose is in --------------------------------------------------------------


def test_a_robot_on_a_lane_belongs_to_the_node_it_is_driving_to():
    site = build(STRAIGHT, [("a", "b"), ("b", "c")])
    assert site.nearest_cell(0.0, 0.0) == "a", "standing on the start node"
    assert site.nearest_cell(0.04, 0.0) == "a", "still inside the node"
    assert site.nearest_cell(0.1, 0.0) == "b", "on the way, so the lane's unit is the end node's"
    assert site.nearest_cell(1.0, 0.0) == "b"
    assert site.nearest_cell(1.5, 0.0) == "c"


def test_on_a_crossing_a_robot_belongs_to_the_lane_it_is_on():
    site = build({"w": (0, 0), "e": (2, 0), "n": (1, 1), "s": (1, -1)},
                 [("w", "e"), ("n", "s")])
    assert site.nearest_cell(0.99, 0.0) == "e"
    assert site.nearest_cell(1.0, 0.01) == "s"


def test_a_zone_is_its_lanes_from_the_entry_point_on():
    site = build(STRAIGHT, [("a", "b"), ("b", "c")])
    assert site.distance_to_zone(1.0, 0.0, "b") == 0.0
    assert site.distance_to_zone(0.5, 0.0, "b") == 0.0
    assert site.distance_to_zone(0.5, 0.04, "b") == 0.0, "inside the half width"
    assert site.distance_to_zone(0.5, 0.15, "b") == pytest.approx(0.10)
    assert site.distance_to_zone(0.0, 0.0, "b") == pytest.approx(0.15), "not entered at the start"
    assert site.distance_to_zone(1.25, 0.0, "b") == pytest.approx(0.20), "past the end node"


# ---- lock units ---------------------------------------------------------------------------


def test_each_node_is_a_unit_of_its_own_by_default():
    site = build(STRAIGHT, [("a", "b"), ("b", "c")])
    assert site.zones == {"a": frozenset({"a"}), "b": frozenset({"b"}), "c": frozenset({"c"})}
    assert site.zone_of == {"a": "a", "b": "b", "c": "c"}


def test_lanes_into_one_node_share_its_unit():
    """They end at the same point, so they could not be used together anyway."""
    site = build({"a": (0, 1), "b": (0, -1), "m": (1, 0)}, [("a", "m"), ("b", "m")])
    assert site.lanes_of_zone("m") == {("a", "m"), ("b", "m")}


def test_named_units_group_nodes():
    site = build(STRAIGHT, [("a", "b"), ("b", "c")], units={"ZONE": ["a", "b"]})
    assert site.zone_of == {"a": "ZONE", "b": "ZONE", "c": "c"}
    assert site.lanes_of_zone("ZONE") == {("a", "b")}


def test_a_node_in_two_units_is_refused():
    with pytest.raises(SiteMapError, match="more than one"):
        build(STRAIGHT, [("a", "b")], units={"X": ["a"], "Y": ["a", "b"]})


def test_a_two_way_entry_makes_both_lanes():
    site = build(STRAIGHT, [("b", "c")], two_way={"ROAD": ["a", "b"]})
    assert site.plan("a", "b") == ["a", "b"]
    assert site.plan("b", "a") == ["b", "a"]


def test_a_two_way_pair_is_one_unit_when_bundled():
    site = build(STRAIGHT, [("b", "c")], two_way={"ROAD": ["a", "b"]})
    assert site.zone_of["a"] == site.zone_of["b"] == "ROAD"
    assert site.lanes_of_zone("ROAD") == {("a", "b"), ("b", "a")}


def test_the_bundle_can_be_switched_off():
    data = {"half_width_m": 0.05, "lane_entry_m": 0.2, "origin_m": [0, 0],
            "nodes": nodes(**STRAIGHT), "lanes": lanes(("b", "c")),
            "two_way": {"ROAD": ["a", "b"]}, "stations": {}, "parking": []}
    off = SiteMap(data, bundle_two_way=False)
    assert off.zone_of["a"] == "a" and off.zone_of["b"] == "b"
    assert SiteMap({**data, "bundle_two_way": False}).zone_of["a"] == "a"
    assert SiteMap({**data, "bundle_two_way": False}, bundle_two_way=True).zone_of["a"] == "ROAD"


def test_two_way_segments_that_touch_become_one_unit():
    site = build(STRAIGHT, [], two_way={"W": ["a", "b"], "E": ["b", "c"]})
    assert len({site.zone_of[n] for n in STRAIGHT}) == 1


# ---- conflict pairs -----------------------------------------------------------------------


def pairs_of(site):
    return site.conflict_pairs(CLEARANCE, RELEASE)


def test_lanes_that_cross_conflict():
    site = build({"w": (0, 0), "e": (2, 0), "n": (1, 1), "s": (1, -1)},
                 [("w", "e"), ("n", "s")])
    assert pairs_of(site) == {frozenset({"e", "s"})}


def test_parallel_lanes_conflict_only_closer_than_two_radii():
    def parallel(gap):
        return build({"a": (0, 0), "b": (2, 0), "c": (0, gap), "d": (2, gap)},
                     [("a", "b"), ("c", "d")])

    assert pairs_of(parallel(0.15)) == {frozenset({"b", "d"})}
    assert pairs_of(parallel(0.17)) == set()


def test_a_straight_chain_has_no_conflict_or_nobody_could_follow():
    assert pairs_of(build(STRAIGHT, [("a", "b"), ("b", "c")])) == set()


def _turn(degrees):
    """a -> b -> c, with the angle between the lanes at b measured inside the corner."""
    angle = math.radians(degrees)
    return build({"a": (1, 0), "b": (0, 0), "c": (math.cos(angle), math.sin(angle))},
                 [("a", "b"), ("b", "c")])


def test_a_sharp_turn_conflicts_because_the_leader_is_still_close():
    assert pairs_of(_turn(90)) == set()
    assert pairs_of(_turn(60)) == set(), "0.2 * sin 60 = 0.173"
    assert pairs_of(_turn(45)) == {frozenset({"b", "c"})}, "0.2 * sin 45 = 0.141"


def test_a_narrow_fork_conflicts_and_a_wide_one_does_not():
    def fork(degrees):
        angle = math.radians(degrees)
        return build({"o": (0, 0), "p": (1, 0), "q": (math.cos(angle), math.sin(angle))},
                     [("o", "p"), ("o", "q")])

    assert pairs_of(fork(90)) == set()
    assert pairs_of(fork(45)) == {frozenset({"p", "q"})}


def test_merging_lanes_are_one_unit_so_they_are_never_a_pair():
    assert pairs_of(build({"a": (0, 1), "b": (0, -1), "m": (1, 0)},
                          [("a", "m"), ("b", "m")])) == set()


def test_a_bundled_two_way_lane_is_not_a_pair_with_itself():
    assert pairs_of(build(STRAIGHT, [("b", "c")], two_way={"ROAD": ["a", "b"]})) == set()


def test_with_the_bundle_off_the_two_directions_are_not_paired_either():
    """Off means the naive rule. That is what the bundle's comparison measures."""
    data = {"half_width_m": 0.05, "lane_entry_m": 0.2, "origin_m": [0, 0],
            "nodes": nodes(**STRAIGHT), "lanes": lanes(("b", "c")),
            "two_way": {"ROAD": ["a", "b"]}, "stations": {}, "parking": []}
    assert pairs_of(SiteMap(data, bundle_two_way=False)) == set()


def test_a_lane_that_only_passes_a_node_conflicts_with_that_nodes_unit():
    site = build({"a": (0, 0), "b": (1, 0), "x": (0.5, 1), "y": (0.5, 0.1)},
                 [("a", "b"), ("x", "y")])
    assert pairs_of(site) == {frozenset({"b", "y"})}


# ---- loading ------------------------------------------------------------------------------


@pytest.mark.parametrize("change, message", [
    ({"lanes": lanes(("a", "nowhere"))}, "nowhere"),
    ({"lanes": lanes(("a", "a"))}, "itself"),
    ({"lanes": lanes(("a", "b"), ("a", "b"))}, "twice"),
    ({"parking": ["nowhere"]}, "nowhere"),
    ({"units": {"X": ["nowhere"]}}, "nowhere"),
    ({"two_way": {"X": ["a", "nowhere"]}}, "nowhere"),
    ({"stations": {"S": {"dock": "nowhere", "queue": [], "server": "s", "arms": ["a"],
                         "action": "fill"}}}, "nowhere"),
    ({"stations": {"S": {"dock": "b", "queue": [["c", "a"]], "server": "s", "arms": ["a"],
                         "action": "fill"}}}, "no lane"),
    ({"stations": {"S": {"dock": "b", "queue": [], "commit": "nowhere", "server": "s",
                         "arms": ["a"], "action": "fill"}}}, "nowhere"),
])
def test_an_inconsistent_map_is_refused(change, message):
    data = {"half_width_m": 0.05, "lane_entry_m": 0.2, "origin_m": [0, 0],
            "nodes": nodes(**STRAIGHT), "lanes": lanes(("a", "b"), ("b", "c")),
            "stations": {}, "parking": [], **change}
    with pytest.raises(SiteMapError, match=message):
        SiteMap(data)


def test_a_node_with_no_lane_is_refused():
    with pytest.raises(SiteMapError, match="no lane"):
        build(STRAIGHT, [("a", "b")])


def test_the_commit_node_defaults_to_the_dock():
    site = build(STRAIGHT, [("a", "b"), ("b", "c")], stations={
        "S": {"dock": "c", "queue": [["b", "c"]], "server": "s", "arms": ["a"], "action": "fill"}})
    assert site.stations["S"].commit == "c"
    assert site.stations["S"].queue == (("b", "c"),)


def test_a_lane_leaving_a_unit_must_be_longer_than_the_release_distance():
    site = build({"a": (0, 0), "b": (0.1, 0), "c": (1, 0)}, [("a", "b"), ("b", "c")])
    assert site.shortest_exit_lane() == pytest.approx(0.1)
    inside = build({"a": (0, 0), "b": (0.1, 0), "c": (1, 0)}, [("a", "b"), ("b", "c")],
                   units={"Z": ["a", "b"]})
    assert inside.shortest_exit_lane() == pytest.approx(0.9), "a lane inside a unit never releases"


# ---- planning -----------------------------------------------------------------------------


def test_plan_takes_the_shortest_way_in_metres_not_the_fewest_lanes():
    site = build({"a": (0, 0), "b": (2, 0), "m": (1, 0.1), "far": (1, 5)},
                 [("a", "far"), ("far", "b"), ("a", "m"), ("m", "b"), ("b", "a")])
    assert site.plan("a", "b") == ["a", "m", "b"]
    assert site.path_length(["a", "m", "b"]) == pytest.approx(2 * math.hypot(1, 0.1))


def test_plan_follows_lane_direction_and_refuses_what_cannot_be_reached():
    site = build({"a": (0, 0), "b": (1, 0), "c": (2, 0)}, [("a", "b"), ("c", "b")])
    with pytest.raises(SiteMapError):
        site.plan("b", "a")


# ---- the maps in the repository -----------------------------------------------------------


def test_the_default_map_loads_and_has_two_way_traffic_and_a_crossing():
    site = load_site_map()
    two_way = [(a, b) for (a, b) in site.lanes if (b, a) in site.lanes]
    assert two_way, "two robots must be able to meet head-on"
    directions = {round(math.atan2(lane.uy, lane.ux), 3) for lane in site.lanes.values()}
    assert any(abs(d) not in (0.0, round(math.pi / 2, 3), round(math.pi, 3)) for d in directions), (
        "diagonals are allowed")
    assert site.conflict_pairs(CLEARANCE, RELEASE), "a crossing needs a conflict pair"
    assert site.stations["WORK"].action == "fill"
    assert site.stations["OUTBOUND"].action == "swap"
    assert site.shortest_exit_lane() > RELEASE
