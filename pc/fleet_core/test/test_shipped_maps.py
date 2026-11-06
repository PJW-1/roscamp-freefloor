"""sim/fake_robots/maps의 지도 셋: 읽힘, 구역 수, 길 연결, 특징 지도의 레인 간격과 구간 길이."""
import itertools
import math
from pathlib import Path

import pytest
from fleet_core.traffic.lane_map import _segment_distance, load_lane_map

MAPS = Path(__file__).resolve().parents[3] / "sim" / "fake_robots" / "maps"
ALL = sorted(MAPS.glob("*.yaml"))
RELEASE = 0.2  # d
CLEARANCE = 0.16  # 2r
GAP_MIN = 0.3  # 2r + 2e + 여유, e는 시뮬 값 0.067 m


def lane_gap(site, lanes_a, lanes_b) -> float:
    def ends(key):
        lane = site.lane(*key)
        return (lane.ax, lane.ay), (lane.bx, lane.by)

    return min(_segment_distance(*ends(a), *ends(b)) for a in lanes_a for b in lanes_b)


def test_the_map_set_is_there():
    names = {p.name for p in ALL}
    assert {"feature_map.yaml", "two_lane_avenue.yaml", "two_lane_segments.yaml"} <= names


@pytest.mark.parametrize("path", ALL, ids=lambda p: p.name)
def test_every_map_loads_and_every_waypoint_reaches_docks_and_parking(path):
    site = load_lane_map(path)
    assert site.parking and site.docks
    assert site.shortest_exit_lane() > RELEASE
    for waypoint in site._pose:
        for goal in (*site.docks, site.parking[0]):
            assert site.plan(waypoint, goal)
    spots = (*site.docks, *site.parking)
    for start, goal in itertools.product(spots, spots):
        assert site.plan(start, goal)


@pytest.mark.parametrize("path", ALL, ids=lambda p: p.name)
def test_every_map_uses_the_current_waypoints_key(path):
    text = path.read_text(encoding="utf-8")
    assert "nodes:" not in text
    assert "waypoints:" in text


# ---- 특징 지도 ----------------------------------------------------------------------------

ROAD_E = [("E0", "E1"), ("E1", "E2")]
ROAD_F = [("F0", "F1"), ("F1", "F2")]


@pytest.fixture(scope="module")
def feature():
    return load_lane_map(MAPS / "feature_map.yaml")


def test_feature_map_stays_small(feature):
    assert len(feature.zones) <= 15


def test_feature_map_road_lanes_are_far_enough_apart_and_split_short(feature):
    assert lane_gap(feature, ROAD_E, ROAD_F) >= GAP_MIN
    for lane in ROAD_E + ROAD_F:
        assert 0.3 <= feature.lane(*lane).span <= 0.4
    assert len(ROAD_E) in (2, 3) and len(ROAD_F) in (2, 3)
    assert not any(frozenset({feature.zone_of[a], feature.zone_of[b]})
                   in feature.conflict_pairs(CLEARANCE, RELEASE)
                   for a, b in itertools.product(("E0", "E1", "E2"), ("F0", "F1", "F2")))


def test_feature_map_has_one_two_way_bundle_and_a_way_around_it(feature):
    two_way = sorted((a, b) for (a, b) in feature.lanes if (b, a) in feature.lanes and a < b)
    assert two_way == [("T1", "T2")]
    assert feature.zone_of["T1"] == feature.zone_of["T2"]
    # 통로 레인을 빼도 양쪽 방향 모두 돌아갈 수 있다
    around = {k: v for k, v in feature.lanes.items() if k not in {("T1", "T2"), ("T2", "T1")}}
    for start, goal in (("F2", "QO"), ("F0", "QW")):
        seen, todo = {start}, [start]
        while todo:
            here = todo.pop()
            for (a, b) in around:
                if a == here and b not in seen:
                    seen.add(b)
                    todo.append(b)
        assert goal in seen


def test_feature_map_has_exactly_one_crossing_without_a_waypoint(feature):
    crossings = []
    for one, other in itertools.combinations(feature.lanes.values(), 2):
        if {one.a, one.b} & {other.a, other.b}:
            continue
        if _segment_distance((one.ax, one.ay), (one.bx, one.by),
                             (other.ax, other.ay), (other.bx, other.by)) == 0.0:
            crossings.append(frozenset({feature.zone_of[one.b], feature.zone_of[other.b]}))
    assert len(set(crossings)) == 1
    assert set(crossings) <= feature.conflict_pairs(CLEARANCE, RELEASE)


def test_feature_map_only_conflict_is_the_crossing(feature):
    pairs = feature.conflict_pairs(CLEARANCE, RELEASE)
    assert pairs == {frozenset({"P-1", "TUN"})}


def test_feature_map_has_both_stations_and_waiting_spots(feature):
    assert feature.stations["WORK"].action == "fill"
    assert feature.stations["OUTBOUND"].action == "swap"
    assert len(feature.parking) >= 2


# ---- 비교용 지도 --------------------------------------------------------------------------


def test_two_lane_maps_keep_the_demo_spacing_and_warn_about_it():
    avenue = load_lane_map(MAPS / "two_lane_avenue.yaml")
    segments = load_lane_map(MAPS / "two_lane_segments.yaml")
    assert lane_gap(avenue, [("AE", "BE")], [("BW", "AW")]) == pytest.approx(0.2)
    assert lane_gap(segments, [("AE", "E1"), ("E1", "E2"), ("E2", "BE")],
                    [("BW", "W2"), ("W2", "W1"), ("W1", "AW")]) == pytest.approx(0.2)
    for key in (("AE", "E1"), ("E1", "E2"), ("E2", "BE"), ("BW", "W2"), ("W2", "W1"), ("W1", "AW")):
        assert 0.36 <= segments.lane(*key).span <= 0.37 + 1e-9
        assert segments.lane(*key).span > RELEASE
    for name in ("two_lane_avenue.yaml", "two_lane_segments.yaml"):
        text = (MAPS / name).read_text(encoding="utf-8")
        assert "안전한 배치가 아니다" in text and "0.3 m" in text


def test_comparison_maps_have_the_same_stations_as_the_default_map():
    default = load_lane_map()
    for name in ("two_lane_avenue.yaml", "two_lane_segments.yaml"):
        site = load_lane_map(MAPS / name)
        assert site.stations["WORK"].dock == default.stations["WORK"].dock
        assert site.parking == default.parking
        assert math.isclose(site.waypoint_pose("W")[0], default.waypoint_pose("W")[0])
