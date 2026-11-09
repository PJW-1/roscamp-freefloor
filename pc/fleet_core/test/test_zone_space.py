"""zone_space: /fleet/zones의 구역 이름을 자리로 잰다(자유바닥을 ROS·Gazebo로 띄울 때).

lanes면 구역은 레인 지도의 구역이고, freefloor면 칸 이름(c12_17)이다. 구역 거리를 재는
노드(fleet_check, 판정기, RViz 화면)가 두 방식을 같은 함수로 잰다.
"""
from dataclasses import replace

import pytest
from fleet_core.common.params import load_params
from fleet_core.traffic.floor.floor_map import load_floor_map
from fleet_core.traffic.lane_map import load_lane_map
from fleet_core.traffic.modes import zone_space

PARKING = (0.35, 1.55)  # floor_map.yaml과 lane_map.yaml의 P-3


@pytest.fixture(scope="module")
def site():
    return load_lane_map()


@pytest.fixture(scope="module")
def floor():
    return load_floor_map()


def space(site, floor, mode="freefloor"):
    return zone_space(site, replace(load_params(), traffic_mode=mode), floor)


def a_cell(floor, x, y):
    cell = floor.cell_of(x, y)
    assert floor.inside(cell)
    return floor.cell_id(cell)


def test_lanes_use_the_lane_map_as_it_is(site, floor):
    assert space(site, floor, "lanes") is site


def test_a_cell_name_is_measured_to_the_cell_square(site, floor):
    zones = space(site, floor)
    name = a_cell(floor, *PARKING)
    assert zones.distance_to_zone(*PARKING, name) == 0.0
    x, y = PARKING[0] + 0.5, PARKING[1]
    assert zones.distance_to_zone(x, y, name) == pytest.approx(
        floor.box_distance(floor.cell_at(name), x, y))
    assert zones.distance_to_zone(x, y, name) > 0.3


def test_a_lane_zone_name_still_uses_the_lane_map(site, floor):
    zones = space(site, floor)
    zone = sorted(site.zones)[0]
    assert zones.distance_to_zone(*PARKING, zone) == site.distance_to_zone(*PARKING, zone)


def test_an_unknown_name_fails_as_it_does_with_lanes(site, floor):
    with pytest.raises(KeyError):
        space(site, floor).distance_to_zone(*PARKING, "no_such_zone")


def test_cell_box_is_the_square_around_the_cell(site, floor):
    zones = space(site, floor)
    name = a_cell(floor, *PARKING)
    x0, y0, x1, y1 = zones.cell_box(name)
    assert x1 - x0 == pytest.approx(floor.cell_m) and y1 - y0 == pytest.approx(floor.cell_m)
    assert x0 <= PARKING[0] <= x1 and y0 <= PARKING[1] <= y1
    assert zones.cell_box(sorted(site.zones)[0]) is None
    assert space(site, floor).is_cell(name) and not space(site, floor).is_cell("WORK")


def test_everything_else_comes_from_the_lane_map(site, floor):
    zones = space(site, floor)
    assert zones.half_width == site.half_width
    assert zones.stations is site.stations
    assert zones.zone_of is site.zone_of


def test_the_floor_map_is_loaded_when_none_is_given(site):
    zones = zone_space(site, replace(load_params(), traffic_mode="freefloor"))
    assert zones.distance_to_zone(*PARKING, a_cell(load_floor_map(), *PARKING)) == 0.0


def test_an_unknown_mode_is_refused(site, floor):
    with pytest.raises(ValueError, match="traffic_mode"):
        space(site, floor, "rails")
