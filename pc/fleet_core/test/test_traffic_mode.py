"""traffic_mode 스위치(자유바닥 실험 P0): 교통 층만 갈아 끼운다. lanes는 지금 코드 그대로다."""
from dataclasses import asdict, replace

import pytest
import yaml
from fleet_core.common.params import DEFAULT_PARAMS, FleetParams, load_params
from fleet_core.manager import FleetManager
from fleet_core.traffic.controller import TrafficController
from fleet_core.traffic.floor.controller import FloorController
from fleet_core.traffic.lane_map import load_lane_map

ROBOTS = ["pinky1", "pinky2", "pinky3"]


def manager(**changes):
    params = replace(load_params(), **changes)
    return FleetManager(load_lane_map(), params, ROBOTS, lambda: 0.0)


def test_the_shipped_params_pick_lanes():
    assert load_params().traffic_mode == "lanes"
    assert yaml.safe_load(DEFAULT_PARAMS.read_text(encoding="utf-8"))["traffic_mode"] == "lanes"


def test_a_record_without_the_value_replays_with_lanes():
    fields = asdict(load_params())
    del fields["traffic_mode"]
    assert FleetParams(**fields).traffic_mode == "lanes"


def test_lanes_builds_the_lane_controller_itself():
    assert type(manager().traffic) is TrafficController


def test_freefloor_builds_the_floor_controller():
    traffic = manager(traffic_mode="freefloor").traffic
    assert isinstance(traffic, FloorController)


def test_an_unknown_mode_is_refused_with_the_known_ones():
    with pytest.raises(ValueError, match="lanes.*freefloor"):
        manager(traffic_mode="roads")


def test_the_manager_api_is_the_same_in_both_modes():
    lanes, floor = manager(), manager(traffic_mode="freefloor")
    for name in ("on_robot_state", "tick", "zones_view", "robots_view", "confirm_clear",
                 "reinstate", "assignments_view"):
        assert callable(getattr(lanes, name)) and callable(getattr(floor, name))
    assert floor.tick() == []  # nothing reported yet: nothing moves


def test_a_freefloor_record_is_refused_until_replay_knows_the_floor():
    floor = manager(traffic_mode="freefloor")
    with pytest.raises(ValueError, match="freefloor"):
        floor.record_to(lambda line: None)
