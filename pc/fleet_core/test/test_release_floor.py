"""해제 규칙 보강(파트 7): 예각 합류에서는 나가는 레인의 s만으로 구역을 벗어났다고 볼 수 없다.

특징 지도의 T1에서 T1→F2와 T1→QW는 53.1°로 벌어져, s = 0.2에서 F2→T1까지 0.2·0.8 = 0.16이다.
보강하면 구역의 다른 바닥 선에서도 중심선 거리 d 이상이어야 놓는다.
"""
from dataclasses import asdict, replace
from pathlib import Path

import pytest
import yaml
from fleet_core.common.params import FleetParams, load_params
from fleet_core.traffic.controller import TrafficController
from fleet_core.traffic.lane_map import load_lane_map

REPO = Path(__file__).resolve().parents[3]
MAPS = REPO / "sim/fake_robots/maps"
FEATURE = MAPS / "feature_map.yaml"


def core_on(site, on=True):
    params = replace(load_params(), release_clear_of_floor=on)
    return TrafficController(site, params, ["r1"], lambda: 0.0)


def at_t1_exit(core, along, toward_f2):
    """T2 T1 QW 경로에서 T1→QW를 along만큼 가고 F2→T1 쪽으로 toward_f2만큼 치우친 로봇."""
    m = core._missions["r1"]
    m.path, m.idx = ["T2", "T1", "QW"], 2
    lane = core.site.lane("T1", "QW")
    x, y = core.site.waypoint_pose("T1")
    fx, fy = core.site.waypoint_pose("F2")
    nx, ny = -lane.uy, lane.ux
    if nx * (fx - x) + ny * (fy - y) < 0:  # F2 쪽 법선으로
        nx, ny = -nx, -ny
    m.x = x + lane.ux * along + nx * toward_f2
    m.y = y + lane.uy * along + ny * toward_f2
    return m


def test_the_yaml_turns_it_on_and_a_record_without_it_keeps_the_old_rule():
    assert load_params().release_clear_of_floor is True
    fields = asdict(load_params())
    del fields["release_clear_of_floor"]
    assert FleetParams(**fields).release_clear_of_floor is False


def test_just_past_d_on_the_exit_lane_but_close_to_the_lane_from_f2_keeps_the_zone():
    core = core_on(load_lane_map(FEATURE))
    m = at_t1_exit(core, core.params.release_distance_m + 0.01, 0.02)
    assert not core._clear_of(m, "TUN")


def test_further_along_the_exit_lane_lets_the_zone_go():
    core = core_on(load_lane_map(FEATURE))
    m = at_t1_exit(core, 0.3, 0.0)
    assert core._clear_of(m, "TUN")


def test_without_the_flag_the_old_rule_lets_it_go_just_past_d():
    core = core_on(load_lane_map(FEATURE), on=False)
    m = at_t1_exit(core, core.params.release_distance_m + 0.01, 0.02)
    assert core._clear_of(m, "TUN")


def test_a_straight_run_is_released_as_before():
    core = core_on(load_lane_map())
    m = core._missions["r1"]
    site = core.site
    a, b = next((a, b) for (a, b) in site.lanes
                if site.zone_of[a] != site.zone_of[b]
                and all(site.zone_of[k[0]] != site.zone_of[a] for k in site.lanes
                        if k[1] == a and k[0] != b))
    m.path, m.idx = [a, b], 1
    lane = site.lane(a, b)
    x, y = site.waypoint_pose(a)
    m.x, m.y = x + lane.ux * (core.params.release_distance_m + 0.01), y + lane.uy * (
        core.params.release_distance_m + 0.01)
    assert core._clear_of(m, site.zone_of[a])


@pytest.mark.parametrize("path", [None, *sorted(MAPS.glob("*.yaml"))],
                         ids=lambda p: p.name if p else "default")
def test_every_shipped_map_passes_the_start_check(path):
    core_on(load_lane_map(path))


def test_an_exit_lane_ending_near_the_zone_floor_is_refused_at_start(tmp_path):
    data = yaml.safe_load(FEATURE.read_text(encoding="utf-8"))
    for w in data["waypoints"]:
        if w["id"] == "QW":
            w["x"], w["y"] = 0.85, 0.85  # T1→QW 끝이 F2→T1에서 0.15
    bad = tmp_path / "bad.yaml"
    bad.write_text(yaml.safe_dump(data), encoding="utf-8")
    site = load_lane_map(bad)
    core_on(site, on=False)
    with pytest.raises(ValueError, match="T1 to QW"):
        core_on(site)
