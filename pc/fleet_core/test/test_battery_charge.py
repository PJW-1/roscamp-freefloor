"""배터리가 낮은 로봇은 새 일을 받지 않고 충전 자리(대기 지점)로 간다.

battery_low_pct가 0이면 끈다(기본). 보고의 battery는 0-100이고 0 이하는 모르는 값이다.
"""
from dataclasses import replace

from core_helpers import codes
from fleet_core.common.params import load_params
from fleet_core.traffic.controller import TrafficController
from grid_map import grid_site
from test_traffic_controller import WORK, at_station, drive

FRONT, BEHIND, AWAY = (3, 4), (3, 5), (9, 0)  # 대기 줄의 앞과 뒤, 줄 밖


def make(cells, low=20.0, battery=None):
    now = [0.0]
    params = replace(load_params(), battery_low_pct=low)
    core = TrafficController(grid_site(), params, list(cells), lambda: now[0])
    for robot, cell in cells.items():
        report(core, robot, cell, (battery or {}).get(robot, 90.0))
    core.on_run_mode("RUN")
    return core, now


def report(core, robot, cell, battery=90.0):
    x, y = core.site.waypoint_pose(cell)
    for _ in range(core.params.release_confirm_samples):
        core.on_robot_state(robot, x, y, 0.0, "IDLE", battery)


def test_the_battery_is_not_looked_at_by_default():
    assert load_params().battery_low_pct == 0.0
    core, _ = make({"r1": FRONT}, low=0.0, battery={"r1": 5.0})
    core.on_dispatch(1, WORK)
    core.tick()
    assert core.robot_view("r1")["called"] is True
    assert core.robot_view("r1")["charging"] is False


def test_a_robot_with_a_low_battery_is_not_called_and_another_goes():
    core, _ = make({"r1": FRONT, "r2": AWAY}, battery={"r1": 20.0})
    core.on_dispatch(1, WORK)
    out = core.tick()
    assert core.robot_view("r1")["called"] is False
    assert core.robot_view("r1")["charging"] is True
    assert core.robot_view("r2")["called"] is True
    assert codes(out).count("ROBOT_BATTERY_LOW") == 1
    report(core, "r1", FRONT, 15.0)
    assert "ROBOT_BATTERY_LOW" not in codes(core.tick())  # 한 번만 알린다


def test_an_unknown_battery_value_changes_nothing():
    for value in (-1.0, 0.0, float("nan")):
        core, _ = make({"r1": FRONT}, battery={"r1": value})
        core.on_dispatch(1, WORK)
        core.tick()
        assert core.robot_view("r1")["called"] is True, value


def test_a_called_robot_whose_battery_runs_low_before_the_commit_zone_goes_to_charge():
    core, _ = make({"r1": (3, 6), "r2": AWAY})
    core.on_dispatch(1, WORK)
    core.tick()
    assert core.robot_view("r1")["state"] == "TO_WORK"
    report(core, "r1", (3, 6), 10.0)
    core.tick()
    assert core.robot_view("r1")["called"] is False
    assert core.robot_view("r1")["state"] in ("TO_PARK", "PARKED")
    assert core.robot_view("r2")["called"] is True  # 호출은 다른 로봇이 받는다


def test_a_robot_that_is_low_keeps_out_of_work_until_it_is_charged():
    core, _ = make({"r1": FRONT}, battery={"r1": 10.0})
    core.on_dispatch(1, WORK)
    core.tick()
    report(core, "r1", FRONT, 60.0)  # 충전 중: 아직 battery_charged_pct(80) 아래
    assert "ROBOT_CHARGED" not in codes(core.tick())
    assert core.robot_view("r1")["called"] is False
    report(core, "r1", FRONT, 80.0)
    out = core.tick()
    assert codes(out).count("ROBOT_CHARGED") == 1
    assert core.robot_view("r1")["charging"] is False
    assert core.robot_view("r1")["called"] is True


def test_a_robot_at_the_dock_that_runs_low_leaves_for_the_parking_when_released():
    """하던 일은 마친다: 도크에서 놓아 주면 열린 호출이 있어도 충전하러 간다."""
    core, _ = make({"r1": (4, 3)})
    core.on_dispatch(1, WORK)
    drive(core, "r1", lambda: at_station(core, "r1"))
    core.on_dispatch(2, WORK)
    core.on_robot_state("r1", *core.site.waypoint_pose(core._missions["r1"].path[-1]), 0.0,
                        "IDLE", 10.0)
    core.on_release("r1")
    core.tick()
    core.tick()
    assert core.robot_view("r1")["called"] is False
    assert core.robot_view("r1")["state"] == "TO_PARK"


def test_on_lanes_a_charging_robot_at_the_front_of_the_queue_is_not_overtaken():
    """레인 대기 줄은 한 줄이다: 앞에서 충전하는 로봇을 뒤 로봇이 지나갈 수 없다."""
    core, _ = make({"r1": FRONT, "r2": BEHIND}, battery={"r1": 10.0})
    core.on_dispatch(1, WORK)
    core.tick()
    assert core.robot_view("r1")["called"] is False
    assert core.robot_view("r2")["called"] is False



def test_on_the_free_floor_a_charging_robot_does_not_hold_up_the_robots_behind_it():
    """자유바닥은 대기 지점에서 곧장 나간다: 충전 중인 로봇은 호출만 받지 않는다."""
    from fleet_core.manager import FleetManager
    from fleet_core.traffic.lane_map import load_lane_map
    site = load_lane_map()
    params = replace(load_params(), traffic_mode="freefloor", battery_low_pct=20.0)
    core = FleetManager(site, params, ["a"], lambda: 0.0)
    core.on_robot_state("a", *site.waypoint_pose(site.parking[-1]), 0.0, "IDLE", 10.0)
    info = core.traffic._robot_info(core.traffic._missions["a"])
    assert core.traffic._missions["a"].charge
    assert not info.eligible and info.available
