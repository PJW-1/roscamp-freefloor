"""gz.launch.py가 시나리오에서 정하는 것: 핑키를 놓는 자리와 방향, 돌릴 수 없는 시나리오."""
import math

import pytest

pytest.importorskip("fleet_core")
pytest.importorskip("fake_robots")
from fleet_gz_sim.fleet_setup import ScenarioUnfit, load, scenario_path, start_poses  # noqa: E402


def share():
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    try:
        return get_package_share_directory("fake_robots")
    except PackageNotFoundError:
        pytest.skip("fake_robots is not installed")


def test_pinkys_start_on_their_scenario_waypoints_facing_along_the_lane_in():
    """basic_orders: pinky1 P-6, pinky2 P-5, pinky3 P-4. 대기 줄을 따라 들어온 방향을 본다."""
    site, scenario = load(scenario_path("basic_orders", share()))
    poses = start_poses(site, scenario)
    assert list(poses) == ["pinky1", "pinky2", "pinky3"]
    assert poses["pinky1"] == pytest.approx((1.20, 2.05, 0.0))  # P-5 → P-6, 동쪽
    assert poses["pinky2"] == pytest.approx((0.80, 2.05, math.atan2(0.10, 0.45)))  # P-4 → P-5
    assert poses["pinky3"] == pytest.approx((0.35, 1.95, math.pi / 2))  # P-3 → P-4, 북쪽


def test_a_scenario_with_robot_faults_is_refused():
    """로봇 고장(stuck 등)은 가짜 핑키가 일으킨다. 진짜 pinky_fsm에는 일으킬 곳이 없다."""
    with pytest.raises(ScenarioUnfit, match="stuck"):
        load(scenario_path("stuck_robot", share()))


def test_a_scenario_is_a_name_in_fake_robots_or_a_file(tmp_path):
    assert scenario_path("basic_orders", share()).name == "basic_orders.yaml"
    mine = tmp_path / "mine.yaml"
    mine.write_text("{}")
    assert scenario_path(str(mine), share()) == mine
    with pytest.raises(ScenarioUnfit, match="nowhere"):
        scenario_path("no_such_scenario", share())
