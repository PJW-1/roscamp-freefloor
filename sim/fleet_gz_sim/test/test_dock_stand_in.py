"""시뮬 전용 도크 대역의 판정. ROS 없이 돈다."""
import pytest
from fleet_gz_sim.dock_stand_in import dock_outcome, dock_phase


def test_phases_follow_fake_pinky():
    assert dock_phase(0.0, 3.0) == "APPROACH"
    assert dock_phase(1.6, 3.0) == "ALIGN"
    assert dock_phase(3.0, 3.0) == "DONE"


def test_docked_only_when_the_robot_already_stands_at_the_dock_waypoint():
    """대역은 움직이지 않는다. 관제의 move_to가 데려다 놓은 자리를 실제 위치로 잴 뿐이다."""
    ok, ex, ey, eyaw = dock_outcome((1.06, 1.53, 0.3), (1.05, 1.55))
    assert ok and ex == pytest.approx(1.0) and ey == pytest.approx(-2.0) and eyaw == 0.0
    assert dock_outcome((1.30, 1.55, 0.0), (1.05, 1.55))[0] is False  # 0.25 m 떨어짐
    assert dock_outcome((1.05, 1.70, 0.0), (1.05, 1.55), reach_m=0.15)[0] is True  # 경계
