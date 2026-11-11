"""막힌 물체 보고(제안): 가짜 핑키가 BLOCKED/RECOVERING에서 채우는 물체 위치. ROS 없이 돈다."""
import pytest
from fake_robots.motion import Motion
from fake_robots.obstacle_report import obstacle_fields, point_along
from fake_robots.recovery import ObstacleRecovery


def test_point_along_straight_and_corner():
    route = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    assert point_along(route, 0.5) == pytest.approx((0.5, 0.0))
    assert point_along(route, 1.5) == pytest.approx((1.0, 0.5))


def test_point_along_clamps_to_end_and_start():
    route = [(0.0, 0.0), (1.0, 0.0)]
    assert point_along(route, 9.0) == pytest.approx((1.0, 0.0))
    assert point_along(route, -1.0) == pytest.approx((0.0, 0.0))


def test_fields_default_is_radius_plus_gap_ahead_with_radius_zero():
    x, y, r = obstacle_fields((0.0, 0.0), [(2.0, 0.0)], 0.0, 0.08, 0.02)
    assert (x, y, r) == (pytest.approx(0.10), pytest.approx(0.0), 0.0)


def test_fields_object_is_further_when_stop_point_is_ahead():
    x, _, _ = obstacle_fields((0.0, 0.0), [(2.0, 0.0)], 0.5, 0.08, 0.02)
    assert x == pytest.approx(0.60)


def test_fields_true_centre_and_radius_win():
    assert obstacle_fields((0.0, 0.0), [(2.0, 0.0)], 0.0, 0.08, 0.02,
                           centre=(1.2, 0.3), radius=0.1) == (1.2, 0.3, 0.1)


def test_recovery_object_stays_put_while_robot_backs_up():
    m = Motion(0.0, 0.0)
    m.set_target(3.0, 0.0)
    rec = ObstacleRecovery()
    assert rec.obstacle_ahead(m) is None  # 물체가 없다
    t, dt = 0.0, 0.05
    first, backed = None, False
    while t < 8.0:
        t += dt
        rec.watch(t, t > 3.0, True, m)
        rec.drive(t, dt, m, 0.2)
        if first is None and rec.recovering:
            first, stop_x = rec.obstacle_ahead(m), m.x
        backed = backed or rec.backing_up
    assert backed and first is not None
    assert first[0] == pytest.approx(stop_x + 0.08 + 0.02, abs=1e-6)
    assert first[1] == pytest.approx(0.0, abs=1e-9) and first[2] == 0.0
    # 후진을 하고 다시 서도 물체는 같은 자리다
    x, _, _ = rec.obstacle_ahead(m)
    assert x == pytest.approx(first[0], abs=1e-6)
