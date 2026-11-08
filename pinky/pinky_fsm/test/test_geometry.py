"""경로 기하: 펼치기, 투영, 남은 선. ROS 없이 돈다."""
import math
from itertools import pairwise

import pytest
from pinky_fsm.geometry import Polyline, poses_along, quaternion_yaw, yaw_quaternion


def test_poses_along_spaces_points_at_most_one_step_apart():
    poses = poses_along([(0.0, 0.0), (0.4, 0.0), (0.4, 0.3)], final_yaw=1.0, step=0.05)
    gaps = [math.dist(a[:2], b[:2]) for a, b in pairwise(poses)]
    assert max(gaps) <= 0.05 + 1e-9
    assert poses[0][:2] == (0.0, 0.0) and poses[-1][:2] == pytest.approx((0.4, 0.3))


def test_poses_along_faces_each_segment_and_ends_on_the_final_yaw():
    poses = poses_along([(0.0, 0.0), (0.4, 0.0), (0.4, 0.3)], final_yaw=1.0, step=0.05)
    first_leg = [p for p in poses if p[1] == 0.0 and p[0] < 0.4]
    assert all(p[2] == pytest.approx(0.0) for p in first_leg)
    corner_up = [p for p in poses if p[0] == pytest.approx(0.4) and 0.0 < p[1] < 0.3]
    assert all(p[2] == pytest.approx(math.pi / 2) for p in corner_up)
    assert poses[-1][2] == pytest.approx(1.0)


def test_poses_along_one_point_keeps_it_with_the_final_yaw():
    assert poses_along([(1.0, 2.0)], final_yaw=0.5) == [(1.0, 2.0, 0.5)]


def test_poses_along_drops_repeated_points():
    poses = poses_along([(0.0, 0.0), (0.0, 0.0), (0.1, 0.0)], final_yaw=0.0, step=0.05)
    assert [p[:2] for p in poses] == [(0.0, 0.0), pytest.approx((0.05, 0.0)),
                                      pytest.approx((0.1, 0.0))]


def test_project_gives_distance_along_and_off_the_line():
    line = Polyline([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])
    s, off = line.project((0.3, 0.1))
    assert s == pytest.approx(0.3) and off == pytest.approx(0.1)
    s, off = line.project((1.2, 0.5))
    assert s == pytest.approx(1.5) and off == pytest.approx(0.2)
    assert line.length == pytest.approx(2.0)


def test_project_before_the_start_measures_to_the_first_point():
    s, off = Polyline([(0.0, 0.0), (1.0, 0.0)]).project((-0.3, 0.4))
    assert s == 0.0 and off == pytest.approx(0.5)


def test_project_can_be_limited_to_a_window():
    """같은 곳을 두 번 지나는 선에서는 이미 지난 쪽으로 뛰지 않는다."""
    line = Polyline([(0.0, 0.0), (1.0, 0.0), (1.0, 0.2), (0.0, 0.2)])
    s, _ = line.project((0.5, 0.12))
    assert s == pytest.approx(1.7)  # 가장 가까운 것은 돌아오는 쪽
    s, _ = line.project((0.5, 0.12), s_max=1.0)
    assert s == pytest.approx(0.5)


def test_rest_starts_at_the_projection_and_keeps_later_vertices():
    line = Polyline([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])
    assert line.rest(0.25) == [pytest.approx((0.25, 0.0)), (1.0, 0.0), (1.0, 1.0)]
    assert line.rest(1.0) == [pytest.approx((1.0, 0.0)), (1.0, 1.0)]
    assert line.rest(5.0) == [(1.0, 1.0)]


def test_sample_covers_the_window_in_small_steps():
    line = Polyline([(0.0, 0.0), (1.0, 0.0)])
    pts = line.sample(0.2, 0.7, step=0.1)
    assert pts[0] == pytest.approx((0.2, 0.0)) and pts[-1] == pytest.approx((0.7, 0.0))
    assert len(pts) == 6
    assert line.sample(0.9, 1.4, step=0.1)[-1] == pytest.approx((1.0, 0.0))


def test_single_point_polyline():
    line = Polyline([(1.0, 1.0), (1.0, 1.0)])
    assert line.length == 0.0
    assert line.project((1.0, 1.3)) == (0.0, pytest.approx(0.3))
    assert line.rest(0.0) == [(1.0, 1.0)]


def test_yaw_quaternion_round_trip():
    for yaw in (-3.0, -1.0, 0.0, 0.7, 3.1):
        assert quaternion_yaw(*yaw_quaternion(yaw)) == pytest.approx(yaw)


def test_heading_at_is_the_direction_of_the_segment_behind_the_point():
    """At a corner the robot came along the segment behind it; backing up goes along that one."""
    line = Polyline([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])
    assert line.heading_at(0.0) == pytest.approx(0.0)
    assert line.heading_at(0.5) == pytest.approx(0.0)
    assert line.heading_at(1.0) == pytest.approx(0.0)  # the corner itself
    assert line.heading_at(1.2) == pytest.approx(math.pi / 2)
    assert line.heading_at(9.0) == pytest.approx(math.pi / 2)
    assert Polyline([(2.0, 3.0)]).heading_at(0.0) == 0.0


def test_vertex_before_is_the_last_vertex_at_or_behind_the_point():
    line = Polyline([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])
    assert line.vertex_before(0.0) == pytest.approx(0.0)
    assert line.vertex_before(0.5) == pytest.approx(0.0)  # the first point counts
    assert line.vertex_before(1.0) == pytest.approx(1.0)  # the corner itself
    assert line.vertex_before(1.2) == pytest.approx(1.0)
    assert line.vertex_before(2.0) == pytest.approx(2.0)
    assert Polyline([(2.0, 3.0)]).vertex_before(0.5) == 0.0
