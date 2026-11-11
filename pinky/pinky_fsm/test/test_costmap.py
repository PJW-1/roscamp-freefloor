"""local costmap에서 경로 앞 물체를 찾는다. ROS 없이 돈다."""
import math

import pytest
from pinky_fsm.costmap import Grid, invert_frame, obstacle_location, obstacle_on, to_frame


def grid(cells=None, origin=(0.0, 0.0)):
    data = [0] * (40 * 40)
    for (i, j), v in (cells or {}).items():
        data[j * 40 + i] = v
    return Grid(resolution=0.025, origin_x=origin[0], origin_y=origin[1], origin_yaw=0.0,
                width=40, height=40, data=data)


def test_inscribed_or_lethal_cell_on_the_points_is_an_obstacle():
    g = grid({(20, 10): 100, (30, 10): 99})
    assert obstacle_on(g, [(20 * 0.025 + 0.01, 10 * 0.025 + 0.01)])
    assert obstacle_on(g, [(30 * 0.025 + 0.01, 10 * 0.025 + 0.01)])


def test_inflation_below_inscribed_and_unknown_are_not_obstacles():
    g = grid({(20, 10): 98, (21, 10): -1})
    assert not obstacle_on(g, [(20 * 0.025 + 0.01, 0.26), (21 * 0.025 + 0.01, 0.26)])


def test_points_outside_the_window_are_ignored():
    g = grid({(0, 0): 100})
    assert not obstacle_on(g, [(-0.5, 0.0), (5.0, 5.0)])


def test_grid_origin_is_respected():
    g = grid({(0, 0): 100}, origin=(2.0, 3.0))
    assert obstacle_on(g, [(2.01, 3.01)])
    assert not obstacle_on(g, [(0.01, 0.01)])


def test_to_frame_applies_translation_then_rotation():
    # map 점을 odom 틀로: odom에서 본 map의 자세가 (1, 0, 90도)
    [(x, y)] = to_frame([(1.0, 0.0)], (1.0, 0.0, math.pi / 2))
    assert (x, y) == (pytest.approx(1.0), pytest.approx(1.0))


def block(i0, j0, i1, j1, v=100):
    return {(i, j): v for i in range(i0, i1 + 1) for j in range(j0, j1 + 1)}


def test_location_none_without_obstacle():
    assert obstacle_location(grid(), [(0.1, 0.1), (0.5, 0.1)]) is None


def test_location_is_blob_centre_and_radius():
    # 4x4 칸(0.1 m) 물체가 경로 위에 있다
    g = grid(block(20, 8, 23, 11))
    x, y, r = obstacle_location(g, [(0.1, 0.25), (0.6, 0.25), (0.9, 0.25)])
    assert (x, y) == (pytest.approx(22 * 0.025), pytest.approx(10 * 0.025))
    assert r == pytest.approx(0.05 * math.sqrt(2), abs=1e-6)


def test_location_ignores_other_blob_off_the_path():
    cells = block(20, 8, 21, 9)
    cells.update(block(5, 30, 8, 33))
    x, y, _ = obstacle_location(grid(cells), [(0.1, 0.22), (0.52, 0.22), (0.7, 0.22)])
    assert x == pytest.approx(21 * 0.025)


def test_location_inscribed_only_falls_back_to_point_radius_zero():
    g = grid({(20, 10): 99})
    x, y, r = obstacle_location(g, [(0.1, 0.26), (0.5, 0.26), (0.9, 0.26)])
    assert r == 0.0 and x == pytest.approx(0.5, abs=0.03)


def test_location_uses_first_blocked_point_along_path():
    cells = block(10, 10, 11, 11)
    cells.update(block(30, 10, 31, 11))
    x, _, _ = obstacle_location(grid(cells), [(0.05, 0.27), (0.27, 0.27), (0.77, 0.27)])
    assert x == pytest.approx(11 * 0.025)


def test_location_respects_grid_origin():
    g = grid(block(0, 0, 1, 1), origin=(2.0, 3.0))
    x, y, _ = obstacle_location(g, [(2.01, 3.01)])
    assert (x, y) == (pytest.approx(2.025), pytest.approx(3.025))


def test_invert_frame_round_trips():
    tf = (1.0, -2.0, 0.7)
    pts = [(0.3, 0.4), (-1.0, 2.0)]
    back = to_frame(to_frame(pts, tf), invert_frame(tf))
    for (a, b), (c, d) in zip(pts, back):
        assert (a, b) == (pytest.approx(c), pytest.approx(d))
