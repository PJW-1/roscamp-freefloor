"""local costmap에서 경로 앞 물체를 찾는다. ROS 없이 돈다."""
import math

import pytest
from pinky_fsm.costmap import Grid, obstacle_on, to_frame


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
