"""SimWorld와 FloorWorld가 RetreatCmd를 핑키처럼 실행한다.

path가 비면 보는 방향 그대로 뒤로 곧게(Nav2 BackUp), 있으면 그 꺾은선을 후진으로 따라간다
(로봇은 돌아서지 않는다). 달리던 이동은 먼저 서고 CANCELED로 끝난다. 남은 길에서 몸이 다른 로봇이나
지도에 없는 물체(object 고장)에 닿으면 그 자리에 서서 OBSTACLE_BEHIND다.
"""
import math

import pytest
from fleet_core.common.commands import RetreatCmd
from floor_world import FloorWorld
from sim_world import SimWorld

ONE = {"r1": {"start": "P-1", "speed_mps": 0.2, "pose_noise_m": 0.0}}
TWO = {**ONE, "r2": {"start": "P-2", "speed_mps": 0.2, "pose_noise_m": 0.0}}


def world_with(robots, **kw):
    w = SimWorld(orders=[], robots=robots, run_mode="PAUSE", **kw)  # 시험이 명령을 직접 넣는다
    w.step()
    return w


def results(w, kind="retreat"):
    seen = []
    name = f"on_{kind}_result"
    setattr(w.core.traffic, name, lambda *a: seen.append(a))
    return seen


def test_a_retreat_backs_straight_up_and_reports_success():
    w = world_with(ONE)
    seen = results(w)
    r = w.virtual["r1"]
    r.heading = math.pi / 2  # 북쪽을 본다
    x, y = r.x, r.y
    w._apply(RetreatCmd("r1", 901, 0.1, 0.1))
    w.run(2.0)
    assert seen == [("r1", 901, True, "")]
    assert math.dist((r.x, r.y), (x, y - 0.1)) < 1e-9
    assert r.heading == math.pi / 2


def test_a_retreat_runs_at_its_own_speed():
    w = world_with(ONE)
    seen = results(w)
    r = w.virtual["r1"]
    x = r.x
    w._apply(RetreatCmd("r1", 905, 0.1, 0.05))  # 0.1 m at 0.05 m/s: 2 s
    w.run(1.0)
    assert not seen
    assert math.isclose(x - r.x, 0.05, abs_tol=1e-9)
    w.run(1.1)
    assert seen == [("r1", 905, True, "")]


def test_a_robot_behind_ends_a_retreat_with_obstacle_behind():
    w = world_with(TWO)
    seen = results(w)
    r1, r2 = w.virtual["r1"], w.virtual["r2"]
    r1.heading = math.atan2(r1.y - r2.y, r1.x - r2.x)  # r2를 등진다
    start = (r1.x, r1.y)
    w._apply(RetreatCmd("r1", 902, 0.4, 0.1))
    w.run(6.0)
    assert seen and seen[0][:3] == ("r1", 902, False) and seen[0][3] == "OBSTACLE_BEHIND"
    assert (r1.x, r1.y) == start, "남은 길 전체를 보고 처음부터 서 있는다"


def test_a_robot_in_front_does_not_stop_a_retreat():
    w = world_with(TWO)
    seen = results(w)
    r1, r2 = w.virtual["r1"], w.virtual["r2"]
    r1.heading = math.atan2(r2.y - r1.y, r2.x - r1.x)  # r2를 본다
    w._apply(RetreatCmd("r1", 906, 0.1, 0.1))
    w.run(2.0)
    assert seen == [("r1", 906, True, "")]


def test_a_retreat_first_stops_the_running_move_which_ends_canceled():
    w = world_with(ONE)
    moves, retreats = results(w, "move"), results(w)
    r = w.virtual["r1"]
    r.goal_id, r.goal_kind, r.target = 41, "move", (r.x + 1.0, r.y)  # 달리던 이동
    w._apply(RetreatCmd("r1", 907, 0.1, 0.1))
    w.run(2.0)
    assert moves == [("r1", 41, False, "CANCELED")]
    assert retreats == [("r1", 907, True, "")]


def test_a_goal_that_replaces_a_retreat_ends_it_canceled():
    w = world_with(ONE)
    seen = results(w)
    w._apply(RetreatCmd("r1", 903, 0.3, 0.05))
    w.step()
    w._preempt(w.virtual["r1"])
    w.step()
    assert seen == [("r1", 903, False, "CANCELED")]  # 이동·도킹 goal도 같은 _preempt를 부른다


def test_a_stuck_robot_ends_a_retreat_with_timeout():
    w = world_with(ONE)
    seen = results(w)
    r = w.virtual["r1"]
    r.stuck = True
    start = (r.x, r.y)
    w._apply(RetreatCmd("r1", 908, 0.1, 0.1))
    w.run(3.0)
    assert seen == [("r1", 908, False, "TIMEOUT")]
    assert (r.x, r.y) == start


def test_the_report_carries_the_heading_as_yaw_only_when_asked():
    w = world_with(ONE)
    w.virtual["r1"].heading = 1.0
    w.step()
    assert w.core.traffic._missions["r1"].yaw == 0.0, "예전 세계는 yaw 0을 보고한다"
    w = world_with(ONE, report_heading=True)
    w.virtual["r1"].heading = 1.0
    w.step()
    assert w.core.traffic._missions["r1"].yaw == 1.0


def object_at(x, y, radius=0.06, at_s=0.0, duration_s=100.0):
    return {"type": "object", "at_s": at_s, "duration_s": duration_s, "x": x, "y": y,
            "radius_m": radius}


def test_an_object_behind_ends_a_retreat_with_obstacle_behind():
    """지도에 없는 물체(object 고장)도 뒤 물체다."""
    r_start = SimWorld(orders=[], robots=ONE).virtual["r1"]
    x, y = r_start.x, r_start.y
    w = world_with(ONE, faults=[object_at(x, y - 0.25)])
    seen = results(w)
    r = w.virtual["r1"]
    r.heading = math.pi / 2
    w._apply(RetreatCmd("r1", 904, 0.3, 0.1))
    w.run(4.0)
    assert seen and seen[0][:3] == ("r1", 904, False) and seen[0][3] == "OBSTACLE_BEHIND"
    assert (r.x, r.y) == (x, y)


def test_an_object_that_is_gone_does_not_stop_a_retreat():
    r_start = SimWorld(orders=[], robots=ONE).virtual["r1"]
    x, y = r_start.x, r_start.y
    w = world_with(ONE, faults=[object_at(x, y - 0.15, at_s=0.0, duration_s=0.05)])
    seen = results(w)
    w.virtual["r1"].heading = math.pi / 2
    w._apply(RetreatCmd("r1", 909, 0.1, 0.1))
    w.run(2.0)
    assert seen == [("r1", 909, True, "")]


# ---- path ---------------------------------------------------------------------------------


def test_a_retreat_follows_its_path_backwards_without_turning_round():
    w = world_with(ONE)
    seen = results(w)
    r = w.virtual["r1"]
    x, y = r.x, r.y
    r.heading = 0.0  # 동쪽을 본다
    path = ((x, y, 0.0), (x - 0.1, y, 0.0), (x - 0.1, y - 0.05, math.pi / 2))
    w._apply(RetreatCmd("r1", 910, 0.15, 0.1, path))
    w.run(0.5)
    assert math.dist((r.x, r.y), (x - 0.05, y)) < 1e-9
    assert r.heading == 0.0, "서쪽으로 후진하는 동안 동쪽을 본다"
    w.run(0.8)
    assert math.dist((r.x, r.y), (x - 0.1, y - 0.03)) < 1e-9
    assert math.isclose(r.heading, math.pi / 2), "남쪽으로 후진하는 동안 북쪽을 본다"
    w.run(0.5)
    assert seen == [("r1", 910, True, "")]
    assert math.dist((r.x, r.y), (x - 0.1, y - 0.05)) < 1e-9


def test_a_robot_behind_on_the_path_ends_it_with_obstacle_behind_where_it_stands():
    w = world_with(TWO)
    seen = results(w)
    r1, r2 = w.virtual["r1"], w.virtual["r2"]
    x, y = r1.x, r1.y
    # 처음 선분은 r2에서 멀어지고, 둘째 선분이 r2로 간다
    away = ((x - r2.x), (y - r2.y))
    norm = math.hypot(*away)
    ux, uy = away[0] / norm, away[1] / norm
    bend = (x + ux * 0.1, y + uy * 0.1)
    path = ((x, y, 0.0), (*bend, 0.0), (r2.x + ux * 0.1, r2.y + uy * 0.1, 0.0))
    length = 0.1 + math.dist(bend, path[2][:2])
    w._apply(RetreatCmd("r1", 911, length, 0.1, path))
    w.run(6.0)
    assert seen and seen[0][:3] == ("r1", 911, False) and seen[0][3] == "OBSTACLE_BEHIND"
    assert (r1.x, r1.y) == (x, y), "남은 길 전체를 본다"


def test_a_path_that_does_not_start_at_the_robot_fails():
    w = world_with(ONE)
    seen = results(w)
    r = w.virtual["r1"]
    x, y = r.x, r.y
    path = ((x + 1.0, y, 0.0), (x + 0.8, y, 0.0))
    w._apply(RetreatCmd("r1", 912, 0.2, 0.1, path))
    w.run(1.0)
    assert seen == [("r1", 912, False, "FAILED")]
    assert (r.x, r.y) == (x, y)


@pytest.mark.parametrize("cmd", [RetreatCmd("r1", 1, 0.0, 0.1),
                                 RetreatCmd("r1", 1, 0.1, 0.0),
                                 RetreatCmd("r1", 1, 5.0, 0.1)])
def test_a_retreat_out_of_its_limits_breaks_the_world(cmd):
    """거리 0, 속도 0, floor_retreat_max_m보다 먼 물러나기는 관제 결함이다."""
    w = world_with(ONE)
    with pytest.raises(AssertionError):
        w._apply(cmd)


def test_a_path_whose_length_is_not_its_distance_breaks_the_world():
    w = world_with(ONE)
    r = w.virtual["r1"]
    path = ((r.x, r.y, 0.0), (r.x - 0.2, r.y, 0.0))
    with pytest.raises(AssertionError, match="length"):
        w._apply(RetreatCmd("r1", 1, 0.3, 0.1, path))


# ---- free floor ---------------------------------------------------------------------------

FLOOR_ONE = {"pinky1": {"start": "P-6", "speed_mps": 0.2, "pose_noise_m": 0.0}}


def floor_world():
    w = FloorWorld(orders=[], robots=FLOOR_ONE, run_mode="PAUSE")
    for _ in range(5):
        w.step()
    return w


def grant_way(w, robot, points):
    """시험이 관제 대신 지나갈 칸을 잠근다."""
    r = w.virtual[robot]
    cells = set()
    for a, b in zip(points, points[1:], strict=False):
        for k in range(21):
            x = a[0] + (b[0] - a[0]) * k / 20
            y = a[1] + (b[1] - a[1]) * k / 20
            cells |= w._footprint(x, y, r.heading, turning=True)
    for cell in sorted(cells):
        assert w.core.zones.request(robot, cell), cell


def test_the_floor_world_backs_a_robot_along_its_path_on_cells_it_holds():
    w = floor_world()
    seen = results(w)
    r = w.virtual["pinky1"]
    x, y = r.x, r.y
    path = ((x, y, 0.0), (x - 0.2, y, 0.0), (x - 0.2, y - 0.2, 0.0))
    grant_way(w, "pinky1", [p[:2] for p in path])
    w._apply(RetreatCmd("pinky1", 920, 0.4, 0.1, path))
    w.run(5.0)
    assert seen == [("pinky1", 920, True, "")]
    assert math.dist((r.x, r.y), (x - 0.2, y - 0.2)) < 1e-9


def test_the_floor_world_refuses_a_retreat_over_cells_the_robot_does_not_hold():
    w = floor_world()
    r = w.virtual["pinky1"]
    r.heading = 0.0
    with pytest.raises(AssertionError, match="does not hold"):
        w._apply(RetreatCmd("pinky1", 921, 0.5, 0.1))
