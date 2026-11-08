"""핑키 자기 복구(제안, 2026-11-09 공유 명세 '로봇 쪽'): 짧게 후진하고 다시 간다."""
import math

import pytest
from fake_robots.motion import Motion
from fake_robots.recovery import ObstacleRecovery, RecoveryParams

DT = 0.05
SPEED = 0.2


def label(rec, driving=True):
    if driving and rec.recovering:
        return "RECOVERING"
    if rec.blocked:
        return "BLOCKED"
    return "MOVING" if driving else "IDLE"


class Run:
    """직선 (0,0)->(3,0)을 가는 로봇 하나. 물체는 시각으로 켜고 끈다."""

    def __init__(self, start_x=0.0, params=None):
        self.motion = Motion(start_x, 0.0)
        self.motion.set_target(3.0, 0.0)
        self.rec = ObstacleRecovery(params)
        self.t = 0.0
        self.states = []
        self.xs = []

    def tick(self, ahead=False, others=(), wall_behind_m=None):
        self.t += DT
        self.rec.watch(self.t, ahead, True, self.motion)
        self.rec.drive(self.t, DT, self.motion, SPEED, others, wall_behind_m)
        state = label(self.rec)
        if not self.states or self.states[-1] != state:
            self.states.append(state)
        self.xs.append(self.motion.x)

    def drive_to(self, x):
        while self.motion.x < x - 1e-9:
            self.tick()
        assert self.motion.x == pytest.approx(x, abs=SPEED * DT)
        self.xs = [self.motion.x]  # 뒤의 검사는 여기서부터 본다

    def run(self, seconds, **kw):
        for _ in range(round(seconds / DT)):
            self.tick(**kw)


def reversals(xs, eps=1e-6):
    """앞으로 가다 뒤로 가기 시작한 횟수(후진 수)."""
    count, going = 0, 1
    for a, b in zip(xs, xs[1:], strict=False):
        step = b - a
        if step < -eps and going > 0:
            count, going = count + 1, -1
        elif step > eps:
            going = 1
    return count


def test_the_back_up_distance_rule():
    """d = min(0.15, s_ref - 0.08 - 0.15). s_ref는 마지막으로 지난 path 꼭짓점부터 온 거리."""
    p = RecoveryParams()
    assert p.backup_margin_m == pytest.approx(0.2 - 0.08 + 0.03)  # release_distance_m에서
    assert p.backup_distance(1.0) == pytest.approx(0.15)  # backup_max_m
    assert p.backup_distance(0.3) == pytest.approx(0.07)  # s_ref - 0.08 - 0.15
    assert p.backup_distance(0.25) == pytest.approx(0.02)  # backup_min_m는 아직 후진한다
    assert p.backup_distance(0.245) == 0.0  # 0.015 < 0.02: 후진 안 함
    assert p.backup_distance(0.0) == 0.0


def test_a_persistent_obstacle_gives_two_attempts_then_blocked():
    r = Run()
    r.drive_to(1.0)
    r.run(20.0, ahead=True)
    assert r.states == ["MOVING", "RECOVERING", "BLOCKED"]
    assert r.rec.used == 2
    assert reversals(r.xs) == 2, "one back-up per attempt"
    assert min(r.xs) == pytest.approx(1.0 - 0.15, abs=1e-6)
    assert r.motion.x == pytest.approx(1.0, abs=1e-6), "it drives back up to the object"


def test_the_attempts_take_their_time_before_blocked():
    """1 s 서기, 0.15 m 후진(0.08 m/s), 0.15 m 되돌아가기(0.2 m/s)를 두 번, 1 s 서기."""
    r = Run()
    r.drive_to(1.0)
    start = r.t
    while not r.rec.blocked:
        r.tick(ahead=True)
    expected = 1.0 + 2 * (0.15 / 0.08 + 0.15 / SPEED + 1.0)
    assert r.t - start == pytest.approx(expected, abs=4 * DT)


def test_near_the_path_start_there_is_no_back_up():
    r = Run()
    r.drive_to(0.24)  # d = 0.24 - 0.23 < 0.02
    r.run(5.0, ahead=True)
    assert r.states == ["MOVING", "RECOVERING", "BLOCKED"]
    assert min(r.xs[r.xs.index(max(r.xs)):]) == pytest.approx(max(r.xs)), "it backed up"


def test_an_obstacle_at_the_start_gives_recovering_twice_then_blocked_after_three_seconds():
    """pinky_fsm 비교 시험과 같은 경우: 물체가 0.2 m 앞, 로봇은 path 첫 점에 있다."""
    r = Run()
    while not r.rec.blocked:
        r.tick(ahead=True)
    # 처음 1 s는 MOVING이다(물체 앞에 섰지만 아직 알리지 않는다)
    assert r.states == ["MOVING", "RECOVERING", "BLOCKED"] and r.rec.used == 2
    assert r.t == pytest.approx(3.0, abs=2 * DT)
    assert set(r.xs) == {0.0}


def test_no_back_up_into_a_robot_behind():
    r = Run()
    r.drive_to(1.0)
    behind = [(0.75, 0.0, 0.08)]  # 후진 끝(0.85)에서 0.10 m < 0.08 + 0.08
    r.run(3.0, ahead=True, others=behind)
    assert "RECOVERING" in r.states and r.rec.used >= 1
    assert min(r.xs[-60:]) == pytest.approx(r.motion.x), "it backed into the robot"


def test_a_robot_far_enough_behind_or_ahead_does_not_stop_the_back_up():
    r = Run()
    r.drive_to(1.0)
    r.run(3.5, ahead=True, others=[(0.6, 0.0, 0.08), (1.1, 0.0, 0.08)])
    assert min(r.xs) == pytest.approx(0.85, abs=1e-6)


def test_no_back_up_into_an_obstacle_behind():
    r = Run()
    r.drive_to(1.0)
    r.run(3.0, ahead=True, wall_behind_m=0.2)  # 후진 끝에서 0.05 m < 반지름 0.08
    assert "RECOVERING" in r.states
    assert min(r.xs[-60:]) == pytest.approx(r.motion.x)


def test_a_robot_coming_up_behind_stops_the_back_up_early():
    r = Run()
    r.drive_to(1.0)
    r.run(1.0 + 0.5, ahead=True)  # 1 s 서고 0.5 s 후진: 0.04 m
    backed = r.motion.x
    assert 0.95 < backed < 0.97
    r.run(1.0, ahead=True, others=[(backed - 0.25, 0.0, 0.08)])
    assert min(r.xs) == pytest.approx(backed, abs=1e-6), "it kept backing up"
    assert r.rec.recovering


def test_after_the_obstacle_goes_it_resumes_and_says_moving():
    r = Run()
    r.drive_to(1.0)
    r.run(1.2, ahead=True)  # 첫 시도가 시작됐다
    assert r.rec.recovering
    while r.motion.x < 1.04:
        r.tick()
        assert r.rec.recovering
    r.drive_to(1.1)
    assert r.states == ["MOVING", "RECOVERING", "MOVING"]
    assert r.rec.used == 1, "the attempt stays counted"


def test_attempts_reset_after_three_tenths_of_a_metre():
    r = Run()
    r.drive_to(1.0)
    r.run(1.2, ahead=True)
    r.drive_to(1.25)
    assert r.rec.used == 1
    r.drive_to(1.3)
    r.tick()
    assert r.rec.used == 0


def test_attempts_do_not_reset_by_a_new_goal_but_do_on_success():
    r = Run()
    r.drive_to(1.0)
    r.run(1.2, ahead=True)
    r.rec.restart()  # 바꾸는 목표나 취소
    assert not r.rec.recovering and r.rec.used == 1
    r.rec.succeeded()
    assert r.rec.used == 0


def test_a_cancel_stops_the_back_up():
    r = Run()
    r.drive_to(1.0)
    r.run(1.5, ahead=True)
    assert r.rec.backing_up
    r.rec.restart()
    assert not r.rec.backing_up and not r.rec.recovering


def test_with_no_attempts_it_is_blocked_after_a_second_as_before():
    r = Run(params=RecoveryParams(recovery_attempts=0))
    r.drive_to(1.0)
    start = r.t
    while not r.rec.blocked:
        r.tick(ahead=True)
    assert r.t - start == pytest.approx(1.0, abs=2 * DT)
    assert r.states == ["MOVING", "BLOCKED"]


def test_blocked_ends_a_second_after_the_object_goes():
    r = Run(params=RecoveryParams(recovery_attempts=0))
    r.drive_to(1.0)
    r.run(2.0, ahead=True)
    assert r.rec.blocked
    r.run(0.9)
    assert r.rec.blocked and r.motion.x == pytest.approx(1.0, abs=1e-6)
    r.run(0.3)
    assert not r.rec.blocked and r.motion.x > 1.0


def test_a_new_goal_while_the_object_is_there_does_not_move():
    r = Run()
    r.drive_to(1.0)
    r.run(0.5, ahead=True)
    r.rec.restart()
    r.motion.set_target(3.0, 1.0)
    r.run(0.5, ahead=True)
    assert r.motion.x == pytest.approx(1.0, abs=1e-6) and r.motion.y == 0.0


def test_an_obstacle_behind_that_comes_during_the_back_up_stops_it():
    r = Run()
    r.drive_to(1.0)
    r.run(1.5, ahead=True)  # 후진하는 중
    backed = r.motion.x
    assert backed < 1.0 and r.rec.backing_up
    r.run(1.0, ahead=True, wall_behind_m=0.1)
    assert min(r.xs) == pytest.approx(backed, abs=1e-6), "it kept backing up"


def test_a_robot_beside_the_way_back_round_a_corner_stops_the_back_up():
    """모서리를 막 돌았으면 지나온 선이 지금 방향의 앞쪽 옆에 있을 수 있다(검토 지적)."""
    m = Motion(0.0, 0.0)
    m.set_target(0.0, 0.25, via=[(1.0, 0.0), (1.0, 0.25)])  # 동, 북, 서로 되돌아간다
    rec = ObstacleRecovery()
    t = 0.0
    while m.travelled < 1.3 - 1e-9:  # (0.95, 0.25), 서쪽을 본다
        t += DT
        rec.watch(t, False, True, m)
        rec.drive(t, DT, m, SPEED)
    start = (m.x, m.y)
    other = (0.9, 0.05, 0.08)  # 후진 끝 (1.0, 0.15)에서 0.14 m. 지금 방향(서쪽)으로는 앞
    assert (other[0] - m.x) * math.cos(m.yaw) + (other[1] - m.y) * math.sin(m.yaw) > 0
    moved = 0.0
    for _ in range(80):
        t += DT
        rec.watch(t, True, True, m)
        rec.drive(t, DT, m, SPEED, [other])
        moved = max(moved, math.dist(start, (m.x, m.y)))
    assert rec.used >= 1
    assert moved < 1e-6, "it backed towards the robot"


def test_a_robot_just_ahead_does_not_stop_the_back_up():
    r = Run()
    r.drive_to(1.0)
    r.run(3.5, ahead=True, others=[(1.1, 0.0, 0.08)])  # 몸은 이미 닿아 있다. 멀어지는 건 된다
    assert min(r.xs) == pytest.approx(0.85, abs=1e-6)


def test_the_back_up_is_measured_from_the_last_vertex_passed():
    """꼭짓점 뒤 0.2 m는 다른 로봇이 설 수 있다(구역 해제). 중심은 꼭짓점에서 0.23 m 앞에 둔다."""
    m = Motion(0.0, 0.0)
    m.set_target(1.0, 1.0, via=[(1.0, 0.0)])
    rec = ObstacleRecovery()
    t = 0.0
    while m.travelled < 1.27 - 1e-9:  # 모서리 (1, 0)을 지나 0.27 m
        t += DT
        rec.watch(t, False, True, m)
        rec.drive(t, DT, m, SPEED)
    lowest = m.y
    for _ in range(80):
        t += DT
        rec.watch(t, True, True, m)
        rec.drive(t, DT, m, SPEED)
        lowest = min(lowest, m.y)
    assert rec.used >= 1
    assert lowest == pytest.approx(0.27 - 0.04, abs=1e-6), "d = 0.27 - 0.23"
    assert m.x == pytest.approx(1.0)


def test_a_closer_obstacle_behind_during_the_back_up_stops_it():
    """먼저 본 먼 뒤 물체가 나중에 나타난 가까운 물체를 가리지 않는다(검토 지적)."""
    r = Run()
    r.drive_to(1.0)
    r.run(1.2, ahead=True, wall_behind_m=0.5)  # 후진을 막지 않는 물체
    assert r.rec.backing_up
    r.run(0.3, ahead=True, wall_behind_m=0.5)
    backed = r.motion.x
    r.run(1.0, ahead=True, wall_behind_m=0.1)
    assert min(r.xs) == pytest.approx(backed, abs=1e-6), "it kept backing up"
