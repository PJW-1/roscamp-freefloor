"""지도에 없는 물체(object 고장)를 헤드리스 세계가 핑키처럼 다룬다.

로봇에 묶이지 않는다. 갈 길 앞에서 물체를 본 로봇은 그 앞에 서고, 자기 복구(조금 후진하고 다시
가 보기, RECOVERING)를 거쳐 BLOCKED를 보고한다(fake_robots/recovery.py). 물체가 사라져
block_clear_s 동안 안 보이면 이어 간다. 관제는 물체를 모르고 로봇의 보고만 본다.
"""
import math

import pytest
from fleet_core.traffic.lane_map import load_lane_map
from floor_world import FloorWorld
from sim_world import SimWorld

ONE = {"pinky1": {"start": "P-6", "speed_mps": 0.2, "pose_noise_m": 0.0}}
ORDER = [{"at_s": 0.0, "id": "O-001", "items": {"A": 1}}]
RADIUS = 0.06


def watch_states(world):
    """로봇이 보고한 상태 (시각, 로봇, 상태)."""
    seen = []
    report = world.core.on_robot_state

    def record(robot, x, y, yaw, state, *rest, **kw):
        seen.append((world.t, robot, state))
        return report(robot, x, y, yaw, state, *rest, **kw)

    world.core.on_robot_state = record
    return seen


def place_on_the_route(make, at_s, ahead_s):
    """물체 없이 한 번 돌려 at_s + ahead_s에 로봇이 있던 자리를 찾는다. 그 자리에 물체를 둔다."""
    world = make([])
    for _ in range(round((at_s + ahead_s) / 0.1)):
        world.step()
    r = world.virtual["pinky1"]
    assert r.goal_kind == "move", "the robot has to be driving there"
    return r.x, r.y


def lane_world(faults):
    return SimWorld(seed=3, robots=dict(ONE), orders=list(ORDER), faults=faults)


def floor_world(faults):
    return FloorWorld(seed=3, robots=dict(ONE), orders=list(ORDER), faults=faults)


def an_object(x, y, at_s, duration_s, radius=RADIUS):
    return {"type": "object", "at_s": at_s, "duration_s": duration_s, "x": x, "y": y,
            "radius_m": radius}


@pytest.mark.parametrize("make", [lane_world, floor_world], ids=["lanes", "freefloor"])
def test_an_object_far_from_every_route_changes_nothing(make):
    plain = make([])
    far = make([an_object(-0.9, -0.9, 1.0, 1000.0)])  # 웨이포인트 범위 밖, 아무도 안 간다
    for _ in range(600):
        plain.step()
        far.step()
        for name, r in plain.virtual.items():
            other = far.virtual[name]
            assert (r.x, r.y, r.heading, r.goal_id) == (other.x, other.y, other.heading,
                                                         other.goal_id), plain.t
    assert plain.reports == far.reports


@pytest.mark.parametrize("make", [lane_world, floor_world], ids=["lanes", "freefloor"])
def test_a_box_left_on_the_route_stops_the_robot_which_recovers_then_reports_blocked(make):
    x, y = place_on_the_route(make, 4.0, 1.0)
    world = make([an_object(x, y, 4.0, 30.0)])
    states = watch_states(world)
    for _ in range(round(34.0 / 0.1)):
        world.step()
    mine = [s for t, robot, s in states if robot == "pinky1" and t >= 4.0]
    assert "RECOVERING" in mine and "BLOCKED" in mine
    assert mine.index("RECOVERING") < mine.index("BLOCKED"), "self-recovery comes first"
    assert world.min_object_gap >= 0.0, "it stops before the object and never touches it"
    assert world.virtual["pinky1"].stopped_at is not None or world.virtual["pinky1"].blocked


def test_far_from_a_corner_the_robot_backs_up_a_little_before_it_tries_again():
    """후진 거리는 마지막 꼭짓점에서 온 거리로 정한다(recovery.py). 긴 직선 위에 물체를 둔다."""
    def make(faults):
        return SimWorld(seed=3, robots=dict(ONE), orders=list(ORDER), faults=faults,
                        site=load_lane_map())

    plain = make([])
    r = plain.virtual["pinky1"]
    while not (r.goal_kind == "move" and r.trail and math.dist(r.trail[-1], (r.x, r.y)) > 0.35
               and math.dist((r.x, r.y), r.via[0] if r.via else r.target) > 0.35):
        plain.step()
        assert plain.t < 120.0, "no long straight drive found"
    at_s = plain.t
    x, y = r.x + math.cos(r.heading) * 0.24, r.y + math.sin(r.heading) * 0.24
    world = make([an_object(x, y, at_s, 30.0)])
    r = world.virtual["pinky1"]
    gaps = []
    for _ in range(round((at_s + 10.0) / 0.1)):
        world.step()
        gaps.append(math.dist((r.x, r.y), (x, y)))
    stop = gaps.index(min(gaps))
    assert max(gaps[stop:]) > min(gaps) + 0.05
    assert world.min_object_gap >= 0.0


@pytest.mark.parametrize("make", [lane_world, floor_world], ids=["lanes", "freefloor"])
def test_the_robot_goes_on_when_the_box_is_gone(make):
    x, y = place_on_the_route(make, 4.0, 1.0)
    world = make([an_object(x, y, 4.0, 30.0)])
    states = watch_states(world)
    assert world.run_until(world.done, 600.0)
    after = [s for t, robot, s in states if robot == "pinky1" and t >= 36.0]
    assert "BLOCKED" not in after, "unseen for block_clear_s: the object is gone"
    assert [r.status for r in world.reports] == ["SHIPPED"]


@pytest.mark.parametrize("make", [lane_world, floor_world], ids=["lanes", "freefloor"])
def test_a_person_walking_through_only_holds_the_robot_a_moment(make):
    x, y = place_on_the_route(make, 4.0, 1.0)
    world = make([an_object(x, y, 4.0, 0.5)])
    states = watch_states(world)
    assert world.run_until(world.done, 600.0)
    assert "BLOCKED" not in [s for _, robot, s in states if robot == "pinky1"]
    assert world.min_object_gap >= 0.0
    assert world.virtual["pinky1"].stopped_at is None


def test_objects_are_there_only_for_their_time():
    world = lane_world([an_object(1.0, 0.5, 2.0, 1.0, radius=0.2), an_object(0.5, 0.5, 0.0, 9.0)])
    assert world.objects_now() == [(0.5, 0.5, RADIUS)]
    world.run(2.0)
    assert sorted(world.objects_now()) == [(0.5, 0.5, RADIUS), (1.0, 0.5, 0.2)]
    world.run(1.0)
    assert world.objects_now() == [(0.5, 0.5, RADIUS)]


def test_a_fast_robot_looks_at_least_one_step_ahead():
    """Codex: a robot that drives farther than OBJECT_LOOK_M in one step must not jump an object."""
    fast = {"pinky1": {**ONE["pinky1"], "speed_mps": 3.0}}  # 0.3 m per step
    r0 = SimWorld(robots=fast, orders=[]).virtual["pinky1"]
    world = SimWorld(robots=fast, orders=[], faults=[an_object(r0.x + 0.4, r0.y, 0.0, 60.0, 0.02)])
    r = world.virtual["pinky1"]
    r.goal_id, r.goal_kind, r.target, r.via = 1, "move", (r.x + 1.0, r.y), []
    assert world._object_ahead(r)


def test_an_object_that_appears_on_a_robot_is_not_driven_into():
    """사람이 서 있는 로봇에 바짝 붙어 선다(몸이 겹친다). 로봇은 그쪽으로 더 가지 않는다."""
    r = lane_world([]).virtual["pinky1"]
    world = lane_world([an_object(r.x + 0.1, r.y, 0.0, 600.0)])
    world.run(20.0)  # 물체 쪽으로 가까워지면 세계가 AssertionError를 낸다
    assert world.min_object_gap < 0.0

