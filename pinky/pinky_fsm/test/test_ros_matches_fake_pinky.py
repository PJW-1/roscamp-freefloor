"""같은 목표 순서에 FakePinky와 pinky_fsm(+FakeNav)이 같은 결과와 상태 값을 내는지 본다.

FakePinky는 관제가 시험받은 기준 동작이다(안내 1절, 4절 2번). fake_robots는 이 비교에만 쓰고
pinky_fsm 패키지 코드는 그것을 import하지 않는다.

Compared: lane goals, replacement, cancel, point goals, heartbeat loss, PATH_NOT_CONNECTED,
BLOCKED (hold, cancel while blocked, resume) and LOCALIZING at start. The known differences are
listed in KNOWN_DIFFERENCES below; the dock one is pinned by a test so the list cannot go stale.
"""
import time

import pytest
import yaml

pytest.importorskip("rclpy")
pytest.importorskip("nav2_msgs.action")
pytest.importorskip("fake_robots.fake_pinky_node")

from conftest import CONTRACT, Harness, wait_until  # noqa: E402
from fake_nav import FakeNav  # noqa: E402
from fleet_interfaces.action import Dock  # noqa: E402
from fleet_interfaces.msg import RobotState  # noqa: E402
from probe import Probe, lane_goal  # noqa: E402

SPEED = 0.5
LOCALIZING_S = 3.0  # FakePinky reports LOCALIZING this long after it starts
OBSTACLE = (0.35, 1.75)  # 0.2 m in front of the robot on P-3, on the lane to P-4

# Behaviour that differs on purpose or is not built yet. Keep it in step with the tests.
KNOWN_DIFFERENCES = {
    "dock": "pinky_fsm has no pinky_dock yet and fails every dock (DOCK_UNAVAILABLE in the log); "
            "FakePinky succeeds after dock.duration_s. Precise docking is a separate task.",
    "current_goal while docking": "both report the dock_id (Pinky contract section 2), but "
                                  "pinky_fsm never reaches DOCKING, so it is not compared.",
    "goal while LOCALIZING": "pinky_fsm fails it with NAV_ABORTED; FakePinky drives it. The fleet "
                             "manager sends no goal to a LOCALIZING robot.",
    "pose lost while moving": "pinky_fsm stops and fails with NAV_ABORTED; FakePinky has no fault "
                              "that loses the pose.",
}


class World:
    """What a step may change around the robot: an obstacle ahead, the pose becoming known."""

    def __init__(self, block, localize):
        self.block = block
        self.localize = localize


def scenario_file(tmp_path, localizing_s=0.0):
    robot = {"speed_mps": SPEED, "pose_noise_m": 0.0}
    data = {
        "seed": 3, "speed_factor": 1.0, "timeout_s": 600,
        "stock": {"arm1": {"A": 3}, "arm2": {"C": 3}},
        "robots": {"pinky1": {"start": "P-3", "localizing_s": localizing_s, **robot},
                   "pinky2": {"start": "P-5", **robot},
                   "pinky3": {"start": "P-4", **robot}},
        "arm": {"seconds_per_item": 0.2, "swap_seconds": 0.4, "fail_rate": 0.0},
        "dock": {"duration_s": 0.6, "error_cm": 1.0},
    }
    path = tmp_path / "scenario.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return str(path)


def fake_pinky(harness, tmp_path, heartbeat_timeout_s, localizing):
    from fake_robots.fake_pinky_node import FakePinkyNode
    node = harness.add(FakePinkyNode(parameter_overrides=harness.params(
        robot_id="pinky1", contract_file=str(CONTRACT),
        scenario_file=scenario_file(tmp_path, LOCALIZING_S if localizing else 0.0),
        state_rate_hz=10.0, heartbeat_timeout_s=heartbeat_timeout_s)))
    blocked = [False]
    # Test hook: the obstacle comes and goes when a step says so, not on the fault schedule
    node._faults.obstacle = lambda _t: blocked[0]
    # FakePinky stops reporting LOCALIZING by itself after LOCALIZING_S
    return World(block=lambda on: blocked.__setitem__(0, on), localize=lambda: None)


def real_fsm(harness, _tmp_path, heartbeat_timeout_s, localizing):
    from pinky_fsm.node import PinkyFsmNode
    nav = harness.add(FakeNav(speed=SPEED, publish_tf=not localizing))
    harness.add(PinkyFsmNode(parameter_overrides=harness.params(
        robot_id="pinky1", contract_file=str(CONTRACT),
        heartbeat_timeout_s=heartbeat_timeout_s)))

    def localize():
        nav.publish_tf = True

    return World(block=lambda on: setattr(nav, "obstacle", OBSTACLE if on else None),
                 localize=localize)


def run(rclpy_session, tmp_path, make, steps, heartbeat_timeout_s=0.0, localizing=False):
    harness = Harness(rclpy_session)
    try:
        world = make(harness, tmp_path, heartbeat_timeout_s, localizing)
        probe = harness.add(Probe())
        assert probe.ready()
        if not localizing:
            assert wait_until(lambda: (s := probe.last_state()) is not None
                              and s.state == RobotState.STATE_IDLE, 10.0)
        return steps(probe, world)
    finally:
        harness.close()


def outcome(probe, handle, t0):
    res = probe.result(handle).result
    time.sleep(0.3)  # 끝난 뒤의 상태도 본다
    return res.success, res.reason, sorted(set(probe.states_since(t0)))


def last_state_is(probe, state, timeout):
    return wait_until(lambda: (s := probe.last_state()) is not None and s.state == state, timeout)


def stood_still(probe, since):
    poses = {(round(m.pose.x, 3), round(m.pose.y, 3)) for t, m in probe.states if t >= since}
    return len(poses) == 1


def lane_steps(probe, _world):
    out = {}
    t0 = time.monotonic()
    out["arrive"] = outcome(probe, probe.send(lane_goal(["P-3", "P-4"])), t0)

    t0 = time.monotonic()
    first = probe.send(lane_goal(["P-4", "P-5"]))
    time.sleep(0.3)
    second = probe.send(lane_goal(["P-4", "P-5", "P-6"]))
    out["replaced"] = probe.result(first).result.success, probe.result(first).result.reason
    out["extended"] = outcome(probe, second, t0)

    t0 = time.monotonic()
    h = probe.send(lane_goal(["P-6", "N"]))
    time.sleep(0.4)
    h.cancel_goal_async()
    out["canceled"] = outcome(probe, h, t0)

    t0 = time.monotonic()
    out["point"] = outcome(probe, probe.send(lane_goal(["P-6", "N"], path=False)), t0)
    return out


def heartbeat_steps(probe, _world):
    t0 = time.monotonic()
    return {"lost": outcome(probe, probe.send(lane_goal(["P-3", "P-4", "P-5", "P-6"])), t0)}


def path_steps(probe, _world):
    out = {}
    t0 = time.monotonic()
    out["idle"] = outcome(probe, probe.send(lane_goal(["P-5", "P-6"])), t0)  # 0.67 m away

    t0 = time.monotonic()
    first = probe.send(lane_goal(["P-3", "P-4", "P-5", "P-6"]))
    time.sleep(0.3)
    far = probe.send(lane_goal(["P-6", "N"]))
    out["replaced"] = probe.result(first).result.success, probe.result(first).result.reason
    out["moving"] = outcome(probe, far, t0)
    mark = time.monotonic()
    time.sleep(0.3)
    out["still"] = stood_still(probe, mark)
    return out


def blocked_steps(probe, world):
    out = {}
    goal = lane_goal(["P-3", "P-4", "P-5"])
    world.block(True)
    h = probe.send(goal)
    assert last_state_is(probe, RobotState.STATE_BLOCKED, 8.0)
    h.cancel_goal_async()
    res = probe.result(h).result
    mark = time.monotonic()
    time.sleep(0.5)
    out["canceled"] = res.success, res.reason, sorted(set(probe.states_since(mark)))
    world.block(False)
    out["cleared"] = last_state_is(probe, RobotState.STATE_IDLE, 8.0)

    world.block(True)
    t0 = time.monotonic()
    h = probe.send(goal)
    assert last_state_is(probe, RobotState.STATE_BLOCKED, 8.0)
    world.block(False)
    out["resumed"] = outcome(probe, h, t0)
    return out


def localizing_steps(probe, world):
    assert wait_until(lambda: probe.states, 5.0)
    first = probe.states[0][1].state
    time.sleep(1.0)
    world.localize()
    assert last_state_is(probe, RobotState.STATE_IDLE, 10.0)
    localizing = [m for _, m in probe.states if m.state == RobotState.STATE_LOCALIZING]
    return {"first": first, "states": sorted(set(probe.states_since(0.0))),
            "zero_pose": all(m.pose.x == 0.0 and m.pose.y == 0.0 for m in localizing)}


def dock_steps(probe, _world):
    return probe.result(probe.send(Dock.Goal(dock_id="WORK"), client=probe.dock)).result.success


def test_lane_goals_give_the_same_results_as_fake_pinky(rclpy_session, tmp_path):
    fake = run(rclpy_session, tmp_path, fake_pinky, lane_steps)
    real = run(rclpy_session, tmp_path, real_fsm, lane_steps)
    assert real == fake
    assert fake["arrive"] == (True, "", ["IDLE", "MOVING"])
    assert fake["replaced"] == (False, "CANCELED")
    assert fake["canceled"][:2] == (False, "CANCELED")


def test_heartbeat_loss_gives_the_same_result_as_fake_pinky(rclpy_session, tmp_path):
    fake = run(rclpy_session, tmp_path, fake_pinky, heartbeat_steps, heartbeat_timeout_s=1.0)
    real = run(rclpy_session, tmp_path, real_fsm, heartbeat_steps, heartbeat_timeout_s=1.0)
    assert real == fake
    assert fake["lost"][:2] == (False, "HEARTBEAT_LOST")


def test_a_path_away_from_the_robot_gives_the_same_result_as_fake_pinky(rclpy_session, tmp_path):
    fake = run(rclpy_session, tmp_path, fake_pinky, path_steps)
    real = run(rclpy_session, tmp_path, real_fsm, path_steps)
    assert real == fake
    assert fake["idle"] == (False, "PATH_NOT_CONNECTED", ["IDLE"])
    assert fake["replaced"] == (False, "CANCELED")
    assert fake["moving"][:2] == (False, "PATH_NOT_CONNECTED")
    assert fake["still"] is True


def test_an_obstacle_gives_the_same_blocked_behaviour_as_fake_pinky(rclpy_session, tmp_path):
    fake = run(rclpy_session, tmp_path, fake_pinky, blocked_steps)
    real = run(rclpy_session, tmp_path, real_fsm, blocked_steps)
    assert real == fake
    assert fake["canceled"] == (False, "CANCELED", ["BLOCKED"])
    assert fake["cleared"] is True
    assert fake["resumed"] == (True, "", ["BLOCKED", "IDLE", "MOVING"])


def test_localizing_at_start_matches_fake_pinky(rclpy_session, tmp_path):
    fake = run(rclpy_session, tmp_path, fake_pinky, localizing_steps, localizing=True)
    real = run(rclpy_session, tmp_path, real_fsm, localizing_steps, localizing=True)
    assert real == fake
    assert fake == {"first": "LOCALIZING", "states": ["IDLE", "LOCALIZING"], "zero_pose": True}


def test_dock_is_still_a_known_difference(rclpy_session, tmp_path):
    """Pins KNOWN_DIFFERENCES["dock"]. When pinky_dock arrives this fails: compare docks then."""
    assert run(rclpy_session, tmp_path, fake_pinky, dock_steps) is True
    assert run(rclpy_session, tmp_path, real_fsm, dock_steps) is False
