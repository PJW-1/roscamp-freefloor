"""같은 목표 순서에 FakePinky와 pinky_fsm(+FakeNav)이 같은 결과와 상태 값을 내는지 본다.

FakePinky는 관제가 시험받은 기준 동작이다(안내 1절, 4절 2번). fake_robots는 이 비교에만 쓰고
pinky_fsm 패키지 코드는 그것을 import하지 않는다.

비교하지 않는 것(차이가 알려져 있다)
- PATH_NOT_CONNECTED: FakePinky의 검사는 관제 쪽 지시서 7.5에서 들어간다. 아직 없다.
- dock: FakePinky는 시간이 지나면 성공하고, pinky_fsm은 pinky_dock이 없어 실패를 돌려준다.
"""
import time

import pytest
import yaml

pytest.importorskip("rclpy")
pytest.importorskip("nav2_msgs.action")
pytest.importorskip("fake_robots.fake_pinky_node")

from conftest import CONTRACT, Harness, wait_until  # noqa: E402
from fake_nav import FakeNav  # noqa: E402
from fleet_interfaces.msg import RobotState  # noqa: E402
from probe import Probe, lane_goal  # noqa: E402

SPEED = 0.5


def scenario_file(tmp_path):
    robot = {"speed_mps": SPEED, "pose_noise_m": 0.0}
    data = {
        "seed": 3, "speed_factor": 1.0, "timeout_s": 600,
        "stock": {"arm1": {"A": 3}, "arm2": {"C": 3}},
        "robots": {"pinky1": {"start": "P-3", **robot}, "pinky2": {"start": "P-5", **robot},
                   "pinky3": {"start": "P-4", **robot}},
        "arm": {"seconds_per_item": 0.2, "swap_seconds": 0.4, "fail_rate": 0.0},
        "dock": {"duration_s": 0.6, "error_cm": 1.0},
    }
    path = tmp_path / "scenario.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return str(path)


def fake_pinky(harness, tmp_path, heartbeat_timeout_s):
    from fake_robots.fake_pinky_node import FakePinkyNode
    harness.add(FakePinkyNode(parameter_overrides=harness.params(
        robot_id="pinky1", scenario_file=scenario_file(tmp_path), contract_file=str(CONTRACT),
        state_rate_hz=10.0, heartbeat_timeout_s=heartbeat_timeout_s)))


def real_fsm(harness, _tmp_path, heartbeat_timeout_s):
    from pinky_fsm.node import PinkyFsmNode
    harness.add(FakeNav(speed=SPEED))
    harness.add(PinkyFsmNode(parameter_overrides=harness.params(
        robot_id="pinky1", contract_file=str(CONTRACT),
        heartbeat_timeout_s=heartbeat_timeout_s)))


def run(rclpy_session, tmp_path, make, steps, heartbeat_timeout_s=0.0):
    harness = Harness(rclpy_session)
    try:
        make(harness, tmp_path, heartbeat_timeout_s)
        probe = harness.add(Probe())
        assert probe.ready()
        assert wait_until(lambda: (s := probe.last_state()) is not None
                          and s.state == RobotState.STATE_IDLE, 10.0)
        return steps(probe)
    finally:
        harness.close()


def outcome(probe, handle, t0):
    res = probe.result(handle).result
    time.sleep(0.3)  # 끝난 뒤의 상태도 본다
    return res.success, res.reason, sorted(set(probe.states_since(t0)))


def lane_steps(probe):
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


def heartbeat_steps(probe):
    t0 = time.monotonic()
    return {"lost": outcome(probe, probe.send(lane_goal(["P-3", "P-4", "P-5", "P-6"])), t0)}


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
