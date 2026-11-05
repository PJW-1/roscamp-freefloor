"""scripts/check_interfaces.py against running fakes."""
import subprocess
import sys

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces.msg")

from conftest import CONTRACT, REPO, write_scenario  # noqa: E402

SCRIPT = REPO / "scripts" / "check_interfaces.py"
# Providers that are not fakes: they belong to the fleet manager.
SKIP = "assignments,zones,confirm_clear"


def start_all(harness, tmp_path):
    from fake_robots.fake_arm_node import FakeArmNode
    from fake_robots.fake_central_node import FakeCentralNode
    from fake_robots.fake_pinky_node import FakePinkyNode
    from fake_robots.fake_vision_node import FakeVisionNode
    from fake_robots.fake_workcell_node import FakeWorkcellNode
    scn = write_scenario(tmp_path)
    for r in ("pinky1", "pinky2", "pinky3"):
        harness.add(FakePinkyNode(parameter_overrides=harness.params(
            robot_id=r, scenario_file=scn)))
    for a in ("arm1", "arm2"):
        harness.add(FakeVisionNode(parameter_overrides=harness.params(
            arm_id=a, scenario_file=scn)))
    harness.add(FakeWorkcellNode(parameter_overrides=harness.params(scenario_file=scn)))
    harness.add(FakeArmNode(parameter_overrides=harness.params(
        arm_id="arm3", scenario_file=scn)))
    harness.add(FakeCentralNode(parameter_overrides=harness.params(scenario_file=scn)))


def check(*extra):
    return subprocess.run(
        [sys.executable, str(SCRIPT), "--contract", str(CONTRACT), "--mode", "fake",
         "--wait-s", "12", "--skip", SKIP, *extra],
        capture_output=True, text=True, timeout=60)


def test_fakes_satisfy_the_contract(harness, tmp_path):
    start_all(harness, tmp_path)
    out = check()
    assert out.returncode == 0, out.stdout + out.stderr


def test_wrong_qos_is_caught(harness, tmp_path):
    import rclpy.node
    from fleet_interfaces.msg import RobotState
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    start_all(harness, tmp_path)
    rogue = harness.add(rclpy.node.Node("rogue"))
    rogue.create_publisher(RobotState, "/pinky1/state", QoSProfile(
        depth=5, reliability=ReliabilityPolicy.RELIABLE))
    out = check()
    assert out.returncode == 1, out.stdout + out.stderr
    assert "missing" not in out.stdout
    assert "/pinky1/state" in out.stdout and "publisher reliability is reliable" in out.stdout


def test_missing_provider_is_reported(harness, tmp_path):
    out = check()
    assert out.returncode == 1 and "missing" in out.stdout
