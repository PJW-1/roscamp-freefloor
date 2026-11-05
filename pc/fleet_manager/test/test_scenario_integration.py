"""CI item 5: each scenario runs to the end and the live graph matches contract.yaml."""
import json
import os
import signal
import subprocess
import sys
from pathlib import Path

import pytest

pytest.importorskip("rclpy")

from ament_index_python.packages import (  # noqa: E402
    PackageNotFoundError,
    get_package_share_directory,
)

SCENARIOS = ["basic_orders", "order_burst", "shared_lane_failure", "stock_shortage",
             "stuck_robot", "operator_recovery"]
WALL_TIMEOUT_S = 420
FIRST_DOMAIN = 50  # one domain per scenario, inside the range kept free of the robot network
CHECKER = Path(__file__).parents[3] / "scripts" / "check_interfaces.py"


@pytest.mark.parametrize("scenario", SCENARIOS)
def test_scenario_finishes_and_matches_the_contract(scenario, tmp_path):
    try:
        for package in ("iponic_bringup", "fake_robots"):
            get_package_share_directory(package)
    except PackageNotFoundError as exc:
        pytest.skip(f"not built in this workspace: {exc}")
    env = dict(os.environ, ROS_DOMAIN_ID=str(FIRST_DOMAIN + SCENARIOS.index(scenario)),
               ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
    log_path = tmp_path / "launch.log"
    with log_path.open("w", encoding="utf-8") as log:
        launch = subprocess.Popen(
            ["ros2", "launch", "iponic_bringup", "pc.launch.py", "use_fake:=true",
             f"scenario:={scenario}", f"result_dir:={tmp_path}"],
            env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
        )
        try:
            check = subprocess.run(
                [sys.executable, str(CHECKER), "--mode", "fake", "--wait-s", "15"],
                env=env, capture_output=True, text=True, timeout=120,
            )
            launch.wait(timeout=WALL_TIMEOUT_S)
        finally:
            if launch.poll() is None:
                os.killpg(launch.pid, signal.SIGINT)
                try:
                    launch.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid, signal.SIGKILL)

    tail = log_path.read_text(encoding="utf-8")[-3000:]
    assert check.returncode == 0, f"contract check failed:\n{check.stdout}\n{check.stderr}"
    summary_path = tmp_path / "summary.json"
    assert summary_path.exists(), f"no summary was written. launch log tail:\n{tail}"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert summary["pass"], f"{summary}\nlaunch log tail:\n{tail}"
    assert summary["orders_shipped"] >= 1, "a scenario must not pass by shipping nothing"
