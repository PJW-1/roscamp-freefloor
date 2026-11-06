"""Helpers for the ROS tests. Importing this file never needs rclpy (pure tests run on Windows)."""
import os
import threading
import time
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).parents[3]
CONTRACT = REPO / "interfaces" / "fleet_interfaces" / "contract.yaml"
GRID_MAP = REPO / "pc" / "fleet_core" / "test" / "data" / "grid_cells.yaml"


def write_scenario(tmp_path, **over):
    data = {
        "seed": 3,
        "speed_factor": 1.0,
        "timeout_s": 600,
        "stock": {"arm1": {"A": 3, "B": 1}, "arm2": {"C": 3}},
        "robots": {
            "pinky1": {"start": "P-6", "speed_mps": 1.0, "pose_noise_m": 0.0},
            "pinky2": {"start": "P-5", "speed_mps": 1.0, "pose_noise_m": 0.0},
            "pinky3": {"start": "P-4", "speed_mps": 1.0, "pose_noise_m": 0.0},
        },
        "arm": {"seconds_per_item": 0.2, "swap_seconds": 0.4, "fail_rate": 0.0},
        "dock": {"duration_s": 0.6, "error_cm": 1.0},
    }
    data.update(over)
    if data.get("faults"):  # the loader refuses a fault scenario that expects nothing shipped
        data.setdefault("expect", {"orders_shipped_min": 1})
    path = tmp_path / "scenario.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return str(path)


def wait_until(predicate, timeout=10.0, step=0.02):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(step)
    return predicate()


@pytest.fixture(scope="session")
def rclpy_session():
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("fleet_interfaces.msg")
    # These nodes publish /pinky1/state and serve /pinky1/move_to like real robots would.
    if (os.environ.get("ROS_DOMAIN_ID", "") in ("", "6")
            or os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST"):
        pytest.skip("set ROS_DOMAIN_ID (not 6) and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST first")
    rclpy.init()
    yield rclpy
    rclpy.shutdown()


class Harness:
    """Runs nodes in one executor thread and destroys all of them afterwards."""

    def __init__(self, rclpy):
        from rclpy.executors import MultiThreadedExecutor
        self.rclpy = rclpy
        self.executor = MultiThreadedExecutor(num_threads=16)
        self.nodes = []
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()

    def add(self, node):
        self.nodes.append(node)
        self.executor.add_node(node)
        return node

    def params(self, **values):
        from rclpy.parameter import Parameter
        return [Parameter(k, value=v) for k, v in values.items()]

    def close(self):
        for node in self.nodes:
            getattr(node, "stop", lambda: None)()  # lets blocked action goals return
        self.executor.shutdown()
        self._thread.join(5.0)
        for node in self.nodes:
            node.destroy_node()


@pytest.fixture
def harness(rclpy_session):
    h = Harness(rclpy_session)
    yield h
    h.close()
