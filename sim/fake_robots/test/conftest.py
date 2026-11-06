"""ROS 테스트 도우미. 이 파일을 가져와도 rclpy가 필요 없다(순수 테스트는 Windows에서 돈다)."""
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
    if data.get("faults"):  # 출고 0건을 기대하는 고장 시나리오는 로더가 거부한다
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
    # 이 노드들은 실제 로봇처럼 /pinky1/state를 발행하고 /pinky1/move_to를 받는다.
    if (os.environ.get("ROS_DOMAIN_ID", "") in ("", "6")
            or os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST"):
        pytest.skip("set ROS_DOMAIN_ID (not 6) and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST first")
    rclpy.init()
    yield rclpy
    rclpy.shutdown()


class Harness:
    """노드를 한 실행기 스레드에서 돌리고 끝나면 모두 없앤다."""

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
            getattr(node, "stop", lambda: None)()  # 막힌 액션 목표가 돌아오게 한다
        self.executor.shutdown()
        self._thread.join(5.0)
        for node in self.nodes:
            node.destroy_node()


@pytest.fixture
def harness(rclpy_session):
    h = Harness(rclpy_session)
    yield h
    h.close()
