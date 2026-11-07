"""ROS 시험 도우미. 이 파일을 가져와도 rclpy가 필요 없다(순수 시험은 ROS 없이 돈다)."""
import os
import threading
import time
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
CONTRACT = REPO / "interfaces" / "fleet_interfaces" / "contract.yaml"


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
    pytest.importorskip("nav2_msgs.action")
    # 이 노드는 실제 로봇처럼 /pinky1/state를 발행하고 /pinky1/move_to를 받는다.
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
            getattr(node, "stop", lambda: None)()
        self.executor.shutdown()
        self._thread.join(5.0)
        for node in self.nodes:
            node.destroy_node()


@pytest.fixture
def harness(rclpy_session):
    h = Harness(rclpy_session)
    yield h
    h.close()
