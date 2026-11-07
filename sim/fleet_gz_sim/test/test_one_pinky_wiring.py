"""one_pinky.launch.py가 무엇을 띄우기 전에 격리를 검사하는지 본다."""
import importlib.util
from pathlib import Path

import pytest

launch = pytest.importorskip("launch")
pytest.importorskip("rclpy")

LAUNCH = Path(__file__).parents[1] / "launch" / "one_pinky.launch.py"
FAST_DDS = ("ROS_STATIC_PEERS", "RMW_IMPLEMENTATION", "ROS_DISCOVERY_SERVER",
            "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE",
            "FASTDDS_ENVIRONMENT_FILE", "FASTDDS_BUILTIN_TRANSPORTS")


@pytest.fixture
def one(monkeypatch, tmp_path):
    spec = importlib.util.spec_from_file_location("one_pinky_launch", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setenv("ROS_DOMAIN_ID", "44")
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    for name in FAST_DDS:
        monkeypatch.delenv(name, raising=False)
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(module, "get_rmw_implementation_identifier", lambda: "rmw_fastrtps_cpp")
    return module


def run(module):
    """ros2 launch처럼 인자 기본값을 넣고 OpaqueFunction을 실행한다."""
    context = launch.LaunchContext()
    *args, start = module.generate_launch_description().entities
    for action in args:
        action.visit(context)
    return start.execute(context)


def test_domain_6_starts_nothing(one, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "6")
    with pytest.raises(RuntimeError, match="ROS_DOMAIN_ID"):
        run(one)


def test_other_rmw_starts_nothing(one, monkeypatch):
    monkeypatch.setattr(one, "get_rmw_implementation_identifier", lambda: "rmw_cyclonedds_cpp")
    with pytest.raises(RuntimeError, match="rmw_cyclonedds_cpp"):
        run(one)


def test_isolated_launch_keeps_gazebo_transport_on_this_computer(one, monkeypatch):
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    for package in ("pinky_description", "pinky_nav_config", "pinky_fsm"):
        try:
            get_package_share_directory(package)
        except PackageNotFoundError:
            pytest.skip(f"{package} is not installed")
    monkeypatch.delenv("GZ_IP", raising=False)
    monkeypatch.delenv("GZ_PARTITION", raising=False)
    actions = run(one)
    names = {"".join(s.perform(None) for s in a.name)
             for a in actions if type(a).__name__ == "SetEnvironmentVariable"}
    assert {"GZ_IP", "GZ_PARTITION"} <= names
    kinds = [type(a).__name__ for a in actions]
    assert "ExecuteProcess" in kinds and "IncludeLaunchDescription" in kinds
