"""gz.launch.py가 지금 환경, 실제 RMW, 작업 폴더를 격리 검사에 넘기는지 본다."""
import importlib.util
from pathlib import Path

import pytest

launch = pytest.importorskip("launch")
rclpy = pytest.importorskip("rclpy")

LAUNCH = Path(__file__).parents[1] / "launch" / "gz.launch.py"
FAST_DDS = ("ROS_STATIC_PEERS", "RMW_IMPLEMENTATION", "ROS_DISCOVERY_SERVER",
            "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE",
            "FASTDDS_ENVIRONMENT_FILE", "FASTDDS_BUILTIN_TRANSPORTS")


@pytest.fixture
def gz(monkeypatch, tmp_path):
    spec = importlib.util.spec_from_file_location("gz_launch", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setenv("ROS_DOMAIN_ID", "44")
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    for name in FAST_DDS:
        monkeypatch.delenv(name, raising=False)
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(module, "get_rmw_implementation_identifier", lambda: "rmw_fastrtps_cpp")
    return module


def test_the_rmw_comes_from_rclpy():
    spec = importlib.util.spec_from_file_location("gz_launch_plain", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.get_rmw_implementation_identifier is rclpy.get_rmw_implementation_identifier


def run(gz):
    """ros2 launch처럼 launch description의 동작을 실행한다."""
    [action] = gz.generate_launch_description().entities
    return action.execute(launch.LaunchContext())


def test_isolated_launch_only_logs(gz):
    actions = run(gz)
    assert len(actions) == 1 and type(actions[0]).__name__ == "LogInfo"


def test_launch_reads_the_current_environment(gz, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "6")
    with pytest.raises(RuntimeError, match="ROS_DOMAIN_ID"):
        run(gz)


def test_launch_checks_the_loaded_rmw(gz, monkeypatch):
    monkeypatch.setattr(gz, "get_rmw_implementation_identifier", lambda: "rmw_cyclonedds_cpp")
    with pytest.raises(RuntimeError, match="rmw_cyclonedds_cpp"):
        run(gz)


def test_launch_checks_the_working_directory(gz, tmp_path):
    (tmp_path / "DEFAULT_FASTRTPS_PROFILES.xml").write_text("<dds/>")
    with pytest.raises(RuntimeError, match="DEFAULT_FASTRTPS_PROFILES.xml"):
        run(gz)
