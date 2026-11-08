"""one_pinky.launch.py가 무엇을 띄우기 전에 격리를 검사하는지 본다."""
import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

launch = pytest.importorskip("launch")
pytest.importorskip("rclpy")

LAUNCH = Path(__file__).parents[1] / "launch" / "one_pinky.launch.py"
URDF = Path(__file__).parents[1] / "urdf" / "pinky_sim.urdf.xacro"
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


def need(*packages):
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    for package in packages:
        try:
            get_package_share_directory(package)
        except PackageNotFoundError:
            pytest.skip(f"{package} is not installed")


def environment(actions) -> dict:
    return {"".join(s.perform(None) for s in a.name): "".join(s.perform(None) for s in a.value)
            for a in actions if type(a).__name__ == "SetEnvironmentVariable"}


def test_isolated_launch_keeps_gazebo_transport_on_this_computer(one, monkeypatch):
    need("pinky_description", "pinky_nav_config", "pinky_fsm")
    monkeypatch.delenv("GZ_IP", raising=False)
    monkeypatch.delenv("GZ_PARTITION", raising=False)
    actions = run(one)
    env = environment(actions)
    assert env["GZ_IP"] == "127.0.0.1" and env["GZ_PARTITION"] == "fleet_gz_44"
    kinds = [type(a).__name__ for a in actions]
    assert "ExecuteProcess" in kinds and "IncludeLaunchDescription" in kinds


def test_gazebo_ip_from_the_shell_is_not_used(one, monkeypatch):
    """셸에 LAN 주소가 GZ_IP로 남아 있어도 Gazebo는 이 컴퓨터 안에서만 통신한다(Codex 리뷰)."""
    need("pinky_description", "pinky_nav_config", "pinky_fsm")
    monkeypatch.setenv("GZ_IP", "192.168.0.10")
    assert environment(run(one))["GZ_IP"] == "127.0.0.1"


def test_one_bridge_carries_the_clock_and_the_robot(one, monkeypatch):
    """gz.launch.py와 같은 조각을 쓴다. 한 대일 때도 브리지 프로세스는 하나다."""
    need("pinky_description", "pinky_nav_config", "pinky_fsm")
    from fleet_gz_sim import launch_parts
    made = []
    monkeypatch.setattr(launch_parts, "Node", lambda *a, **kw: made.append(kw) or kw)
    run(one)
    [bridge] = [kw for kw in made if kw["executable"] == "parameter_bridge"]
    topics = [a.split("@")[0] for a in bridge["arguments"]]
    assert topics.count("/clock") == 1 and "/model/pinky1/cmd_vel" in topics


def test_joint_states_cover_every_moving_joint():
    """빠진 관절은 TF가 없어 RViz 로봇 모델이 오류가 된다(캐스터 두 개가 빠졌었다)."""
    xacro = pytest.importorskip("xacro")
    need("pinky_description")
    root = ET.fromstring(xacro.process_file(str(URDF), mappings={"name": "pinky1"}).toxml())
    moving = {j.get("name") for j in root.iter("joint") if j.get("type") != "fixed"}
    plugin = next(p for p in root.iter("plugin")
                  if p.get("name") == "gz::sim::systems::JointStatePublisher")
    listed = {e.text for e in plugin.iter("joint_name")}
    assert moving and (not listed or moving <= listed)
