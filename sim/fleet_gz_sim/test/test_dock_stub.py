"""sim_dock_stub: off by default, only on Gazebo time in an isolated sim, never from the real
bringup."""
from pathlib import Path

import pytest
from fleet_gz_sim.dock_stand_in import dock_stub_problems, require_dock_stub_allowed

REPO = Path(__file__).resolve().parents[3]
ISOLATED = {"ROS_DOMAIN_ID": "97", "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp"}
ROBOT_NETWORK = {"ROS_DOMAIN_ID": "6", "ROS_AUTOMATIC_DISCOVERY_RANGE": "SUBNET"}


def test_allowed_only_on_sim_time_in_an_isolated_sim(tmp_path):
    assert dock_stub_problems(True, ISOLATED, workdir=tmp_path) == []
    assert any("use_sim_time" in p for p in dock_stub_problems(False, ISOLATED))


def test_the_robot_network_blocks_the_stub_even_on_sim_time():
    found = dock_stub_problems(True, ROBOT_NETWORK)
    assert any("ROS_DOMAIN_ID" in p for p in found)
    assert any("LOCALHOST" in p for p in found)
    with pytest.raises(RuntimeError, match="sim_dock_stub"):
        require_dock_stub_allowed(True, ROBOT_NETWORK)


def test_a_wall_clock_run_blocks_the_stub_even_when_isolated():
    with pytest.raises(RuntimeError, match="use_sim_time"):
        require_dock_stub_allowed(False, ISOLATED)


def test_the_loaded_rmw_and_fast_dds_files_count(tmp_path):
    with pytest.raises(RuntimeError, match="rmw_cyclonedds_cpp"):
        require_dock_stub_allowed(True, ISOLATED, rmw="rmw_cyclonedds_cpp")
    (tmp_path / "DEFAULT_FASTRTPS_PROFILES.xml").write_text("<dds/>")
    with pytest.raises(RuntimeError, match="DEFAULT_FASTRTPS_PROFILES.xml"):
        require_dock_stub_allowed(True, ISOLATED, workdir=tmp_path)


# -- the real bringup cannot reach the stub -----------------------------------------------------

# A quoted package name is a use of the package (Node package=, share lookups); prose may
# mention it.
STUB_WORDS = ("sim_dock_stub", "dock_stub", "sim_stand_in", "\"fleet_gz_sim\"", "'fleet_gz_sim'")


def real_launch_files() -> list[Path]:
    """Launch files a real run starts from: the PC bringup, the fakes it includes, the pinky."""
    found = sorted((REPO / "bringup").rglob("*.launch.py"))
    found += sorted((REPO / "sim" / "fake_robots" / "launch").glob("*.launch.py"))
    found += sorted((REPO / "pinky").glob("*/launch/*.launch.py"))
    return found


def test_no_real_launch_file_names_the_stub():
    files = real_launch_files()
    assert any(f.name == "pc.launch.py" for f in files)
    assert any(f.name == "pinky_fsm.launch.py" for f in files)
    for path in files:
        text = path.read_text(encoding="utf-8")
        for word in STUB_WORDS:
            assert word not in text, f"{path.relative_to(REPO)} mentions {word}"


def test_the_bringup_package_does_not_depend_on_the_sim():
    text = (REPO / "bringup" / "iponic_bringup" / "package.xml").read_text(encoding="utf-8")
    assert "fleet_gz_sim" not in text


def test_passing_sim_dock_stub_to_the_pc_bringup_starts_nothing_from_the_sim():
    """ros2 launch accepts undeclared arguments silently. Make sure this one changes nothing."""
    launch = pytest.importorskip("launch")
    import importlib.util

    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    try:
        get_package_share_directory("fake_robots")
    except PackageNotFoundError:
        pytest.skip("fake_robots is not installed")
    path = REPO / "bringup" / "iponic_bringup" / "launch" / "pc.launch.py"
    spec = importlib.util.spec_from_file_location("pc_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    def made(**overrides):
        context = launch.LaunchContext()
        *args, setup = module.generate_launch_description().entities
        for action in args:
            action.visit(context)
        context.launch_configurations.update(overrides)
        return setup.execute(context)

    def describe(actions) -> str:
        return repr([(type(a).__name__, vars(a)) for a in actions])

    assert "sim_dock_stub" not in {a.name for a in module.generate_launch_description().entities
                                   if type(a).__name__ == "DeclareLaunchArgument"}
    plain = made()
    stubbed = made(sim_dock_stub="true")
    assert len(stubbed) == len(plain)
    for word in STUB_WORDS:
        assert word not in describe(stubbed)


# -- the node refuses on its own, whatever launched it -------------------------------------------

@pytest.fixture
def ros(monkeypatch, tmp_path):
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("fake_robots")
    for name, value in ISOLATED.items():
        monkeypatch.setenv(name, value)
    for name in ("ROS_STATIC_PEERS", "ROS_DISCOVERY_SERVER", "FASTRTPS_DEFAULT_PROFILES_FILE",
                 "FASTDDS_DEFAULT_PROFILES_FILE", "FASTDDS_ENVIRONMENT_FILE",
                 "FASTDDS_BUILTIN_TRANSPORTS"):
        monkeypatch.delenv(name, raising=False)
    monkeypatch.chdir(tmp_path)
    if rclpy.get_rmw_implementation_identifier() != "rmw_fastrtps_cpp":
        pytest.skip("needs rmw_fastrtps_cpp")
    rclpy.init()
    yield rclpy
    rclpy.try_shutdown()


def stand_in(rclpy, **params):
    from fleet_gz_sim.stand_in_node import SimStandIn
    from rclpy.parameter import Parameter
    return SimStandIn(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])


def test_the_stand_in_serves_no_dock_unless_asked(ros):
    node = stand_in(ros, use_sim_time=True)
    try:
        assert node.dock_servers == []
    finally:
        node.destroy_node()


def test_the_stand_in_refuses_the_stub_on_the_wall_clock(ros):
    with pytest.raises(RuntimeError, match="use_sim_time"):
        stand_in(ros, dock_stub=True)


def test_the_stand_in_refuses_the_stub_outside_the_sim_domains(ros, monkeypatch):
    # The context was opened on an isolated domain; only the check sees this value.
    monkeypatch.setenv("ROS_DOMAIN_ID", "42")
    with pytest.raises(RuntimeError, match="ROS_DOMAIN_ID"):
        stand_in(ros, dock_stub=True, use_sim_time=True)


def test_the_stub_answers_only_while_gazebo_time_runs():
    """No /clock (Gazebo not running): a dock goal fails instead of succeeding."""
    from fleet_gz_sim.dock_stand_in import stub_may_answer
    assert stub_may_answer(ros_time_active=True, now_s=12.0)
    assert not stub_may_answer(ros_time_active=False, now_s=12.0)
    assert not stub_may_answer(ros_time_active=True, now_s=0.0)


def test_a_stalled_clock_counts_as_stopped():
    """/clock that ran once and then stopped keeps its last value; the stub must notice."""
    from fleet_gz_sim.dock_stand_in import ClockWatch
    watch = ClockWatch(limit_s=5.0)
    assert not watch.stalled(sim_now=10.0, wall_now=100.0)
    assert not watch.stalled(sim_now=10.5, wall_now=104.0)
    assert not watch.stalled(sim_now=10.5, wall_now=108.9)
    assert watch.stalled(sim_now=10.5, wall_now=109.1)
    assert not watch.stalled(sim_now=10.6, wall_now=109.2)  # runs again


def test_a_dock_ends_only_after_gazebo_time_moved():
    """duration_s 0 must not succeed on a clock that stopped before the goal came (Codex 19bd994).

    The loop then keeps waiting until the clock moves or ClockWatch calls it stalled."""
    from fleet_gz_sim.dock_stand_in import dock_done
    assert not dock_done(elapsed=0.0, duration=0.0)
    assert dock_done(elapsed=0.004, duration=0.0)
    assert not dock_done(elapsed=1.9, duration=2.0)
    assert dock_done(elapsed=2.0, duration=2.0)


def test_the_stub_keeps_gazebo_time_once_started(ros):
    """Switching use_sim_time off later would put the stub on the wall clock."""
    from ament_index_python.packages import get_package_share_directory
    from rclpy.parameter import Parameter
    scenario = f"{get_package_share_directory('fake_robots')}/scenarios/basic_orders.yaml"
    node = stand_in(ros, dock_stub=True, use_sim_time=True, scenario_file=scenario)
    try:
        [result] = node.set_parameters([Parameter("use_sim_time", value=False)])
        assert not result.successful
        assert node.get_parameter("use_sim_time").value is True
    finally:
        node.destroy_node()


def test_the_stand_in_serves_every_dock_when_allowed(ros):
    from ament_index_python.packages import get_package_share_directory
    scenario = f"{get_package_share_directory('fake_robots')}/scenarios/basic_orders.yaml"
    node = stand_in(ros, dock_stub=True, use_sim_time=True, scenario_file=scenario)
    try:
        assert len(node.dock_servers) == 3
    finally:
        node.destroy_node()


class _Handle:
    """Just enough of a dock goal handle for SimStandIn._dock."""

    def __init__(self, dock_id):
        from types import SimpleNamespace
        self.request = SimpleNamespace(dock_id=dock_id)
        self.is_cancel_requested = False
        self.ended = ""

    def abort(self):
        self.ended = "aborted"

    def succeed(self):
        self.ended = "succeeded"

    def canceled(self):
        self.ended = "canceled"

    def publish_feedback(self, feedback):
        pass


def _docking_stub(ros, monkeypatch, sim_now):
    """A stub with duration_s 0, the robot on the WORK dock and Gazebo time given by sim_now."""
    from ament_index_python.packages import get_package_share_directory
    from fleet_gz_sim import stand_in_node
    scenario = f"{get_package_share_directory('fake_robots')}/scenarios/basic_orders.yaml"
    node = stand_in(ros, dock_stub=True, use_sim_time=True, scenario_file=scenario)
    node._duration = 0.0
    monkeypatch.setattr(stand_in_node, "CLOCK_STALL_S", 0.1)
    monkeypatch.setattr(stand_in_node, "stub_may_answer", lambda active, now: True)
    monkeypatch.setattr(node, "_now", sim_now)
    x, y = node._site.waypoint_pose(node._site.stations["WORK"].dock)[:2]
    node._pose["pinky1"] = (x, y, 0.0)
    return node


def test_the_dock_loop_fails_a_goal_on_a_clock_that_stopped_before_it(ros, monkeypatch):
    """The _dock loop itself, not only dock_done (Codex review of c98c086)."""
    node = _docking_stub(ros, monkeypatch, lambda: 12.0)
    try:
        handle = _Handle("WORK")
        assert not node._dock("pinky1", handle).success
        assert handle.ended == "aborted"
    finally:
        node.destroy_node()


def test_the_dock_loop_succeeds_once_the_clock_moves(ros, monkeypatch):
    ticks = iter(12.0 + 0.01 * i for i in range(10_000))
    node = _docking_stub(ros, monkeypatch, lambda: next(ticks))
    try:
        handle = _Handle("WORK")
        assert node._dock("pinky1", handle).success
        assert handle.ended == "succeeded"
    finally:
        node.destroy_node()
