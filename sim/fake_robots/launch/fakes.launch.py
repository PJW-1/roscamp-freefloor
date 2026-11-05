"""Start fake robots, workcell, arm, vision, central stand-in, sim clock and scenario monitor.

  ros2 launch fake_robots fakes.launch.py scenario:=basic_orders result_dir:=/tmp/run1
  ros2 launch fake_robots fakes.launch.py fake_robots:="pinky2 pinky3"   # pinky1 is real
  ros2 launch fake_robots fakes.launch.py fake_arms:="arm3"              # real workcell

fake_arms: arm1 and arm2 together start the fake workcell and their two vision nodes, because
one fill_basket server answers for both. arm3 starts the fake outbound arm.
"""
import os

from ament_index_python.packages import get_package_share_directory
from fake_robots.common import split_fake_arms
from fleet_core.site_map import load_site_map
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ALL_ROBOTS = "pinky1 pinky2 pinky3"
ALL_ARMS = "arm1 arm2 arm3"
CLOCK_START_DELAY_S = 5.0  # wall time for every node to start and find the others
CLOCK_RATE_HZ = 50.0  # every node wakes up on every tick; 0.1 s of sim time at speed factor 5


def _truthy(text: str) -> bool:
    return text.strip().lower() in ("1", "true", "yes", "on")


def _scenario_path(value: str) -> str:
    if os.path.isfile(value):
        return os.path.abspath(value)
    share = get_package_share_directory("fake_robots")
    path = os.path.join(share, "scenarios", value if value.endswith(".yaml") else value + ".yaml")
    if not os.path.isfile(path):
        raise RuntimeError(f"scenario {value!r} is neither a file nor in {share}/scenarios")
    return path


def _setup(context):
    arg = {name: LaunchConfiguration(name).perform(context) for name in (
        "scenario", "fake_robots", "fake_arms", "fake_central", "use_sim_clock", "result_dir",
        "monitor", "map_file", "contract_file", "params_file")}
    scenario = _scenario_path(arg["scenario"])
    sim_clock = _truthy(arg["use_sim_clock"])
    common = {"use_sim_time": sim_clock, "scenario_file": scenario}
    if arg["map_file"]:
        common["map_file"] = arg["map_file"]
    if arg["contract_file"]:
        common["contract_file"] = arg["contract_file"]

    def node(executable, name, **extra):
        return Node(package="fake_robots", executable=executable, name=name, output="screen",
                    parameters=[{**common, **extra}])

    actions = []
    if sim_clock:
        import yaml
        with open(scenario, encoding="utf-8") as f:
            factor = float(yaml.safe_load(f).get("speed_factor", 1.0))
        actions.append(Node(package="fake_robots", executable="sim_clock_node",
                            name="sim_clock", output="screen",
                            parameters=[{"speed_factor": factor,
                                         "start_delay_s": CLOCK_START_DELAY_S,
                                         "rate_hz": CLOCK_RATE_HZ}]))
    for robot in arg["fake_robots"].split():
        actions.append(node("fake_pinky_node", f"fake_{robot}", robot_id=robot,
                            params_file=arg["params_file"]))
    workcell, vision_arms, swap_arms = split_fake_arms(
        arg["fake_arms"], load_site_map(arg["map_file"] or None))
    if workcell:
        actions.append(node("fake_workcell_node", "fake_workcell"))
    for arm in vision_arms:
        actions.append(node("fake_vision_node", f"fake_vision_{arm}", arm_id=arm))
    for arm in swap_arms:
        actions.append(node("fake_arm_node", f"fake_{arm}", arm_id=arm))
    if _truthy(arg["fake_central"]):
        actions.append(node("fake_central_node", "fake_central", params_file=arg["params_file"]))
    if _truthy(arg["monitor"]):
        monitor = Node(package="fake_robots", executable="scenario_monitor_node",
                       name="scenario_monitor", output="screen",
                       parameters=[{**common, "result_dir": arg["result_dir"],
                                    "params_file": arg["params_file"]}],
                       on_exit=Shutdown())
        actions.append(monitor)
    return actions


def generate_launch_description():
    def declare(name, default, doc):
        return DeclareLaunchArgument(name, default_value=default, description=doc)

    return LaunchDescription([
        declare("scenario", "basic_orders", "scenario name or path to a scenario YAML"),
        declare("fake_robots", ALL_ROBOTS, "space separated robots to fake (others are real)"),
        declare("fake_arms", ALL_ARMS, "arm1 and arm2 together (workcell, vision), arm3"),
        declare("fake_central", "true", "run the central stand-in"),
        declare("use_sim_clock", "true", "run the sim clock and use sim time everywhere"),
        declare("result_dir", "sim_result", "where the monitor writes summary.json and CSVs"),
        declare("monitor", "true", "run the scenario monitor, which shuts the launch down"),
        declare("map_file", "", "site map YAML (default: the one in fleet_core)"),
        declare("contract_file", "", "contract YAML (default: the installed one)"),
        declare("params_file", "", "fleet parameters YAML, for basket capacity and release limits"),
        OpaqueFunction(function=_setup),
    ])
