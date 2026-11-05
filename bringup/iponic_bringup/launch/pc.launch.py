"""Start the fleet manager on the PC, with any mix of fake and real devices.

All fake:  ros2 launch iponic_bringup pc.launch.py use_fake:=true scenario:=shared_lane_failure
Mixed:     ros2 launch iponic_bringup pc.launch.py use_fake:=true fake_robots:="pinky2 pinky3"
Real arms: ros2 launch iponic_bringup pc.launch.py use_fake:=true fake_arms:="arm3"
All real:  ros2 launch iponic_bringup pc.launch.py use_fake:=false

fake_arms: arm1 and arm2 together start the fake workcell server and their vision nodes (one
fill_basket server answers for both, so listing only one of them is an error). arm3 starts the
fake outbound arm.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    use_fake = get("use_fake").lower() == "true"
    use_sim_clock = use_fake and get("use_sim_clock").lower() == "true"

    actions = [
        Node(
            package="fleet_manager",
            executable="fms_node",
            name="fms_node",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_clock,
                "robots": get("robots").split(),
                "map_file": get("map_file"),
                "params_file": get("params_file"),
                "contract_file": get("contract_file"),
            }],
        ),
    ]
    if use_fake:
        share = FindPackageShare("fake_robots").perform(context)
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{share}/launch/fakes.launch.py"),
            launch_arguments={
                "scenario": get("scenario"),
                "fake_robots": get("fake_robots"),
                "fake_arms": get("fake_arms"),
                "fake_central": get("fake_central"),
                "use_sim_clock": str(use_sim_clock).lower(),
                "result_dir": get("result_dir"),
                "map_file": get("map_file"),
                "params_file": get("params_file"),
                "contract_file": get("contract_file"),
            }.items(),
        ))
    return actions


def generate_launch_description():
    defaults = {
        "use_fake": "true",
        "scenario": "basic_orders",
        "robots": "pinky1 pinky2 pinky3",
        "fake_robots": "pinky1 pinky2 pinky3",
        "fake_arms": "arm1 arm2 arm3",
        "fake_central": "true",
        "use_sim_clock": "true",
        "result_dir": "/tmp/iponic_result",
        "map_file": "",
        "params_file": "",
        "contract_file": "",
    }
    declared = [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
    return LaunchDescription([*declared, OpaqueFunction(function=_setup)])
