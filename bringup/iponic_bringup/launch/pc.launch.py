"""PC에서 플릿 매니저를 띄운다. 가짜와 실물 장치를 섞어 쓸 수 있다.

All fake:  ros2 launch iponic_bringup pc.launch.py use_fake:=true scenario:=shared_lane_failure
Mixed:     ros2 launch iponic_bringup pc.launch.py use_fake:=true fake_robots:="pinky2 pinky3"
Real arms: ros2 launch iponic_bringup pc.launch.py use_fake:=true fake_arms:="arm3"
All real:  ros2 launch iponic_bringup pc.launch.py use_fake:=false

record:=PATH 는 tools/replay.py용 FMS 기록을 남긴다.

fake_arms: arm1과 arm2는 함께 가짜 워크셀 서버와 비전 노드를 띄운다.
fill_basket 서버 하나가 둘을 맡으므로 하나만 적으면 오류다. arm3는 가짜 출고 팔을 띄운다.
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
    record = get("record")

    actions = [
        Node(
            package="fleet_manager",
            executable="fleet_manager_node",
            name="fleet_manager",
            output="screen",
            arguments=["--record", record] if record else [],
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
        "record": "",
    }
    declared = [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
    return LaunchDescription([*declared, OpaqueFunction(function=_setup)])
