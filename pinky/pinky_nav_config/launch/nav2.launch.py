"""핑키 한 대의 Nav2를 로봇 namespace 안에 띄운다.

    ros2 launch pinky_nav_config nav2.launch.py map:=<지도.yaml> map_fine:=<1cm 지도.yaml> \
        initial_x:=0.35 initial_y:=1.55 initial_yaw:=1.5708

- 노드: map_server(5 cm, map), map_server_fine(1 cm, map_fine), amcl, controller_server,
  planner_server, behavior_server, bt_navigator와 lifecycle manager 둘.
- 모두 /{namespace} 안에서 돌고 TF는 /{namespace}/tf, /{namespace}/tf_static을 쓴다(R17).
  프레임 이름(map, odom, base_footprint)에는 접두사를 붙이지 않는다.
- controller_server는 cmd_vel로 바로 낸다(velocity_smoother, collision_monitor 없음).
- AMCL은 1 cm 지도를, costmap은 5 cm 지도를 쓴다(R15).
- bt_navigator는 계약의 navigate_to_pose를 위해 띄운다. 복구 없이 한 번 계획하는 BT(R14).
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.descriptions import ParameterFile, ParameterValue
from nav2_common.launch import ReplaceString, RewrittenYaml

SHARE = Path(get_package_share_directory("pinky_nav_config"))
LOCALIZATION = ["map_server", "map_server_fine", "amcl"]
NAVIGATION = ["controller_server", "planner_server", "behavior_server", "bt_navigator"]
TF_REMAPS = [("/tf", "tf"), ("/tf_static", "tf_static")]


def generate_launch_description():
    arg = LaunchConfiguration
    namespace = arg("namespace")
    # costmap 층의 절대 토픽 이름(<robot_namespace>/scan)을 채운 뒤
    # 노드 이름 키를 namespace 아래에 넣는다
    params = ParameterFile(RewrittenYaml(
        source_file=ReplaceString(source_file=arg("params_file"),
                                  replacements={"<robot_namespace>": ("/", namespace)}),
        root_key=namespace, param_rewrites={}, convert_types=True), allow_substs=True)
    log = ["--ros-args", "--log-level", arg("log_level")]

    def nav_node(package, executable, name, *extra):
        return Node(package=package, executable=executable, name=name, output="screen",
                    parameters=[params, *extra], remappings=TF_REMAPS, arguments=log)

    def manager(name, nodes):
        return Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name=name,
                    output="screen", arguments=log,
                    parameters=[{"autostart": ParameterValue(arg("autostart"), value_type=bool),
                                 "node_names": nodes}])

    initial_pose = {
        "set_initial_pose": True,
        "initial_pose.x": ParameterValue(arg("initial_x"), value_type=float),
        "initial_pose.y": ParameterValue(arg("initial_y"), value_type=float),
        "initial_pose.yaw": ParameterValue(arg("initial_yaw"), value_type=float),
    }
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="pinky1"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map", description="5 cm 지도 yaml(costmap)"),
        DeclareLaunchArgument("map_fine", default_value=arg("map"),
                              description="1 cm 지도 yaml(AMCL, R15). 없으면 map"),
        DeclareLaunchArgument("params_file",
                              default_value=str(SHARE / "params" / "nav2_params.yaml")),
        DeclareLaunchArgument("bt_xml",
                              default_value=str(SHARE / "bt" / "plan_once_no_recovery.xml")),
        DeclareLaunchArgument("initial_x", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="info"),
        GroupAction([
            PushRosNamespace(namespace),
            SetParameter("use_sim_time", arg("use_sim_time")),
            nav_node("nav2_map_server", "map_server", "map_server",
                     {"yaml_filename": arg("map")}),
            nav_node("nav2_map_server", "map_server", "map_server_fine",
                     {"yaml_filename": arg("map_fine")}),
            nav_node("nav2_amcl", "amcl", "amcl", initial_pose),
            nav_node("nav2_controller", "controller_server", "controller_server"),
            nav_node("nav2_planner", "planner_server", "planner_server"),
            nav_node("nav2_behaviors", "behavior_server", "behavior_server"),
            nav_node("nav2_bt_navigator", "bt_navigator", "bt_navigator",
                     {"default_nav_to_pose_bt_xml": arg("bt_xml")}),
            manager("lifecycle_manager_localization", LOCALIZATION),
            manager("lifecycle_manager_navigation", NAVIGATION),
        ]),
    ])
