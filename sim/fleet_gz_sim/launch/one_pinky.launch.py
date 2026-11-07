"""Gazebo에서 핑키 한 대를 Nav2, pinky_fsm과 함께 돌린다. 관제(fleet_manager)는 띄우지 않는다.

    ros2 launch fleet_gz_sim one_pinky.launch.py                          # Gazebo 화면 + RViz
    ros2 launch fleet_gz_sim one_pinky.launch.py gui:=false rviz:=false   # 화면 없이

주행은 따로 `ros2 run fleet_gz_sim drive_check`(관제 흉내)로 시키거나 RViz의 2D Goal Pose
(bt_navigator, 한 번 계획)로 시킨다. 격리 검사(isolation.py)를 통과해야 무엇이든 띄운다.
월드 좌표 = 지도 좌표 = lane_map.yaml 좌표다(fleet_gz_sim.arena).
"""
import os
import tempfile
from pathlib import Path

from fleet_gz_sim.isolation import require_isolated
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from rclpy import get_rmw_implementation_identifier

ARGS = {
    "namespace": ("pinky1", "로봇 이름이자 namespace"),
    "x": ("0.35", "시작 위치(지도 좌표). 기본은 대기 줄 P-3"),
    "y": ("1.55", ""),
    "yaw": ("1.5708", "시작 방향(rad). 기본은 P-4 쪽"),
    "gui": ("true", "Gazebo 화면"),
    "rviz": ("true", "RViz"),
    "fsm": ("true", "pinky_fsm(move_to, dock, state)"),
    "heartbeat_timeout_s": ("1.5", "pinky_fsm heartbeat 끊김 판정. 0이면 서지 않는다"),
    "nav_log_level": ("warn", "Nav2 노드 로그 수준"),
}
TF_REMAPS = [("/tf", "tf"), ("/tf_static", "tf_static")]


def _bridge_args(ns: str) -> tuple[list[str], list[tuple[str, str]]]:
    """gz 토픽 /model/<ns>/... ↔ ROS 토픽 /<ns>/..."""
    pairs = [
        ("scan", "scan", "sensor_msgs/msg/LaserScan", "[", "gz.msgs.LaserScan"),
        ("odom", "odom", "nav_msgs/msg/Odometry", "[", "gz.msgs.Odometry"),
        ("tf", "tf", "tf2_msgs/msg/TFMessage", "[", "gz.msgs.Pose_V"),
        ("joint_states", "joint_states", "sensor_msgs/msg/JointState", "[", "gz.msgs.Model"),
        ("cmd_vel", "cmd_vel", "geometry_msgs/msg/Twist", "]", "gz.msgs.Twist"),
        ("truth", "sim/truth", "nav_msgs/msg/Odometry", "[", "gz.msgs.Odometry"),
    ]
    args = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
    remaps = []
    for gz_name, ros_name, ros_type, direction, gz_type in pairs:
        gz_topic = f"/model/{ns}/{gz_name}"
        args.append(f"{gz_topic}@{ros_type}{direction}{gz_type}")
        remaps.append((gz_topic, f"/{ns}/{ros_name}"))
    return args, remaps


def _rviz_config(share: Path, ns: str) -> str:
    text = (share / "rviz" / "pinky_nav.rviz").read_text(encoding="utf-8")
    out = Path(tempfile.gettempdir()) / f"pinky_nav_{ns}.rviz"
    out.write_text(text.replace("<robot_namespace>", f"/{ns}"), encoding="utf-8")
    return str(out)


def _start(context):
    # 노드는 launch의 작업 폴더를 물려받는다(gz.launch.py와 같은 검사)
    require_isolated(os.environ, rmw=get_rmw_implementation_identifier(), workdir=Path.cwd())
    from ament_index_python.packages import get_package_share_directory
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import Command
    from launch_ros.actions import Node
    from launch_ros.parameter_descriptions import ParameterValue

    def value(name):
        return LaunchConfiguration(name).perform(context)

    ns = value("namespace")
    on = {name: value(name).lower() in ("true", "1") for name in ("gui", "rviz", "fsm")}
    me = Path(get_package_share_directory("fleet_gz_sim"))
    nav = Path(get_package_share_directory("pinky_nav_config"))
    description = Path(get_package_share_directory("pinky_description"))
    world = me / "worlds" / "lane_arena.sdf"
    bridge_args, bridge_remaps = _bridge_args(ns)
    robot_description = ParameterValue(
        Command(["xacro ", str(me / "urdf" / "pinky_sim.urdf.xacro"), f" name:={ns}"]),
        value_type=str)
    server = ExecuteProcess(cmd=["gz", "sim", "-r", "-s", "-v", "2", str(world)],
                            name="gz_server", output="screen")
    actions = [
        # Gazebo 통신(gz-transport)도 이 컴퓨터 안에서만. 다른 시뮬과 섞이지 않게 칸막이를 나눈다
        SetEnvironmentVariable("GZ_IP", os.environ.get("GZ_IP", "127.0.0.1")),
        SetEnvironmentVariable("GZ_PARTITION", os.environ.get(
            "GZ_PARTITION", f"fleet_gz_{os.environ['ROS_DOMAIN_ID']}")),
        # 제조사 메시(package://pinky_description/...)를 Gazebo가 찾게 한다
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(description.parent)),
        server,
        RegisterEventHandler(OnProcessExit(target_action=server, on_exit=[Shutdown()])),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             namespace=ns, output="screen", remappings=TF_REMAPS,
             parameters=[{"robot_description": robot_description, "use_sim_time": True}]),
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-world", "lane_arena", "-name", ns, "-topic", f"/{ns}/robot_description",
                        "-x", value("x"), "-y", value("y"), "-z", "0.01", "-Y", value("yaw")]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name=f"{ns}_gz_bridge",
             output="screen", arguments=bridge_args, remappings=bridge_remaps,
             parameters=[{"use_sim_time": True}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav / "launch" / "nav2.launch.py")),
            launch_arguments={
                "namespace": ns, "use_sim_time": "true",
                "map": str(me / "maps" / "lane_arena.yaml"),
                "map_fine": str(me / "maps" / "lane_arena_fine.yaml"),
                "initial_x": value("x"), "initial_y": value("y"), "initial_yaw": value("yaw"),
                "log_level": value("nav_log_level"),
            }.items()),
    ]
    if on["gui"]:
        actions.append(ExecuteProcess(cmd=["gz", "sim", "-g", "-v", "2"], name="gz_gui",
                                      output="screen"))
    if on["fsm"]:
        fsm = Path(get_package_share_directory("pinky_fsm"))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(fsm / "launch" / "pinky_fsm.launch.py")),
            launch_arguments={"namespace": ns, "use_sim_time": "true",
                              "heartbeat_timeout_s": value("heartbeat_timeout_s")}.items()))
    if on["rviz"]:
        actions.append(Node(package="rviz2", executable="rviz2", namespace=ns, output="log",
                            arguments=["-d", _rviz_config(nav, ns)], remappings=TF_REMAPS,
                            parameters=[{"use_sim_time": True}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=text)
          for name, (default, text) in ARGS.items()),
        OpaqueFunction(function=_start),
    ])
