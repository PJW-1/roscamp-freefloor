"""one_pinky.launch.py와 gz.launch.py가 같이 쓰는 launch 조각: Gazebo 월드 하나, 브리지 하나,
핑키 한 대씩.

월드 좌표 = 지도 좌표 = lane_map.yaml 좌표다(fleet_gz_sim.arena).
ROS 프로세스 하나가 DDS 참여자 하나다. LOCALHOST 탐색에서는 32개 정도만 서로 보이므로
브리지는 모든 로봇을 하나로 맡고, 3대일 때는 Nav2도 로봇마다 프로세스 하나로 묶는다.
"""
from __future__ import annotations

import os
import tempfile
from pathlib import Path

from launch.actions import (
    AppendEnvironmentVariable,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue

WORLD = "lane_arena"
TF_REMAPS = [("/tf", "tf"), ("/tf_static", "tf_static")]


def share(package: str) -> Path:
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(package))


def bridge_args(ns: str) -> tuple[list[str], list[tuple[str, str]]]:
    """gz 토픽 /model/<ns>/... ↔ ROS 토픽 /<ns>/... /clock은 bridge()가 한 번만 넣는다."""
    pairs = [
        ("scan", "scan", "sensor_msgs/msg/LaserScan", "[", "gz.msgs.LaserScan"),
        ("odom", "odom", "nav_msgs/msg/Odometry", "[", "gz.msgs.Odometry"),
        ("tf", "tf", "tf2_msgs/msg/TFMessage", "[", "gz.msgs.Pose_V"),
        ("joint_states", "joint_states", "sensor_msgs/msg/JointState", "[", "gz.msgs.Model"),
        ("cmd_vel", "cmd_vel", "geometry_msgs/msg/Twist", "]", "gz.msgs.Twist"),
        ("truth", "sim/truth", "nav_msgs/msg/Odometry", "[", "gz.msgs.Odometry"),
    ]
    args, remaps = [], []
    for gz_name, ros_name, ros_type, direction, gz_type in pairs:
        gz_topic = f"/model/{ns}/{gz_name}"
        args.append(f"{gz_topic}@{ros_type}{direction}{gz_type}")
        remaps.append((gz_topic, f"/{ns}/{ros_name}"))
    return args, remaps


def world(gui: bool) -> list:
    """Gazebo 서버(끝나면 launch도 끝)와 화면. 브리지는 bridge()가 따로 만든다."""
    server = ExecuteProcess(cmd=["gz", "sim", "-r", "-s", "-v", "2",
                                 str(share("fleet_gz_sim") / "worlds" / f"{WORLD}.sdf")],
                            name="gz_server", output="screen")
    actions = [
        # Gazebo 통신(gz-transport)도 이 컴퓨터 안에서만. 셸에 남은 GZ_IP(LAN 주소일 수 있다)는
        # 쓰지 않는다. 다른 시뮬과 섞이지 않게 칸막이를 나눈다
        SetEnvironmentVariable("GZ_IP", "127.0.0.1"),
        SetEnvironmentVariable("GZ_PARTITION", os.environ.get(
            "GZ_PARTITION", f"fleet_gz_{os.environ['ROS_DOMAIN_ID']}")),
        # 제조사 메시(package://pinky_description/...)를 Gazebo가 찾게 한다
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(share("pinky_description").parent)),
        server,
        RegisterEventHandler(OnProcessExit(target_action=server, on_exit=[Shutdown()])),
    ]
    if gui:
        actions.append(ExecuteProcess(cmd=["gz", "sim", "-g", "-v", "2"], name="gz_gui",
                                      output="screen"))
    return actions


def bridge(robots) -> Node:
    """Gazebo 시계와 모든 로봇의 토픽을 잇는 브리지 프로세스 하나.

    /clock을 여럿이 내면 표본 순서가 섞여 시간이 뒤로 갈 수 있으므로 한 번만 넣는다.
    토픽 이름이 모두 절대 이름이라 namespace는 필요 없다."""
    args, remaps = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], []
    for ns in robots:
        robot_args, robot_remaps = bridge_args(ns)
        args += robot_args
        remaps += robot_remaps
    return Node(package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge",
                output="screen", arguments=args, remappings=remaps,
                parameters=[{"use_sim_time": True}])


def _rviz_config(ns: str) -> str:
    text = (share("pinky_nav_config") / "rviz" / "pinky_nav.rviz").read_text(encoding="utf-8")
    out = Path(tempfile.gettempdir()) / f"pinky_nav_{ns}.rviz"
    out.write_text(text.replace("<robot_namespace>", f"/{ns}"), encoding="utf-8")
    return str(out)


def pinky(ns: str, x, y, yaw, *, heartbeat_timeout_s="1.5", nav_log_level="warn", fsm=True,
          rviz=False, dock_aside=False, nav_composition=False) -> list:
    """핑키 한 대: 모델, Nav2, pinky_fsm, RViz. 브리지는 bridge()에 이 로봇 이름을 넣는다.

    dock_aside: pinky_fsm의 dock 서버를 /<ns>/dock_fsm으로 비켜 둔다(시뮬 전용 도크 대역용).
    nav_composition: Nav2 노드 9개를 프로세스 하나(component container)로 띄운다."""
    me, nav = share("fleet_gz_sim"), share("pinky_nav_config")
    x, y, yaw = str(x), str(y), str(yaw)
    description = ParameterValue(
        Command(["xacro ", str(me / "urdf" / "pinky_sim.urdf.xacro"), f" name:={ns}"]),
        value_type=str)
    actions = [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             namespace=ns, output="screen", remappings=TF_REMAPS,
             parameters=[{"robot_description": description, "use_sim_time": True}]),
        Node(package="ros_gz_sim", executable="create", namespace=ns, output="screen",
             arguments=["-world", WORLD, "-name", ns, "-topic", f"/{ns}/robot_description",
                        "-x", x, "-y", y, "-z", "0.01", "-Y", yaw]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav / "launch" / "nav2.launch.py")),
            launch_arguments={
                "namespace": ns, "use_sim_time": "true",
                "map": str(me / "maps" / "lane_arena.yaml"),
                "map_fine": str(me / "maps" / "lane_arena_fine.yaml"),
                "initial_x": x, "initial_y": y, "initial_yaw": yaw,
                "log_level": nav_log_level,
                "use_composition": "true" if nav_composition else "false",
            }.items()),
    ]
    if fsm:
        include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share("pinky_fsm") / "launch" /
                                              "pinky_fsm.launch.py")),
            launch_arguments={"namespace": ns, "use_sim_time": "true",
                              "heartbeat_timeout_s": str(heartbeat_timeout_s)}.items())
        actions.append(GroupAction([SetRemap(f"/{ns}/dock", f"/{ns}/dock_fsm"), include])
                       if dock_aside else include)
    if rviz:
        actions.append(Node(package="rviz2", executable="rviz2", namespace=ns, output="log",
                            arguments=["-d", _rviz_config(ns)], remappings=TF_REMAPS,
                            parameters=[{"use_sim_time": True}]))
    return actions
