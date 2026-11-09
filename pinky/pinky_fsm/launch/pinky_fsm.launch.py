"""핑키 한 대의 FSM. 실물에서는 핑키 Pi에서, 시뮬에서는 fleet_gz_sim이 띄운다.

  ros2 launch pinky_fsm pinky_fsm.launch.py namespace:=pinky1

namespace가 robot_id가 된다. TF는 Nav2와 같이 namespace의 tf, tf_static을 쓴다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from pinky_fsm.core import FsmParams


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="pinky1"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("heartbeat_timeout_s", default_value="1.5",
                              description="0이면 heartbeat가 끊겨도 서지 않는다"
                                          "(관제 없이 시험할 때)"),
        DeclareLaunchArgument("through_controller_id", default_value="FollowPathThrough",
                              description="잠금 경계에서 끝나는 경로에 쓸 Nav2 controller"
                                          "(pinky_nav_config가 함께 띄운다)."
                                          " 비우면 모두 controller_id(접근 감속 그대로)"),
        DeclareLaunchArgument("horizon_cap_mps", default_value=str(FsmParams().horizon_cap_mps),
                              description="잠금 경계 속도 제한의 상한(m/s). RPP는 그 제한을"
                                          " 속도로 쓰므로 Nav2 desired_linear_vel 이하로"),
        Node(
            package="pinky_fsm", executable="pinky_fsm_node", name="pinky_fsm",
            namespace=namespace, output="screen",
            parameters=[{
                "robot_id": namespace,
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"),
                                               value_type=bool),
                "heartbeat_timeout_s": ParameterValue(
                    LaunchConfiguration("heartbeat_timeout_s"), value_type=float),
                "through_controller_id": ParameterValue(
                    LaunchConfiguration("through_controller_id"), value_type=str),
                "horizon_cap_mps": ParameterValue(
                    LaunchConfiguration("horizon_cap_mps"), value_type=float),
            }],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")]),
    ])
