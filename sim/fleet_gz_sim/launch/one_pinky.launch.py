"""Gazebo에서 핑키 한 대를 Nav2, pinky_fsm과 함께 돌린다. 관제(fleet_manager)는 띄우지 않는다.

    ros2 launch fleet_gz_sim one_pinky.launch.py                          # Gazebo 화면 + RViz
    ros2 launch fleet_gz_sim one_pinky.launch.py gui:=false rviz:=false   # 화면 없이

주행은 따로 `ros2 run fleet_gz_sim drive_check`(관제 흉내)로 시키거나 RViz의 2D Goal Pose
(bt_navigator, 한 번 계획)로 시킨다. 격리 검사(isolation.py)를 통과해야 무엇이든 띄운다.
월드와 핑키는 gz.launch.py와 같은 조각(fleet_gz_sim.launch_parts)으로 만든다.
"""
import os
from pathlib import Path

from fleet_gz_sim import launch_parts
from fleet_gz_sim.isolation import require_isolated
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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


def _start(context):
    # 노드는 launch의 작업 폴더를 물려받는다(gz.launch.py와 같은 검사)
    require_isolated(os.environ, rmw=get_rmw_implementation_identifier(), workdir=Path.cwd())

    def value(name):
        return LaunchConfiguration(name).perform(context)

    on = {name: value(name).lower() in ("true", "1") for name in ("gui", "rviz", "fsm")}
    return [*launch_parts.world(on["gui"]),
            launch_parts.bridge([value("namespace")]),
            *launch_parts.pinky(value("namespace"), value("x"), value("y"), value("yaw"),
                                heartbeat_timeout_s=value("heartbeat_timeout_s"),
                                nav_log_level=value("nav_log_level"),
                                fsm=on["fsm"], rviz=on["rviz"])]


def generate_launch_description():
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=text)
          for name, (default, text) in ARGS.items()),
        OpaqueFunction(function=_start),
    ])
