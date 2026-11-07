"""Gazebo 3대 시뮬의 자리. 지금은 격리 검사만 하고 아무것도 띄우지 않는다.

채울 것: Gazebo 월드, 핑키 모델 3대, 진짜 fleet_manager + pinky_fsm + Nav2,
central과 팔은 fake_robots. 시계는 Gazebo /clock을 쓰므로 sim_clock은 띄우지 않는다.
"""
import os
from pathlib import Path

from fleet_gz_sim.isolation import require_isolated
from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction
from rclpy import get_rmw_implementation_identifier


def _start(context):
    # 노드는 launch의 작업 폴더를 물려받는다. 노드에 cwd를 따로 주면 그 폴더도 봐야 한다.
    require_isolated(os.environ, rmw=get_rmw_implementation_identifier(), workdir=Path.cwd())
    return [LogInfo(msg="fleet_gz_sim: 격리 확인. 월드와 로봇은 아직 없다(자리만 있음).")]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_start)])
