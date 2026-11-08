"""Gazebo 핑키 3대 + 진짜 관제(안내 4절 3번: 관제·pinky_fsm·Nav2는 진짜, central·팔은 가짜).

    ros2 launch fleet_gz_sim gz.launch.py                       # Gazebo 화면 + 3대 RViz
    ros2 launch fleet_gz_sim gz.launch.py gui:=false rviz:=none   # 화면 없이

- 핑키는 시나리오(fake_robots/scenarios, 기본 basic_orders)의 시작 지점에 놓는다(fleet_setup).
- 모든 로봇이 IDLE이 되면(wait_ready) 관제, 가짜 작업셀·비전·출고 팔·central, 시나리오 모니터를
  띄운다. 모니터가 기대값을 판정해 result_dir에 summary.json을 쓰고 launch를 끝낸다.
- 시계는 Gazebo /clock 하나를 모두 쓴다. sim_clock을 띄우지 않으므로 시나리오의 speed_factor는
  쓰지 않는다. 주문 시각(at_s)은 Gazebo 시계 0부터 센다.
- 도크: pinky_dock이 아직 없어 시뮬 전용 대역(sim_stand_in)이 /{pinky}/dock을 맡고, pinky_fsm의
  dock 서버는 /{pinky}/dock_fsm으로 비켜 둔다. 실물에서는 쓰지 않는다.
- ROS 프로세스 수: LOCALHOST 탐색에서는 DDS 참여자 32개 정도만 서로 보인다. 그래서 브리지는
  하나로 모으고, 로봇마다 Nav2를 프로세스 하나로 묶는다(nav_composition). 노드를 더하면
  test_launch_wiring의 프로세스 수 시험이 한도를 알려 준다.
"""
import os
from pathlib import Path

from fleet_gz_sim import launch_parts
from fleet_gz_sim.isolation import require_isolated
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rclpy import get_rmw_implementation_identifier

ARGS = {
    "scenario": ("basic_orders", "시나리오 이름(fleet_gz_sim, fake_robots 순서로 찾음)이나 파일."
                                  " 기본 지도, 로봇 고장 없음. gz_roam은 24분 순환 운행"),
    "gui": ("true", "Gazebo 화면"),
    "rviz": ("fleet", "fleet이면 3대를 한 RViz에(fleet_view), 로봇 이름이면 그 로봇의 Nav2 화면,"
                      " none이면 띄우지 않는다"),
    "heartbeat_timeout_s": ("1.5", "pinky_fsm heartbeat 끊김 판정"),
    "nav_log_level": ("warn", "Nav2 노드 로그 수준"),
    "nav_composition": ("true", "로봇마다 Nav2 노드 9개를 프로세스 하나로. false면 9개"
                                "(3대면 DDS 참여자가 LOCALHOST 탐색 한도 32개를 넘는다)"),
    "result_dir": ("gz_result", "시나리오 모니터 결과 폴더"),
    "fleet_params_file": ("", "관제 파라미터 파일. 비우면 설치된 fleet_params.yaml"),
    "monitor": ("true", "시나리오 판정. 끝나면 launch를 끝낸다"),
    "obstacles": ("", "레인에 상자를 놓았다 치우는 일정(fleet_gz_sim/obstacles의 이름이나 파일)."
                      " 비면 장애물 없음. roam은 300 s마다 4개"),
}
ARMS = "arm1 arm2 arm3"


def fleet(scenario_file: str, robots: list[str], result_dir: str, monitor: bool,
          params_file: str = "", obstacles_file: str = "") -> list:
    """진짜 관제와 가짜 작업셀·비전·출고 팔·central·모니터, 장애물(있으면). 모두 Gazebo 시계."""
    from fake_robots.common import split_fake_arms
    from fleet_core.traffic.lane_map import load_lane_map
    common = {"use_sim_time": True, "scenario_file": scenario_file}

    def fake(executable, name, **extra):
        return Node(package="fake_robots", executable=executable, name=name, output="screen",
                    parameters=[{**common, **extra}])

    actions = [Node(package="fleet_manager", executable="fleet_manager_node", name="fleet_manager",
                    output="screen",
                    parameters=[{"use_sim_time": True, "robots": robots,
                                 "params_file": params_file}])]
    workcell, vision_arms, swap_arms = split_fake_arms(ARMS, load_lane_map(None))
    if workcell:
        actions.append(fake("fake_workcell_node", "fake_workcell"))
    actions += [fake("fake_vision_node", f"fake_vision_{arm}", arm_id=arm) for arm in vision_arms]
    actions += [fake("fake_arm_node", f"fake_{arm}", arm_id=arm) for arm in swap_arms]
    actions.append(fake("fake_central_node", "fake_central", params_file=params_file))
    if obstacles_file:
        # 로봇 실제 위치(sim/truth)를 보고 비어 있을 때만 놓는다. 일정 시각은 Gazebo 시계
        actions.append(Node(package="fleet_gz_sim", executable="sim_obstacles",
                            name="sim_obstacles", output="screen",
                            parameters=[{"use_sim_time": True, "robots": robots,
                                         "obstacles_file": obstacles_file}]))
    if monitor:
        node = Node(package="fake_robots", executable="scenario_monitor_node",
                    name="scenario_monitor", output="screen",
                    parameters=[{**common, "result_dir": result_dir, "params_file": params_file}],
                    on_exit=Shutdown())
        actions.append(node)
    return actions


def fleet_rviz(robots: list[str], lane_map: str = "") -> list:
    """3대를 한 RViz에. 로봇 TF는 namespace로 나뉘므로 위치는 fleet_view가 마커로 그린다."""
    config = launch_parts.share("fleet_gz_sim") / "rviz" / "fleet_view.rviz"
    return [Node(package="fleet_gz_sim", executable="fleet_view", name="fleet_view",
                 output="screen",
                 parameters=[{"use_sim_time": True, "robots": robots, "lane_map": lane_map}]),
            Node(package="rviz2", executable="rviz2", name="fleet_rviz", output="log",
                 arguments=["-d", str(config)], parameters=[{"use_sim_time": True}])]


def _start(context):
    # 노드는 launch의 작업 폴더를 물려받는다. 노드에 cwd를 따로 주면 그 폴더도 봐야 한다.
    require_isolated(os.environ, rmw=get_rmw_implementation_identifier(), workdir=Path.cwd())
    from fleet_gz_sim.fleet_setup import load, obstacles_path, scenario_path, start_poses
    from fleet_gz_sim.obstacles import load_schedule

    def value(name):
        return LaunchConfiguration(name).perform(context)

    scenario_file = str(scenario_path(value("scenario"), launch_parts.share("fleet_gz_sim"),
                                      launch_parts.share("fake_robots")))
    site, scenario = load(scenario_file)
    obstacles_file = obstacles_path(value("obstacles"), launch_parts.share("fleet_gz_sim"))
    if obstacles_file:
        load_schedule(obstacles_file, site)  # 틀린 일정이면 Gazebo를 띄우기 전에 멈춘다
    poses = start_poses(site, scenario)
    robots = list(poses)
    composed = value("nav_composition").lower() in ("true", "1")
    actions = [*launch_parts.world(value("gui").lower() in ("true", "1")),
               launch_parts.bridge(robots)]
    for robot, (x, y, yaw) in poses.items():
        actions += launch_parts.pinky(robot, x, y, yaw,
                                      heartbeat_timeout_s=value("heartbeat_timeout_s"),
                                      nav_log_level=value("nav_log_level"),
                                      rviz=robot == value("rviz"), dock_aside=True,
                                      nav_composition=composed)
    if value("rviz") == "fleet":
        actions += fleet_rviz(robots)
    actions.append(Node(package="fleet_gz_sim", executable="sim_stand_in", name="sim_stand_in",
                        output="screen",
                        parameters=[{"use_sim_time": True, "robots": robots,
                                     "scenario_file": scenario_file}]))
    wait = Node(package="fleet_gz_sim", executable="wait_ready", name="wait_ready",
                output="screen", parameters=[{"robots": robots}])
    monitor = value("monitor").lower() in ("true", "1")

    def after_wait(event, _context):
        if event.returncode != 0:
            return [LogInfo(msg="fleet_gz_sim: 로봇이 IDLE이 되지 않아 끝낸다"), Shutdown()]
        return fleet(scenario_file, robots, value("result_dir"), monitor,
                     value("fleet_params_file"), obstacles_file)

    actions += [wait, RegisterEventHandler(OnProcessExit(target_action=wait, on_exit=after_wait))]
    return actions


def generate_launch_description():
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=text)
          for name, (default, text) in ARGS.items()),
        OpaqueFunction(function=_start),
    ])
