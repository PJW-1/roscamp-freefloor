"""Gazebo 핑키 3대 + 진짜 관제(안내 4절 3번: 관제·pinky_fsm·Nav2는 진짜, central·팔은 가짜).

    ros2 launch fleet_gz_sim gz.launch.py                       # Gazebo 화면 + 3대 RViz
    ros2 launch fleet_gz_sim gz.launch.py gui:=false rviz:=none   # 화면 없이

- 핑키는 시나리오(fake_robots/scenarios, 기본 basic_orders)의 시작 지점에 놓는다(fleet_setup).
- 경기장은 시나리오의 map이 고른다(fleet_setup.arena_for). 없으면 lane_arena, map: warehouse면
  창고 월드, Nav2 지도, 관제·가짜·fleet_view·장애물의 레인 지도를 모두 창고 것으로.
    ros2 launch fleet_gz_sim gz.launch.py scenario:=gz_warehouse sim_dock_stub:=true \
        obstacles:=warehouse
- 모든 로봇이 IDLE이 되면(wait_ready) 관제, 가짜 작업셀·비전·출고 팔·central, 시나리오 모니터를
  띄운다. 모니터가 기대값을 판정해 result_dir에 summary.json을 쓰고 launch를 끝낸다.
- 시계는 Gazebo /clock 하나를 모두 쓴다. sim_clock을 띄우지 않으므로 시나리오의 speed_factor는
  쓰지 않는다. 주문 시각(at_s)은 Gazebo 시계 0부터 센다.
- 도크: 기본은 진짜 pinky_fsm이 도킹을 DOCK_UNAVAILABLE로 실패시킨다(pinky_dock이 아직 없다).
  임무가 도크에서 멈추고 관제가 재시도한 뒤 격리한다. sim_dock_stub:=true면 시뮬 전용 대역
  (sim_stand_in)이 /{pinky}/dock을 맡고 pinky_fsm의 dock 서버는 /{pinky}/dock_fsm으로 비켜 둔다.
  The stand-in refuses the stub unless use_sim_time is true and isolation.py passes.
- fleet_check(기본 켜짐)가 로봇 사이 최소 거리, 정지 횟수, 쥔 구역 밖 위치 수를
  result_dir/fleet_check.json에 쓴다.
- recovery_probe(fleet_check과 한 프로세스, fleet_check:=false면 따로)가 로봇이 RECOVERING일
  때 전후의 AMCL 위치 오차(Gazebo 실제 위치 대비)를 result_dir/recovery_probe.csv에 쓴다.
- ROS 프로세스 수: LOCALHOST 탐색에서는 DDS 참여자 32개 정도만 서로 보인다. 그래서 브리지는
  하나로 모으고, 로봇마다 Nav2를 프로세스 하나로 묶는다(nav_composition). 노드를 더하면
  test_launch_wiring의 프로세스 수 시험이 한도를 알려 준다.
"""
import math
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
    "fleet_params_file": ("", "관제 파라미터 파일. 비우면 설치된 fleet_params.yaml, 경로 없는"
                              " 이름이면 fleet_core/data/<이름>.yaml(예: fleet_params_real_arena)"),
    "traffic_mode": ("", "lanes 또는 freefloor면 파라미터 파일을 그 방식으로 바꾼 사본"
                         "(result_dir/fleet_params.yaml)으로 돈다. 비우면 파일 그대로"),
    "monitor": ("true", "시나리오 판정. 끝나면 launch를 끝낸다"),
    "obstacles": ("", "레인에 상자를 놓았다 치우는 일정(fleet_gz_sim/obstacles의 이름이나 파일)."
                      " 비면 장애물 없음. roam은 300 s마다 4개"),
    "sim_dock_stub": ("false", "true면 시뮬 전용 도킹 대역: 도크 지점 근처면 몇 초 뒤 성공."
                               " false면 진짜 pinky_fsm(DOCK_UNAVAILABLE)"),
    "fleet_check": ("true", "로봇 사이 최소 거리, 정지 횟수, 쥔 구역 밖 위치 수를"
                            " result_dir/fleet_check.json에"),
    "nav_max_speed": ("", "두 RPP의 최고 속도(m/s). 비면 Nav2 설정 그대로(0.25)."
                          " real_arena는 0.12"),
    "nav_collision_check": ("true", "false면 두 RPP의 충돌 검사를 끈다. 틈이 0.25 m인 real_arena에서"
                                   " 라이다 잡음이 벽을 발자국 안에 찍어 멈추는 것을 막는다"),
    "nav_pass_through": ("true", "잠금 경계에서 끝나는 이동(final_goal 거짓)을 접근 감속 없는 둘째"
                                " RPP(FollowPathThrough)로. false면 모두 FollowPath(둘 다 뜬다)"),
}
ARMS = "arm1 arm2 arm3"


def flag(text: str, name: str) -> bool:
    """A true/false launch argument. A typo must not quietly mean false."""
    value = text.strip().lower()
    if value not in ("true", "false", "1", "0"):
        raise ValueError(f"{name} must be true or false, got {text!r}")
    return value in ("true", "1")


def params_file_of(value: str) -> str:
    """"" stays "", a path stays as given, a bare name is fleet_core/data/<name>.yaml."""
    from fleet_core.common.params import DEFAULT_PARAMS
    if not value or "/" in value or value.endswith(".yaml"):
        return value
    path = DEFAULT_PARAMS.parent / f"{value}.yaml"
    if not path.is_file():
        raise ValueError(f"fleet_params_file {value!r}: no {path}")
    return str(path)


def speed_of(text: str) -> float | None:
    """nav_max_speed: "" means the Nav2 file's speed. A typo must not quietly mean full speed."""
    if not text.strip():
        return None
    try:
        value = float(text)
    except ValueError:
        raise ValueError(f"nav_max_speed must be a number in m/s, got {text!r}") from None
    if not (math.isfinite(value) and value > 0):
        raise ValueError(f"nav_max_speed must be above 0 m/s, got {text!r}")
    # pinky_fsm backs up at backup_speed_mps and never slows a lock-boundary path below
    # horizon_min_mps; a cap under those would not hold
    from pinky_fsm.core import FsmParams
    fsm = FsmParams()
    slowest = max(fsm.backup_speed_mps, fsm.horizon_min_mps)
    if value < slowest:
        raise ValueError(f"nav_max_speed must be at least {slowest} m/s (pinky_fsm backup and "
                         f"slow-down speeds), got {text!r}")
    return value


def params_for(params_file: str, traffic_mode: str, result_dir: str) -> str:
    """The params file every fleet node reads. With a traffic_mode, a copy set to that mode."""
    import yaml
    from fleet_core.common.params import DEFAULT_PARAMS
    from fleet_core.traffic.modes import MODES
    if not traffic_mode:
        return params_file
    if traffic_mode not in MODES:
        raise ValueError(f"traffic_mode must be one of {', '.join(MODES)} or empty, "
                         f"got {traffic_mode!r}")
    source = Path(params_file) if params_file else DEFAULT_PARAMS
    values = yaml.safe_load(source.read_text(encoding="utf-8"))
    if traffic_mode == "freefloor":
        # Without it every rolling move would be final (dataclass default false)
        values.setdefault("mark_final_goal", True)
    copy = Path(result_dir) / "fleet_params.yaml"
    if copy.resolve() == source.resolve() or (copy.exists() and copy.samefile(source)):
        raise ValueError(f"fleet_params_file {source} is where the {traffic_mode} copy goes;"
                         " give another result_dir")
    copy.parent.mkdir(parents=True, exist_ok=True)
    copy.write_text(yaml.safe_dump({**values, "traffic_mode": traffic_mode}, sort_keys=False,
                                   allow_unicode=True), encoding="utf-8")
    return str(copy)


def traffic_mode_of(traffic_mode: str, params_file: str) -> str:
    """The mode the fleet runs: the launch argument, else the params file (lanes by default)."""
    if traffic_mode:
        return traffic_mode
    from fleet_core.common.params import load_params
    given = Path(params_file) if params_file else None
    return (load_params(given) if given and given.exists() else load_params()).traffic_mode


def fleet(scenario_file: str, robots: list[str], result_dir: str, monitor: bool,
          params_file: str = "", obstacles_file: str = "", check: bool = True,
          lane_map: str = "") -> list:
    """진짜 관제와 가짜 작업셀·비전·출고 팔·central·모니터, 장애물(있으면). 모두 Gazebo 시계.
    lane_map: 관제 레인 지도 파일. 비면 기본 지도(노드에 넘기지 않는다)."""
    from fake_robots.common import split_fake_arms
    from fleet_core.traffic.lane_map import load_lane_map
    map_file = {"map_file": lane_map} if lane_map else {}
    lane = {"lane_map": lane_map} if lane_map else {}
    common = {"use_sim_time": True, "scenario_file": scenario_file, **map_file}

    def fake(executable, name, **extra):
        return Node(package="fake_robots", executable=executable, name=name, output="screen",
                    parameters=[{**common, **extra}])

    actions = [Node(package="fleet_manager", executable="fleet_manager_node", name="fleet_manager",
                    output="screen",
                    parameters=[{"use_sim_time": True, "robots": robots,
                                 "params_file": params_file, **map_file}])]
    workcell, vision_arms, swap_arms = split_fake_arms(ARMS, load_lane_map(lane_map or None))
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
                                         "obstacles_file": obstacles_file, **lane}]))
    probe = {"use_sim_time": True, "robots": robots, "result_dir": result_dir, "settle_s": 2.0}
    if check:
        # recovery_probe는 같은 프로세스에서 돈다(DDS 참여자를 아낀다)
        actions.append(Node(package="fleet_gz_sim", executable="fleet_check", name="fleet_check",
                            output="screen",
                            parameters=[{**probe, "recovery_probe": True,
                                         "params_file": params_file,
                                         "output": str(Path(result_dir) / "fleet_check.json"),
                                         **lane}]))
    else:
        actions.append(Node(package="fleet_gz_sim", executable="recovery_probe",
                            name="recovery_probe", output="screen", parameters=[probe]))
    if monitor:
        node = Node(package="fake_robots", executable="scenario_monitor_node",
                    name="scenario_monitor", output="screen",
                    parameters=[{**common, "result_dir": result_dir, "params_file": params_file}],
                    on_exit=Shutdown())
        actions.append(node)
    return actions


def fleet_rviz(robots: list[str], lane_map: str = "", params_file: str = "") -> list:
    """3대를 한 RViz에. 로봇 TF는 namespace로 나뉘므로 위치는 fleet_view가 마커로 그린다."""
    config = launch_parts.share("fleet_gz_sim") / "rviz" / "fleet_view.rviz"
    return [Node(package="fleet_gz_sim", executable="fleet_view", name="fleet_view",
                 output="screen",
                 parameters=[{"use_sim_time": True, "robots": robots, "lane_map": lane_map,
                              "params_file": params_file}]),
            Node(package="rviz2", executable="rviz2", name="fleet_rviz", output="log",
                 arguments=["-d", str(config)], parameters=[{"use_sim_time": True}])]


def _start(context):
    # 노드는 launch의 작업 폴더를 물려받는다. 노드에 cwd를 따로 주면 그 폴더도 봐야 한다.
    require_isolated(os.environ, rmw=get_rmw_implementation_identifier(), workdir=Path.cwd())
    from fleet_gz_sim.fleet_setup import (
        arena_for,
        load,
        obstacles_path,
        scenario_path,
        start_poses,
    )
    from fleet_gz_sim.obstacles import load_schedule

    def value(name):
        return LaunchConfiguration(name).perform(context)

    gz_share, fake_share = launch_parts.share("fleet_gz_sim"), launch_parts.share("fake_robots")
    scenario_file = str(scenario_path(value("scenario"), gz_share, fake_share))
    arena = arena_for(scenario_file, gz_share, fake_share)  # 시나리오의 map이 경기장을 고른다
    site, scenario = load(scenario_file, arena.lane_map)
    obstacles_file = obstacles_path(value("obstacles"), gz_share)
    if obstacles_file:
        load_schedule(obstacles_file, site)  # 틀린 일정이면 Gazebo를 띄우기 전에 멈춘다
    poses = start_poses(site, scenario)
    robots = list(poses)
    composed = flag(value("nav_composition"), "nav_composition")
    stub = flag(value("sim_dock_stub"), "sim_dock_stub")
    check = flag(value("fleet_check"), "fleet_check")
    through = flag(value("nav_pass_through"), "nav_pass_through")
    params_file = params_for(params_file_of(value("fleet_params_file")), value("traffic_mode"),
                             value("result_dir"))
    max_speed = speed_of(value("nav_max_speed"))
    collision_check = flag(value("nav_collision_check"), "nav_collision_check")
    # robots leave the lanes on the free floor: no lanes painted
    floor = traffic_mode_of(value("traffic_mode"), params_file) == "freefloor"
    actions = [*launch_parts.world(flag(value("gui"), "gui"), arena.world, lane_paint=not floor),
               launch_parts.bridge(robots)]
    for robot, (x, y, yaw) in poses.items():
        actions += launch_parts.pinky(robot, x, y, yaw,
                                      heartbeat_timeout_s=value("heartbeat_timeout_s"),
                                      nav_log_level=value("nav_log_level"),
                                      rviz=robot == value("rviz"), dock_aside=stub,
                                      nav_composition=composed, nav_map=arena.nav_map,
                                      nav_map_fine=arena.nav_map_fine,
                                      through_controller_id=(launch_parts.THROUGH_CONTROLLER
                                                             if through else ""),
                                      nav_max_speed=max_speed,
                                      nav_collision_check=collision_check)
    if value("rviz") == "fleet":
        actions += fleet_rviz(robots, arena.lane_map, params_file=params_file)
    map_file = {"map_file": arena.lane_map} if arena.lane_map else {}
    actions.append(Node(package="fleet_gz_sim", executable="sim_stand_in", name="sim_stand_in",
                        output="screen",
                        parameters=[{"use_sim_time": True, "robots": robots,
                                     "scenario_file": scenario_file, "dock_stub": stub,
                                     **map_file}]))
    wait = Node(package="fleet_gz_sim", executable="wait_ready", name="wait_ready",
                output="screen", parameters=[{"robots": robots}])
    monitor = flag(value("monitor"), "monitor")

    def after_wait(event, _context):
        if event.returncode != 0:
            return [LogInfo(msg="fleet_gz_sim: 로봇이 IDLE이 되지 않아 끝낸다"), Shutdown()]
        return fleet(scenario_file, robots, value("result_dir"), monitor,
                     params_file, obstacles_file, check, arena.lane_map)

    actions += [wait, RegisterEventHandler(OnProcessExit(target_action=wait, on_exit=after_wait))]
    return actions


def generate_launch_description():
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=text)
          for name, (default, text) in ARGS.items()),
        OpaqueFunction(function=_start),
    ])
