"""gz.launch.py: 격리 검사를 먼저 하고, 핑키 3대와 진짜 관제, 가짜 central·팔을 잇는 것을 본다."""
import importlib.util
import math
from pathlib import Path

import pytest

launch = pytest.importorskip("launch")
rclpy = pytest.importorskip("rclpy")

LAUNCH = Path(__file__).parents[1] / "launch" / "gz.launch.py"
FAST_DDS = ("ROS_STATIC_PEERS", "RMW_IMPLEMENTATION", "ROS_DISCOVERY_SERVER",
            "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE",
            "FASTDDS_ENVIRONMENT_FILE", "FASTDDS_BUILTIN_TRANSPORTS")
ROBOTS = ["pinky1", "pinky2", "pinky3"]
STARTS = {"pinky1": (1.20, 2.05, 0.0), "pinky2": (0.80, 2.05, math.atan2(0.10, 0.45)),
          "pinky3": (0.35, 1.95, math.pi / 2)}  # basic_orders: P-6, P-5, P-4


@pytest.fixture
def gz(monkeypatch, tmp_path):
    spec = importlib.util.spec_from_file_location("gz_launch", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setenv("ROS_DOMAIN_ID", "44")
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    for name in FAST_DDS:
        monkeypatch.delenv(name, raising=False)
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(module, "get_rmw_implementation_identifier", lambda: "rmw_fastrtps_cpp")
    return module


def test_the_rmw_comes_from_rclpy():
    spec = importlib.util.spec_from_file_location("gz_launch_plain", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.get_rmw_implementation_identifier is rclpy.get_rmw_implementation_identifier


def run(gz, **overrides):
    """ros2 launch처럼 인자 기본값을 넣고(overrides로 바꾸고) OpaqueFunction을 실행한다."""
    context = launch.LaunchContext()
    *args, start = gz.generate_launch_description().entities
    for action in args:
        action.visit(context)
    context.launch_configurations.update(overrides)
    return start.execute(context)


def test_launch_reads_the_current_environment(gz, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "6")
    with pytest.raises(RuntimeError, match="ROS_DOMAIN_ID"):
        run(gz)


def test_launch_checks_the_loaded_rmw(gz, monkeypatch):
    monkeypatch.setattr(gz, "get_rmw_implementation_identifier", lambda: "rmw_cyclonedds_cpp")
    with pytest.raises(RuntimeError, match="rmw_cyclonedds_cpp"):
        run(gz)


def test_launch_checks_the_working_directory(gz, tmp_path):
    (tmp_path / "DEFAULT_FASTRTPS_PROFILES.xml").write_text("<dds/>")
    with pytest.raises(RuntimeError, match="DEFAULT_FASTRTPS_PROFILES.xml"):
        run(gz)


# -- 배선: launch 동작 대신 만든 인자를 기록해 본다 ------------------------------------------

KINDS = ("Node", "IncludeLaunchDescription", "PythonLaunchDescriptionSource", "ExecuteProcess",
         "GroupAction", "SetRemap", "RegisterEventHandler", "OnProcessExit")


class Made:
    def __init__(self, kind, args, kwargs):
        self.kind, self.args, self.kw = kind, args, kwargs

    def flat(self):
        yield self
        if self.kind == "GroupAction":
            for child in self.args[0]:
                yield from (child.flat() if isinstance(child, Made) else [child])


def need(*packages):
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    for package in packages:
        try:
            get_package_share_directory(package)
        except PackageNotFoundError:
            pytest.skip(f"{package} is not installed")


@pytest.fixture
def made(gz, monkeypatch):
    need("pinky_description", "pinky_nav_config", "pinky_fsm", "fake_robots", "fleet_manager")
    from fleet_gz_sim import launch_parts
    for module in (gz, launch_parts):
        for kind in KINDS:
            if hasattr(module, kind):
                monkeypatch.setattr(module, kind, lambda *a, _k=kind, **kw: Made(_k, a, kw))
    return gz


def flat(actions):
    for a in actions:
        yield from (a.flat() if isinstance(a, Made) else [a])


def nodes(actions, executable=None):
    return [a for a in flat(actions) if isinstance(a, Made) and a.kind == "Node"
            and (executable is None or a.kw["executable"] == executable)]


def params(node) -> dict:
    out = {}
    for p in node.kw.get("parameters", []):
        out.update(p)
    return out


def includes(actions, launch_file):
    return [a for a in flat(actions) if isinstance(a, Made) and a.kind == "IncludeLaunchDescription"
            and str(a.args[0].args[0]).endswith(launch_file)]


def launch_args(include) -> dict:
    return dict(include.kw["launch_arguments"])


def test_three_pinkys_start_where_the_scenario_puts_them(made):
    actions = run(made)
    spawned = {}
    for create in nodes(actions, "create"):
        a = create.kw["arguments"]
        spawned[a[a.index("-name") + 1]] = tuple(float(a[a.index(k) + 1])
                                                 for k in ("-x", "-y", "-Y"))
    assert spawned.keys() == STARTS.keys()
    for robot, pose in STARTS.items():
        assert spawned[robot] == pytest.approx(pose)
    navs = {launch_args(i)["namespace"]: launch_args(i)
            for i in includes(actions, "nav2.launch.py")}
    assert navs.keys() == STARTS.keys()
    for robot, nav in navs.items():  # AMCL 시작 자세가 놓은 자리와 같다
        assert tuple(float(nav[k]) for k in ("initial_x", "initial_y", "initial_yaw")) == \
            pytest.approx(spawned[robot])


def test_gazebo_time_and_every_pinky_share_one_bridge(made):
    """/clock을 브리지 여럿이 같이 내면 표본 순서가 섞여 시간이 뒤로 갈 수 있다.
    브리지를 로봇마다 두면 DDS 참여자도 그만큼 는다(test_ros_process_budget_...)."""
    [bridge] = nodes(run(made), "parameter_bridge")
    topics = [str(x).split("@")[0] for x in bridge.kw["arguments"]]
    assert topics.count("/clock") == 1
    for robot in ROBOTS:
        for name in ("scan", "odom", "tf", "joint_states", "cmd_vel", "truth"):
            assert topics.count(f"/model/{robot}/{name}") == 1
    remaps = dict(bridge.kw["remappings"])
    assert remaps["/model/pinky2/cmd_vel"] == "/pinky2/cmd_vel"
    assert remaps["/model/pinky3/truth"] == "/pinky3/sim/truth"


def declared(launch_file) -> set:
    from ament_index_python.packages import get_package_share_directory
    package = {"nav2.launch.py": "pinky_nav_config", "pinky_fsm.launch.py": "pinky_fsm"}
    path = Path(get_package_share_directory(package[launch_file])) / "launch" / launch_file
    spec = importlib.util.spec_from_file_location(launch_file.replace(".", "_"), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return {a.name for a in module.generate_launch_description().entities
            if type(a).__name__ == "DeclareLaunchArgument"}


def test_arguments_do_not_leak_into_the_robot_launches(gz):
    """include는 인자 범위를 나누지 않는다. gz.launch.py의 params_file(빈 값)이 Nav2의
    params_file 기본값을 덮어 launch가 바로 끝났다. 같은 뜻으로 넘기는 값만 겹쳐도 된다."""
    need("pinky_nav_config", "pinky_fsm")
    shared = {"heartbeat_timeout_s"}  # pinky_fsm에 같은 값을 그대로 넘긴다
    for launch_file in ("nav2.launch.py", "pinky_fsm.launch.py"):
        assert set(gz.ARGS) & declared(launch_file) <= shared, launch_file


def test_nav2_runs_composed_unless_asked_not_to(made):
    """한 대의 Nav2 노드 9개를 프로세스 하나에 담는다(pinky_nav_config use_composition)."""
    composed = [launch_args(i)["use_composition"] for i in includes(run(made), "nav2.launch.py")]
    assert composed == ["true"] * 3
    separate = [launch_args(i)["use_composition"]
                for i in includes(run(made, nav_composition="false"), "nav2.launch.py")]
    assert separate == ["false"] * 3


NAV2_NODES = 9  # pinky_nav_config: map_server 둘, amcl, 서버 넷, lifecycle manager 둘
LOCALHOST_PARTICIPANTS = 32  # Fast DDS LOCALHOST 탐색에서 서로 보이는 참여자 수(실측)
SPARE = 6  # launch 프로세스, RViz, 사람이 띄우는 ros2 명령 몇 개


def ros_processes(actions, fleet_actions) -> int:
    """ROS 프로세스 하나가 DDS 참여자 하나다. Gazebo 서버와 화면은 ROS가 아니다."""
    count = len(nodes(actions)) + len(nodes(fleet_actions))
    for nav in includes(actions, "nav2.launch.py"):
        count += 1 if launch_args(nav)["use_composition"] == "true" else NAV2_NODES
    return count + len(includes(actions, "pinky_fsm.launch.py"))


def test_ros_process_budget_leaves_room_under_the_localhost_limit(made):
    """3대를 처음 띄웠을 때 참여자가 32개를 넘어 pinky3의 토픽이 보이지 않았다.

    launch가 만드는 프로세스를 세는 예산 계산이다. 실제로 서로 보이는지는 Gazebo를 띄워
    확인해야 한다(README). 잠깐 돌고 끝나는 create와 wait_ready도 넉넉하게 같이 센다."""
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory("fake_robots")) / "scenarios" / "basic_orders.yaml"
    fleet = made.fleet(str(path), ROBOTS, result_dir="out", monitor=True,
                       obstacles_file="roam.yaml")  # 장애물 노드까지 띄운 가장 많은 경우
    assert ros_processes(run(made), fleet) + SPARE <= LOCALHOST_PARTICIPANTS
    assert ros_processes(run(made, nav_composition="false"), fleet) > LOCALHOST_PARTICIPANTS


def dock_aside_groups(actions, robot):
    return [a for a in flat(actions) if isinstance(a, Made) and a.kind == "GroupAction"
            and any(isinstance(c, Made) and c.kind == "SetRemap"
                    and c.args == (f"/{robot}/dock", f"/{robot}/dock_fsm") for c in a.flat())]


def test_the_real_pinky_fsm_answers_dock_by_default(made):
    """Work order H8: the real pinky_fsm fails docking with DOCK_UNAVAILABLE unless the sim-only
    stub is asked for. The stand-in still relays the true pose for the scenario monitor."""
    actions = run(made)
    fsms = {launch_args(i)["namespace"] for i in includes(actions, "pinky_fsm.launch.py")}
    assert fsms == set(ROBOTS)
    for robot in ROBOTS:
        assert dock_aside_groups(actions, robot) == []
    assert not [a for a in flat(actions) if isinstance(a, Made) and a.kind == "SetRemap"]
    [stand_in] = nodes(actions, "sim_stand_in")
    assert params(stand_in)["dock_stub"] is False
    assert params(stand_in)["robots"] == ROBOTS and params(stand_in)["use_sim_time"] is True


def test_sim_dock_stub_moves_the_pinky_fsm_dock_aside_and_turns_the_stub_on(made):
    actions = run(made, sim_dock_stub="true")
    for robot in ROBOTS:
        [group] = dock_aside_groups(actions, robot)
        [fsm] = includes([group], "pinky_fsm.launch.py")
        assert launch_args(fsm)["namespace"] == robot
    [stand_in] = nodes(actions, "sim_stand_in")
    assert params(stand_in)["dock_stub"] is True and params(stand_in)["use_sim_time"] is True


def test_sim_dock_stub_still_needs_an_isolated_network(made, monkeypatch):
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET")
    with pytest.raises(RuntimeError, match="LOCALHOST"):
        run(made, sim_dock_stub="true")


@pytest.mark.parametrize("name", ["sim_dock_stub", "fleet_check", "gui", "monitor",
                                  "nav_composition"])
def test_a_misspelt_switch_stops_the_launch(made, name):
    with pytest.raises(ValueError, match=name):
        run(made, **{name: "ture"})


def test_fleet_check_records_into_the_result_dir_on_gazebo_time(made):
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory("fake_robots")) / "scenarios" / "basic_orders.yaml"
    [check] = nodes(made.fleet(str(path), ROBOTS, result_dir="out", monitor=True),
                    "fleet_check")
    assert params(check).items() >= {"use_sim_time": True, "robots": ROBOTS,
                                     "output": str(Path("out") / "fleet_check.json")}.items()
    assert not nodes(made.fleet(str(path), ROBOTS, result_dir="out", monitor=True,
                                check=False), "fleet_check")


def test_fleet_check_follows_the_launch_switch(made, monkeypatch):
    seen = {}
    real = made.fleet

    def spy(*args, **kw):
        seen["check"] = args[6] if len(args) > 6 else kw.get("check", True)
        return real(*args, **kw)

    monkeypatch.setattr(made, "fleet", spy)
    for value, expected in (("true", True), ("false", False)):
        actions = run(made, fleet_check=value)
        [on_exit] = [a.args[0].kw["on_exit"] for a in flat(actions) if isinstance(a, Made)
                     and a.kind == "RegisterEventHandler"
                     and callable(a.args[0].kw["on_exit"])]  # the one after wait_ready
        on_exit(type("Exit", (), {"returncode": 0})(), None)
        assert seen["check"] is expected


def test_the_fleet_waits_until_every_pinky_is_ready(made):
    """주문은 시나리오 시간(Gazebo 시계)으로 나간다. 로봇이 서기 전에 관제가 명령하지 않게 한다."""
    actions = run(made)
    [wait] = nodes(actions, "wait_ready")
    assert params(wait)["robots"] == ROBOTS
    assert not nodes(actions, "fleet_manager_node")  # 기다린 뒤에 띄운다


def test_the_real_fleet_manager_runs_with_fake_central_and_arms_on_gazebo_time(made):
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory("fake_robots")) / "scenarios" / "basic_orders.yaml"
    actions = made.fleet(str(path), ROBOTS, result_dir="out", monitor=True)
    [fleet] = nodes(actions, "fleet_manager_node")
    assert params(fleet)["robots"] == ROBOTS and params(fleet)["use_sim_time"] is True
    fakes = [n for n in nodes(actions) if n.kw["package"] == "fake_robots"]
    assert sorted(n.kw["executable"] for n in fakes) == sorted(
        ["fake_workcell_node", "fake_vision_node", "fake_vision_node", "fake_arm_node",
         "fake_central_node", "scenario_monitor_node"])  # 가짜 핑키와 sim_clock은 없다
    for n in fakes:
        assert params(n)["use_sim_time"] is True and params(n)["scenario_file"] == str(path)


def test_scenarios_the_arena_cannot_run_start_nothing(made):
    from fleet_gz_sim.fleet_setup import ScenarioUnfit
    with pytest.raises(ScenarioUnfit):
        run(made, scenario="stuck_robot")


def test_gazebo_transport_stays_on_this_computer(made, monkeypatch):
    monkeypatch.setenv("GZ_IP", "192.168.0.10")
    monkeypatch.delenv("GZ_PARTITION", raising=False)
    env ={"".join(s.perform(None) for s in a.name): "".join(s.perform(None) for s in a.value)
           for a in run(made) if type(a).__name__ == "SetEnvironmentVariable"}
    assert env["GZ_IP"] == "127.0.0.1" and env["GZ_PARTITION"] == "fleet_gz_44"


def test_obstacles_are_off_unless_a_schedule_is_given(made):
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory("fake_robots")) / "scenarios" / "basic_orders.yaml"
    assert not nodes(made.fleet(str(path), ROBOTS, result_dir="out", monitor=True),
                     "sim_obstacles")


def test_the_obstacle_node_places_boxes_on_gazebo_time_away_from_every_pinky(made):
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory("fake_robots")) / "scenarios" / "basic_orders.yaml"
    actions = made.fleet(str(path), ROBOTS, result_dir="out", monitor=True,
                         obstacles_file="/some/roam.yaml")
    [node] = nodes(actions, "sim_obstacles")
    assert params(node) == {"use_sim_time": True, "robots": ROBOTS,
                            "obstacles_file": "/some/roam.yaml"}


def test_an_unknown_obstacle_schedule_stops_the_launch_before_anything_starts(made):
    from fleet_gz_sim.obstacles import ScheduleError
    with pytest.raises(ScheduleError, match="no_such_schedule"):
        run(made, obstacles="no_such_schedule")


def test_one_rviz_shows_the_whole_fleet_by_default(made):
    """로봇마다 TF가 /pinkyN/tf로 나뉘어 RViz 하나는 한 대만 그린다. 기본은 3대 표시 노드 + RViz."""
    actions = run(made)
    [view] = nodes(actions, "fleet_view")
    assert params(view)["robots"] == ROBOTS and params(view)["use_sim_time"] is True
    [rviz] = nodes(actions, "rviz2")
    assert rviz.kw.get("namespace") in (None, "")
    assert rviz.kw["arguments"][-1].endswith("fleet_view.rviz")
    assert params(rviz)["use_sim_time"] is True


def test_rviz_can_follow_one_pinky_or_stay_closed(made):
    actions = run(made, rviz="pinky2")
    assert not nodes(actions, "fleet_view")
    [rviz] = nodes(actions, "rviz2")
    assert rviz.kw["namespace"] == "pinky2"
    closed = run(made, rviz="none")
    assert not nodes(closed, "fleet_view") and not nodes(closed, "rviz2")


def test_the_recovery_probe_shares_the_fleet_check_process(made):
    """A process of its own would cost a DDS participant (test_ros_process_budget_...)."""
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory("fake_robots")) / "scenarios" / "basic_orders.yaml"
    fleet = made.fleet(str(path), ROBOTS, result_dir="out", monitor=True)
    assert nodes(fleet, "recovery_probe") == []
    [check] = nodes(fleet, "fleet_check")
    assert params(check).items() >= {"use_sim_time": True, "robots": ROBOTS,
                                     "recovery_probe": True, "result_dir": "out",
                                     "settle_s": 2.0}.items()


def test_without_fleet_check_the_recovery_probe_runs_on_its_own(made):
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory("fake_robots")) / "scenarios" / "basic_orders.yaml"
    fleet = made.fleet(str(path), ROBOTS, result_dir="out", monitor=True, check=False)
    [probe] = nodes(fleet, "recovery_probe")
    assert params(probe) == {"use_sim_time": True, "robots": ROBOTS, "result_dir": "out",
                             "settle_s": 2.0}


# -- traffic_mode: free-floor runs on a copy of the params with that mode ----------------------


def after_wait(actions):
    [on_exit] = [a.args[0].kw["on_exit"] for a in flat(actions) if isinstance(a, Made)
                 and a.kind == "RegisterEventHandler" and callable(a.args[0].kw["on_exit"])]
    return on_exit(type("Exit", (), {"returncode": 0})(), None)


def params_files(actions) -> dict:
    return {n.kw["executable"]: params(n)["params_file"] for n in nodes(actions)
            if "params_file" in params(n)}


def test_traffic_mode_runs_the_fleet_on_a_copy_of_the_params_with_that_mode(made, tmp_path):
    import yaml
    from fleet_core.common.params import DEFAULT_PARAMS
    out = tmp_path / "result"
    actions = run(made, traffic_mode="freefloor", result_dir=str(out))
    used = params_files([*actions, *after_wait(actions)])
    assert set(used) >= {"fleet_manager_node", "fake_central_node", "scenario_monitor_node",
                         "fleet_check", "fleet_view"}
    [path] = set(used.values())
    assert Path(path).parent == out
    copy = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    given = yaml.safe_load(DEFAULT_PARAMS.read_text(encoding="utf-8"))
    assert copy == {**given, "traffic_mode": "freefloor"}


def test_without_traffic_mode_every_node_gets_the_params_file_as_given(made):
    actions = run(made, fleet_params_file="mine.yaml")
    used = params_files([*actions, *after_wait(actions)])
    assert {"fleet_check", "fleet_view", "fleet_manager_node"} <= set(used)
    assert set(used.values()) == {"mine.yaml"}


def test_an_unknown_traffic_mode_stops_the_launch(made):
    with pytest.raises(ValueError, match="traffic_mode"):
        run(made, traffic_mode="rails")


def test_the_params_copy_never_overwrites_the_file_it_copies(gz, tmp_path):
    out = tmp_path / "result"
    out.mkdir()
    mine = out / "fleet_params.yaml"
    mine.write_text("traffic_mode: lanes  # mine\n", encoding="utf-8")
    with pytest.raises(ValueError, match="fleet_params_file"):
        gz.params_for(str(mine), "freefloor", str(out))
    assert mine.read_text(encoding="utf-8") == "traffic_mode: lanes  # mine\n"


def test_the_params_copy_never_writes_through_a_hard_link_to_its_source(gz, tmp_path):
    import os
    mine = tmp_path / "mine.yaml"
    mine.write_text("traffic_mode: lanes  # mine\n", encoding="utf-8")
    out = tmp_path / "result"
    out.mkdir()
    os.link(mine, out / "fleet_params.yaml")
    with pytest.raises(ValueError, match="fleet_params_file"):
        gz.params_for(str(mine), "freefloor", str(out))
    assert mine.read_text(encoding="utf-8") == "traffic_mode: lanes  # mine\n"


# -- 시나리오의 map이 경기장을 고른다 ---------------------------------------------------------

def after_ready(actions):
    """wait_ready가 0으로 끝났을 때 띄우는 관제 쪽 노드들."""
    exits = [a.args[0] for a in flat(actions)
             if isinstance(a, Made) and a.kind == "RegisterEventHandler"]
    [handler] = [e for e in exits if callable(e.kw.get("on_exit"))]

    class Exit:
        returncode = 0
    return handler.kw["on_exit"](Exit(), None)


def world_file(actions) -> str:
    [server] = [a for a in flat(actions) if isinstance(a, Made) and a.kind == "ExecuteProcess"
                and a.kw.get("name") == "gz_server"]
    return server.kw["cmd"][-1]


def test_without_a_map_the_lane_arena_is_used_everywhere(made):
    actions = run(made)
    assert world_file(actions).endswith("/worlds/lane_arena.sdf")
    for nav in includes(actions, "nav2.launch.py"):
        assert launch_args(nav)["map"].endswith("/maps/lane_arena.yaml")
        assert launch_args(nav)["map_fine"].endswith("/maps/lane_arena_fine.yaml")
    fleet = after_ready(actions)
    for node in [*nodes(actions), *nodes(fleet)]:
        assert not params(node).get("map_file") and not params(node).get("lane_map"), node.kw


def test_a_warehouse_scenario_runs_on_the_warehouse_world_maps_and_lane_map(made):
    from ament_index_python.packages import get_package_share_directory
    lane_map = str(Path(get_package_share_directory("fake_robots")) / "maps" / "warehouse.yaml")
    actions = run(made, scenario="gz_warehouse", obstacles="warehouse")
    assert world_file(actions).endswith("/worlds/warehouse.sdf")
    navs = includes(actions, "nav2.launch.py")
    assert len(navs) == 3
    for nav in navs:
        assert launch_args(nav)["map"].endswith("/maps/warehouse.yaml")
        assert launch_args(nav)["map_fine"].endswith("/maps/warehouse_fine.yaml")
    spawned = sorted((float(c.kw["arguments"][c.kw["arguments"].index("-x") + 1]),
                      float(c.kw["arguments"][c.kw["arguments"].index("-y") + 1]))
                     for c in nodes(actions, "create"))
    assert spawned == pytest.approx([(2.65, 3.15), (3.0, 3.15), (3.35, 3.15)])  # P-3..P-1
    [view] = nodes(actions, "fleet_view")
    assert params(view)["lane_map"] == lane_map
    [stand_in] = nodes(actions, "sim_stand_in")
    assert params(stand_in)["map_file"] == lane_map
    fleet = after_ready(actions)
    [manager] = nodes(fleet, "fleet_manager_node")
    assert params(manager)["map_file"] == lane_map
    for node in nodes(fleet):
        if node.kw["package"] == "fake_robots":
            assert params(node)["map_file"] == lane_map, node.kw["executable"]
    for executable in ("sim_obstacles", "fleet_check"):
        [node] = nodes(fleet, executable)
        assert params(node)["lane_map"] == lane_map, executable


def test_an_obstacle_schedule_is_checked_against_the_scenario_map(made):
    from fleet_gz_sim.obstacles import ScheduleError
    with pytest.raises(ScheduleError):
        run(made, scenario="gz_warehouse", obstacles="roam")  # roam의 레인은 창고 지도에 없다
    with pytest.raises(ScheduleError):
        run(made, obstacles="warehouse")


def test_free_floor_on_the_warehouse_gives_every_floor_user_one_lane_map_and_params(made,
                                                                                   tmp_path):
    """The floor map follows the lane map (floor_map_for), so one lane map and one params copy
    for every node that reads cells means one floor map."""
    from ament_index_python.packages import get_package_share_directory
    from fleet_core.common.params import load_params
    from fleet_core.traffic.floor.floor_map import DEFAULT_FLOOR
    from fleet_gz_sim.fleet_setup import load_zone_space
    lane_map = str(Path(get_package_share_directory("fake_robots")) / "maps" / "warehouse.yaml")
    out = tmp_path / "result"
    actions = run(made, scenario="gz_warehouse", traffic_mode="freefloor", result_dir=str(out),
                  sim_dock_stub="true")
    every = [*nodes(actions), *nodes(after_ready(actions))]
    floor_users = {"fleet_manager_node", "scenario_monitor_node", "fleet_check", "fleet_view"}
    used = {n.kw["executable"]: params(n) for n in every if n.kw["executable"] in floor_users}
    assert set(used) == floor_users
    [params_file] = {p["params_file"] for p in used.values()}
    assert Path(params_file).parent == out
    assert load_params(params_file).traffic_mode == "freefloor"
    for executable, p in used.items():
        assert p.get("map_file", p.get("lane_map")) == lane_map, executable
    space = load_zone_space(lane_map, params_file)
    warehouse = DEFAULT_FLOOR.parent / "floor_map_warehouse.yaml"
    assert space.floor.source_text == warehouse.read_text(encoding="utf-8")


def test_the_freefloor_copy_marks_partial_moves_unless_the_file_says_otherwise(gz, tmp_path):
    """Without mark_final_goal the dataclass default (false) would mark every move final, and the
    pass-through controller would never be used."""
    import yaml
    mine = tmp_path / "mine.yaml"
    mine.write_text("traffic_mode: lanes\n", encoding="utf-8")
    copy = yaml.safe_load(Path(gz.params_for(str(mine), "freefloor", str(tmp_path / "a")))
                          .read_text(encoding="utf-8"))
    assert copy == {"traffic_mode": "freefloor", "mark_final_goal": True}
    mine.write_text("traffic_mode: lanes\nmark_final_goal: false\n", encoding="utf-8")
    copy = yaml.safe_load(Path(gz.params_for(str(mine), "freefloor", str(tmp_path / "b")))
                          .read_text(encoding="utf-8"))
    assert copy["mark_final_goal"] is False, "an explicit choice is kept"
    lanes = yaml.safe_load(Path(gz.params_for(str(mine), "lanes", str(tmp_path / "c")))
                           .read_text(encoding="utf-8"))
    assert lanes == {"traffic_mode": "lanes", "mark_final_goal": False}


# -- pass-through RPP: no approach slow-down for moves that end at a lock boundary -------------

THROUGH = "FollowPathThrough"


def nav2_source() -> dict:
    import yaml
    from ament_index_python.packages import get_package_share_directory
    path = (Path(get_package_share_directory("pinky_nav_config")) / "params"
            / "nav2_params.yaml")
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def nav2_files(actions) -> dict:
    """{robot namespace: the Nav2 params that robot's Nav2 gets}, read now (the files are
    per robot and process, so a later launch in this test writes over them)."""
    import yaml
    out = {}
    for nav in includes(actions, "nav2.launch.py"):
        args = launch_args(nav)
        out[args["namespace"]] = yaml.safe_load(
            Path(args["params_file"]).read_text(encoding="utf-8"))
    return out


def test_gazebo_can_give_every_nav2_a_second_rpp_without_the_approach_slow_down(made, tmp_path):
    actions = run(made, nav_pass_through="true", result_dir=str(tmp_path / "b"))
    with_through = nav2_files(actions)
    fsms = includes(actions, "pinky_fsm.launch.py")
    assert len(fsms) == 3
    assert {launch_args(i)["through_controller_id"] for i in fsms} == {THROUGH}
    plain = nav2_files(run(made, nav_pass_through="false", result_dir=str(tmp_path / "a")))
    assert len(with_through) == 3 and set(with_through) == set(plain)
    base = nav2_source()["controller_server"]["ros__parameters"]
    for robot, used in with_through.items():
        server = used["controller_server"]["ros__parameters"]
        assert server["controller_plugins"] == [*base["controller_plugins"], THROUGH]
        assert server[THROUGH] == {**base["FollowPath"], "approach_velocity_scaling_dist": 0.05}
        assert server["FollowPath"] == base["FollowPath"], "final goals keep the normal RPP"
        without = {k: v for k, v in server.items() if k not in (THROUGH, "controller_plugins")}
        same = plain[robot]["controller_server"]["ros__parameters"]
        assert without == {k: v for k, v in same.items() if k != "controller_plugins"}
        assert {k: v for k, v in used.items() if k != "controller_server"} == \
            {k: v for k, v in plain[robot].items() if k != "controller_server"}, "sim AMCL kept"


def test_by_default_gazebo_runs_the_robot_nav2_controllers(made, tmp_path):
    """A (flag only) is the default; B (flag + through controller) is nav_pass_through:=true."""
    actions = run(made, result_dir=str(tmp_path / "result"))
    base = nav2_source()["controller_server"]["ros__parameters"]
    for used in nav2_files(actions).values():
        server = used["controller_server"]["ros__parameters"]
        assert server["controller_plugins"] == base["controller_plugins"]
        assert THROUGH not in server
    fsms = includes(actions, "pinky_fsm.launch.py")
    assert {launch_args(i)["through_controller_id"] for i in fsms} == {""}


def test_a_misspelt_pass_through_switch_stops_the_launch(made):
    with pytest.raises(ValueError, match="nav_pass_through"):
        run(made, nav_pass_through="ture")


def test_the_pinky_fsm_launch_takes_the_through_controller_id():
    need("pinky_fsm")
    assert "through_controller_id" in declared("pinky_fsm.launch.py")
