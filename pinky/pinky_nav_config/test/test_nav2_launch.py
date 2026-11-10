"""nav2.launch.py가 use_composition에 따라 노드 아홉을 프로세스 아홉 또는 컨테이너 하나로
띄우는지 본다.

프로세스는 띄우지 않는다. launch가 하는 대로 조건과 namespace 범위를 따라 액션을 훑고,
컨테이너에 보낼 LoadNode 요청은 launch_ros가 만드는 그대로 만든다. ROS 2(launch_ros)와
설치된 pinky_nav_config가 필요하다(colcon test).
"""
import importlib.util
from pathlib import Path

import pytest

launch = pytest.importorskip("launch")
launch_ros = pytest.importorskip("launch_ros")
from launch_ros.actions import ComposableNodeContainer, Node  # noqa: E402
from launch_ros.actions.load_composable_nodes import get_composable_node_load_request  # noqa: E402
from rclpy.parameter import parameter_value_to_python  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
NAMES = ["map_server", "map_server_fine", "amcl", "controller_server", "planner_server",
         "behavior_server", "bt_navigator", "lifecycle_manager_localization",
         "lifecycle_manager_navigation"]
PLUGINS = {
    "map_server": "nav2_map_server::MapServer",
    "map_server_fine": "nav2_map_server::MapServer",
    "amcl": "nav2_amcl::AmclNode",
    "controller_server": "nav2_controller::ControllerServer",
    "planner_server": "nav2_planner::PlannerServer",
    "behavior_server": "behavior_server::BehaviorServer",
    "bt_navigator": "nav2_bt_navigator::BtNavigator",
    "lifecycle_manager_localization": "nav2_lifecycle_manager::LifecycleManager",
    "lifecycle_manager_navigation": "nav2_lifecycle_manager::LifecycleManager",
}


def _launch_description():
    spec = importlib.util.spec_from_file_location("nav2_launch", PKG / "launch" / "nav2.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _walk(entities, context, found):
    """조건을 따지며 액션을 방문한다. Node는 실행하지 않고 그때의 context로 펼친다."""
    for entity in entities:
        condition = getattr(entity, "condition", None)
        if condition is not None and not condition.evaluate(context):
            continue
        if isinstance(entity, Node):
            entity._perform_substitutions(context)
            requests = []
            if isinstance(entity, ComposableNodeContainer):
                descriptions = entity._ComposableNodeContainer__composable_node_descriptions
                requests = [get_composable_node_load_request(d, context) for d in descriptions]
            found.append((entity, requests))
            continue
        _walk(entity.visit(context) or [], context, found)


def _expand(use_composition, use_sim_time="true"):
    context = launch.LaunchContext()
    context.launch_configurations.update({
        "use_composition": use_composition, "use_sim_time": use_sim_time,
        "map": "/maps/coarse.yaml", "map_fine": "/maps/fine.yaml",
        "initial_x": "0.35", "initial_y": "1.55", "initial_yaw": "1.5708",
    })
    found = []
    _walk(_launch_description().entities, context, found)
    return context, found


def _params(request):
    return {p.name: parameter_value_to_python(p.value) for p in request.parameters}


def test_without_composition_nine_nodes_run_as_today():
    _, found = _expand("false")
    nodes = [node for node, _ in found]
    assert not any(isinstance(n, ComposableNodeContainer) for n in nodes)
    assert [n.node_name for n in nodes] == [f"/pinky1/{name}" for name in NAMES]
    executables = [n.node_executable for n in nodes]
    assert executables == ["map_server", "map_server", "amcl", "controller_server",
                           "planner_server", "behavior_server", "bt_navigator",
                           "lifecycle_manager", "lifecycle_manager"]
    for node in nodes[:7]:
        assert node.expanded_remapping_rules == [("/tf", "tf"), ("/tf_static", "tf_static")]


def test_with_composition_one_container_loads_the_nine_nodes():
    _, found = _expand("true")
    assert len(found) == 1
    container, requests = found[0]
    assert isinstance(container, ComposableNodeContainer)
    assert container.node_package == "rclcpp_components"
    assert container.node_executable == "component_container_isolated"
    assert container.node_name == "/pinky1/nav2_container"
    assert [r.node_name for r in requests] == NAMES
    assert {r.node_name: r.plugin_name for r in requests} == PLUGINS
    for request in requests:
        assert request.node_namespace == "/pinky1"
        assert _params(request)["use_sim_time"] is True
    for request in requests[:7]:  # lifecycle manager는 프로세스로 띄울 때처럼 TF를 쓰지 않는다
        assert list(request.remap_rules) == ["/tf:=tf", "/tf_static:=tf_static"]
    # 컨테이너 안에서 만드는 하위 노드(costmap)를 위해 컨테이너 프로세스에도 같은 remap을 준다
    assert container.expanded_remapping_rules == [("/tf", "tf"), ("/tf_static", "tf_static")]


def test_composed_nodes_get_the_same_parameters_as_the_processes():
    _, [(_, requests)] = _expand("true")
    params = {r.node_name: _params(r) for r in requests}
    # 파라미터 파일의 namespace 뿌리 키(pinky1:)가 노드마다 맞아 들어가야 한다
    assert params["controller_server"]["controller_plugins"] == [
        "FollowPath", "FollowPathThrough", "FollowPathReverse"]
    assert params["amcl"]["map_topic"] == "map_fine"
    assert params["map_server"]["yaml_filename"] == "/maps/coarse.yaml"
    assert params["map_server_fine"]["yaml_filename"] == "/maps/fine.yaml"
    assert params["amcl"]["set_initial_pose"] is True
    assert (params["amcl"]["initial_pose.x"], params["amcl"]["initial_pose.y"],
            params["amcl"]["initial_pose.yaw"]) == (0.35, 1.55, 1.5708)
    assert params["bt_navigator"]["default_nav_to_pose_bt_xml"].endswith(
        "plan_once_no_recovery.xml")
    loc, nav = params["lifecycle_manager_localization"], params["lifecycle_manager_navigation"]
    assert loc["node_names"] == ["map_server", "map_server_fine", "amcl"]
    assert nav["node_names"] == ["controller_server", "planner_server", "behavior_server",
                                 "bt_navigator"]
    assert loc["autostart"] is True and nav["autostart"] is True


def test_composed_use_sim_time_follows_the_argument():
    _, [(_, requests)] = _expand("true", use_sim_time="false")
    assert all(_params(r)["use_sim_time"] is False for r in requests)
