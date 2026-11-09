"""Nav2 설정이 핑키 약속을 지키는지 본다. ROS 없이 돈다.

근거: Drive 핑키_FSM_Nav2_구현_안내_2026-11-07(3절), 핑키_제어파트_전달사항_2026-11-06(R4, R12~R15).
"""
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parents[1]
PARAMS = yaml.safe_load((PKG / "params" / "nav2_params.yaml").read_text(encoding="utf-8"))
BT = ET.parse(PKG / "bt" / "plan_once_no_recovery.xml")


def params(node, sub=None):
    section = PARAMS[node] if sub is None else PARAMS[node][sub]
    return section["ros__parameters"]


def test_controller_uses_rpp_with_the_agreed_start_values():
    c = params("controller_server")
    assert c["controller_plugins"] == ["FollowPath", "FollowPathThrough"]
    assert c["failure_tolerance"] == 1.0
    rpp = c["FollowPath"]
    assert rpp["plugin"] == "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
    assert rpp["rotate_to_heading_min_angle"] == 0.25  # R12: 올리지 않는다
    assert rpp["use_rotate_to_heading"] is True and rpp["allow_reversing"] is False
    assert (rpp["lookahead_dist"], rpp["min_lookahead_dist"], rpp["max_lookahead_dist"]) == (
        0.15, 0.10, 0.25)
    assert rpp["desired_linear_vel"] == 0.25
    assert rpp["use_collision_detection"] is True
    assert rpp["max_allowed_time_to_collision_up_to_carrot"] == 0.6


def test_the_through_controller_is_followpath_without_the_approach_slow_down():
    """pinky_fsm sends paths that end at a lock boundary (MoveTo.final_goal false) to
    FollowPathThrough. Jazzy RPP always applies the approach constraint, so only a short
    approach_velocity_scaling_dist turns it off; every other key must match FollowPath."""
    c = params("controller_server")
    normal, through = c["FollowPath"], c["FollowPathThrough"]
    assert through["approach_velocity_scaling_dist"] == 0.05
    assert normal["approach_velocity_scaling_dist"] == 0.6
    assert set(through) == set(normal)
    assert {k: v for k, v in through.items() if k != "approach_velocity_scaling_dist"} == {
        k: v for k, v in normal.items() if k != "approach_velocity_scaling_dist"}


def test_no_obstacle_beside_the_path_slows_the_robot():
    """Jazzy RPP scales the speed by cost_scaling_gain * d / cost_scaling_dist when d, the
    distance it reads from the cost under the robot, is below cost_scaling_dist, and d is at
    least the inscribed radius (half the 12 cm footprint) until the robot touches. With 0.6 m
    a box within the 0.15 m inflation pinned the robot to the 0.05 m/s floor in every arena gap
    (user, 2026-11-10): the scaling may no longer cut the speed short of a touch."""
    c = params("controller_server")
    footprint = yaml.safe_load(params("local_costmap", "local_costmap")["footprint"])
    inscribed = min(max(abs(x) for x, _ in footprint), max(abs(y) for _, y in footprint))
    for name in ("FollowPath", "FollowPathThrough"):
        rpp = c[name]
        assert rpp["use_cost_regulated_linear_velocity_scaling"] is True
        assert rpp["cost_scaling_dist"] == 0.05
        assert rpp["cost_scaling_gain"] * inscribed / rpp["cost_scaling_dist"] >= 1.0, name


def test_goal_checker_is_tighter_than_the_arrival_tolerance():
    c = params("controller_server")
    assert c["goal_checker_plugins"] == ["general_goal_checker"]
    goal = c["general_goal_checker"]
    assert goal["xy_goal_tolerance"] == 0.05 and goal["xy_goal_tolerance"] < 0.15  # R4
    assert goal["yaw_goal_tolerance"] == 0.12
    assert c["progress_checker_plugins"] == ["progress_checker"]
    assert c["progress_checker"]["required_movement_radius"] == 0.05


def test_speed_limit_topic_stays_for_version_2():
    assert params("controller_server")["speed_limit_topic"] == "speed_limit"


def test_local_costmap_is_fine_and_sent_whole_for_blocked():
    local = params("local_costmap", "local_costmap")
    assert local["resolution"] == 0.025
    assert local["always_send_full_costmap"] is True  # pinky_fsm이 칸 값으로 BLOCKED를 판단한다
    assert "obstacle_layer" in local["plugins"]


def test_costmap_topics_are_absolute_inside_the_robot_namespace():
    """Jazzy costmap 층은 상대 토픽을 costmap 노드 안으로 풀므로 launch가 절대 이름을 넣는다."""
    local = params("local_costmap", "local_costmap")
    glob = params("global_costmap", "global_costmap")
    assert local["obstacle_layer"]["scan"]["topic"] == "<robot_namespace>/scan"
    assert glob["obstacle_layer"]["scan"]["topic"] == "<robot_namespace>/scan"
    assert glob["static_layer"]["map_topic"] == "<robot_namespace>/map"


def test_global_costmap_has_the_lidar_layer_for_version_2_detours():
    glob = params("global_costmap", "global_costmap")
    assert glob["plugins"] == ["static_layer", "obstacle_layer", "inflation_layer"]


def test_planner_server_runs_for_compute_path_and_is_path_valid():
    assert params("planner_server")["planner_plugins"] == ["GridBased"]


def test_amcl_reads_the_fine_map():
    amcl = params("amcl")
    assert amcl["map_topic"] == "map_fine"  # R15
    assert amcl["sigma_hit"] == 0.05
    assert amcl["base_frame_id"] == "base_footprint" and amcl["scan_topic"] == "scan"


def test_backup_exists_only_as_a_behavior_server_plugin():
    b = params("behavior_server")
    assert b["behavior_plugins"] == ["backup"]
    assert b["backup"]["plugin"] == "nav2_behaviors::BackUp"


def test_bt_navigator_serves_only_navigate_to_pose():
    assert params("bt_navigator")["navigators"] == ["navigate_to_pose"]


def test_behavior_tree_plans_once_without_recovery():
    tags = [el.tag for el in BT.iter()]
    for banned in ("Spin", "BackUp", "Wait", "RecoveryNode", "RoundRobin",
                   "ClearEntireCostmap", "RateController", "DistanceController"):
        assert banned not in tags  # R14, 안내 5절
    [planner] = BT.getroot().iter("GoalUpdatedController")
    assert [child.tag for child in planner] == ["ComputePathToPose"]
    [follow] = BT.getroot().iter("FollowPath")
    assert follow.get("controller_id") == "FollowPath"
    assert follow.get("goal_checker_id") == "general_goal_checker"


def test_no_velocity_smoother_or_collision_monitor_between_controller_and_cmd_vel():
    assert "velocity_smoother" not in PARAMS and "collision_monitor" not in PARAMS
