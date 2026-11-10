"""sim_stand_in의 시뮬 배터리: Gazebo 시계와 실제 위치로 /{pinky}/sim/battery를 낸다."""
import threading
import time

import pytest
import yaml

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("fake_robots")
pytest.importorskip("sensor_msgs.msg")


@pytest.fixture
def ros():
    rclpy.init()
    yield rclpy
    rclpy.try_shutdown()


def scenario_file(tmp_path, faults) -> str:
    data = {"seed": 1, "timeout_s": 100,
            "stock": {"arm1": {"A": 3}, "arm2": {"C": 3}},
            "orders": [{"at_s": 0, "id": "O-1", "items": {"A": 1}}],
            "robots": {"pinky1": {"start": "P-6"}, "pinky2": {"start": "P-5"}},
            "faults": faults, "expect": {"orders_shipped_min": 1}}
    path = tmp_path / "s.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return str(path)


def stand_in(**params):
    from fleet_gz_sim.stand_in_node import SimStandIn
    from rclpy.parameter import Parameter
    return SimStandIn(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])


def test_without_a_scenario_file_there_is_no_battery(ros):
    node = stand_in(use_sim_time=True)
    try:
        assert node._batteries == {}
    finally:
        node.destroy_node()


def test_the_battery_follows_gazebo_time_and_charges_at_the_parking_spot(ros, tmp_path):
    from fleet_core.traffic.lane_map import load_lane_map
    from nav_msgs.msg import Odometry
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import BatteryState

    path = scenario_file(tmp_path, [{"type": "low_battery", "robot": "pinky1", "at_s": 1.0,
                                     "duration_s": 2.0}])
    node = stand_in(use_sim_time=True, robots=["pinky1", "pinky2"], scenario_file=path)
    probe = rclpy.create_node("battery_probe")
    seen = {"pinky1": [], "pinky2": []}
    for robot in seen:
        probe.create_subscription(BatteryState, f"/{robot}/sim/battery",
                                  lambda m, r=robot: seen[r].append(m.percentage),
                                  qos_profile_sensor_data)
    clock = probe.create_publisher(Clock, "/clock", 10)
    truth = {r: probe.create_publisher(Odometry, f"/{r}/sim/truth", 10) for r in seen}
    site = load_lane_map()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(probe)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        sim_t = 0.0
        while sim_t < 5.0:  # Gazebo 시간 5 s를 벽시계 약 1 s에
            sim_t += 0.05
            stamp = Clock()
            stamp.clock.sec, stamp.clock.nanosec = int(sim_t), int((sim_t % 1.0) * 1e9)
            clock.publish(stamp)
            for robot, start in (("pinky1", "P-6"), ("pinky2", "P-5")):
                odom = Odometry()
                odom.header.frame_id = "map"
                odom.pose.pose.position.x, odom.pose.pose.position.y = site.waypoint_pose(start)
                odom.pose.pose.orientation.w = 1.0
                truth[robot].publish(odom)
            time.sleep(0.01)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not (seen["pinky1"] and
                                                   seen["pinky1"][-1] > 0.85):
            time.sleep(0.05)
    finally:
        executor.shutdown()
        thread.join(2.0)
        probe.destroy_node()
        node.destroy_node()
    levels = seen["pinky1"]
    assert levels and levels[0] == pytest.approx(0.9)
    assert min(levels) == pytest.approx(0.1, abs=0.05)
    low = levels.index(min(levels))
    assert levels[low:] == sorted(levels[low:]) and levels[-1] == pytest.approx(0.9)
    assert seen["pinky2"] and all(v == pytest.approx(0.9) for v in seen["pinky2"])  # 고장 없음
