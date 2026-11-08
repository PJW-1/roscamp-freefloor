"""fleet_check node: records the three-pinky driving check (fleet_check.FleetCheck) during a run.

    ros2 run fleet_gz_sim fleet_check --ros-args -p output:=gz_result/fleet_check.json

gz.launch.py starts it with the fleet (fleet_check:=true, the default) and writes
<result_dir>/fleet_check.json. It reads /<robot>/sim/truth (Gazebo true pose), /<robot>/state and
/fleet/zones, rewrites the file every period_s wall seconds and once more at shutdown, and logs
a one-line summary every log_every_s.
"""
from __future__ import annotations

import json
import os
import time
from pathlib import Path

import rclpy
from fleet_core.common.contract import load_contract, to_ros_qos
from fleet_core.traffic.lane_map import load_lane_map
from fleet_interfaces.msg import RobotState, ZoneStateArray
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from fleet_gz_sim.fleet_check import MARGIN_M, FleetCheck


class FleetCheckNode(Node):
    def __init__(self):
        super().__init__("fleet_check")
        robots = list(self.declare_parameter("robots", ["pinky1", "pinky2", "pinky3"]).value)
        lane_map = self.declare_parameter("lane_map", "").value
        contract_file = self.declare_parameter("contract_file", "").value
        self.output = Path(self.declare_parameter("output", "fleet_check.json").value)
        period = float(self.declare_parameter("period_s", 10.0).value)
        self.log_every = float(self.declare_parameter("log_every_s", 60.0).value)
        margin = float(self.declare_parameter("margin_m", MARGIN_M).value)
        if not contract_file:
            from ament_index_python.packages import get_package_share_directory
            contract_file = get_package_share_directory("fleet_interfaces") + "/contract.yaml"
        contract = load_contract(contract_file)
        self.check = FleetCheck(load_lane_map(lane_map or None), robots, margin_m=margin)
        for robot in robots:
            self.create_subscription(Odometry, f"/{robot}/sim/truth",
                                     lambda msg, r=robot: self._on_truth(r, msg),
                                     qos_profile_sensor_data)
            self.create_subscription(RobotState, contract.name("robot_state", pinky=robot),
                                     lambda msg, r=robot: self.check.on_state(r, msg.state),
                                     to_ros_qos(contract.qos("robot_state")))
        self.create_subscription(ZoneStateArray, contract.name("zones"), self._on_zones,
                                 to_ros_qos(contract.qos("zones")))
        self.last_log = time.monotonic()
        self.create_timer(period, self.write, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f"fleet check for {robots} -> {self.output}")

    def _on_truth(self, robot: str, msg: Odometry) -> None:
        p, v, w = msg.pose.pose.position, msg.twist.twist.linear, msg.twist.twist.angular
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.check.on_truth(robot, t, p.x, p.y, (v.x * v.x + v.y * v.y) ** 0.5, w.z,
                            wall=time.monotonic())

    def _on_zones(self, msg: ZoneStateArray) -> None:
        self.check.on_zones({z.zone_id: z.holder for z in msg.zones})

    def write(self) -> dict:
        summary = self.check.summary()
        self.output.parent.mkdir(parents=True, exist_ok=True)
        part = self.output.with_name(self.output.name + ".part")
        part.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        os.replace(part, self.output)
        if time.monotonic() - self.last_log >= self.log_every:
            self.last_log = time.monotonic()
            self.get_logger().info(
                f"min distance {summary['min_distance_m']} m {summary['min_distance_pair']}, "
                f"stops {summary['stops']} ({summary['stops_while_moving']} while MOVING), "
                f"outside held zones {summary['outside_held_samples']}/"
                f"{summary['zone_samples']}, sim {summary['sim_span_s']} s, "
                f"rtf {summary['real_time_factor']}")
        return summary


def main(args=None):
    rclpy.init(args=args)
    node = FleetCheckNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        summary = node.write()
        print(json.dumps(summary), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
