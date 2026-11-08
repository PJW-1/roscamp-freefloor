"""관제 화면 노드(시뮬레이션 전용): 함대 전체를 RViz 하나에 마커로 그린다.

/fleet/robots, /fleet/zones, /central/events와 로봇마다 자세(기본은 Gazebo 실제 위치
/<robot>/sim/truth, map 기준)를 받아 /fleet_view/markers에 5 Hz로 낸다.
로봇 TF는 네임스페이스 안에 있으므로 전역 /tf_static에 map -> fleet_view를 내서
네임스페이스 없는 RViz에도 map 프레임이 있게 한다.
"""
import math

import rclpy
from ament_index_python.packages import get_package_share_directory
from fleet_core.common.contract import load_contract, to_ros_qos
from fleet_core.traffic.lane_map import load_lane_map
from fleet_interfaces.msg import FleetEvent, RobotStatusArray, ZoneStateArray
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from fleet_gz_sim.fleet_view import FleetView

FRAME = "map"
PERIOD_S = 0.2


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def to_marker(spec, stamp) -> Marker:
    m = Marker()
    m.header.frame_id = FRAME
    m.header.stamp = stamp
    m.ns, m.id = spec.ns, spec.id
    m.type = Marker.TEXT_VIEW_FACING if spec.kind == "TEXT" else getattr(Marker, spec.kind)
    m.action = Marker.ADD
    x, y, z, yaw = spec.pose
    m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
    m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    m.scale.x, m.scale.y, m.scale.z = (float(v) for v in spec.scale)
    m.color.r, m.color.g, m.color.b, m.color.a = (float(v) for v in spec.color)
    m.points = [Point(x=float(px), y=float(py)) for px, py in spec.points]
    m.text = spec.text
    return m


class FleetViewNode(Node):
    def __init__(self):
        super().__init__("fleet_view")
        self.declare_parameter("robots", ["pinky1", "pinky2", "pinky3"])
        self.declare_parameter("lane_map", "")
        self.declare_parameter("pose_source", "truth")
        self.declare_parameter("contract_file", "")
        robots = list(self.get_parameter("robots").value)
        self.pose_source = self.get_parameter("pose_source").value
        if self.pose_source not in ("truth", "fleet"):
            raise ValueError(f"pose_source must be truth or fleet, not {self.pose_source}")
        contract = load_contract(self.get_parameter("contract_file").value or (
            get_package_share_directory("fleet_interfaces") + "/contract.yaml"))
        self.view = FleetView(load_lane_map(self.get_parameter("lane_map").value or None))

        def sub(msg_type, key, callback):
            self.create_subscription(msg_type, contract.name(key), callback,
                                     to_ros_qos(contract.qos(key)))

        sub(RobotStatusArray, "robots", self._on_robots)
        sub(ZoneStateArray, "zones", lambda msg: self.view.set_zones(msg.zones))
        sub(FleetEvent, "events",
            lambda msg: self.view.on_event(msg.code, msg.source, msg.detail))
        if self.pose_source == "truth":
            for robot in robots:
                self.create_subscription(
                    Odometry, f"/{robot}/sim/truth",
                    lambda msg, robot=robot: self._on_truth(robot, msg),
                    qos_profile_sensor_data)
        self.pub = self.create_publisher(MarkerArray, "/fleet_view/markers", 5)
        self.tf_static = StaticTransformBroadcaster(self)
        frame = TransformStamped()
        frame.header.stamp = self.get_clock().now().to_msg()
        frame.header.frame_id = FRAME
        frame.child_frame_id = "fleet_view"
        frame.transform.rotation.w = 1.0
        self.tf_static.sendTransform(frame)
        self.create_timer(PERIOD_S, self._draw)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_robots(self, msg):
        self.view.set_statuses(msg.robots)
        if self.pose_source == "fleet":
            now = self._now()
            for r in msg.robots:
                self.view.set_pose(r.robot_id, r.x, r.y, r.yaw, now)

    def _on_truth(self, robot, msg):
        p = msg.pose.pose
        self.view.set_pose(robot, p.position.x, p.position.y, _yaw(p.orientation), self._now())

    def _draw(self):
        stamp = self.get_clock().now().to_msg()
        clear = Marker(action=Marker.DELETEALL)
        clear.header.frame_id = FRAME
        out = MarkerArray(markers=[clear])
        out.markers += [to_marker(spec, stamp) for spec in self.view.specs(self._now())]
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FleetViewNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
