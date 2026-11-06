"""FakeVision: publishes what one workspace arm's camera sees, from the true stock."""
from __future__ import annotations

import json
import threading

import rclpy
from geometry_msgs.msg import Pose2D
from gg_fleet_core.common.contract import to_ros_qos
from gg_interfaces.msg import ItemArray
from rclpy.node import Node
from std_msgs.msg import String

from gg_sim.common import (
    STOCK_TOPIC,
    load_contract_param,
    load_lane_map_param,
    load_scenario_param,
    node_name,
    station_with_action,
)
from gg_sim.fake_workcell_node import STOCK_QOS

REPUBLISH_S = 1.0
ITEM_SPACING_M = 0.03  # items are spread out only so that no two poses are the same


class FakeVisionNode(Node):
    def __init__(self, parameter_overrides=None, **kwargs):
        super().__init__(node_name("fake_vision", "arm_id", parameter_overrides),
                         parameter_overrides=parameter_overrides, **kwargs)
        self.declare_parameter("arm_id", "arm1")
        self._arm = self.get_parameter("arm_id").value
        contract = load_contract_param(self)
        site = load_lane_map_param(self)
        scenario = load_scenario_param(self, site)
        station = station_with_action(site, "fill")
        if self._arm not in station.arms:
            raise ValueError(f"{self._arm} has no camera on an item area")
        self._pose = site.waypoint_pose(station.dock)
        self._stock = dict(scenario.stock.get(self._arm, {}))
        self._lock = threading.Lock()
        self._pub = self.create_publisher(
            ItemArray, contract.name("vision_items", arm=self._arm),
            to_ros_qos(contract.qos("vision_items")))
        self.create_subscription(
            String, STOCK_TOPIC.format(arm=self._arm), self._on_stock, STOCK_QOS)
        self.create_timer(REPUBLISH_S, self._publish)
        self._publish()

    def _on_stock(self, msg: String) -> None:
        with self._lock:
            self._stock = json.loads(msg.data)
        self._publish()

    def _publish(self) -> None:
        with self._lock:
            kinds = [kind for kind, count in sorted(self._stock.items()) for _ in range(count)]
        x, y = self._pose
        self._pub.publish(ItemArray(
            arm_id=self._arm, kinds=kinds,
            poses=[Pose2D(x=x + i * ITEM_SPACING_M, y=y, theta=0.0) for i in range(len(kinds))],
            stamp=self.get_clock().now().to_msg()))


def main(args=None):
    rclpy.init(args=args)
    node = FakeVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:  # SIGINT can land inside a message take; a real error has ROS still up
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
