"""SimClock: publishes /clock at speed_factor times wall time. Must not use sim time itself.

The clock stays at zero for start_delay_s of wall time. At speed_factor 5 a node that takes two
seconds to start would otherwise miss ten scenario seconds: the fleet manager reports robots as
not reporting and the judge misses the first moves.
"""
from __future__ import annotations

import time

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


class SimClockNode(Node):
    def __init__(self, parameter_overrides=None, **kwargs):
        super().__init__("sim_clock", parameter_overrides=parameter_overrides, **kwargs)
        self.declare_parameter("speed_factor", 1.0)
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("start_delay_s", 0.0)
        self._factor = float(self.get_parameter("speed_factor").value)
        self._pub = self.create_publisher(Clock, "/clock", 10)
        self._start = time.monotonic() + float(self.get_parameter("start_delay_s").value)
        self._publish()
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._publish)

    def _publish(self) -> None:
        sim = max(0.0, time.monotonic() - self._start) * self._factor
        sec = int(sim)
        self._pub.publish(Clock(clock=Time(sec=sec, nanosec=int((sim - sec) * 1e9))))


def main(args=None):
    rclpy.init(args=args)
    node = SimClockNode()
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
