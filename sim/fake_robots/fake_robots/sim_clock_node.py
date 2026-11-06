"""SimClock: /clock을 실제 시간의 speed_factor배로 발행한다. 자신은 시뮬 시간을 쓰면 안 된다.

시계는 실제 시간으로 start_delay_s 동안 0에 머문다. speed_factor 5에서 노드 시작에 2초가
걸리면 시나리오 10초를 놓친다. 플릿 매니저가 로봇을 무응답으로 보고하고 심판이
첫 이동을 놓친다.
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
    except Exception:  # SIGINT가 메시지 수신 중에 떨어질 수 있다. 진짜 오류면 ROS가 아직 살아 있다
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
