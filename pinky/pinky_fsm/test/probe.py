"""관제 쪽 흉내: move_to·dock 클라이언트, 상태 구독, heartbeat 발행.

rclpy가 있을 때만 import한다.
"""
from __future__ import annotations

import math
import time

from conftest import CONTRACT, wait_until
from fleet_interfaces.action import Dock, MoveTo
from fleet_interfaces.msg import RobotState
from geometry_msgs.msg import Pose, Pose2D
from pinky_fsm.contract import load_contract, to_ros_qos
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Header

C = load_contract(CONTRACT)
WAYPOINTS = {"P-3": (0.35, 1.55), "P-4": (0.35, 1.95), "P-5": (0.80, 2.05),
             "P-6": (1.20, 2.05), "N": (2.00, 1.70), "S": (2.00, 0.60)}


def lane_goal(ids, path=True, target_id=None):
    """관제처럼 만든 목표: 지점 좌표의 폴리라인, theta는 마지막 레인의 방향."""
    pts = [WAYPOINTS[i] for i in ids]
    (ax, ay), (bx, by) = pts[-2], pts[-1]
    goal = MoveTo.Goal(target_id=target_id or ids[-1],
                       target_pose=Pose2D(x=bx, y=by, theta=math.atan2(by - ay, bx - ax)))
    if path:
        for x, y in pts:
            p = Pose()
            p.position.x, p.position.y = x, y
            goal.path.append(p)
    return goal


class Probe(Node):
    def __init__(self, robot="pinky1", beat_hz=0.0):
        super().__init__("probe")
        self.states: list[tuple[float, RobotState]] = []
        self.create_subscription(RobotState, C.name("robot_state", pinky=robot),
                                 lambda m: self.states.append((time.monotonic(), m)),
                                 to_ros_qos(C.qos("robot_state")))
        self.move = ActionClient(self, MoveTo, C.name("move_to", pinky=robot))
        self.dock = ActionClient(self, Dock, C.name("dock", pinky=robot))
        self._beat = self.create_publisher(Header, C.name("heartbeat"),
                                           to_ros_qos(C.qos("heartbeat")))
        self.beating = beat_hz > 0
        if beat_hz > 0:
            self.create_timer(1.0 / beat_hz, self._send_beat)

    def _send_beat(self):
        if self.beating:
            self._beat.publish(Header(frame_id="fleet_manager"))

    def ready(self, timeout=10.0):
        return self.move.wait_for_server(timeout) and self.dock.wait_for_server(timeout)

    def send(self, goal, client=None):
        fut = (client or self.move).send_goal_async(goal)
        assert wait_until(fut.done, 5.0), "goal not answered"
        handle = fut.result()
        assert handle.accepted
        return handle

    @staticmethod
    def result(handle, timeout=20.0):
        fut = handle.get_result_async()
        assert wait_until(fut.done, timeout), "no result"
        return fut.result()

    def last_state(self):
        return self.states[-1][1] if self.states else None

    def states_since(self, t0):
        return [m.state for t, m in self.states if t >= t0]
