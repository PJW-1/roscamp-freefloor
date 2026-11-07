"""시험용 가짜 Nav2 controller와 클라이언트. rclpy가 있을 때만 import한다.

FakeNav는 follow_path(FollowPath)를 받아 경로를 따라 TF(map → odom → base_footprint)와
odom을 낸다. Nav2 controller처럼 새 목표가 오면 옛 목표를 ABORTED로 끝내고 서지 않고
새 경로로 넘어간다(nav2_util simple_action_server의 accept_pending_goal). 앞에 물체가 있으면
서고, 1초 뒤 FollowPath를 실패로 끝낸다(RPP 충돌 검사 + failure_tolerance 1.0).
"""
from __future__ import annotations

import math
import threading
import time

from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

STEP_HZ = 50.0
STOP_GAP_M = 0.25  # 물체가 경로 앞 이 거리 안이면 선다
FAIL_AFTER_S = 1.0


class FakeNav(Node):
    def __init__(self, start=(0.35, 1.55, math.pi / 2), speed=0.5, publish_tf=True):
        super().__init__("fake_nav")
        self.pose = list(start)
        self.speed = speed
        self.publish_tf = publish_tf
        self.lock = threading.RLock()
        self.current = None  # 실행 중인 목표 손잡이
        self.path: list[tuple[float, float, float]] = []
        self.index = 0
        self.v = 0.0
        self.goals = []  # (시각, 점 수)
        self.cancels = []  # 시각
        self.speed_log = []  # (시각, 선속도)
        self.obstacle: tuple[float, float] | None = None
        self._blocked_since: float | None = None
        self._outcome: dict[int, str] = {}
        self._tf = TransformBroadcaster(self)
        static = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = "map", "odom"
        t.transform.rotation.w = 1.0
        static.sendTransform(t)
        self._odom = self.create_publisher(Odometry, "odom", 10)
        self._grid = self.create_publisher(
            OccupancyGrid, "local_costmap/costmap",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE))
        group = ReentrantCallbackGroup()
        self._server = ActionServer(
            self, FollowPath, "follow_path", execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=self._on_cancel, handle_accepted_callback=self._accepted,
            callback_group=group)
        self.create_timer(1.0 / STEP_HZ, self._step, callback_group=group)
        self.create_timer(0.2, self._publish_grid, callback_group=group)

    # -- 액션 ------------------------------------------------------------------------------

    def _on_cancel(self, _handle):
        with self.lock:
            self.cancels.append(time.monotonic())
        return CancelResponse.ACCEPT

    def _accepted(self, handle):
        with self.lock:
            old = self.current
            if old is not None:
                self._outcome[id(old)] = "preempted"
            self.current = handle
            self.path = [(p.pose.position.x, p.pose.position.y,
                          2 * math.atan2(p.pose.orientation.z, p.pose.orientation.w))
                         for p in handle.request.path.poses]
            self.index = 0
            self.goals.append((time.monotonic(), len(self.path)))
            self._blocked_since = None
        handle.execute()

    def _execute(self, handle):
        while True:
            with self.lock:
                outcome = self._outcome.pop(id(handle), None)
                if handle.is_cancel_requested and outcome is None:
                    if self.current is handle:
                        self.current, self.v = None, 0.0
                    handle.canceled()
                    return FollowPath.Result()
            if outcome == "preempted":
                handle.abort()
                return FollowPath.Result()
            if outcome == "done":
                handle.succeed()
                return FollowPath.Result()
            if outcome == "failed":
                handle.abort()
                return FollowPath.Result(error_code=FollowPath.Result.NO_VALID_CONTROL,
                                         error_msg="fake collision ahead")
            time.sleep(0.005)

    # -- 움직임 ----------------------------------------------------------------------------

    def _step(self):
        now = time.monotonic()
        with self.lock:
            self.v = 0.0
            handle = self.current
            if handle is not None and self.path:
                if self._blocked():
                    if self._blocked_since is None:
                        self._blocked_since = now
                    elif now - self._blocked_since >= FAIL_AFTER_S:
                        self._outcome[id(handle)] = "failed"
                        self.current = None
                else:
                    self._blocked_since = None
                    self._advance(handle)
            self.speed_log.append((now, self.v))
            x, y, yaw, v = *self.pose, self.v
        self._publish_motion(x, y, yaw, v)

    def _advance(self, handle):
        budget = self.speed / STEP_HZ
        while budget > 0 and self.index < len(self.path):
            tx, ty, tyaw = self.path[self.index]
            d = math.hypot(tx - self.pose[0], ty - self.pose[1])
            if d <= budget:
                self.pose[:2] = [tx, ty]
                budget -= d
                self.index += 1
                if self.index == len(self.path):
                    self.pose[2] = tyaw
            else:
                f = budget / d
                self.pose[2] = math.atan2(ty - self.pose[1], tx - self.pose[0])
                self.pose[0] += f * (tx - self.pose[0])
                self.pose[1] += f * (ty - self.pose[1])
                budget = 0.0
            self.v = self.speed
        if self.index >= len(self.path):
            self._outcome[id(handle)] = "done"
            self.current = None

    def _blocked(self) -> bool:
        if self.obstacle is None:
            return False
        ox, oy = self.obstacle
        x, y, yaw = self.pose
        ahead = (ox - x) * math.cos(yaw) + (oy - y) * math.sin(yaw)
        side = abs(-(ox - x) * math.sin(yaw) + (oy - y) * math.cos(yaw))
        return 0.0 <= ahead <= STOP_GAP_M and side < 0.1

    def _publish_motion(self, x, y, yaw, v):
        stamp = self.get_clock().now().to_msg()
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, "odom", "base_footprint"
            t.transform.translation.x, t.transform.translation.y = x, y
            t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            self._tf.sendTransform(t)
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, "odom", "base_footprint"
        o.twist.twist.linear.x = v
        self._odom.publish(o)

    def _publish_grid(self):
        res, w, h = 0.025, 200, 120  # odom 틀의 (0,0)부터 5 m x 3 m
        data = [0] * (w * h)
        if self.obstacle is not None:
            ox, oy = self.obstacle
            for j in range(h):
                for i in range(w):
                    cx, cy = (i + 0.5) * res, (j + 0.5) * res
                    d = math.hypot(cx - ox, cy - oy)
                    if d < 0.05:
                        data[j * w + i] = 100
                    elif d < 0.15:
                        data[j * w + i] = 99
        g = OccupancyGrid()
        g.header.stamp, g.header.frame_id = self.get_clock().now().to_msg(), "odom"
        g.info.resolution, g.info.width, g.info.height = res, w, h
        g.info.origin.orientation.w = 1.0
        g.data = data
        self._grid.publish(g)

    def stopped_between(self, t0: float, t1: float) -> bool:
        with self.lock:
            return any(v == 0.0 for t, v in self.speed_log if t0 <= t <= t1)
