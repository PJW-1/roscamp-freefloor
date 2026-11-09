"""시험용 가짜 Nav2 controller와 클라이언트. rclpy가 있을 때만 import한다.

FakeNav는 follow_path(FollowPath)를 받아 경로를 따라 TF(map → odom → base_footprint)와
odom을 낸다. Nav2 controller처럼 새 목표가 오면 옛 목표를 ABORTED로 끝내고 서지 않고
새 경로로 넘어간다(nav2_util simple_action_server의 accept_pending_goal). 앞에 물체가 있으면
서고, 1초 뒤 FollowPath를 실패로 끝낸다(RPP 충돌 검사 + failure_tolerance 1.0).

behavior_server의 backup(BackUp)도 흉내 낸다: 바라보는 방향의 반대로 곧게 물러나고, 뒤에
벽(wall_behind_m)이 있으면 거기서 서고 COLLISION_AHEAD로 끝낸다. local costmap 비우기
(ClearEntireCostmap)는 부른 시각만 적는다.
"""
from __future__ import annotations

import math
import threading
import time

from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import BackUp, FollowPath
from nav2_msgs.msg import SpeedLimit
from nav2_msgs.srv import ClearEntireCostmap
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
        self.controller_ids = []  # FollowPath controller_id of each accepted goal
        self.cancels = []  # 시각
        self.speed_log = []  # (시각, 선속도)
        self.obstacle: tuple[float, float] | None = None
        self.backups = []  # (시각, 거리, 속도)
        self.backup_ends = []  # (outcome, error_code)
        self.clears = []  # 시각
        self.speed_limits = []  # 받은 speed_limit(m/s, 0이면 제한 없음). 주행에는 쓰지 않는다
        self.wall_behind_m: float | None = None  # 물러날 때 이만큼 가면 뒤에 벽
        self._backing: dict | None = None
        self._backing_of: dict[int, dict] = {}
        self._stopping = False
        self._blocked_since: float | None = None
        self._outcome: dict[int, str] = {}
        self._tf = TransformBroadcaster(self)
        self.create_subscription(SpeedLimit, "speed_limit",
                                 lambda m: self.speed_limits.append(m.speed_limit), 10)
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
        self._backup_server = ActionServer(
            self, BackUp, "backup", execute_callback=self._execute_backup,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _h: CancelResponse.ACCEPT,
            handle_accepted_callback=self._accepted_backup, callback_group=group)
        self.create_service(ClearEntireCostmap, "local_costmap/clear_entirely_local_costmap",
                            self._on_clear, callback_group=group)
        self.create_timer(1.0 / STEP_HZ, self._step, callback_group=group)
        self.create_timer(0.2, self._publish_grid, callback_group=group)

    def stop(self) -> None:
        """Harness가 실행기를 끄기 전에 부른다. 도는 목표를 모두 끝내야 실행기가 멈춘다."""
        with self.lock:
            self._stopping = True

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
            self.controller_ids.append(handle.request.controller_id)
            self._blocked_since = None
        handle.execute()

    def _execute(self, handle):
        while True:
            with self.lock:
                outcome = self._outcome.pop(id(handle), None)
                if self._stopping and outcome is None:
                    outcome = "preempted"
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

    def _on_clear(self, _req, res):
        with self.lock:
            self.clears.append(time.monotonic())
        return res

    def _accepted_backup(self, handle):
        g = handle.request
        with self.lock:
            if self._backing is not None:
                self._backing["outcome"] = ("preempted", 0)
            self.backups.append((time.monotonic(), abs(g.target.x), abs(g.speed)))
            self._backing = {"handle": handle, "left": abs(g.target.x), "travelled": 0.0,
                             "speed": abs(g.speed), "outcome": None}
            self._backing_of[id(handle)] = self._backing
        handle.execute()

    def _execute_backup(self, handle):
        with self.lock:
            b = self._backing_of.pop(id(handle))
        while True:
            with self.lock:
                outcome = b["outcome"]
                if outcome is None and handle.is_cancel_requested:
                    outcome = ("canceled", 0)
                if outcome is None and self._stopping:
                    outcome = ("preempted", 0)
                if outcome is not None:
                    if self._backing is b:
                        self._backing, self.v = None, 0.0
                    self.backup_ends.append(outcome)
                travelled = b["travelled"]
            if outcome is None:
                fb = BackUp.Feedback()
                fb.distance_traveled = float(travelled)
                handle.publish_feedback(fb)
                time.sleep(0.02)
                continue
            kind, code = outcome
            res = BackUp.Result(error_code=code)
            if kind == "done":
                handle.succeed()
            elif kind == "canceled":
                handle.canceled()
            else:
                handle.abort()
            return res

    def _back_up(self, b):
        """바라보는 방향의 반대로 한 걸음 물러난다."""
        if b["outcome"] is not None:
            return
        wall = self.wall_behind_m
        if wall is not None and b["travelled"] >= wall - 1e-9:
            b["outcome"] = ("collision", BackUp.Result.COLLISION_AHEAD)
            return
        d = min(b["speed"] / STEP_HZ, b["left"])
        if wall is not None:
            d = min(d, wall - b["travelled"])
        x, y, yaw = self.pose
        self.pose[0], self.pose[1] = x - d * math.cos(yaw), y - d * math.sin(yaw)
        b["left"] -= d
        b["travelled"] += d
        self.v = -b["speed"]
        if b["left"] <= 1e-9:
            b["outcome"] = ("done", 0)

    # -- 움직임 ----------------------------------------------------------------------------

    def _step(self):
        now = time.monotonic()
        with self.lock:
            self.v = 0.0
            handle = self.current
            if self._backing is not None:
                self._back_up(self._backing)  # BackUp이 cmd_vel을 쥔다
                self._blocked_since = None
            elif handle is not None and self.path:
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
