"""시뮬 장애물 노드: 일정대로 Gazebo 레인 위에 상자를 놓고 치운다.

    ros2 run fleet_gz_sim sim_obstacles --ros-args -p use_sim_time:=true \\
        -p obstacles_file:=<share>/obstacles/roam.yaml

로봇이 clearance_m 안에 있으면 놓지 않고 기다린다(실제 위치 /<robot>/sim/truth로 본다).
A pose older than TRUTH_MAX_AGE_S counts as unknown, so the box waits.
gz service 호출은 작업 스레드 하나에서 차례로 돌린다. The worker checks the robots again right
before each spawn and reports every outcome back to the planner, which retries failed removes
and removes spawns that got no clear answer. 끝날 때 Gazebo에 있을 수 있는 상자를 모두 치운다.
놓인 상자는 /sim/obstacle_markers(map 틀)로 RViz에 보인다. 바뀔 때와 1초마다 낸다.
Gazebo 통신 설정(GZ_IP, GZ_PARTITION)은 launch에서 물려받는다.
"""
from __future__ import annotations

import math
import queue
import subprocess
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import rclpy
from ament_index_python.packages import get_package_share_directory
from fleet_core.traffic.lane_map import load_lane_map
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray

from fleet_gz_sim import obstacles

TICK_S = 0.2
CALL_LIMIT_S = 5.0  # gz service 자체 한도(3 s)에 여유를 둔다
TRUTH_MAX_AGE_S = 1.0  # wall seconds; truth arrives at about 30 Hz
SHUTDOWN_TIMEOUT_MS = 1000  # 끝낼 때는 launch가 기다려 주는 동안 다 치우도록 짧게
MARKER_PERIOD_S = 1.0
MARKER_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
MARKER_TYPES = {"cube": Marker.CUBE, "text": Marker.TEXT_VIEW_FACING}


class ObstacleNode(Node):
    def __init__(self):
        super().__init__("sim_obstacles")
        robots = self.declare_parameter("robots", ["pinky1", "pinky2", "pinky3"]).value
        path = self.declare_parameter("obstacles_file", "").value
        lane_map_path = self.declare_parameter("lane_map", "").value
        self.world = self.declare_parameter("world", "lane_arena").value
        self.clearance_m = obstacles.check_clearance(
            float(self.declare_parameter("clearance_m", 0.35).value))
        self.max_delay_s = float(self.declare_parameter("max_delay_s", 20.0).value)
        if not math.isfinite(self.max_delay_s) or self.max_delay_s < 0:  # same rule as Planner
            raise ValueError(f"max_delay_s must be a finite number >= 0, got {self.max_delay_s}")
        if not path:
            path = f"{get_package_share_directory('fleet_gz_sim')}/obstacles/roam.yaml"
        self.schedule = obstacles.load_schedule(path, load_lane_map(lane_map_path or None))
        self.robots = list(robots)
        self.truth: dict[str, tuple[tuple[float, float], float]] = {}  # (x, y), wall time
        for robot in self.robots:
            self.create_subscription(Odometry, f"/{robot}/sim/truth",
                                     lambda msg, r=robot: self._on_truth(r, msg),
                                     qos_profile_sensor_data)
        self.planner: obstacles.Planner | None = None
        self.told_waiting: set[int] = set()
        self.outcomes: queue.SimpleQueue = queue.SimpleQueue()  # from the worker to _tick
        # Gazebo에 놓인 상자(이름 → 장애물). 작업 스레드에서만 바꾸고, 표시는 잠금으로 읽는다
        self.live: dict[str, obstacles.Occurrence] = {}
        self.live_lock = threading.Lock()
        self.markers = self.create_publisher(MarkerArray, "/sim/obstacle_markers", MARKER_QOS)
        self.worker = ThreadPoolExecutor(max_workers=1)
        self.create_timer(TICK_S, self._tick)
        self.create_timer(MARKER_PERIOD_S, self._publish_markers)
        self.get_logger().info(f"{len(self.schedule.obstacles)} obstacles from {path}, "
                               f"repeat every {self.schedule.repeat_every_s} s")

    def _on_truth(self, robot: str, msg: Odometry):
        p = msg.pose.pose.position
        self.truth[robot] = ((p.x, p.y), time.monotonic())

    def _positions(self) -> list:
        return obstacles.fresh_positions(self.truth, self.robots, time.monotonic(),
                                         TRUTH_MAX_AGE_S)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        now = self._now()
        if now <= 0.0:
            return  # /clock를 아직 못 받았다
        if self.planner is None:
            self.planner = obstacles.Planner(self.schedule, now, self.max_delay_s)
        while not self.outcomes.empty():
            kind, occ, at, result = self.outcomes.get()
            if kind == "spawn":
                self.planner.spawned(occ, at, result)
            else:
                self.planner.removed(occ, at, result)
        step = self.planner.step(now, self._positions(), self.clearance_m)
        for occ in step.spawn:
            self.worker.submit(self._spawn, occ)
        for occ in step.remove:
            self.worker.submit(self._remove, occ)
        for occ in step.give_up:
            self.get_logger().warn(f"{occ.name} on {occ.obstacle.label} given up: a robot "
                                   f"stayed within {self.clearance_m} m for "
                                   f"{self.max_delay_s} s")
        for occ in step.lost:
            self.get_logger().error(f"{occ.name} on {occ.obstacle.label} could not be removed "
                                    f"after {obstacles.MAX_REMOVE_TRIES} tries; it may still "
                                    f"be in Gazebo")
        for occ in step.waiting:
            if occ.number not in self.told_waiting:
                self.told_waiting.add(occ.number)
                self.get_logger().info(f"{occ.name} on {occ.obstacle.label} waits for robots "
                                       f"to move away")

    def _call(self, argv: list[str]) -> bool:
        try:
            done = subprocess.run(argv, capture_output=True, text=True, timeout=CALL_LIMIT_S)
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.get_logger().warn(f"{argv[3]} failed: {exc}")
            return False
        if done.returncode != 0 or not obstacles.reply_ok(done.stdout):
            self.get_logger().warn(f"{argv[3]} failed: {(done.stdout + done.stderr).strip()}")
            return False
        return True

    def _spawn(self, occ: obstacles.Occurrence):
        one = occ.obstacle
        argv = obstacles.spawn_argv(self.world, occ.name, one, self.schedule.size_m)
        outcome = obstacles.try_spawn(occ, self._positions, self.clearance_m,
                                      lambda: self._call(argv),
                                      lambda: self.planner.too_late(occ, self._now()))
        self.outcomes.put(("spawn", occ, self._now(), outcome))
        if outcome == obstacles.PLACED:
            with self.live_lock:
                self.live[occ.name] = occ
            self._publish_markers()
            self.get_logger().info(f"{occ.name} on {one.label} at ({one.x:.2f}, {one.y:.2f}) "
                                   f"placed for {one.duration_s:g} s")
        elif outcome == obstacles.SKIPPED:
            self.get_logger().info(f"{occ.name} on {one.label}: not placed (a robot came close "
                                   f"or it is past max_delay_s)")
        else:
            self.get_logger().warn(f"{occ.name} on {one.label}: no clear answer, removing it "
                                   f"in case it landed")

    def _remove(self, occ: obstacles.Occurrence):
        ok = self._call(obstacles.remove_argv(self.world, occ.name))
        self.outcomes.put(("remove", occ, self._now(), ok))
        if ok:
            with self.live_lock:
                self.live.pop(occ.name, None)
            self._publish_markers()
            self.get_logger().info(f"{occ.name} on {occ.obstacle.label} removed")

    def _publish_markers(self):
        with self.live_lock:
            placed = sorted(self.live.values(), key=lambda occ: occ.number)
        stamp = self.get_clock().now().to_msg()
        out = MarkerArray()
        for spec in obstacles.marker_specs(placed, self.schedule.size_m):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = stamp
            if spec["action"] == "deleteall":
                marker.action = Marker.DELETEALL
                out.markers.append(marker)
                continue
            marker.ns, marker.id = spec["ns"], spec["id"]
            marker.type, marker.action = MARKER_TYPES[spec["type"]], Marker.ADD
            marker.pose.position.x, marker.pose.position.y = spec["x"], spec["y"]
            marker.pose.position.z = spec["z"]
            marker.pose.orientation.z = math.sin(spec["yaw"] / 2)
            marker.pose.orientation.w = math.cos(spec["yaw"] / 2)
            marker.scale.x, marker.scale.y, marker.scale.z = spec["scale"]
            (marker.color.r, marker.color.g, marker.color.b,
             marker.color.a) = spec["color"]
            marker.text = spec.get("text", "")
            out.markers.append(marker)
        self.markers.publish(out)

    def remove_all(self):
        """작업 스레드를 멈춘 뒤 남은 상자를 치운다(될 수 있는 만큼)."""
        self.worker.shutdown(wait=True, cancel_futures=True)
        # 작업 스레드가 멈췄으니 잠금 없이 읽는다. The planner also knows spawns without an answer
        maybe = {occ.name for occ in self.planner.in_gazebo()} if self.planner else set()
        for name in sorted(maybe | set(self.live)):
            argv = obstacles.remove_argv(self.world, name, SHUTDOWN_TIMEOUT_MS)
            try:
                subprocess.run(argv, capture_output=True, timeout=CALL_LIMIT_S)
            except (OSError, subprocess.TimeoutExpired):
                pass
        self.live.clear()
        try:
            self._publish_markers()  # RViz에 남은 상자도 지운다
        except Exception:  # noqa: BLE001 - 이미 ROS가 내려갔으면 그만이다
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.remove_all()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
