"""시뮬 장애물 노드: 일정대로 Gazebo 레인 위에 상자를 놓고 치운다.

    ros2 run fleet_gz_sim sim_obstacles --ros-args -p use_sim_time:=true \\
        -p obstacles_file:=<share>/obstacles/roam.yaml

로봇이 clearance_m 안에 있으면 놓지 않고 기다린다(실제 위치 /<robot>/sim/truth로 본다).
gz service 호출은 작업 스레드 하나에서 차례로 돌린다. 끝날 때 놓은 상자를 모두 치운다.
Gazebo 통신 설정(GZ_IP, GZ_PARTITION)은 launch에서 물려받는다.
"""
from __future__ import annotations

import subprocess
from concurrent.futures import ThreadPoolExecutor

import rclpy
from ament_index_python.packages import get_package_share_directory
from fleet_core.traffic.lane_map import load_lane_map
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from fleet_gz_sim import obstacles

TICK_S = 0.2
CALL_LIMIT_S = 5.0  # gz service 자체 한도(3 s)에 여유를 둔다
SHUTDOWN_TIMEOUT_MS = 1000  # 끝낼 때는 launch가 기다려 주는 동안 다 치우도록 짧게


class ObstacleNode(Node):
    def __init__(self):
        super().__init__("sim_obstacles")
        robots = self.declare_parameter("robots", ["pinky1", "pinky2", "pinky3"]).value
        path = self.declare_parameter("obstacles_file", "").value
        lane_map_path = self.declare_parameter("lane_map", "").value
        self.world = self.declare_parameter("world", "lane_arena").value
        self.clearance_m = float(self.declare_parameter("clearance_m", 0.35).value)
        self.max_delay_s = float(self.declare_parameter("max_delay_s", 20.0).value)
        if not path:
            path = f"{get_package_share_directory('fleet_gz_sim')}/obstacles/roam.yaml"
        self.schedule = obstacles.load_schedule(path, load_lane_map(lane_map_path or None))
        self.robots = list(robots)
        self.truth: dict[str, tuple[float, float]] = {}
        for robot in self.robots:
            self.create_subscription(Odometry, f"/{robot}/sim/truth",
                                     lambda msg, r=robot: self._on_truth(r, msg),
                                     qos_profile_sensor_data)
        self.planner: obstacles.Planner | None = None
        self.told_waiting: set[int] = set()
        self.live: set[str] = set()  # Gazebo에 놓인 상자. 작업 스레드에서만 바꾼다
        self.worker = ThreadPoolExecutor(max_workers=1)
        self.create_timer(TICK_S, self._tick)
        self.get_logger().info(f"{len(self.schedule.obstacles)} obstacles from {path}, "
                               f"repeat every {self.schedule.repeat_every_s} s")

    def _on_truth(self, robot: str, msg: Odometry):
        p = msg.pose.pose.position
        self.truth[robot] = (p.x, p.y)

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0.0:
            return  # /clock를 아직 못 받았다
        if self.planner is None:
            self.planner = obstacles.Planner(self.schedule, now, self.max_delay_s)
        step = self.planner.step(now, [self.truth.get(r) for r in self.robots],
                                 self.clearance_m)
        for occ in step.spawn:
            self.worker.submit(self._spawn, occ)
        for occ in step.remove:
            self.worker.submit(self._remove, occ)
        for occ in step.give_up:
            self.get_logger().warn(f"{occ.name} on {occ.obstacle.label} given up: a robot "
                                   f"stayed within {self.clearance_m} m for "
                                   f"{self.max_delay_s} s")
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
        if self._call(obstacles.spawn_argv(self.world, occ.name, one, self.schedule.size_m)):
            self.live.add(occ.name)
            self.get_logger().info(f"{occ.name} on {one.label} at ({one.x:.2f}, {one.y:.2f}) "
                                   f"placed for {one.duration_s:g} s")

    def _remove(self, occ: obstacles.Occurrence):
        if occ.name not in self.live:
            return  # 놓지 못했다
        if self._call(obstacles.remove_argv(self.world, occ.name)):
            self.live.discard(occ.name)
            self.get_logger().info(f"{occ.name} on {occ.obstacle.label} removed")

    def remove_all(self):
        """작업 스레드를 멈춘 뒤 남은 상자를 치운다(될 수 있는 만큼)."""
        self.worker.shutdown(wait=True, cancel_futures=True)
        for name in sorted(self.live):
            argv = obstacles.remove_argv(self.world, name, SHUTDOWN_TIMEOUT_MS)
            try:
                subprocess.run(argv, capture_output=True, timeout=CALL_LIMIT_S)
            except (OSError, subprocess.TimeoutExpired):
                pass
        self.live.clear()


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
