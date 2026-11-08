"""Gazebo 3대 시뮬 전용 노드 둘. 실물에서는 쓰지 않는다.

sim_stand_in: 로봇마다
  - 실제 위치 중계: /{pinky}/sim/truth(Odometry, map 좌표) → /sim/truth/{pinky}(RobotState).
    시나리오 모니터(fake_robots)가 구역 판정에 쓰는 잡음 없는 자세다. 늘 켜져 있다.
  - dock_stub:=true일 때만 /{pinky}/dock(Dock) 대역(dock_stand_in). 기본은 꺼짐이고 진짜
    pinky_fsm이 도킹을 DOCK_UNAVAILABLE로 실패시킨다. 켜면 gz.launch.py가 pinky_fsm의 dock
    서버를 /{pinky}/dock_fsm으로 비켜 둔다. The node refuses to start the stub unless
    use_sim_time is true and the network passes isolation.py.
wait_ready: 모든 로봇의 최근 /{pinky}/state가 IDLE인 채로 1 s 이어지면 0으로 끝난다(ready.py).
  gz.launch.py가 그때 관제와 가짜 central·팔을 띄운다. timeout_s(벽시계) 안에 안 되면 1로 끝난다.
"""
from __future__ import annotations

import math
import os
import sys
import threading
import time
from pathlib import Path

import rclpy
from fake_robots.common import (
    TRUTH_TOPIC,
    load_contract_param,
    load_lane_map_param,
    load_scenario_param,
)
from fleet_core.common.contract import to_ros_qos
from fleet_interfaces.action import Dock
from fleet_interfaces.msg import RobotState
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from fleet_gz_sim.dock_stand_in import dock_outcome, dock_phase, require_dock_stub_allowed
from fleet_gz_sim.ready import everyone_idle

READY_HOLD_S = 1.0
REPORT_FRESH_S = 0.5  # pinky_fsm은 state를 10 Hz로 낸다


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SimStandIn(Node):
    def __init__(self, **kwargs):
        super().__init__("sim_stand_in", **kwargs)
        self.declare_parameter("robots", ["pinky1", "pinky2", "pinky3"])
        stub = bool(self.declare_parameter("dock_stub", False).value)
        if stub:  # before anything else: never serve a dock outside an isolated Gazebo run
            require_dock_stub_allowed(bool(self.get_parameter("use_sim_time").value),
                                      os.environ, rmw=rclpy.get_rmw_implementation_identifier(),
                                      workdir=Path.cwd())
        contract = load_contract_param(self)
        self._site = load_lane_map_param(self)
        self._duration = load_scenario_param(self, self._site).dock.duration_s if stub else 0.0
        self._lock = threading.Lock()
        self._pose: dict[str, tuple[float, float, float]] = {}
        self._turn: dict[str, int] = {}  # 로봇마다 마지막으로 받은 도크 목표 번호
        qos = to_ros_qos(contract.qos("robot_state"))
        group = ReentrantCallbackGroup()
        self.dock_servers = []
        for robot in self.get_parameter("robots").value:
            pub = self.create_publisher(RobotState, TRUTH_TOPIC.format(robot=robot), qos)
            self.create_subscription(Odometry, f"/{robot}/sim/truth",
                                     lambda msg, r=robot, p=pub: self._on_truth(r, p, msg), 10)
            if not stub:
                continue
            self.dock_servers.append(ActionServer(
                self, Dock, contract.name("dock", pinky=robot),
                execute_callback=lambda handle, r=robot: self._dock(r, handle),
                goal_callback=lambda _goal: GoalResponse.ACCEPT,
                cancel_callback=lambda _handle: CancelResponse.ACCEPT,
                callback_group=group))
        robots = self.get_parameter("robots").value
        what = "SIM DOCK STUB and truth" if stub else "truth (dock left to pinky_fsm)"
        self.get_logger().info(f"sim stand-in: {what} for {robots}")

    def _on_truth(self, robot: str, pub, msg: Odometry) -> None:
        p = msg.pose.pose
        pose = (p.position.x, p.position.y, _yaw(p.orientation))
        with self._lock:
            self._pose[robot] = pose
        out = RobotState(robot_id=robot, state=RobotState.STATE_IDLE, stamp=msg.header.stamp)
        out.pose.x, out.pose.y, out.pose.theta = pose
        pub.publish(out)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _dock(self, robot: str, handle):
        with self._lock:
            turn = self._turn[robot] = self._turn.get(robot, 0) + 1
        station = self._site.stations.get(handle.request.dock_id)
        if station is None:
            self.get_logger().warn(f"{robot}: dock {handle.request.dock_id!r} is not a station")
            handle.abort()
            return Dock.Result(success=False)
        target = self._site.waypoint_pose(station.dock)
        start, phase = self._now(), ""
        while rclpy.ok():
            with self._lock:
                replaced = self._turn[robot] != turn
            if handle.is_cancel_requested:
                handle.canceled()
                return Dock.Result(success=False)
            if replaced:  # FakePinky처럼 새 목표가 오면 이전 목표는 실패로 끝난다
                handle.abort()
                return Dock.Result(success=False)
            elapsed = self._now() - start
            now_phase = dock_phase(elapsed, self._duration)
            if now_phase != phase:
                phase = now_phase
                handle.publish_feedback(Dock.Feedback(phase=phase))
            if elapsed >= self._duration:
                break
            time.sleep(0.02)
        with self._lock:
            pose = self._pose.get(robot)
        if pose is None:
            self.get_logger().warn(f"{robot}: no truth pose yet, dock fails")
            handle.abort()
            return Dock.Result(success=False)
        ok, ex, ey, eyaw = dock_outcome(pose, target)
        self.get_logger().info(f"{robot}: dock {handle.request.dock_id} "
                               f"{'ok' if ok else 'FAILED'} ({ex:.1f}, {ey:.1f}) cm")
        handle.succeed() if ok else handle.abort()
        return Dock.Result(success=ok, error_x_cm=ex, error_y_cm=ey, error_yaw_deg=eyaw)


def main(args=None):
    rclpy.init(args=args)
    node = SimStandIn()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def wait_ready(args=None):
    rclpy.init(args=args)
    node = Node("wait_ready")
    node.declare_parameter("robots", ["pinky1", "pinky2", "pinky3"])
    node.declare_parameter("timeout_s", 180.0)
    contract = load_contract_param(node)
    robots = list(node.get_parameter("robots").value)
    timeout = float(node.get_parameter("timeout_s").value)
    last: dict[str, tuple[str, float]] = {}  # 로봇마다 (상태, 받은 벽시계 시각)
    for robot in robots:
        node.create_subscription(
            RobotState, contract.name("robot_state", pinky=robot),
            lambda msg, r=robot: last.__setitem__(r, (msg.state, time.monotonic())),
            to_ros_qos(contract.qos("robot_state")))
    deadline, since, code = time.monotonic() + timeout, None, 1
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            ready = everyone_idle(last, robots, time.monotonic(), REPORT_FRESH_S)
            since = (since or time.monotonic()) if ready else None
            if since is not None and time.monotonic() - since >= READY_HOLD_S:
                node.get_logger().info(f"all robots IDLE: {robots}")
                code = 0
                break
        else:
            states = {r: last[r][0] for r in last}
            node.get_logger().error(f"robots not IDLE after {timeout:.0f} s: {states}")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)
