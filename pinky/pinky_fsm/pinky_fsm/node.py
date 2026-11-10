"""pinky_fsm 노드. 핑키 한 대의 move_to, dock을 받아 Nav2 FollowPath로 넘기고 상태를 보고한다.

판단은 모두 core.PinkyFsm이 한다. 이 파일은 ROS와 잇기만 한다.
- 액션 서버: 계약의 move_to(MoveTo), dock(Dock), retreat(Retreat, 버전 2 1-A, 계약 0.9-draft 제안.
  path가 있으면 후진 controller reverse_controller_id로 따른다)
- 발행: 계약의 robot_state(RobotState, 기본 10 Hz), Nav2 controller의 speed_limit
  (잠금 경계 목표만, 버전 2 4절 제안)
- 구독: 계약의 heartbeat, odom(속도), local costmap(BLOCKED 판단)
- 액션 클라이언트: Nav2 controller의 follow_path(FollowPath), behavior_server의
  backup(BackUp, 자가 복구. 핑키_계약 3절 7번 2026-11-09 제안)
- 서비스 클라이언트: local costmap 비우기(ClearEntireCostmap, 자가 복구)
- 위치: TF map → base_footprint(AMCL). 시간이 지난 TF는 위치를 모르는 것으로 친다

로봇 namespace 안에서 돌고 /tf, /tf_static을 namespace의 tf, tf_static으로 바꿔 띄운다.
"""
from __future__ import annotations

import math
import threading
import time
from functools import partial

import rclpy
from action_msgs.msg import GoalStatus
from fleet_interfaces.action import Dock, MoveTo, Retreat
from fleet_interfaces.msg import RobotState
from geometry_msgs.msg import Pose2D, PoseStamped
from nav2_msgs.action import FollowPath
from nav2_msgs.msg import SpeedLimit
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from pinky_fsm.backup import BackUp, CancelBackUp, reason_of
from pinky_fsm.backup_ros import BackUpRunner
from pinky_fsm.contract import default_contract_path, load_contract, to_ros_qos
from pinky_fsm.core import (
    RECOVERING,
    CancelNav,
    ClearCostmap,
    Finish,
    FsmParams,
    LimitSpeed,
    PinkyFsm,
    SendPath,
)
from pinky_fsm.costmap import Grid, obstacle_on, to_frame
from pinky_fsm.geometry import quaternion_yaw, yaw_quaternion

FEEDBACK_PERIOD_S = 0.2
UNKNOWN_BATTERY = -1.0  # 배터리 토픽이 없을 때. 관제는 보여 주기만 한다
_OUTCOME = {GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_ABORTED: "aborted",
            GoalStatus.STATUS_CANCELED: "canceled"}
_RESULT = {"move": MoveTo.Result, "dock": Dock.Result, "retreat": Retreat.Result}
_FLOAT_PARAMS = (
    "arrival_tolerance_m", "connect_tolerance_m", "refresh_within_m", "refresh_every_s",
    "heartbeat_timeout_s", "stopped_speed_mps", "stopped_turn_rps", "block_report_s",
    "block_clear_s", "block_timeout_s", "block_lookahead_m", "path_step_m",
    # 자가 복구
    "backup_max_m", "release_distance_m", "backup_margin_m", "backup_min_m",
    "backup_lateral_limit_m", "backup_speed_mps", "backup_time_allowance_s",
    "backup_result_grace_s", "recovery_pass_m", "recovery_timeout_s", "recovery_reset_m",
    # 관제가 시키는 물러나기(버전 2 1-A, 경로 물러나기)
    "retreat_stop_wait_s", "reverse_speed_mps",
    # 잠금 경계 목표의 속도 제한(버전 2 4절 horizon 최소판)
    "horizon_decel_mps2", "horizon_min_mps", "horizon_cap_mps", "horizon_latency_s")
_INT_PARAMS = ("recovery_attempts",)


class _Slot:
    """액션 목표 하나. core가 Finish를 내면 결과를 담고 done을 세운다."""

    def __init__(self, handle):
        self.handle = handle
        self.done = threading.Event()
        self.success = False
        self.reason = ""


class PinkyFsmNode(Node):
    def __init__(self, **kwargs):
        super().__init__("pinky_fsm", **kwargs)
        d = FsmParams()
        declare = self.declare_parameter
        declare("robot_id", self.get_namespace().strip("/") or "pinky1")
        declare("contract_file", "")
        declare("map_frame", "map")
        declare("base_frame", "base_footprint")
        declare("follow_path_action", "follow_path")
        declare("backup_action", "backup")
        declare("clear_costmap_service", "local_costmap/clear_entirely_local_costmap")
        declare("controller_id", "FollowPath")
        # Controller for paths that end at a lock boundary (MoveTo.final_goal false): an RPP
        # without the approach slow-down, so the speed limit is the only slow-down there.
        # Empty = off: every path uses controller_id, as before. Nav2 must load this plugin
        declare("through_controller_id", "")
        # 물러나기 경로(Retreat.path)를 후진으로 따르는 controller. pinky_nav_config가 늘 띄운다
        declare("reverse_controller_id", "FollowPathReverse")
        declare("goal_checker_id", "general_goal_checker")
        declare("progress_checker_id", "progress_checker")
        declare("odom_topic", "odom")
        declare("costmap_topic", "local_costmap/costmap")
        declare("speed_limit_topic", "speed_limit")  # nav2 controller_server speed_limit_topic
        declare("battery_topic", "")
        declare("state_rate_hz", 10.0)
        declare("control_rate_hz", 20.0)
        declare("pose_timeout_s", 1.0)
        for name in _FLOAT_PARAMS:
            declare(name, float(getattr(d, name)))
        for name in _INT_PARAMS:
            declare(name, int(getattr(d, name)))
        get = self.get_parameter
        self.robot_id = get("robot_id").value
        self._map_frame = get("map_frame").value
        self._base_frame = get("base_frame").value
        self._pose_timeout = float(get("pose_timeout_s").value)
        self._nav_ids = (get("controller_id").value, get("goal_checker_id").value,
                         get("progress_checker_id").value)
        self._through_id = get("through_controller_id").value
        self._reverse_id = get("reverse_controller_id").value
        self._fsm = PinkyFsm(FsmParams(
            **{name: float(get(name).value) for name in _FLOAT_PARAMS},
            **{name: int(get(name).value) for name in _INT_PARAMS}))
        self._last_state = ""
        contract = load_contract(get("contract_file").value or default_contract_path())

        self._lock = threading.RLock()
        self._alive = True
        self._slots: dict[bytes, _Slot] = {}
        self._nav_handles: dict[int, object] = {}  # 순번 → 수락된 FollowPath 목표
        self._nav_pending: set[int] = set()  # 보냈지만 수락 여부를 아직 모른다
        self._cancel_on_accept: set[int] = set()
        self._late: list[tuple[int, str]] = []  # 다음 주기에 core에 넘길 FollowPath 결과
        self._late_backup: list[tuple[int, str]] = []  # 다음 주기에 넘길 BackUp 결과
        self._speed = (0.0, 0.0)
        self._costmap: tuple[Grid, str] | None = None  # (격자, 틀). 한 번에 바꿔 짝이 맞게 둔다
        self._battery = UNKNOWN_BATTERY

        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        group = ReentrantCallbackGroup()
        self._speed_pub = self.create_publisher(SpeedLimit, get("speed_limit_topic").value, 10)
        self._state_pub = self.create_publisher(
            RobotState, contract.name("robot_state", pinky=self.robot_id),
            to_ros_qos(contract.qos("robot_state")))
        self.create_subscription(Header, contract.name("heartbeat"), self._on_heartbeat,
                                 to_ros_qos(contract.qos("heartbeat")), callback_group=group)
        self.create_subscription(Odometry, get("odom_topic").value, self._on_odom,
                                 qos_profile_sensor_data, callback_group=group)
        # Nav2 costmap은 reliable로 발행한다. volatile로 받으면 transient_local 발행과도 맞는다
        self.create_subscription(
            OccupancyGrid, get("costmap_topic").value, self._on_costmap,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE), callback_group=group)
        if get("battery_topic").value:
            self.create_subscription(BatteryState, get("battery_topic").value,
                                     self._on_battery, qos_profile_sensor_data)
        self._nav = ActionClient(self, FollowPath, get("follow_path_action").value,
                                 callback_group=group)
        self._backup = BackUpRunner(self, get("backup_action").value, self._on_backup_result,
                                    callback_group=group)
        self._clear = self.create_client(ClearEntireCostmap, get("clear_costmap_service").value,
                                         callback_group=group)
        self._move_server = self._server(MoveTo, contract.name("move_to", pinky=self.robot_id),
                                         "move", group)
        self._dock_server = self._server(Dock, contract.name("dock", pinky=self.robot_id),
                                         "dock", group)
        self._retreat_server = self._server(
            Retreat, contract.name("retreat", pinky=self.robot_id), "retreat", group)
        # 판단과 보고는 벽시계로 돈다(heartbeat는 벽시계로 재야 하고, 시뮬 시계가 멈춰도 보고한다)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / float(get("control_rate_hz").value), self._tick,
                          callback_group=group, clock=steady)
        self.create_timer(1.0 / float(get("state_rate_hz").value), self._publish_state,
                          callback_group=group, clock=steady)
        self.get_logger().info(f"pinky_fsm for {self.robot_id} ready")

    def _server(self, action_type, name, kind, group):
        return ActionServer(
            self, action_type, name,
            execute_callback=partial(self._execute, kind=kind),
            # 정상이면 거절하지 않는다(계약 3절 1번)
            goal_callback=lambda _req: GoalResponse.ACCEPT,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            handle_accepted_callback=partial(self._accepted, kind=kind),
            callback_group=group)

    # -- 수명 ------------------------------------------------------------------------------

    def stop(self) -> None:
        with self._lock:
            self._alive = False
            self._cancel_all_nav()
            self._backup.cancel_all()

    def destroy_node(self):
        self.stop()
        self._move_server.destroy()
        self._dock_server.destroy()
        self._retreat_server.destroy()
        self._nav.destroy()
        self._backup.destroy()
        return super().destroy_node()

    # -- 입력 ------------------------------------------------------------------------------

    def _on_heartbeat(self, _msg) -> None:
        with self._lock:
            self._fsm.heartbeat(time.monotonic())

    def _on_odom(self, msg: Odometry) -> None:
        t = msg.twist.twist
        self._speed = (math.hypot(t.linear.x, t.linear.y), t.angular.z)

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        o = msg.info.origin
        grid = Grid(msg.info.resolution, o.position.x, o.position.y,
                    quaternion_yaw(o.orientation.x, o.orientation.y, o.orientation.z,
                                   o.orientation.w),
                    msg.info.width, msg.info.height, msg.data)
        self._costmap = (grid, msg.header.frame_id)

    def _on_battery(self, msg: BatteryState) -> None:
        if not math.isnan(msg.percentage):
            self._battery = float(msg.percentage) * 100.0

    def _pose(self):
        """map 틀의 (x, y, yaw, stamp). 모르면 None."""
        try:
            tf = self._tf.lookup_transform(self._map_frame, self._base_frame, Time())
        except TransformException:
            return None
        stamp = Time.from_msg(tf.header.stamp)
        if (self.get_clock().now() - stamp).nanoseconds * 1e-9 > self._pose_timeout:
            return None  # AMCL이 멈췄다
        q, p = tf.transform.rotation, tf.transform.translation
        return (p.x, p.y, quaternion_yaw(q.x, q.y, q.z, q.w), tf.header.stamp)

    def _obstacle(self, points) -> bool:
        costmap = self._costmap
        if costmap is None or not points:
            return False
        grid, frame = costmap
        if frame and frame != self._map_frame:
            try:
                tf = self._tf.lookup_transform(frame, self._map_frame, Time())
            except TransformException:
                return False
            q, p = tf.transform.rotation, tf.transform.translation
            points = to_frame(points, (p.x, p.y, quaternion_yaw(q.x, q.y, q.z, q.w)))
        return obstacle_on(grid, points)

    # -- 액션 서버 --------------------------------------------------------------------------

    def _accepted(self, handle, kind: str) -> None:
        key = bytes(handle.goal_id.uuid)
        with self._lock:
            self._slots[key] = _Slot(handle)
            pose = self._pose()
            here = pose[:3] if pose else None
            now = time.monotonic()
            req = handle.request
            if kind == "move":
                t = req.target_pose
                path = [(p.position.x, p.position.y) for p in req.path]
                cmds = self._fsm.move(key, now, req.target_id, (t.x, t.y, t.theta), path, here,
                                      final_goal=req.final_goal)
            elif kind == "retreat":
                path = [(p.position.x, p.position.y) for p in req.path]
                cmds = self._fsm.retreat(key, now, req.distance_m, req.speed_mps, path, here)
            else:
                cmds = self._fsm.dock(key, now, req.dock_id)
            self._run(cmds)
        handle.execute()

    def _execute(self, handle, kind: str):
        key = bytes(handle.goal_id.uuid)
        slot = self._slots[key]
        last_fb = 0.0
        while not slot.done.is_set():
            if not (self._alive and rclpy.ok()):
                _abort_quietly(handle)
                return _RESULT[kind]()
            if handle.is_cancel_requested:
                with self._lock:
                    self._run(self._fsm.cancel(key, time.monotonic()))
            now = time.monotonic()
            if kind == "move" and now - last_fb >= FEEDBACK_PERIOD_S:
                last_fb = now
                with self._lock:
                    # 교체된 목표에는 새 목표의 남은 거리를 보내지 않는다(곧 결과가 나간다).
                    # 교체도 이 잠금 안에서 일어나므로 잠금을 쥔 채 보낸다
                    if self._fsm.is_running(key):
                        pose = self._pose()
                        left = self._fsm.distance_remaining(pose[:3] if pose else None)
                        handle.publish_feedback(MoveTo.Feedback(distance_remaining=float(left)))
            slot.done.wait(0.02)
        with self._lock:
            self._slots.pop(key, None)
        if slot.success:
            handle.succeed()
        elif handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.abort()
        if kind == "move":
            return MoveTo.Result(success=slot.success, reason=slot.reason)
        if kind == "retreat":
            return Retreat.Result(success=slot.success, reason=slot.reason)
        if not slot.success:
            self.get_logger().warn(f"dock failed: {slot.reason}")
        return Dock.Result(success=slot.success)

    # -- core 명령 실행 ----------------------------------------------------------------------

    def _run(self, cmds) -> None:
        """self._lock을 쥔 채 부른다."""
        for cmd in cmds:
            if isinstance(cmd, SendPath):
                self._send_path(cmd)
            elif isinstance(cmd, CancelNav):
                self._cancel_all_nav()
            elif isinstance(cmd, BackUp):
                self.get_logger().info(
                    f"backup #{cmd.seq} {cmd.distance_m:.3f} m (self-recovery attempts "
                    f"{self._fsm.recovery_attempts_used})")
                if not self._backup.send(cmd.seq, cmd.distance_m, cmd.speed_mps,
                                         cmd.time_allowance_s):
                    self.get_logger().warn("backup server is not available")
                    self._late_backup.append((cmd.seq, "rejected"))
            elif isinstance(cmd, CancelBackUp):
                self._backup.cancel_all()
            elif isinstance(cmd, ClearCostmap):
                self._clear_costmap()
            elif isinstance(cmd, LimitSpeed):
                msg = SpeedLimit(percentage=False, speed_limit=float(cmd.mps))
                msg.header.stamp = self.get_clock().now().to_msg()
                self._speed_pub.publish(msg)
            elif isinstance(cmd, Finish):
                slot = self._slots.get(cmd.goal_id)
                if slot is not None:
                    slot.success, slot.reason = cmd.success, cmd.reason
                    slot.done.set()
                if not cmd.success:
                    self.get_logger().info(f"goal ended: {cmd.reason}")

    def _send_path(self, cmd: SendPath) -> None:
        if not self._nav.server_is_ready():
            self.get_logger().warn("follow_path server is not available")
            self._late.append((cmd.seq, "rejected"))
            return
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id, path.header.stamp = self._map_frame, stamp
        for x, y, yaw in cmd.poses:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            qx, qy, qz, qw = yaw_quaternion(yaw)
            ps.pose.orientation.z, ps.pose.orientation.w = qz, qw
            path.poses.append(ps)
        controller, checker, progress = self._nav_ids
        if self._through_id and not cmd.final_goal:
            # Nav2 switches controllers on a preempting goal (controller_server
            # updateGlobalPath), and the speed limit reaches every loaded controller
            controller = self._through_id
        if cmd.reverse:
            controller = self._reverse_id
        goal = FollowPath.Goal(path=path, controller_id=controller, goal_checker_id=checker,
                               progress_checker_id=progress)
        self._nav_pending.add(cmd.seq)
        self._nav.send_goal_async(goal).add_done_callback(partial(self._on_nav_response,
                                                                  cmd.seq))

    def _clear_costmap(self) -> None:
        if not self._clear.service_is_ready():
            self.get_logger().warn("clear costmap service is not available")
            return
        self._clear.call_async(ClearEntireCostmap.Request())

    def _on_backup_result(self, seq: int, outcome: str, error_code: int,
                          travelled: float) -> None:
        reason = reason_of(outcome, error_code)
        self.get_logger().info(f"backup #{seq} {outcome} {reason or 'OK'} "
                               f"after {travelled:.3f} m")
        with self._lock:
            pose = self._pose()
            self._run(self._fsm.backup_result(seq, outcome, time.monotonic(),
                                              pose[:3] if pose else None, error_code))

    def _cancel_all_nav(self) -> None:
        for handle in self._nav_handles.values():
            handle.cancel_goal_async()
        self._cancel_on_accept |= self._nav_pending

    def _on_nav_response(self, seq: int, future) -> None:
        handle = future.result()
        with self._lock:
            self._nav_pending.discard(seq)
            if not handle.accepted:
                self._late.append((seq, "rejected"))
                return
            self._nav_handles[seq] = handle
            if seq in self._cancel_on_accept:
                self._cancel_on_accept.discard(seq)
                handle.cancel_goal_async()  # 멈추는 동안 수락된 목표는 바로 취소한다
        handle.get_result_async().add_done_callback(partial(self._on_nav_result, seq))

    def _on_nav_result(self, seq: int, future) -> None:
        res = future.result()
        outcome = _OUTCOME.get(res.status, "aborted")
        if outcome != "succeeded" and res.result.error_code:
            self.get_logger().info(f"follow_path #{seq} {outcome}: "
                                   f"{res.result.error_code} {res.result.error_msg}")
        with self._lock:
            self._nav_handles.pop(seq, None)
            pose = self._pose()
            self._run(self._fsm.nav_result(seq, outcome, time.monotonic(),
                                           pose[:3] if pose else None, self._obstacle,
                                           int(res.result.error_code)))

    # -- 주기 ------------------------------------------------------------------------------

    def _tick(self) -> None:
        with self._lock:
            if not self._alive:
                return
            pose = self._pose()
            here = pose[:3] if pose else None
            now = time.monotonic()
            late, self._late = self._late, []
            for seq, outcome in late:
                self._run(self._fsm.nav_result(seq, outcome, now, here, self._obstacle))
            late_backup, self._late_backup = self._late_backup, []
            for seq, outcome in late_backup:
                self._run(self._fsm.backup_result(seq, outcome, now, here))
            self._run(self._fsm.tick(now, here, self._speed, self._obstacle))
            self._log_state()

    def _log_state(self) -> None:
        """자가 복구가 들고 나는 것을 남긴다(시뮬 기록과 맞춰 보려고)."""
        state = self._fsm.state()
        if state != self._last_state and RECOVERING in (state, self._last_state):
            self.get_logger().info(f"state {self._last_state} -> {state} "
                                   f"(self-recovery attempts used "
                                   f"{self._fsm.recovery_attempts_used})")
        self._last_state = state

    def _publish_state(self) -> None:
        with self._lock:
            state, goal = self._fsm.state(), self._fsm.current_goal()
            pose = self._pose()
        msg = RobotState(robot_id=self.robot_id, battery=float(self._battery), state=state,
                         current_goal=goal)
        if pose is not None and state != RobotState.STATE_LOCALIZING:
            msg.pose = Pose2D(x=float(pose[0]), y=float(pose[1]), theta=float(pose[2]))
            msg.stamp = pose[3]  # 위치를 잰 시각(계약 2절)
        else:
            msg.stamp = self.get_clock().now().to_msg()
        self._state_pub.publish(msg)


def _abort_quietly(handle) -> None:
    try:
        handle.abort()
    except Exception:  # noqa: BLE001 - 종료 중에는 상태 발행이 실패할 수 있다
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PinkyFsmNode()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception:  # SIGINT가 메시지 수신 중에 떨어질 수 있다. 진짜 오류면 ROS가 살아 있다
        if rclpy.ok():
            raise
    finally:
        node.stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
