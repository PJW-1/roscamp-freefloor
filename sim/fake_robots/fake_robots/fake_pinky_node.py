"""FakePinky: robot_state, move_to, dock을 가진 가짜 이동 로봇. 일정대로 고장을 낸다."""
from __future__ import annotations

import random
import threading
import time

import rclpy
from fleet_core.common.contract import to_ros_qos
from fleet_interfaces.action import Dock, MoveTo
from fleet_interfaces.msg import RobotState
from geometry_msgs.msg import Pose2D
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Header

from fake_robots.common import (
    TRUTH_TOPIC,
    ScenarioClock,
    load_contract_param,
    load_lane_map_param,
    load_params_param,
    load_scenario_param,
    node_name,
)
from fake_robots.motion import (
    DelayLine,
    FaultSchedule,
    Motion,
    abort_at_shutdown,
    finish_action,
    noisy_pose,
)

NAV_ABORT_DELAY_S = 1.0
OFF_COURSE_M = 1.0  # 운영자가 치운 로봇이 가는 곳(지도 모서리 밖)
FEEDBACK_PERIOD_S = 0.2
CONTROL_RATE_HZ = 20.0


class _Goal:
    def __init__(self, kind: str, handle):
        self.kind = kind
        self.handle = handle
        self.done = threading.Event()
        self.success = False
        self.reason = ""
        self.abort_at: float | None = None
        self.end_at = 0.0
        self.start_at = 0.0
        self.error = (0.0, 0.0, 0.0)
        self.target_id = ""

    def finish(self, success: bool, reason: str = "") -> None:
        if not self.done.is_set():
            self.success, self.reason = success, reason
            self.done.set()


class FakePinkyNode(Node):
    def __init__(self, parameter_overrides=None, **kwargs):
        super().__init__(node_name("fake", "robot_id", parameter_overrides),
                         parameter_overrides=parameter_overrides, **kwargs)
        self.declare_parameter("robot_id", "pinky1")
        self.declare_parameter("state_rate_hz", 10.0)
        self.declare_parameter("heartbeat_timeout_s", -1.0)  # <0: 파일 값, 0: 안 멈춤
        self.robot_id = self.get_parameter("robot_id").value
        contract = load_contract_param(self)
        site = load_lane_map_param(self)
        scenario = load_scenario_param(self, site)
        cfg = scenario.robots[self.robot_id]
        self._cfg, self._dock_cfg = cfg, scenario.dock
        self._rng = random.Random(f"{scenario.seed}-{self.robot_id}")
        self._faults = FaultSchedule(scenario.faults, self.robot_id)
        x, y = site.waypoint_pose(cfg.start)
        self._motion = Motion(x, y)
        self._reports = DelayLine(cfg.report_delay_s)
        self._off_course = (site.origin[0] - OFF_COURSE_M, site.origin[1] - OFF_COURSE_M)
        self._site = site
        self._removed = False
        self._sc = ScenarioClock(self)
        self._lock = threading.RLock()
        self._alive = True
        self._active: _Goal | None = None
        self._goals: dict[bytes, _Goal] = {}
        self._rest = RobotState.STATE_IDLE
        self._last_t = self._sc.now()
        timeout = float(self.get_parameter("heartbeat_timeout_s").value)
        self._beat_timeout = (timeout if timeout >= 0
                              else float(load_params_param(self)["heartbeat_timeout_s"]))
        self._last_beat = time.monotonic()

        self._state_pub = self.create_publisher(
            RobotState, contract.name("robot_state", pinky=self.robot_id),
            to_ros_qos(contract.qos("robot_state")))
        self._truth_pub = self.create_publisher(
            RobotState, TRUTH_TOPIC.format(robot=self.robot_id),
            to_ros_qos(contract.qos("robot_state")))
        self.create_subscription(
            Header, contract.name("heartbeat"), self._on_heartbeat,
            to_ros_qos(contract.qos("heartbeat")))
        group = ReentrantCallbackGroup()
        self._move_server = self._server(
            MoveTo, contract.name("move_to", pinky=self.robot_id), "move", group)
        self._dock_server = self._server(
            Dock, contract.name("dock", pinky=self.robot_id), "dock", group)
        self.create_timer(1.0 / CONTROL_RATE_HZ, self._tick)
        self.create_timer(1.0 / float(self.get_parameter("state_rate_hz").value),
                          self._publish_state)

    def _server(self, action_type, name, kind, group):
        return ActionServer(
            self, action_type, name,
            execute_callback=lambda gh: self._execute(gh, action_type),
            goal_callback=lambda _req: self._goal_policy(),
            cancel_callback=lambda _gh: self._cancel_policy(),
            handle_accepted_callback=lambda gh: self._accepted(gh, kind),
            callback_group=group)

    # -- lifecycle ----------------------------------------------------------------------

    def stop(self) -> None:
        self._alive = False

    def destroy_node(self):
        self.stop()
        self._move_server.destroy()
        self._dock_server.destroy()
        return super().destroy_node()

    def _running(self) -> bool:
        return self._alive and rclpy.ok()

    # -- the heartbeat ------------------------------------------------------------------

    def _on_heartbeat(self, _msg) -> None:
        if not self._faults.comms_down(self._sc.now()):
            self._last_beat = time.monotonic()

    def _heartbeat_lost(self) -> bool:
        """실제 시계. 프로세스를 감시하는 것이고, 부하가 걸리면 시뮬 시간이 앞서가기 때문이다."""
        return 0 < self._beat_timeout < time.monotonic() - self._last_beat

    # -- goal handling ------------------------------------------------------------------

    def _unreachable(self) -> bool:
        t = self._sc.now()
        return self._faults.comms_down(t) or self._faults.stuck(t) or self._faults.removed(t)

    def _goal_policy(self):
        return GoalResponse.REJECT if self._unreachable() else GoalResponse.ACCEPT

    def _cancel_policy(self):
        return CancelResponse.REJECT if self._unreachable() else CancelResponse.ACCEPT

    def _end(self, goal: _Goal, success: bool, reason: str = "") -> None:
        goal.finish(success, reason)
        if self._active is goal:
            self._rest = (RobotState.STATE_DOCKED if goal.kind == "dock" and success
                          else RobotState.STATE_IDLE)
            self._motion.clear_target()

    def _accepted(self, handle, kind: str) -> None:
        goal = _Goal(kind, handle)
        with self._lock:
            t = self._sc.now()
            if self._active is not None:
                self._end(self._active, False, MoveTo.Result.REASON_CANCELED)
            self._active = goal
            self._last_beat = time.monotonic()  # 목표가 왔으면 누군가 있다는 뜻
            goal.start_at = t
            if kind == "move":
                pose = handle.request.target_pose
                goal.target_id = handle.request.target_id
                self._motion.set_target(*noisy_pose(pose.x, pose.y, self._cfg.stop_error_m,
                                                    self._rng))
                if self._faults.take_nav_abort(t):
                    eta = self._motion.remaining() / self._cfg.speed_mps
                    goal.abort_at = t + min(NAV_ABORT_DELAY_S, eta / 2)  # 도착 전에 중단
            else:
                self._motion.clear_target()
                goal.end_at = t + self._dock_cfg.duration_s
            self._goals[bytes(handle.goal_id.uuid)] = goal
        handle.execute()

    def _execute(self, handle, action_type):
        goal = self._goals[bytes(handle.goal_id.uuid)]
        last_fb, last_phase = 0.0, ""
        while not goal.done.is_set() and not handle.is_cancel_requested:
            if not self._running():
                abort_at_shutdown(handle)
                return action_type.Result()
            now = time.monotonic()
            if not self._faults.comms_down(self._sc.now()):
                if goal.kind == "move" and now - last_fb >= FEEDBACK_PERIOD_S:
                    last_fb = now
                    with self._lock:
                        remaining = self._motion.remaining()
                    handle.publish_feedback(MoveTo.Feedback(distance_remaining=remaining))
                elif goal.kind == "dock":
                    phase = self._dock_phase(goal)
                    if phase != last_phase:
                        last_phase = phase
                        handle.publish_feedback(Dock.Feedback(phase=phase))
            time.sleep(0.02)
        if handle.is_cancel_requested and not goal.done.is_set():
            with self._lock:
                self._end(goal, False, MoveTo.Result.REASON_CANCELED)
        while self._running() and self._faults.comms_down(self._sc.now()):
            time.sleep(0.02)  # 통신이 돌아올 때까지 결과를 쥐고 있는다
        if not self._running():
            abort_at_shutdown(handle)
            return action_type.Result()
        self._goals.pop(bytes(handle.goal_id.uuid), None)
        result = self._result(goal, action_type)
        finish_action(handle, goal.success)
        return result

    def _dock_phase(self, goal: _Goal) -> str:
        span = max(goal.end_at - goal.start_at, 1e-6)
        done = (self._sc.now() - goal.start_at) / span
        return "APPROACH" if done < 0.5 else "ALIGN" if done < 1.0 else "DONE"

    def _result(self, goal: _Goal, action_type):
        if action_type is MoveTo:
            return MoveTo.Result(success=goal.success, reason=goal.reason)
        ex, ey, eyaw = goal.error
        return Dock.Result(success=goal.success, error_x_cm=ex, error_y_cm=ey,
                           error_yaw_deg=eyaw)

    # -- simulation ---------------------------------------------------------------------

    def _tick(self) -> None:
        with self._lock:
            t = self._sc.now()
            dt, self._last_t = t - self._last_t, t
            goal = self._active
            if self._faults.removed(t):
                self._remove(goal)
                return
            if self._removed:
                self._put_back(self._faults.put_back(t))
            if dt <= 0 or goal is None or goal.done.is_set() or self._faults.stuck(t):
                return
            if self._heartbeat_lost():
                self._end(goal, False, MoveTo.Result.REASON_CANCELED)
                return
            if goal.kind == "move":
                if goal.abort_at is not None and t >= goal.abort_at:
                    self._end(goal, False, MoveTo.Result.REASON_NAV_ABORTED)
                    return
                self._motion.step(dt, self._cfg.speed_mps)
                if not self._motion.moving:
                    self._end(goal, True)
            elif t >= goal.end_at:
                e = self._dock_cfg.error_cm
                goal.error = (self._rng.gauss(e, 0.1 * e), self._rng.gauss(e, 0.1 * e),
                              self._rng.gauss(0.5, 0.1))
                self._end(goal, True)

    def _remove(self, goal: _Goal | None) -> None:
        """운영자가 로봇을 코스에서 들어내 전원을 껐다."""
        if self._removed:
            return
        self._removed = True
        if goal is not None and not goal.done.is_set():
            self._end(goal, False, MoveTo.Result.REASON_NAV_ABORTED)
        self._motion.clear_target()
        self._motion.x, self._motion.y = self._off_course

    def _put_back(self, waypoint) -> None:
        """운영자가 로봇을 이 지점에 다시 놓고 켰다. 바구니는 시나리오가 정한 대로다."""
        self._removed = False
        self._motion.clear_target()
        self._motion.x, self._motion.y = self._site.waypoint_pose(waypoint)
        self._rest = RobotState.STATE_IDLE
        self._reports.clear()

    def _publish_state(self) -> None:
        with self._lock:
            t = self._sc.now()
            self._truth_pub.publish(RobotState(
                robot_id=self.robot_id,
                pose=Pose2D(x=self._motion.x, y=self._motion.y, theta=self._motion.yaw)))
            if self._unreachable():
                self._reports.clear()  # 못 나간 보고는 링크와 함께 사라진다
                return
            goal = self._active
            busy = goal is not None and not goal.done.is_set()
            if busy:
                state = RobotState.STATE_MOVING if goal.kind == "move" else RobotState.STATE_DOCKING
            else:
                state = self._rest
            x, y = noisy_pose(self._motion.x, self._motion.y, self._cfg.pose_noise_m, self._rng)
            self._reports.push(t, RobotState(
                robot_id=self.robot_id, pose=Pose2D(x=x, y=y, theta=self._motion.yaw),
                battery=90.0, state=state,
                current_goal=goal.target_id if busy and goal.kind == "move" else "",
                stamp=self.get_clock().now().to_msg()))
            due = self._reports.due(t)
        for msg in due:
            self._state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakePinkyNode()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception:  # SIGINT가 메시지 수신 중에 떨어질 수 있다. 진짜 오류면 ROS가 아직 살아 있다
        if rclpy.ok():
            raise
    finally:
        node.stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
