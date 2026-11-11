"""FakePinky: robot_state, move_to, dock을 가진 가짜 이동 로봇. 일정대로 고장을 낸다.

retreat(버전 2 1-A, 계약 0.9-draft 제안)는 보는 방향 그대로 뒤로 간다. path가 없으면 곧게,
있으면 그 꺾은선(첫 점은 로봇 자리, 끝 점은 설 곳)을 따라 시킨 속도로 간다. 뒤에 다른 로봇이나
물체가 있으면 그 자리에 서서 OBSTACLE_BEHIND로 끝낸다(자기 복구의 뒤 보기와 같은 검사).
"""
from __future__ import annotations

import math
import random
import threading
import time
from dataclasses import fields

import rclpy
from fleet_core.common.contract import to_ros_qos
from fleet_interfaces.action import Dock, MoveTo, Retreat
from fleet_interfaces.msg import RobotState
from geometry_msgs.msg import Pose2D
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Header

from fake_robots.battery import Battery
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
    drifted,
    finish_action,
    noisy_pose,
    path_gap,
)
from fake_robots.recovery import ObstacleRecovery, RecoveryParams

NAV_ABORT_DELAY_S = 0.2  # 해제 거리(0.2 m)를 지나기 전, 관제가 선을 늘리기 전에 중단한다
OFF_COURSE_M = 1.0  # 운영자가 치운 로봇이 가는 곳(지도 모서리 밖)
FEEDBACK_PERIOD_S = 0.2
CONTROL_RATE_HZ = 20.0
# 실물 pinky_fsm의 경로 물러나기와 같게(FollowPathReverse 속도, 이보다 짧게 남으면 FAILED)
REVERSE_SPEED_MPS = 0.05
REVERSE_MIN_M = 0.02
RETREAT_REASONS = (Retreat.Result.REASON_OBSTACLE_BEHIND, Retreat.Result.REASON_TIMEOUT,
                   Retreat.Result.REASON_CANCELED, Retreat.Result.REASON_FAILED)


def _project(px: float, py: float, path) -> tuple[int, float]:
    """(px, py)에 가장 가까운 선분의 번호와 그 투영점부터 path 끝까지의 길이. 실물 pinky_fsm의
    Polyline.project처럼 거리가 같으면 앞 선분이다."""
    best, i, s_at, s = math.inf, 0, 0.0, 0.0
    for k, ((ax, ay), (bx, by)) in enumerate(zip(path, path[1:], strict=False)):
        seg = math.dist((ax, ay), (bx, by))
        f = 0.0 if seg == 0.0 else min(max(
            ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / (seg * seg), 0.0), 1.0)
        d = math.dist((px, py), (ax + f * (bx - ax), ay + f * (by - ay)))
        if d < best - 1e-12:
            best, i, s_at = d, k, s + f * seg
        s += seg
    return i, s - s_at


class _Goal:
    def __init__(self, kind: str, handle):
        self.kind = kind
        self.handle = handle
        self.done = threading.Event()
        self.success = False
        self.reason = ""
        self.abort_at: float | None = None
        self.abort_from = None  # 쓴 고장 일정
        self.aborting = False  # Nav2가 이 목표를 중단하고 있다(자기 복구 중에도 계속)
        self.end_at = 0.0
        self.start_at = 0.0
        self.error = (0.0, 0.0, 0.0)
        self.target_id = ""  # target_id of a move, dock_id of a dock (current_goal)
        self.left = 0.0  # retreat: 남은 거리
        self.speed = 0.0

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
        # 충전 자리는 주차 지점이다. low_battery가 없으면 늘 90
        self._battery = Battery([f for f in scenario.faults if f.robot == self.robot_id],
                                [site.waypoint_pose(w) for w in site.parking])
        x, y = site.waypoint_pose(cfg.start)
        self._motion = Motion(x, y)
        self._reports = DelayLine(cfg.report_delay_s)
        self._off_course = (site.origin[0] - OFF_COURSE_M, site.origin[1] - OFF_COURSE_M)
        self._site = site
        self._removed = False
        # 앞 물체와 자기 복구(제안). 값은 노드 파라미터로 바꾼다(recovery_attempts 0: 복구 없음)
        self._recovery = ObstacleRecovery(RecoveryParams(**{
            f.name: self.declare_parameter(f.name, f.default).value
            for f in fields(RecoveryParams)}))
        self._others = {}  # 다른 로봇의 진실 자세. 뒤에 있으면 후진하지 않는다
        self._sc = ScenarioClock(self)
        self._lock = threading.RLock()
        self._alive = True
        self._active: _Goal | None = None
        self._goals: dict[bytes, _Goal] = {}
        self._rest = RobotState.STATE_IDLE
        self._last_t = self._sc.now()
        params = load_params_param(self)
        timeout = float(self.get_parameter("heartbeat_timeout_s").value)
        self._beat_timeout = timeout if timeout >= 0 else float(params["heartbeat_timeout_s"])
        # 지시서 7.5: path가 로봇 자리에서 이보다 멀면 따라가지 않는다(관제 lane_error_warn_m)
        self._connect_tolerance = float(params["lane_error_warn_m"])
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
        for other in scenario.robots:
            if other != self.robot_id:
                self.create_subscription(
                    RobotState, TRUTH_TOPIC.format(robot=other), self._on_other,
                    to_ros_qos(contract.qos("robot_state")))
        group = ReentrantCallbackGroup()
        self._move_server = self._server(
            MoveTo, contract.name("move_to", pinky=self.robot_id), "move", group)
        self._dock_server = self._server(
            Dock, contract.name("dock", pinky=self.robot_id), "dock", group)
        self._retreat_server = self._server(
            Retreat, contract.name("retreat", pinky=self.robot_id), "retreat", group)
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
        self._retreat_server.destroy()
        return super().destroy_node()

    def _running(self) -> bool:
        return self._alive and rclpy.ok()

    def _on_other(self, msg: RobotState) -> None:
        self._others[msg.robot_id] = (msg.pose.x, msg.pose.y)

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
        if (goal.abort_at is not None and not goal.done.is_set()
                and reason != MoveTo.Result.REASON_NAV_ABORTED):
            self._faults.give_back_nav_abort(goal.abort_from)  # 대체·취소되면 다음 목표로
        if goal.kind == "retreat" and not success and reason not in RETREAT_REASONS:
            reason = Retreat.Result.REASON_FAILED  # 계약 0.9: Retreat reason은 이 넷뿐이다
        goal.finish(success, reason)
        if self._active is goal:
            self._rest = (RobotState.STATE_DOCKED if goal.kind == "dock" and success
                          else RobotState.STATE_IDLE)
            self._motion.clear_target()
            self._recovery.restart()  # 후진과 RECOVERING을 멈춘다(명세 7번)

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
                path = [(p.position.x, p.position.y) for p in handle.request.path]
                if path and self._path_misses_me(t, path):
                    # 지시서 7.5: 따라가지 않고 실패한다. 이전 목표는 위에서 끝났으니 선다(R6)
                    self._end(goal, False, MoveTo.Result.REASON_PATH_NOT_CONNECTED)
                else:
                    # path: 첫 점은 로봇 자리라 건너뛰고, 끝 점은 target_pose다
                    self._motion.set_target(*noisy_pose(pose.x, pose.y, self._cfg.stop_error_m,
                                                        self._rng), via=path[1:-1],
                                            start=path[0] if path else None)
                    if self._faults.take_nav_abort(t, goal.target_id):
                        eta = self._motion.remaining() / self._cfg.speed_mps
                        goal.abort_at = t + min(NAV_ABORT_DELAY_S, eta / 2)  # 도착 전에 중단
                        goal.abort_from = self._faults.last_taken
            elif kind == "retreat":
                self._start_retreat(t, goal, handle.request)
            else:
                goal.target_id = handle.request.dock_id
                self._motion.clear_target()
                goal.end_at = t + self._dock_cfg.duration_s
            self._goals[bytes(handle.goal_id.uuid)] = goal
        handle.execute()

    def _start_retreat(self, t: float, goal: _Goal, request) -> None:
        """보는 방향 그대로 뒤로. 갈 길을 지나온 선(trail)으로 두고 Motion.back으로 간다.

        path가 없으면 뒤로 곧게 distance_m을 시킨 속도로. 있으면 실물 pinky_fsm처럼 제 자리에서
        가장 가까운 선분부터 끝 점까지 차례로, 0.05 m/s를 넘지 않게 간다. 로봇 자리에서 시작하지
        않는 path(이동의 path와 같은 연결 허용 거리)나 제 자리를 투영한 점부터 끝까지가 0.02 m보다
        짧은 path는 따르지 않는다. 투영은 실물(Polyline.project)과 같이 같으면 앞 선분이다.
        """
        distance, speed = float(request.distance_m), float(request.speed_mps)
        path = [(p.position.x, p.position.y) for p in request.path]
        m = self._motion
        rest = math.inf
        if path:
            i, rest = _project(m.x, m.y, path)
            ahead = path[i + 1:]
            speed = min(speed, REVERSE_SPEED_MPS)
        else:
            ahead = [(m.x - math.cos(m.yaw) * distance, m.y - math.sin(m.yaw) * distance)]
        length = sum(math.dist(a, b) for a, b in zip([(m.x, m.y), *ahead], ahead, strict=False))
        if (distance <= 0.0 or speed <= 0.0 or length <= 1e-9 or rest < REVERSE_MIN_M
                or (path and self._path_misses_me(t, path))):
            self._end(goal, False, Retreat.Result.REASON_FAILED)
            return
        m.clear_target()
        m.trail = list(reversed(ahead))  # Motion.back은 trail 끝에서부터 되짚는다
        goal.left, goal.speed = length, speed

    def _retreat_step(self, t: float, dt: float, goal: _Goal) -> None:
        radius = self._recovery.p.robot_radius_m
        wall = self._faults.behind(t)
        others = [(x, y, radius) for x, y in list(self._others.values())]
        # 자기 복구의 뒤 보기와 같은 검사(recovery.py). 남은 길 전체에서 몸이 닿는가
        if ((wall is not None and wall <= goal.left + radius)
                or self._recovery._something_behind(self._motion, goal.left, others)):
            self._end(goal, False, Retreat.Result.REASON_OBSTACLE_BEHIND)
            return
        step = min(goal.speed * dt, goal.left)
        self._motion.back(step)
        goal.left -= step
        if goal.left <= 1e-9:
            self._end(goal, True)

    def _path_misses_me(self, t: float, path) -> bool:
        """받은 path가 로봇이 아는 제 자리에서 연결 허용 거리보다 먼가(지시서 7.5).

        로봇이 보고하는 자리로 잰다. drift는 들어가고 잡음은 빠진다. 실물 핑키도 제가 아는
        자리, 곧 보고하는 자리로 잰다. path가 비면(point 모드) 부르지 않는다.
        """
        x, y = drifted(self._motion.x, self._motion.y, self._motion.yaw, self._faults.drift(t))
        return path_gap(x, y, path) > self._connect_tolerance

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
        if action_type is Retreat:
            return Retreat.Result(success=goal.success, reason=goal.reason)
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
            moving = goal is not None and not goal.done.is_set() and goal.kind == "move"
            self._recovery.watch(t, self._faults.obstacle(t), moving, self._motion)
            if dt <= 0 or goal is None or goal.done.is_set() or self._faults.stuck(t):
                return
            if self._heartbeat_lost():
                self._end(goal, False, MoveTo.Result.REASON_HEARTBEAT_LOST)
                return
            if goal.kind == "move":
                if goal.abort_at is not None and t >= goal.abort_at:
                    if self._recovery.nav_aborted(t, self._motion, self._faults.behind(t)):
                        self._end(goal, False, MoveTo.Result.REASON_NAV_ABORTED)
                        return
                    # 시도를 시작했다. 고장은 이미 났으니 취소돼도 돌려주지 않는다
                    goal.abort_at, goal.abort_from, goal.aborting = None, None, True
                if (goal.aborting and goal.abort_at is None
                        and not self._recovery.backing_up):
                    goal.abort_at = t + NAV_ABORT_DELAY_S  # 다시 보낸 FollowPath도 중단된다
                # 물체가 있으면 선다. 시도가 남았으면 후진하고 다시 가 보고, 없으면 BLOCKED
                radius = self._recovery.p.robot_radius_m
                self._recovery.drive(t, dt, self._motion, self._cfg.speed_mps,
                                     [(x, y, radius) for x, y in list(self._others.values())],
                                     self._faults.behind(t))
                if not self._motion.moving:
                    self._recovery.succeeded()
                    self._end(goal, True)
            elif goal.kind == "retreat":
                self._retreat_step(t, dt, goal)
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
        self._recovery.forget()  # 앞 물체는 들어낸 자리에 두고 간다

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
            battery = self._battery.percent(t, self._motion.x, self._motion.y,
                                            not self._motion.moving)
            if t < self._cfg.localizing_s:
                # 위치를 아직 못 잡았다. 자세는 아무 값이다.
                self._reports.push(t, RobotState(
                    robot_id=self.robot_id, pose=Pose2D(), battery=battery,
                    state=RobotState.STATE_LOCALIZING, stamp=self.get_clock().now().to_msg()))
            else:
                self._push_report(t, goal, busy, battery)
            due = self._reports.due(t)
        for msg in due:
            self._state_pub.publish(msg)

    def _push_report(self, t: float, goal: _Goal | None, busy: bool, battery: float) -> None:
        if busy and goal.kind == "move" and self._recovery.recovering:
            state = RobotState.STATE_RECOVERING  # 명세 8번: LOCALIZING 다음
        elif self._recovery.blocked:
            state = RobotState.STATE_BLOCKED
        elif busy:
            state = RobotState.STATE_DOCKING if goal.kind == "dock" else RobotState.STATE_MOVING
        else:
            state = self._rest
        x, y = noisy_pose(self._motion.x, self._motion.y, self._cfg.pose_noise_m, self._rng)
        x, y = drifted(x, y, self._motion.yaw, self._faults.drift(t))  # 진실 토픽은 그대로
        msg = RobotState(
            robot_id=self.robot_id, pose=Pose2D(x=x, y=y, theta=self._motion.yaw),
            battery=battery, state=state,
            current_goal=goal.target_id if busy else "",  # Pinky contract section 2
            stamp=self.get_clock().now().to_msg())
        if state in (RobotState.STATE_BLOCKED, RobotState.STATE_RECOVERING):
            seen = self._recovery.obstacle_ahead(self._motion)
            if seen is not None:
                msg.obstacle_seen = True
                msg.obstacle_x, msg.obstacle_y, msg.obstacle_radius = seen
        self._reports.push(t, msg)


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
