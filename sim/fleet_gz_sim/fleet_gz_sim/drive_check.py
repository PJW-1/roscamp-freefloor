"""관제 없이 Gazebo 핑키 한 대를 레인 위로 달리게 하고 안내 4절 지표를 잰다.

    ros2 run fleet_gz_sim drive_check            # 서 있는 정지 지점부터 한 바퀴 + heartbeat 끊기
    ros2 run fleet_gz_sim drive_check --laps 3 --no-heartbeat-cut

관제(fleet_manager)처럼 /{robot}/move_to에 목표를 보내고, 달리는 중에 경로를 늘린 새 목표를
보낸다(drive_plan.Trip). /fleet/heartbeat는 2 Hz로 보낸다. 시각은 모두 벽시계다. 지표:
  1. 경로 연장 때 정지 횟수: 연장 전 0.5 s 달리다가 연장 뒤 0.5 s 안에 cmd_vel 선속도와
     각속도가 모두 0인 채 0.1 s 넘게 이어짐
  2. 끝에서 서 있는 시간: 끝 지점 0.15 m 안에 들어온 뒤 성공까지(6 s 넘으면 실패, R13)
  3. 성공 결과 때 실제 위치(Gazebo /{robot}/sim/truth)와 목표 사이 거리(0.15 m 안, R4)
  4. heartbeat를 끊은 뒤 정지까지 시간과 이동 거리(R10)
  5. PATH_NOT_CONNECTED 결과 수(정상이면 0)
덧붙여 보고 위치(/{robot}/state, AMCL)와 실제 위치의 차이(R3 ε)도 잰다.
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import threading
import time

import rclpy
from fleet_core.common.contract import load_contract, to_ros_qos
from fleet_core.traffic.lane_map import load_lane_map
from fleet_interfaces.action import MoveTo
from fleet_interfaces.msg import RobotState
from geometry_msgs.msg import Pose, Pose2D, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header

from fleet_gz_sim.drive_plan import (
    Trip,
    full_stop_after,
    lap_stops,
    settle_time,
    speeds,
    still_episodes,
    stopped_after,
    time_to_stop,
    turn_speeds,
)

# 대기 줄 → WORK 도크 → OUTBOUND 도크 → 대기 줄. 로봇이 서 있는 지점부터 돈다
CYCLE = ("P-3", "W", "O")
TRUTH_GAP_S = 0.25  # 실제 위치 기록(약 30 Hz)이 이보다 오래 비면 그동안은 보지 못한 것이다
SETTLE_LIMIT_S = 6.0
ARRIVAL_M = 0.15
HEARTBEAT_PERIOD_S = 0.5


class DriveCheck(Node):
    def __init__(self, robot: str, contract_path: str):
        super().__init__("drive_check")
        contract = load_contract(contract_path)
        self.robot = robot
        self.lock = threading.Lock()
        self.state: RobotState | None = None
        self.truth: list[tuple[float, tuple[float, float]]] = []
        self.speed: list[tuple[float, float]] = []
        self.cmd: list[tuple[float, float, float]] = []  # (시각, 선속도, 각속도)
        self.sim_stamps: list[tuple[float, float]] = []
        self.truth_sim_t: list[float] = []
        self.truth_sim_xy: list[tuple[float, float]] = []
        self.truth_yaw: list[tuple[float, float]] = []  # (벽시계, 방향 rad)
        self.amcl_error: list[float] = []
        self.results: list[dict] = []
        self.extension_times: list[float] = []
        self.create_subscription(RobotState, contract.name("robot_state", pinky=robot),
                                 self._on_state, to_ros_qos(contract.qos("robot_state")))
        self.create_subscription(Odometry, f"/{robot}/sim/truth", self._on_truth,
                                 qos_profile_sensor_data)
        self.create_subscription(Twist, f"/{robot}/cmd_vel", self._on_cmd, 10)
        self.move = ActionClient(self, MoveTo, contract.name("move_to", pinky=robot))
        self.beat = self.create_publisher(Header, contract.name("heartbeat"),
                                          to_ros_qos(contract.qos("heartbeat")))
        self.beating = True
        self.last_beat = 0.0
        self.last_accept = 0.0
        self.create_timer(HEARTBEAT_PERIOD_S, self._send_beat,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _on_state(self, msg):
        self.state = msg
        if msg.state == RobotState.STATE_LOCALIZING:
            return
        # 보고 위치(AMCL, stamp는 위치를 잰 시뮬 시각)와 같은 시뮬 시각의 실제 위치를 비교한다(R3)
        t = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        with self.lock:
            i = bisect.bisect_left(self.truth_sim_t, t)
            near = [j for j in (i - 1, i) if 0 <= j < len(self.truth_sim_t)]
            if near:
                j = min(near, key=lambda k: abs(self.truth_sim_t[k] - t))
                if abs(self.truth_sim_t[j] - t) <= 0.05:
                    self.amcl_error.append(math.dist((msg.pose.x, msg.pose.y),
                                                     self.truth_sim_xy[j]))

    def _on_truth(self, msg):
        now = time.monotonic()
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        sim_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self.truth.append((now, (p.x, p.y)))
            self.truth_yaw.append(
                (now, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
            self.speed.append((now, math.hypot(v.x, v.y)))
            self.sim_stamps.append((now, sim_t))
            self.truth_sim_t.append(sim_t)
            self.truth_sim_xy.append((p.x, p.y))

    def _on_cmd(self, msg):
        with self.lock:
            self.cmd.append((time.monotonic(), msg.linear.x, msg.angular.z))

    def _send_beat(self):
        if self.beating:
            self.beat.publish(Header(frame_id="drive_check"))
            self.last_beat = time.monotonic()

    def pose(self) -> tuple[float, float] | None:
        s = self.state
        return None if s is None else (s.pose.x, s.pose.y)

    def true_pose(self) -> tuple[float, float] | None:
        with self.lock:
            return self.truth[-1][1] if self.truth else None

    def send(self, ids: list[str], points: list[tuple[float, float]]):
        """관제처럼 만든 목표를 보내고 수락 handle과 결과 기록 dict를 돌려준다."""
        (ax, ay), (bx, by) = points[-2], points[-1]
        goal = MoveTo.Goal(target_id=ids[-1],
                           target_pose=Pose2D(x=bx, y=by, theta=math.atan2(by - ay, bx - ax)))
        for x, y in points:
            p = Pose()
            p.position.x, p.position.y = x, y
            goal.path.append(p)
        record = {"target": ids[-1], "sent": time.monotonic(), "done": None}
        fut = self.move.send_goal_async(goal)
        end = time.monotonic() + 5.0
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.01)
        handle = fut.result() if fut.done() else None
        if handle is None or not handle.accepted:
            raise RuntimeError(f"move_to to {ids[-1]} was not accepted")
        self.last_accept = time.monotonic()

        def finished(f, record=record):
            res = f.result().result
            record.update(done=time.monotonic(), success=res.success, reason=res.reason)
        handle.get_result_async().add_done_callback(finished)
        self.results.append(record)
        return record


def wait_for(predicate, timeout: float, step: float = 0.05) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(step)
    return predicate()


def drive_trip(node: DriveCheck, lane_map, ids: list[str], cut_heartbeat: bool = False,
               extend: bool = True, timeout: float = 180.0) -> dict:
    """extend=False면 비교용으로 전체 경로를 한 번에 보내고 늘리지 않는다."""
    points = [lane_map.waypoint_pose(i) for i in ids]
    trip = Trip(points)
    path = trip.first() if extend else list(range(len(points)))
    record = node.send([ids[i] for i in path], [points[i] for i in path])
    extensions: list[float] = []
    cut: dict | None = None
    start = time.monotonic()
    while record["done"] is None and time.monotonic() - start < timeout:
        time.sleep(0.1)
        pose = node.pose()
        if cut_heartbeat and cut is None and pose is not None:
            moved = math.dist(pose, points[0])
            with node.lock:
                fast = node.speed and node.speed[-1][1] > 0.1
            if moved > 0.4 and fast:
                cut = cut_signal(node)
                continue
        if cut is not None or pose is None:
            continue
        grown = trip.update(*pose)  # 비교 주행에서도 지난 지점은 센다
        if grown and extend:
            extensions.append(time.monotonic())
            node.extension_times.append(extensions[-1])
            record = node.send([ids[i] for i in grown], [points[i] for i in grown])
    if cut is not None:
        time.sleep(2.0)  # 결과는 정지 명령 직전에 온다. 감속이 끝날 때까지 더 기록한다
    out = {"route": "→".join(ids), "extensions": len(extensions), "segment": trip.segment}
    with node.lock:
        cmd, truth, yaw = list(node.cmd), list(node.truth), list(node.truth_yaw)
        recorded_until = time.monotonic()
    linear = [(t, v) for t, v, _ in cmd]
    # 주행 중 선속도 0이 0.1 s 넘게 이어진 구간 수. 연장 주행과 한 번에 보낸 주행을 비교하면
    # 연장 때문에 더 선 횟수가 나온다(모서리 제자리 회전은 양쪽에 같이 있다)
    end = record["done"] or time.monotonic()
    out["linear_zero_episodes"] = still_episodes(linear, start, end)
    out["halt_episodes"] = still_episodes(
        [(t, max(abs(v), 0.2 * abs(w))) for t, v, w in cmd], start, end)
    zero = [stopped_after(linear, t) for t in extensions]
    halt = [full_stop_after(cmd, t) for t in extensions]
    # 안내 2.1의 지표: 연장 직후 controller가 속도 0(선속도·각속도 모두)을 낸 횟수
    out["stops_at_extension"] = sum(1 for d in halt if d)
    # 참고: 선속도만 0이 된 횟수. 모서리 제자리 회전(R12)이 0.5 s 안에 겹치면 여기 들어간다
    out["linear_zero_at_extension"] = sum(1 for d in zero if d)
    out["extensions_judged"] = sum(1 for d in halt if d is not None)
    if record["done"] is None:
        out.update(result="timeout")
        return out
    out.update(result="success" if record["success"] else record["reason"])
    if cut is not None:
        out["heartbeat_cut"] = finish_cut(cut, record, cmd, truth, yaw, recorded_until)
    elif record["success"]:
        target = points[-1]
        true_xy = min(truth, key=lambda s: abs(s[0] - record["done"]))[1]
        out["truth_error_m"] = round(math.dist(true_xy, target), 4)
        reported = node.pose()
        out["reported_error_m"] = round(math.dist(reported, target), 4) if reported else None
        settle = settle_time(truth, target, record["done"], ARRIVAL_M)
        out["end_settle_s"] = None if settle is None else round(settle, 2)
    return out


def cut_signal(node: DriveCheck) -> dict:
    """관제가 죽은 것처럼 heartbeat와 목표를 끊는다. 다음 heartbeat 차례에 끊는다."""
    wait_for(lambda: time.monotonic() - node.last_beat > HEARTBEAT_PERIOD_S - 0.05, 1.0, 0.005)
    node.beating = False
    return {"cut": time.monotonic(), "last_signal": max(node.last_beat, node.last_accept),
            "pose_at_cut": node.true_pose()}


def finish_cut(cut: dict, record: dict, cmd, truth, yaw, recorded_until: float) -> dict:
    last = cut["last_signal"]
    # cmd_vel은 선속도·각속도가 모두 0이 되어 기록 끝까지(controller는 0을 한 번 내고 멈춘다)
    # 이어진 때. 실제 위치는 0.2 s 동안 0.02 m/s 밑으로 움직이고 0.1 rad/s 밑으로 돈 때
    # (제자리 회전을 정지로 치지 않는다). 같은 메시지라 표본이 짝이 맞다
    cmd_stop = time_to_stop([(t, max(abs(v), abs(w))) for t, v, w in cmd], cut["cut"],
                            end=recorded_until)
    motion = [(t, max(v, 0.2 * w)) for (t, v), (_, w) in zip(speeds(truth), turn_speeds(yaw),
                                                             strict=True)]
    true_stop = time_to_stop(motion, cut["cut"], still=0.02, max_gap=TRUTH_GAP_S)
    stop_xy = None
    if true_stop is not None:
        stop_xy = min(truth, key=lambda s: abs(s[0] - (cut["cut"] + true_stop)))[1]
    last_xy = min(truth, key=lambda s: abs(s[0] - last))[1]
    return {
        "result": "success" if record.get("success") else record.get("reason"),
        "result_after_last_signal_s": round(record["done"] - last, 3),
        "cmd_vel_zero_after_cut_s": None if cmd_stop is None else round(cmd_stop, 3),
        "cmd_vel_zero_after_last_signal_s":
            None if cmd_stop is None else round(cut["cut"] + cmd_stop - last, 3),
        "truth_stopped_after_last_signal_s":
            None if true_stop is None else round(cut["cut"] + true_stop - last, 3),
        "moved_after_last_signal_m":
            None if stop_xy is None else round(math.dist(last_xy, stop_xy), 3),
    }


def real_time_factor(node: DriveCheck) -> float | None:
    with node.lock:
        stamps = list(node.sim_stamps)
    if len(stamps) < 2 or stamps[-1][0] - stamps[0][0] < 1.0:
        return None
    return round((stamps[-1][1] - stamps[0][1]) / (stamps[-1][0] - stamps[0][0]), 3)


def summarize(trips: list[dict], node: DriveCheck) -> dict:
    done = [t for t in trips if "truth_error_m" in t]
    cuts = [t["heartbeat_cut"] for t in trips if "heartbeat_cut" in t]
    reasons: dict[str, int] = {}
    for r in node.results:
        if r["done"] is not None and not r["success"]:
            reasons[r["reason"]] = reasons.get(r["reason"], 0) + 1
    settles = [t["end_settle_s"] for t in done if t.get("end_settle_s") is not None]
    with node.lock:
        errors = sorted(node.amcl_error)
    amcl = None
    if errors:
        amcl = {"samples": len(errors), "mean_m": round(sum(errors) / len(errors), 4),
                "p95_m": round(errors[int(0.95 * (len(errors) - 1))], 4),
                "max_m": round(errors[-1], 4)}
    return {
        "trips": len(trips),
        "arrived": len(done),
        "extensions": sum(t["extensions"] for t in trips),
        "stops_at_extension": sum(t["stops_at_extension"] for t in trips),
        "linear_zero_at_extension": sum(t["linear_zero_at_extension"] for t in trips),
        "linear_zero_episodes": sum(t["linear_zero_episodes"] for t in trips),
        "halt_episodes": sum(t["halt_episodes"] for t in trips),
        "max_end_settle_s": max(settles) if settles else None,
        "settles_over_6s": sum(1 for s in settles if s > SETTLE_LIMIT_S),
        "max_truth_error_m": max((t["truth_error_m"] for t in done), default=None),
        "arrivals_outside_0_15m": sum(1 for t in done if t["truth_error_m"] > ARRIVAL_M),
        "heartbeat_cut": cuts,
        "failed_results": reasons,
        "path_not_connected": reasons.get(MoveTo.Result.REASON_PATH_NOT_CONNECTED, 0),
        "reported_pose_error": amcl,  # 보고 위치 - 실제 위치(R3 ε, 같은 시뮬 시각끼리)
        "real_time_factor": real_time_factor(node),
    }


def run(args) -> dict:
    lane_map = load_lane_map(args.lane_map)
    node = DriveCheck(args.robot, args.contract)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        if not node.move.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("move_to server not found: is pinky_fsm running?")
        if not wait_for(lambda: node.state is not None and node.state.state == "IDLE"
                        and node.true_pose() is not None, 60.0):
            raise RuntimeError("robot is not IDLE with a pose (AMCL, Gazebo truth)")
        trips = []
        x, y = node.pose()
        start = lane_map.nearest_waypoint(x, y, waypoints=CYCLE)
        if math.dist((x, y), lane_map.waypoint_pose(start)) > ARRIVAL_M:
            raise RuntimeError(f"robot at ({x:.2f}, {y:.2f}) is not on {', '.join(CYCLE)}")
        stops = lap_stops(CYCLE, start, args.laps)
        for a, b in zip(stops, stops[1:], strict=False):
            trips.append(drive_trip(node, lane_map, lane_map.plan(a, b), extend=args.extend))
            print(json.dumps(trips[-1], ensure_ascii=False), flush=True)
            if trips[-1]["result"] != "success":
                break
            time.sleep(1.0)
        if args.heartbeat_cut and trips and trips[-1]["result"] == "success":
            ids = lane_map.plan(stops[-1], stops[1])
            trips.append(drive_trip(node, lane_map, ids, cut_heartbeat=True))
            print(json.dumps(trips[-1], ensure_ascii=False), flush=True)
            node.beating = True
            time.sleep(1.0)
            rest = ids[trips[-1]["segment"]:]  # 마지막으로 지난 지점부터 이어서 끝까지
            trips.append(drive_trip(node, lane_map, rest))
            print(json.dumps(trips[-1], ensure_ascii=False), flush=True)
        if args.dump:
            with node.lock, open(args.dump, "w", encoding="utf-8") as f:
                json.dump({"extensions": node.extension_times, "cmd": node.cmd,
                           "truth": node.truth, "results": node.results}, f)
        return summarize(trips, node)
    finally:
        executor.shutdown()
        node.destroy_node()


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--robot", default="pinky1")
    parser.add_argument("--laps", type=int, default=1)
    parser.add_argument("--no-heartbeat-cut", dest="heartbeat_cut", action="store_false")
    parser.add_argument("--whole", dest="extend", action="store_false",
                        help="비교용: 주행마다 전체 경로를 한 번에 보내고 늘리지 않는다")
    parser.add_argument("--lane-map", default=None)
    parser.add_argument("--contract", default=None)
    parser.add_argument("--json", default=None, help="요약을 이 파일에도 쓴다")
    parser.add_argument("--dump", default=None,
                        help="원자료(연장 시각, cmd_vel, 실제 위치, 결과)를 이 파일에 쓴다")
    args = parser.parse_args(argv)
    if args.contract is None:
        from ament_index_python.packages import get_package_share_directory
        args.contract = get_package_share_directory("fleet_interfaces") + "/contract.yaml"
    rclpy.init()
    try:
        summary = run(args)
    finally:
        rclpy.try_shutdown()
    text = json.dumps(summary, ensure_ascii=False, indent=2)
    print(text)
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            f.write(text + "\n")


if __name__ == "__main__":
    main()
