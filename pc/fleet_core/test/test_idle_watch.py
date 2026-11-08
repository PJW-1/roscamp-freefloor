"""조용히 서 있는 로봇 감시(1.1 지시서 H2): 목적지가 있는데 goal도 구역 대기표도 없이 선 로봇.

재현 1: 특징 지도에서 O→F0 goal이 F0 0.13 m 앞에서 성공으로 끝나면, 관제는 도착을 받아들이고
(arrival_tolerance_m 안) O를 놓지 못해(바닥 기준) 다음 구역을 청하지 않는다. goal이 없으니 멈춤
감시도 돌지 않는다. 감시는 idle_watch_s 뒤 ROBOT_IDLE을 한 번 내고 그 지점 goal을 다시 보낸다.
"""
import math
from dataclasses import replace
from pathlib import Path

from fleet_core.common.commands import EventOut, MoveCmd
from fleet_core.common.params import FleetParams, load_params
from fleet_core.traffic.controller import TrafficController
from fleet_core.traffic.lane_map import load_lane_map
from fleet_core.traffic.mission import PARKED, TO_PARK

REPO = Path(__file__).resolve().parents[3]
FEATURE = REPO / "sim/fake_robots/maps/feature_map.yaml"
PARAMS = load_params()
DT = 0.1
SPEED = 0.2


class Drive:
    """로봇이 받은 선을 따라 0.2 m/s로 간다. short[(로봇, 지점)] m 앞에서 서고 성공을 보낸다.

    한 번만 그렇게 서고, keep_short이면 매번 그렇게 선다.
    """

    def __init__(self, starts, short=None, keep_short=False, **over):
        self.now = [0.0]
        self.site = load_lane_map(FEATURE)
        self.core = TrafficController(self.site, replace(PARAMS, **over), list(starts),
                                      lambda: self.now[0])
        self.pose = {r: self.site.waypoint_pose(w) for r, w in starts.items()}
        self.goal = {r: None for r in starts}  # (goal id, 지점 id, 남은 점들)
        self.short = dict(short or {})
        self.keep_short = keep_short
        self.out = []
        for robot in starts:
            self.report(robot)
        self.core.on_run_mode("RUN")

    def report(self, robot):
        state = "MOVING" if self.goal[robot] else "IDLE"
        self.core.on_robot_state(robot, *self.pose[robot], 0.0, state)

    def tick(self):
        for robot in self.pose:
            self.move(robot)
            self.report(robot)
        for item in self.core.tick():
            self.out.append((self.now[0], item))
            if isinstance(item, MoveCmd):
                points = [p[:2] for p in item.path[1:]] or [(item.x, item.y)]
                self.goal[item.robot] = (item.goal_id, item.target_id, list(points))
        self.now[0] = round(self.now[0] + DT, 6)

    def move(self, robot):
        if self.goal[robot] is None:
            return
        goal_id, target, points = self.goal[robot]
        stop_short = self.short.get((robot, target), 0.0)
        step = SPEED * DT
        x, y = self.pose[robot]
        while points and step > 0:
            px, py = points[0]
            gap = math.dist((x, y), (px, py))
            last = len(points) == 1
            if last and stop_short and gap - step <= stop_short:
                ux, uy = (px - x) / gap, (py - y) / gap
                x, y = px - ux * stop_short, py - uy * stop_short
                points.clear()
                if not self.keep_short:
                    del self.short[(robot, target)]
                break
            if gap <= step:
                x, y = px, py
                points.pop(0)
                step -= gap
            else:
                x, y = x + (px - x) / gap * step, y + (py - y) / gap * step
                step = 0
        self.pose[robot] = (x, y)
        if not points:
            self.goal[robot] = None
            self.report(robot)
            self.core.on_move_result(robot, goal_id, True, "")

    def run(self, seconds):
        for _ in range(round(seconds / DT)):
            self.tick()

    def run_until(self, done, seconds):
        for _ in range(round(seconds / DT)):
            self.tick()
            if done():
                return True
        return False

    def moves(self, robot, after=-1.0):
        return [(t, c) for t, c in self.out
                if isinstance(c, MoveCmd) and c.robot == robot and t > after]

    def events(self, code):
        return [(t, c) for t, c in self.out if isinstance(c, EventOut) and c.code == code]

    def held(self, robot):
        return set(self.core.zones.held_by(robot))


def short_of_f0(keep_short=False):
    """r1이 O에서 대기 줄로 가다 F0 0.13 m 앞에서 성공으로 선다. 그 시각을 돌려준다."""
    drive = Drive({"r1": "O"}, short={("r1", "F0"): 0.13}, keep_short=keep_short)
    m = drive.core._missions["r1"]
    assert drive.run_until(lambda: drive.moves("r1"), 1.0)
    assert m.phase == TO_PARK and m.path[:3] == ["O", "F0", "F1"]
    assert drive.moves("r1")[0][1].target_id == "F0"
    assert drive.run_until(lambda: drive.goal["r1"] is None, 5.0)
    return drive, drive.now[0]


# ---- 값 ------------------------------------------------------------------------------------


def test_the_values():
    assert PARAMS.idle_watch_s == 10.0


def test_a_record_without_the_value_replays_without_the_watch():
    fields = {f: getattr(PARAMS, f) for f in PARAMS.__dataclass_fields__ if f != "idle_watch_s"}
    assert FleetParams(**fields).idle_watch_s == 0.0


# ---- 재현 1: 덜 가서 섬 ---------------------------------------------------------------------


def test_short_of_f0_the_arrival_is_taken_and_o_is_kept():
    drive, stopped = short_of_f0()
    drive.run(2.0)
    m = drive.core._missions["r1"]
    assert m.path[m.idx] == "F0" and m.goal is None
    assert drive.held("r1") == {"O", "F0"}
    assert drive.core.zones.waiting_for("r1") is None


def test_a_robot_short_of_f0_is_reported_once_and_sent_on_to_f0_then_parks():
    drive, stopped = short_of_f0()
    assert drive.run_until(lambda: drive.events("ROBOT_IDLE"), 30.0)
    (when, event), = drive.events("ROBOT_IDLE")
    assert stopped + PARAMS.idle_watch_s - DT <= when <= stopped + PARAMS.idle_watch_s + 0.3
    assert event.level == "WARN" and event.robot == "r1"
    for word in ("TO_PARK", "'O'", "'F0'", "SUCCEEDED"):
        assert word in event.detail
    again = drive.moves("r1", after=stopped)
    assert [(t, c.target_id) for t, c in again] == [(when, "F0")]
    assert drive.run_until(lambda: drive.core._missions["r1"].phase == PARKED, 120.0)
    assert "O" not in drive.held("r1")
    assert len(drive.events("ROBOT_IDLE")) == 1
    assert not drive.events("GOAL_FAILED")


def test_without_the_watch_the_robot_stands_with_o_and_f0_for_two_minutes():
    """idle_watch_s 0(기록 재생)은 예전대로다. 재현 그대로 120 s 동안 명령이 없다."""
    drive = Drive({"r1": "O"}, short={("r1", "F0"): 0.13}, idle_watch_s=0.0)
    assert drive.run_until(lambda: drive.moves("r1"), 1.0)
    assert drive.run_until(lambda: drive.goal["r1"] is None, 5.0)
    stopped = drive.now[0]
    drive.run(120.0)
    assert not drive.moves("r1", after=stopped)
    assert drive.held("r1") == {"O", "F0"}
    assert not drive.events("ROBOT_IDLE")


def test_a_robot_that_keeps_stopping_short_is_counted_and_isolated_past_the_retry_limit():
    """다시 보낸 goal도 같은 자리에서 성공으로 끝나면 시도로 이어 센다(성공이 수를 지우지 않는다).

    한도를 넘으면 GOAL_FAILED로 격리한다.
    """
    drive, stopped = short_of_f0(keep_short=True)
    m = drive.core._missions["r1"]
    assert drive.run_until(lambda: drive.events("GOAL_FAILED"), 120.0)
    assert m.fault == "ERROR" and m.fault_reason == "ROBOT_IDLE"
    assert len(drive.events("ROBOT_IDLE")) == PARAMS.move_retry_limit + 1
    assert len(drive.moves("r1", after=stopped)) == PARAMS.move_retry_limit
    assert drive.events("ROBOT_ISOLATED")
    assert drive.held("r1") == {"O", "F0"}  # 격리된 로봇은 구역을 쥔다


# ---- 대상이 아닌 로봇 ------------------------------------------------------------------------


def test_a_stop_or_a_pause_is_not_idle():
    drive, stopped = short_of_f0()
    for mode in ("STOP", "PAUSE"):
        drive.core.on_run_mode(mode)
        drive.run(30.0)
        assert not drive.events("ROBOT_IDLE")
    drive.core.on_run_mode("RUN")
    assert drive.run_until(lambda: drive.events("ROBOT_IDLE"), PARAMS.idle_watch_s + 1.0)


def test_a_robot_waiting_in_a_zone_queue_is_not_idle():
    drive = Drive({"r1": "O", "r2": "F0"})
    drive.core.on_robot_fault("r2", "test")  # r2가 F0에 서서 움직이지 않는다
    drive.run(40.0)
    assert drive.core.zones.waiting_for("r1") == "F0"
    assert not drive.events("ROBOT_IDLE")


def test_a_parked_robot_or_one_at_a_station_is_not_idle():
    drive = Drive({"r1": "P-3"})
    drive.run(60.0)
    assert drive.core._missions["r1"].phase == PARKED
    assert not drive.events("ROBOT_IDLE")


def test_a_robot_in_a_deadlock_halt_is_not_watched():
    """교착 멈춤 동안은 교착 처리 몫이다(H1에서 순환에 든 로봇만으로 좁힌다)."""
    drive, stopped = short_of_f0()
    drive.core._deadlock_since = drive.now[0]
    drive.core._check_deadlock = lambda now: True
    drive.run(30.0)
    assert not drive.events("ROBOT_IDLE")
