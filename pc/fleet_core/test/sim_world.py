"""가짜 시계로 FleetCore를 구동하는 결정적 가상 창고.

로봇은 직선으로 달리며 잡음 섞인 자세를 보고한다. 가짜 중앙 서버가 주문판을 관리한다.
장애 일정으로 통신 두절, 내비 중단, 도킹 실패, 정지, 앞 물체(막힘), 보고 밀림,
로봇에서 떨어진 path(bad_path), 팔 실패와 무응답, 배터리 부족(low_battery)을 넣는다.
로봇은 핑키처럼 받은 path가 제 자리를 지나는지 본다(지시서 7.5). 멀면 PATH_NOT_CONNECTED.
매 단계 안전 불변식을 검사하므로 이 세계를 쓰는 테스트는 모두 불변식도 검증한다.
"""
from __future__ import annotations

import itertools
import math
import random
from dataclasses import dataclass, field

from fleet_core.common.commands import (
    HELD,
    CancelCmd,
    DockCmd,
    EventOut,
    FillCmd,
    MoveCmd,
    OrderRecord,
    ReportOrderCmd,
    SwapCmd,
)
from fleet_core.common.params import load_params
from fleet_core.traffic.lane_map import _point_segment
from grid_map import grid_site
from legacy_core import FleetCore

DT = 0.1
NAV_ABORT_AFTER_S = 0.5
NAV_TIMEOUT_AFTER_S = 2.0
BLOCK_REPORT_S = 1.0  # 물체 앞에서 이만큼 서 있으면 BLOCKED(핑키 계약 4절: 2초 안)
BLOCK_CLEAR_S = 1.0  # 물체가 이만큼 안 보이면 BLOCKED를 끄고 이어 간다
LINE_TOLERANCE_M = 1e-6

DEFAULT_ROBOTS = {
    "pinky1": {"start": "P-6", "speed_mps": 0.20, "pose_noise_m": 0.03},
    "pinky2": {"start": "P-5", "speed_mps": 0.18, "pose_noise_m": 0.03},
    "pinky3": {"start": "P-4", "speed_mps": 0.20, "pose_noise_m": 0.03},
}
DEFAULT_STOCK = {"A": 20, "B": 20, "C": 20, "D": 20}


def path_gap(x, y, points):
    """(x, y)에서 꺾은선 points까지의 가장 짧은 거리. 점이 하나면 그 점까지."""
    if len(points) == 1:
        return math.dist((x, y), points[0])
    return min(_point_segment(x, y, *a, *b) for a, b in zip(points, points[1:], strict=False))


def six_orders(at_s=0.0):
    return [{"at_s": at_s, "id": f"O-{i:03d}", "items": {"AB"[i % 2]: 1, "CD"[i % 2]: 1}}
            for i in range(1, 7)]


@dataclass
class VirtualRobot:
    name: str
    x: float
    y: float
    speed: float
    noise: float
    goal_id: int | None = None
    goal_kind: str = ""  # 이동 또는 도킹
    target: tuple[float, float] | None = None
    goal_elapsed: float = 0.0
    dock_left: float = 0.0
    docked_at: str = ""
    dock_station: str = ""
    has_basket: bool = True
    basket: dict = field(default_factory=dict)  # 실제로 들어 있는 것
    comm_up: bool = True
    removed: bool = False
    stuck: bool = False
    obstacle_until: float | None = None  # 이 시각까지 경로 앞에 물체
    stopped_at: float | None = None  # 물체 때문에 선 시각, 풀릴 때까지
    clear_at: float | None = None  # 물체가 사라진 시각
    blocked: bool = False  # BLOCKED로 보고한다
    localizing_until: float = 0.0  # 이 시각까지 위치를 못 잡아 LOCALIZING으로 보고
    nav_aborts: int = 0
    dock_fails: int = 0
    via: list = field(default_factory=list)  # 목표 전에 거칠 점(path 모드)
    heading: float = 0.0  # 마지막 진행 방향
    drift: float = 0.0  # 보고를 진행 방향 왼쪽으로 미는 거리, 실제 위치는 그대로
    bad_paths: int = 0  # 다음 path 목표 몇 개를 옆으로 밀린 채 받는다
    bad_path_offset: float = 0.0


@dataclass
class ArmJob:
    cmd: object
    next_at: float
    left: list = field(default_factory=list)  # 담기: 아직 시도할 품목
    placed: dict = field(default_factory=dict)
    missing: dict = field(default_factory=dict)
    fail_after: int | None = None  # 담기: 이만큼 놓은 뒤 실패
    steps: list = field(default_factory=list)  # 교환: 남은 "take", "put" 단계
    took_full: bool = False
    put_empty_done: bool = False
    fail_after_take: bool = False  # 교환: put 단계가 실패
    fail_now: bool = False  # 교환: 시작 전에 실패
    silent: bool = False  # 응답이 없고 취소도 듣지 못함


@dataclass
class SimWorld:
    seed: int = 7
    orders: list = field(default_factory=six_orders)
    stock: dict = field(default_factory=lambda: dict(DEFAULT_STOCK))
    robots: dict = field(default_factory=lambda: dict(DEFAULT_ROBOTS))
    faults: list = field(default_factory=list)
    run_mode: str = "RUN"
    arm_s_per_item: float = 2.0
    swap_step_s: float = 2.0
    dock_s: float = 2.0
    params: object = None
    gaussian_noise: bool = False  # 한계 없는 잡음. 실제 위치추정과 ROS 가짜 노드가 그렇다
    site: object = None  # 예전 테스트는 셀 배치를 쓴다

    def __post_init__(self):
        self.t = 0.0
        self.rng = random.Random(self.seed)
        self.site = self.site or grid_site()
        self.params = self.params or load_params()
        self.total_items = sum(self.stock.values())
        self.shipped_items: dict[str, int] = {}
        self.future_orders = sorted(self.orders, key=lambda o: o["at_s"])
        self.registered: list[OrderRecord] = []
        self.registration: list[str] = []
        self.reports: list[ReportOrderCmd] = []
        self._unprocessed_reports: list[ReportOrderCmd] = []
        self.jobs: dict[str, ArmJob] = {}
        self.arm_faults = {"fill_fail": [], "fill_silent": 0, "swap_fail": 0, "swap_partial": 0,
                           "swap_silent": 0}
        self.pending_faults = sorted(self.faults, key=lambda f: f["at_s"])
        self.reported: dict[str, tuple[float, float]] = {}
        self.virtual = {}
        for name, spec in self.robots.items():
            cell = spec.get("cell") or min(self.site.zones[spec["start"]])
            x, y = self.site.waypoint_pose(cell)
            self.virtual[name] = VirtualRobot(name, x, y, spec["speed_mps"], spec["pose_noise_m"],
                                              localizing_until=spec.get("localizing_s", 0.0))
        self.core = FleetCore(self.site, self.params, list(self.robots), lambda: self.t)
        self.events: list[EventOut] = []
        self.event_log: list[tuple[float, EventOut]] = []  # (시각, 이벤트)
        self.commands: list[tuple[float, object]] = []
        self.visits: list[tuple[float, str, tuple[int, int], str]] = []
        self._last_cell: dict[str, tuple[int, int]] = {}
        self._queued_results: list = []
        self.min_true_release_distance = float("inf")
        self.path_refusals: list[tuple[float, str, int]] = []  # (시각, 로봇, goal_id)
        self.batteries = self._battery_models()

    def _battery_models(self) -> dict:
        """low_battery가 있는 로봇만 배터리 모형을 둔다. 나머지는 예전처럼 배터리를 안 보낸다."""
        low = [f for f in self.faults if f["type"] == "low_battery"]
        if not low:
            return {}
        from fake_robots.battery import Battery  # 이 고장을 쓰는 세계만 fake_robots가 필요하다
        spots = [self.site.waypoint_pose(w) for w in getattr(self.site, "parking", ())]
        return {name: Battery([f for f in low if f["robot"] == name], spots)
                for name in sorted({f["robot"] for f in low})}

    # ---- public helpers for tests -------------------------------------------------------

    def step(self):
        self.t = round(self.t + DT, 6)
        self._fire_faults()
        self._watch_obstacles()
        self._deliver_queued_results()
        self._move_robots()
        self._run_arms()
        self._report_states()
        self._central()
        before = self.core.zones.snapshot()
        out = self.core.tick()
        self._check_releases(before)
        for item in out:
            self._apply(item)
        self._check_invariants()

    def run(self, seconds):
        for _ in range(round(seconds / DT)):
            self.step()

    def run_until(self, predicate, timeout_s=1500.0):
        """조건이 참이 될 때까지 진행한다. 시간이 넘으면 False."""
        for _ in range(round(timeout_s / DT)):
            self.step()
            if predicate():
                return True
        return False

    def confirm_clear(self, robot, zone_ids, note="operator"):
        """운영자가 로봇을 치우고 잠금을 푼다."""
        self.virtual[robot].removed = True
        return self.core.confirm_clear(robot, zone_ids, note)

    def state(self, robot):
        return self.core.robot_view(robot)["state"]

    def all_parked(self, robots=None):
        names = robots or list(self.robots)
        return all(self.state(r) == "PARKED" for r in names)

    def all_orders_final(self):
        return not self.future_orders and not self.registered and not self._unprocessed_reports

    def done(self):
        return self.all_orders_final() and self.all_parked()

    def report_statuses(self):
        return {r.order_id: r.status for r in self.reports}

    def bound_order(self):
        """ORDER_BOUND 순서의 주문 id. 모든 실행에서 등록 순서와 같아야 한다."""
        return [e.detail for e in self.events if e.code == "ORDER_BOUND"]

    def shipped_order(self):
        return [r.order_id for r in self.reports if r.status != HELD]

    def event_codes(self, level=None):
        return [e.code for e in self.events if level is None or e.level == level]

    def commands_of(self, kind):
        return [c for _, c in self.commands if isinstance(c, kind)]

    # ---- faults -------------------------------------------------------------------------

    def _fire_faults(self):
        while self.pending_faults and self.pending_faults[0]["at_s"] <= self.t:
            fault = self.pending_faults.pop(0)
            kind = fault["type"]
            robot = self.virtual.get(fault.get("robot", ""))
            if kind == "comm_loss":
                robot.comm_up = False
                self.pending_faults.append({
                    "at_s": self.t + fault["duration_s"], "type": "comm_back",
                    "robot": robot.name})
                self.pending_faults.sort(key=lambda f: f["at_s"])
            elif kind == "comm_back":
                robot.comm_up = True
            elif kind == "drift":
                robot.drift = fault["offset_m"]
                self.pending_faults.append({
                    "at_s": self.t + fault["duration_s"], "type": "drift_end",
                    "robot": robot.name})
                self.pending_faults.sort(key=lambda f: f["at_s"])
            elif kind == "drift_end":
                robot.drift = 0.0
            elif kind == "bad_path":
                robot.bad_paths = fault.get("count", 1)
                robot.bad_path_offset = fault["offset_m"]
            elif kind == "nav_abort":
                robot.nav_aborts = fault["count"]
            elif kind == "dock_fail":
                robot.dock_fails = fault["count"]
            elif kind == "stuck":
                robot.stuck = True
            elif kind == "blocked":
                robot.obstacle_until = self.t + fault["duration_s"]
            elif kind == "fill_fail":
                self.arm_faults["fill_fail"] += [fault.get("place", 0)] * fault["count"]
            elif kind in ("fill_silent", "swap_fail", "swap_partial", "swap_silent"):
                self.arm_faults[kind] += fault.get("count", 1)
            elif kind == "run_mode":
                self.run_mode = fault["mode"]
            elif kind == "low_battery":
                pass  # 배터리 모형(self.batteries)이 시각을 직접 본다
            else:
                raise ValueError(kind)

    def _watch_obstacles(self):
        """move 중 물체 앞에서 서면 1초 뒤 BLOCKED. 물체가 1초 안 보이면 끄고 이어 간다.

        취소된 뒤에도 물체가 있는 동안은 BLOCKED다. 창이 열릴 때 move가 없으면 다음 move에 걸린다.
        """
        for r in self.virtual.values():
            if r.obstacle_until is not None and self.t < r.obstacle_until:
                r.clear_at = None
                if r.goal_kind == "move" and r.stopped_at is None:
                    r.stopped_at = self.t
                if r.stopped_at is not None and self.t - r.stopped_at >= BLOCK_REPORT_S:
                    r.blocked = True
                continue
            r.obstacle_until = None
            if r.stopped_at is None and not r.blocked:
                continue
            if r.clear_at is None:
                r.clear_at = self.t
            if self.t - r.clear_at >= BLOCK_CLEAR_S:
                r.blocked = False
                r.stopped_at = r.clear_at = None

    # ---- central stand-in ----------------------------------------------------------------

    def _central(self):
        finals = {r.order_id for r in self._unprocessed_reports}
        self._unprocessed_reports = []
        self.registered = [o for o in self.registered if o.order_id not in finals]
        while self.future_orders and self.future_orders[0]["at_s"] <= self.t:
            order = self.future_orders.pop(0)
            self.registered.append(OrderRecord(order["id"], dict(order["items"])))
            self.registration.append(order["id"])
        self.core.on_order_board(self.run_mode, list(self.registered))

    # ---- physics ------------------------------------------------------------------------

    def _deliver_queued_results(self):
        queued, self._queued_results = self._queued_results, []
        for robot, fn, args in queued:
            if robot is None or self.virtual[robot].comm_up:  # 팔은 자기 링크가 따로 있다
                fn(*args)

    def _move_robots(self):
        for r in self.virtual.values():
            if r.removed or r.goal_id is None:
                continue
            r.goal_elapsed += DT
            if r.goal_kind == "dock":
                self._drive_dock(r)
            else:
                self._drive_move(r)

    def _finish_goal(self, r, callback, *args):
        goal_id = r.goal_id
        r.goal_id = None
        r.goal_kind = ""
        r.target = None
        r.via = []
        if r.comm_up:
            callback(r.name, goal_id, *args)

    def _drive_move(self, r):
        if r.stuck:
            if r.goal_elapsed >= NAV_TIMEOUT_AFTER_S:
                self._finish_goal(r, self.core.on_move_result, False, "TIMEOUT")
            return
        if r.stopped_at is not None:
            return  # 물체 앞에서 goal을 쥔 채 서 있다
        if r.nav_aborts > 0 and r.goal_elapsed >= NAV_ABORT_AFTER_S:
            r.nav_aborts -= 1
            self._finish_goal(r, self.core.on_move_result, False, "NAV_ABORTED")
            return
        step = r.speed * DT
        while r.via:  # 선분을 차례로 따른다
            gap = math.dist((r.x, r.y), r.via[0])
            if gap > step:
                break
            r.x, r.y = r.via.pop(0)
            step -= gap
        aim = r.via[0] if r.via else r.target
        dx, dy = aim[0] - r.x, aim[1] - r.y
        dist = math.hypot(dx, dy)
        if dist > 0:
            r.heading = math.atan2(dy, dx)
        if dist <= step and not r.via:
            r.x, r.y = r.target
            self._finish_goal(r, self.core.on_move_result, True, "")
        elif dist > 0:
            r.x += dx / dist * min(step, dist)
            r.y += dy / dist * min(step, dist)

    def _drive_dock(self, r):
        r.dock_left -= DT
        if r.dock_left > 0:
            return
        if r.dock_fails > 0:
            r.dock_fails -= 1
            self._finish_goal(r, self.core.on_dock_result, False, 5.0, 5.0, 5.0)
        else:
            r.docked_at = r.dock_station
            self._finish_goal(r, self.core.on_dock_result, True, 0.4, 0.3, 0.5)

    def _run_arms(self):
        for server, job in list(self.jobs.items()):
            if self.t >= job.next_at:
                if isinstance(job.cmd, FillCmd):
                    self._run_fill(server, job)
                else:
                    self._run_swap(server, job)

    def _run_fill(self, server, job):
        robot = self.virtual[job.cmd.robot]
        if job.fail_after is not None and sum(job.placed.values()) >= job.fail_after:
            del self.jobs[server]
            self.core.on_fill_result(job.cmd.goal_id, False, dict(job.placed), {}, "GRIPPER")
            return
        if job.left:
            kind = job.left.pop(0)
            if self.stock.get(kind, 0) > 0:
                self.stock[kind] -= 1
                robot.basket[kind] = robot.basket.get(kind, 0) + 1
                job.placed[kind] = job.placed.get(kind, 0) + 1
            else:
                job.missing[kind] = job.missing.get(kind, 0) + 1
            job.next_at = self.t + self.arm_s_per_item
            return
        del self.jobs[server]
        if not job.silent:
            self.core.on_fill_result(job.cmd.goal_id, True, dict(job.placed), dict(job.missing))

    def _run_swap(self, server, job):
        robot = self.virtual[job.cmd.robot]
        if job.fail_now:
            del self.jobs[server]
            self.core.on_swap_result(job.cmd.goal_id, False, False, False, "GRIPPER")
            return
        if job.steps and job.steps[0] == "take":
            job.steps.pop(0)
            for kind, count in robot.basket.items():
                self.shipped_items[kind] = self.shipped_items.get(kind, 0) + count
            robot.basket = {}
            robot.has_basket = False
            job.took_full = True
            job.next_at = self.t + self.swap_step_s
            if job.fail_after_take:
                del self.jobs[server]
                self.core.on_swap_result(job.cmd.goal_id, False, True, False, "GRIPPER")
            return
        if job.steps:
            job.steps.pop(0)
            robot.has_basket = True
            job.put_empty_done = True
        del self.jobs[server]
        if not job.silent:
            self.core.on_swap_result(job.cmd.goal_id, True, job.took_full, job.put_empty_done)

    def _report_states(self):
        for r in self.virtual.values():
            if r.removed or not r.comm_up:
                continue
            if self.gaussian_noise:
                x = r.x + self.rng.gauss(0.0, r.noise)
                y = r.y + self.rng.gauss(0.0, r.noise)
            else:
                x = r.x + self.rng.uniform(-r.noise, r.noise)
                y = r.y + self.rng.uniform(-r.noise, r.noise)
            if self.t < r.localizing_until:
                self.core.on_robot_state(r.name, 0.0, 0.0, 0.0, "LOCALIZING")
                continue
            x -= math.sin(r.heading) * r.drift
            y += math.cos(r.heading) * r.drift
            self.reported[r.name] = (x, y)
            state = "BLOCKED" if r.blocked else "MOVING" if r.goal_id else "IDLE"
            battery = self.batteries.get(r.name)
            if battery is None:
                self.core.on_robot_state(r.name, x, y, 0.0, state)
            else:
                # 실제 자세와 서 있는지로 충전한다
                level = battery.percent(self.t, r.x, r.y, r.goal_kind != "move")
                self.core.on_robot_state(r.name, x, y, 0.0, state, battery=level)

    # ---- commands -----------------------------------------------------------------------

    def _apply(self, item):
        if isinstance(item, EventOut):
            self.events.append(item)
            self.event_log.append((self.t, item))
            return
        self.commands.append((self.t, item))
        if isinstance(item, ReportOrderCmd):
            self._take_report(item)
        elif isinstance(item, FillCmd):
            self._start_fill(item)
        elif isinstance(item, SwapCmd):
            self._start_swap(item)
        elif isinstance(item, MoveCmd):
            self._start_move(item)
        elif isinstance(item, DockCmd):
            self._start_dock(item)
        elif isinstance(item, CancelCmd):
            self._cancel(item)
        else:
            raise AssertionError(f"unknown output {item!r}")

    def _preempt(self, r):
        """교체되는 실행 중 목표도 ROS 액션처럼 결과를 보고한다."""
        if r.goal_id is None:
            return
        old = r.goal_id
        if r.goal_kind == "move":
            self._queued_results.append((r.name, self.core.on_move_result,
                                         (r.name, old, False, "CANCELED")))
        else:
            self._queued_results.append((r.name, self.core.on_dock_result,
                                         (r.name, old, False)))
        r.goal_id = None

    def _start_move(self, cmd):
        r = self.virtual[cmd.robot]
        if not r.comm_up or r.removed:
            return
        cell = next(c for c in self.site.zone_of if self.site.waypoint_id(c) == cmd.target_id)
        zone = self.site.zone_of[cell]
        assert zone in self.core.zones.held_by(cmd.robot), f"{cmd} targets an ungranted zone"
        assert (cmd.x, cmd.y) == self.site.waypoint_pose(cell), f"{cmd} pose is not the cell centre"
        if self._refuses_path(r, cmd):
            return
        # path 모드: 첫 점은 보고 위치라 건너뛰고 실제 위치에서 다음 점들을 잇는다
        via = [tuple(p[:2]) for p in cmd.path[1:-1]]
        if cmd.path:
            assert tuple(cmd.path[-1][:2]) == (cmd.x, cmd.y), f"{cmd} path ends off the target"
        while via:  # 이미 지난 앞부분은 건너뛴다(FollowPath처럼)
            a, b = via[0], via[1] if len(via) > 1 else (cmd.x, cmd.y)
            on = _point_segment(r.x, r.y, *a, *b) < LINE_TOLERANCE_M
            if not (on and math.dist((r.x, r.y), a) > LINE_TOLERANCE_M):
                break
            via.pop(0)
        corners = [(r.x, r.y), *via, (cmd.x, cmd.y)]
        for (ax, ay), (bx, by) in zip(corners, corners[1:], strict=False):
            for step in range(5):
                x = ax + (bx - ax) * step / 4
                y = ay + (by - ay) * step / 4
                off = min(_point_segment(x, y, lane.ax, lane.ay, lane.bx, lane.by)
                          for lane in self.site.lanes.values())
                assert off < LINE_TOLERANCE_M, f"{cmd} from ({r.x}, {r.y}) leaves the lanes"
        self._preempt(r)
        r.docked_at = ""
        r.goal_id = cmd.goal_id
        r.goal_kind = "move"
        r.target = (cmd.x, cmd.y)
        r.via = via
        r.goal_elapsed = 0.0

    def _refuses_path(self, r, cmd) -> bool:
        """핑키의 연결 검사(지시서 7.5). 받은 path가 로봇이 아는 제 자리(보고 자리에서 잡음을
        뺀 것)에서 lane_error_warn_m보다 멀면 따라가지 않는다. 이전 목표는 끝나고 로봇은 선다.

        이 세계는 보고하고 같은 step에 명령을 받으므로 관제가 만든 path는 관제가 받아들인 마지막
        위치를 지난다(관제가 버린 보고는 그 위치를 바꾸지 않는다). 거절할 만큼 먼 path가 주입한
        것도 아니고 그 위치도 지나지 않으면 관제 결함이라 거절로 가리지 않고 불변식 위반으로
        멈춘다. 그 위치를 지나는데도 멀면 원인을 가리지 않고 핑키처럼 거절한다(예: 한계 없는
        gaussian_noise가 보고를 튀게 한 경우).
        """
        path = [tuple(p[:2]) for p in cmd.path]
        if not path:
            return False
        injected = r.bad_paths > 0
        if injected:  # 오는 길에 path가 틀렸다(옛 path, 중간이 빠진 path를 흉내)
            r.bad_paths -= 1
            path = [(x + r.bad_path_offset, y + r.bad_path_offset) for x, y in path]
        here = (r.x - math.sin(r.heading) * r.drift, r.y + math.cos(r.heading) * r.drift)
        gap = path_gap(*here, path)
        if gap <= self.params.lane_error_warn_m:
            return False
        if not injected:
            m = self.core.traffic._missions[r.name]
            assert path_gap(m.x, m.y, path) <= 1e-6, (
                f"{cmd} does not pass the accepted pose {(m.x, m.y)} "
                f"(the robot is {gap:.3f} m away)")
        self._preempt(r)
        r.target, r.via = None, []
        self.path_refusals.append((self.t, r.name, cmd.goal_id))
        self._queued_results.append((r.name, self.core.on_move_result,
                                     (r.name, cmd.goal_id, False, "PATH_NOT_CONNECTED")))
        return True

    def _start_dock(self, cmd):
        r = self.virtual[cmd.robot]
        if not r.comm_up or r.removed:
            return
        station = self.site.stations[cmd.dock_id]
        assert self.site.nearest_waypoint(r.x, r.y) == station.dock, f"{cmd} away from the dock"
        self._preempt(r)
        r.goal_id = cmd.goal_id
        r.goal_kind = "dock"
        r.dock_left = self.dock_s
        r.dock_station = cmd.dock_id

    def _cancel(self, cmd):
        if cmd.robot not in self.virtual:
            self._cancel_arm(cmd)
            return
        r = self.virtual[cmd.robot]
        if not r.comm_up or r.removed or r.goal_id != cmd.goal_id:
            return
        self._preempt(r)

    def _cancel_arm(self, cmd):
        job = self.jobs.get(cmd.robot)
        if job is None or job.cmd.goal_id != cmd.goal_id or job.silent:
            return
        del self.jobs[cmd.robot]
        if isinstance(job.cmd, FillCmd):
            self._queued_results.append((None, self.core.on_fill_result, (
                cmd.goal_id, False, dict(job.placed), {}, "CANCELED")))
        else:
            self._queued_results.append((None, self.core.on_swap_result, (
                cmd.goal_id, False, job.took_full, job.put_empty_done, "CANCELED")))

    def _start_fill(self, cmd):
        station = self.site.stations[cmd.station_id]
        assert station.action == "fill", f"{cmd} at a {station.action} station"
        assert station.server not in self.jobs, f"{station.server} is busy"
        r = self.virtual[cmd.robot]
        assert r.docked_at == cmd.station_id, f"{cmd} but {cmd.robot} is at {r.docked_at!r}"
        assert r.has_basket, f"{cmd} with no basket on the tray"
        assert cmd.items and all(v > 0 for v in cmd.items.values()), f"{cmd} places nothing"
        wanted = next(o for o in self.registered if o.order_id == cmd.order_id).items
        still = {k: v - r.basket.get(k, 0) for k, v in wanted.items() if v > r.basket.get(k, 0)}
        assert cmd.items == still, f"{cmd} but the basket still needs {still}"
        count = sum(r.basket.values()) + sum(cmd.items.values())
        assert count <= self.params.basket_capacity, f"{cmd} overfills the basket"
        job = ArmJob(cmd, self.t + self.arm_s_per_item,
                     left=[k for k, v in sorted(cmd.items.items()) for _ in range(v)])
        if self.arm_faults["fill_fail"]:
            job.fail_after = self.arm_faults["fill_fail"].pop(0)
        if self.arm_faults["fill_silent"] > 0:
            self.arm_faults["fill_silent"] -= 1
            job.silent = True
        self.jobs[station.server] = job

    def _start_swap(self, cmd):
        station = self.site.stations[cmd.station_id]
        assert station.action == "swap", f"{cmd} at a {station.action} station"
        assert cmd.arm == station.server
        assert cmd.arm not in self.jobs, f"{cmd.arm} is busy"
        r = self.virtual[cmd.robot]
        assert r.docked_at == cmd.station_id, f"{cmd} but {cmd.robot} is at {r.docked_at!r}"
        assert cmd.take_full == r.has_basket, f"{cmd} but has_basket is {r.has_basket}"
        assert cmd.put_empty, f"{cmd} leaves the tray without a basket"
        job = ArmJob(cmd, self.t + self.swap_step_s,
                     steps=["take", "put"] if cmd.take_full else ["put"])
        if self.arm_faults["swap_fail"] > 0:
            self.arm_faults["swap_fail"] -= 1
            job.fail_now = True
        elif self.arm_faults["swap_partial"] > 0 and cmd.take_full:
            self.arm_faults["swap_partial"] -= 1
            job.fail_after_take = True
        if self.arm_faults["swap_silent"] > 0:
            self.arm_faults["swap_silent"] -= 1
            job.silent = True
        self.jobs[cmd.arm] = job

    def _take_report(self, cmd):
        assert cmd.order_id not in {r.order_id for r in self.reports}, f"{cmd} reported twice"
        assert cmd.order_id in {o.order_id for o in self.registered}, f"{cmd} is not on the board"
        self.reports.append(cmd)
        self._unprocessed_reports.append(cmd)

    # ---- invariants ---------------------------------------------------------------------

    def _check_releases(self, before):
        """구역은 보고된 자세가 구역 중심선에서 2r 밖일 때만 점유자를 떠난다.
        그 선 위의 로봇은 떠난 로봇에 닿을 수 없다.
        """
        after = self.core.zones.snapshot()
        least = 2 * self.params.robot_radius_m
        for zone, old in before.items():
            holder = old["holder"]
            if holder and after[zone]["holder"] != holder:
                x, y = self.reported[holder]
                dist = self.site.distance_to_zone(x, y, zone) + self.site.half_width
                assert dist >= least, f"{holder} released {zone} at distance {dist:.3f}"
                true = self.virtual[holder]
                self.min_true_release_distance = min(
                    self.min_true_release_distance,
                    self.site.distance_to_zone(true.x, true.y, zone))

    def _physical_zone(self, r):
        return self.site.zone_of[self.site.nearest_waypoint(r.x, r.y)]

    def _check_invariants(self):
        zones = self.core.zones
        owners = {}
        for name in self.robots:
            held = zones.held_by(name)
            assert len(held) == len(set(held))
            for zone in held:
                assert zone not in owners, f"{zone} has two holders: {owners[zone]} and {name}"
                assert zones.holder(zone) == name
                owners[zone] = name
            assert len(held) <= 2, f"{name} holds {held}"
        inside = {}
        for r in self.virtual.values():
            if r.removed:
                continue
            zone = self._physical_zone(r)
            assert zone not in inside, f"{r.name} and {inside[zone]} are both in {zone} at {self.t}"
            inside[zone] = r.name
            if self.t < r.localizing_until:
                # 아직 지도에 없다. 다른 로봇에게 그 구역을 주지만 않으면 된다.
                assert owners.get(zone) in (None, r.name), f"{zone} given away under {r.name}"
                continue
            assert owners.get(zone) == r.name, (
                f"{r.name} is in {zone} held by {owners.get(zone)} at {self.t}")
            cell = self.site.nearest_waypoint(r.x, r.y)
            if self._last_cell.get(r.name) != cell:
                self._last_cell[r.name] = cell
                self.visits.append((self.t, r.name, cell, self.core.robot_view(r.name)["basket"]))
        present = [r for r in self.virtual.values() if not r.removed]
        for a, b in itertools.combinations(present, 2):
            gap = math.hypot(a.x - b.x, a.y - b.y)
            assert gap >= 2 * self.params.robot_radius_m, (
                f"{a.name} and {b.name} touch at {self.t}: {gap:.3f} m")
        in_baskets = sum(sum(r.basket.values()) for r in self.virtual.values())
        left = sum(self.stock.values())
        assert all(v >= 0 for v in self.stock.values())
        assert left + in_baskets + sum(self.shipped_items.values()) == self.total_items, "item lost"
        assert not (self.jobs.keys() - {s.server for s in self.site.stations.values()})
        for r in present:
            at_outbound = r.goal_kind == "" and r.docked_at == self.core._outbound
            assert r.has_basket or at_outbound, f"{r.name} left the dock without a basket"
