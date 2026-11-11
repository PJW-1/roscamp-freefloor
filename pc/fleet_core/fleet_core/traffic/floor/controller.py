"""자유바닥 교통 제어기. TrafficController의 바깥 API를 그대로 지키고 이동과 잠금만 바꾼다.

물려받는 것: 입력 처리, 배차, 임무 단계(TO_WORK, DOCKING, AT_STATION, ...), 도착 뒤 다음 일,
로봇 선택, 통신 점검, 격리. 바꾸는 것: 구역 대신 칸 잠금(CellLocks), 레인 대신 바닥 경로.

안전 규칙(각각 시험이 있다):
  - 로봇이 쓰는 칸 = 경로를 따라 몸(반지름 + 여유)이 쓸고 지나가는 칸. 지도에 반폭(robot_half_width_m)이
    있으면 곧은 데는 반폭 + 여유만, 제자리에서 돌 수 있는 데(_turn_spans)는 몸 원 전체.
  - 굴러가는 잠금: 앞쪽으로 floor_lock_ahead_m만큼 미리 잠근다. 늘릴 칸은 한꺼번에 잡을 수
    있을 때만 늘린다. 다 못 잡으면 floor_extend_step_m씩 짧게 다시 한꺼번에 시도한다.
  - 보내는 path는 잠근 끝까지만이다. 잠금이 늘면 MoveTo를 교체한다(서지 않는다).
  - 몸 아래 칸, 잠근 앞 칸, 아직 따라가는 MoveTo의 남은 칸은 쥔다. 나머지 칸은 연속 보고
    release_confirm_samples번이 칸에서 release_distance_m보다 멀 때 놓는다.
  - 고정 선(도킹 접근, 도크에서 나감)은 들어가기 전에 선 끝까지 한꺼번에 잡는다. 지도의 rolling_lines
    (한 방향 선)는 바닥처럼 굴려 잠그고 접근선 한 대 규칙도 쓰지 않는다.
  - 교차 상자(_junction_boxes): 한 방향 선이 꺾이는 점을 다른 한 방향 선이 다른 쪽에서 들어와
    지나가면 그 점의 몸 원이 상자다. 한 방향 선을 가는 경로는 몸이 상자에 닿는 구간을 고정 선처럼
    한꺼번에 잡거나, 몸이 상자에 닿지 않는 그 앞에 선다(_box_spans, _candidates). 두 로봇이
    양쪽에서 굴려 와 상자의 칸을 나눠 쥐고 서로 기다리지 않게 한다(arena v4 가운데 통로 합류점).
m.path는 [출발할 때 가까운 지점, 목적지 지점] 두 개다. 선택과 단계 처리가 그대로 쓴다.

여러 대(P3). 안전은 칸 잠금, 효율은 계획:
  - 계획은 서 있는 로봇이 쥔 칸을 피하고, 남의 예약(경로를 계약 속도로 달릴 때 칸에 머무는 시간,
    floor_plan_window_s 앞까지)과 겹치는 칸에서는 그 칸이 빌 때까지 서서 기다리는 값을 문다
    (기다림과 돌아가는 길을 같은 단위로 견준다. 시간축 없는 협조 A*의 가벼운 꼴). 마주 오는 로봇의
    칸은 기다려도 안 비므로 칸마다 벌점이다.
    지나가는 로봇(바닥 경로를 가는 중이고 막히지 않음)의 칸은 곧 비므로 벽으로 보지 않는다.
  - 길이 엇갈리는 두 로봇은 엇갈리는 곳에 먼저 닿는 쪽이 지나간다(YIELD_TIE_S 안이면 우선순위).
    늦는 쪽은 먼저 갈 로봇이 쓸 칸을 잡지 않고 그 앞에 서서 기다렸다가 제 길로 간다(_give_way).
  - 잠금이 늘지 않아 서게 되면 floor_replan_s마다 다시 계획한다(감속하는 동안). 막은 로봇이
    모두 지나가는 중이면 floor_pass_wait_s까지는 서서 기다린다. 두 로봇이 서로 막았으면 비켜 서기와
    같은 순서로 낮은 쪽만 상대의 앞길(YIELD_AHEAD_M)을 비켜 돌아가고, 높은 쪽은 제 길을 지킨다.
  - 대기 그래프에 순환이 deadlock_grace_s 이어지면 우선순위가 낮은 로봇이 옆 빈 칸으로 비켜 선다.
    비켜 설 곳이 없으면 DEADLOCK_UNRESOLVED, 그 뒤 deadlock_unresolved_s 이어지면 격리한다.
  - 도크 접근선은 한 번에 한 대: 다른 로봇이 그 선으로 가는 중이거나 선 칸을 쥐고 있으면
    고정 선 칸에 닿지 않는 빈 자리에서 기다리고, 선이 비면 먼저 온 로봇부터 들어간다.
  - BLOCKED가 blocked_mark_s 이어지면 앞 칸을 막힘(주인 BLOCK_OWNER)으로 표시하고 목표를 거둔다.
    BLOCKED가 아닌 보고가 release_confirm_samples번 오면 표시를 지우고 다시 계획한다.
  - 지름길(지도 shortcuts): 로봇은 원래 접근선으로 출발하고, 잠금이 갈림목 TURN_LEAD_M 앞에
    닿을 때 한 번 고른다(_take_shortcut). 같은 도크로 그 선을 앞서 가는 로봇이 없고, 다른 로봇이
    지름길에서 다른 선과 겹치는 구간에 닿기(도크에서 일하는 로봇은 일이 끝나는 것부터)
    SHORTCUT_MARGIN_S 전에 이 로봇이 그 구간을 지나고, 그 구간 끝까지의 칸을 한꺼번에 잡으면
    지름길로 간다. 그 구간 안에서는 서지 않으므로 나오는 로봇과 틈에서 마주 서지 않는다.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

from fleet_core.common.commands import MoveCmd, RetreatCmd
from fleet_core.traffic.controller import TrafficController
from fleet_core.traffic.deadlock import choose_retreat, find_cycle, wait_graph
from fleet_core.traffic.floor.cells import CellLocks, disk_cells, swept_cells
from fleet_core.traffic.floor.eta import route_s
from fleet_core.traffic.floor.floor_map import floor_map_for
from fleet_core.traffic.floor.planner import FloorPlanner, Route, _nearest_on, rest_of_exit
from fleet_core.traffic.mission import (
    AT_STATION,
    BLOCKED,
    DOCKING,
    ERROR,
    HEARTBEAT_LOST,
    IDLE,
    LOCALIZING,
    MOVE,
    NOT_PLACEABLE,
    ON_A_GOAL,
    OUT_OF_SERVICE,
    PARKED,
    PLACEMENT_CONFLICT,
    RETREAT,
    RUN,
    TO_OUTBOUND,
    TO_PARK,
    TO_WORK,
    _Goal,
    _Mission,
)

AT_END_M = 1e-6
BLOCK_OWNER = "#blocked"  # pseudo holder of cells in front of an object
RESERVE_PENALTY_M = 0.5  # extra cost of a cell another robot plans to be in at that time
SIDESTEP_REACH_M = 1.2  # how far a robot looks for a free spot to step aside to
SIDESTEP_HOLD_S = 3.0  # how long it stays aside before planning its leg again
FIRST_MOVE_REPORTS = 20  # 첫 출발 전에 모을 서 있는 보고 수(10 Hz면 2 s): 평균의 잡음이 1/4.5로 준다
FIRST_MOVE_WAIT_S = 5.0  # 자리가 가라앉지 않아도 이만큼 기다렸으면 떠난다(예전처럼 마지막 자리로)
STAND_REPORTS = 50  # 서 있는 로봇의 자리로 평균 낼 최근 보고 수(10 Hz면 5 s): 조금씩 밀린 로봇을 따라간다
STAND_STILL_M = 0.001  # 서 있는 보고가 모두 이 안에 모이면 잡음이 없다: 모으지 않고 떠난다
STAND_JUMP_M = 0.12  # 보고가 평균에서 이보다 멀면 로봇이 옮겨진 것으로 보고 평균을 다시 시작한다
MOVING_REPORTS = 5  # 달리는 로봇의 자리로 평균 낼 최근 보고 수(10 Hz, 0.15 m/s면 3 cm 뒤처진다)
LAG_M = 0.15  # 평균 낸 자리가 진행점보다 이만큼까지 뒤처질 수 있다고 본다(진행점은 튄 보고로 앞서고 평균은 늦다)
MOVING_JUMP_M = 0.2  # 달리는 로봇의 보고가 평균에서 이보다 멀면 평균을 다시 시작한다
YIELD_TIE_S = 0.5  # 엇갈리는 곳에 닿는 시각 차가 이보다 작으면 우선순위(같으면 이름)로 가른다
YIELD_AHEAD_M = 1.2  # 서로 막았을 때 돌아가는 로봇이 비켜 갈 상대의 앞길(비켜 서기와 같은 거리)
PLANS_PER_TICK = 1  # one plan can take ~10 ms: the other robots plan on the next ticks
WAIT_SPOT_M = (0.3, 1.2)  # a robot waiting for an approach line stands this far from its start
WAIT_TRIES = 8  # 출발할 수 없는 대기 자리 길을 이만큼 보면 그만 찾는다
# a robot at rest lets go down to its body only after this many reports in a row: noise drawn
# anew for each report (sigma 0.03 m) crosses the 5 cm body margin about 1 time in 20
REST_RELEASE_SAMPLES = 10
# 반폭 잠금(지도 robot_half_width_m). RPP가 제자리에서 돌 수 있는 곳만 몸 원 전체를 잠근다
# 보고 한 번에 진행점이 나아가는 한도(10 Hz, 0.3 m/s면 0.03 m): 꺾여 되돌아오는 경로에서 튄 보고가
# 진행점을 모퉁이 너머로 넘기지 못하게 한다
FOLLOW_AHEAD_M = 0.15
TURN_LEAD_M = 0.25  # RPP max_lookahead_dist: 꺾이기 이만큼 앞에서부터 돌기 시작할 수 있다
TURN_START_M = 0.05  # 다리 출발 뒤 이만큼은 출발 자리에서 돌 수 있다
# 지름길: 다른 로봇이 겹치는 구간에 닿기 이만큼 전에 지나갈 수 있어야 탄다(ETA 오차 몫)
SHORTCUT_MARGIN_S = 1.0
# 교차 상자 앞에 서는 로봇의 몸과 상자 사이 여유: 평균 낸 자리의 잡음으로 서 있는 로봇도, 지나가는
# 로봇도 상자 밖으로 한 칸씩 더 잡는다(여유 없이, 또 0.03 m로는 헤드리스 실행에서 상자 가장자리 칸을
# 두고 잠깐씩 서로 기다렸다. 0.06 m에서 세 재현 실행 모두 대기 순환 0)
BOX_SLACK_M = 0.06
# 물러나기(floor_block_retreat, floor_corridor_clear, 제안)
FORK_STEP_M = 0.05  # 갈림길을 찾을 때 지나온 길을 이만큼씩 거슬러 본다(한 번에 계획 하나)
FOOT_LOOK_M = 0.5  # 로봇 자리를 지나온 길 끝 이만큼 안에서 찾는다
TRAIL_STEP_M = 0.02  # 보낸 경로 기록에 진행점을 이만큼마다 적는다
BACK_DONE_M = 0.02  # 통로: 설 곳까지 이만큼 안이면 다 물러났다
RETREAT_TRIES = 2  # 통로: Retreat가 잇달아 이만큼 실패하면 그 자리에서 나온 것으로 친다
SOLO, CORRIDOR = "solo", "corridor"
SEARCH, BACKING, REROUTED, HOLD, TURN, OUT = (
    "SEARCH", "BACKING", "REROUTED", "HOLD", "TURN", "OUT")


@dataclass
class _Leg:
    route: Route | None
    goal_xy: tuple
    out_line: str | None = None
    in_line: str | None = None
    s: float = 0.0  # progress along the route
    s_lock: float = 0.0  # the route is locked up to here (the centre may go this far)
    s_sent: float = 0.0  # end of the last MoveTo
    followed: tuple = ()  # (report, route, progress when it came): one report's step
    done: bool = False  # the MoveTo to the route end succeeded
    slowed: bool = False  # the locked part ahead is short (counted once per episode)
    planned_at: float = 0.0
    no_path_warned: bool = False
    stuck_since: float | None = None  # the lock could not grow since then
    resume: tuple | None = None  # sidestep leg: (phase, dest, goal waypoint) to go on with
    resume_at: float | None = None  # sidestep done: plan the real leg again from then
    waiting: bool = False  # another robot has the approach line: the route ends at a wait spot
    asked_at: float = 0.0  # when this leg asked for its approach line (first come first served)
    admitted: bool = False  # 시간창: 접근선 차례를 받음(선이 비기 전 미리 출발 포함)
    early: bool = True  # 시간창: 선이 비기 전에 출발해도 되나(설 자리가 없던 다리는 끔)
    no_way_since: float | None = None  # 막혀서 다른 길을 찾았는데 못 찾았다(잠금이 늘면 지움)
    behind: tuple = ()  # 엇갈리는 곳을 먼저 지나게 해 줄 로봇들(정한 차례)
    yields: tuple = ()  # 그중 지금 이 로봇의 잠금을 막고 있는 로봇들(서서 기다린다)
    yield_since: float | None = None
    no_yield: tuple = ()  # 너무 오래 기다려 이 다리에서는 더 비켜 주지 않는 로봇들
    short_decided: bool = False  # 지름길을 탈지 골랐다(다리마다 한 번)


@dataclass
class _Trail:
    """보낸 경로 기록: 이 임무 단계에서 로봇이 보낸 경로를 따라 지나온 점들(진행점까지)."""
    phase: str
    points: list = field(default_factory=list)
    route: Route | None = None  # 마지막으로 적은 경로와 거기서 적은 진행
    s: float = 0.0


@dataclass
class _Back:
    """물러나기 일 하나. SOLO: 막힌 로봇 혼자(floor_block_retreat), CORRIDOR: 통로 비우기
    (floor_corridor_clear, 막힌 로봇과 같은 선 뒤쪽 로봇들)."""
    kind: str
    state: str  # SEARCH, BACKING, REROUTED, HOLD, TURN(통로: 물러날 차례), OUT(통로: 다 나옴)
    owner: str  # 막힘 표시를 가진 로봇(혼자면 자신)
    tries: list | None = None  # 갈림길 찾기: 아직 볼 뒤 거리(m). None이면 처음부터
    reach: float = 0.0  # 갈림길 찾기: 거슬러 볼 수 있는 거리(m)
    cells: frozenset = frozenset()  # 물러나는 동안 쥐는 칸(몸과 물러날 길)
    retreated: bool = False  # 한 번이라도 물러났다: 막힘 표시를 첫 풀린 보고에 지우지 않는다
    leg: _Leg | None = None  # REROUTED: 새 길을 받은 다리
    since: float = 0.0  # REROUTED: 새 길을 받은 때
    until: float = 0.0  # HOLD: 기다림이 끝나는 때
    keep: frozenset = frozenset()  # 거슬러 지나갈 한 방향 선 칸(통째로 쥔다)
    keep_until: float = 0.0  # 새 길에서 이 진행을 지나면 keep을 보통 규칙으로 놓는다
    line: str | None = None  # CORRIDOR: 비울 선
    target: float | None = None  # CORRIDOR: 설 곳(보낸 경로 기록 처음부터 m)
    out_at: float | None = None  # CORRIDOR: 다 나온 때
    wait_since: float | None = None  # CORRIDOR: 물러날 칸을 기다리기 시작한 때
    fails: int = 0  # CORRIDOR: 잇달아 실패한 Retreat 수
    goal_id: int | None = None  # 마지막 Retreat: 결과 없이 놓쳐도 늦은 결과가 오면 칸을 놓는다


def _along(points, xy) -> tuple[float, float]:
    """꺾은선 위로 xy를 투영한 거리(처음부터, 0~길이)와 꺾은선 길이."""
    best, base, total = (math.inf, 0.0), 0.0, 0.0
    for a, b in zip(points, points[1:], strict=False):
        seg = math.dist(a, b)
        if seg > 0:
            t = ((xy[0] - a[0]) * (b[0] - a[0]) + (xy[1] - a[1]) * (b[1] - a[1])) / seg ** 2
            t = min(1.0, max(0.0, t))
            q = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
            best = min(best, (math.dist(q, xy), base + t * seg))
        base += seg
    total = base
    return best[1], total


class FloorController(TrafficController):
    def __init__(self, site, params, robots, clock, next_goal_id=None, floor=None):
        super().__init__(site, params, robots, clock, next_goal_id)
        self.floor = floor or floor_map_for(site)
        if not params.floor_shortcut:
            # 지름길을 끄면 지름길이 없는 바닥으로 돈다: 그 선도, 그 선 밑의 좁은 칸도 없다
            self.floor = self.floor.without_shortcuts()
        if self.floor.robot_radius_m < params.robot_radius_m:
            raise ValueError(f"the floor map radius {self.floor.robot_radius_m} m is smaller "
                             f"than the fleet radius {params.robot_radius_m} m")
        self.zones = CellLocks(clock)
        self.planner = FloorPlanner(self.floor)
        self._body = self.floor.robot_radius_m + self.floor.cell_margin_m
        half = self.floor.robot_half_width_m
        # around the robot itself on a straight run: half width + cell_margin_m
        self._near = min(self._body, half + self.floor.cell_margin_m) if half else self._body
        # beside the line it locks ahead: half width + side_margin_m (default cell_margin_m)
        self._side = min(self._near, half + self.floor.side_margin_m) if half else self._body
        # a band of radius _side ends in a half circle; the square body's corners (half, +-half)
        # stay inside it only when the band runs on this far past either end
        self._cap = (max(0.0, half - math.sqrt(max(0.0, self._side ** 2 - half ** 2)))
                     if half else 0.0)
        self._legs: dict[str, _Leg] = {}
        # robot -> goal id -> path of every MoveTo the robot may still follow (until its result)
        self._active: dict[str, dict[int, Route]] = {name: {} for name in robots}
        self._off_cells: dict[str, int] = {name: 0 for name in robots}
        # robot -> cell -> reports in a row beyond the body while at rest (kept apart from
        # m.outside, which counts beyond release_distance_m)
        self._rest_outside: dict[str, dict[str, int]] = {name: {} for name in robots}
        self._wp_cell: dict = {}
        self.metrics = {"slowdowns": 0, "replans": 0, "sidesteps": 0, "no_path": 0,
                        "pre_admits": 0, "shortcuts": 0}
        self._block_since: dict[str, float] = {}  # robot -> first BLOCKED report of this run
        self._clear_n: dict[str, int] = {}  # robot -> non-BLOCKED reports since its mark
        self._marks: dict[str, set[str]] = {}  # robot -> cells it marked as blocked
        self._ahead: dict[str, set[str]] = {}  # robot -> cells to mark once its goal has ended
        self._deadlock_key: tuple = ()
        # cell offsets whose box comes nearer than the body radius to a cell's box
        step = self.floor.cell_m
        n = math.ceil(self._body / step) + 1
        self._reach = [(di, dj) for di in range(-n, n + 1) for dj in range(-n, n + 1)
                       if math.hypot(max(0, abs(di) - 1) * step,
                                     max(0, abs(dj) - 1) * step) < self._body]
        self._line_ids = {name: self._swept(line.points)
                          for name, line in self.floor.fixed_lines.items()}
        self._all_line_ids = set().union(*self._line_ids.values()) if self._line_ids else set()
        self._boxes = self._junction_boxes()
        self._box_cells = frozenset().union(*self._boxes)
        # 지름길: 접근선 -> 지름길, 지름길 -> (갈림목에서 접근선 끝까지 m, 겹치는 구간 끝에서
        # 지름길 끝까지 m, 합류점에서 지름길 끝까지 m)
        self._shortcut = {base: short for short, base in self.floor.shortcuts.items()}
        self._short_route = {short: Route(list(self.floor.fixed_lines[short].points))
                             for short in self.floor.shortcuts}
        self._short_geo = {short: self._shortcut_geometry(short) for short in self.floor.shortcuts}
        self._deadlock_seen = 0.0
        self._unresolved_robot = ""
        self._plan_budget = PLANS_PER_TICK
        self.station_left = None  # 매니저가 꽂는다: 로봇 -> 스테이션 작업 남은 초 또는 None
        self._phase_at: dict[str, tuple[str, float]] = {}  # 로봇 -> (단계, 들어간 시각)
        self._stand_ban: dict[str, set[str]] = {}  # 접근선 -> 미리 출발한 로봇이 못 서는 칸
        # 스테이션 도크에 선 로봇의 몸 둘레 칸 (i, j): 지나가는 길이 벽으로 피한다(_route)
        self._dock_keep = frozenset(c for dock in self.floor.docks.values()
                                    for c in disk_cells(self.floor, *dock, self._body))
        self._stand: dict[str, deque] = {}  # 로봇 -> 서 있는 동안의 최근 보고 자리(STAND_REPORTS개)
        # 로봇 -> 첫 출발을 기다리기 시작한 시각(다 기다렸으면 None)
        self._unsettled: dict[str, float | None] = {}
        self._started: set[str] = set()  # 자리 잡은 뒤 MoveTo를 한 번이라도 받은 로봇
        self._recent: dict[str, deque] = {}  # 로봇 -> 달리는 동안의 최근 보고 자리
        self._span_cache: dict[str, tuple] = {}  # 로봇 -> (경로, 나아간 칸 수, 칸별 시각표)
        self._cross_cache: dict = {}  # (로봇, 로봇) -> 엇갈림(시각표가 바뀌면 다시 구한다)
        # 물러나기(제안). 둘 다 꺼져 있으면 아래 것은 비어 있고 아무 데서도 쓰지 않는다
        self._back_on = bool(params.floor_block_retreat or params.floor_corridor_clear)
        self._back: dict[str, _Back] = {}  # 로봇 -> 물러나기 일
        self._trail: dict[str, _Trail] = {}  # 로봇 -> 보낸 경로 기록
        self._mark_at: dict[str, float] = {}  # 로봇 -> 막힘 표시를 한 때
        if self._back_on:
            self.metrics.update(retreats=0, reroutes=0, corridor_clears=0)

    # ---- cells --------------------------------------------------------------------------

    def _ids(self, cells) -> set[str]:
        return {self.floor.cell_id(c) for c in cells}

    def _body_cells(self, x: float, y: float) -> set[str]:
        return self._ids(disk_cells(self.floor, x, y, self._body))

    def _standing_cells(self, x: float, y: float) -> set[str]:
        """로봇이 서 있는 동안 쥐는 칸: 몸 칸과 아직 release_distance_m 안이라 놓지 않는 칸."""
        return self._ids(disk_cells(self.floor, x, y,
                                    max(self._body, self.params.release_distance_m)))

    def _swept(self, points) -> set[str]:
        return self._ids(swept_cells(self.floor, points, self._body))

    def _turn_spans(self, route) -> list:
        """경로에서 제자리에서 돌 수 있는 구간 [(s0, s1)]: 출발점, 끝점, 그리고 TURN_LEAD_M 안에서
        모두 eta_turn_min_rad(RPP rotate_to_heading_min_angle)보다 꺾이는 점들 앞 TURN_LEAD_M."""
        spans = getattr(route, "turn_spans", None)
        if spans is not None:
            return spans
        pts, cum = route.points, route.cum
        turns = []
        for k in range(1, len(pts) - 1):
            a, b, c = pts[k - 1], pts[k], pts[k + 1]
            if math.dist(a, b) < 1e-9 or math.dist(b, c) < 1e-9:
                continue
            h0 = math.atan2(b[1] - a[1], b[0] - a[0])
            h1 = math.atan2(c[1] - b[1], c[0] - b[0])
            turns.append((cum[k], abs(math.remainder(h1 - h0, math.tau))))
        spans = [(0.0, 0.0), (route.length, route.length)]
        limit = self.params.eta_turn_min_rad
        for i, (s, _) in enumerate(turns):
            window = [(t, a) for t, a in turns[i:] if t <= s + TURN_LEAD_M + 1e-9]
            if sum(a for _, a in window) > limit:
                spans.append((max(0.0, s - TURN_LEAD_M), window[-1][0]))
        route.turn_spans = spans
        return spans

    def _junction_boxes(self) -> list[frozenset[str]]:
        """교차 상자: 한 방향 선 A가 eta_turn_min_rad보다 꺾이는 점의 몸 원(제자리에서 도는 칸) 중,
        다른 한 방향 선 B의 가운데 선이 그 점을 반폭 띠(_side) 안으로 지나가고(합류·교차) 두 선이
        서로 다른 쪽에서 들어오는 것(처음 몸이 원에 닿는 자리가 몸 반지름보다 멀다). 같은 쪽에서
        들어와 갈라지는 곳은 한 줄로 오므로 상자가 아니다. 지름길은 겹치는 구간을 이미 통째로
        잡으므로(_take_shortcut) 상자를 만들지 않는다."""
        lines = sorted(set(self.floor.rolling_lines) - set(self.floor.shortcuts))
        routes = {name: Route(list(self.floor.fixed_lines[name].points)) for name in lines}
        boxes: list[frozenset[str]] = []
        for a in lines:
            pts = routes[a].points
            for k in range(1, len(pts) - 1):
                p, q, r = pts[k - 1], pts[k], pts[k + 1]
                turn = math.remainder(math.atan2(r[1] - q[1], r[0] - q[0])
                                      - math.atan2(q[1] - p[1], q[0] - p[0]), math.tau)
                if abs(turn) <= self.params.eta_turn_min_rad:
                    continue
                box = frozenset(self._body_cells(*q))
                for b in lines:
                    if (b == a or box in boxes or routes[b].project(*q)[1] > self._side
                            or math.dist(self._entry(routes[a], box),
                                         self._entry(routes[b], box)) <= self._body):
                        continue
                    boxes.append(box)
        return boxes

    def _entry(self, route, cells) -> tuple[float, float]:
        """경로에서 몸이 cells에 처음 닿는 자리(닿지 않으면 끝점)."""
        s, step = 0.0, self.floor.cell_m / 2
        while s < route.length and self._body_cells(*route.at(s)).isdisjoint(cells):
            s += step
        return route.at(s)

    def _box_spans(self, route) -> list:
        """한 방향 선을 가는 경로에서 몸(BOX_SLACK_M 더)이 교차 상자 칸에 닿는 구간 [(s0, s1)].
        s0은 그 앞에서 닿지 않는 마지막 자리, s1은 지나서 처음 닿지 않는 자리(경로 끝이면 끝)."""
        spans = getattr(route, "box_spans", None)
        if spans is not None:
            return spans
        spans = []
        rolling = self.floor.rolling_lines
        if self._box_cells and any(line in rolling for _, _, line in route.fixed):
            n = max(1, math.ceil(route.length / (self.floor.cell_m / 2)))
            clear, start = 0.0, None
            reach = self._body + BOX_SLACK_M
            for k in range(n + 1):
                s = route.length * k / n
                near = self._ids(disk_cells(self.floor, *route.at(s), reach))
                if not near.isdisjoint(self._box_cells):
                    start = clear if start is None else start
                    continue
                if start is not None:
                    spans.append((start, s))
                    start = None
                clear = s
            if start is not None:
                spans.append((start, route.length))
        route.box_spans = spans
        return spans

    def _turning_at(self, route, s: float) -> bool:
        return any(a - 1e-9 <= s <= b + 1e-9 for a, b in self._turn_spans(route))

    def _lock_cells(self, route, a: float, b: float) -> set[str]:
        """경로 a..b를 달리며 쓰는 칸: 곧은 데는 반폭(_side), 제자리에서 돌 수 있는 데는 몸 원."""
        if self._side >= self._body:
            return self._swept(route.between(a, b))
        lo, hi = max(0.0, a - self._cap), min(route.length, b + self._cap)
        cells = self._ids(swept_cells(self.floor, route.between(lo, hi), self._side))
        for s0, s1 in self._turn_spans(route):
            # within _cap of a span the band's run-on reaches it (a route end stops the run-on)
            lo, hi = max(a - self._cap, s0), min(b + self._cap, s1)
            if lo <= hi + 1e-9:
                cells |= self._swept(route.between(lo, max(lo, hi)))
        return cells

    def _reach_now(self, m: _Mission) -> float:
        """지금 보고 자리에서 쥘 몸 반지름: 경로를 곧게 달리는 중이면 반폭, 아니면 몸 원."""
        if self._side < self._body:
            leg = self._legs.get(m.name)
            if (leg is not None and leg.route is not None and leg.s > TURN_START_M
                    and not self._turning_at(leg.route, leg.s)):
                return self._near
        return self._body

    def _body_now(self, m: _Mission) -> set[str]:
        return self._ids(disk_cells(self.floor, *self._here(m), self._reach_now(m)))

    # ---- where a standing robot is --------------------------------------------------------

    def _charger_blocks_queue(self) -> bool:
        """자유바닥은 대기 지점에서 곧장 나간다: 충전 중인 로봇이 줄을 막지 않는다."""
        return False

    def _stands(self, m: _Mission) -> bool:
        """서 있다: goal도, 아직 따라갈 수 있는 옛 MoveTo도 없고 움직인다고 보고하지 않는다."""
        return (m.goal is None and not self._active[m.name]
                and m.robot_state not in ON_A_GOAL + (BLOCKED,))

    def _note_standing(self, m: _Mission) -> None:
        """보고 자리를 모은다. 서 있는 동안은 모두(평균에서 STAND_JUMP_M 넘게 튀면 다시),
        달리는 동안은 최근 MOVING_REPORTS개(MOVING_JUMP_M 넘게 튀면 다시)."""
        recent = self._recent.setdefault(m.name, deque(maxlen=MOVING_REPORTS))
        if not self._stands(m):
            self._stand.pop(m.name, None)
            if recent and math.dist((sum(p[0] for p in recent) / len(recent),
                                     sum(p[1] for p in recent) / len(recent)),
                                    (m.x, m.y)) > MOVING_JUMP_M:
                recent.clear()
            recent.append((m.x, m.y))
            return
        recent.clear()
        seen = self._stand.get(m.name)
        if seen is not None and math.dist(self._mean(seen), (m.x, m.y)) > STAND_JUMP_M:
            seen = None  # 옮겨졌다
        if seen is None:
            seen = self._stand[m.name] = deque(maxlen=STAND_REPORTS)
        seen.append((m.x, m.y))

    @staticmethod
    def _mean(points) -> tuple[float, float]:
        return (sum(p[0] for p in points) / len(points), sum(p[1] for p in points) / len(points))

    def _here(self, m: _Mission) -> tuple[float, float]:
        """로봇 자리: 서 있으면 최근 보고 STAND_REPORTS개의 평균, 달리면 최근 MOVING_REPORTS개의
        평균(보고 잡음이 준다. 달릴 때는 조금 뒤처지는데 뒤는 아직 쥔 칸이다). 모은 것이 없으면
        마지막 보고."""
        seen = self._stand.get(m.name) or self._recent.get(m.name)
        return self._mean(seen) if seen else (m.x, m.y)

    def _knows_place(self, m: _Mission) -> bool:
        """서 있는 자리를 평균으로 믿을 만하다: 서 있는 보고가 FIRST_MOVE_REPORTS번 모였거나,
        보고가 전혀 흔들리지 않는다(둘 이상이 STAND_STILL_M 안)."""
        seen = self._stand.get(m.name)
        if not seen:
            return False
        if len(seen) >= FIRST_MOVE_REPORTS:
            return True
        xs, ys = [p[0] for p in seen], [p[1] for p in seen]
        return len(seen) >= 2 and max(max(xs) - min(xs), max(ys) - min(ys)) <= STAND_STILL_M

    def _body_reach(self, m: _Mission) -> float:
        """서 있는 로봇의 몸이 평균 자리에서 닿을 수 있는 반지름: 제자리에서 돌 때 모서리
        (반폭의 sqrt(2)배, 지도의 몸 반지름보다 작지 않게) + 평균의 오차(3 x 보고 표준편차 /
        sqrt(보고 수)). 곧게 달릴 때 쥐는 반지름(_near)보다 작게 보지 않고, 보고가 둘보다 적으면
        몸 원 전체로 본다."""
        seen = self._stand.get(m.name)
        half = self.floor.robot_half_width_m
        if not seen or len(seen) < 2 or not half:
            return self._body
        n = len(seen)
        mx, my = self._mean(seen)
        var = sum((x - mx) ** 2 + (y - my) ** 2 for x, y in seen) / (2 * (n - 1))
        corner = max(self.floor.robot_radius_m, math.hypot(half, half))  # 폭보다 긴 몸
        return max(self._near, corner + 3.0 * math.sqrt(var / n))

    def _settled(self, m: _Mission) -> bool:
        """첫 출발을 해도 된다: 이미 달린 적이 있거나, 서 있는 자리를 믿을 만하거나(_knows_place),
        FIRST_MOVE_WAIT_S를 기다렸다(보고가 계속 튀거나 BLOCKED여서 자리가 가라앉지 않는 로봇)."""
        if m.name in self._started or self._knows_place(m):
            self._unsettled.pop(m.name, None)
            return True
        since = self._unsettled.setdefault(m.name, self._clock())
        if since is None:
            return True  # 이미 다 기다렸다: 보내지 못해 다시 물어도 처음부터 기다리지 않는다
        if self._clock() - since < FIRST_MOVE_WAIT_S:
            return False
        self._event("WARN", "FLOOR_PLACE_UNSETTLED",
                    f"first move after {FIRST_MOVE_WAIT_S:.0f} s without a settled place", m.name)
        self._unsettled[m.name] = None
        return True

    def _hold_where_it_stands(self, m: _Mission) -> None:
        """서 있는 로봇은 평균 자리 둘레의 빈 칸을 채워 둔다. 남이 쥔 칸은 청하지 않는다.
        떠날 때 쓸 칸을 미리 쥐어, 뒤에 붙어 선 로봇의 잠금 끝과 서로 기다리지 않게 한다."""
        if m.name not in self._stand or m.fault == ERROR or m.block_cancel is not None:
            return
        held = self.zones.held_set(m.name)
        free = {c for c in self._body_now(m) - held if self.zones.holder(c) is None}
        if free:
            self.zones.acquire_all(m.name, held | free)

    def _drive_cells(self, m: _Mission, leg, end: float, points) -> set[str]:
        """leg.s..end를 달리고 보낼 path(points)를 따라가는 데 쥘 칸."""
        cells = self._drive_cells_all(m, leg, end, points)
        return cells - self._spare_edge(m, cells, points)

    def _drive_cells_all(self, m: _Mission, leg, end: float | None = None, points=None) -> set[str]:
        """_drive_cells에서 가장자리 칸을 빼기 전: 잠글 길, 로봇 자리 둘레, 보낼 path의 띠."""
        if end is None:
            end = min(leg.route.length, leg.s + self.params.floor_lock_ahead_m)
        if points is None:
            points = self._drive_path(m, leg, end)
        band = self._side if self._reach_now(m) < self._body else self._body
        return (self._lock_cells(leg.route, leg.s, end) | self._body_now(m)
                | self._ids(swept_cells(self.floor, points, band)))

    def _spare_edge(self, m: _Mission, cells, points) -> set[str]:
        """자리를 아는(_settled) 서 있는 로봇이 출발 원에서 요구하지 않을 칸: 원의 맨 가장자리
        (몸이 닿을 수 있는 반지름 _body_reach보다 먼 칸)이고, 갈 길의 띠(반폭 띠) 밖이고, 다른 로봇이
        쥔 칸. 맞닿아 선 두 로봇의 원이 칸 한 줄 겹쳐 서로 기다리는 것을 막는다. 반폭을 적은
        지도에서, 서 있는 자리를 믿을 만할 때만 한다(_knows_place)."""
        if self._near >= self._body or not self._knows_place(m):
            return set()
        here = self._here(m)
        inner = self._body_reach(m)
        if inner >= self._body:
            return set()
        box, at = self.floor.box_distance, self.floor.cell_at
        # 출발 원 둘레만 본다(원은 길의 첫 점 둘레로도 잡혀 평균 자리 기준으로는 조금 넘친다)
        outer = self._body + self.floor.cell_m
        edge = {c for c in cells
                if self.zones.holder(c) not in (None, m.name)
                and inner < box(at(c), *here) <= outer}
        if not edge:
            return edge
        return edge - self._ids(swept_cells(self.floor, points, self._near))

    def _zone(self, waypoint) -> str:
        """지점이 든 칸. 물려받은 대기 지점·commit 판단이 이 칸의 주인을 본다."""
        cell = self._wp_cell.get(waypoint)
        if cell is None:
            cell = self.floor.cell_id(self.floor.cell_of(*self.site.waypoint_pose(waypoint)))
            self._wp_cell[waypoint] = cell
        return cell

    # ---- reports ------------------------------------------------------------------------

    def _place(self, m: _Mission) -> None:
        """보고 자리의 몸 칸을 잡는다. 못 잡으면 맵 밖에 두고 다음 보고 때 다시 한다."""
        if m.robot_state in NOT_PLACEABLE:
            return
        # 새로 올린 로봇: 예전에 서 있던 자리와 달린 적을 잊는다(첫 출발을 다시 기다린다)
        for memory in (self._stand, self._recent, self._unsettled):
            memory.pop(m.name, None)
        self._started.discard(m.name)
        cells = self._body_cells(m.x, m.y)
        if not self.zones.force_acquire(m.name, cells):
            in_the_way = sorted({self.zones.holder(c) for c in cells} - {None, m.name})
            if m.fault_reason != PLACEMENT_CONFLICT:
                m.fault = ERROR
                m.fault_reason = PLACEMENT_CONFLICT
                self._event("ERROR", PLACEMENT_CONFLICT,
                            f"{m.name} cannot take its cells, held by {in_the_way}", m.name)
            return
        m.placed = True
        self._joined(m)
        waypoint = self.site.nearest_waypoint(m.x, m.y)
        m.path = [waypoint]
        m.idx = 0
        near = math.dist((m.x, m.y), self.site.waypoint_pose(waypoint))
        m.phase = PARKED if (waypoint in self.site.parking
                             and near <= self.params.arrival_tolerance_m) else IDLE
        if m.fault_reason == PLACEMENT_CONFLICT:
            m.fault = ""
            m.fault_reason = ""

    def _count_reports_outside(self, m: _Mission) -> None:
        self._note_standing(m)
        d = self.params.release_distance_m
        # a robot standing at the end of its trip needs only its body: the cells it came over
        # within release_distance_m would shut a parking spot 0.35 m away
        rest = self._rest_outside.setdefault(m.name, {})
        at_rest = self._rests(m)
        held = self.zones.held_set(m.name)
        for cell_id in list(rest):
            if not at_rest or cell_id not in held:
                del rest[cell_id]  # a cell let go and taken again counts from zero
        x, y = self._here(m)
        box = self.floor.box_distance
        for cell_id in list(held):
            cell = self.floor.cell_at(cell_id)
            # 보고 자리와 평균 자리 둘 다에서 멀어야 센다: 평균이 틀려도 예전보다 일찍 놓지 않는다
            gap = min(box(cell, m.x, m.y), box(cell, x, y))
            if gap > d:
                m.outside[cell_id] = m.outside.get(cell_id, 0) + 1
            else:
                m.outside.pop(cell_id, None)
            if at_rest and gap > self._body:
                rest[cell_id] = rest.get(cell_id, 0) + 1
            else:
                rest.pop(cell_id, None)

    def _rests(self, m: _Mission) -> bool:
        """몸 둘레 칸만 쥐면 되는 로봇: 다 온 로봇(_at_rest), 그리고 다리는 있지만 앞을 하나도
        잠그지 못한 채 서 있는 로봇(대기 자리, 남의 칸 앞). 지나온 칸을 놓아 남의 길을 연다."""
        if self._at_rest(m):
            return True
        leg = self._legs.get(m.name)
        return (leg is not None and m.name in self._stand and m.phase != DOCKING
                and (leg.route is None or leg.s_lock <= leg.s + 1e-9))

    def _at_rest(self, m: _Mission) -> bool:
        """The trip is over: no leg, no goal and no old MoveTo it may still follow."""
        return (m.name not in self._legs and m.goal is None and not self._active[m.name]
                and m.phase != DOCKING and m.robot_state not in ON_A_GOAL + (BLOCKED,))

    def _watch_lane_error(self, m: _Mission) -> None:
        """보고 자리의 몸 칸을 쥐었나 본다. 아니면 한꺼번에 잡고, 못 잡는 보고가
        release_confirm_samples번 이어지면 세운다(레인 이탈 감시 대신)."""
        if not m.available:
            return
        need = self._body_now(m)
        # 평균은 늦거나 틀릴 수 있다: 보고 자리의 몸(여유 없는 반지름)이 남의 칸 위인지도 본다
        body = self._ids(disk_cells(self.floor, m.x, m.y, self.floor.robot_radius_m))
        on_others = any(self.zones.holder(c) not in (None, m.name) for c in body)
        if not on_others and (need <= self.zones.held_set(m.name)
                              or self.zones.acquire_all(m.name, need)):
            self._off_cells[m.name] = 0
            return
        self._off_cells[m.name] += 1
        if self._off_cells[m.name] >= self.params.release_confirm_samples and m.goal:
            self._cancel_goal(m)
            self._event("WARN", "FLOOR_OFF_CELLS",
                        f"reported at ({m.x:.2f}, {m.y:.2f}) on cells held by others, stopped",
                        m.name)

    def _note_block_report(self, m: _Mission) -> None:
        """BLOCKED가 blocked_mark_s 이어지면 앞 칸을 막힘으로 표시한다. 풀리면 지운다."""
        now = self._clock()
        name = m.name
        if m.robot_state == "BLOCKED":
            self._clear_n[name] = 0
            since = self._block_since.setdefault(name, now)
            self._take_ahead(m)
            if name not in self._marks and now - since >= self.params.blocked_mark_s:
                self._mark_blocked(m)
            elif self._blocked_again(m, since, now):
                self._mark_blocked(m)  # 새 길에서 또 막혔다: 표시를 더하고 처음부터 기다린다
                self._hold_back(self._back[name], now)
            elif now - since >= self.params.blocked_timeout_s and not m.fault:
                self._fail(m, "BLOCKED")
            return
        self._block_since.pop(name, None)
        if name in self._marks and not self._keeps_mark(name):
            self._clear_n[name] = self._clear_n.get(name, 0) + 1
            if self._clear_n[name] >= self.params.release_confirm_samples:
                self._ahead.pop(name, None)
                for cell in self._marks.pop(name):
                    self.zones.release(BLOCK_OWNER, cell)
                self._event("INFO", "FLOOR_CLEARED", "the object is gone", name)
                leg = self._legs.get(name)
                if leg is not None and leg.in_line in self._short_route:
                    # 지름길의 겹치는 구간에서 막혔다: 남은 구간의 빈 칸을 나오는 로봇보다 먼저 쥔다
                    self.zones.acquire_all(name, {c for c in self._stretch_ahead(m, leg.in_line)
                                                  if self.zones.holder(c) is None})
                if leg is not None and leg.route is not None:
                    self._plan(m, leg)

    def _mark_blocked(self, m: _Mission) -> None:
        leg = self._legs.get(m.name)
        ahead = set()
        if leg and leg.route:
            ahead = self._swept(leg.route.between(leg.s, leg.s + 0.3))
        ahead -= self._body_cells(m.x, m.y)
        if m.goal:
            self._cancel_goal(m)
        # cells the robot holds stay its own until the cancelled goal has ended (it may still
        # follow it until the result): _take_ahead moves them to the mark then
        free = {c for c in ahead if self.zones.holder(c) is None}
        self.zones.acquire_all(BLOCK_OWNER, free)
        old = self._marks.get(m.name)  # 물러나기: 지키던 표시에 더한다
        self._marks[m.name] = free if old is None else old | free
        self._ahead[m.name] = ahead - free
        if self._back_on:
            self._mark_at[m.name] = self._clock()
        if leg and leg.route:
            leg.s_lock = leg.s_sent = leg.s
        self._event("WARN", "FLOOR_BLOCKED",
                    f"object ahead, {len(free)} cells marked blocked", m.name)

    def _take_ahead(self, m: _Mission) -> None:
        """표시할 앞 칸 중 로봇이 쥐고 있던 칸: goal이 모두 끝난 뒤에야 막힘 표시로 옮긴다."""
        ahead = self._ahead.get(m.name)
        if not ahead or m.goal is not None or self._active[m.name]:
            return
        body = self._body_cells(m.x, m.y)
        for cell in sorted(ahead):
            holder = self.zones.holder(cell)
            if cell in body or holder not in (None, m.name):
                continue
            if holder == m.name:
                self.zones.release(m.name, cell)
            if self.zones.acquire_all(BLOCK_OWNER, {cell}):
                self._marks[m.name].add(cell)
        self._ahead.pop(m.name)

    def blocked_cells(self) -> list[str]:
        return self.zones.held_by(BLOCK_OWNER)

    def _held_for_block(self, m: _Mission) -> bool:
        """막힌 동안과 표시를 지우기 전에는 새 goal을 내지 않는다. 물러나기 일이 있으면 그 일이
        정한다: 막힌 칸을 벽으로 본 새 길을 받은 로봇(REROUTED)만 달린다."""
        back = self._back.get(m.name)
        if back is not None:
            return back.state != REROUTED
        return m.name in self._block_since or m.name in self._marks

    def _queue_rests(self, m: _Mission, lane=None) -> bool:
        m.queue_since = None
        return False

    # ---- tick ---------------------------------------------------------------------------

    def tick_progress(self, now: float) -> frozenset[str]:
        self._plan_budget = PLANS_PER_TICK
        for m in self._missions.values():
            if self._phase_at.get(m.name, ("",))[0] != m.phase:
                self._phase_at[m.name] = (m.phase, now)
            if m.available:
                self._follow(m)
                self._release_cells(m)
                self._watch_stall(m, now)
            else:
                m.stall_from = m.stall_mean = None
                m.stall_hold = False
        if self._back_on:
            self._back_tick(now)
        return self._check_deadlock(now)

    def _follow(self, m: _Mission) -> None:
        """경로 위 진행 s를 보고 자세로 넘긴다. 뒤로는 가지 않고 보낸 끝을 넘지 않는다."""
        if m.goal is None and m.robot_state not in ON_A_GOAL + (BLOCKED,):
            # it stands: nothing old is followed any more (BLOCKED only pauses the goal)
            self._active[m.name].clear()
        leg = self._legs.get(m.name)
        if leg is None or leg.route is None:
            return
        # no further ahead than FOLLOW_AHEAD_M a report: where a route folds back, a thrown
        # report lies nearer the stretch after the corner than the one the robot is on. The
        # ticks between two reports see the same one, so the step starts where the progress
        # was when the report came
        seen = leg.followed
        if not (seen and seen[0] == (m.last_seen, m.x, m.y) and seen[1] is leg.route):
            seen = leg.followed = ((m.last_seen, m.x, m.y), leg.route, leg.s)
        ahead = min(max(leg.s, leg.s_sent), max(leg.s, seen[2] + FOLLOW_AHEAD_M))
        s, _ = leg.route.project(m.x, m.y, leg.s, ahead)
        leg.s = max(leg.s, s)
        if self._back_on:
            self._note_trail(m, leg)
        short = (leg.s_lock < leg.route.length - AT_END_M
                 and leg.s_lock - leg.s < self.params.floor_lock_ahead_m / 2)
        if short and m.goal and not leg.slowed:
            self.metrics["slowdowns"] += 1
        leg.slowed = short and (leg.slowed or m.goal is not None)

    def _release_cells(self, m: _Mission) -> None:
        if m.fault == ERROR or m.block_cancel is not None:
            return  # an isolated robot keeps its cells until confirm_clear
        self._hold_where_it_stands(m)
        keep = self._body_now(m) | self._ids(disk_cells(self.floor, m.x, m.y, self._reach_now(m)))
        leg = self._legs.get(m.name)
        if leg and leg.route and leg.s_lock > leg.s:
            keep |= self._lock_cells(leg.route, leg.s, leg.s_lock)
        if leg is not None and leg.in_line in self._short_route:
            keep |= self._stretch_ahead(m, leg.in_line)
        if leg and leg.route:
            # 교차 상자 구간은 통째로 잡았으니 진행이 구간을 지날 때까지 통째로 쥔다: 모퉁이에서
            # 튄 보고로 진행이 모퉁이를 넘어도 제자리에서 도는 로봇 밑의 칸을 놓지 않는다
            for s0, s1 in self._box_spans(leg.route):
                if s0 - 1e-9 <= leg.s < s1 - 1e-9 <= leg.s_lock:
                    keep |= self._lock_cells(leg.route, s0, s1)
        if self._back:
            keep |= self._back_keep(m, leg)
        for active in self._active[m.name].values():
            s, _ = active.project(m.x, m.y)
            keep |= self._lock_cells(active, s, active.length)
        rest = self._rest_outside.get(m.name, {}) if self._rests(m) else {}
        rest_need = max(REST_RELEASE_SAMPLES, self.params.release_confirm_samples)
        for cell_id in list(self.zones.held_set(m.name)):
            if cell_id in keep:
                continue
            if (m.outside.get(cell_id, 0) >= self.params.release_confirm_samples
                    or rest.get(cell_id, 0) >= rest_need):
                self.zones.release(m.name, cell_id)
                m.outside.pop(cell_id, None)
                rest.pop(cell_id, None)

    # ---- legs ---------------------------------------------------------------------------

    def _line(self, station: str, way: str) -> str | None:
        name = f"{station.lower()}_{way}"
        return name if name in self.floor.fixed_lines else None

    def _dock_here(self, m: _Mission) -> str | None:
        for station, pose in self.floor.docks.items():
            if math.dist((m.x, m.y), pose) <= self.params.arrival_tolerance_m:
                return station
        return None

    def _start_leg(self, m: _Mission, phase: str, dest: str | None, goal_waypoint,
                   way=None) -> None:
        """지금 자리에서 goal_waypoint까지 새 바닥 경로. 따라가던 MoveTo는 교체될 때까지 쥔다."""
        self.zones.cancel_all_requests(m.name)
        here = self._dock_here(m)
        station = next((s for s, st in self.site.stations.items() if st.dock == goal_waypoint),
                       None)
        out_line = self._line(here, "out") if here and here != station else None
        old = self._legs.get(m.name)
        span = old.route.fixed_at(old.s) if old is not None and old.route is not None else None
        if out_line is None and span is not None and span[2] == old.out_line:
            out_line = old.out_line  # 출구선을 나가던 중: 남은 선을 마저 간다
        elif (out_line is None and old is not None and old.route is None and old.out_line
              and rest_of_exit(self._here(m),
                               self.floor.fixed_lines[old.out_line].points) is not None):
            # 앞 다리가 길 없이 끝났다(이번 틱에 계획할 몫이 없었다, 길을 못 찾았다): 아직 그 선
            # 위면 나가던 선을 잊지 않는다. 선을 벗어난 뒤의 다리는 잊는다(나중에 다시 타지 않게)
            out_line = old.out_line
        in_line = self._line(station, "in") if station else None
        if (span is not None and span[2] == old.in_line
                and self._shortcut.get(in_line) == old.in_line):
            in_line = old.in_line  # 지름길 위에서 같은 도크로: 지름길을 마저 간다
        leg = _Leg(None, self.site.waypoint_pose(goal_waypoint), out_line=out_line,
                   in_line=in_line)
        m.path = [self.site.nearest_waypoint(m.x, m.y), goal_waypoint]
        m.idx = 0
        m.settle_idx = None
        m.phase = phase
        m.dest = dest
        leg.asked_at = self._clock()
        self._legs[m.name] = leg
        self._plan(m, leg)

    def _plan(self, m: _Mission, leg: _Leg) -> None:
        leg.done = False
        leg.behind, leg.yields, leg.yield_since = (), (), None  # 새 길: 차례를 처음부터 정한다
        if self._plan_budget <= 0:
            leg.route = None  # planned on the next tick
            leg.planned_at = self._clock() - self.params.floor_replan_s
            leg.waiting = bool(leg.in_line)  # no claim on an approach line before a plan
            if m.goal:
                self._cancel_goal(m)  # stop meanwhile, not on to the old target
            return
        self._plan_budget -= 1
        leg.planned_at = self._clock()
        start = self._here(m)
        out_line = leg.out_line
        line = self.floor.fixed_lines[out_line] if out_line else None
        if line and math.dist(start, line.start) > self.params.arrival_tolerance_m \
                and rest_of_exit(start, line.points) is None:
            out_line = None  # 도크에도 선 위에도 없다(밀려났거나 선을 다 나갔다)
        leg.waiting = bool(leg.in_line) and not self._admitted(m, leg)
        if leg.waiting:
            self._plan_wait(m, leg, out_line)
            return
        if math.dist(start, leg.goal_xy) <= self.params.arrival_tolerance_m and not out_line:
            moving = bool(m.goal and m.goal.kind == MOVE)
            if moving:
                self._cancel_goal(m)  # 옛 MoveTo가 로봇을 목표 밖으로 끌고 가지 않게
            leg.route = Route([leg.goal_xy])
            leg.s = leg.s_lock = leg.s_sent = 0.0
            # 달리던 로봇은 목표 자리로 짧게 보내고 그 결과로 도착한다(취소 결과 전에 도착 아님).
            # 앞서 취소한 옛 MoveTo가 아직 끝나지 않았어도 그렇다
            leg.done = m.goal is None and not moving and not self._active[m.name]
            return
        route = self._route(m, start, leg.goal_xy, out_line, leg.in_line)
        if route is None:
            self.metrics["no_path"] += 1
            leg.route = None  # never drive on to the end of an old route (a wait spot)
            if not leg.no_path_warned:
                leg.no_path_warned = True
                self._event("WARN", "FLOOR_NO_PATH",
                            f"no floor path from ({m.x:.2f}, {m.y:.2f}) to {m.path[-1]}", m.name)
            return
        leg.route = route
        leg.s = leg.s_lock = leg.s_sent = 0.0
        leg.stuck_since = leg.no_way_since = None
        if (leg.admitted and self._line_held(leg.in_line, m.name)
                and self._stop_before_line(leg) is None):
            # 선 칸·주차 지점 밖에 설 곳이 없는 길: 미리 출발하지 않고 대기 자리에서 기다린다
            leg.admitted, leg.early = False, False
            leg.waiting = True
            self._plan_wait(m, leg, out_line)

    # ---- one robot at a time on an approach line -------------------------------------------

    def _admitted(self, m: _Mission, leg: _Leg) -> bool:
        """leg가 접근선에 들어가도 되나: 다른 로봇이 그 선으로 가는 중이 아니고 선 칸을 쥐지 않았고
        먼저 기다리기 시작한 로봇이 없을 때. 한 방향 선(rolling_lines)은 늘 된다: 칸 잠금이 줄을 세운다."""
        if leg.in_line in self.floor.rolling_lines:
            return True
        if self.params.floor_dock_schedule:
            leg.admitted = self._admitted_in_time(m, leg)
            return leg.admitted
        line = leg.in_line
        cells = self._line_ids[line]
        for name in self._missions:
            if name == m.name:
                continue
            if not cells.isdisjoint(self.zones.held_set(name)):
                return False
            other = self._legs.get(name)
            if (other is not None and other.in_line == line and not other.waiting
                    and self._in_service(name)):
                return False
        mine = (leg.asked_at, m.name)
        return not any((other.asked_at, name) < mine for name, other in self._legs.items()
                       if name != m.name and other.in_line == line and other.waiting
                       and self._in_service(name))

    def _in_service(self, name: str) -> bool:
        """격리·제거되지 않은 로봇. 격리·제거된 로봇의 다리는 줄을 잡지 않는다."""
        other = self._missions[name]
        return other.live and other.fault != ERROR  # out of contact keeps its turn

    # ---- ETA (관측) ----------------------------------------------------------------------

    def _line_station(self, line: str) -> str | None:
        line = self.floor.shortcuts.get(line, line)  # 지름길은 그 선의 도크로 간다
        for sid in self.site.stations:
            if line in (f"{sid.lower()}_in", f"{sid.lower()}_out"):
                return sid
        return None

    def _work_s(self, station: str) -> float:
        p = self.params
        if self.site.stations[station].action == "fill":
            return p.eta_fill_base_s + p.eta_fill_items_default * p.eta_fill_s_per_item
        return p.eta_swap_s

    def _since(self, m: _Mission) -> float:
        phase, at = self._phase_at.get(m.name, (m.phase, self._clock()))
        return self._clock() - at if phase == m.phase else 0.0

    def _left_on_line(self, m: _Mission, leg: _Leg | None, line: str, station: str) -> float:
        """이 로봇이 접근선을 놓기까지(초)."""
        p = self.params
        if leg is not None and leg.in_line == line and not leg.waiting and leg.route is not None:
            ahead = [(m.x, m.y)] + leg.route.between(leg.s, leg.route.length)
            return (route_s(ahead, p, heading=m.yaw) + p.eta_dock_s + self._work_s(station)
                    + p.floor_leave_s)
        if m.phase == DOCKING:
            return (max(0.0, p.eta_dock_s - self._since(m)) + self._work_s(station)
                    + p.floor_leave_s)
        if m.phase == AT_STATION and leg is None:
            left = self.station_left(m.name) if self.station_left else None
            if left is None:
                if self._since(m) >= 1.0:
                    return math.inf  # 팔 일 없이 서 있음: 다음 주문을 기다림
                left = self._work_s(station)  # 방금 닿음: 팔 일이 곧 온다
            return left + p.floor_leave_s
        if leg is not None and leg.out_line:  # 도크를 떠나는 중: 출구선을 따라 나간 만큼 줄인다
            pts = self.floor.fixed_lines[leg.out_line].points  # 다리를 만든 시각은 출발이 아니다
            gone, length = _along(pts, (m.x, m.y))
            return p.floor_leave_s * max(0.0, 1.0 - gone / length) if length > 0 else \
                p.floor_leave_s
        if leg is not None:  # 선 칸에서 다른 곳으로 출발
            return p.floor_leave_s
        return math.inf  # 서 있는 로봇: 언제 비킬지 모른다

    def line_free_in(self, line: str, exclude: str | None = None) -> float | None:
        """접근선이 빌 때까지(초). 아무도 쥐지 않고 가는 로봇도 없으면 None."""
        ids = self._line_ids[line]
        for cell in ids:
            holder = self.zones.holder(cell)
            if holder is not None and holder != exclude and (
                    holder not in self._missions or not self._in_service(holder)):
                return math.inf  # 막힘 칸이나 격리된 로봇: 운영자가 풀 때까지
        station = self._line_station(line)
        worst = None
        for name, m in self._missions.items():
            if name == exclude or not self._in_service(name):
                continue
            leg = self._legs.get(name)
            going = (leg is not None and leg.in_line == line and not leg.waiting
                     and leg.route is not None)
            if not going and not ids & self.zones.held_set(name):
                continue
            t = self._left_on_line(m, leg, line, station)
            worst = t if worst is None else max(worst, t)
        return worst

    def line_eta(self, robot: str) -> float | None:
        """접근선 시작에 닿을 때까지(초). 접근선으로 가는 다리가 아니면 None."""
        m, leg = self._missions[robot], self._legs.get(robot)
        if leg is None or not leg.in_line:
            return None
        start = self.floor.fixed_lines[leg.in_line].points[0]
        here = [(m.x, m.y)]
        if leg.route is None:
            return route_s(here + [start], self.params, heading=m.yaw)
        if leg.waiting:
            ahead = leg.route.between(leg.s, leg.route.length) + [start]
        else:
            s0 = next((a for a, _, name in leg.route.fixed if name == leg.in_line),
                      leg.route.length)
            ahead = leg.route.between(leg.s, s0) if s0 > leg.s else []
        return route_s(here + ahead, self.params, heading=m.yaw)

    # ---- 접근선 시간창 ----------------------------------------------------------------------

    def _line_queue(self, line: str, me: str) -> list[str]:
        """접근선 차례. 오래 기다린 로봇은 먼저 온 순서, 나머지는 가장 빨리 닿는 순서."""
        now = self._clock()
        names = [me] + [n for n, other in self._legs.items()
                        if n != me and other.in_line == line and other.waiting
                        and self._in_service(n)]

        def key(name):
            leg = self._legs[name]
            if now - leg.asked_at >= self.params.floor_dock_aging_s:
                return (0, leg.asked_at, 0.0)
            eta = self.line_eta(name)
            return (1, math.inf if eta is None else eta, leg.asked_at)

        return sorted(names, key=key)

    def _admitted_in_time(self, m: _Mission, leg: _Leg) -> bool:
        """시간창 차례: 다른 로봇이 그 선으로 가는 중이 아니고, 줄 맨 앞이고, 선이 비었거나
        닿을 때쯤 빈다(비기 전에 출발해도 선 칸은 빌 때까지 못 잡는다. 안전은 칸 잠금)."""
        line = leg.in_line
        for name, other in self._legs.items():
            if (name != m.name and other.in_line == line
                    and (not other.waiting or other.admitted) and self._in_service(name)):
                return False  # 차례를 받은 다리는 계획을 기다리는 동안에도 차례를 쥔다
        free_in = self.line_free_in(line, exclude=m.name)
        if free_in == math.inf:
            return False  # 선 위에 언제 비킬지 모르는 로봇
        if leg.admitted:
            return True  # 이미 차례를 받음: 예측이 늦어져도 거두지 않는다(오가지 않게)
        if self._line_queue(line, m.name)[0] != m.name:
            return False
        if free_in is None:
            return True
        if not leg.early:
            return False  # 선 앞에 설 자리가 없는 다리: 선이 빈 뒤에만
        eta = self.line_eta(m.name)
        if eta is None or free_in > eta + self.params.floor_pre_admit_margin_s:
            return False
        self.metrics["pre_admits"] += 1
        return True

    def _line_cap(self, m: _Mission, leg: _Leg) -> float | None:
        """미리 출발한 로봇은 선이 남에게 잡혀 있으면 설 자리(_stop_before_line)까지만 잠근다.

        가는 동안은 선 칸에 줄을 서지 않는다: 떠나는 로봇이 이 로봇이 지나가며 쥔 출구선 칸을
        기다리면 대기 순환이 된다. 설 자리에 서면 줄을 선다(_extend): 떠나는 로봇이 이 로봇의
        칸을 기다리면 순환이 보여 교착 처리가 맡는다. 출구선 칸이나 주차 지점에 서면 떠나는
        로봇이나 그 지점으로 가는 로봇을 막는다(주차는 매 tick 다시 출발해 격리까지 갔다).
        설 자리가 없으면 None: 보통 다리처럼 줄을 선다. 계획 때 설 자리가 없으면 _plan이
        미리 출발을 거둔다. 선이 비면 다음 tick에 선 끝까지 한 번에 잡는다.
        """
        if leg.in_line in self._short_route and leg.route is not None:
            return self._short_cap(m, leg)
        if (not leg.admitted or leg.in_line is None or leg.route is None
                or leg.in_line in self.floor.rolling_lines):
            return None
        if not self._line_held(leg.in_line, m.name):
            return None
        return self._stop_before_line(leg)

    def _short_cap(self, m: _Mission, leg: _Leg) -> float | None:
        """지름길 다리의 설 자리: 겹치는 구간 끝. 거기서 도크까지는 한꺼번에만 잡는다(_extend).
        그 끝에서 도크까지는 도크에 선 로봇을 두 몸과 보고 잡음보다 가깝게 지나가서, 잠금을 굴려
        다가가면 도크에 선 로봇의 자리와 서로 칸을 기다린다(비켜 설 곳 없는 교착). 구간 끝에서는
        다른 선을 막지 않는다. 이미 지났으면(끝까지 잡았다) None."""
        short = leg.in_line
        end = next((b for _, b, line in leg.route.fixed if line == short), None)
        if end is None:
            return None
        cap = end - self._short_geo[short][1]
        if leg.s_lock > cap + 1e-9 or cap <= 0.0:
            return None
        return cap

    def _line_held(self, line: str, me: str) -> bool:
        """선 칸 중 하나라도 남(로봇 또는 막힘 표시)이 쥐었나."""
        return any(self.zones.holder(c) not in (None, me) for c in self._line_ids[line])

    def _stop_before_line(self, leg: _Leg) -> float | None:
        """경로에서 접근선 고정 구간 시작까지, 다른 고정 선 칸과 주차 지점에 닿지 않고 서는 마지막
        자리. 선 시작에서 대기 자리 거리(WAIT_SPOT_M) 안에 없으면 None."""
        s0 = low = None
        for a, b, line in leg.route.fixed:
            if line == leg.in_line:
                s0 = a
                break
            low = b  # 앞선 고정 선(출구선) 끝보다 뒤로는 안 간다
        if s0 is None:
            return None
        ban = self._stand_ban.get(leg.in_line)
        if ban is None:  # 다른 고정 선(제 출구선 포함) 칸과 주차 지점의 몸 칸. 제 선에만 속한 칸은 된다
            others = set().union(*(ids for line, ids in self._line_ids.items()
                                   if line != leg.in_line))
            ban = others.union(*(self._body_cells(*self.site.waypoint_pose(p))
                                 for p in self.site.parking))
            self._stand_ban[leg.in_line] = ban
        start = self.floor.fixed_lines[leg.in_line].points[0]
        s, low = s0, low or 0.0
        while s > low + 1e-9:
            x, y = leg.route.at(s)
            if math.dist((x, y), start) > WAIT_SPOT_M[1]:
                return None  # 대기 자리보다 멀다: 선이 빈 뒤 먼 길을 가느니 대기 자리가 낫다
            if self._standing_cells(x, y).isdisjoint(ban):
                return s
            s -= self.params.floor_extend_step_m
        return None

    def _plan_wait(self, m: _Mission, leg: _Leg, out_line) -> None:
        """접근선 시작 근처, 고정 선 칸·남의 칸·다른 대기 자리에 닿지 않는 가장 가까운 빈 자리로."""
        start = self._here(m)
        line_start = self.floor.fixed_lines[leg.in_line].points[0]
        banned = set(self._all_line_ids) | set(self.zones.held_by_others(m.name))
        for name, other in self._legs.items():
            if name != m.name and other.waiting and other.route is not None:
                banned |= self._standing_cells(*other.route.points[-1])
        near, far = WAIT_SPOT_M
        if not out_line and self._standing_cells(*start).isdisjoint(banned) and \
                near <= math.dist(start, line_start) <= far:
            leg.route = Route([start])  # already standing in a good place
            if m.goal:
                self._cancel_goal(m)  # stop here, not at the old leg's target
            leg.done = True
            leg.s = leg.s_lock = leg.s_sent = 0.0
            return
        spots = sorted((c for c in self.floor.clear
                        if near <= math.dist(line_start, self.floor.centre(c)) <= far),
                       key=lambda c: math.dist(line_start, self.floor.centre(c)))
        first, stuck_ways = None, 0
        for cell in spots:
            spot = self.floor.centre(cell)
            if not self._standing_cells(*spot).isdisjoint(banned):
                continue
            route = self._route(m, start, spot, out_line, cautious=True)
            if route is None:
                continue
            if not self._can_start(m.name, route):
                first = first or route  # 첫 구간이 남의 칸에 닿는다: 출발할 수 있는 다음 자리를 본다
                stuck_ways += 1
                if stuck_ways >= WAIT_TRIES:
                    break  # 계획 비용: 더 보지 않는다
                continue
            first = route
            break
        if first is None:
            leg.route = None  # nowhere to wait now: try again after floor_replan_s
            return
        # 출발할 수 있는 자리가 없으면 처음 찾은 길(예전대로): 칸을 기다려 교착 처리에 보인다
        leg.route = first
        leg.s = leg.s_lock = leg.s_sent = 0.0
        leg.stuck_since = None

    # ---- 지름길 ----------------------------------------------------------------------------

    def _shortcut_geometry(self, short: str) -> tuple[float, float, float]:
        """지름길 short의 (갈림목에서 원래 선 끝까지, 겹치는 구간 끝에서 지름길 끝까지, 합류점에서
        지름길 끝까지) m. 겹치는 구간: 갈림목부터 몸이 다른 고정 선(원래 선 말고)을 따라가는 몸에
        닿는 동안. 그 끝에 선 로봇은 그 선들로 가는 로봇을 막지 않는다. 도크 가까이에서 다시 닿는
        것(도크에서 나가는 선의 시작)은 원래 선도 같다. 그 밖에서 다시 닿으면 ValueError."""
        base = self.floor.shortcuts[short]
        fork, merge = self.floor.shortcut_parts[short]
        route = self._short_route[short]
        others = set().union(*(ids for name, ids in self._line_ids.items()
                               if name not in (short, base)))
        step = self.floor.cell_m / 2
        clear = s = fork
        while s < merge + 1e-9 and not self._body_cells(*route.at(s)).isdisjoint(others):
            s += step
            clear = s
        clear = min(merge, clear)
        # 그 뒤 합류점까지는 합류점 뒤 꼬리(원래 선도 가는 도크 앞)가 닿는 칸에만 닿아도 된다.
        # 다른 데서 다시 닿으면 잡지 않은 채 남의 선에 선다: 지도 잘못
        tail_cells = self._swept(route.between(merge, route.length))
        while s < merge + 1e-9:
            if (self._body_cells(*route.at(s)) & others) - tail_cells:
                x, y = route.at(s)
                raise ValueError(f"shortcut {short} meets another line again at ({x:.3f}, "
                                 f"{y:.3f}), away from its dock")
            s += step
        gone, length = _along(self.floor.fixed_lines[base].points, route.at(fork))
        return length - gone, route.length - clear, route.length - merge

    def _stretch_ahead(self, m: _Mission, short: str) -> set[str]:
        """지름길 short 위 로봇 m이 아직 지나지 않은 겹치는 구간의 칸. 다시 계획해도 지나갈 때까지
        놓지 않는다: 나오는 로봇이 틈의 먼 끝을 잡아 틈에서 마주 서지 않게."""
        route = self._short_route[short]
        end = route.length - self._short_geo[short][1]
        s, _ = route.project(m.x, m.y)
        return self._lock_cells(route, s, end) if s < end else set()

    def _shortcut_from(self, here, short: str) -> Route | None:
        """here에서 지름길 short를 따라 도크까지. 선에서 JOIN_M보다 멀면 None.
        새 다리처럼 계획하면(planner.leg) 보고가 선 옆으로 몇 cm 벗어났을 때 선 시작으로 되돌아가
        다시 온다(로봇은 원래 선의 틈에 서 있고 지름길은 다른 이름이라 선 위로 치지 않는다)."""
        rest = rest_of_exit(here, self.floor.fixed_lines[short].points)
        if rest is None:
            return None
        if short in self.floor.narrow_lines:
            rest = self.planner._ahead(here, short)  # 좁은 선: 발에서 조금 앞에 비스듬히 붙는다
        route = Route([tuple(here), *(p for p in rest if math.dist(p, here) > 1e-9)])
        route.fixed = [(0.0, route.length, short)]
        return route

    def _take_shortcut(self, m: _Mission, leg: _Leg) -> None:
        """원래 접근선을 가는 로봇이 갈림목 앞에 오면 지름길을 탈지 한 번 고른다(모듈 설명)."""
        short = self._shortcut.get(leg.in_line)
        if short is None or leg.short_decided or leg.route is None or leg.waiting:
            return
        to_end, clear_left, tail = self._short_geo[short]
        route = leg.route
        s_fork = route.length - to_end
        span = next((a for a, _, line in route.fixed if line == leg.in_line), None)
        if span is None or s_fork < span - 1e-9 or leg.s >= s_fork - 1e-6:
            leg.short_decided = True  # 갈림목을 지났거나 선을 갈림목 뒤에서 탔다
            return
        if leg.s + self.params.floor_lock_ahead_m < s_fork - TURN_LEAD_M:
            return  # 아직 멀다: 잠금이 갈림목에서 돌기 시작하는 곳에 닿을 때 고른다
        leg.short_decided = True
        mine = route.length - leg.s
        for name, other in self._legs.items():
            if (name != m.name and other.in_line == leg.in_line and not other.waiting
                    and other.route is not None and self._in_service(name)
                    and tail < other.route.length - other.s <= mine):
                return  # 합류점 전에 같은 선을 앞서 가는 로봇: 두 길로 합류점을 다투지 않는다
        here = self._here(m)
        new = self._shortcut_from(here, short)
        if new is None:
            return
        end = new.length - clear_left
        if end <= 0.0:
            return
        trial = _Leg(new, leg.goal_xy, in_line=short)
        cells = self._drive_cells(m, trial, end, self._drive_path(m, trial, end))
        through = route_s([here] + new.between(0.0, end), self.params, heading=m.yaw)
        for name in self._missions:
            if name == m.name:
                continue
            if not cells.isdisjoint(self.zones.held_set(name)):
                return
            if not self._in_service(name):
                continue
            t = self._reach_s(name, cells, (leg.in_line, short))
            if t is not None and t < through + SHORTCUT_MARGIN_S:
                return  # 다른 로봇이 지나가기 전에 겹치는 구간에 온다: 원래 선으로
        if not self.zones.acquire_all(m.name, cells):
            self.zones.cancel_all_requests(m.name)  # 막힘 표시 따위: 원래 선으로(줄 서지 않는다)
            return
        base = leg.in_line
        leg.route, leg.in_line = new, short
        leg.s, leg.s_lock, leg.s_sent = 0.0, end, 0.0
        leg.planned_at = self._clock()
        leg.stuck_since = leg.no_way_since = None
        leg.behind, leg.yields, leg.yield_since = (), (), None
        self.metrics["shortcuts"] += 1
        self._event("INFO", "FLOOR_SHORTCUT", f"takes {short} instead of {base} "
                    f"(through the shared stretch in {through:.1f} s)", m.name)

    def _reach_s(self, name: str, cells, ways) -> float | None:
        """name의 몸이 cells에 처음 닿기까지(초, ETA). 닿지 않거나, ways(같은 도크로 가는 선)를 가는
        로봇이거나(앞선 로봇은 따로 보고 뒤 로봇은 따라온다), 갈 길을 모르면 None. 도크에 서 있는
        로봇은 일이 끝난 뒤 그 도크의 출구선으로 나간다고 본다."""
        r, leg = self._missions[name], self._legs.get(name)
        if leg is not None and leg.in_line in ways:
            return None
        wait = 0.0
        if leg is not None and leg.route is not None:
            pts = [(r.x, r.y)] + leg.route.between(leg.s, leg.route.length)
        else:
            station = self._dock_here(r)
            out = self._line(station, "out") if station else None
            if out is None:
                return None
            pts = list(self.floor.fixed_lines[out].points)
            if leg is None:
                wait = self._work_left_s(r, station)
        until = self._until_cells(pts, cells)
        return None if until is None else wait + route_s(until, self.params, heading=r.yaw)

    def _work_left_s(self, r: _Mission, station: str) -> float:
        """도크에 선 로봇이 일을 마치기까지(초). 모르면 0: 곧 나갈 수 있다."""
        p = self.params
        if r.phase == DOCKING:
            return max(0.0, p.eta_dock_s - self._since(r)) + self._work_s(station)
        if r.phase == AT_STATION:
            left = self.station_left(r.name) if self.station_left else None
            if left is None:
                left = self._work_s(station) if self._since(r) < 1.0 else 0.0
            return left
        return 0.0

    def _until_cells(self, pts, cells) -> list | None:
        """꺾은선 pts 위를 가는 몸이 cells에 처음 닿는 곳까지의 꺾은선. 닿지 않으면 None."""
        step = self.floor.cell_m / 2
        done = [tuple(pts[0])]
        if not self._body_cells(*pts[0]).isdisjoint(cells):
            return done
        for a, b in zip(pts, pts[1:], strict=False):
            n = max(1, math.ceil(math.dist(a, b) / step))
            for k in range(1, n + 1):
                q = (a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
                if not self._body_cells(*q).isdisjoint(cells):
                    return done + [q]
            done.append(tuple(b))
        return None

    def _route(self, m: _Mission, start, goal, out_line=None, in_line=None, extra=frozenset(),
               cautious=False):
        """_route_past에 스테이션 도크의 몸 둘레 칸(_dock_keep)을 벽으로 더해 찾는다: 도크 옆을
        지나는 길이 도킹한 로봇의 칸에서 몸 하나만큼 떨어진다(그 옆에 서도 돌 수 있고, 도크를 나오는
        로봇과 서로 기다리지 않는다). 그렇게는 길이 없으면 예전처럼 찾는다."""
        if self._dock_keep:
            route = self._route_past(m, start, goal, out_line, in_line, extra, cautious,
                                     self._dock_keep)
            if route is not None:
                return route
        return self._route_past(m, start, goal, out_line, in_line, extra, cautious)

    def _route_past(self, m: _Mission, start, goal, out_line, in_line, extra, cautious,
                    keep=frozenset()):
        """서 있는 로봇이 쥔 칸을 피하고 남의 예약에 벌점을 준 경로. 없으면 피하지 않고 다시 찾는다.
        keep: 늘 벽으로 볼 칸 (i, j). 있으면 마지막의 아무것도 피하지 않는 길은 찾지 않는다.
        지나가는 로봇(가로지르거나 같은 쪽으로 감)의 칸은 곧 빈다: 피하지 않고 예약 벌점만 받는다
        (안전은 칸 잠금이 맡는다). 마주 오는 로봇은 비키지 않으므로 벽으로 피한다.
        마주 오는 로봇을 피할 길이 없으면, 서 있는 로봇을 피하지 않은 길이 아무와도 마주치지 않을 때
        그 길을 쓴다(서 있는 로봇 앞에서 기다리는 편이 한 방향 통로에서 맞서는 것보다 낫다).
        cautious(대기 자리로 가는 길): 급하지 않으니 남이 쥔 칸을 모두 피한다."""
        penalty = self._wait_for(m.name)
        walls = (self._others_cells(m.name) if cautious else self._wall_cells(m.name)) | keep
        # 지금 몸이 있는 내 칸만: 멀리 쥔 칸(앞 잠금)은 금지(extra)를 그대로 따른다
        mine = {self.floor.cell_at(c)
                for c in self.zones.held_set(m.name) & set(self._body_cells(*start))}
        route = self._plan_around(start, goal, out_line, in_line, walls, extra, penalty, mine)
        if route is not None and not cautious:
            oncoming = self._oncoming(m.name, route)
            if oncoming:
                walls |= {self.floor.cell_at(c) for n in oncoming for c in self.zones.held_set(n)}
                clear = self._plan_around(start, goal, out_line, in_line, walls, extra,
                                          penalty, mine)
                if clear is None or self._oncoming(m.name, clear):
                    # 서 있는 로봇을 벽으로 보지 않은 길: 그 앞에서 칸을 기다리게 된다
                    plain = self._plan_around(start, goal, out_line, in_line, set(keep), extra,
                                              penalty, mine)
                    if plain is not None and not self._oncoming(m.name, plain):
                        clear = plain
                route = clear or route
        if route is None and not extra and not keep:
            route = self.planner.leg(start, goal, out_line, in_line)
        return route

    def _plan_around(self, start, goal, out_line, in_line, walls, extra, penalty, mine=frozenset()):
        avoid = self._inflate(walls) | extra
        # 내가 쥔 칸으로는 나갈 수 있다: 붙어 선 로봇의 넓힌 칸에 출발 칸 둘레가 다 막혀도
        avoid -= {self.floor.cell_of(*start)} | mine
        return self.planner.leg(start, goal, out_line, in_line, frozenset(avoid), wait=penalty)

    def _headings(self, points, limit: float) -> dict:
        """꺾은선 앞 limit m를 몸 반지름으로 쓸 때 칸 (i, j) -> 그 칸을 지나는 진행 방향들(단위 벡터)."""
        out, walked, step = {}, 0.0, self.floor.cell_m / 2
        for a, b in zip(points, points[1:], strict=False):
            seg = math.dist(a, b)
            if seg <= 0:
                continue
            d = ((b[0] - a[0]) / seg, (b[1] - a[1]) / seg)
            for k in range(math.ceil(seg / step) + 1):
                t = min(seg, k * step, limit - walked)  # 조각 끝, 창 끝까지
                for c in disk_cells(self.floor, a[0] + d[0] * t, a[1] + d[1] * t, self._body):
                    ways = out.setdefault(c, [])
                    if d not in ways:
                        ways.append(d)
                if walked + t >= limit - 1e-9:
                    return out
            walked += seg
        return out

    def _oncoming(self, robot: str, route: Route) -> set:
        """route 앞을 마주 오는 지나가는 로봇: 둘의 앞길(계약 속도로 floor_plan_window_s)이 겹치는
        칸에서 방향이 120° 넘게 반대다. 가로지르거나 같은 쪽으로 가는 로봇은 들지 않는다."""
        reach = self.params.detour_speed_mps * self.params.floor_plan_window_s
        mine = self._headings(route.points, reach)
        out = set()
        for name, leg in self._legs.items():
            if name == robot or not self._passing(name):
                continue
            ahead = leg.route.between(leg.s, min(leg.route.length, leg.s + reach))
            theirs = self._headings(ahead, reach)
            if any(md[0] * d[0] + md[1] * d[1] < -0.5
                   for c, ways in theirs.items() for d in ways for md in mine.get(c, ())):
                out.add(name)
        return out

    def _inflate(self, cells) -> set:
        """칸 (i, j)들 -> 로봇 중심이 그 칸 어디에 있어도 몸이 cells 중 하나에 닿는 칸까지 넓힌 것.
        계획기는 중심이 지나는 칸만 보므로 남의 칸을 피하려면 몸 반지름만큼 넓혀서 넘긴다."""
        out = set()
        for i, j in cells:
            out.update((i + di, j + dj) for di, dj in self._reach)
        return out

    def _others_cells(self, robot: str) -> set:
        """다른 로봇(과 막힘 표시)이 쥔 칸 (i, j)."""
        return {self.floor.cell_at(c) for c in self.zones.held_by_others(robot)}

    def _passing(self, name: str) -> bool:
        """곧 칸을 비울 로봇: 바닥 경로를 가는 중이고 막히지 않았다(막힌 다리는 stuck_since가 있다).
        방금 새 길을 받아 MoveTo가 다음 tick에 나갈 로봇도 든다(양보한 로봇이 벽으로 보이면 상대도
        돈다). 고정 선 위, 비켜 서기, 대기 자리로 가는 다리, 길 끝에 닿은 다리는 아니다."""
        m, leg = self._missions.get(name), self._legs.get(name)
        if not (m is not None and m.available and leg is not None and leg.route is not None
                and not leg.done and not leg.waiting and leg.resume is None
                and leg.stuck_since is None and leg.route.fixed_at(leg.s) is None
                and not self.zones.waited_cells(name)):  # 남의 칸을 기다리면 곧 선다
            return False
        cap = self._line_cap(m, leg)
        return cap is None or leg.s_lock < cap - 1e-9  # 시간창: 설 자리까지 잠갔으면 서 있다

    def _wall_cells(self, robot: str) -> set:
        """계획이 벽으로 피할 칸 (i, j): 다른 로봇이 쥔 칸 중 지나가는 로봇의 것은 뺀다."""
        out = set()
        for c in self.zones.held_by_others(robot):
            if not self._passing(self.zones.holder(c)):
                out.add(self.floor.cell_at(c))
        return out

    def _can_start(self, robot: str, route: Route) -> bool:
        """route의 첫 구간(floor_extend_step_m)을 몸으로 쓸 칸이 남(막힘 표시 포함)의 칸에 닿지 않는다."""
        first = route.between(0.0, min(route.length, self.params.floor_extend_step_m))
        return self._swept(first).isdisjoint(self.zones.held_by_others(robot))

    def _blockers(self, robot: str) -> set:
        """robot이 기다리는 칸(못 잡은 칸)의 주인. 막힘 표시는 BLOCK_OWNER."""
        return {self.zones.holder(c) for c in self.zones.waited_cells(robot)} - {None, robot}

    def _can_go_around(self, name: str) -> bool:
        """_replan_if_stuck이 name에게 다른 바닥 길을 줄 수 있나: 바닥 경로를 가는 다리다
        (대기 자리로 가는 다리, 비켜 서기, 고정 선 위는 아니다)."""
        m, leg = self._missions.get(name), self._legs.get(name)
        return (m is not None and m.available and leg is not None and leg.route is not None
                and not leg.waiting and leg.resume is None and leg.route.fixed_at(leg.s) is None)

    def _waits_on(self, robot: str, other: str) -> bool:
        return not self.zones.held_set(other).isdisjoint(self.zones.waited_cells(robot))

    def _way_ahead(self, name: str, start) -> set:
        """name이 곧 갈 길(YIELD_AHEAD_M): 돌아가는 로봇의 중심이 들어가면 몸이 닿는 칸 (i, j).
        start 둘레는 뺀다: 돌아가는 로봇이 지금 그 길 위에 서 있어도 옆으로 벗어날 수 있게."""
        leg = self._legs.get(name)
        if leg is None or leg.route is None:
            return set()
        ahead = leg.route.between(leg.s, min(leg.route.length, leg.s + YIELD_AHEAD_M))
        near = disk_cells(self.floor, *start, 2 * self._body + self.floor.cell_m)
        return swept_cells(self.floor, ahead, 2 * self._body) - near

    def _spans(self, name: str) -> dict:
        """name이 계획한 길을 계약 속도로 달릴 때 칸 (i, j) -> [들어가는 시각, 나가는 시각, 방향 x, y]
        (floor_plan_window_s 앞까지, 방향은 그 칸에 들어설 때). 길이 없으면 빈 표.
        몸이 닿는 칸은 중심 칸 둘레(_reach)로 넉넉히 잡는다(중심이 칸 어디에 있어도 덮는다).
        로봇이 한 칸 나아갈 때마다 다시 만든다."""
        leg = self._legs.get(name)
        if leg is None or leg.route is None or not self._missions[name].available:
            return {}
        route, step = leg.route, self.floor.cell_m
        mark = int(leg.s / step)
        cached = self._span_cache.get(name)
        if cached is not None and cached[0] is route and cached[1] == mark:
            return cached[2]
        v = self.params.detour_speed_mps
        hold = self._body / v
        line = route.between(leg.s, min(route.length, leg.s + v * self.params.floor_plan_window_s))
        cell_of = self.floor.cell_of
        stops: list[list] = []  # [중심 칸, 닿는 시각, 떠나는 시각, 방향]
        walked = 0.0
        for p, q in zip(line, line[1:], strict=False):
            seg = math.dist(p, q)
            if seg <= 1e-9:
                continue
            way = ((q[0] - p[0]) / seg, (q[1] - p[1]) / seg)
            n = max(1, math.ceil(seg / step))
            for k in range(n + 1):
                d = seg * k / n
                cell = cell_of(p[0] + way[0] * d, p[1] + way[1] * d)
                at = (walked + d) / v
                if stops and stops[-1][0] == cell:
                    stops[-1][2] = at
                else:
                    stops.append([cell, at, at, way])
            walked += seg
        if not stops:
            stops.append([cell_of(*line[0]), 0.0, 0.0, (0.0, 0.0)])  # 길 끝에 섰다
        spans: dict[tuple, list] = {}
        for (i, j), t0, t1, way in stops:
            lo, hi = t0 - hold, t1 + hold
            for di, dj in self._reach:
                cell = (i + di, j + dj)
                span = spans.get(cell)
                if span is None:
                    spans[cell] = [lo, hi, way[0], way[1]]
                else:
                    if lo < span[0]:
                        span[0] = lo
                    if hi > span[1]:
                        span[1] = hi
        self._span_cache[name] = (route, mark, spans)
        return spans

    def _reservations(self, exclude: str | None = None) -> dict[str, list]:
        """칸 -> [(로봇, 들어가는 시각, 나가는 시각)]: 계획한 경로를 계약 속도로 달릴 때."""
        table: dict[str, list] = {}
        inside, ids = self.floor.inside, self.floor.cell_id
        for name in self._legs:
            if name != exclude:
                for cell, span in self._spans(name).items():
                    if inside(cell):
                        table.setdefault(ids(cell), []).append((name, span[0], span[1]))
        return table

    def _wait_for(self, robot: str):
        """계획기 wait: 남이 쓰는 시각에 그 칸에 닿으면 (빌 때까지 서 있는 값 m, 벌점 m).
        여럿이 연달아 쓰면 마지막 로봇이 비울 때까지 기다린다. 마주 오는 로봇의 칸은 기다려도
        비지 않는다: 기다림 대신 벌점 RESERVE_PENALTY_M."""
        others = [spans for spans in (self._spans(n) for n in self._legs if n != robot) if spans]
        v = self.params.detour_speed_mps
        lag = max(0.0, self.params.release_distance_m - self._body) / v  # 지나간 칸을 놓기까지

        def wait(cell, s: float, heading) -> tuple[float, float]:
            t = free = s / v
            flat, turn = 0.0, -0.5 * math.hypot(*heading)
            found = [span for span in (spans.get(cell) for spans in others) if span is not None]
            again = bool(found)
            while again:  # 기다린 뒤 시각에 또 걸리는 로봇이 있으면 이어서 기다린다
                again = False
                for span in found:
                    if not span[0] <= free <= span[1] + lag:
                        continue
                    if heading[0] * span[2] + heading[1] * span[3] < turn:
                        flat = RESERVE_PENALTY_M
                    elif span[1] + lag > free:
                        free, again = span[1] + lag, True
            return (free - t) * v, flat

        return wait if others else None

    # ---- crossings ------------------------------------------------------------------------

    def _moving(self, name: str) -> bool:
        """곧 지나갈 로봇: 길을 가는 중이고 잠금이 막히지 않았고 남에게 비켜 주느라 서 있지 않다.
        물체에 막혔거나(BLOCKED) 접근선 앞 설 자리까지 잠근 로봇은 서 있는다."""
        m, leg = self._missions.get(name), self._legs.get(name)
        if not (m is not None and m.available and leg is not None and leg.route is not None
                and not leg.done and leg.stuck_since is None and not leg.yields
                and m.robot_state != BLOCKED and not self.zones.waited_cells(name)):
            return False
        cap = self._line_cap(m, leg)
        if cap is not None and leg.in_line in self._short_route:
            return not self._at_stop(m, leg, cap)  # 구간 끝으로 달리는 동안은 지나가는 중이다
        return cap is None or leg.s_lock < cap - 1e-9

    def _crossing(self, a: str, b: str):
        """a와 b의 길이 엇갈리는 곳: (먼저 지날 로봇, a가 비켜 줄 칸). 같은 시각에 쓰는 칸이 없으면 None.
        먼저 지날 로봇은 엇갈리는 칸에 먼저 닿는 쪽, 시각 차가 YIELD_TIE_S 안이면 우선순위가 높은 쪽
        (같으면 choose_retreat 순서). 비켜 줄 칸은 상대가 엇갈리는 곳을 다 지날 때까지 쓸 칸에
        cell_margin_m을 두른 것이다(상대는 보고 자리 둘레도 잡는다: 보고 잡음만큼 길 밖으로 나간다).
        두 시각표와 우선순위가 그대로면 앞서 구한 답을 쓴다."""
        first, second = (a, b) if a < b else (b, a)
        sa, sb = self._spans(first), self._spans(second)
        rank = (self._priority(first), self._priority(second))
        found = self._cross_cache.get((first, second))
        if found is None or found[0] is not sa or found[1] is not sb or found[2] != rank:
            ta = tb = math.inf
            clear_a = clear_b = -math.inf
            for cell in sa.keys() & sb.keys():
                x, y = sa[cell], sb[cell]
                if x[0] > y[1] or y[0] > x[1]:
                    continue  # 다른 시각에 쓴다
                ta, tb = min(ta, x[0]), min(tb, y[0])
                clear_a, clear_b = max(clear_a, x[1]), max(clear_b, y[1])
            lead = None
            if ta < math.inf:
                if abs(ta - tb) < YIELD_TIE_S:
                    loser = choose_retreat([first, second], self._priority)
                    lead = second if loser == first else first
                else:
                    lead = first if ta < tb else second
            # [표, 표, 우선순위, 먼저 갈 로봇, 상대가 다 지나는 시각 둘, 비켜 줄 칸 둘(쓸 때 만든다)]
            found = [sa, sb, rank, lead, (clear_b, clear_a), [None, None]]
            self._cross_cache[first, second] = found
        if found[3] is None:
            return None
        side = 0 if a == first else 1
        if found[5][side] is None:
            theirs = sb if side == 0 else sa
            found[5][side] = self._ringed(c for c, span in theirs.items()
                                          if span[0] <= found[4][side])
        return found[3], found[5][side]

    def _ringed(self, cells) -> set[str]:
        """칸 (i, j)들에 cell_margin_m만큼(칸 수로 올림) 둘레를 두른 것의 칸 이름."""
        n = max(1, math.ceil(self.floor.cell_margin_m / self.floor.cell_m - 1e-9))
        ring = [(di, dj) for di in range(-n, n + 1) for dj in range(-n, n + 1)]
        out = set()
        for i, j in cells:
            out.update((i + di, j + dj) for di, dj in ring)
        inside, name = self.floor.inside, self.floor.cell_id
        return {name(c) for c in out if inside(c)}

    def _give_way(self, m: _Mission, leg: _Leg) -> dict[str, set]:
        """길이 엇갈리는 곳을 먼저 지날 로봇 -> 그 로봇이 지나갈 때까지 잡지 않을 칸.
        한 번 정한 차례는 엇갈림이 끝날 때까지 지킨다. 상대가 서 버렸거나(막힘) 상대도 이 로봇을
        먼저 보내기로 했으면 비켜 주지 않는다. 차례를 정할 때 이 로봇의 몸이 이미 상대가 갈 길
        위에 있으면(좁은 데서 마주 옴) 서서 기다려도 풀리지 않으므로 비켜 주지 않는다. 기다림이 floor_pass_wait_s + floor_plan_window_s를
        넘으면 이 다리에서는 그 로봇에게 더 비켜 주지 않는다(예전 재계획과 교착 처리가 맡는다).
        고정 선 위의 로봇은 비켜 주지 않는다: 도크를 드나드는 길이 병목이라 선을 빨리 비운다."""
        bans: dict[str, set] = {}
        if len(self._legs) > 1 and leg.route.fixed_at(leg.s) is None:
            held = self.zones.held_set(m.name)
            body = self._body_cells(m.x, m.y)
            for other, theirs in self._legs.items():
                if other == m.name or other in leg.no_yield or m.name in theirs.behind:
                    continue
                if not self._moving(other):
                    continue
                cross = self._crossing(m.name, other)
                if cross is None:
                    continue
                # 이미 비켜 주는 중이면 보고 잡음으로 몸이 걸쳐 보여도 차례를 지킨다
                if other in leg.behind or (cross[0] == other and cross[1].isdisjoint(body)):
                    bans[other] = cross[1] - held
        leg.behind = tuple(bans)
        return bans

    def _drive(self, m: _Mission) -> None:
        leg = self._legs.get(m.name)
        if leg is None:
            return
        if m.phase == TO_PARK and self._spot_taken(m):
            self._start_park_leg(m)  # someone stopped on the spot first: pick again
            leg = self._legs[m.name]
        if leg.waiting and self._admitted(m, leg):
            if m.goal:
                self._cancel_goal(m)
            self.zones.cancel_all_requests(m.name)
            self._plan(m, leg)  # its turn on the approach line
        if leg.route is None:
            if self._clock() - leg.planned_at >= self.params.floor_replan_s:
                self._plan(m, leg)
            return
        if leg.done and m.goal is None:
            if leg.waiting:
                return  # at the wait spot until the line is free
            if leg.resume is not None:
                self._after_sidestep(m, leg)
                return
            del self._legs[m.name]
            m.idx = len(m.path) - 1
            self._arrive(m)
            return
        if m.goal is None and self._clock() < m.retry_at:
            return
        self._take_shortcut(m, leg)
        grew = self._extend(m, leg)
        cap = self._line_cap(m, leg)
        if cap is not None and leg.s_lock >= cap - 1e-9:
            if (leg.in_line not in self._short_route  # 지름길은 구간 끝이 설 자리다
                    and self.line_free_in(leg.in_line, exclude=m.name) == math.inf):
                leg.admitted = False  # 앞 로봇이 언제 비킬지 모른다: 대기 자리로
                if m.goal:
                    self._cancel_goal(m)
                self._plan(m, leg)
                return
            if not self._at_stop(m, leg, cap):
                self._send(m, leg)  # 서 있으면 마지막 몇 cm를 다시 보내지 않는다(줄이 지워진다)
            leg.stuck_since = None  # 선 앞에서 선이 비기를 기다린다: 다른 길이 없다
            return
        self._send(m, leg)
        self._replan_if_stuck(m, leg, grew)

    def _at_stop(self, m: _Mission, leg: _Leg, cap: float) -> bool:
        """설 자리로 가는 MoveTo가 끝났고 설 자리 floor_extend_step_m 안이다."""
        return m.goal is None and cap - leg.s <= self.params.floor_extend_step_m

    def _replan_if_stuck(self, m: _Mission, leg: _Leg, grew: bool) -> None:
        """잠금이 floor_replan_s 동안 못 늘고 앞이 짧으면 남을 피해 다시 계획한다."""
        now = self._clock()
        route = leg.route
        short = leg.s_lock - leg.s < self.params.floor_extend_step_m
        if grew or not short or leg.s_lock >= route.length - AT_END_M:
            leg.stuck_since = None
            leg.no_way_since = None
            return
        if leg.stuck_since is None:
            leg.stuck_since = now
            return
        if (now - leg.stuck_since < self.params.floor_replan_s
                or now - leg.planned_at < self.params.floor_replan_s
                or route.fixed_at(leg.s) is not None):
            return  # inside a fixed line there is no other way
        if leg.yields:
            leg.stuck_since = now  # 막힌 시간은 비켜 주기가 끝난 때부터 센다
            return  # 엇갈리는 곳을 먼저 지날 로봇을 기다린다: 지나가면 제 길로 간다(_give_way)
        blockers = self._blockers(m.name)
        if (blockers and all(self._passing(b) for b in blockers)
                and now - leg.stuck_since < self.params.floor_pass_wait_s):
            return  # 지나가는 로봇: 곧 비킨다. 돌아가지도 대기 자리를 바꾸지도 않고 서서 기다린다
        if leg.waiting:
            if m.goal:
                self._cancel_goal(m)
            self.zones.cancel_all_requests(m.name)
            self._plan(m, leg)  # another wait spot (or the line, if it is free now)
            self.metrics["replans"] += 1
            return
        mutual = [b for b in blockers if b in self._missions and self._waits_on(b, m.name)]
        if (now - leg.stuck_since < self.params.floor_pass_wait_s
                and any(choose_retreat([m.name, b], self._priority) == b and self._can_go_around(b)
                        and self._legs[b].no_way_since is None for b in mutual)):
            # 서로 막았다: 둘 다 돌면 같은 쪽으로 돌다 또 만난다. 낮은 쪽(비켜 서기와 같은 순서)만
            # 돌고 이 로봇은 제 길을 지킨다. 낮은 쪽이 길을 못 찾았거나 floor_pass_wait_s가 지나면
            # 이 로봇이 돈다
            return
        if self._plan_budget <= 0:
            return  # try on the next tick
        self._plan_budget -= 1
        leg.planned_at = now
        old_left = route.length - leg.s
        here = self._here(m)
        # 막은 로봇은 지나가는 중이어도 벽으로 본다(오래 막았다)
        walls = self._inflate({self.floor.cell_at(c) for b in blockers
                               for c in self.zones.held_set(b)})
        ahead = set().union(*(self._way_ahead(b, here) for b in mutual)) if mutual else set()
        new = None
        # 상대의 앞길까지 피한 길, 안 되면 상대 칸만 피한 길. 첫 구간이 남의 칸에 닿으면 잡을 수 없다
        for avoid in ([walls | ahead] if ahead else []) + [walls | self._others_cells(m.name)]:
            new = self._route(m, here, leg.goal_xy, None, leg.in_line, extra=frozenset(avoid))
            if new is not None and self._can_start(m.name, new):
                break
            new = None
        # 비켜 설 자리도 없어 못 풂으로 고른 로봇: 곧 격리되므로 먼 길이라도 돈다
        last_resort = self._unresolved_at is not None and self._unresolved_robot == m.name
        if new is not None and not last_resort and self._oncoming(m.name, new):
            new = None  # 돌아갈 길에서 마주 오는 로봇을 만난다: 돌지 않고 기다린다
        if new is None or (new.length > old_left + 3.0 and not last_resort):
            if leg.no_way_since is None:
                leg.no_way_since = now  # 상대가 이를 보고 기다리지 않는다
            return
        leg.no_way_since = None
        if m.goal:
            self._cancel_goal(m)
        self.zones.cancel_all_requests(m.name)
        leg.route = new
        leg.s = leg.s_lock = leg.s_sent = 0.0
        leg.stuck_since = None
        leg.behind, leg.yields, leg.yield_since = (), (), None  # 새 길: 차례를 처음부터 정한다
        self.metrics["replans"] += 1

    # ---- wait cycles ----------------------------------------------------------------------

    def _check_deadlock(self, now: float) -> frozenset[str]:
        """대기 순환을 찾는다. 아무도 세우지 않는다: 순환에 든 로봇도 다시 계획을 이어 간다.

        deadlock_grace_s 이어지면 choose_retreat가 고른 로봇이 비켜 선다. 비켜 설 곳이 없거나
        비켜 선 뒤에도 같은 순환이 deadlock_unresolved_s 이어지면 그 로봇을 격리한다.
        """
        graph = wait_graph(self.zones)
        graph.pop(BLOCK_OWNER, None)
        cycle = find_cycle(graph)
        if not cycle:
            # a robot that got a shorter extension stops waiting for a tick: the same cycle
            # coming back within a second is the same episode
            if self._deadlock_key and now - self._deadlock_seen > 1.0:
                self._deadlock_key = ()
                self._unresolved_at = None  # 풀렸다: 먼 길 허락(못 풂)도 끝
            return frozenset()
        members, key = set(cycle), set(self._deadlock_key)
        same = now - self._deadlock_seen <= 1.0 and (
            members <= key or (members & key and not self._deadlock_acted))
        self._deadlock_seen = now
        if same:
            # robots that take turns getting a shorter lock change the cycle inside the same
            # group of robots: one episode over all of them
            self._deadlock_key = tuple(sorted(members | key))
        else:
            self._deadlock_key = tuple(sorted(cycle))
            self._deadlock_since = now
            self._deadlock_acted = False
            self._unresolved_at = None
            # free-floor robots meet head-on in narrow aisles and step aside: a warning here;
            # one that cannot be resolved is still an error (DEADLOCK_UNRESOLVED)
            self._event("WARN", "DEADLOCK_DETECTED", f"wait cycle {cycle}")
            return frozenset()
        if not self._deadlock_acted and now - self._deadlock_since >= \
                self.params.deadlock_grace_s:
            self._deadlock_acted = True
            # lowest priority first, as choose_retreat; one that cannot step aside (on a fixed
            # line, docking) passes the turn to the next
            turns = sorted(cycle, key=lambda name: (self._priority(name), name))
            if not any(self._sidestep(self._missions[name], cycle) for name in turns):
                robot = choose_retreat(cycle, self._priority)
                self._event("ERROR", "DEADLOCK_UNRESOLVED",
                            f"{robot} of {cycle} has no free spot to step aside to", robot)
                self._unresolved_at = now
                self._unresolved_robot = robot
        elif (self._deadlock_acted and self._unresolved_at is None
              and now - self._deadlock_since >= 2 * self.params.deadlock_grace_s + SIDESTEP_HOLD_S
              and not any(self._legs[name].resume for name in cycle if name in self._legs)):
            robot = choose_retreat(cycle, self._priority)
            self._event("ERROR", "DEADLOCK_UNRESOLVED",
                        f"{robot} of {cycle} stepped aside but the cycle remains", robot)
            self._unresolved_at = now
            self._unresolved_robot = robot
        elif (self._unresolved_at is not None
              and now - self._unresolved_at >= self.params.deadlock_unresolved_s):
            self._unresolved_at = None
            if self._unresolved_robot in cycle:
                self._fail(self._missions[self._unresolved_robot], "DEADLOCK_UNRESOLVED")
            else:
                # the robot found unresolved has left the cycle: judge the cycle as it is now
                self._deadlock_since = now
                self._deadlock_acted = False
        return frozenset()

    def _sidestep(self, m: _Mission, cycle) -> bool:
        """남의 칸과 남이 가려는 길에서 벗어난 가장 가까운 빈 자리로 비켜 선다."""
        if not m.available or m.phase not in (TO_WORK, TO_OUTBOUND, TO_PARK):
            return False
        leg = self._legs.get(m.name)
        if leg is None or leg.route is None or leg.route.fixed_at(leg.s) is not None:
            return False
        others = self._others_cells(m.name)
        wanted = set()
        for name in cycle:
            other = self._legs.get(name)
            if name != m.name and other is not None and other.route is not None:
                ahead = other.route.between(other.s, min(other.route.length, other.s + 1.2))
                wanted |= swept_cells(self.floor, ahead, self._body)
        blocked = others | wanted
        here = self._here(m)
        avoid_ids = self._inflate(others) | wanted
        spots = sorted((c for c in self.floor.clear
                        if 0.25 <= math.dist(here, self.floor.centre(c)) <= SIDESTEP_REACH_M),
                       key=lambda c: math.dist(here, self.floor.centre(c)))
        for cell in spots:
            spot = self.floor.centre(cell)
            if disk_cells(self.floor, *spot,
                          max(self._body, self.params.release_distance_m)) & blocked:
                continue
            route = self.planner.leg(here, spot, avoid=frozenset(avoid_ids - {self.floor.cell_of(
                *here)}))
            if route is None:
                continue
            if m.goal:
                self._cancel_goal(m)
            self.zones.cancel_all_requests(m.name)
            goal = m.path[-1]
            side = _Leg(route, spot, resume=(m.phase, m.dest, goal))
            self._legs[m.name] = side
            self.metrics["sidesteps"] += 1
            self._event("WARN", "FLOOR_SIDESTEP",
                        f"steps aside to ({spot[0]:.2f}, {spot[1]:.2f}) to break {cycle}",
                        m.name)
            return True
        return False

    def _after_sidestep(self, m: _Mission, leg: _Leg) -> None:
        now = self._clock()
        if leg.resume_at is None:
            leg.resume_at = now + SIDESTEP_HOLD_S
        if now < leg.resume_at:
            return
        phase, dest, goal = leg.resume
        self._start_leg(m, phase, dest, goal)

    def _spot_taken(self, m: _Mission) -> bool:
        leg = self._legs.get(m.name)
        if leg is not None and leg.resume is not None:
            return False  # stepping aside: the spot is not where it goes now
        holder = self.zones.holder(self._zone(m.path[-1]))
        return holder is not None and holder != m.name

    def _candidates(self, leg: _Leg, cap: float | None = None) -> list[float]:
        """Lock ends to try, best first. Inside a fixed line: its end, else stop before it.
        A rolling (one-way) line rolls like the floor, but inside a junction box span
        (_box_spans) it is the same: the span's end, else stop before it (no further than cap:
        where the span ends past cap, only the stop before it, and nothing once the lock is
        there: it waits until cap moves)."""
        route = leg.route
        want = min(route.length, leg.s + self.params.floor_lock_ahead_m)
        if cap is not None:
            want = min(want, cap)  # _line_cap
        ends, e = [], want
        while e > leg.s_lock + 1e-9:
            ends.append(e)
            e -= self.params.floor_extend_step_m
        out: list[float] = []
        rolling = self.floor.rolling_lines
        for e in ends:
            for s0, s1, line in route.fixed:
                if line in rolling:
                    continue  # 한 방향 선: 바닥처럼 굴린다
                if s0 + 1e-9 < e < s1 - 1e-9 and leg.s_lock < s1 - 1e-9:
                    picks = [s1] + ([s0] if s0 > leg.s_lock + 1e-9 else [])
                    break
            else:
                picks = [e]
                for s0, s1 in self._box_spans(route):
                    if s0 + 1e-9 < e < s1 - 1e-9 and leg.s_lock < s1 - 1e-9:
                        picks = ([s1] if cap is None or s1 <= cap + 1e-9 else []) + (
                            [s0] if s0 > leg.s_lock + 1e-9 else [])
                        break
            out.extend(p for p in picks if p not in out)
        return out

    def _extend(self, m: _Mission, leg: _Leg) -> bool:
        cap = self._line_cap(m, leg)
        if cap is not None and leg.s_lock >= cap - 1e-9:
            short = leg.in_line in self._short_route
            if not short and not self._at_stop(m, leg, cap):
                # 아직 달리는 중: 줄을 서지 않는다(지나온 칸을 아직 쥘 수 있어 떠나는 로봇과
                # 잠깐씩 순환이 생긴다). 서면 무엇을 쥐었든 줄을 선다(순환이 보이게)
                self.zones.cancel_all_requests(m.name)
                return False
            end = next(b for _, b, line in leg.route.fixed if line == leg.in_line)
            cells = self._lock_cells(leg.route, leg.s, end) | self._body_now(m)
            if self.zones.acquire_all(m.name, cells):  # 섰다: 선 전체에 줄을 선다
                leg.s_lock = end
                return True
            if short and not self._at_stop(m, leg, cap):
                # 지름길: 달리면서도 도크까지 한꺼번에 잡아 본다. 못 잡으면 달리는 동안은
                # 줄을 서지 않는다
                self.zones.cancel_all_requests(m.name)
            return False
        bans = self._give_way(m, leg)
        yields, queued = [], False
        for end in self._candidates(leg, cap):
            cells = self._drive_cells(m, leg, end, self._drive_path(m, leg, end))
            first = [name for name, ban in bans.items() if not cells.isdisjoint(ban)]
            if first:
                yields += [name for name in first if name not in yields]
                continue  # 먼저 갈 로봇이 쓸 칸: 잡지 않는다
            if self.zones.acquire_all(m.name, cells):
                leg.s_lock = end
                leg.yields, leg.yield_since = (), None
                return True
            queued = True  # 남이 쥔 칸에 줄을 섰다: 비켜 주는 것이 아니다
        if queued:
            yields = []
        leg.yields = tuple(yields)
        if yields:
            # 비켜 주는 동안은 줄을 서지 않는다(상대의 칸을 기다리면 대기 순환으로 보인다)
            self.zones.cancel_all_requests(m.name)
        if yields or (queued and leg.yield_since is not None):
            # 비켜 주다 줄을 서다 해도 기다린 시간은 잠금이 늘 때까지 이어서 센다
            now = self._clock()
            if leg.yield_since is None:
                leg.yield_since = now
            elif now - leg.yield_since >= (self.params.floor_pass_wait_s
                                           + self.params.floor_plan_window_s):
                leg.no_yield += tuple(n for n in bans if n not in leg.no_yield)
                leg.yields, leg.yield_since = (), None
        else:
            leg.yield_since = None
        return False

    def _send(self, m: _Mission, leg: _Leg) -> None:
        route = leg.route
        end = leg.s_lock
        final = end >= route.length - AT_END_M
        if m.goal is None:
            if not self._settled(m):
                return  # 첫 출발: 보고 자리가 가라앉을 때까지 기다린다
            if end - leg.s <= 0.01 and not final:
                return
            if final and math.dist((m.x, m.y), route.points[-1]) <= 0.01 and leg.s_sent >= end:
                if not self._active[m.name]:
                    leg.done = True  # 취소한 옛 MoveTo도 끝났다(로봇이 섰다)
                return
        elif m.goal.kind != MOVE:
            return
        elif end - leg.s_sent < self.params.floor_extend_step_m - 1e-9 and not (
                final and leg.s_sent < end - AT_END_M):
            return
        points = self._drive_path(m, leg, end)
        if not self.zones.acquire_all(m.name, self._drive_cells(m, leg, end, points)):
            if leg.yields:
                self.zones.cancel_all_requests(m.name)  # 비켜 주는 중: 줄을 서지 않는다
            return  # the first stretch from the reported pose is not free: wait
        poses = []
        for k, (x, y) in enumerate(points):
            a, b = (points[k], points[k + 1]) if k + 1 < len(points) else (points[k - 1],
                                                                            points[k])
            poses.append((x, y, math.atan2(b[1] - a[1], b[0] - a[0])))
        target = m.path[-1] if final else f"{m.path[-1]}~{end:.2f}"
        goal_id = self._next_goal_id()
        m.goal = _Goal(MOVE, goal_id, 1 if final else 0)
        self._started.add(m.name)
        m.stall_hold = False
        leg.s_sent = end
        self._active[m.name][goal_id] = Route(points)
        x, y, yaw = poses[-1]
        # final_goal: only the move that ends at the route's last point; every partial move
        # that ends at the rolling lock boundary says False (contract 0.8-draft proposal)
        self._out.append(MoveCmd(m.name, goal_id, target, x, y, yaw, tuple(poses),
                                 final or not self.params.mark_final_goal))

    def _drive_path(self, m: _Mission, leg: _Leg, end: float) -> list:
        """보낼 꺾은선: 로봇 자리(서 있으면 보고 평균)에서 경로 위 조금 앞(되돌아가지 않게)을 지나
        end까지."""
        route = leg.route
        ahead = route.between(min(leg.s + 0.05, end), end) if end - leg.s > 0.05 else [
            route.at(end)]
        here = self._here(m)
        # 평균 낸 자리는 달릴 때 뒤처진다: 길을 따라 진행점 옆으로 당긴다(옆으로 벗어난 만큼은 둔다).
        # LAG_M보다 더 뒤에 있으면 정말 뒤에 있는 것이다: 그 자리에서 잇는다
        back, _ = route.project(*here, max(0.0, leg.s - LAG_M), leg.s)
        if 1e-9 < leg.s - back < LAG_M - 1e-9:
            a, b = route.at(back), route.at(leg.s)
            here = (here[0] + b[0] - a[0], here[1] + b[1] - a[1])
        points = [here] + [p for p in ahead if math.dist(p, here) > 1e-9]
        if len(points) == 1:
            points.append(route.at(end))
        return points

    def _step_parked(self, m: _Mission) -> None:
        """대기 입구가 비도록 빈 지점으로 계속 앞으로 간다. 자세를 믿을 보고를 아직 못 받은 로봇이
        있으면 그 로봇이 선 칸을 모르니 기다린다(첫 보고 전의 빈 칸은 빈 칸이 아닐 수 있다)."""
        if any(not o.placed and o.robot_state in ("", LOCALIZING) and o.fault != OUT_OF_SERVICE
               for o in self._missions.values()):
            return  # 보고가 없거나 LOCALIZING뿐: 어디 섰는지 모른다
        if self._run_mode == RUN and self._park_goal(m, m.path[m.idx]):
            self._start_park_leg(m)

    # ---- 물러나기(제안, 실측 전) ---------------------------------------------------------------
    #
    # floor_block_retreat(혼자): 막힘 표시 뒤 floor_retreat_after_s 동안 BLOCKED가 이어지면
    # 막힌 칸을 벽으로 보고 지금 자리에서 다른 길을 찾는다(FLOOR_REROUTE). 없으면 보낸 경로
    # 기록을 거슬러 FORK_STEP_M씩, floor_retreat_max_m 안에서 다른 길이 시작되는 첫 점
    # (갈림길)을 찾아 물러날 칸을 한꺼번에 잡고 Retreat(경로 포함)를 낸다(FLOOR_RETREAT_START).
    # 물러나는 동안 몸과 물러날 칸을 쥐고 막힌 쪽 앞 칸은 놓는다. 다 물러나면
    # (FLOOR_RETREAT_DONE) 그 자리에서 다시 찾는다.
    # floor_corridor_clear(통로): 좁은 선이나 한 방향 선 위 로봇이 막힘 표시를 받으면 바로, 그
    # 로봇과 같은 선 뒤쪽 로봇들이 지나온 길을 거슬러 선 칸 밖으로 물러난다(FLOOR_CORRIDOR_CLEAR).
    # 맨 뒤부터: 물러날 칸을 한꺼번에 잡을 수 있는 로봇이 먼저 간다. 막힌 로봇이 다 나온 뒤
    # floor_retreat_after_s가 지나면 표시를 지우고 모두 다시 계획한다.
    # 못 하면(뒤 칸을 못 잡음, 갈림길이 없음, Retreat 실패) FLOOR_RETREAT_FAILED(WARN)이고 서서
    # floor_retreat_after_s 기다린 뒤 다시 본다. blocked_timeout_s 뒤 ERROR는 예전 그대로다.

    def _note_trail(self, m: _Mission, leg: _Leg) -> None:
        """보낸 경로 기록에 진행점까지를 적는다. 단계가 바뀌면 새로 적는다. 끝에서
        2 x floor_retreat_max_m만 둔다(통로를 비울 때 앞 로봇이 설 자리만큼 더 물러난다)."""
        trail = self._trail.get(m.name)
        if trail is None or trail.phase != m.phase:
            trail = self._trail[m.name] = _Trail(m.phase)
        if leg.route is None:
            return
        if trail.route is not leg.route:
            trail.route, trail.s = leg.route, 0.0
            new = leg.route.between(0.0, leg.s)
        elif leg.s - trail.s >= TRAIL_STEP_M:
            new = leg.route.between(trail.s, leg.s)
        else:
            return
        trail.s = leg.s
        pts = trail.points
        for p in new:
            if not pts or math.dist(p, pts[-1]) > 1e-6:
                pts.append(tuple(p))
        limit = 2 * self.params.floor_retreat_max_m
        total = sum(math.dist(a, b) for a, b in zip(pts, pts[1:], strict=False))
        while len(pts) > 2 and total - math.dist(pts[0], pts[1]) >= limit:
            total -= math.dist(pts[0], pts[1])
            pts.pop(0)

    def _foot(self, m: _Mission):
        """(기록 경로, 기록 위 로봇 자리 s, 로봇 자리, 기록에서 떨어진 거리). 기록이 없으면 None."""
        trail = self._trail.get(m.name)
        if trail is None or trail.phase != m.phase or len(trail.points) < 2:
            return None
        record = Route(list(trail.points))
        here = self._here(m)
        s, off = record.project(*here, max(0.0, record.length - FOOT_LOOK_M), record.length)
        return record, s, here, off

    @staticmethod
    def _way_back(record: Route, s: float, here, d: float) -> list:
        """here에서 기록을 거슬러 기록 위 s - d까지의 꺾은선."""
        way = [tuple(here)]
        for p in reversed(record.between(s - d, s)):
            if math.dist(p, way[-1]) > 1e-6:
                way.append(tuple(p))
        return way

    def _cut_trail(self, m: _Mission) -> None:
        """물러난 뒤: 기록을 지금 자리까지로 자른다(지나온 앞쪽은 다시 지나야 한다)."""
        found = self._foot(m)
        if found is not None:
            record, s, _, _ = found
            self._trail[m.name].points = record.between(0.0, s)

    def _marked(self) -> set[str]:
        return set(self.zones.held_by(BLOCK_OWNER))

    def _keeps_mark(self, name: str) -> bool:
        """물러났거나 새 길을 받은 로봇의 표시는 첫 풀린 보고에 지우지 않는다(물체를 더 못 본다)."""
        back = self._back.get(name)
        return back is not None and back.owner == name and (back.retreated
                                                            or back.state == REROUTED)

    def _blocked_again(self, m: _Mission, since: float, now: float) -> bool:
        """새 길을 받은 로봇이 그 뒤로 blocked_mark_s 넘게 BLOCKED다."""
        back = self._back.get(m.name)
        return (back is not None and back.state == REROUTED
                and now - max(since, back.since) >= self.params.blocked_mark_s)

    def _hold_back(self, back: _Back, now: float) -> None:
        # back.cells는 그대로: 결과 없이 놓친 Retreat면 로봇이 아직 물러나는 중일 수 있다.
        # 결과가 오거나(on_retreat_result) 다음 Retreat나 에피소드 끝에서 바뀐다
        back.state, back.until, back.tries = HOLD, now + self.params.floor_retreat_after_s, None
        back.leg, back.keep = None, frozenset()

    def _back_keep(self, m: _Mission, leg: _Leg | None) -> set[str]:
        """물러나는 동안 쥘 칸, 그리고 새 길이 거슬러 지나갈 한 방향 선의 칸(지날 때까지)."""
        back = self._back.get(m.name)
        if back is None:
            return set()
        keep = set(back.cells)
        if back.keep and leg is back.leg and leg.s < back.keep_until:
            keep |= back.keep
        return keep

    def _back_tick(self, now: float) -> None:
        for name, m in self._missions.items():
            back = self._back.get(name)
            if back is None:
                if self.running:
                    self._start_back(m, now)
                continue
            if m.fault == ERROR:
                del self._back[name]  # 격리: 표시와 칸은 예전처럼 둔다
                continue
            if back.state != BACKING and back.owner not in self._marks:
                self._end_back(name, plan=name != back.owner)  # 표시가 지워졌다(물체가 사라짐)
                continue
            if not self.running or not m.available:
                continue
            if back.state == SEARCH:
                self._search(m, back)
            elif back.state == HOLD and now >= back.until:
                self._after_hold(m, back)
            elif back.state == TURN:
                self._corridor_step(m, back, now)
            elif back.state == REROUTED and self._legs.get(name) is not back.leg:
                self._clear_mark(name, "the rerouted leg has ended")
                del self._back[name]
            elif back.state == BACKING and (m.goal is None or m.goal.kind != RETREAT):
                self._back_fail(m, back, "the retreat goal was dropped")
            elif back.state == OUT and name == back.owner:
                self._corridor_done(back, now)
        for name, back in list(self._back.items()):
            if back.kind == CORRIDOR and back.owner == name and back.state != OUT:
                self._join_corridor(back)

    def _start_back(self, m: _Mission, now: float) -> None:
        """막힘 표시를 받은 로봇: 통로 위면 바로 통로를 비우고, 아니면 floor_retreat_after_s 뒤
        혼자 물러난다(그동안 계속 BLOCKED일 때)."""
        name = m.name
        leg = self._legs.get(name)
        if (name not in self._marks or not m.available or leg is None
                or m.phase not in (TO_WORK, TO_OUTBOUND, TO_PARK) or m.goal is not None
                or self._active[name] or name in self._ahead):
            return
        line = self._corridor_of(leg) if self.params.floor_corridor_clear else None
        if line is not None:
            self._start_corridor(m, line)
        elif (self.params.floor_block_retreat and name in self._block_since
              and now - self._mark_at.get(name, now) >= self.params.floor_retreat_after_s):
            self._back[name] = _Back(SOLO, SEARCH, name)
            self._search(m, self._back[name])

    def _after_hold(self, m: _Mission, back: _Back) -> None:
        """기다림이 끝났다. 통로: 다시 물러나 본다. 혼자: 아직 막혔으면 다시 찾고, 아니면 지키던
        표시를 지우고 하던 다리를 다시 계획한다(물체가 그대로면 또 막혀 처음부터 한다)."""
        if back.kind == CORRIDOR:
            back.state = TURN
            return
        if m.name in self._block_since:
            back.state, back.tries = SEARCH, None
            self._search(m, back)
            return
        self._clear_mark(m.name, f"kept {self.params.floor_retreat_after_s:.0f} s after the "
                                 "retreat, planning again")
        self._end_back(m.name, plan=True)

    def _end_back(self, name: str, plan: bool) -> None:
        del self._back[name]
        leg = self._legs.get(name)
        if plan and leg is not None:
            m = self._missions[name]
            if m.goal is not None and m.goal.kind == MOVE:
                self._cancel_goal(m)
            self.zones.cancel_all_requests(name)
            self._plan(m, leg)

    def _clear_mark(self, owner: str, why: str) -> None:
        self._ahead.pop(owner, None)
        for cell in self._marks.pop(owner, ()):
            self.zones.release(BLOCK_OWNER, cell)
        self._mark_at.pop(owner, None)
        self._clear_n.pop(owner, None)
        self._event("INFO", "FLOOR_CLEARED", why, owner)

    def _back_fail(self, m: _Mission, back: _Back, why: str) -> None:
        self.zones.cancel_all_requests(m.name)
        self._event("WARN", "FLOOR_RETREAT_FAILED", why, m.name)
        self._hold_back(back, self._clock())

    # ---- 혼자: 다른 길, 갈림길까지 물러나기 ----

    def _search(self, m: _Mission, back: _Back) -> None:
        """지금 자리(뒤 거리 0)부터 기록을 거슬러 다른 길을 찾는다. tick마다 계획 예산만큼 본다."""
        leg = self._legs.get(m.name)
        if leg is None:
            self._end_back(m.name, plan=False)
            return
        found = self._foot(m)
        if back.tries is None:
            self._trim_mark(m)
            reach = min(found[1], self.params.floor_retreat_max_m - found[3]) if found else 0.0
            back.tries = [0.0] + [k * FORK_STEP_M
                                  for k in range(1, int(reach / FORK_STEP_M + 1e-9) + 1)]
            back.reach = reach
        while back.tries and self._plan_budget > 0:
            self._plan_budget -= 1
            d = back.tries.pop(0)
            if d > 0.0 and found is None:
                continue
            start = self._here(m) if d == 0.0 else found[0].at(found[1] - d)
            way = self._route_from(m, leg, start)
            if way is None:
                continue
            if d == 0.0:
                if self._reroute(m, leg, back, *way):
                    return
                continue
            record, s, here, _ = found
            self._retreat(m, back, self._way_back(record, s, here, d),
                          f"to the fork at ({start[0]:.2f}, {start[1]:.2f})")
            return
        if not back.tries:
            self._back_fail(m, back, f"no other way within {back.reach:.2f} m "
                                     "back along the way it came")

    def _trim_mark(self, m: _Mission) -> None:
        """표시는 로봇 몸 칸을 덮지 않는다(_take_ahead와 같다). 표시할 때의 보고 한 번과 지금 평균
        자리가 다르거나, 표시가 앞 경로를 칸 1/8 넓게 쓴 것(_swept)이 몸 칸에 걸치면 그 칸은
        로봇에게 돌려준다: 그대로면 그 자리에서 돌거나 떠날 칸(_swept)을 못 잡아 떠나지 못한다."""
        mine = self._marks.get(m.name)
        if not mine:
            return
        body = self._ids(disk_cells(self.floor, *self._here(m),
                                    self._body + self.floor.cell_m / 8)) & mine
        for cell in body:
            self.zones.release(BLOCK_OWNER, cell)
        mine -= body
        self.zones.acquire_all(m.name, body)

    def _route_from(self, m: _Mission, leg: _Leg, start):
        """start에서 leg의 목표까지 막힌 칸을 벽으로 본 길: (경로, 접근선, 통째로 잡을 한 방향 선
        칸, 그 칸을 지날 진행). 접근선은 다리의 것, 안 되면 그 지름길(또는 원래 선). 없으면 None."""
        marks = self._marked()
        walls = self._inflate({self.floor.cell_at(c) for c in marks})
        out_line = leg.out_line
        if out_line:
            line = self.floor.fixed_lines[out_line]
            if math.dist(start, line.start) > self.params.arrival_tolerance_m \
                    and rest_of_exit(start, line.points) is None:
                out_line = None
        lines = [leg.in_line]
        other = self._shortcut.get(leg.in_line) or self.floor.shortcuts.get(leg.in_line)
        if other:
            lines.append(other)
        # 나가던 선의 남은 부분이 막혔으면 선에서 바로 바닥으로 나간다
        ways = [(o, i) for o in ([out_line, None] if out_line else [None]) for i in lines]
        for out, in_line in ways:
            extra = frozenset(walls)
            for _ in range(2):
                route = self._route(m, start, leg.goal_xy, out, in_line, extra=extra)
                if route is None or not self._lock_cells(route, 0.0, route.length).isdisjoint(
                        marks):
                    break  # 좁은 선의 틈은 벽을 보지 않고 선을 따른다: 막힌 칸을 지나면 안 된다
                against, until = self._against(route)
                cells = set().union(*(self._line_ids[n] for n in against)) if against else set()
                if all(self.zones.holder(c) in (None, m.name) for c in cells):
                    return route, out, in_line, frozenset(cells), until
                extra = extra | frozenset(self._inflate({self.floor.cell_at(c) for c in cells}))
        return None

    def _against(self, route: Route) -> tuple[set[str], float]:
        """route의 바닥 경로 부분이 거슬러 따라가는 한 방향 선들과 그 선에서 벗어나는 진행.
        선에서 칸 하나 안을 선과 120° 넘게 반대로 가면 거스른다(가로지르기는 아니다)."""
        names, until = set(), 0.0
        rolling = self.floor.rolling_lines
        if not rolling:
            return names, until
        step = self.floor.cell_m / 2
        for k, (a, b) in enumerate(zip(route.points, route.points[1:], strict=False)):
            seg = math.dist(a, b)
            if seg <= 1e-9:
                continue
            d = ((b[0] - a[0]) / seg, (b[1] - a[1]) / seg)
            for i in range(math.ceil(seg / step) + 1):
                t = min(seg, i * step)
                s = route.cum[k] + t
                if route.fixed_at(s) is not None:
                    continue  # 제 고정 선 구간: 선을 따라 간다
                p = (a[0] + d[0] * t, a[1] + d[1] * t)
                for name in rolling:
                    pts = self.floor.fixed_lines[name].points
                    near = _nearest_on(p, pts)
                    if near is None:
                        continue
                    if near[0] <= self._body * 2:
                        if name in names:
                            until = max(until, s)
                        if near[0] > self.floor.cell_m:
                            continue
                        (x0, y0), (x1, y1) = pts[near[1]], pts[near[1] + 1]
                        length = math.dist((x0, y0), (x1, y1))
                        if (d[0] * (x1 - x0) + d[1] * (y1 - y0)) / length < -0.5:
                            names.add(name)
                            until = max(until, s)
        return names, until

    def _reroute(self, m: _Mission, leg: _Leg, back: _Back, route: Route, out_line, in_line,
                 cells, until: float) -> bool:
        """지금 자리에서 막힌 칸을 벽으로 본 새 길로 간다. 거스를 한 방향 선은 통째로 잡는다."""
        if cells and not self.zones.acquire_all(m.name, cells):
            self.zones.cancel_all_requests(m.name)
            return False
        if m.goal is not None:
            self._cancel_goal(m)
        self.zones.cancel_all_requests(m.name)
        if in_line != leg.in_line:
            leg.short_decided = True  # 지름길(또는 원래 선)로 바꿨다: 다시 고르지 않는다
        leg.route, leg.out_line, leg.in_line = route, out_line, in_line
        leg.s = leg.s_lock = leg.s_sent = 0.0
        leg.done = leg.waiting = False
        leg.planned_at = self._clock()
        leg.stuck_since = leg.no_way_since = None
        leg.behind, leg.yields, leg.yield_since = (), (), None
        back.state, back.leg, back.since = REROUTED, leg, self._clock()
        back.keep, back.keep_until = frozenset(cells), until
        self.metrics["reroutes"] += 1
        lines = sorted({n for n, ids in self._line_ids.items() if ids <= cells})
        self._event("INFO", "FLOOR_REROUTE",
                    f"another way to {m.path[-1]} ({route.length:.2f} m), the blocked cells "
                    "as walls" + (f", holding the one-way {lines} whole" if lines else ""), m.name)
        return True

    def _retreat(self, m: _Mission, back: _Back, way: list, why: str) -> bool:
        """way(첫 점은 로봇 자리)를 거슬러 물러난다. 몸과 물러날 칸을 한꺼번에 잡고 나머지는 놓는다.
        못 잡으면 False(혼자면 실패로 알린다)."""
        cells = self._swept(way) | self._body_now(m)
        taken = sorted({self.zones.holder(c) for c in cells} - {None, m.name})
        if taken or not self.zones.acquire_all(m.name, cells):
            self.zones.cancel_all_requests(m.name)
            if back.kind == SOLO:
                self._back_fail(m, back, f"cannot back up: cells behind held by {taken}")
            return False
        rest = self._rest_outside.get(m.name, {})
        for cell in list(self.zones.held_set(m.name) - cells):  # 막힌 쪽 앞 칸
            self.zones.release(m.name, cell)
            m.outside.pop(cell, None)
            rest.pop(cell, None)
        poses = []
        for k, (x, y) in enumerate(way):
            a, b = (way[k + 1], way[k]) if k + 1 < len(way) else (way[k], way[k - 1])
            poses.append((x, y, math.atan2(b[1] - a[1], b[0] - a[0])))  # 보는 방향(가는 쪽 반대)
        # 구간 길이를 더한 값이 한도를 소수 끝자리만큼 넘지 않게 한다(1.0000000000000002 m)
        distance = round(sum(math.dist(a, b) for a, b in zip(way, way[1:], strict=False)), 6)
        goal_id = self._next_goal_id()
        m.goal = _Goal(RETREAT, goal_id)
        self._out.append(RetreatCmd(m.name, goal_id, distance,
                                    self.params.floor_retreat_speed_mps, tuple(poses)))
        back.state, back.cells, back.retreated = BACKING, frozenset(cells), True
        back.goal_id = goal_id
        self.metrics["retreats"] += 1
        self._event("INFO", "FLOOR_RETREAT_START", f"backs {distance:.2f} m {why}", m.name)
        return True

    def on_retreat_result(self, robot, goal_id, success, reason="") -> None:
        m = self._missions.get(robot)
        back = self._back.get(robot)
        if back is not None and back.goal_id == goal_id and reason != "LOST":
            back.cells, back.goal_id = frozenset(), None  # 로봇이 섰다: 물러날 칸이 더 필요 없다
        # LOST는 섰는지 모른다: 칸을 그대로 쥔다. 늦은 결과, 다음 Retreat, 일의 끝에서 놓는다
        if m is None or m.goal is None or m.goal.id != goal_id or m.goal.kind != RETREAT:
            return
        m.goal = None
        if back is None:
            return
        self._cut_trail(m)
        if not success:
            self._back_fail(m, back, f"the retreat ended {reason}")
            back.fails += 1
            if back.kind == CORRIDOR and back.fails >= RETREAT_TRIES:
                back.state, back.out_at = OUT, self._clock()  # 그 자리에서 나온 것으로 친다
            return
        back.fails = 0
        self._event("INFO", "FLOOR_RETREAT_DONE", "backed up", robot)
        if back.kind == SOLO:
            back.state, back.tries = SEARCH, None  # 갈림길에서: 막힌 칸을 벽으로 다시 찾는다
        else:
            back.state = TURN

    # ---- 통로 비우기 ----

    def _corridor_of(self, leg: _Leg) -> str | None:
        """leg가 지금 가는 좁은 선이나 한 방향 선."""
        if leg.route is None:
            return None
        span = leg.route.fixed_at(leg.s)
        if span is None:
            return None
        line = span[2]
        return line if line in self.floor.narrow_lines or line in self.floor.rolling_lines else None

    def _behind_on(self, line: str, owner: str) -> list[tuple[float, str]]:
        """owner보다 선 위 뒤에 있는 로봇들 (선 위 거리, 이름): 몸이 선 칸에 닿고 다리를 가는 중."""
        ids = self._line_ids[line]
        pts = self.floor.fixed_lines[line].points
        mine = _along(pts, self._here(self._missions[owner]))[0]
        found = []
        for name, r in self._missions.items():
            back = self._back.get(name)
            if (name == owner or not r.available or name not in self._legs
                    or r.phase not in (TO_WORK, TO_OUTBOUND, TO_PARK)
                    or (back is not None and back.owner != owner)):
                continue
            here = self._here(r)
            if self._body_cells(*here).isdisjoint(ids):
                continue
            s = _along(pts, here)[0]
            if s < mine - 1e-6:
                found.append((s, name))
        return sorted(found)

    def _start_corridor(self, m: _Mission, line: str) -> None:
        names = [n for _, n in self._behind_on(line, m.name)] + [m.name]
        for name in names:
            self._enlist(name, m.name, line)
        self._set_targets(m.name)
        self.metrics["corridor_clears"] += 1
        self._event("INFO", "FLOOR_CORRIDOR_CLEAR",
                    f"{line} blocked in front of {m.name}: {names} back out of it, rear first",
                    m.name)

    def _enlist(self, name: str, owner: str, line: str) -> None:
        r = self._missions[name]
        if r.goal is not None and r.goal.kind == MOVE:
            self._cancel_goal(r)
        leg = self._legs[name]
        if leg.route is not None:
            leg.s_lock = leg.s_sent = leg.s
        self.zones.cancel_all_requests(name)
        self._back[name] = _Back(CORRIDOR, TURN, owner, line=line)

    def _join_corridor(self, owner_back: _Back) -> None:
        """통로를 비우는 동안 그 선에 들어온 뒤쪽 로봇도 함께 물러난다."""
        new = [n for _, n in self._behind_on(owner_back.line, owner_back.owner)
               if n not in self._back]
        for name in new:
            self._enlist(name, owner_back.owner, owner_back.line)
        if new:
            self._set_targets(owner_back.owner)
            self._event("INFO", "FLOOR_CORRIDOR_CLEAR",
                        f"{new} came onto {owner_back.line} behind: back out too", owner_back.owner)

    def _set_targets(self, owner: str) -> None:
        """설 곳: 몸이 선 칸에서 벗어나는 기록 위 첫 점에서, 앞(선 위 더 깊이)에 있는 로봇마다
        몸 둘과 칸 하나만큼 더 뒤. 기록이 모자라면 기록 처음까지."""
        members = [(n, b) for n, b in self._back.items() if b.owner == owner and b.target is None]
        if not members:
            return
        line = members[0][1].line
        ids = self._line_ids[line]
        pts = self.floor.fixed_lines[line].points
        depth = {n: _along(pts, self._here(self._missions[n]))[0]
                 for n, b in self._back.items() if b.owner == owner}
        spacing = 2 * self._body + self.floor.cell_m
        for name, back in members:
            found = self._foot(self._missions[name])
            if found is None:
                continue
            record, s, _, _ = found
            d = 0.0
            while d < s and not self._body_cells(*record.at(s - d)).isdisjoint(ids):
                d += self.floor.cell_m
            ahead = sum(1 for n, v in depth.items() if v > depth[name] + 1e-6)
            back.target = max(0.0, s - min(s, d) - ahead * spacing)

    def _corridor_step(self, m: _Mission, back: _Back, now: float) -> None:
        """제 차례면 다음 Retreat(floor_retreat_max_m 이하)를 낸다. 물러날 칸을 다 잡을 수 없으면
        다음 tick에 다시 본다(맨 뒤 로봇이 먼저 가게 된다)."""
        if m.goal is not None or self._rear_busy(m.name, back):
            return  # 뒤 멤버가 아직 나오지 않았다
        if self._active[m.name]:  # 취소한 이동의 결과가 아직 없다: 움직일 수 있다
            if back.wait_since is None:
                back.wait_since = now
            elif now - back.wait_since >= self.params.floor_retreat_after_s:
                self._event("WARN", "FLOOR_RETREAT_FAILED",
                            f"the cancelled move has not ended for "
                            f"{self.params.floor_retreat_after_s:.0f} s: stays where it is", m.name)
                back.state, back.out_at, back.wait_since = OUT, now, None
            return
        self._trim_mark(m)
        found = self._foot(m)
        if found is None or back.target is None:
            self._back_fail(m, back, "no record of the way it came to back along")
            back.state, back.out_at = OUT, now  # 다시 해도 같다: 그 자리를 끝으로 친다
            return
        record, s, here, off = found
        left = s - back.target
        if left <= BACK_DONE_M:
            back.state, back.out_at = OUT, now
            return
        d = min(left, self.params.floor_retreat_max_m - off)
        if d <= BACK_DONE_M:
            self._back_fail(m, back, f"{off:.2f} m off the way it came")
            back.state, back.out_at = OUT, now
            return
        way = self._way_back(record, s, here, d)
        if self._retreat(m, back, way, f"out of {back.line}"):
            back.wait_since = None
            return
        if back.wait_since is None:
            back.wait_since = now
        elif now - back.wait_since >= self.params.floor_retreat_after_s:
            # 뒤 칸이 오래 비지 않는다(선 밖에 선 로봇 따위): 그 자리에서 나온 것으로 친다
            cells = self._swept(way) | self._body_now(m)
            taken = sorted({self.zones.holder(c) for c in cells} - {None, m.name})
            self._event("WARN", "FLOOR_RETREAT_FAILED",
                        f"cells behind held by {taken} for {self.params.floor_retreat_after_s:.0f}"
                        f" s: stays {left:.2f} m short of where it was to stand", m.name)
            back.state, back.out_at, back.wait_since = OUT, now, None

    def _rear_busy(self, name: str, back: _Back) -> bool:
        """선 위 name보다 뒤(덜 깊이)에 있는 같은 무리 멤버 중 아직 나오지 않은 로봇이 있다.
        쓸 수 없는 로봇(두절 따위)은 기다리지 않는다: 그 칸은 잠금이 막는다."""
        pts = self.floor.fixed_lines[back.line].points
        mine = _along(pts, self._here(self._missions[name]))[0]
        return any(_along(pts, self._here(self._missions[n]))[0] < mine - 1e-6
                   for n, b in self._back.items()
                   if n != name and b.owner == back.owner and b.state != OUT
                   and self._missions[n].available)

    def _corridor_done(self, back: _Back, now: float) -> None:
        """막힌 로봇이 다 나오고 floor_retreat_after_s가 지나면 표시를 지우고 모두 다시 계획한다."""
        owner = back.owner
        group = [n for n, b in self._back.items() if b.owner == owner]
        if (now - back.out_at < self.params.floor_retreat_after_s
                or any(self._back[n].state != OUT and self._missions[n].available
                       for n in group)):
            return
        self._clear_mark(owner, f"kept {self.params.floor_retreat_after_s:.0f} s after "
                                f"{back.line} was cleared, planning again")
        for name in group:
            self._end_back(name, plan=self._missions[name].available)

    # ---- goal results -------------------------------------------------------------------

    def on_move_result(self, robot, goal_id, success, reason) -> None:
        m = self._missions.get(robot)
        if m:
            self._cancel_finished(m, goal_id)
            self._active[robot].pop(goal_id, None)  # any goal, old ones too: it has ended
        goal = m.goal if m else None
        if goal is None or goal.id != goal_id or goal.kind != MOVE:
            return
        m.goal = None
        m.not_sent_warned = False
        m.last_result = "SUCCEEDED" if success else reason
        leg = self._legs.get(robot)
        if success:
            m.attempts = 0
            if leg and leg.route and leg.s_sent >= leg.route.length - AT_END_M:
                leg.done = True
        elif reason == HEARTBEAT_LOST:
            m.retry_at = self._clock() + self.params.move_retry_wait_s
        else:
            self._goal_failed(m, reason)

    def on_goal_rejected(self, robot, goal_id) -> None:
        super().on_goal_rejected(robot, goal_id)
        if robot in self._active:
            self._active[robot].pop(goal_id, None)

    def on_goal_not_sent(self, robot, goal_id) -> None:
        super().on_goal_not_sent(robot, goal_id)
        if robot in self._active:
            self._active[robot].pop(goal_id, None)

    def on_goal_lost(self, robot, goal_id) -> None:
        super().on_goal_lost(robot, goal_id)
        if robot in self._active:
            self._active[robot].pop(goal_id, None)

    # ---- link recovery ------------------------------------------------------------------

    def _finish_recovery(self, m: _Mission) -> None:
        """통신이 돌아왔고 자세를 확인했다. 지금 몸 칸을 잡고 하던 다리를 그 자리에서 다시 잡는다.

        레인판은 지점 하나의 구역만 다시 잡고 쥔 구역을 레인 규칙으로 놓는다. 여기서는 칸을
        놓는 일을 보통 해제 규칙(_release_cells)에 맡긴다.
        """
        m.comm = ""
        need = self._body_cells(m.x, m.y)
        if not self.zones.acquire_all(m.name, need):
            in_the_way = sorted({self.zones.holder(c) for c in need} - {None, m.name})
            self._fail(m, "RECOVERY_CONFLICT")
            self._event("ERROR", "RECOVERY_CONFLICT",
                        f"{m.name} stands on cells held by {in_the_way}", m.name)
            return
        m.path = [self.site.nearest_waypoint(m.x, m.y), *m.path[-1:]]
        m.idx = 0
        m.settle_idx = None
        leg = self._legs.get(m.name)
        if leg is not None and m.phase in (TO_PARK, TO_WORK, TO_OUTBOUND):
            self._plan(m, leg)  # from where it stands now, same goal
            return
        here = self._dock_here(m)
        if m.phase == AT_STATION or (m.phase == DOCKING and here == m.dest):
            return  # still on its dock, as the lane recovery keeps it
        if m.dispatch:
            self._start_dispatch_leg(m)
            return
        m.path = [m.path[0]]
        near = math.dist((m.x, m.y), self.site.waypoint_pose(m.path[0]))
        m.phase = PARKED if (m.path[0] in self.site.parking
                             and near <= self.params.arrival_tolerance_m) else IDLE
        m.dest = None
