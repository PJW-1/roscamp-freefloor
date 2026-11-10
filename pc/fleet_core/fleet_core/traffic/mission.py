"""교통 제어기가 보는 로봇 하나: 단계, goal, 배차와 상수.

_Mission의 속성 셋 말고는 로직 없는 기록이다. TrafficController가 가진다.
"""
from __future__ import annotations

from dataclasses import dataclass, field

# 임무 단계
IDLE = "IDLE"
PARKED = "PARKED"
TO_PARK = "TO_PARK"
TO_WORK = "TO_WORK"
TO_OUTBOUND = "TO_OUTBOUND"
DOCKING = "DOCKING"
AT_STATION = "AT_STATION"
# 단계 대신 보이는 고장 상태
ERROR = "ERROR"
OUT_OF_SERVICE = "OUT_OF_SERVICE"
COMM_LOST = "COMM_LOST"
RECOVERING = "RECOVERING"
PLACEMENT_CONFLICT = "PLACEMENT_CONFLICT"
NO_FAULT = "NONE"  # 이상 없을 때 /fleet/robots에 나가는 값
# 로봇 상태 보고에 오는 값
# 로봇이 스스로 짧게 후진하고 다시 간다(계약 4절, 제안). 위 fault RECOVERING과 값만 같다.
# 이동 중으로 보되 막힘 시계와 멈춤 감시는 쉰다(blocking.py, controller._watch_stall).
SELF_RECOVERING = "RECOVERING"
ON_A_GOAL = ("MOVING", "DOCKING", SELF_RECOVERING)
BLOCKED = "BLOCKED"  # 경로 앞 물체로 섰다(계약 4절)
LOCALIZING = "LOCALIZING"  # 위치를 아직 못 잡았다. 자세를 믿지 않는다
NOT_PLACEABLE = ON_A_GOAL + (BLOCKED, LOCALIZING)  # 이 상태로는 지도에 올리지 않는다
HEARTBEAT_LOST = "HEARTBEAT_LOST"  # heartbeat가 끊겨 로봇이 스스로 끝낸 이동의 이유
_COMM_REASON = {COMM_LOST: "NO_STATE", RECOVERING: "CHECKING_POSE"}
STALLED = "STALLED"  # 목표가 열린 채 제자리
NOT_AT_TARGET = "NOT_AT_TARGET"  # 이동은 성공이라는데 자세는 아니라고 할 때

RUN = "RUN"
STOP = "STOP"

MOVE = "move"
DOCK = "dock"

# 맵에 적힌 스테이션 동작
FILL = "fill"
SWAP = "swap"


@dataclass
class _Goal:
    kind: str
    id: int
    target_idx: int = 0  # move 전용: 임무 경로의 인덱스
    step: tuple[float, float] | None = None  # move 전용: 진행 방향 단위 벡터


@dataclass
class _Dispatch:
    goal_id: int
    station_id: str


@dataclass
class _Block:
    """BLOCKED로 선 한 번. lane은 그때 달리던 레인 (a, b)."""
    lane: tuple
    total: float = 0.0  # BLOCKED로 보고된 시간의 합
    seen: float = 0.0  # total을 마지막으로 잰 시각
    clear_n: int = 0  # BLOCKED가 아닌 연속 보고 수
    marked: bool = False  # LANE_BLOCKED를 냈다
    still_n: int = 0  # 취소가 끝난 뒤 서 있는 연속 보고 수


@dataclass
class _Mission:
    name: str
    placed: bool = False
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    robot_state: str = ""
    battery: float = 0.0
    charge: bool = False  # 배터리가 낮다: 새 일을 받지 않고 충전 자리(대기 지점)에 있는다
    last_seen: float = 0.0
    phase: str = IDLE
    fault: str = ""  # ERROR 또는 OUT_OF_SERVICE, 둘 다 운영자가 처리
    fault_reason: str = ""
    comm: str = ""  # COMM_LOST 또는 RECOVERING
    recover_at: float = 0.0
    path: list = field(default_factory=list)
    idx: int = 0  # 지금 구역의 지점: 도달했거나 직전 지점에서 d 미만
    dest: str | None = None  # 가는 중이거나 도킹한 스테이션
    goal: _Goal | None = None
    settle_idx: int | None = None  # 재개한 goal이 레인을 벗어나지 않게 먼저 여기로 간다
    attempts: int = 0
    retry_at: float = 0.0  # 실패 뒤 이 시각 전에는 새 이동이나 도킹 없음
    arrival_by: float | None = None  # 성공이 자세보다 먼저 옴, 이 시각까지 보고를 기다린다
    heard_since_success: bool = False  # 성공 뒤 보고가 와서 자세가 새 값이다
    not_sent_warned: bool = False  # GOAL_NOT_SENT를 알렸고 그 뒤 서버에 닿은 goal 없음
    called: bool = False
    dispatch: _Dispatch | None = None  # 이 로봇을 지정한 배차
    hold: bool = False  # 놓아 주거나 다음으로 보낼 때까지 스테이션에 둔다
    await_pick: bool = False  # 할 일 없이 스테이션에 있음, 다음 선택 뒤에 움직인다
    outside: dict = field(default_factory=dict)  # 구역 -> 벗어난 연속 보고 수
    off_lane: int = 0  # 지금 달리는 레인에서 너무 먼 연속 보고 수
    off_lane_warned: bool = False
    wait_zone: str | None = None  # wait_since부터 기다리는 구역
    wait_since: float = 0.0
    wait_warned: bool = False
    stall_from: tuple[float, float] | None = None  # 멈춤을 재기 시작한 위치
    stall_since: float = 0.0
    stall_mean: tuple[float, float] | None = None  # 매끄럽게 한 보고 위치
    stall_seen: float = 0.0
    stall_hold: bool = False  # heartbeat로 선 goal을 다시 보낼 때까지 멈춤 시계를 이어 잰다
    block: _Block | None = None
    block_rejoin: tuple | None = None  # (레인, 풀린 시각, 합). 곧 같은 레인에서 막히면 이어 센다
    block_cancel: int | None = None  # 막힘 표시 때 취소한 goal. 결과가 오면 None. 풀려도 남는다
    queue_since: float | None = None  # BLOCKED로 줄 서기를 시작한 시각
    self_recovery_since: float | None = None  # 로봇이 RECOVERING을 보고하기 시작한 시각
    self_recovery_long: bool = False  # 이번 RECOVERING이 self_recovery_limit_s를 넘었다
    detour_at: object = None  # 우회를 고른 지점. 여기를 떠나면 detour_avoid는 뜻이 없다
    detour_avoid: tuple = ()  # 그 지점에서 포기한 다음 구역들. 되돌아가지 않는다
    lane_ref: tuple | None = None  # off_lane을 센 레인(a, b)
    off_lane_since: float | None = None  # 레인 이탈로 세운 시각
    off_lane_at: tuple | None = None  # 돌아와야 할 레인(a, b)
    back_on_lane: int = 0  # 그 레인 half_width 안의 연속 보고 수
    back_on_lane_at: float | None = None  # 그 수가 찬 시각. 다시 벗어나면 None
    last_result: str = ""  # 마지막 이동 goal 결과: SUCCEEDED 또는 실패 이유
    idle_since: float | None = None  # 조용히 서 있기 시작한 시각(idle_watch.py)
    idle_spot: tuple | None = None  # 마지막 ROBOT_IDLE의 (지점, 쥔 구역)
    idle_attempts: int = 0  # 그때의 시도 수. 같은 자리에서 다시 서면 이어 센다

    @property
    def live(self) -> bool:
        return self.placed and not self.fault

    @property
    def available(self) -> bool:
        return self.live and not self.comm and self.off_lane_since is None

    @property
    def state(self) -> str:
        return self.fault or self.comm or self.phase
