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
ON_A_GOAL = ("MOVING", "DOCKING")  # 로봇 상태 보고에 오는 값
_COMM_REASON = {COMM_LOST: "NO_STATE", RECOVERING: "CHECKING_POSE"}
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
class _Mission:
    name: str
    placed: bool = False
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    robot_state: str = ""
    battery: float = 0.0
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
    off_lane: int = 0  # 주행 레인에서 너무 먼 연속 보고 수
    off_lane_warned: bool = False
    wait_zone: str | None = None  # wait_since부터 기다리는 구역
    wait_since: float = 0.0
    wait_warned: bool = False

    @property
    def live(self) -> bool:
        return self.placed and not self.fault

    @property
    def available(self) -> bool:
        return self.live and not self.comm

    @property
    def state(self) -> str:
        return self.fault or self.comm or self.phase
