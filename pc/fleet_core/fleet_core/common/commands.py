"""FMS 코어의 명령, 결과, 뷰. ROS는 모른다.

앞부분은 ROS 래퍼가 goal과 메시지로 바꾸고, 뒷부분은 FleetManager 안에서만 오간다.
"""
from __future__ import annotations

from dataclasses import dataclass

SHIPPED = "SHIPPED"
PARTIAL = "PARTIAL"
HELD = "HELD"


@dataclass(frozen=True)
class OrderRecord:
    order_id: str
    items: dict[str, int]


@dataclass(frozen=True)
class MoveCmd:
    robot: str
    goal_id: int
    target_id: str
    x: float
    y: float
    yaw: float
    # 레인 폴리라인 (x, y, yaw)들. 첫 점은 로봇 위치, 끝 점은 목표. 비면 점 목표(point 모드)
    path: tuple = ()
    # 끝이 임무의 마지막 지점인가(버전 2 4절 horizon 최소판, 계약 0.8-draft 제안)
    final_goal: bool = True


@dataclass(frozen=True)
class RetreatCmd:
    """관제가 시키는 물러나기(제안). 로봇의 retreat 액션으로 간다. path가 비면 보는 방향 그대로
    곧게 distance_m 후진, 있으면 그 꺾은선 (x, y, yaw)들을 후진으로 따라간다(첫 점은 로봇 위치,
    끝 점은 설 곳. distance_m은 그 길이)."""
    robot: str
    goal_id: int
    distance_m: float
    speed_mps: float
    path: tuple = ()


@dataclass(frozen=True)
class DockCmd:
    robot: str
    goal_id: int
    dock_id: str


@dataclass(frozen=True)
class CancelCmd:
    robot: str  # 로봇 이름 또는 팔 작업 서버
    goal_id: int


@dataclass(frozen=True)
class FillCmd:
    goal_id: int
    robot: str
    station_id: str
    order_id: str
    items: dict[str, int]  # 바구니에 아직 없는 것만


@dataclass(frozen=True)
class SwapCmd:
    arm: str
    goal_id: int
    robot: str
    station_id: str
    order_id: str
    take_full: bool
    put_empty: bool


@dataclass(frozen=True)
class ReportOrderCmd:
    order_id: str
    robot: str
    status: str  # SHIPPED | PARTIAL | HELD
    placed: dict[str, int]
    missing: dict[str, int]
    detail: str


@dataclass(frozen=True)
class EventOut:
    level: str  # INFO | WARN | ERROR
    code: str
    detail: str
    robot: str = ""


@dataclass(frozen=True)
class ZoneView:
    zone_id: str
    holder: str
    queue: tuple[str, ...]
    acquired_at: float | None


@dataclass(frozen=True)
class AssignmentView:
    robot_id: str
    order_id: str
    station_id: str
    task_type: str  # TO_WORK | FILL | TO_OUTBOUND | SWAP | WAIT


@dataclass(frozen=True)
class RobotStatusView:
    robot_id: str
    fsm_state: str  # 핑키가 보낸 값 그대로
    task_type: str  # AssignmentView와 같음, 코스 밖(투입 전, OUT_OF_SERVICE)이면 빈 값
    order_id: str
    fault: str  # NONE | ERROR | COMM_LOST | OUT_OF_SERVICE | RECOVERING
    fault_reason: str
    wait_zone: str
    wait_robots: tuple[str, ...]
    battery: float
    x: float
    y: float
    yaw: float


# ---- inside the FMS: orders -> traffic --------------------------------------------------------


@dataclass(frozen=True)
class DispatchCmd:
    goal_id: int
    station_id: str
    robot_id: str = ""  # 비면 교통이 로봇을 고른다


@dataclass(frozen=True)
class CancelDispatchCmd:
    goal_id: int


@dataclass(frozen=True)
class ReleaseCmd:
    robot: str


@dataclass(frozen=True)
class RobotFaultCmd:
    """주문 쪽이 이 로봇을 포기했으니 교통도 실패로 다룬다."""

    robot: str
    reason: str


# ---- inside the FMS: orders -> arm -------------------------------------------------------------

# The station action written in the map, and the kind of an arm job
FILL = "fill"
SWAP = "swap"


@dataclass(frozen=True)
class ArmFillCmd:
    """One attempt to fill this robot's basket. The arm side sends a goal or answers ArmDeclined."""

    robot: str
    station_id: str
    order_id: str
    items: dict[str, int]  # only what is not in the basket yet


@dataclass(frozen=True)
class ArmSwapCmd:
    robot: str
    station_id: str
    order_id: str
    take_full: bool
    put_empty: bool


@dataclass(frozen=True)
class ArmStopCmd:
    """Cancel the running goal and wait for its result."""

    robot: str


@dataclass(frozen=True)
class ArmDropCmd:
    """Cancel the running goal and forget it. order_id and bound come back in ArmLost."""

    robot: str
    isolated: bool = False
    order_id: str = ""
    bound: int = 0


@dataclass(frozen=True)
class ArmFailedCmd:
    """The result just given was a failed attempt: count it; past the limit, fault the robot."""

    robot: str


@dataclass(frozen=True)
class ArmResetCmd:
    """This robot's arm work is done: attempts back to 0. With forget, its job record goes too."""

    robot: str
    forget: bool = False


ARM_COMMANDS = (ArmFillCmd, ArmSwapCmd, ArmStopCmd, ArmDropCmd, ArmFailedCmd, ArmResetCmd)


# ---- inside the FMS: arm -> orders -------------------------------------------------------------


@dataclass(frozen=True)
class ArmDeclined:
    """No goal was sent now (wait after a failure, server still stopping). Ask again."""

    robot: str


@dataclass(frozen=True)
class ArmFillResult:
    robot: str
    success: bool
    placed: dict[str, int]


@dataclass(frozen=True)
class ArmSwapResult:
    robot: str
    took_full: bool
    put_empty_done: bool


@dataclass(frozen=True)
class ArmLost:
    """A goal with no result: never answered (cleared false) or dropped. A late one may come."""

    robot: str
    goal_id: int
    server: str
    kind: str  # FILL or SWAP
    cleared: bool = False
    isolated: bool = False
    order_id: str | None = None  # as ArmDropCmd gave it; None: the orders take what is bound now
    bound: int = 0


@dataclass(frozen=True)
class ArmLate:
    """The result of a goal no robot waits for."""

    goal_id: int
    kind: str  # FILL or SWAP
    placed: dict[str, int] | None = None
    took_full: bool = False
    put_empty_done: bool = False


ARM_REPORTS = (ArmDeclined, ArmFillResult, ArmSwapResult, ArmLost, ArmLate)


# ---- inside the FMS: traffic -> orders --------------------------------------------------------

NO_ROBOT = "NO_ROBOT"  # 지금 읽는 피드백 단계는 이것뿐


@dataclass(frozen=True)
class RobotIsolated:
    """교통이 이 로봇을 ERROR로 세웠다. 구역은 쥐고, 주문 쪽이 주문을 되돌린다."""

    robot: str
    reason: str


@dataclass(frozen=True)
class DispatchFeedback:
    goal_id: int
    robot_id: str
    phase: str


@dataclass(frozen=True)
class DispatchResult:
    goal_id: int
    success: bool
    robot_id: str
    reason: str = ""
    error_x_cm: float = 0.0
    error_y_cm: float = 0.0
    error_yaw_deg: float = 0.0
