"""The commands, results and views of the FMS cores.

The first part is what the ROS wrapper turns into goals and messages. The second part is what
the orders and traffic modules say to each other through FleetManager, and never reaches ROS.
Nothing here knows about ROS.
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


@dataclass(frozen=True)
class DockCmd:
    robot: str
    goal_id: int
    dock_id: str


@dataclass(frozen=True)
class CancelCmd:
    robot: str  # robot name, or the server of an arm job
    goal_id: int


@dataclass(frozen=True)
class FillCmd:
    goal_id: int
    robot: str
    station_id: str
    order_id: str
    items: dict[str, int]  # only what is still missing from the basket


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
    fsm_state: str  # what the pinky reported, as it reported it
    task_type: str  # as in AssignmentView, empty when off the floor
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
    robot_id: str = ""  # empty: traffic picks the robot


@dataclass(frozen=True)
class CancelDispatchCmd:
    goal_id: int


@dataclass(frozen=True)
class ReleaseCmd:
    robot: str


@dataclass(frozen=True)
class RobotFaultCmd:
    """The arm side gave up on this robot, so traffic must treat it as failed."""

    robot: str
    reason: str


# ---- inside the FMS: traffic -> orders --------------------------------------------------------

NO_ROBOT = "NO_ROBOT"  # the only feedback phase anything reads today


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
