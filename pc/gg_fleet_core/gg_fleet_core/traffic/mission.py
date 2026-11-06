"""One robot as the traffic controller sees it: phase, goal, dispatch and the constants they use.

Plain records, no logic beyond the three properties of _Mission. TrafficController owns them.
"""
from __future__ import annotations

from dataclasses import dataclass, field

# Mission phases.
IDLE = "IDLE"
PARKED = "PARKED"
TO_PARK = "TO_PARK"
TO_WORK = "TO_WORK"
TO_OUTBOUND = "TO_OUTBOUND"
DOCKING = "DOCKING"
AT_STATION = "AT_STATION"
# Fault states shown instead of the phase.
ERROR = "ERROR"
OUT_OF_SERVICE = "OUT_OF_SERVICE"
COMM_LOST = "COMM_LOST"
RECOVERING = "RECOVERING"
PLACEMENT_CONFLICT = "PLACEMENT_CONFLICT"
NO_FAULT = "NONE"  # what /fleet/robots says when nothing is wrong
ON_A_GOAL = ("MOVING", "DOCKING")  # what a robot says in its state report
_COMM_REASON = {COMM_LOST: "NO_STATE", RECOVERING: "CHECKING_POSE"}
NOT_AT_TARGET = "NOT_AT_TARGET"  # a move said it succeeded, and the pose says otherwise

RUN = "RUN"
STOP = "STOP"

MOVE = "move"
DOCK = "dock"

# Station actions, as written in the site map.
FILL = "fill"
SWAP = "swap"


@dataclass
class _Goal:
    kind: str
    id: int
    target_idx: int = 0  # move only: index into the mission path
    step: tuple[float, float] | None = None  # move only: direction of travel, a unit vector


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
    fault: str = ""  # ERROR or OUT_OF_SERVICE, both need the operator
    fault_reason: str = ""
    comm: str = ""  # COMM_LOST or RECOVERING
    recover_at: float = 0.0
    path: list = field(default_factory=list)
    idx: int = 0  # the waypoint whose zone it is in: reached, or less than d from the last one
    dest: str | None = None  # station it is driving to or docked at
    goal: _Goal | None = None
    settle_idx: int | None = None  # drive here first, so a resumed goal stays on the lane
    attempts: int = 0
    retry_at: float = 0.0  # no new move or dock before this, after a failed attempt
    arrival_by: float | None = None  # a success came ahead of the pose: reports have until then
    heard_since_success: bool = False  # a report came after that success, so the pose is news
    not_sent_warned: bool = False  # GOAL_NOT_SENT said, and no goal has reached a server since
    called: bool = False
    dispatch: _Dispatch | None = None  # a dispatch that named this robot
    hold: bool = False  # kept at the station until it is released or sent on
    await_pick: bool = False  # at a station with no duty: acts after the next pick, not before
    outside: dict = field(default_factory=dict)  # zone -> consecutive reports clear of it
    off_lane: int = 0  # consecutive reports too far off the lane it is driving
    off_lane_warned: bool = False
    wait_zone: str | None = None  # zone it has been queueing for since wait_since
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
