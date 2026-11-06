"""플릿 매니저 정책 값. data/fleet_params.yaml에서 읽는다."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

DEFAULT_PARAMS = Path(__file__).parents[1] / "data" / "fleet_params.yaml"


@dataclass(frozen=True)
class FleetParams:
    basket_capacity: int
    ship_partial: bool
    robot_radius_m: float
    release_margin_extra_m: float
    release_distance_m: float
    release_confirm_samples: int
    lane_error_warn_m: float
    move_retry_limit: int
    move_retry_wait_s: float
    arrival_tolerance_m: float
    arrival_settle_s: float
    startup_wait_s: float
    comm_timeout_s: float
    heartbeat_period_s: float
    heartbeat_timeout_s: float
    recover_check_s: float
    arm_cancel_wait_s: float
    arm_retry_wait_s: float
    fill_timeout_s: float
    swap_timeout_s: float
    cancel_retry_s: float
    cancel_warn_after: int
    report_retry_s: float
    zone_wait_warn_s: float
    deadlock_grace_s: float
    score_same_call: float
    score_distance_weight: float
    priority_at_station: int
    priority_has_cargo: int
    # 기본값: 이 값이 없던 기록도 재생된다.
    input_pose_margin_m: float = 1.0
    stall_s: float = 6.0
    stall_move_m: float = 0.05
    stall_smooth_s: float = 1.0
    dock_stall_s: float = 15.0
    blocked_mark_s: float = 5.0
    blocked_timeout_s: float = 60.0
    blocked_rejoin_s: float = 10.0
    queue_gap_m: float = 0.6
    off_lane_timeout_s: float = 20.0


def load_params(path=None) -> FleetParams:
    source = Path(path) if path else DEFAULT_PARAMS
    return FleetParams(**yaml.safe_load(source.read_text(encoding="utf-8")))
