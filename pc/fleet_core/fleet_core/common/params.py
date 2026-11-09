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
    # 우회(지시서 5번). 이 값이 없던 기록은 우회 없이 재생한다. 파일의 기본은 true다
    detour_enabled: bool = False
    station_wait_estimate_s: float = 30.0
    detour_speed_mps: float = 0.2
    off_lane_timeout_s: float = 20.0
    # path: 레인 폴리라인(FollowPath), point: 지점 하나(비교용). 이 값이 없던 기록은 point였다.
    lane_goal_mode: str = "point"
    # 해제 보강: 구역의 다른 바닥 선에서도 d 이상(예각 합류). 이 값이 없던 기록은 예전 규칙
    release_clear_of_floor: bool = False
    # H1: after DEADLOCK_UNRESOLVED, the same wait cycle lasting this long isolates one robot.
    deadlock_unresolved_s: float = 30.0
    # 조용히 서 있는 로봇 감시(1.1 지시서 H2). 0이면 끈다. 이 값이 없던 기록은 감시 없이 재생한다
    idle_watch_s: float = 0.0
    # Proposal (work order 1.1, H3): a failed outbound dispatch is sent again after the wait,
    # and failing more than the limit is an ERROR. Records without these values replay with them
    dispatch_retry_wait_s: float = 2.0
    dispatch_retry_limit: int = 3
    # Proposal (pinky self-recovery, 2026-11-09): a robot that keeps reporting RECOVERING longer
    # than this is taken as BLOCKED. The pinky FSM's two attempts take about 35 s at most.
    # Records from before RECOVERING never report it, so the value does not change their replay
    self_recovery_limit_s: float = 45.0
    # Proposal (version 2 section 4, contract 0.8-draft MoveTo.final_goal): mark goals that end
    # at a lock boundary as not final. Records without it replay with every move final
    mark_final_goal: bool = False
    # Free-floor experiment (P0): which traffic layer runs. lanes is the lane control as it was;
    # freefloor is fleet_core/traffic/floor. Records from before this value replay with lanes
    traffic_mode: str = "lanes"
    # Free-floor traffic only (traffic/floor). Proposals from the experiment order, not measured
    floor_lock_ahead_m: float = 0.5  # cells locked ahead of the robot along its route
    floor_extend_step_m: float = 0.1  # a shorter all-or-nothing try when the full one fails
    floor_replan_s: float = 1.0  # a robot whose lock cannot grow looks for another route
    floor_plan_window_s: float = 8.0  # reservations of other robots looked at this far ahead
    floor_pass_wait_s: float = 4.0  # 지나가는 로봇에 막히면 다른 길을 찾기 전에 이만큼 서서 기다린다
    # 자유바닥 ETA와 접근선 시간창(제안, 실측 전). eta_*는 버전 2 공식 ETA와 같은 값
    eta_speed_mps: float = 0.2
    eta_turn_s_per_rad: float = 1.0  # 제자리 회전 초/rad. 0이면 회전을 안 셈
    eta_turn_min_rad: float = 0.25  # 이보다 작은 꺾임은 달리면서 지남(RPP 회전 기준)
    eta_dock_s: float = 3.0
    eta_fill_s_per_item: float = 4.0
    eta_fill_base_s: float = 4.0  # 첫 품목 전 준비 시간(가짜 팔은 품목 수 + 1번)
    eta_fill_items_default: int = 2  # 주문을 모를 때 채울 품목 수
    eta_swap_s: float = 8.0
    floor_leave_s: float = 2.5  # 도크를 떠나기 시작해서 접근선이 빌 때까지
    # 끔: 접근선은 먼저 온 순서. 켬: 가장 빨리 닿는 로봇이 다음이고, 비울 때에 맞춰 미리 출발
    floor_dock_schedule: bool = False
    floor_pre_admit_margin_s: float = 1.0  # 딱 맞는 때보다 이만큼 먼저 출발
    floor_dock_aging_s: float = 30.0  # 이만큼 기다리면 다시 먼저 온 순서(굶김 방지)


def load_params(path=None) -> FleetParams:
    source = Path(path) if path else DEFAULT_PARAMS
    return FleetParams(**yaml.safe_load(source.read_text(encoding="utf-8")))
