"""traffic_mode로 교통 층을 고른다. 주문 코어와 FleetManager의 바깥 API는 그대로다."""
from __future__ import annotations

from fleet_core.traffic.controller import TrafficController

LANES = "lanes"
FREEFLOOR = "freefloor"
MODES = (LANES, FREEFLOOR)


def build_traffic(site, params, robots, clock, next_goal_id, floor=None):
    """lanes는 레인 제어기 그대로, freefloor는 자유바닥 제어기."""
    mode = params.traffic_mode
    if mode == LANES:
        return TrafficController(site, params, robots, clock, next_goal_id)
    if mode == FREEFLOOR:
        # 실험 모듈은 고를 때만 읽는다. lanes는 이 모듈 없이도 같게 돈다
        from fleet_core.traffic.floor.controller import FloorController
        return FloorController(site, params, robots, clock, next_goal_id, floor)
    raise ValueError(f"traffic_mode must be one of {', '.join(MODES)}, not {mode!r}")
