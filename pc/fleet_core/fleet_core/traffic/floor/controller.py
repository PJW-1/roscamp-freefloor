"""자유바닥 교통 제어기. TrafficController의 바깥 API를 그대로 지키고 이동과 잠금만 바꾼다.

P0 뼈대: 아직 레인 제어기의 동작을 그대로 물려받는다. 이후 파트에서 칸 잠금(P1),
한 대 계획과 굴러가는 잠금(P2), 여러 대(P3)로 바꾼다.
"""
from __future__ import annotations

from fleet_core.traffic.controller import TrafficController


class FloorController(TrafficController):
    def __init__(self, site, params, robots, clock, next_goal_id=None, floor=None):
        super().__init__(site, params, robots, clock, next_goal_id)
        self.floor = floor
