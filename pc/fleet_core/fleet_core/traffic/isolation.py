"""자동 격리(지시서 5.5): 지도에 오른 로봇이 ERROR가 되는 순간 한 번 한다.

  1. 구역은 그대로 쥔다. 몸이 거기 있으므로 confirm_clear 전에는 놓지 않는다.
  2. RobotIsolated를 내보낸다. 주문 쪽이 받아 그 로봇의 주문을 대기 맨 앞으로 되돌린다.
  3. ROBOT_ISOLATED(ERROR)를 한 번 낸다. detail에 이유가 있다.
  4. 열린 요청(구역 요청, 지정 배차, 진행 중인 goal)은 _fail과 여기서 취소한다.
다른 로봇은 그 구역을 비켜 가고, 길이 없으면 기다린다.

TrafficController에서 떼어 낸 부분이다. 제어기 상태를 쓰므로 혼자 쓰지 않는다.
"""
from __future__ import annotations

from fleet_core.common.commands import RobotIsolated
from fleet_core.traffic.mission import _Mission


class TrafficIsolation:
    def _isolate(self, m: _Mission, reason: str) -> None:
        """_fail이 ERROR로 바꾼 직후 부른다. 이미 ERROR였거나 지도에 없으면 하지 않는다."""
        if not m.placed:
            return
        if m.goal:
            self._cancel_goal(m)
        self._out.append(RobotIsolated(m.name, reason))
        self._event("ERROR", "ROBOT_ISOLATED",
                    f"{reason}: stopped where it is, holding {self.zones.held_by(m.name)}",
                    m.name)
