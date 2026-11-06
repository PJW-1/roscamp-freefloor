"""관리자가 치운 로봇을 다시 투입한다(11-06 6번).

OUT_OF_SERVICE를 풀고 기록을 새로 만들어 맵 밖에 둔다. 다음 위치 보고 때 시작과 같은
절차(_place)로 올라온다. 시작 때와 달리 올라오기 전에도 다른 로봇은 멈추지 않는다.
이미 운행 중이고, 이 로봇은 치운 동안에도 잠금 없이 서 있었기 때문이다.
TrafficController 상태를 쓰므로 혼자 쓰지 않는다.
"""
from __future__ import annotations

from fleet_core.traffic.mission import OUT_OF_SERVICE, _Mission


class TrafficReinstate:
    def reinstate_refusal(self, robot) -> str:
        """다시 투입할 수 없는 이유, 할 수 있으면 "". 모르는 로봇은 FleetManager가 거른다."""
        m = self._missions.get(robot)
        if m is None:
            return f"no such robot {robot}"
        if m.fault != OUT_OF_SERVICE:
            return f"{robot} is {m.state}, not {OUT_OF_SERVICE}"
        held = self.zones.held_by(robot)
        if held:
            # 위치로 풀리지 않는 구역이다. 올라온 뒤 구역이 셋이 될 수 있다.
            return f"{robot} still holds {held}, confirm_clear them first"
        return ""

    def reinstate(self, robot) -> bool:
        """받아들이면 True. 거절이면 아무것도 안 바꾼다."""
        if self.reinstate_refusal(robot):
            return False
        old = self._missions[robot]
        # 마지막 보고 값만 남긴다. 자리 잡기는 새 보고로 한다.
        self._missions[robot] = _Mission(
            robot, x=old.x, y=old.y, yaw=old.yaw, robot_state=old.robot_state,
            battery=old.battery, last_seen=old.last_seen)
        self.zones.cancel_all_requests(robot)
        self._joining.add(robot)
        return True

    def _joined(self, m: _Mission) -> None:
        """맵에 올랐다. 이제 다른 로봇과 같다."""
        self._joining.discard(m.name)
