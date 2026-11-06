"""관리자가 치운 로봇을 다시 투입한다(11-06 6번).

OUT_OF_SERVICE를 풀고 기록을 새로 만들어 맵 밖에 둔다. 다음 위치 보고 때 시작과 같은
절차(_place)로 올라온다. 시작 때와 달리 올라오기 전에도 다른 로봇은 멈추지 않는다.
이미 운행 중이고, 이 로봇은 치운 동안에도 잠금 없이 서 있었기 때문이다.
예외: 남이 쥔 구역에 보고되면(PLACEMENT_CONFLICT) 그 주인만 세운다. 목표와 구역 요청을
취소하고 새 배차와 호출을 주지 않는다. 진행 중인 지정 배차는 남겨 풀린 뒤 이어 간다.
둘 다 제자리면 저절로 풀리지 않는다. 관리자가 재투입 로봇을 옮겨 다시 보고하게 하거나
다시 치워야 한다.
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
        self._blocking.pop(robot, None)
        return True

    def _joined(self, m: _Mission) -> None:
        """맵에 올랐다. 이제 다른 로봇과 같다."""
        self._joining.discard(m.name)
        self._blocking.pop(m.name, None)

    def _placement_blocked(self, m: _Mission, in_the_way: dict) -> str:
        """재투입 로봇이 잠금 없이 남의 구역에 섰다. 그 주인을 세운다(목표 취소, 새 목표 없음).

        로봇이 맵에 오르거나 다시 치워질 때까지다. 이벤트 detail에 붙일 문구를 돌려준다.
        시작 때의 충돌은 전원이 기다리므로 여기서 하지 않는다.
        """
        if m.name not in self._joining:
            return ""
        holders = set(in_the_way.values())
        self._blocking[m.name] = holders
        for name in sorted(holders):
            held = self._missions[name]
            if held.goal:
                if held.block:
                    # 막힘이 열렸으면 _mark_block처럼 취소 결과를 기다린다
                    held.block_cancel = held.goal.id
                self._cancel_goal(held)
            # 줄에 남은 요청은 뒤 로봇까지 막는다. 풀리면 _request_next_zone이 다시 청한다.
            self.zones.cancel_all_requests(name)
            held.wait_zone = None
            held.called = False
        return f", {sorted(holders)} stop until it is placed or cleared"

    def _held_for_placement(self, robot) -> bool:
        """맵에 못 오른 재투입 로봇 때문에 세운 로봇이면 True."""
        for joiner, holders in self._blocking.items():
            m = self._missions[joiner]
            if robot in holders and not m.placed and m.fault != OUT_OF_SERVICE:
                return True
        return False
