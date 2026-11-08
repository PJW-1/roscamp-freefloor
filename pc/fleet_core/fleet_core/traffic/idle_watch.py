"""조용히 서 있는 로봇 감시(1.1 지시서 H2): ROBOT_IDLE.

목적지가 있는데(주행 단계) 열린 goal도 구역 대기표도 없이 idle_watch_s 동안 선 로봇.
멈춤 감시는 goal이 있어야 돌므로 이런 로봇은 아무도 보지 않는다. 예: goal이 지점
arrival_tolerance_m 안에서 덜 가 끝나 뒤 구역을 놓지 못하면 다음 구역을 청하지 않는다.
구역 해제 규칙은 바꾸지 않는다(안전 규칙). 대신 마저 가게 한다.
  - ROBOT_IDLE(WARN)을 한 번 낸다. detail에 단계, 쥔 구역, 마지막 goal 결과.
  - 지금 지점으로 가는 goal을 곧바로 다시 보낸다. 실패한 시도 하나로 센다(move_retry_limit).
    같은 자리에서 다시 서면 그 사이 성공이 수를 지우지 않는다.
  - 한도를 넘으면 GOAL_FAILED(ERROR)로 격리한다.
대상이 아닌 로봇은 _idle_watch_applies에 모았다. idle_watch_s가 0이면 끈다(이 값이 없던 기록).

TrafficController에서 떼어 낸 부분이다. 제어기 상태를 쓰므로 혼자 쓰지 않는다.
"""
from __future__ import annotations

from fleet_core.traffic.mission import RUN, TO_OUTBOUND, TO_PARK, TO_WORK, _Mission

ROBOT_IDLE = "ROBOT_IDLE"
DRIVING = (TO_WORK, TO_OUTBOUND, TO_PARK)


class TrafficIdleWatch:
    def _idle_watch_applies(self, m: _Mission, now: float) -> bool:
        """감시 대상인가. 스테이션 작업과 도킹, STOP·PAUSE, 격리·OUT_OF_SERVICE, 두절과 복구 확인,
        레인 이탈, 재투입 충돌로 세움, 교착 멈춤, 막힘 표시와 그 취소 결과 대기, 재시도와 도착
        확인 대기는 다른 감시나 사람이 맡는다."""
        return (self.params.idle_watch_s > 0 and self._run_mode == RUN
                and self.everyone_placed and self.available(m.name)
                and m.phase in DRIVING and m.goal is None
                and self.zones.waiting_for(m.name) is None
                and not m.block and m.block_cancel is None
                and m.arrival_by is None and now >= m.retry_at
                and self._deadlock_since is None)

    def _watch_idle(self, m: _Mission, now: float) -> None:
        if not self._idle_watch_applies(m, now):
            m.idle_since = None
            return
        if m.idle_since is None:
            m.idle_since = now
            return
        if now - m.idle_since < self.params.idle_watch_s:
            return
        held = sorted(self.zones.held_by(m.name))
        waypoint = m.path[m.idx]
        self._event("WARN", ROBOT_IDLE,
                    f"{m.phase} at {waypoint} for {now - m.idle_since:.0f} s with no goal and "
                    f"no zone request, holding {held}, last goal {m.last_result or 'none'}; "
                    f"sending {waypoint} again", m.name)
        m.idle_since = None
        spot = (waypoint, tuple(held))
        if m.idle_spot == spot:
            m.attempts = max(m.attempts, m.idle_attempts)
        m.idle_spot = spot
        m.settle_idx = m.idx  # _issue_move가 이 지점 goal을 다시 낸다
        self._goal_failed(m, ROBOT_IDLE, announce=False)
        m.idle_attempts = m.attempts
        m.retry_at = now  # 이미 idle_watch_s 동안 서 있었다. 이번 tick에 보낸다
