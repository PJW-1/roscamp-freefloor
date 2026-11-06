"""레인 이탈: 많이 벗어난 로봇을 세우고, 레인에 돌아오면 같은 목표를 다시 보낸다.

벗어난 로봇의 실제 위치는 모른다. 그동안 구역을 쥔 채 해제 판단을 멈춘다.
통신 상태(m.comm)와 따로 둔다. /fleet/robots에는 RECOVERING, OFF_LANE으로 보인다.
TrafficController의 상태를 쓰므로 혼자 쓰지 않는다.
"""
from __future__ import annotations

from fleet_core.traffic.mission import (
    _COMM_REASON,
    BLOCKED,
    MOVE,
    NO_FAULT,
    RECOVERING,
    TO_OUTBOUND,
    _Mission,
)

OFF_LANE = "OFF_LANE"


class TrafficOffLane:
    # ---- 보고마다 ----------------------------------------------------------------------

    def _watch_lane_error(self, m: _Mission) -> None:
        """지금 달리는 레인에서 이탈 e를 잰다. 레인이 바뀔 때만 횟수를 0으로 한다.

        경로 첫 지점으로 가는 목표와 도킹은 기준 레인이 없어 보지 않는다.
        """
        if m.off_lane_since is not None:
            self._count_back_on_lane(m)
            return
        lane = self._error_lane(m)
        if lane is None or m.fault:
            return
        if lane != m.lane_ref:
            m.lane_ref, m.off_lane, m.off_lane_warned = lane, 0, False
        s, e = self.site.lane_progress(*lane, m.x, m.y)
        m.off_lane = m.off_lane + 1 if e > self.params.lane_error_warn_m else 0
        if m.off_lane >= self.params.release_confirm_samples and not m.off_lane_warned:
            m.off_lane_warned = True
            self._event("WARN", "LANE_ERROR",
                        f"{e:.2f} m off {lane[0]} to {lane[1]} at s={s:.2f}, stopping", m.name)
            self._stop_off_lane(m, lane)

    def _error_lane(self, m: _Mission) -> tuple | None:
        """이탈을 재는 레인: 목표 안에서 시작점을 지난 마지막 레인.

        blocking.py의 _running_lane(가까운 쪽)과 다르다. 직선 목표에서 지점을 지나면 바로 바뀐다.
        """
        goal = m.goal
        if not (goal and goal.kind == MOVE and goal.target_idx > 0):
            return None
        first = max(min(m.idx, goal.target_idx) - 1, 0)
        found = first
        for k in range(first, goal.target_idx):
            s, _ = self.site.lane_progress(m.path[k], m.path[k + 1], m.x, m.y)
            if s >= 0:
                found = k
        return m.path[found], m.path[found + 1]

    def _count_back_on_lane(self, m: _Mission) -> None:
        s, e = self.site.lane_progress(*m.off_lane_at, m.x, m.y)
        half = self.site.half_width
        span = self.site.lane(*m.off_lane_at).span
        on = e <= half and -half <= s <= span + half  # 직선이 아니라 구간 위
        m.back_on_lane = m.back_on_lane + 1 if on else 0
        if not on:
            m.back_on_lane_at = None
        elif m.back_on_lane == self.params.release_confirm_samples:
            m.back_on_lane_at = self._clock()

    def _stop_off_lane(self, m: _Mission, lane: tuple) -> None:
        """목표를 취소하고 구역은 쥔다. 기다리던 요청은 뒤 로봇을 막지 않게 거둔다."""
        if m.goal:
            self._cancel_goal(m)  # settle_idx가 남아 돌아오면 같은 목표를 보낸다
        if m.block and not m.block.marked:
            # 이탈이 먼저 세운다. 표시 전 막힘은 닫고, 돌아와 다시 BLOCKED면 이어 센다(rejoin)
            self._clear_block(m, "")
        m.off_lane_since = self._clock()
        m.off_lane_at = lane
        m.back_on_lane = 0
        m.back_on_lane_at = None
        m.stall_from = None
        m.wait_zone = None
        self.zones.cancel_all_requests(m.name)

    # ---- 틱마다 ------------------------------------------------------------------------

    def _watch_off_lane(self, m: _Mission, now: float) -> None:
        """돌아왔으면 재개, off_lane_timeout_s 안에 못 오면 ERROR(OFF_LANE)."""
        if m.off_lane_since is None:
            return
        if m.fault:  # 다른 고장이나 운영자 해제가 덮었다
            self._end_off_lane(m)
            return
        lane = m.off_lane_at
        deadline = m.off_lane_since + self.params.off_lane_timeout_s
        back_in_time = m.back_on_lane_at is not None and m.back_on_lane_at < deadline
        if now >= deadline and not back_in_time:
            # 마감 전에 돌아왔으면 통신 복구를 기다려도 시간 초과가 아니다.
            # 마감 뒤에 찬 복귀보다는 시간 초과가 먼저다.
            self._end_off_lane(m)
            self._fail(m, OFF_LANE)
            self._event("ERROR", OFF_LANE,
                        f"not back on {lane[0]} to {lane[1]} in "
                        f"{self.params.off_lane_timeout_s:.0f} s", m.name)
            return
        if m.back_on_lane >= self.params.release_confirm_samples:
            if m.comm:
                return  # 통신 복구가 끝난 뒤 재개한다
            self._end_off_lane(m)
            self._event("INFO", "BACK_ON_LANE", f"back on {lane[0]} to {lane[1]}, resuming",
                        m.name)
            if m.phase == TO_OUTBOUND and m.dispatch is None:
                self._start_park_leg(m)  # 서 있는 동안 배차가 취소됐다

    def _end_off_lane(self, m: _Mission) -> None:
        m.off_lane_since = None
        m.off_lane_at = None
        m.back_on_lane, m.back_on_lane_at = 0, None
        m.off_lane, m.off_lane_warned, m.lane_ref = 0, False, None
        m.outside.clear()  # 서 있던 동안의 보고로 놓지 않는다

    def _still_off_lane(self, m: _Mission) -> bool:
        """레인에 아직 못 돌아왔다. 이 동안은 자세로 아무것도 판단하지 않는다."""
        return (m.off_lane_since is not None
                and m.back_on_lane < self.params.release_confirm_samples)

    # ---- 보이는 값 ---------------------------------------------------------------------

    def _shown_fault(self, m: _Mission) -> tuple[str, str]:
        """/fleet/robots의 fault와 이유. 고장, 통신, 이탈, 막힘 순이다."""
        if m.fault:
            return m.fault, m.fault_reason
        if m.comm:
            return m.comm, _COMM_REASON.get(m.comm, "")
        if m.off_lane_since is not None:
            return RECOVERING, OFF_LANE
        return NO_FAULT, BLOCKED if m.block else ""
