"""레인 막힘 1단계: 서고 알리기(지시서 4번, docs/핑키_계약.md 4절).

로봇이 이동 중 BLOCKED로 보고하면 목표는 그대로 두고 막힌 시간을 잰다.
  - blocked_mark_s: 달리던 레인을 막힘으로 표시하고(LANE_BLOCKED) 목표를 취소한다.
    몸이 있는 레인의 구역은 쥐고, 들어가지 않은 앞 구역은 취소 결과가 오고 선 것을 본 뒤 놓는다.
  - 풀림: 통신이 정상이고 BLOCKED가 아닌 보고가 연속 release_confirm_samples번.
    표시했으면 LANE_CLEARED를 내고 같은 목표를 다시 보낸다. 시도로 세지 않는다.
    두절과 복구 확인 중에는 풀지도 놓지도 않고, 연속 횟수는 처음부터 다시 센다.
  - blocked_timeout_s: ERROR(BLOCKED). 시간은 BLOCKED로 보고된 동안만 센다.
    풀린 뒤 blocked_rejoin_s 안에 같은 레인에서 다시 막히면 이어 센다.
줄 서기(같은 레인이나 다음 레인 앞 queue_gap_m 안의 정상 로봇)는 막힘으로 치지 않는다.
막힘을 연 뒤에 줄 서기가 되면 그동안 시계를 멈춘다.
줄 서기가 blocked_timeout_s를 넘으면 막힘으로 친다. BLOCKED가 아닌 보고나 두절이 끼면 다시 잰다.
표시 동안과 취소한 goal의 결과 전에는 다른 자동 해제가 쉬고, 여기서만 놓는다.
결과 전에는 단계도 진행하지 않는다(controller._step_phase).
heartbeat가 끊겨 로봇이 스스로 끝낸 이동(HEARTBEAT_LOST)도 여기서 다룬다.

TrafficController에서 떼어 낸 부분이다. 제어기 상태를 쓰므로 혼자 쓰지 않는다.
"""
from __future__ import annotations

from fleet_core.traffic.lane_map import _point_segment
from fleet_core.traffic.mission import (
    BLOCKED,
    HEARTBEAT_LOST,
    LOCALIZING,
    MOVE,
    ON_A_GOAL,
    _Block,
    _Goal,
    _Mission,
)


class TrafficBlocking:
    def blocked_lanes(self) -> list[tuple]:
        """표시한 막힘 (로봇, a, b). 파트 5 우회가 피할 레인이다."""
        return [(m.name, *m.block.lane) for m in self._missions.values()
                if m.block and m.block.marked]

    # ---- 보고마다 ------------------------------------------------------------------------

    def _note_block_report(self, m: _Mission) -> None:
        b = m.block
        if b is None:
            if m.robot_state == BLOCKED:
                self._open_block(m)
            return
        if m.comm:
            # 두절과 복구 확인 중에는 풀지도 놓지도 않는다. 그동안의 BLOCKED를 못 봤을 수 있다
            b.clear_n = b.still_n = 0
            return
        if b.marked and m.block_cancel is None:
            b.still_n = 0 if m.robot_state in ON_A_GOAL else b.still_n + 1
        b.clear_n = 0 if m.robot_state == BLOCKED else b.clear_n + 1
        if b.clear_n >= self.params.release_confirm_samples:
            self._clear_block(m, "")

    def _open_block(self, m: _Mission) -> None:
        """이동 중 BLOCKED. 목표 없이 섰거나 줄 서기면 상태만 보인다."""
        if m.comm or m.fault or not (m.goal and m.goal.kind == MOVE):
            return
        lane = self._running_lane(m)
        if lane is None or self._queue_rests(m, lane):
            return
        now = self._clock()
        total = 0.0
        if m.block_rejoin:
            last_lane, cleared_at, last_total = m.block_rejoin
            if last_lane == lane and now - cleared_at <= self.params.blocked_rejoin_s:
                total = last_total
        m.block_rejoin = None
        m.block = _Block(lane, total, now)

    def _clear_block(self, m: _Mission, how: str) -> None:
        b, m.block = m.block, None
        m.block_rejoin = (b.lane, self._clock(), b.total)
        m.stall_from = None  # 멈춤 시계는 0부터 다시 잰다
        if b.marked:
            a, z = b.lane
            self._event("INFO", "LANE_CLEARED", f"{a} to {z} {how or 'is clear'}", m.name)

    def _held_for_block(self, m: _Mission) -> bool:
        """막힘 표시 중이거나 표시 때 취소한 goal의 결과를 기다린다. 단계도 새 호출도 없다."""
        return bool(m.block and m.block.marked) or m.block_cancel is not None

    def _cancel_finished(self, m: _Mission, goal_id) -> None:
        """표시할 때 취소한 goal이 끝났다(결과, 거절, 미전송). 막힘이 먼저 풀려도 같다."""
        if m.block_cancel is not None and m.block_cancel == goal_id:
            m.block_cancel = None

    def _heartbeat_stopped(self, m: _Mission, goal: _Goal) -> None:
        """heartbeat가 끊겨 로봇이 스스로 섰다. 실패로 세지 않고 잠시 뒤 같은 목표를 보낸다.

        진전 없이 되풀이되면 멈춤 감시가 잡도록, 다시 보낼 때까지 멈춤 시계를 잇는다.
        """
        m.settle_idx = goal.target_idx
        m.retry_at = self._clock() + self.params.move_retry_wait_s
        m.stall_hold = True
        self._event("WARN", HEARTBEAT_LOST, "stopped without the fleet heartbeat, sending again",
                    m.name)

    # ---- tick마다 ------------------------------------------------------------------------

    def _watch_block(self, m: _Mission, now: float) -> None:
        """막힌 시간을 잰다. 줄 서기 동안은 쉬고, 고장 난 로봇은 시각만 따라간다."""
        b = m.block
        if (m.live and not m.comm and m.robot_state == BLOCKED
                and not self._queue_rests(m, b.lane)):
            b.total += now - b.seen
        b.seen = now
        if not m.live:
            return
        if not b.marked and b.total >= self.params.blocked_mark_s:
            self._mark_block(m)
        if b.marked and m.block_cancel is None and b.still_n >= self.params.release_confirm_samples:
            self._free_unentered(m)
        if b.total >= self.params.blocked_timeout_s:
            self._fail(m, BLOCKED)
            self._event("ERROR", "BLOCKED_TOO_LONG",
                        f"{b.lane[0]} to {b.lane[1]} blocked for {b.total:.0f} s", m.name)

    def _mark_block(self, m: _Mission) -> None:
        """레인을 막힘으로 표시하고 목표를 취소한다. 쥔 구역은 아직 그대로 둔다."""
        b = m.block
        b.marked = True
        self.zones.cancel_all_requests(m.name)
        m.wait_zone = None
        if m.goal:
            m.block_cancel = m.goal.id
            self._cancel_goal(m)
        a, z = b.lane
        self._event("WARN", "LANE_BLOCKED", f"{a} to {z} blocked for {b.total:.0f} s", m.name)

    def _free_unentered(self, m: _Mission) -> None:
        """취소가 끝나고 선 로봇이 들어가지 않은 앞 구역을 놓는다. 몸이 있는 레인의 구역은 쥔다.

        표시 동안 구역을 놓는 곳은 여기뿐이다(_release_if_cleared는 쉰다).
        """
        body = {self._zone(waypoint) for waypoint in m.block.lane}
        for zone in self.zones.held_by(m.name):
            if (zone not in body and self._off_the_floor(m, zone)
                    and m.outside.get(zone, 0) >= self.params.release_confirm_samples):
                self.zones.release(m.name, zone)
                m.outside.pop(zone, None)

    # ---- 레인과 줄 서기 ---------------------------------------------------------------------

    def _running_lane(self, m: _Mission) -> tuple | None:
        """목표를 따라 지금 달리는 레인 (a, b). 목표가 없거나 못 찾으면 None.

        idx는 그 지점을 d 넘게 지나야 넘어가므로 로봇은 들어오는 레인이나 나가는 레인 앞쪽에 있다.
        더 가까운 레인이고, 같으면 나가는 레인이다(지점에 서서 막히면 앞이 막힌 것이다).
        """
        if not (m.goal and m.goal.kind == MOVE) or not m.path:
            return None
        here = m.path[m.idx]
        lanes = []
        if m.idx < m.goal.target_idx:
            lanes.append((here, m.path[m.idx + 1]))
        if m.idx > 0:
            lanes.append((m.path[m.idx - 1], here))
        else:
            # 복구나 처음 올린 뒤에는 경로에 들어온 레인이 없어 지도에서 찾는다.
            lanes += [key for key in self.site.lanes if key[1] == here
                      and self._gap_to(m, key) <= self.params.lane_error_warn_m]
        return min(lanes, key=lambda key: self._gap_to(m, key), default=None)

    def _gap_to(self, m: _Mission, key: tuple) -> float:
        lane = self.site.lane(*key)
        return _point_segment(m.x, m.y, lane.ax, lane.ay, lane.bx, lane.by)

    def _lane_after(self, m: _Mission, lane: tuple) -> tuple | None:
        """경로에서 lane 다음 레인. 경로가 바뀌어 lane이 지금 자리에 없으면 None."""
        i, path = m.idx, m.path
        if i < len(path) and path[i] == lane[1]:
            end = i  # 들어오는 레인
        elif i + 1 < len(path) and (path[i], path[i + 1]) == lane:
            end = i + 1  # 나가는 레인
        else:
            return None
        return (path[end], path[end + 1]) if end + 1 < len(path) else None

    def _queue_rests(self, m: _Mission, lane: tuple | None = None) -> bool:
        """BLOCKED인데 줄 서기라 막힘과 멈춤 감시를 쉰다.

        blocked_timeout_s 넘게 이어지면 쉬지 않는다. 서로 막고 서 있을 수 있어서다.
        """
        if m.robot_state != BLOCKED or not self._queued_behind(m, lane):
            m.queue_since = None
            return False
        now = self._clock()
        if m.queue_since is None:
            m.queue_since = now
        return now - m.queue_since < self.params.blocked_timeout_s

    def _queued_behind(self, m: _Mission, lane: tuple | None = None) -> bool:
        """같은 레인이나 다음 레인 앞 queue_gap_m 안에 정상인 다른 로봇이 있다.

        막혔거나 위치를 잡는 중인 로봇은 곧 움직이지 않으니 줄로 보지 않는다.
        """
        lane = lane or self._running_lane(m)
        if lane is None:
            return False
        s_here, _ = self.site.lane_progress(*lane, m.x, m.y)
        ahead = [(lane, -s_here)]
        following = self._lane_after(m, lane)
        if following:
            ahead.append((following, self.site.lane(*lane).span - s_here))
        for other in self._missions.values():
            if (other is m or not other.available or other.block
                    or other.robot_state in (BLOCKED, LOCALIZING)):
                continue
            for key, offset in ahead:
                s, e = self.site.lane_progress(*key, other.x, other.y)
                if (e <= self.params.lane_error_warn_m and 0.0 <= s <= self.site.lane(*key).span
                        and 0.0 < offset + s <= self.params.queue_gap_m):
                    return True
        return False
