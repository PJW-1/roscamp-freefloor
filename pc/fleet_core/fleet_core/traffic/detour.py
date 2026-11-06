"""우회 규칙(지시서 5번). 다음 구역을 청할 때만 길을 다시 고른다.

비용 = 레인 길이 + 들어가는 구역의 벌점.
  - 막힌 레인(blocked_lanes, 양방향 쌍은 반대쪽도)은 못 지난다.
  - 지금 경로에 없던 대기 칸으로는 못 간다. 대기 줄은 앞지르지 않는 줄이라 지나는 길로 안 쓴다.
  - 서 있는 로봇이 쥔 구역(충돌 쌍 포함)은 예상 대기 시간 × detour_speed_mps. 무한이면 못 지난다.
  - 달리는 로봇이 쥔 구역은 벌점이 없다.
서 있음과 달림(_standing_wait_s):
  - 무한: 고장(ERROR, OUT_OF_SERVICE), 두절이나 복구 확인, 막힘 기록, BLOCKED 보고.
  - 달린다: 열린 이동이나 도킹 goal이 있다.
  - 구역을 기다린다: 그 구역을 기다린 시간.
  - 재시도나 도착 확인을 기다린다: 남은 시간.
  - 달린다: 주행 단계(TO_*, DOCKING)인데 위 대기가 아니다. 목표 사이라 이 tick에 다음 목표를 받는다.
  - 그 밖(스테이션, 대기 칸, 할 일 대기): station_wait_estimate_s.
고르는 때: 현재 구역 말고 쥔 것이 없어 다음 구역을 청할 때. 다음 구역을 이미 쥐었으면 안 바꾼다.
다음 구역에 벌점이 있거나 남은 경로에 막힌 레인이 있을 때만 계산한다.
새 길이 지금 길보다 싸야 바꾼다. 동점이면 그대로, 길이 없으면 그대로 기다린다.
한 지점에서 포기한 다음 구역은 그 지점을 떠날 때까지 못 지난다(왕복 금지).
입력과 시계만 쓰므로 기록 재생에서 같은 결정이 나온다.

TrafficController에서 떼어 낸 부분이다. 제어기 상태를 쓰므로 혼자 쓰지 않는다.
"""
from __future__ import annotations

import math

from fleet_core.traffic.mission import (
    BLOCKED,
    DOCKING,
    TO_OUTBOUND,
    TO_PARK,
    TO_WORK,
    _Mission,
)


class TrafficDetour:
    def _standing_wait_s(self, m: _Mission) -> float | None:
        """달리면 None, 서 있으면 예상 대기 시간(초). 못 비키면 inf."""
        if m.fault or m.comm or m.block or m.robot_state == BLOCKED:
            return math.inf
        if m.goal is not None:
            return None
        now = self._clock()
        zone = self.zones.waiting_for(m.name)
        if zone is not None:
            return max(0.0, now - m.wait_since) if m.wait_zone == zone else 0.0
        later = max(m.retry_at, m.arrival_by or 0.0)
        if later > now:
            return later - now
        if m.phase in (TO_WORK, TO_OUTBOUND, TO_PARK, DOCKING):
            return None  # 목표 사이다. 이 tick에 다음 목표를 받는다
        return self.params.station_wait_estimate_s

    def _zone_penalty(self, m: _Mission, zone: str, cache: dict) -> tuple[float, str]:
        """m이 zone에 들어갈 때의 벌점(미터)과 그 이유."""
        if zone in cache:
            return cache[zone]
        worst, why = 0.0, ""
        for held, holder in sorted(self.zones.holders_around(zone, m.name).items()):
            other = self._missions.get(holder)
            wait = self._standing_wait_s(other) if other else None  # 모르는 보유자는 벌점 없음
            if wait is None:
                continue
            penalty = math.inf if wait == math.inf else wait * self.params.detour_speed_mps
            if penalty > worst:
                worst = penalty
                why = f"{held} held by {holder}, standing {wait:.0f} s"
        cache[zone] = (worst, why)
        return worst, why

    def _shut_lanes(self) -> set:
        shut = set()
        for _, a, b in self.blocked_lanes():
            shut.update({(a, b), (b, a)})  # 양방향 쌍이면 반대쪽도 같은 통로다
        return shut

    def _consider_detour(self, m: _Mission, following: str) -> bool:
        """다음 구역을 청하기 직전. 더 싼 길로 바꿨으면 True."""
        if not self.params.detour_enabled or m.block:
            return False
        anchor = self._anchor(m)
        here = m.path[anchor]
        if m.detour_at != here:
            m.detour_at, m.detour_avoid = here, ()
        cache: dict = {}
        penalty, why = self._zone_penalty(m, following, cache)
        shut = self._shut_lanes()
        tail = m.path[anchor:]
        shut_ahead = [key for key in zip(tail, tail[1:], strict=False) if key in shut]
        if penalty <= 0.0 and not shut_ahead:
            return False
        avoid = set(m.detour_avoid)
        parking = set(self.site.parking) - set(tail)

        def lane_cost(lane) -> float:
            if (lane.a, lane.b) in shut or lane.b in parking:
                return math.inf
            zone = self._zone(lane.b)
            if zone == self._zone(lane.a):
                return lane.length
            if zone in avoid:
                return math.inf
            return lane.length + self._zone_penalty(m, zone, cache)[0]

        now_cost = 0.0
        for a, b in zip(tail, tail[1:], strict=False):
            now_cost += lane_cost(self.site.lane(a, b))
        found = self.site.cheapest(here, tail[-1], lane_cost)
        if found is None or found[0] >= now_cost or found[1] == tail:
            return False
        way = found[1]
        current = self._zone(here)
        new_next = next((self._zone(w) for w in way if self._zone(w) != current), None)
        if new_next != following:
            m.detour_avoid = (*m.detour_avoid, following)
        self._start_leg(m, m.phase, m.dest, way[-1], way=way)
        reasons = [why] if penalty > 0.0 else []
        reasons += [f"lane {a} to {b} blocked" for a, b in shut_ahead]
        route = " ".join(self.site.waypoint_id(w) for w in way)
        self._event("INFO", "DETOUR",
                    f"avoids {'; '.join(reasons)}; new path {route}", m.name)
        return True
