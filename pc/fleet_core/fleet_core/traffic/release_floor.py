"""해제 규칙 보강(파트 7): 예각 합류.

경로 위 해제는 나가는 레인을 따라 s > d면 놓았다. 그 근거 d > 2r은 구역의 레인들이 90도 넘게
벌어질 때만 맞다. 예각 θ로 합류하는 레인이 있으면 s만큼 가도 그 레인과 s·sinθ만 떨어진다
(특징 지도 T1: 53.1도, s = 0.2에서 0.16). 그래서 구역의 다른 바닥 선에서도 중심선 거리 d
이상이어야 놓는다. 경로 밖 해제(_off_the_floor)와 같은 기준이다.
시작 때는 구역을 떠나는 레인마다 끝 지점이 그 기준을 넘는지 본다. 로봇이 지점에 선다고
가정한다. arrival_tolerance_m만큼 못 미쳐 서면 안 놓일 수 있다(사용자 결정 대기).

TrafficController에서 떼어 낸 부분이다. 제어기 상태를 쓰므로 혼자 쓰지 않는다.
"""
from __future__ import annotations

from fleet_core.traffic.mission import _Mission


class TrafficReleaseFloor:
    def _check_exit_floor(self) -> None:
        if not self.params.release_clear_of_floor:
            return
        d = self.params.release_distance_m
        bad = [f"{a} to {b} ends {gap:.3f} m" for a, b, gap in self.site.exit_floor_gaps()
               if gap < d]
        if bad:
            raise ValueError(
                f"lanes leaving a zone must end {d} m (release distance) clear of the rest of "
                f"that zone, or the zone is never released: {', '.join(bad)}")

    def _clear_of_floor(self, m: _Mission, zone: str, lane: tuple) -> bool:
        if not self.params.release_clear_of_floor:
            return True
        gap = self.site.floor_gap(zone, lane, m.x, m.y)
        return gap >= self.params.release_distance_m
