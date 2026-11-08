"""시뮬 전용 도크 대역의 판정. ROS 없이 돈다.

pinky_dock(스테이션 도크에 ±1~2 cm로 붙기)이 아직 없어서 Gazebo 3대 시뮬에서만 쓴다. 실물에는
넣지 않는다. 대역은 로봇을 움직이지 않는다. 관제의 move_to가 도크 지점에 데려다 놓은 로봇의
실제 위치를 재고, FakePinky처럼 duration_s 동안 APPROACH → ALIGN → DONE 피드백을 낸 뒤 결과를 낸다.
"""
from __future__ import annotations

import math

REACH_M = 0.15  # 성공 결과의 위치 기준(R4)과 같다


def dock_phase(elapsed: float, duration: float) -> str:
    done = elapsed / max(duration, 1e-6)
    return "APPROACH" if done < 0.5 else "ALIGN" if done < 1.0 else "DONE"


def dock_outcome(pose, dock_xy, reach_m: float = REACH_M) -> tuple[bool, float, float, float]:
    """(성공, x 오차 cm, y 오차 cm, 방향 오차 deg). 오차는 지도 좌표다.

    레인 지도에는 도크 방향이 없어서 방향 오차는 재지 않는다(0)."""
    ex, ey = pose[0] - dock_xy[0], pose[1] - dock_xy[1]
    return math.hypot(ex, ey) <= reach_m + 1e-9, ex * 100.0, ey * 100.0, 0.0
