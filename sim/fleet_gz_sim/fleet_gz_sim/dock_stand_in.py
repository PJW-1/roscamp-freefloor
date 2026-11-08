"""시뮬 전용 도크 대역(sim_dock_stub)의 판정과 켜도 되는지 검사. ROS 없이 돈다.

pinky_dock(스테이션 도크에 ±1~2 cm로 붙기)이 아직 없어서 진짜 pinky_fsm은 도킹을
DOCK_UNAVAILABLE로 실패시킨다. 임무를 끝까지 돌려 보고 싶을 때만 Gazebo 시뮬에서 켠다(기본 꺼짐).
대역은 로봇을 움직이지 않는다. 관제의 move_to가 도크 지점에 데려다 놓은 로봇의
실제 위치를 재고, FakePinky처럼 duration_s 동안 APPROACH → ALIGN → DONE 피드백을 낸 뒤 결과를 낸다.

It may run only on Gazebo time (use_sim_time) in a network that passes isolation.py, so it can
never answer for a real pinky. The node checks this itself (require_dock_stub_allowed).
"""
from __future__ import annotations

import math
from collections.abc import Mapping
from pathlib import Path

from fleet_gz_sim.isolation import problems as isolation_problems

REACH_M = 0.15  # 성공 결과의 위치 기준(R4)과 같다


def dock_phase(elapsed: float, duration: float) -> str:
    done = elapsed / max(duration, 1e-6)
    return "APPROACH" if done < 0.5 else "ALIGN" if done < 1.0 else "DONE"


def dock_outcome(pose, dock_xy, reach_m: float = REACH_M) -> tuple[bool, float, float, float]:
    """(성공, x 오차 cm, y 오차 cm, 방향 오차 deg). 오차는 지도 좌표다.

    레인 지도에는 도크 방향이 없어서 방향 오차는 재지 않는다(0)."""
    ex, ey = pose[0] - dock_xy[0], pose[1] - dock_xy[1]
    return math.hypot(ex, ey) <= reach_m + 1e-9, ex * 100.0, ey * 100.0, 0.0


class ClockWatch:
    """Says when Gazebo time has not advanced for limit_s wall seconds (a stalled /clock keeps
    its last value, so ROS time stays active and positive)."""

    def __init__(self, limit_s: float):
        self.limit_s = limit_s
        self._sim: float | None = None
        self._since = 0.0

    def stalled(self, sim_now: float, wall_now: float) -> bool:
        if self._sim is None or sim_now != self._sim:
            self._sim, self._since = sim_now, wall_now
            return False
        return wall_now - self._since > self.limit_s


def stub_may_answer(ros_time_active: bool, now_s: float) -> bool:
    """Answer a dock goal only while Gazebo time is running (ROS time from /clock, past 0)."""
    return ros_time_active and now_s > 0.0


def dock_stub_problems(use_sim_time: bool, env: Mapping[str, str], *, rmw: str | None = None,
                       workdir: Path | None = None) -> list[str]:
    """Reasons the dock stub must not run. Empty means it may."""
    found = [] if use_sim_time else ["use_sim_time is false: the dock stub runs only on "
                                     "Gazebo time"]
    return found + isolation_problems(env, rmw=rmw, workdir=workdir)


def require_dock_stub_allowed(use_sim_time: bool, env: Mapping[str, str], *,
                              rmw: str | None = None, workdir: Path | None = None) -> None:
    found = dock_stub_problems(use_sim_time, env, rmw=rmw, workdir=workdir)
    if found:
        raise RuntimeError("sim_dock_stub refused:\n  " + "\n  ".join(found))
