"""레인 폴리라인 목표(지시서 7번, FollowPath).

path 모드: 잠금을 쥔 데까지 레인을 한 목표로 묶는다. 방향이 바뀌어도 잇고, 유턴만 끊는다.
MoveCmd.path에 선을 싣는다. 첫 점은 로봇 위치, 이어서 아직 지나지 않은 지점, 끝은 목표 지점.
point 모드: 같은 방향 레인만 묶고 선을 싣지 않는다. 호환과 비교용이다.
도착 판정과 끝 근처 재전송은 핑키 몫이다(docs/핑키_계약.md).

TrafficController에서 떼어 낸 부분이다. 제어기 상태를 쓰므로 혼자 쓰지 않는다.
"""
from __future__ import annotations

import math

from fleet_core.traffic.mission import _Mission

POINT = "point"
PATH = "path"
GOAL_MODES = (POINT, PATH)


class TrafficPath:
    def _check_goal_mode(self) -> None:
        if self.params.lane_goal_mode not in GOAL_MODES:
            raise ValueError(f"lane_goal_mode must be one of {GOAL_MODES}, "
                             f"not {self.params.lane_goal_mode!r}")

    @property
    def _follows_path(self) -> bool:
        return self.params.lane_goal_mode == PATH

    def _joins(self, m: _Mission, index: int, step) -> bool:
        """index에서 나가는 레인을 목표에 이을 수 있는가.

        point는 같은 방향일 때만. path는 되돌아가는 레인(양방향 레인의 유턴)만 끊는다.
        유턴을 이으면 경로를 바꿀 때마다 목표 끝이 앞으로 밀려 경로가 끝없이 늘어난다.
        """
        if not self._follows_path:
            return self._step_between(m, index) == step
        return index == 0 or m.path[index + 1] != m.path[index - 1]

    def _goal_yaw(self, m: _Mission, target_idx: int, step) -> float:
        """목표 지점에서 볼 방향. path는 마지막 레인 방향이다."""
        if self._follows_path and target_idx > 0:
            step = self._step_into(m, target_idx)
        return math.atan2(step[1], step[0]) if step else m.yaw

    def _lane_polyline(self, m: _Mission, target_idx: int, yaw: float) -> tuple:
        """(x, y, yaw)들. point 모드면 빈 값이다."""
        if not self._follows_path:
            return ()
        points = [(m.x, m.y)]
        ahead = [self.site.waypoint_pose(w)
                 for w in m.path[self._first_ahead(m, target_idx):target_idx + 1]]
        if len(ahead) > 1 and math.dist(points[0], ahead[0]) < 1e-9:
            ahead.pop(0)  # 지점 위에 서 있다
        points += ahead
        out = []
        for here, there in zip(points, points[1:], strict=False):
            gap = (there[0] - here[0], there[1] - here[1])
            out.append((*here, math.atan2(gap[1], gap[0]) if any(gap) else yaw))
        out.append((*points[-1], yaw))
        return tuple(out)

    def _first_ahead(self, m: _Mission, target_idx: int) -> int:
        """선에 넣을 첫 지점. 지났으면 뺀다.

        지났다: 나가는 레인을 따라 arrival_tolerance_m 넘게 갔고, 들어오는 레인보다 나가는 레인에
        가깝다. 그보다 가까우면 넣어 위치 잡음으로 모서리를 가로지르지 않게 한다. 90도 넘게
        꺾이는 곳에서는 다가오는 로봇도 나가는 레인 방향으로 앞서 보이므로 거리도 본다.
        로봇 뒤에 남은 짧은 앞부분은 핑키가 건너뛴다(FollowPath는 가장 가까운 점부터 따른다).
        """
        i = min(m.idx, target_idx)
        if i >= target_idx:
            return i
        out = (m.path[i], m.path[i + 1])
        s, _ = self.site.lane_progress(*out, m.x, m.y)
        if s <= self.params.arrival_tolerance_m:
            return i
        if i > 0 and self._gap_to(m, (m.path[i - 1], m.path[i])) < self._gap_to(m, out):
            return i
        return i + 1

    def _error_lane(self, m: _Mission) -> tuple:
        """이탈을 재는 레인. point는 목표 직전 레인, path는 지금 달리는 레인이다."""
        last = (m.path[m.goal.target_idx - 1], m.path[m.goal.target_idx])
        if not self._follows_path:
            return last
        return self._running_lane(m) or last
