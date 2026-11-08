"""Back-up primitive. ROS를 모른다.

얼마나 물러날지(경로 시작과 레인 옆 거리로 제한), 무엇을 시킬지(BackUp, CancelBackUp),
결과가 무엇을 뜻하는지(reason_of)를 한곳에 둔다. 노드는 BackUp을 Nav2 behavior_server의
backup 액션으로 옮긴다(backup_ros.BackUpRunner).

쓰는 곳
- 지금: core의 자가 복구(핑키_계약 3절 7번, 2026-11-09 제안).
- 나중: FMS 개선 계획 버전 2의 1-A 물러나기. 계약의 `Retreat` 액션(goal distance_m,
  speed_mps / result success, reason)은 같은 BackUp 명령과 BackUpRunner를 쓰고, 결과의
  reason은 reason_of로 만든다. Retreat goal을 받기 전에 FollowPath를 먼저 취소하는 것도 같다
  (둘 다 cmd_vel을 낸다).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

# 버전 2 Retreat 결과의 reason. FAILED는 문서에 없는 값이다(서버 없음, TF 오류, 잘못된 입력. 제안)
OBSTACLE_BEHIND = "OBSTACLE_BEHIND"
TIMEOUT = "TIMEOUT"
CANCELED = "CANCELED"
FAILED = "FAILED"

# nav2_msgs/action/BackUp 결과의 error_code (Jazzy, `ros2 interface show`로 2026-11-09 확인)
NAV2_TIMEOUT = 711
NAV2_COLLISION_AHEAD = 714  # 움직이는 쪽, 즉 로봇 뒤에 물체


@dataclass(frozen=True)
class BackUp:
    """뒤로 곧게 distance_m만큼 speed_mps로 간다(Nav2 BackUp). seq로 결과를 짝짓는다."""
    seq: int
    distance_m: float
    speed_mps: float
    time_allowance_s: float


@dataclass(frozen=True)
class CancelBackUp:
    """달리는 BackUp을 모두 멈춘다."""


def backup_distance(travelled_m: float, max_m: float, radius_m: float, margin_m: float) -> float:
    """경로 시작점에서 온 거리로 물러날 거리를 정한다.

    경로의 첫 점은 관제가 마지막으로 본 로봇 위치라서 로봇이 쥔 구역 안이다. 몸 뒤끝이 그
    점을 넘지 않게 반지름과 여유만큼 덜 간다. 0 이하면 물러나지 않는다.
    """
    return max(0.0, min(max_m, travelled_m - radius_m - margin_m))


def keep_lateral(distance_m: float, lateral_m: float, heading_err: float, limit_m: float) -> float:
    """물러난 뒤 레인 옆 거리가 limit_m(이미 넘었으면 지금 값)을 넘지 않게 거리를 줄인다.

    lateral_m은 레인 왼쪽이 +인 옆 거리, heading_err는 로봇 방향 - 레인 방향(rad)이다.
    BackUp은 로봇 방향으로 곧게 물러나므로 d만큼 가면 옆 거리가 d·sin(heading_err)만큼 준다.
    """
    k = math.sin(heading_err)
    if abs(k) < 1e-9 or distance_m <= 0.0:
        return max(0.0, distance_m)
    bound = max(limit_m, abs(lateral_m))
    reach = (lateral_m + bound) / k if k > 0 else (bound - lateral_m) / -k
    return max(0.0, min(distance_m, reach))


def reason_of(outcome: str, error_code: int) -> str:
    """BackUp 결과(outcome: succeeded, aborted, canceled, rejected)를 Retreat reason으로."""
    if outcome == "succeeded":
        return ""
    if outcome == "canceled":
        return CANCELED
    if outcome == "aborted" and error_code == NAV2_COLLISION_AHEAD:
        return OBSTACLE_BEHIND
    if outcome == "aborted" and error_code == NAV2_TIMEOUT:
        return TIMEOUT
    return FAILED
