"""wait_ready의 판정. ROS 없이 돈다.

gz.launch.py는 모든 핑키가 IDLE을 보고해야 관제를 띄운다. 마지막으로 받은 값만 보면 IDLE을 한 번
보내고 끊긴 로봇도 준비된 것으로 치므로, 받은 시각까지 본다.
"""
from __future__ import annotations

IDLE = "IDLE"  # RobotState.STATE_IDLE


def everyone_idle(last: dict, robots, now: float, fresh_s: float) -> bool:
    """last: 로봇마다 (상태, 받은 시각). 모든 로봇의 마지막 보고가 IDLE이고 fresh_s 안이면 참."""
    return all(robot in last and last[robot][0] == IDLE and now - last[robot][1] <= fresh_s
               for robot in robots)
