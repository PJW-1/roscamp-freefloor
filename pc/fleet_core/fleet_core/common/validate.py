"""FMS 밖에서 온 입력 검사. 잘못된 이유를 돌려주고, 정상이면 "".

FleetManager가 코어보다 먼저 부르므로 재생도 같은 입력을 거부한다. 무엇을 받아도 예외를 내지 않는다.
"""
from __future__ import annotations

import math
from numbers import Real

RUN_MODES = ("RUN", "PAUSE", "STOP")


def shown(value) -> str:
    """이유 문장에 넣을 값. repr이 실패해도 예외를 내지 않는다."""
    try:
        return repr(value)
    except Exception:
        return f"<{type(value).__name__}>"


def text(value) -> str:
    """문구, 출처 키, 저장에 쓸 글자. str 하위 클래스는 내용만 남긴 str로 바꾼다."""
    if isinstance(value, str):
        return str.__str__(value)
    return shown(value)


def checked(check, *args) -> str:
    """검사기를 부른다. 비교·해시·repr이 실패하는 값으로 예외가 나면 그것을 거부 이유로 돌려준다."""
    try:
        return check(*args)
    except Exception as error:
        return f"the input could not be checked ({type(error).__name__})"


def _number(value) -> bool:
    return isinstance(value, Real) and not isinstance(value, bool)


def pose_problem(x, y, yaw, bounds, margin) -> str:
    """bounds는 웨이포인트의 (min_x, min_y, max_x, max_y)."""
    for label, value in (("x", x), ("y", y), ("theta", yaw)):
        if not _number(value) or not math.isfinite(value):
            return f"pose {label}={shown(value)} is not a finite number"
    min_x, min_y, max_x, max_y = bounds
    if not (min_x - margin <= x <= max_x + margin and min_y - margin <= y <= max_y + margin):
        return (f"pose ({x:.2f}, {y:.2f}) is more than {margin} m outside the map "
                f"x {min_x}..{max_x}, y {min_y}..{max_y}")
    return ""


def robot_problem(robot, robots) -> str:
    if not isinstance(robot, str) or robot not in robots:
        return f"no such robot {shown(robot)}"
    return ""


def reported_id_problem(robot, reported_id) -> str:
    """robot_id는 비워도 된다. 적었다면 토픽의 로봇과 같아야 한다."""
    if reported_id in (None, "") or reported_id == robot:
        return ""
    return f"robot_id {shown(reported_id)} on the topic of {robot}"


def run_mode_problem(run_mode) -> str:
    if not isinstance(run_mode, str) or run_mode not in RUN_MODES:
        return f"run_mode {shown(run_mode)} is not one of {list(RUN_MODES)}"
    return ""


def clear_problem(robot, zone_ids, robots, zone_exists) -> str:
    """confirm_clear 인자의 로봇과 구역이 있는지만 본다. 해제 가능 여부는 보지 않는다."""
    problem = robot_problem(robot, robots)
    if problem:
        return problem
    if not isinstance(zone_ids, (list, tuple)):
        return f"zone_ids is a {type(zone_ids).__name__}, not a list"
    unknown = [z for z in zone_ids if not isinstance(z, str) or not zone_exists(z)]
    if unknown:
        return "no such zone " + ", ".join(shown(z) for z in unknown)
    return ""
