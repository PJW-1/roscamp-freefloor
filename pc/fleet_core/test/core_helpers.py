"""테스트용 FleetCore 보조 함수."""
from fleet_core.common.params import load_params
from grid_map import grid_site
from legacy_core import FleetCore


def make_core(robots, params=None):
    """코어와 가짜 시간(원소 하나짜리 리스트)을 돌려준다."""
    now = [0.0]
    core = FleetCore(grid_site(), params or load_params(), list(robots), lambda: now[0])
    return core, now


def place(core, robot, cell, yaw=0.0, state="IDLE"):
    """셀 중심에 자세를 보고한다. 해제로 인정될 만큼 여러 번 보낸다."""
    x, y = core.site.waypoint_pose(cell)
    for _ in range(core.params.release_confirm_samples):
        core.on_robot_state(robot, x, y, yaw, state)


def cmds(out, name):
    """틱 출력을 클래스 이름으로 거른다."""
    return [c for c in out if type(c).__name__ == name]


def codes(out):
    return [e.code for e in cmds(out, "EventOut")]


def serve(core, robots, until, limit=300):
    """`until()`이 참이 될 때까지 틱한다. 실행 중인 이동과 도킹은 즉시 끝낸다.

    팔 명령은 응답하지 않아 로봇이 거기서 멈춘다. 시간은 흐르지 않는다.
    목표는 미션에서 읽으므로 이 호출 전에 틱이 보낸 목표도 처리된다.
    틱이 낸 출력을 모두 돌려준다.
    """
    seen = []
    for _ in range(limit):
        seen += core.tick()
        for robot in robots:
            mission = core._missions[robot]
            goal = mission.goal
            if goal is not None and goal.kind == "move":
                place(core, robot, mission.path[goal.target_idx])
                core.on_move_result(robot, goal.id, True, "")
            elif goal is not None:
                core.on_dock_result(robot, goal.id, True)
            place(core, robot, mission.path[mission.idx])
        if until():
            return seen
    raise AssertionError(f"not reached: {[core.robot_view(r) for r in robots]}")
