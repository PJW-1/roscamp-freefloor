"""교착 안전망: 누가 누구를 기다리는지, 대기 순환, 누가 양보하는지."""
from __future__ import annotations


def wait_graph(zone_manager) -> dict[str, set[str]]:
    """대기자 -> 요청한 구역과의 사이를 막는 로봇들.

    소유자만이 아니다. 충돌 쌍에서는 맨 앞 대기자가 빈 구역에서도 기다릴 수 있어
    그 뒤 대기열을 지나는 순환이 생길 수 있다.
    """
    graph: dict[str, set[str]] = {}
    for zone, entry in zone_manager.snapshot().items():
        for waiter in entry["queue"]:
            blockers = zone_manager.blockers(waiter, zone)
            if blockers:
                graph.setdefault(waiter, set()).update(blockers)
    return graph


def find_cycle(graph: dict[str, set[str]]) -> list[str] | None:
    """대기 순환 하나의 로봇들, 없으면 None. 같은 그래프면 결과도 같다."""
    done: set[str] = set()

    def walk(robot: str, trail: list[str]) -> list[str] | None:
        if robot in trail:
            return trail[trail.index(robot):]
        if robot in done:
            return None
        for holder in sorted(graph.get(robot, ())):
            cycle = walk(holder, [*trail, robot])
            if cycle:
                return cycle
        done.add(robot)
        return None

    for start in sorted(graph):
        cycle = walk(start, [])
        if cycle:
            return cycle
    return None


def priority(params, at_station: bool, carrying: bool) -> int:
    """높을수록 양보할 때 잃는 작업이 많다."""
    score = 0
    if at_station:
        score += params.priority_at_station
    if carrying:
        score += params.priority_has_cargo
    return score


def choose_retreat(cycle: list[str], priority_fn) -> str:
    """우선순위가 가장 낮은 로봇이 양보한다. 재생이 같도록 동점은 이름으로 가른다."""
    return min(cycle, key=lambda robot: (priority_fn(robot), robot))
