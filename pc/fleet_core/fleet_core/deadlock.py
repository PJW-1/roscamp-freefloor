"""Deadlock safety net: who waits for whom, wait cycles, and who gives way."""
from __future__ import annotations


def wait_graph(zone_manager) -> dict[str, set[str]]:
    """Return waiter -> holders for every queued request on a held zone."""
    graph: dict[str, set[str]] = {}
    for entry in zone_manager.snapshot().values():
        holder = entry["holder"]
        if holder is None:
            continue
        for waiter in entry["queue"]:
            if waiter != holder:
                graph.setdefault(waiter, set()).add(holder)
    return graph


def find_cycle(graph: dict[str, set[str]]) -> list[str] | None:
    """Return the robots of one wait cycle, or None. Deterministic for a given graph."""
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
    """Higher means more work would be lost by giving way."""
    score = 0
    if at_station:
        score += params.priority_at_station
    if carrying:
        score += params.priority_has_cargo
    return score


def choose_retreat(cycle: list[str], priority_fn) -> str:
    """Lowest priority gives way; the name breaks ties so a run can be replayed."""
    return min(cycle, key=lambda robot: (priority_fn(robot), robot))
