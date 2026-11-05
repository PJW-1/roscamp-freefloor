"""Exhaustive check of the cell layout's one-way lanes and zone locks for three robots.

Ported from zone_model_check.py. It works on the lock graph alone, every placement of it, and
tools/model_check.py is what drives the real core. A move has two phases, as in the real
protocol: acquire (a robot in zone A reserves zone B and keeps A) and complete (it left A, A is
freed).

What this shows and what it does not:
  - It explores all placements, not only those a real assignment policy reaches.
  - "No deadlock" means some robot can always move; "no starvation" means a robot can
    always move again under some ordering. Fairness comes from the FIFO queue in zones.py.
  - A move in progress is assumed to finish. A robot stopping mid-move is only covered by
    the frozen robot table, which checks route cells against the zone the robot holds.
  - Geometry, Nav2 paths and timing are not modelled.
"""
import itertools
from collections import defaultdict

import pytest
from grid_map import grid_site

ROBOTS = 3
WORK, OUTBOUND = (5, 3), (9, 3)

# Sanity control: the return drives back along the outbound lane, so two robots can meet
# head-on. The checker must report deadlocks here or "none" for the design means nothing.
CONTROL = (33480, 66, 8304)  # measured


def design_graph(site):
    """A node is (cell, lane name). A cell here has one lane through it, so the name is empty."""
    succ = defaultdict(set)
    for a, b in site.lanes:
        succ[(a, "")].add((b, ""))
    return succ


def head_on_graph(site):
    """The outbound lane driven in both directions, as separate tracks over the same cells."""
    out = site.plan(WORK, OUTBOUND)
    succ = defaultdict(set)

    def track(cells, name):
        for a, b in zip(cells, cells[1:], strict=False):
            succ[(a, "" if a in site.docks else name)].add((b, "" if b in site.docks else name))

    track(out, "out")
    track(out[::-1], "back")
    track(site.parking, "wait")
    return succ


def check(site, succ, robots=ROBOTS):
    """Return (states, deadlocks, starved robot states) over every placement."""
    zone_of = site.zone_of
    nodes = set(succ) | {v for vs in succ.values() for v in vs}
    single = [(n,) for n in sorted(nodes)]
    single += [(u, v) for u in sorted(succ) for v in sorted(succ[u])]
    held = {s: frozenset(zone_of[n[0]] for n in s) for s in single}

    states = [
        combo for combo in itertools.product(single, repeat=robots)
        if all(held[a].isdisjoint(held[b]) for a, b in itertools.combinations(combo, 2))
    ]

    forward = {}
    deadlocks = 0
    for state in states:
        per_robot = []
        for i, me in enumerate(state):
            if len(me) == 2:  # in transit: leaving the old cell needs no permission
                per_robot.append([state[:i] + ((me[1],),) + state[i + 1:]])
                continue
            busy = set().union(*(held[s] for j, s in enumerate(state) if j != i))
            per_robot.append([
                state[:i] + ((me[0], v),) + state[i + 1:]
                for v in succ.get(me[0], ()) if zone_of[v[0]] not in busy
            ])
        forward[state] = per_robot
        if not any(per_robot):
            deadlocks += 1

    # Robot i is starved in a state if no sequence of moves ever lets i move again.
    reverse = defaultdict(list)
    for state, per_robot in forward.items():
        for targets in per_robot:
            for target in targets:
                reverse[target].append(state)
    starved = 0
    for i in range(robots):
        seen = {s for s in states if forward[s][i]}
        stack = list(seen)
        while stack:
            for prev in reverse[stack.pop()]:
                if prev not in seen:
                    seen.add(prev)
                    stack.append(prev)
        starved += len(states) - len(seen)
    return len(states), deadlocks, starved


def frozen_robot_table(site):
    """For each stop cell, does the WORK-OUTBOUND loop stay free of blocked route cells?"""
    loop = set(site.plan(WORK, OUTBOUND)) | set(site.plan(OUTBOUND, WORK))
    rows = defaultdict(list)
    for cell in sorted(site.zone_of):
        rows[not loop & site.zones[site.zone_of[cell]]].append(cell)
    return rows


@pytest.fixture(scope="module")
def site():
    return grid_site()


def test_design_has_no_deadlock_and_no_starvation(site):
    """The same number as before the map became lanes, so the conversion kept the graph."""
    assert check(site, design_graph(site)) == (149238, 0, 0)


def test_head_on_control_finds_deadlocks(site):
    states, deadlocks, starved = check(site, head_on_graph(site))
    assert deadlocks > 0, "a checker that finds nothing here proves nothing about the design"
    assert (states, deadlocks, starved) == CONTROL


def test_frozen_robot_table(site):
    rows = frozen_robot_table(site)
    assert set(rows[True]) == set(site.parking), "only a waiting cell leaves the loop free"
    assert len(rows[True]) == 6
    assert len(rows[False]) == len(site.zone_of) - 6
