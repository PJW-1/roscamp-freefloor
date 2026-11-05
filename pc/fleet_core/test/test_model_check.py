"""Exhaustive check of the one-way lanes and zone locks for three robots.

Ported from zone_model_check.py, but the graph comes from site_map.yaml so the map the
fleet manager uses is the map being proved. A move has two phases, as in the real protocol:
acquire (a robot in zone A reserves zone B and keeps A) and complete (it left A, A is freed).

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
import yaml
from fleet_core.site_map import load_site_map

ROBOTS = 3

# Sanity control: the return drives back along the outbound lane, so two robots can meet
# head-on. The checker must report deadlocks here or "none" for the design means nothing.
HEAD_ON_ROUTE = "to_work"
CONTROL = (33480, 66, 8304)  # measured


def route_edges(site, skip=()):
    succ = defaultdict(set)
    for name, route in site.routes.items():
        if name in skip:
            continue
        for a, b in zip(route.cells, route.cells[1:], strict=False):
            succ[site.node(a, route.flow)].add(site.node(b, route.flow))
    return succ


def add_links(site, succ):
    for link in yaml.safe_load(site.source_text)["links"]:
        a, b = tuple(link["from"]), tuple(link["to"])
        succ[site.node(a, link["from_flow"])].add(site.node(b, link["to_flow"]))
    return succ


def head_on_graph(site):
    succ = route_edges(site, skip=(HEAD_ON_ROUTE,))
    back = site.routes["to_outbound"].cells[::-1]
    for a, b in zip(back, back[1:], strict=False):
        succ[site.node(a, "back")].add(site.node(b, "back"))
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
    loop = set(site.routes["to_outbound"].cells) | set(site.routes["to_work"].cells)
    rows = defaultdict(list)
    for cell in sorted(site.zone_of):
        rows[not loop & site.zones[site.zone_of[cell]]].append(cell)
    return rows


@pytest.fixture(scope="module")
def site():
    return load_site_map()


def test_design_has_no_deadlock_and_no_starvation(site):
    assert check(site, add_links(site, route_edges(site))) == (149238, 0, 0)


def test_head_on_control_finds_deadlocks(site):
    states, deadlocks, starved = check(site, head_on_graph(site))
    assert deadlocks > 0, "a checker that finds nothing here proves nothing about the design"
    assert (states, deadlocks, starved) == CONTROL


def test_frozen_robot_table(site):
    rows = frozen_robot_table(site)
    assert set(rows[True]) == set(site.parking), "only a waiting cell leaves the loop free"
    assert len(rows[True]) == 6
    assert len(rows[False]) == len(site.zone_of) - 6
