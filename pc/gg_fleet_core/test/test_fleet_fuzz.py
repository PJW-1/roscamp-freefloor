"""Seeded random fault schedules. The world checks its invariants at every step."""
import random

import pytest
from gg_fleet_core.common.commands import SHIPPED
from gg_fleet_core.traffic.lane_map import load_lane_map
from sim_world import SimWorld

ROBOTS = ("pinky1", "pinky2", "pinky3")
KINDS = ("A", "B", "C", "D")


def random_orders(rng):
    orders = []
    for i in range(rng.randint(1, 8)):
        items = {k: 1 for k in rng.sample(KINDS, rng.randint(1, 3))}
        orders.append({"at_s": 0.0, "id": f"O-{i:03d}", "items": items})
    at = 0.0
    for order in orders[len(orders) // 2:]:
        at += rng.uniform(0.0, 60.0)
        order["at_s"] = round(at, 1)
    return orders


def random_faults(rng, horizon_s=200.0):
    """Faults the fleet recovers from by itself, so every order must still ship."""
    faults = []
    once = set()  # the same fault twice in a row could rightly use up the retry budget
    for _ in range(rng.randint(1, 5)):
        at = round(rng.uniform(1.0, horizon_s), 1)
        robot = rng.choice(ROBOTS)
        kind = rng.choice(["comm_loss", "nav_abort", "dock_fail", "fill_fail", "swap_partial",
                           "pause", "stop"])
        if kind == "comm_loss":
            faults.append({"at_s": at, "robot": robot, "type": kind,
                           "duration_s": round(rng.uniform(4.0, 25.0), 1)})
        elif kind in ("nav_abort", "dock_fail"):
            if (robot, kind) not in once:
                once.add((robot, kind))
                faults.append({"at_s": at, "robot": robot, "type": kind,
                               "count": rng.randint(1, 2)})
        elif kind in ("fill_fail", "swap_partial"):
            if kind not in once:
                once.add(kind)
                faults.append({"at_s": at, "type": kind, "count": 1, "place": rng.randint(0, 1)})
        else:
            mode = kind.upper()
            faults.append({"at_s": at, "type": "run_mode", "mode": mode})
            faults.append({"at_s": at + rng.uniform(1.0, 15.0), "type": "run_mode", "mode": "RUN"})
    return faults


def run_seed(seed):
    rng = random.Random(seed)
    orders = random_orders(rng)
    world = SimWorld(seed=seed, orders=orders, faults=random_faults(rng))
    assert world.run_until(lambda: not world.pending_faults and world.done(), 3000), (
        f"seed {seed}: reports {world.report_statuses()}, "
        f"{[world.core.robot_view(r) for r in ROBOTS]}")
    assert set(world.report_statuses().values()) == {SHIPPED}
    assert world.core.shipped == len(orders)
    assert world.bound_order() == world.registration, "orders bound in registration order"
    assert world.shipped_order() == world.registration, "orders shipped in the order bound"
    assert not world.event_codes("ERROR"), world.events
    return world


@pytest.mark.parametrize("seed", range(40))
def test_random_faults_never_lose_an_order_or_break_an_invariant(seed):
    run_seed(seed)


@pytest.mark.parametrize("seed", range(20, 50))
def test_random_faults_on_the_default_map_with_its_avenue_and_crossing(seed):
    """Same faults on the map where robots meet. The world checks its invariants at every step."""
    rng = random.Random(seed)
    orders = random_orders(rng)
    world = SimWorld(seed=seed, orders=orders, faults=random_faults(rng), site=load_lane_map())
    assert world.run_until(lambda: not world.pending_faults and world.done(), 3000), (
        f"seed {seed}: reports {world.report_statuses()}, "
        f"{[world.core.robot_view(r) for r in ROBOTS]}")
    assert set(world.report_statuses().values()) == {SHIPPED}
    assert world.core.shipped == len(orders)
    assert "DEADLOCK_DETECTED" not in world.event_codes("ERROR")
