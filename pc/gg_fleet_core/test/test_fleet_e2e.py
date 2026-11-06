"""Whole-run scenarios on the virtual warehouse. sim_world checks the invariants each step."""
import math
from dataclasses import replace

import pytest
from gg_fleet_core.common.commands import CancelCmd, DockCmd, FillCmd, MoveCmd, SwapCmd
from gg_fleet_core.common.params import load_params
from sim_world import SimWorld, six_orders

SEEDS = [7, 11, 23]


def assert_clean_finish(world, count=None):
    expected = count if count is not None else len(world.orders)
    assert world.core.shipped == expected
    assert world.all_orders_final()
    assert world.all_parked()
    assert not world.event_codes("ERROR"), world.events
    assert world.bound_order() == world.registration, "bound in registration order"
    assert world.shipped_order() == world.registration, "shipped in the order bound"
    assert set(world.report_statuses().values()) == {"SHIPPED"}


def order(name, at_s=0.0, **items):
    return {"at_s": at_s, "id": name, "items": items}


def only_robot(start="P-6"):
    return {"pinky1": {"start": start, "speed_mps": 0.20, "pose_noise_m": 0.03}}


def two_robots(first="P-6", second="Z-R3"):
    return {"pinky1": {"start": first, "speed_mps": 0.20, "pose_noise_m": 0.03},
            "pinky2": {"start": second, "speed_mps": 0.20, "pose_noise_m": 0.03}}


# ---- normal runs ----------------------------------------------------------------------------


@pytest.mark.parametrize("seed", SEEDS)
def test_six_orders_ship_in_order_and_every_robot_ends_up_waiting(seed):
    world = SimWorld(seed=seed)
    assert world.run_until(world.done)
    assert_clean_finish(world)
    for name in world.robots:
        assert world.core.robot_view(name)["waypoint"] in world.site.parking
        assert world.core.robot_view(name)["basket"] == "EMPTY"
        assert world.virtual[name].has_basket
    assert sum(world.shipped_items.values()) == 12


def test_no_orders_sends_every_robot_to_waiting():
    """A robot left on the lane drives through the docks and over the link."""
    robots = {"pinky1": {"start": "Z-R1", "speed_mps": 0.2, "pose_noise_m": 0.03},
              "pinky2": {"start": "Q-OUT-2", "speed_mps": 0.2, "pose_noise_m": 0.03},
              "pinky3": {"start": "Z-WORK", "speed_mps": 0.2, "pose_noise_m": 0.03}}
    world = SimWorld(orders=[], robots=robots)
    assert world.run_until(world.all_parked, 300)
    assert not world.commands_of(DockCmd)
    assert not world.commands_of(FillCmd)


def test_waiting_keeps_the_entrance_clear():
    """Waiting robots take the deepest cells, so the first waiting cells stay free."""
    world = SimWorld()
    assert world.run_until(world.done)
    cells = {world.core.robot_view(n)["waypoint"] for n in world.robots}
    assert cells == set(world.site.parking[-3:])


def test_order_arriving_after_everyone_waits_wakes_the_front_robot():
    world = SimWorld(orders=[order("O-001", A=1), order("O-002", at_s=200.0, B=1)])
    assert world.run_until(lambda: world.t > 100 and world.all_parked())
    front = next(n for n in world.robots
                 if world.core.robot_view(n)["waypoint"] == world.site.parking[-1])
    start = world.t
    assert world.run_until(world.done)
    assert_clean_finish(world)
    first_move = next(c for t, c in world.commands
                      if t >= 200.0 and t >= start and isinstance(c, MoveCmd))
    assert first_move.robot == front


def test_order_arriving_mid_run_ships_too():
    orders = [*six_orders(), order("O-007", at_s=30.0, A=1, B=1)]
    world = SimWorld(orders=orders)
    assert world.run_until(world.done)
    assert_clean_finish(world)


def test_robot_goes_straight_back_to_work_when_orders_are_queued():
    world = SimWorld(robots=only_robot(), orders=[order("O-001", A=1), order("O-002", B=1)])
    assert world.run_until(world.done)
    assert_clean_finish(world)
    swap_done = next(t for t, c in world.commands if isinstance(c, SwapCmd))
    second_fill = [t for t, c in world.commands if isinstance(c, FillCmd)][1]
    waiting = [cell for t, name, cell, _ in world.visits if swap_done < t < second_fill
               and cell in world.site.parking]
    assert not waiting, "no detour through the waiting cells"


def test_order_burst_keeps_every_robot_working_without_waiting():
    orders = [order(f"O-{i:03d}", A=1) for i in range(1, 9)]
    world = SimWorld(orders=orders)
    assert world.run_until(world.done)
    assert_clean_finish(world)
    first = next(t for t, c in world.commands if isinstance(c, SwapCmd))
    last_bind = [t for t, c in world.commands if isinstance(c, FillCmd)][-1]
    waiting = [cell for t, _, cell, _ in world.visits if first < t < last_bind
               and cell in world.site.parking[:3]]
    assert not waiting, "with work queued nobody turns off into the waiting loop"


def test_dock_order_follows_arrival_at_outbound():
    world = SimWorld(seed=11)
    assert world.run_until(world.done)
    queue_cell = world.site.stations["OUTBOUND"].queue[-1][0]  # the node next to the dock
    arrivals = [name for _, name, cell, basket in world.visits
                if cell == queue_cell and basket == "FULL"]
    docks = [c.robot for c in world.commands_of(DockCmd) if c.dock_id == "OUTBOUND"]
    assert docks == arrivals
    assert len(docks) == 6


def test_fifo_bind_with_robots_arriving_out_of_call_order():
    """pinky1 is called first and far away. pinky2 is called later and docks first."""
    robots = two_robots("Z-R1", "P-6")
    robots["pinky1"]["cell"] = (8, 0)
    orders = [order("O-001", A=1), order("O-002", at_s=1.0, B=1)]
    world = SimWorld(robots=robots, orders=orders)
    world.core._missions["pinky1"].called = True  # it kept the call from before pinky2 was free
    assert world.run_until(world.done)
    assert_clean_finish(world)
    bound = [(e.detail, e.robot) for e in world.events if e.code == "ORDER_BOUND"]
    assert bound == [("O-001", "pinky2"), ("O-002", "pinky1")]


# ---- stock ----------------------------------------------------------------------------------


def test_short_stock_ships_partial_and_holds_an_order_it_cannot_start():
    orders = [order("O-001", A=2, C=1), order("O-002", B=1), order("O-003", C=1)]
    world = SimWorld(orders=orders, stock={"A": 1, "B": 0, "C": 5}, robots=only_robot())
    assert world.run_until(world.done)
    assert world.report_statuses() == {"O-001": "PARTIAL", "O-002": "HELD", "O-003": "SHIPPED"}
    report = world.reports[0]
    assert (report.placed, report.missing) == ({"A": 1, "C": 1}, {"A": 1})
    held = next(r for r in world.reports if r.status == "HELD")
    assert (held.placed, held.missing) == ({}, {"B": 1})
    assert world.core.shipped == 2
    assert not world.event_codes("ERROR")
    assert world.shipped_items == {"A": 1, "C": 2}


def test_held_order_frees_the_robot_with_its_empty_basket():
    world = SimWorld(orders=[order("O-001", B=1), order("O-002", A=1)],
                     stock={"A": 1, "B": 0}, robots=only_robot())
    assert world.run_until(world.done)
    assert world.report_statuses() == {"O-001": "HELD", "O-002": "SHIPPED"}
    docks = [c.dock_id for c in world.commands_of(DockCmd)]
    assert docks == ["WORK", "OUTBOUND"], "the next order is taken at the same dock"


def test_over_capacity_order_never_reaches_an_arm_and_the_rest_go_on():
    orders = [order("O-001", A=5), order("O-002", A=1)]
    world = SimWorld(orders=orders)
    assert world.run_until(world.done)
    assert world.report_statuses() == {"O-001": "HELD", "O-002": "SHIPPED"}
    assert "O-001" not in {c.order_id for c in world.commands_of(FillCmd)}
    assert "capacity" in world.reports[0].detail
    assert world.reports[0].missing == {}, "central reads missing as a stock mismatch"
    assert not world.event_codes("ERROR")


def test_without_ship_partial_a_partial_basket_stops_the_robot_for_the_operator():
    params = replace(load_params(), ship_partial=False)
    world = SimWorld(orders=[order("O-001", A=2)], stock={"A": 1}, params=params,
                     robots=only_robot())
    assert world.run_until(lambda: world.state("pinky1") == "ERROR", 300)
    assert world.report_statuses() == {"O-001": "HELD"}
    assert "FILL_FAILED" in world.event_codes("ERROR")
    assert world.reports[0].placed == {"A": 1}
    assert not world.commands_of(SwapCmd)
    assert world.virtual["pinky1"].basket == {"A": 1}


# ---- comm loss ------------------------------------------------------------------------------


@pytest.mark.parametrize("seed", SEEDS)
def test_comm_loss_mid_trip(seed):
    """Zones stay, others take the work, it recovers, nothing is lost."""
    faults = [{"at_s": 40, "robot": "pinky2", "type": "comm_loss", "duration_s": 20}]
    world = SimWorld(seed=seed, faults=faults)
    world.run(40)
    world.run(world.params.comm_timeout_s + 0.5)
    assert world.state("pinky2") == "COMM_LOST"
    assert "COMM_LOST" in world.event_codes("WARN")
    held = world.core.zones.held_by("pinky2")
    assert held, "zones stay with the robot"
    assert world.core.robot_view("pinky2")["called"] is False
    world.run(10)
    assert world.state("pinky2") == "COMM_LOST"
    assert world.core.zones.held_by("pinky2") == held, "no pose, so nothing is released"
    assert world.run_until(world.done)
    assert_clean_finish(world)
    assert "COMM_RESTORED" in world.event_codes()


def test_comm_restored_cancels_waits_and_keeps_the_zone_it_is_in():
    """CancelCmd first, no new goal during the check, then only outside zones go."""
    faults = [{"at_s": 12, "robot": "pinky1", "type": "comm_loss", "duration_s": 8}]
    world = SimWorld(faults=faults)
    assert world.run_until(lambda: world.virtual["pinky1"].comm_up and world.t > 12)
    restored_at = world.t
    world.step()
    assert world.state("pinky1") == "RECOVERING"
    held_during = world.core.zones.held_by("pinky1")
    world.run(world.params.recover_check_s - 0.3)
    moves = [t for t, c in world.commands if isinstance(c, MoveCmd) and c.robot == "pinky1"]
    assert not [t for t in moves if t > restored_at], "no new goal while checking the pose"
    assert world.core.zones.held_by("pinky1") == held_during
    world.run(0.5)
    assert world.state("pinky1") != "RECOVERING"
    assert world.run_until(world.done)
    assert_clean_finish(world)


def test_comm_loss_with_a_full_basket_still_ships_it():
    world = SimWorld(orders=[order("O-001", A=1)])
    assert world.run_until(
        lambda: any(world.core.robot_view(n)["basket"] == "FULL" for n in world.robots))
    carrier = next(n for n in world.robots if world.core.robot_view(n)["basket"] == "FULL")
    world.run(5)
    world.pending_faults.append({
        "at_s": world.t, "robot": carrier, "type": "comm_loss", "duration_s": 10})
    world.run(6)
    assert world.state(carrier) == "COMM_LOST"
    assert world.core.robot_view(carrier)["order"] == "O-001"
    assert world.run_until(world.done)
    assert_clean_finish(world)


# ---- moves and docks failing ----------------------------------------------------------------


def test_nav_abort_twice_then_success():
    faults = [{"at_s": 5, "robot": "pinky1", "type": "nav_abort", "count": 2}]
    world = SimWorld(faults=faults)
    assert world.run_until(world.done)
    assert_clean_finish(world)
    assert world.event_codes("WARN").count("GOAL_RETRY") == 2


def test_robot_failing_on_the_way_does_not_lose_the_order():
    """It holds no order yet, so after the operator clears it the next robot takes the order."""
    faults = [{"at_s": 0.0, "robot": "pinky1", "type": "nav_abort", "count": 3}]
    world = SimWorld(faults=faults, orders=[order("O-001", A=1)], robots=two_robots())
    assert world.run_until(lambda: world.state("pinky1") == "ERROR", 60)
    assert "GOAL_FAILED" in world.event_codes("ERROR")
    assert world.core.robot_view("pinky1")["order"] is None
    assert not world.bound_order()
    assert world.run_until(lambda: world.core.robot_view("pinky2")["called"], 5)
    held = world.core.zones.held_by("pinky1")
    assert held
    world.run(10)
    assert world.core.zones.held_by("pinky1") == held, "error keeps the zones"
    released = world.confirm_clear("pinky1", held, "pushed it off the lane")
    assert set(released) == set(held)
    assert world.state("pinky1") == "OUT_OF_SERVICE"
    world.step()
    assert "ZONES_CLEARED" in world.event_codes("INFO")
    assert "ORDER_REQUEUED" not in world.event_codes(), "it never had the order"
    assert world.run_until(
        lambda: world.all_orders_final() and world.all_parked(["pinky2"]), 300)
    assert world.report_statuses() == {"O-001": "SHIPPED"}
    assert world.bound_order() == ["O-001"]
    assert world.reports[0].robot == "pinky2"


def test_robot_stuck_on_the_lane_blocks_those_behind_and_they_warn():
    """One ring: a robot stopped outside the waiting cells stops the rest, loudly."""
    params = replace(load_params(), zone_wait_warn_s=5.0)
    faults = [{"at_s": 12, "robot": "pinky1", "type": "stuck"}]
    world = SimWorld(faults=faults, params=params)
    assert world.run_until(lambda: world.state("pinky1") == "ERROR", 200)
    assert world.run_until(lambda: "ZONE_WAIT_LONG" in world.event_codes("WARN"), 200)
    stopped = world.core.shipped
    world.run(100)
    assert world.core.shipped == stopped, "nothing gets past it"
    world.confirm_clear("pinky1", world.core.zones.held_by("pinky1"))
    assert world.run_until(
        lambda: world.all_orders_final() and world.all_parked(["pinky2", "pinky3"]))
    finished = world.report_statuses()
    assert set(finished) == {o["id"] for o in world.orders}
    assert set(finished.values()) == {"SHIPPED"}


def test_dock_failure_retries_then_error():
    """Dock failures use the same retry budget as moves."""
    world = SimWorld(orders=[order("O-001", A=1)],
                     faults=[{"at_s": 0.0, "robot": "pinky1", "type": "dock_fail", "count": 2}])
    assert world.run_until(world.done)
    assert_clean_finish(world)

    world = SimWorld(orders=[order("O-001", A=1)],
                     faults=[{"at_s": 0.0, "robot": "pinky1", "type": "dock_fail", "count": 3}])
    assert world.run_until(lambda: world.state("pinky1") == "ERROR", 200)
    assert world.core.zones.held_by("pinky1")
    assert "GOAL_FAILED" in world.event_codes("ERROR")
    assert not world.bound_order(), "docking never finished, so nothing was bound"


# ---- arms failing ---------------------------------------------------------------------------


def test_fill_retry_sends_only_what_remains():
    world = SimWorld(orders=[order("O-001", A=2, C=1)], robots=only_robot(),
                     faults=[{"at_s": 0.0, "type": "fill_fail", "count": 1, "place": 1}])
    assert world.run_until(world.done)
    fills = world.commands_of(FillCmd)
    assert [f.items for f in fills] == [{"A": 2, "C": 1}, {"A": 1, "C": 1}]
    assert "FILL_RETRY" in world.event_codes("WARN")
    assert world.report_statuses() == {"O-001": "SHIPPED"}
    assert world.shipped_items == {"A": 2, "C": 1}, "no item was placed twice"


def test_arm_failing_three_times_then_operator_recovery_finishes_every_order():
    """The operator_recovery scenario: the order comes back and every order ships."""
    orders = [order("O-001", A=2, C=2), order("O-002", B=1), order("O-003", D=1)]
    robots = two_robots("P-6", "P-5")
    world = SimWorld(orders=orders, robots=robots,
                     faults=[{"at_s": 0.0, "type": "fill_fail", "count": 3, "place": 1}])
    assert world.run_until(lambda: world.state("pinky1") == "ERROR", 200)
    assert "FILL_FAILED" in world.event_codes("ERROR")
    assert world.core.robot_view("pinky1")["order"] == "O-001"
    assert world.core.shipped == 0

    world.confirm_clear("pinky1", world.core.zones.held_by("pinky1"), "took it off")
    world.step()
    requeued = next(e for e in world.events if e.code == "ORDER_REQUEUED")
    assert requeued.detail.startswith("O-001")
    assert world.run_until(lambda: world.all_orders_final() and world.all_parked(["pinky2"]))
    assert world.report_statuses() == {o["id"]: "SHIPPED" for o in orders}
    assert world.shipped_order() == ["O-001", "O-002", "O-003"], "it keeps its place"
    assert {r.robot for r in world.reports} == {"pinky2"}


def test_partial_swap_retries_the_put_and_reports_shipped_exactly_once():
    world = SimWorld(orders=[order("O-001", A=1)], robots=only_robot(),
                     faults=[{"at_s": 0.0, "type": "swap_partial", "count": 1}])
    assert world.run_until(world.done)
    swaps = world.commands_of(SwapCmd)
    assert [(s.take_full, s.put_empty) for s in swaps] == [(True, True), (False, True)]
    assert world.report_statuses() == {"O-001": "SHIPPED"}
    assert len(world.reports) == 1
    assert "SWAP_RETRY" in world.event_codes("WARN")
    assert world.virtual["pinky1"].has_basket


def test_swap_failing_every_time_ends_in_swap_failed():
    world = SimWorld(orders=[order("O-001", A=1)], robots=only_robot(),
                     faults=[{"at_s": 0.0, "type": "swap_fail", "count": 3}])
    assert world.run_until(lambda: world.state("pinky1") == "ERROR", 300)
    assert "SWAP_FAILED" in world.event_codes("ERROR")
    assert not world.reports
    world.confirm_clear("pinky1", world.core.zones.held_by("pinky1"), "took it off")
    world.step()
    assert any(e.code == "ORDER_REQUEUED" and "'A': 1" in e.detail for e in world.events)


def test_silent_fill_server_goes_to_error_after_the_timeout():
    params = replace(load_params(), fill_timeout_s=10.0)
    world = SimWorld(orders=[order("O-001", A=1)], robots=only_robot(), params=params,
                     faults=[{"at_s": 0.0, "type": "fill_silent", "count": 1}])
    assert world.run_until(lambda: "ARM_NO_RESULT" in world.event_codes("ERROR"), 300)
    assert world.state("pinky1") == "ERROR"
    assert world.core.robot_view("pinky1")["order"] == "O-001", "the order stays bound"
    assert [c for c in world.commands_of(CancelCmd) if c.robot == "workcell"]


def test_silent_swap_arm_goes_to_error_after_the_timeout():
    params = replace(load_params(), swap_timeout_s=10.0)
    world = SimWorld(orders=[order("O-001", A=1)], robots=only_robot(), params=params,
                     faults=[{"at_s": 0.0, "type": "swap_silent", "count": 1}])
    assert world.run_until(lambda: "ARM_NO_RESULT" in world.event_codes("ERROR"), 300)
    assert world.state("pinky1") == "ERROR"
    assert world.core.shipped == 0


# ---- run modes ------------------------------------------------------------------------------


def test_pause_stops_new_moves_and_grants_then_resumes():
    faults = [{"at_s": 15, "type": "run_mode", "mode": "PAUSE"}]
    world = SimWorld(faults=faults)
    world.run(15.05)
    assert world.run_mode == "PAUSE"
    world.step()
    count = len(world.commands_of(MoveCmd))
    docks = len(world.commands_of(DockCmd))
    holders = {z: v["holder"] for z, v in world.core.zones.snapshot().items()}
    world.run(30)
    assert len(world.commands_of(MoveCmd)) == count
    assert len(world.commands_of(DockCmd)) == docks
    now = {z: v["holder"] for z, v in world.core.zones.snapshot().items()}
    granted = [z for z, h in now.items() if h and h != holders[z]]
    assert not granted, "no grants while paused"
    world.run_mode = "RUN"
    assert world.run_until(world.done)
    assert_clean_finish(world)


def test_stop_cancels_goals_and_arm_jobs_and_run_resumes():
    world = SimWorld()
    assert world.run_until(lambda: "workcell" in world.jobs and world.jobs["workcell"].placed)
    job = world.jobs["workcell"]
    active = [n for n in world.robots if world.virtual[n].target and math.dist(
        (world.virtual[n].x, world.virtual[n].y), world.virtual[n].target) > 0.1]
    world.run_mode = "STOP"
    world.step()
    world.step()
    world.step()
    canceled = {c.robot for c in world.commands_of(CancelCmd)}
    assert set(active) <= canceled
    assert "workcell" in canceled
    assert not world.jobs
    assert all(world.virtual[n].goal_id is None for n in world.robots)
    poses = {n: (world.virtual[n].x, world.virtual[n].y) for n in world.robots}
    count = len(world.commands)
    world.run(20)
    assert not [c for c in world.commands[count:] if not isinstance(c[1], CancelCmd)]
    assert poses == {n: (world.virtual[n].x, world.virtual[n].y) for n in world.robots}
    assert not world.event_codes("WARN")
    assert not world.event_codes("ERROR")
    fills_before = len(world.commands_of(FillCmd))
    world.run_mode = "RUN"
    assert world.run_until(world.done)
    assert_clean_finish(world)
    resumed = world.commands_of(FillCmd)[fills_before]
    assert resumed.order_id == job.cmd.order_id
    assert sum(resumed.items.values()) == sum(job.cmd.items.values()) - sum(job.placed.values())


def test_shared_lane_failure_scenario():
    """Comm loss, then two nav aborts, as in the shared_lane_failure scenario."""
    faults = [
        {"at_s": 40, "robot": "pinky2", "type": "comm_loss", "duration_s": 20},
        {"at_s": 75, "robot": "pinky1", "type": "nav_abort", "count": 2},
    ]
    world = SimWorld(seed=7, faults=faults)
    assert world.run_until(world.done)
    assert_clean_finish(world)


@pytest.mark.parametrize("seed", SEEDS)
def test_noisy_pose_does_not_release_a_zone_the_robot_is_still_next_to(seed):
    """A lone noisy report must not free a zone while the robot's body is still at its edge."""
    world = SimWorld(seed=seed, gaussian_noise=True)
    assert world.run_until(world.done)
    assert_clean_finish(world)
    noise = 0.03
    from_floor = world.min_true_release_distance + world.site.half_width
    assert from_floor >= world.params.release_distance_m - 2 * noise
