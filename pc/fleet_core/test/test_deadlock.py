"""Deadlock safety net. One-way lanes mean it never fires normally, so cycles are forced."""
from core_helpers import cmds, codes, make_core, place
from fleet_core.commands import OrderRecord
from fleet_core.deadlock import choose_retreat, find_cycle, priority, wait_graph
from fleet_core.params import load_params
from fleet_core.zones import ZoneManager

ORDERS = [OrderRecord("O-001", {"A": 1}), OrderRecord("O-002", {"B": 1})]


def forced_cycle():
    zones = ZoneManager(["A", "B", "C"], clock=lambda: 0.0)
    zones.request("r1", "A")
    zones.request("r2", "B")
    zones.request("r1", "B")
    zones.request("r2", "A")
    return zones


def test_t06_forced_cycle_is_found():
    """T-06: two robots each wait for the zone the other holds."""
    graph = wait_graph(forced_cycle())
    assert graph == {"r1": {"r2"}, "r2": {"r1"}}
    assert set(find_cycle(graph)) == {"r1", "r2"}


def test_t06_three_robot_cycle_is_found():
    """T-06: a longer cycle is reported with its members and without bystanders."""
    graph = {"a": {"b"}, "b": {"c"}, "c": {"a"}, "d": {"a"}}
    assert set(find_cycle(graph)) == {"a", "b", "c"}


def test_t06_plain_queue_is_not_a_deadlock():
    """T-06: no false positive when robots queue behind a holder that is not waiting."""
    zones = ZoneManager(["A", "B"], clock=lambda: 0.0)
    zones.request("r1", "A")
    zones.request("r2", "A")
    zones.request("r3", "A")
    zones.request("r2", "B")
    assert find_cycle(wait_graph(zones)) is None
    assert find_cycle({}) is None


def test_t06_retreat_picks_lowest_priority():
    """T-06: at a station +200, carrying +100, lowest total retreats."""
    params = load_params()
    scores = {
        "docked": priority(params, at_station=True, carrying=False),
        "full": priority(params, at_station=False, carrying=True),
        "empty": priority(params, at_station=False, carrying=False),
        "both": priority(params, at_station=True, carrying=True),
    }
    assert scores == {"docked": 200, "full": 100, "empty": 0, "both": 300}
    assert choose_retreat(["docked", "full", "empty"], scores.get) == "empty"
    assert choose_retreat(["docked", "full"], scores.get) == "full"
    assert choose_retreat(["both", "docked"], scores.get) == "docked"


def _cycle_over_real_reservations(cells):
    """Run one tick so each robot holds its real next zone, then force the crossed requests."""
    core, now = make_core(list(cells))
    for robot, cell in cells.items():
        place(core, robot, cell)
    core.on_order_board("RUN", ORDERS)
    core.tick()
    assert core.zones.held_by("r1") == ["P-6", "Q-WORK-2"]
    assert core.zones.held_by("r2") == ["Q-WORK-1", "Z-WORK"]
    core.zones.request("r1", "Z-WORK")
    core.zones.request("r2", "Q-WORK-2")
    return core, now


def _report(core, cells):
    for robot, cell in cells.items():
        place(core, robot, cell)


def test_t06_core_stops_new_moves_then_gives_up_a_reservation():
    """T-06: first step halts and reports, second step frees an unentered zone."""
    cells = {"r1": (3, 4), "r2": (4, 3), "r3": (3, 5)}
    core, now = _cycle_over_real_reservations(cells)

    first = core.tick()
    assert "DEADLOCK_DETECTED" in codes(first)
    assert not cmds(first, "MoveCmd")

    now[0] = core.params.deadlock_grace_s / 2
    _report(core, cells)
    second = core.tick()
    assert "DEADLOCK_DETECTED" not in codes(second), "reported once per episode"
    assert not cmds(second, "MoveCmd")

    now[0] = core.params.deadlock_grace_s + 0.1
    _report(core, cells)
    third = core.tick()
    assert "DEADLOCK_RETREAT" in codes(third)
    assert cmds(third, "CancelCmd"), "its running goal led into the zone it gave up"
    assert core.zones.holder("Q-WORK-2") == "r1", "kept until the robot has had time to stop"

    now[0] += core.params.recover_check_s
    _report(core, cells)
    core.tick()
    assert core.zones.holder("Q-WORK-2") is None, "equal priority: r1 sorts first and retreats"
    assert core.zones.holder("Z-WORK") == "r2"
    assert find_cycle(wait_graph(core.zones)) is None


def test_t06_core_retreats_the_empty_robot_before_the_full_one():
    """T-06: the real priority of each robot decides who gives up its reservation."""
    cells = {"r1": (3, 4), "r2": (4, 3)}
    core, now = _cycle_over_real_reservations(cells)
    core._missions["r1"].basket = "FULL"  # white box: only the priority input is forced

    core.tick()
    now[0] = core.params.deadlock_grace_s + 0.1
    _report(core, cells)
    assert "DEADLOCK_RETREAT" in codes(core.tick())
    now[0] += core.params.recover_check_s
    _report(core, cells)
    core.tick()
    assert core.zones.holder("Z-WORK") is None, "r2 is empty and gives way"
    assert core.zones.holder("Q-WORK-2") == "r1"


def test_t06_core_reports_unresolved_when_the_robot_is_inside_every_zone():
    """T-06: a robot inside all its zones has nothing to give up; reversing is not built."""
    core, now = make_core(["r1", "r2"])
    cells = {"r1": (4, 3), "r2": (3, 3)}  # Q-WORK-1 and Q-WORK-2

    def report():
        for robot, cell in cells.items():
            place(core, robot, cell)

    report()
    core.on_order_board("RUN", ORDERS)
    core.zones.request("r1", "Q-WORK-2")
    core.zones.request("r2", "Q-WORK-1")

    assert "DEADLOCK_DETECTED" in codes(core.tick())
    now[0] = core.params.deadlock_grace_s + 0.1
    report()
    assert "DEADLOCK_UNRESOLVED" in codes(core.tick())
    assert core.zones.holder("Q-WORK-1") == "r1"
    assert core.zones.holder("Q-WORK-2") == "r2"
    now[0] += 0.1
    report()
    assert "DEADLOCK_UNRESOLVED" not in codes(core.tick()), "reported once per episode"


def test_t06_core_keeps_the_zone_if_the_robot_entered_it_before_stopping():
    """T-06: a robot that drove on before the cancel arrived must keep the zone it is in."""
    cells = {"r1": (3, 4), "r2": (4, 3), "r3": (3, 5)}
    core, now = _cycle_over_real_reservations(cells)
    core.tick()
    now[0] = core.params.deadlock_grace_s + 0.1
    _report(core, cells)
    assert "DEADLOCK_RETREAT" in codes(core.tick())
    now[0] += core.params.recover_check_s
    _report(core, {**cells, "r1": (3, 3)})  # it reached the reserved Q-WORK-2 meanwhile
    out = core.tick()
    assert core.zones.holder("Q-WORK-2") == "r1"
    assert "DEADLOCK_UNRESOLVED" in codes(out)


def test_t06_core_looks_again_when_too_few_reports_arrived_in_time():
    """T-06: a robot still outside the zone keeps the retreat alive until reports confirm it."""
    cells = {"r1": (3, 4), "r2": (4, 3), "r3": (3, 5)}
    core, now = _cycle_over_real_reservations(cells)
    core.tick()
    now[0] = core.params.deadlock_grace_s + 0.1
    _report(core, cells)
    assert "DEADLOCK_RETREAT" in codes(core.tick())

    core._missions["r1"].outside.clear()  # the reports that would confirm it are late
    now[0] += core.params.recover_check_s
    out = core.tick()
    assert core.zones.holder("Q-WORK-2") == "r1"
    assert "DEADLOCK_UNRESOLVED" not in codes(out), "it has not entered the zone"

    _report(core, cells)
    now[0] += core.params.recover_check_s
    core.tick()
    assert core.zones.holder("Q-WORK-2") is None


def test_t06_a_retreating_robot_going_silent_ends_the_episode():
    """T-06: COMM_LOST cancels its requests, which dissolves the cycle and the retreat with it."""
    cells = {"r1": (3, 4), "r2": (4, 3), "r3": (3, 5)}
    core, now = _cycle_over_real_reservations(cells)
    core.tick()
    now[0] = core.params.deadlock_grace_s + 0.1
    _report(core, cells)
    assert "DEADLOCK_RETREAT" in codes(core.tick())
    core._missions["r1"].outside.clear()

    seen = []
    others = {k: v for k, v in cells.items() if k != "r1"}
    for _ in range(int(core.params.comm_timeout_s / core.params.recover_check_s) + 3):
        now[0] += core.params.recover_check_s
        _report(core, others)
        seen += codes(core.tick())
    assert "COMM_LOST" in seen
    assert find_cycle(wait_graph(core.zones)) is None
    assert core._retreating is None
    assert core.zones.holder("Q-WORK-2") == "r1", "kept: its position can no longer be confirmed"


def test_a_cycle_through_a_free_zone_of_a_conflict_pair_is_found():
    """r1 waits for r2 across a pair, r2 for r1 in line. No zone in it has r1 as its holder."""
    zones = ZoneManager(["A", "B", "Y"], clock=lambda: 0.0, conflicts=[frozenset({"A", "Y"})])
    assert zones.request("r2", "Y")
    assert not zones.request("r1", "A"), "free, but it is a conflict pair with what r2 holds"
    assert not zones.request("r2", "A"), "behind r1 in line"
    graph = wait_graph(zones)
    assert graph == {"r1": {"r2"}, "r2": {"r1"}}
    assert set(find_cycle(graph)) == {"r1", "r2"}


def test_waiting_for_a_free_zone_of_a_pair_is_not_a_cycle_by_itself():
    zones = ZoneManager(["A", "Y"], clock=lambda: 0.0, conflicts=[frozenset({"A", "Y"})])
    assert zones.request("r2", "Y")
    assert not zones.request("r1", "A")
    assert wait_graph(zones) == {"r1": {"r2"}}
    assert find_cycle(wait_graph(zones)) is None
