"""Deadlocks the controller builds by itself (work order H1).

feature_map.yaml with bundle_two_way=False locks each direction of the tunnel T1-T2 on its own,
so two robots entering it from both ends wait for each other. Every robot in that cycle holds
only the zone it stands in, so the retreat has nothing to give up (DEADLOCK_UNRESOLVED).
"""
import math
from dataclasses import replace
from pathlib import Path

import yaml
from fleet_core.common.commands import CancelCmd, DispatchResult, DockCmd, MoveCmd, RobotIsolated
from fleet_core.common.params import load_params
from fleet_core.traffic.controller import TrafficController
from fleet_core.traffic.deadlock import cycle_members, find_cycle, wait_graph
from fleet_core.traffic.lane_map import load_lane_map
from sim_world import SimWorld

REPO = Path(__file__).resolve().parents[3]
FEATURE = REPO / "sim/fake_robots/maps/feature_map.yaml"
TWO_LANE = REPO / "sim/fake_robots/maps/two_lane_segments.yaml"
BASIC = REPO / "sim/fake_robots/scenarios/basic_orders.yaml"
PARAMS = load_params()
DT = 0.1
SPEED = 0.3  # m/s
DOCK_S = 1.0


class Drive:
    """A bare TrafficController with robots that follow each goal's path at SPEED."""

    def __init__(self, starts, **over):
        self.now = [0.0]
        self.site = load_lane_map(FEATURE, bundle_two_way=False)
        self.core = TrafficController(self.site, replace(PARAMS, **over), list(starts),
                                      lambda: self.now[0])
        self.pose = {robot: self.site.waypoint_pose(w) for robot, w in starts.items()}
        self.goal = {}  # robot -> (kind, goal_id, points or seconds left)
        self.log = []  # (time, output)
        self._report()
        self.core.on_run_mode("RUN")

    def dispatch(self, goal_id, station, robot):
        self.core.on_dispatch(goal_id, station, robot)

    def run(self, seconds, until=None):
        for _ in range(round(seconds / DT)):
            self.now[0] = round(self.now[0] + DT, 6)
            self._move()
            self._report()
            for item in self.core.tick():
                self.log.append((self.now[0], item))
                self._apply(item)
            if until and until():
                return True
        return False

    def moves(self, robot, after=0.0):
        return [c for t, c in self.log if t >= after and isinstance(c, MoveCmd)
                and c.robot == robot]

    def events(self, code):
        return [(t, c) for t, c in self.log
                if type(c).__name__ == "EventOut" and c.code == code]

    def outputs(self, kind):
        return [(t, c) for t, c in self.log if isinstance(c, kind)]

    def _report(self):
        for robot, (x, y) in self.pose.items():
            state = "MOVING" if robot in self.goal else "IDLE"
            self.core.on_robot_state(robot, x, y, 0.0, state)

    def _apply(self, item):
        if isinstance(item, MoveCmd):
            points = [tuple(p[:2]) for p in item.path[1:]] or [(item.x, item.y)]
            self.goal[item.robot] = ("move", item.goal_id, points)
        elif isinstance(item, DockCmd):
            self.goal[item.robot] = ("dock", item.goal_id, DOCK_S)
        elif isinstance(item, CancelCmd):
            goal = self.goal.get(item.robot)
            if goal and goal[1] == item.goal_id:
                del self.goal[item.robot]
                self.core.on_move_result(item.robot, item.goal_id, False, "CANCELED")

    def _move(self):
        for robot, (kind, goal_id, rest) in list(self.goal.items()):
            if kind == "dock":
                if rest - DT > 1e-9:
                    self.goal[robot] = (kind, goal_id, rest - DT)
                    continue
                del self.goal[robot]
                self.core.on_dock_result(robot, goal_id, True)
                continue
            step = SPEED * DT
            x, y = self.pose[robot]
            while rest:
                gap = math.dist((x, y), rest[0])
                if gap > step:
                    x += (rest[0][0] - x) / gap * step
                    y += (rest[0][1] - y) / gap * step
                    break
                x, y = rest.pop(0)
                step -= gap
            self.pose[robot] = (x, y)
            if not rest:
                del self.goal[robot]
                self.core.on_move_result(robot, goal_id, True, "")


def tunnel_deadlock(third=True, **over):
    """r1 at T1 goes east through the tunnel, r2 at T2 goes west; r3 starts on E1."""
    starts = {"r1": "T1", "r2": "T2", **({"r3": "E1"} if third else {})}
    drive = Drive(starts, **over)
    drive.dispatch(1, "OUTBOUND", "r1")
    drive.dispatch(2, "WORK", "r2")
    assert drive.run(2.0, until=lambda: drive.events("DEADLOCK_DETECTED"))
    assert set(find_cycle(wait_graph(drive.core.zones))) == {"r1", "r2"}
    return drive


def test_the_reproduction_is_a_cycle_with_nothing_to_give_up():
    drive = tunnel_deadlock()
    drive.run(PARAMS.deadlock_grace_s + 1.0)
    assert len(drive.events("DEADLOCK_UNRESOLVED")) == 1
    assert not drive.events("DEADLOCK_RETREAT")


def test_a_robot_outside_the_cycle_keeps_moving():
    """r3's way E1-E2-O is free. Before H1 it got no MoveCmd for 300 s after the detection."""
    drive = tunnel_deadlock()
    detected_at = drive.events("DEADLOCK_DETECTED")[0][0]
    drive.run(1.0)
    drive.dispatch(3, "OUTBOUND", "r3")
    drive.run(20.0, until=lambda: any(r.goal_id == 3 for _, r in drive.outputs(DispatchResult)))
    assert drive.moves("r3", after=detected_at + DT), "r3 is not in the cycle"
    done = [r for _, r in drive.outputs(DispatchResult) if r.goal_id == 3]
    assert done and done[0].success, "r3 docked at OUTBOUND"
    assert not drive.moves("r1") or all(c.target_id != "T2" for c in drive.moves("r1")), \
        "the cycle robots still do not enter each other's lane"


def test_the_cycle_robots_send_nothing_new_while_they_wait():
    drive = tunnel_deadlock()
    before = {r: len(drive.moves(r)) for r in ("r1", "r2")}
    drive.run(PARAMS.deadlock_grace_s + 2.0)
    assert {r: len(drive.moves(r)) for r in ("r1", "r2")} == before


def test_an_unresolved_cycle_isolates_its_lowest_priority_robot():
    """Same cycle for deadlock_unresolved_s after DEADLOCK_UNRESOLVED: isolate one robot.

    r1 and r2 both carry a dispatch, so the tie goes to the name: r1. r2 then goes around the
    isolated r1 (T2-QO-O-F0-F1-F2-W) and docks at WORK. Without r3: a robot parking behind
    r2 would wait at F1 for good, since the lane F1-P-1 crosses the tunnel where r1 stands."""
    drive = tunnel_deadlock(third=False)
    wait = drive.core.params.deadlock_unresolved_s
    assert drive.run(PARAMS.deadlock_grace_s + 1.0,
                     until=lambda: drive.events("DEADLOCK_UNRESOLVED"))
    unresolved_at = drive.events("DEADLOCK_UNRESOLVED")[0][0]

    drive.run(wait - 0.5)
    assert not drive.outputs(RobotIsolated), "not before deadlock_unresolved_s"
    drive.run(1.0)
    isolated = drive.outputs(RobotIsolated)
    assert [i.robot for _, i in isolated] == ["r1"]
    assert isolated[0][0] - unresolved_at >= wait - 1e-6
    assert drive.events("ROBOT_ISOLATED")[0][1].robot == "r1"
    failed = [r for _, r in drive.outputs(DispatchResult) if r.goal_id == 1]
    assert failed and not failed[0].success, "r1's dispatch is handed back"
    assert drive.core.zones.held_by("r1"), "the isolated robot keeps the zone it stands in"

    done = lambda: any(r.goal_id == 2 for _, r in drive.outputs(DispatchResult))  # noqa: E731
    assert drive.run(60.0, until=done), "r2 goes around r1 to WORK"
    assert [r.success for _, r in drive.outputs(DispatchResult) if r.goal_id == 2] == [True]
    assert len(drive.events("DEADLOCK_UNRESOLVED")) == 1


def test_a_cycle_that_clears_before_the_wait_isolates_nobody():
    drive = tunnel_deadlock()
    wait = drive.core.params.deadlock_unresolved_s
    drive.run(PARAMS.deadlock_grace_s + 1.0)
    assert drive.events("DEADLOCK_UNRESOLVED")
    drive.core.on_dispatch_cancel(2)  # r2 no longer wants T1; its request goes away
    drive.run(wait + 2.0)
    assert not drive.outputs(RobotIsolated)
    assert find_cycle(wait_graph(drive.core.zones)) is None


def test_the_fleet_builds_a_cycle_itself_on_crossing_lanes():
    """FleetManager in SimWorld: two_lane_segments, whose lanes O-BW and BE-R cross (worklog
    11-06), with basic_orders, seed 3, path mode and gaussian noise. Two robots end up waiting
    for each other near OUTBOUND with nothing to give up. The third keeps getting moves while
    they wait, and one of the two is isolated deadlock_unresolved_s after DEADLOCK_UNRESOLVED.
    """
    data = yaml.safe_load(BASIC.read_text(encoding="utf-8"))
    stock = {}
    for items in data["stock"].values():
        for kind, count in items.items():
            stock[kind] = stock.get(kind, 0) + count
    world = SimWorld(seed=3, orders=[dict(o) for o in data["orders"]], stock=stock,
                     robots={n: dict(spec) for n, spec in data["robots"].items()},
                     site=load_lane_map(TWO_LANE), params=replace(PARAMS, lane_goal_mode="path"),
                     arm_s_per_item=4.0, swap_step_s=4.0, dock_s=3.0, gaussian_noise=True)
    assert world.run_until(lambda: "DEADLOCK_DETECTED" in world.event_codes(), timeout_s=200.0)
    detected_at = world.t
    cycle = set(find_cycle(wait_graph(world.core.zones)))
    [outside] = set(world.robots) - cycle

    assert world.run_until(lambda: "DEADLOCK_UNRESOLVED" in world.event_codes(),
                           timeout_s=PARAMS.deadlock_grace_s + 5.0)
    unresolved_at = world.t
    assert unresolved_at - detected_at >= PARAMS.deadlock_grace_s - 1e-6
    assert world.run_until(lambda: "ROBOT_ISOLATED" in world.event_codes(),
                           timeout_s=PARAMS.deadlock_unresolved_s + 5.0)
    isolated_at = world.t
    assert isolated_at - unresolved_at >= PARAMS.deadlock_unresolved_s - 1e-6
    [isolated] = [e.robot for e in world.events if e.code == "ROBOT_ISOLATED"]
    assert isolated in cycle
    assert world.event_codes().count("DEADLOCK_UNRESOLVED") == 1
    moved = [c for t, c in world.commands
             if detected_at < t < isolated_at and isinstance(c, MoveCmd) and c.robot == outside]
    assert moved, f"{outside} is outside the cycle {sorted(cycle)}"


def held_zone(drive, robot):
    """The zone of the waypoint the robot stands on (it holds it from the start)."""
    zone = drive.core._zone(drive.site.nearest_waypoint(*drive.pose[robot]))
    assert drive.core.zones.holder(zone) == robot
    return zone


def standing(starts):
    """Robots on the map that get no new moves (PAUSE), so only the forced requests count."""
    drive = Drive(starts)
    drive.core.on_run_mode("PAUSE")
    drive.run(DT)
    assert not drive.moves("r1") and not drive.moves("r2")
    return drive


def force_waits(drive, waits):
    """Each robot waits for the zone another one stands in."""
    zones = drive.core.zones
    for robot in waits:
        zones.cancel_all_requests(robot)
    for robot, other in waits.items():
        assert not zones.request(robot, held_zone(drive, other))


def test_every_robot_of_every_cycle_is_found():
    """A second cycle, or one sharing a robot with the first, is not missed (Codex eca660e)."""
    two = {"a": {"b"}, "b": {"a"}, "c": {"d"}, "d": {"c"}, "e": {"a"}}
    assert cycle_members(two) == {"a", "b", "c", "d"}
    eight = {"a": {"b", "c"}, "b": {"a"}, "c": {"a"}}
    assert cycle_members(eight) == {"a", "b", "c"}
    assert cycle_members({"a": {"b"}, "b": {"c"}}) == frozenset()


def test_two_cycles_at_once_stop_all_their_robots():
    drive = standing({"r1": "T1", "r2": "T2", "r3": "E1", "r4": "F2"})
    force_waits(drive, {"r1": "r2", "r2": "r1", "r3": "r4", "r4": "r3"})
    assert drive.core.tick_progress(drive.now[0]) == {"r1", "r2", "r3", "r4"}


def test_the_same_robots_in_another_cycle_are_a_new_episode():
    """r1->r2->r3->r1 becomes r1->r3->r2->r1: the new cycle gets its own grace and retreat."""
    drive = standing({"r1": "T1", "r2": "T2", "r3": "E1"})
    force_waits(drive, {"r1": "r2", "r2": "r3", "r3": "r1"})
    drive.run(PARAMS.deadlock_grace_s + 1.0)
    assert len(drive.events("DEADLOCK_DETECTED")) == 1
    assert len(drive.events("DEADLOCK_UNRESOLVED")) == 1
    force_waits(drive, {"r1": "r3", "r3": "r2", "r2": "r1"})
    drive.run(DT)
    assert len(drive.events("DEADLOCK_DETECTED")) == 2
    drive.run(PARAMS.deadlock_grace_s + 1.0)
    assert len(drive.events("DEADLOCK_UNRESOLVED")) == 2
    assert not drive.outputs(RobotIsolated), "the new cycle waits its own deadlock_unresolved_s"


def test_a_parked_robot_stopped_in_a_cycle_is_not_overtaken():
    """The parking queue runs one way: P-3 is ahead of P-2. A front robot that stands still in a
    wait cycle cannot leave, so the robot behind it must not take the open call."""
    drive = Drive({"r1": "P-3", "r2": "P-2"})
    drive.run(DT)
    drive.dispatch(9, "WORK", "")
    drive.core.tick_pick(frozenset({"r1"}))
    assert not drive.core._missions["r1"].called
    assert not drive.core._missions["r2"].called
