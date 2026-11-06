"""ScenarioJudge: decides whether a scenario run passed. No ROS imports.

The zone message has one holder per zone, so a double booking cannot show up there. The judge
therefore checks physically: where each robot really is, and what the holder was. A fake robot
publishes its noiseless position for this; a single report can be several sigma off and fail a
good run, and it goes silent during a comm loss. A real robot is judged on its reports.

Orders are judged from the event codes in the common design document: the central system's
final reports and the fleet manager's ORDER_BOUND and ORDER_REQUEUED.
"""
from __future__ import annotations

import re

RATE_PERIOD_S = 0.5  # robots.csv is downsampled to 2 Hz
EPS = 1e-9
PLACING = "PLACING"
FINAL_CODES = {"ORDER_SHIPPED": "SHIPPED", "ORDER_PARTIAL": "PARTIAL", "ORDER_HELD": "HELD"}
ORDER_ID = re.compile(r"[^\s:,;]+")


def order_id_of(detail: str) -> str:
    """Details of the order events start with the order ID."""
    match = ORDER_ID.match(detail.strip())
    return match.group(0) if match else ""


def _out_of_order(sequence: list[str], rank: dict[str, int]) -> list[str]:
    """Entries that came after a later-ranked one, or that have no rank at all."""
    wrong = []
    highest = -1
    for item in sequence:
        position = rank.get(item)
        if position is None or position < highest:
            wrong.append(item)
        else:
            highest = position
    return wrong


class ScenarioJudge:
    def __init__(self, lane_map, scenario, margin_m: float = 0.13, grace_s: float = 2.0,
                 radius_m: float = 0.08):
        self.site = lane_map
        self.scenario = scenario
        self.margin_m = margin_m
        self.radius_m = radius_m
        self.grace_s = grace_s
        self.poses: dict[str, tuple[float, float, str]] = {}
        self.truth: dict[str, tuple[float, float]] = {}
        self.holders: dict[str, str] = {}
        self.events_seen: set[str] = set()
        self.event_counts: dict[str, int] = {}
        self.violations: list[str] = []
        self.max_occupancy = 0
        self.release_violations = 0
        self.unheld_violations = 0
        self.robot_rows: list[tuple] = []
        self.zone_rows: list[tuple] = []
        self.event_rows: list[tuple] = []
        self.release_rows: list[tuple] = []
        self._last_logged: dict[str, float] = {}
        self._unheld_since: dict[tuple[str, str], float] = {}
        self._unheld_counted: set[tuple[str, str]] = set()
        self._over: set[tuple[str, ...]] = set()
        self._left_start: set[str] = set()
        # Orders
        self.registered: list[str] = list(scenario.order_ids)
        self.final: dict[str, str] = {}
        self.final_counts = {"SHIPPED": 0, "PARTIAL": 0, "HELD": 0}
        self.bound_log: list[str] = []
        self.ship_log: list[str] = []
        self.order_log: list[tuple[str, str]] = []  # BOUND and REQUEUED, in order of arrival
        self.error_codes: list[str] = []
        self.board: list[str] = []
        self._bound_now: set[str] = set()
        # Arms
        fill_arms = [a for s in lane_map.stations.values() if s.action == "fill" for a in s.arms]
        self._placing: dict[str, list[tuple[float, float]]] = {a: [] for a in fill_arms}
        self._placing_open: dict[str, float | None] = dict.fromkeys(fill_arms)
        self._placing_seen: set[str] = set()
        self._arm_stamp: dict[str, float] = {}
        # Laps
        self._work_zones = self._station_zones("fill")
        self._out_zones = self._station_zones("swap")
        self._waiting_zones = {lane_map.zone_of[c] for c in lane_map.parking}
        self._last_zone: dict[str, str] = {}
        self._from_outbound: set[str] = set()
        self.direct_returns = 0
        self.parked_while_backlog = 0

    def _station_zones(self, action: str) -> set[str]:
        return {self.site.zone_of[s.dock] for s in self.site.stations.values()
                if s.action == action}

    # -- inputs -------------------------------------------------------------------------

    def on_robot_state(self, t: float, robot: str, x: float, y: float, state: str) -> None:
        self.poses[robot] = (x, y, state)
        if t - self._last_logged.get(robot, -1e9) >= RATE_PERIOD_S - EPS:
            self._last_logged[robot] = t
            self.robot_rows.append((t, robot, x, y, state))

    def on_true_pose(self, robot: str, x: float, y: float) -> None:
        self.truth[robot] = (x, y)

    def on_zones(self, t: float, holders: dict[str, str]) -> None:
        for zone, new in holders.items():
            old = self.holders.get(zone, "")
            if new == old:
                continue
            self.holders[zone] = new
            self.zone_rows.append((t, zone, old, new))
            if old:
                self._check_release(t, zone, old)

    def on_event(self, t: float, level: str, source: str, code: str, detail: str) -> None:
        self.events_seen.add(code)
        self.event_counts[code] = self.event_counts.get(code, 0) + 1
        self.event_rows.append((t, level, source, code, detail))
        if level == "ERROR":
            self.error_codes.append(code)
        order = order_id_of(detail)
        if code == "ORDER_ACCEPTED" and order not in self.registered:
            self.registered.append(order)  # a scenario order is registered from the start
        elif code == "ORDER_BOUND":
            self.bound_log.append(order)
            self.order_log.append(("BOUND", order))
            self._bound_now.add(order)
        elif code == "ORDER_REQUEUED":
            self.order_log.append(("REQUEUED", order))
            self._bound_now.discard(order)
        elif code in FINAL_CODES:
            self.final[order] = FINAL_CODES[code]
            self.final_counts[FINAL_CODES[code]] += 1
            if code != "ORDER_HELD":
                self.ship_log.append(order)

    def note(self, t: float, source: str, code: str, detail: str) -> None:
        """Log something for events.csv that must not count as a fleet event."""
        self.event_rows.append((t, "NOTE", source, code, detail))

    def on_board(self, order_ids: list[str]) -> None:
        self.board = list(order_ids)

    def on_arm_state(self, stamp: float, arm: str, state: str) -> None:
        """Takes the arm's own stamp: two topics can deliver out of order against each other."""
        if arm not in self._placing or stamp < self._arm_stamp.get(arm, -1e18):
            return
        self._arm_stamp[arm] = stamp
        if state == PLACING:
            self._placing_seen.add(arm)
        since = self._placing_open[arm]
        if state == PLACING and since is None:
            self._placing_open[arm] = stamp
        elif state != PLACING and since is not None:
            self._placing[arm].append((since, stamp))
            self._placing_open[arm] = None

    # -- checks -------------------------------------------------------------------------

    def _zone_of_pose(self, x: float, y: float) -> str:
        return self.site.zone_of[self.site.nearest_waypoint(x, y)]

    def _position(self, robot: str) -> tuple[float, float] | None:
        if robot in self.truth:
            return self.truth[robot]
        pose = self.poses.get(robot)
        return (pose[0], pose[1]) if pose else None

    def _positions(self) -> dict[str, tuple[float, float]]:
        return {robot: self._position(robot) for robot in {*self.poses, *self.truth}}

    def off_course(self, robot: str) -> bool:
        """True when the robot is known to be clear of every zone."""
        position = self._position(robot)
        if position is None:
            return False
        zone = self._zone_of_pose(*position)
        return self.site.distance_to_zone(position[0], position[1], zone) > self.margin_m

    def held_by(self, robot: str) -> list[str]:
        return sorted(z for z, holder in self.holders.items() if holder == robot)

    def _check_release(self, t: float, zone: str, robot: str) -> None:
        position = self._position(robot)
        if position is None:
            self._violate(t, f"{zone} released from {robot} with no known pose")
            self.release_violations += 1
            return
        cfg = self.scenario.robots.get(robot)
        noise = cfg.pose_noise_m if cfg else 0.0
        dist = self.site.distance_to_zone(position[0], position[1], zone)
        report = self.poses.get(robot)
        reported = self.site.distance_to_zone(report[0], report[1], zone) if report else ""
        self.release_rows.append((t, zone, robot, dist, reported))
        # The manager decides on noisy reports and its extra margin exists to absorb that noise,
        # so the true position only has to show the robot's body clear of the zone.
        limit = self.radius_m if robot in self.truth else max(0.0, self.margin_m - 2 * noise)
        if dist <= limit:
            self._violate(t, f"{zone} released from {robot} while {dist:.2f} m from it")
            self.release_violations += 1

    def _violate(self, t: float, text: str) -> None:
        self.violations.append(f"t={t:.1f} {text}")

    def tick(self, t: float) -> None:
        by_zone: dict[str, list[str]] = {}
        for robot, (x, y) in self._positions().items():
            zone = self._zone_of_pose(x, y)
            if self.site.distance_to_zone(x, y, zone) <= EPS:
                by_zone.setdefault(zone, []).append(robot)
                self._track_lap(t, robot, zone)
        for zone, robots in by_zone.items():
            self.max_occupancy = max(self.max_occupancy, len(robots))
            key = (zone, *sorted(robots))
            if len(robots) > 1 and key not in self._over:
                self._over.add(key)
                self._violate(t, f"{', '.join(sorted(robots))} are in {zone} at the same time")
        self._check_unheld(t)

    def _unbound_on_board(self) -> int:
        return sum(order not in self._bound_now for order in self.board)

    def _track_lap(self, t: float, robot: str, zone: str) -> None:
        """Follow a robot from the outbound zone to whichever it reaches first of the other two.

        With at least as many unbound orders as robots every robot is called, so none of them
        has a reason to enter the waiting loop.
        """
        if self._last_zone.get(robot) == zone:
            return
        self._last_zone[robot] = zone
        if zone in self._out_zones:
            self._from_outbound.add(robot)
        elif robot not in self._from_outbound:
            return
        elif zone in self._work_zones:
            self._from_outbound.discard(robot)
            self.direct_returns += 1
        elif zone in self._waiting_zones:
            self._from_outbound.discard(robot)
            unbound = self._unbound_on_board()
            if unbound >= len(self.scenario.robots):
                self.parked_while_backlog += 1
                self._violate(t, f"{robot} entered {zone} with {unbound} unbound orders waiting")

    def _still_on_start_cell(self, robot: str, x: float, y: float) -> bool:
        """The manager takes a robot's first zone only once it has heard from every robot, and
        at speed_factor 5 that can outlast the grace time. Nothing has driven the robot yet."""
        cfg = self.scenario.robots.get(robot)
        if robot in self._left_start or cfg is None:
            return False
        if self.site.nearest_waypoint(x, y) == cfg.start:
            return True
        self._left_start.add(robot)
        return False

    def _check_unheld(self, t: float) -> None:
        current = set()
        for robot, (x, y) in self._positions().items():
            if self._still_on_start_cell(robot, x, y):
                continue
            zone = self._zone_of_pose(x, y)
            if self.site.distance_to_zone(x, y, zone) > EPS or self.holders.get(zone) == robot:
                continue
            key = (robot, zone)
            current.add(key)
            since = self._unheld_since.setdefault(key, t)
            if t - since >= self.grace_s and key not in self._unheld_counted:
                self._unheld_counted.add(key)
                self.unheld_violations += 1
                self._violate(t, f"{robot} is in {zone} but the holder is "
                                 f"{self.holders.get(zone) or 'nobody'}")
        for key in list(self._unheld_since):
            if key not in current:
                del self._unheld_since[key]
                self._unheld_counted.discard(key)

    def fifo_problems(self) -> list[str]:
        """Bound in registration order and shipped in bound order. A requeued order goes back to
        the front: it must be the next one bound. Nothing is bound twice without a requeue."""
        rank = {o: i for i, o in enumerate(self.registered)}
        problems = []
        open_bound: set[str] = set()  # bound and not requeued since, final or not
        requeued: set[str] = set()  # waiting to be bound again
        seen: set[str] = set()
        first_binds = []
        last_bind: dict[str, int] = {}
        for index, (kind, order) in enumerate(self.order_log):
            if kind == "REQUEUED":
                open_bound.discard(order)
                requeued.add(order)
                continue
            if order in open_bound:
                problems.append(f"{order} bound again without a requeue")
            elif requeued and order != min(requeued, key=lambda o: rank.get(o, len(rank))):
                problems.append(f"{order} bound ahead of requeued {sorted(requeued)}")
            open_bound.add(order)
            requeued.discard(order)
            last_bind[order] = index
            if order not in seen:
                seen.add(order)
                first_binds.append(order)
        problems += [f"{o} bound out of registration order"
                     for o in _out_of_order(first_binds, rank)]
        problems += [f"{o} shipped out of bound order"
                     for o in _out_of_order(self.ship_log, last_bind)]
        return problems

    def arms_never_placing(self) -> list[str]:
        """Arms the shipped orders needed that never showed PLACING. Without their states the
        overlap check would pass on no data. Only meaningful when two arms were needed."""
        stock = self.scenario.stock
        items = {o.id: o.items for o in self.scenario.orders}
        needed = {arm for order in self.ship_log for kind in items.get(order, {})
                  for arm in self._placing if stock.get(arm, {}).get(kind, 0) > 0}
        return sorted(needed - self._placing_seen) if len(needed) > 1 else []

    def dual_arm_overlap_list(self) -> list[tuple[float, float]]:
        """Times at which two workspace arms were PLACING at once. Touching does not count."""
        latest = max(self._arm_stamp.values(), default=0.0)
        spans = {arm: [*done, *([(self._placing_open[arm], latest)]
                                if self._placing_open[arm] is not None else [])]
                 for arm, done in self._placing.items()}
        arms = sorted(spans)
        out = []
        for i, a in enumerate(arms):
            for b in arms[i + 1:]:
                for start_a, end_a in spans[a]:
                    for start_b, end_b in spans[b]:
                        if start_a < end_b and start_b < end_a:
                            out.append((max(start_a, start_b), min(end_a, end_b)))
        return out

    # -- outcome ------------------------------------------------------------------------

    @property
    def all_final(self) -> bool:
        return all(order in self.final for order in self.registered)

    def _all_parked(self) -> bool:
        for robot in self.scenario.robots:
            if robot in self.scenario.gone_robots:
                continue
            pose = self.poses.get(robot)
            if pose is None or pose[2] != "IDLE":
                return False
            if self._zone_of_pose(*self._position(robot)) not in self._waiting_zones:
                return False
        return True

    def _complete(self) -> bool:
        return self.all_final and self._all_parked()

    def finished(self, t: float) -> bool:
        return t >= self.scenario.timeout_s or self._complete()

    def summary(self, t: float) -> dict:
        ex = self.scenario.expect
        counts = self.final_counts
        fifo = self.fifo_problems()
        overlaps = self.dual_arm_overlap_list()
        missing = [c for c in ex.events_include if c not in self.events_seen]
        never_placing = self.arms_never_placing()
        allowed_errors = {*ex.events_include, *ex.event_counts}
        unexpected = [c for c in self.error_codes if c not in allowed_errors]
        timed_out = not self._complete()

        def exact(value, expected):
            return {"value": value, "expected": expected,
                    "ok": expected is None or value == expected}

        def at_least(value, minimum):
            return {"value": value, "minimum": minimum, "ok": value >= minimum}

        checks = {
            "all_orders_finished": {"value": self.all_final, "expected": ex.all_orders_finished,
                                    "ok": self.all_final == ex.all_orders_finished},
            "orders_shipped": exact(counts["SHIPPED"], ex.orders_shipped),
            "orders_partial": exact(counts["PARTIAL"], ex.orders_partial),
            "orders_held": exact(counts["HELD"], ex.orders_held),
            "orders_shipped_min": at_least(counts["SHIPPED"], ex.orders_shipped_min),
            "fifo_violations": exact(len(fifo), ex.fifo_violations),
            "dual_arm_overlaps": {
                "value": len(overlaps), "expected": ex.dual_arm_overlaps,
                "arms_never_placing": never_placing,
                "ok": len(overlaps) == ex.dual_arm_overlaps and not never_placing},
            "zone_holder_max": exact(self.max_occupancy, ex.zone_holder_max),
            "released_without_position_check": exact(self.release_violations,
                                                     ex.released_without_position_check),
            "unheld_zone_pose": exact(self.unheld_violations, ex.unheld_zone_pose),
            "direct_returns_min": at_least(self.direct_returns, ex.direct_returns_min),
            "parked_while_backlog": exact(self.parked_while_backlog, ex.parked_while_backlog),
            "timed_out": exact(timed_out, ex.timed_out),
            "unexpected_errors": {"value": len(unexpected), "expected": ex.unexpected_errors,
                                  "codes": unexpected,
                                  "ok": len(unexpected) == ex.unexpected_errors},
            "order_results": {
                "value": {o: self.final.get(o) for o in ex.order_results},
                "expected": ex.order_results,
                "ok": all(self.final.get(o) == r for o, r in ex.order_results.items())},
            "events_include": {"missing": missing, "ok": not missing},
            "event_counts": {
                "value": {c: self.event_counts.get(c, 0) for c in ex.event_counts},
                "expected": ex.event_counts,
                "ok": all(self.event_counts.get(c, 0) == n for c, n in ex.event_counts.items())},
        }
        return {
            "pass": all(c["ok"] for c in checks.values()),
            "orders_total": len(self.registered),
            "orders_shipped": counts["SHIPPED"],
            "orders_partial": counts["PARTIAL"],
            "orders_held": counts["HELD"],
            "duration_s": float(t),
            "timed_out": timed_out,
            "violations": [*self.violations, *fifo,
                           *(f"unexpected ERROR event {c}" for c in unexpected),
                           *(f"{a} never showed PLACING" for a in never_placing),
                           *(f"arm1 and arm2 both PLACING at {start:.2f}-{end:.2f}"
                             for start, end in overlaps)],
            "min_release_distance_m": min((r[3] for r in self.release_rows), default=None),
            "checks": checks,
        }
