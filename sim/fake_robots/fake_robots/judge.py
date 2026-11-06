"""ScenarioJudge: 시나리오 실행이 통과했는지 판정한다. ROS를 쓰지 않는다.

구역 메시지는 구역마다 보유자가 하나라 이중 예약이 거기에 드러나지 않는다. 그래서
물리적으로 검사한다: 로봇이 실제로 어디에 있는지, 보유자가 누구였는지. 가짜 로봇은 이를 위해
잡음 없는 위치를 내보낸다. 보고 한 번은 표준편차 몇 배씩 틀려 정상 실행을 떨어뜨릴 수 있고
통신 두절 중에는 끊긴다. 실제 로봇은 보고로 판정한다.

주문은 공통 설계 문서의 이벤트 코드로 판정한다: 중앙의 최종 보고와
플릿 매니저의 ORDER_BOUND, ORDER_REQUEUED.
"""
from __future__ import annotations

import re

RATE_PERIOD_S = 0.5  # robots.csv는 2 Hz로 줄여 쓴다
OFF_LANE_GRACE_S = 1.0  # drift 시작부터 FMS가 연속 보고로 알아챌 때까지의 여유
EPS = 1e-9
PLACING = "PLACING"
FINAL_CODES = {"ORDER_SHIPPED": "SHIPPED", "ORDER_PARTIAL": "PARTIAL", "ORDER_HELD": "HELD"}
ORDER_ID = re.compile(r"[^\s:,;]+")


def order_id_of(detail: str) -> str:
    """주문 이벤트의 detail은 주문 ID로 시작한다."""
    match = ORDER_ID.match(detail.strip())
    return match.group(0) if match else ""


def _out_of_order(sequence: list[str], rank: dict[str, int]) -> list[str]:
    """순위가 더 뒤인 항목 다음에 온 항목, 또는 순위가 아예 없는 항목."""
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
        self.off_lane_releases = 0
        # 로봇 -> 보고가 레인 밖으로 밀린 시간 구간들
        self._drifts: dict[str, list[tuple[float, float]]] = {}
        for f in scenario.faults:
            if f.type == "drift":
                self._drifts.setdefault(f.robot, []).append(
                    (f.at_s + OFF_LANE_GRACE_S, f.at_s + f.duration_s))
        self.robot_rows: list[tuple] = []
        self.zone_rows: list[tuple] = []
        self.event_rows: list[tuple] = []
        self.release_rows: list[tuple] = []
        self._last_logged: dict[str, float] = {}
        self._unheld_since: dict[tuple[str, str], float] = {}
        self._unheld_counted: set[tuple[str, str]] = set()
        self._over: set[tuple[str, ...]] = set()
        self._left_start: set[str] = set()
        self._put_back: dict[str, tuple] = {}  # 다시 투입한 로봇 -> (놓인 지점, 면제 끝 시각)
        self._reinstated: set[str] = set()
        self.bound_after_reinstate = 0
        self._bound_by_reinstated: set[str] = set()  # 재투입 뒤 그 로봇이 묶어 아직 쥔 주문
        self.shipped_after_reinstate = 0
        self._back_now: set[str] = set()  # 재투입 뒤 다시 치우지 않은 로봇
        # 주문
        self.registered: list[str] = list(scenario.order_ids)
        self.final: dict[str, str] = {}
        self.final_counts = {"SHIPPED": 0, "PARTIAL": 0, "HELD": 0}
        self.bound_log: list[str] = []
        self.ship_log: list[str] = []
        self.order_log: list[tuple[str, str]] = []  # BOUND와 REQUEUED, 도착 순
        self.error_codes: list[str] = []
        self.board: list[str] = []
        self._bound_now: set[str] = set()
        # 팔
        fill_arms = [a for s in lane_map.stations.values() if s.action == "fill" for a in s.arms]
        self._placing: dict[str, list[tuple[float, float]]] = {a: [] for a in fill_arms}
        self._placing_open: dict[str, float | None] = dict.fromkeys(fill_arms)
        self._placing_seen: set[str] = set()
        self._arm_stamp: dict[str, float] = {}
        # 바퀴
        self._work_zones = self._station_zones("fill")
        self._out_zones = self._station_zones("swap")
        self._waiting_zones = {lane_map.zone_of[c] for c in lane_map.parking}
        self._last_zone: dict[str, str] = {}
        self._from_outbound: set[str] = set()
        self.direct_returns = 0
        self.parked_while_backlog = 0
        # 자동 격리: 첫 ROBOT_ISOLATED부터 격리된 로봇의 첫 ZONES_CLEARED 전까지 출고한 수
        self.shipped_before_operator = 0
        self._isolated: set[str] = set()  # source가 로봇이다
        self._operator_came = False
        # 격리된 로봇(치우기 전)이 쥔 구역을 피한 DETOUR 수. detail의 'held by <로봇>'으로 본다
        self.detours_around_isolated = 0
        self._standing_isolated: set[str] = set()

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
        if code == "ROBOT_ISOLATED":
            self._isolated.add(source)
            self._standing_isolated.add(source)
        elif code == "ZONES_CLEARED" and source in self._isolated:
            self._operator_came = True
            self._standing_isolated.discard(source)
        elif code == "DETOUR":
            holders = set(re.findall(r"held by ([^,;\s]+)", detail))
            if holders & self._standing_isolated:  # 이벤트 하나는 한 번
                self.detours_around_isolated += 1
        order = order_id_of(detail)
        if code == "ORDER_ACCEPTED" and order not in self.registered:
            self.registered.append(order)  # 시나리오 주문은 처음부터 등록돼 있다
        elif code == "ROBOT_REINSTATED":
            self._reinstated.add(source)
            self._back_now.add(source)
        elif code == "ZONES_CLEARED":
            self._back_now.discard(source)
        elif code == "ORDER_BOUND":
            if source in self._reinstated:
                self.bound_after_reinstate += 1
            if source in self._back_now:
                self._bound_by_reinstated.add(order)
            else:
                self._bound_by_reinstated.discard(order)
            self.bound_log.append(order)
            self.order_log.append(("BOUND", order))
            self._bound_now.add(order)
        elif code == "ORDER_REQUEUED":
            self.order_log.append(("REQUEUED", order))
            self._bound_now.discard(order)
            self._bound_by_reinstated.discard(order)
        elif code in FINAL_CODES:
            self.final[order] = FINAL_CODES[code]
            mine = order in self._bound_by_reinstated
            self._bound_by_reinstated.discard(order)  # 최종 이벤트는 한 번만 센다
            self.final_counts[FINAL_CODES[code]] += 1
            if code != "ORDER_HELD":
                self.ship_log.append(order)
                if mine:
                    self.shipped_after_reinstate += 1
                if self._isolated and not self._operator_came:
                    self.shipped_before_operator += 1

    def note(self, t: float, source: str, code: str, detail: str) -> None:
        """events.csv에 남기되 플릿 이벤트로 세면 안 되는 것을 기록한다."""
        self.event_rows.append((t, "NOTE", source, code, detail))

    def put_back(self, t: float, robot: str, waypoint) -> None:
        """관리자가 로봇을 이 지점에 다시 놓았다. 매니저가 잡을 grace_s 동안만 면제한다.

        시작 칸 면제는 더 쓰지 않는다. 못 잡으면 유예가 끝나는 시각에 바로 센다.
        """
        self._put_back[robot] = (waypoint, t, t + self.grace_s)
        self._left_start.add(robot)

    def _put_back_spot(self, robot: str, x: float, y: float):
        """놓인 지점에 아직 서 있으면 (지점, 놓은 시각, 유예 끝), 떠났으면 지우고 None."""
        spot = self._put_back.get(robot)
        if spot is not None and self.site.nearest_waypoint(x, y) != spot[0]:
            del self._put_back[robot]
            return None
        return spot

    def on_board(self, order_ids: list[str]) -> None:
        self.board = list(order_ids)

    def on_arm_state(self, stamp: float, arm: str, state: str) -> None:
        """팔 자신의 스탬프를 쓴다. 두 토픽은 서로 순서가 뒤바뀌어 올 수 있다."""
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
        """로봇이 모든 구역에서 벗어난 것이 확인되면 True."""
        position = self._position(robot)
        if position is None:
            return False
        zone = self._zone_of_pose(*position)
        return self.site.distance_to_zone(position[0], position[1], zone) > self.margin_m

    def held_by(self, robot: str) -> list[str]:
        return sorted(z for z, holder in self.holders.items() if holder == robot)

    def _check_release(self, t: float, zone: str, robot: str) -> None:
        self._check_off_lane_release(t, zone, robot)
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
        # 매니저는 잡음 낀 보고로 판단하고 여유 거리는 그 잡음을 흡수하려는 것이다.
        # 그래서 실제 위치는 몸이 구역을 벗어났음만 보이면 된다.
        limit = self.radius_m if robot in self.truth else max(0.0, self.margin_m - 2 * noise)
        if dist <= limit:
            self._violate(t, f"{zone} released from {robot} while {dist:.2f} m from it")
            self.release_violations += 1

    def _check_off_lane_release(self, t: float, zone: str, robot: str) -> None:
        """보고가 레인 밖인 동안에는 실제 위치를 모르므로 어떤 구역도 놓으면 안 된다."""
        if any(start <= t < end for start, end in self._drifts.get(robot, ())):
            self.off_lane_releases += 1
            self._violate(t, f"{zone} released from {robot} while its reports were off the lane")

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
        """출고 구역에서 나머지 둘 중 먼저 닿는 곳까지 로봇을 따라간다.

        미배정 주문이 로봇 수 이상이면 모든 로봇이 호출되므로
        대기 루프에 들어갈 까닭이 없다.
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
        """매니저는 모든 로봇의 보고를 들은 뒤에야 첫 구역을 잡는데, speed_factor 5에서는
        유예 시간을 넘길 수 있다. 아직 로봇을 움직인 것은 없다."""
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
            spot = self._put_back_spot(robot, x, y)
            if spot is None and self._still_on_start_cell(robot, x, y):
                continue
            zone = self._zone_of_pose(x, y)
            if self.holders.get(zone) == robot and spot is not None:
                del self._put_back[robot]  # 잡았다
            if self.site.distance_to_zone(x, y, zone) > EPS or self.holders.get(zone) == robot:
                continue
            if spot is not None and t < spot[2]:
                continue
            key = (robot, zone)
            current.add(key)
            # 놓인 뒤 계속 못 잡았으면 놓은 시각부터 잰다
            since = self._unheld_since.setdefault(key, spot[1] if spot is not None else t)
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
        """등록 순서로 배정하고 배정 순서로 출고한다. 재큐된 주문은 맨 앞으로 돌아가
        다음에 배정돼야 한다. 재큐 없이 두 번 배정되면 안 된다."""
        rank = {o: i for i, o in enumerate(self.registered)}
        problems = []
        open_bound: set[str] = set()  # 배정 후 재큐되지 않은 것(최종 여부 무관)
        requeued: set[str] = set()  # 다시 배정을 기다리는 것
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
        """출고된 주문에 필요했는데 PLACING을 한 번도 안 보인 팔. 그 상태가 없으면
        겹침 검사가 데이터 없이 통과한다. 팔이 둘 필요했을 때만 의미가 있다."""
        stock = self.scenario.stock
        items = {o.id: o.items for o in self.scenario.orders}
        needed = {arm for order in self.ship_log for kind in items.get(order, {})
                  for arm in self._placing if stock.get(arm, {}).get(kind, 0) > 0}
        return sorted(needed - self._placing_seen) if len(needed) > 1 else []

    def dual_arm_overlap_list(self) -> list[tuple[float, float]]:
        """작업대 팔 둘이 동시에 PLACING이던 시각. 맞닿기만 한 것은 세지 않는다."""
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
            "released_while_off_lane": exact(self.off_lane_releases, ex.released_while_off_lane),
            "direct_returns_min": at_least(self.direct_returns, ex.direct_returns_min),
            "parked_while_backlog": exact(self.parked_while_backlog, ex.parked_while_backlog),
            "shipped_before_operator": at_least(self.shipped_before_operator,
                                                ex.shipped_before_operator_min),
            "detours_around_isolated": at_least(self.detours_around_isolated,
                                                ex.detours_around_isolated_min),
            "timed_out": exact(timed_out, ex.timed_out),
            "bound_after_reinstate_min": at_least(self.bound_after_reinstate,
                                                  ex.bound_after_reinstate_min),
            "shipped_after_reinstate_min": at_least(self.shipped_after_reinstate,
                                                    ex.shipped_after_reinstate_min),
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
