"""Scenario files for the fake robots: parse, default and validate. No ROS imports."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

ORDER_RESULTS = ("SHIPPED", "PARTIAL", "HELD")
ROBOT_FAULTS = ("comm_loss", "nav_abort", "stuck", "operator_remove")
ARM_FAULTS = ("arm_fail", "swap_partial")
FAULT_TYPES = ROBOT_FAULTS + ARM_FAULTS
# A robot with one of these never comes back, so nobody waits for it to park.
GONE_FAULTS = ("stuck", "operator_remove")
EXPECT_DEFAULTS = {
    "all_orders_finished": True,
    "orders_shipped": None,  # None means not checked; a number has to match exactly
    "orders_partial": None,
    "orders_held": None,
    "orders_shipped_min": 0,
    "fifo_violations": 0,
    "dual_arm_overlaps": 0,
    "zone_holder_max": 1,
    "released_without_position_check": 0,
    "unheld_zone_pose": 0,
    "direct_returns_min": 0,
    "parked_while_backlog": 0,
    "timed_out": False,  # true only where the scenario is meant to run out of time
    "unexpected_errors": 0,  # ERROR events whose code is not in events_include or event_counts
    "order_results": {},  # order id: SHIPPED, PARTIAL or HELD
    "events_include": [],
    "event_counts": {},  # code: how many times it has to appear, exactly
}
TOP_LEVEL_KEYS = {"seed", "speed_factor", "timeout_s", "stock", "inventory", "orders", "robots",
                  "arm", "dock", "faults", "expect"}


class ScenarioError(ValueError):
    """The scenario file is invalid."""


@dataclass(frozen=True)
class RobotCfg:
    start: tuple[int, int]
    speed_mps: float = 0.2
    pose_noise_m: float = 0.0
    report_delay_s: float = 0.0  # a report goes out this long after it was measured
    stop_error_m: float = 0.0  # one sigma per axis: where a move really ends around its target


@dataclass(frozen=True)
class ArmCfg:
    seconds_per_item: float = 4.0
    swap_seconds: float = 8.0
    fail_rate: float = 0.0
    cancel_seconds: float = 0.0  # fake workcell: time from a cancel to its result, server busy


@dataclass(frozen=True)
class DockCfg:
    duration_s: float = 3.0
    error_cm: float = 1.0


@dataclass(frozen=True)
class OrderCfg:
    at_s: float
    id: str
    items: dict


@dataclass(frozen=True)
class Fault:
    at_s: float
    robot: str = ""  # target of the robot faults
    type: str = ""
    duration_s: float = 0.0
    count: int = 1
    arm: str = ""  # target of arm_fail
    after_items: int = 0  # arm_fail: how many items the arm places in a goal before it fails


@dataclass(frozen=True)
class Expect:
    all_orders_finished: bool = True
    orders_shipped: int | None = None
    orders_partial: int | None = None
    orders_held: int | None = None
    orders_shipped_min: int = 0
    fifo_violations: int = 0
    dual_arm_overlaps: int = 0
    zone_holder_max: int = 1
    released_without_position_check: int = 0
    unheld_zone_pose: int = 0
    direct_returns_min: int = 0
    parked_while_backlog: int = 0
    timed_out: bool = False
    unexpected_errors: int = 0
    order_results: dict = field(default_factory=dict)
    events_include: list = field(default_factory=list)
    event_counts: dict = field(default_factory=dict)


@dataclass(frozen=True)
class Scenario:
    seed: int
    speed_factor: float
    timeout_s: float
    stock: dict
    inventory: dict
    orders: list  # in registration order
    robots: dict
    arm: ArmCfg
    dock: DockCfg
    faults: list
    expect: Expect

    @property
    def order_ids(self) -> list[str]:
        return [o.id for o in self.orders]

    @property
    def gone_robots(self) -> set[str]:
        return {f.robot for f in self.faults if f.type in GONE_FAULTS}


def _positive(data: dict, key: str, default=None) -> float:
    value = data.get(key, default)
    if not isinstance(value, (int, float)) or isinstance(value, bool) or value <= 0:
        raise ScenarioError(f"{key} must be a positive number, got {value!r}")
    return float(value)


def _not_negative(data: dict, key: str) -> float:
    value = data.get(key, 0.0)
    if not isinstance(value, (int, float)) or isinstance(value, bool) or value < 0:
        raise ScenarioError(f"{key} must not be negative, got {value!r}")
    return float(value)


def _resolve_start(raw, lane_map) -> tuple[int, int]:
    if isinstance(raw, str):
        waypoints = lane_map.zones.get(raw)
        if waypoints is None or len(waypoints) != 1:
            raise ScenarioError(f"start {raw!r} is not a zone of one waypoint")
        return next(iter(waypoints))
    if isinstance(raw, (list, tuple)) and len(raw) == 2:
        # A waypoint of a converted grid map, named by its old [col, row].
        waypoint = (int(raw[0]), int(raw[1]))
        if waypoint not in lane_map.zone_of:
            raise ScenarioError(f"start waypoint {waypoint} is on no route")
        return waypoint
    raise ScenarioError(f"start must be a zone name or [col, row], got {raw!r}")


def _arms_with(lane_map, action: str) -> list[str]:
    return [arm for s in lane_map.stations.values() if s.action == action for arm in s.arms]


def _counts(raw, what: str, minimum: int) -> dict:
    if not isinstance(raw, dict):
        raise ScenarioError(f"{what} must map item kinds to counts, got {raw!r}")
    out = {}
    for kind, count in raw.items():
        if not isinstance(count, int) or isinstance(count, bool) or count < minimum:
            raise ScenarioError(f"{what}: {kind} must be an integer of at least {minimum}, "
                                f"got {count!r}")
        out[str(kind)] = count
    return out


def _parse_stock(data: dict, lane_map) -> dict:
    fill_arms = _arms_with(lane_map, "fill")
    stock = {arm: {} for arm in fill_arms}
    for arm, items in (data.get("stock") or {}).items():
        if arm not in fill_arms:
            raise ScenarioError(f"stock for {arm}, which is not one of {fill_arms}")
        stock[arm] = _counts(items or {}, f"stock of {arm}", 0)
    return stock


def _parse_inventory(data: dict, stock: dict) -> dict:
    if data.get("inventory") is not None:
        return _counts(data["inventory"], "inventory", 0)
    total: dict[str, int] = {}
    for items in stock.values():
        for kind, count in items.items():
            total[kind] = total.get(kind, 0) + count
    return total


def _parse_orders(data: dict) -> list[OrderCfg]:
    orders = []
    for raw in data.get("orders") or []:
        if not raw.get("id") or not raw.get("items"):
            raise ScenarioError(f"order needs an id and items: {raw}")
        at_s = raw.get("at_s", 0)
        if not isinstance(at_s, (int, float)) or at_s < 0:
            raise ScenarioError(f"bad at_s in order {raw}")
        orders.append(OrderCfg(float(at_s), str(raw["id"]),
                               _counts(raw["items"], f"order {raw['id']}", 1)))
    ids = [o.id for o in orders]
    if len(ids) != len(set(ids)):
        raise ScenarioError(f"duplicate order ids in {ids}")
    return sorted(orders, key=lambda o: o.at_s)  # stable: same at_s keeps the file order


def _parse_fault(raw: dict, robots: dict, lane_map) -> Fault:
    kind = raw.get("type")
    if kind not in FAULT_TYPES:
        raise ScenarioError(f"unknown fault type in {raw}")
    if kind in ROBOT_FAULTS and raw.get("robot") not in robots:
        raise ScenarioError(f"fault for unknown robot in {raw}")
    if kind == "arm_fail" and raw.get("arm") not in _arms_with(lane_map, "fill"):
        raise ScenarioError(f"arm_fail needs a workspace arm in {raw}")
    if "at_s" not in raw:
        raise ScenarioError(f"fault has no at_s: {raw}")
    return Fault(float(raw["at_s"]), raw.get("robot", ""), kind,
                 float(raw.get("duration_s", 0.0)), int(raw.get("count", 1)),
                 raw.get("arm", ""), int(raw.get("after_items", 0)))


def _parse_expect(data: dict, faults: list) -> Expect:
    raw = data.get("expect") or {}
    unknown = set(raw) - set(EXPECT_DEFAULTS)
    if unknown:
        raise ScenarioError(f"unknown expect keys: {sorted(unknown)}")
    ids = {str((o or {}).get("id")) for o in data.get("orders") or []}
    for order, result in (raw.get("order_results") or {}).items():
        if order not in ids or result not in ORDER_RESULTS:
            raise ScenarioError(f"order_results: {order}: {result} needs a scenario order and "
                                f"one of {ORDER_RESULTS}")
    expect = Expect(**{**EXPECT_DEFAULTS, **raw})
    if faults and max(expect.orders_shipped or 0, expect.orders_shipped_min) < 1:
        raise ScenarioError("a scenario with faults must expect at least one shipped order "
                            "(orders_shipped or orders_shipped_min), so that it cannot pass "
                            "with nothing shipped")
    return expect


def parse_scenario(data: dict, lane_map) -> Scenario:
    unknown = set(data) - TOP_LEVEL_KEYS
    if unknown:
        raise ScenarioError(f"unknown scenario keys: {sorted(unknown)}")
    robots = {}
    for name, raw in (data.get("robots") or {}).items():
        raw = raw or {}
        if "start" not in raw:
            raise ScenarioError(f"robot {name} has no start")
        robots[name] = RobotCfg(
            start=_resolve_start(raw["start"], lane_map),
            speed_mps=_positive(raw, "speed_mps", 0.2),
            pose_noise_m=float(raw.get("pose_noise_m", 0.0)),
            report_delay_s=_not_negative(raw, "report_delay_s"),
            stop_error_m=_not_negative(raw, "stop_error_m"),
        )
    if not robots:
        raise ScenarioError("scenario has no robots")

    stock = _parse_stock(data, lane_map)
    faults = [_parse_fault(raw, robots, lane_map) for raw in data.get("faults") or []]
    arm = data.get("arm") or {}
    dock = data.get("dock") or {}
    return Scenario(
        seed=int(data.get("seed", 0)),
        speed_factor=_positive(data, "speed_factor", 1.0),
        timeout_s=_positive(data, "timeout_s", 900),
        stock=stock,
        inventory=_parse_inventory(data, stock),
        orders=_parse_orders(data),
        robots=robots,
        arm=ArmCfg(float(arm.get("seconds_per_item", 4.0)), float(arm.get("swap_seconds", 8.0)),
                   float(arm.get("fail_rate", 0.0)), float(arm.get("cancel_seconds", 0.0))),
        dock=DockCfg(float(dock.get("duration_s", 3.0)), float(dock.get("error_cm", 1.0))),
        faults=faults,
        expect=_parse_expect(data, faults),
    )


def load_scenario(path, lane_map=None) -> Scenario:
    if lane_map is None:
        from fleet_core.traffic.lane_map import load_lane_map
        lane_map = load_lane_map()
    return parse_scenario(yaml.safe_load(Path(path).read_text(encoding="utf-8")), lane_map)
