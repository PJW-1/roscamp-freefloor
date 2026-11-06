"""가짜 로봇용 시나리오 파일: 읽기, 기본값, 검증. ROS를 쓰지 않는다."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

ORDER_RESULTS = ("SHIPPED", "PARTIAL", "HELD")
ROBOT_FAULTS = ("comm_loss", "nav_abort", "stuck", "operator_remove", "blocked")
ARM_FAULTS = ("arm_fail", "swap_partial")
FAULT_TYPES = ROBOT_FAULTS + ARM_FAULTS
# 이 고장이 난 로봇은 돌아오지 않으므로 주차를 기다리지 않는다.
GONE_FAULTS = ("stuck", "operator_remove")
EXPECT_DEFAULTS = {
    "all_orders_finished": True,
    "orders_shipped": None,  # None은 검사 안 함, 숫자는 정확히 일치해야 함
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
    "timed_out": False,  # 시간 초과가 목적인 시나리오에서만 true
    "unexpected_errors": 0,  # 코드가 events_include나 event_counts에 없는 ERROR 이벤트
    "order_results": {},  # 주문 id: SHIPPED, PARTIAL, HELD
    "events_include": [],
    "event_counts": {},  # 코드: 정확히 몇 번 나와야 하는지
}
TOP_LEVEL_KEYS = {"seed", "speed_factor", "timeout_s", "stock", "inventory", "orders", "robots",
                  "arm", "dock", "faults", "expect"}


class ScenarioError(ValueError):
    """시나리오 파일이 잘못됐다."""


@dataclass(frozen=True)
class RobotCfg:
    start: tuple[int, int]
    speed_mps: float = 0.2
    pose_noise_m: float = 0.0
    report_delay_s: float = 0.0  # 측정하고 이만큼 지난 뒤에 보고가 나간다
    stop_error_m: float = 0.0  # 축마다 표준편차: 이동이 목표 주변 어디서 끝나는지
    localizing_s: float = 0.0  # 시작하고 이만큼 LOCALIZING으로 보고한다(계약 3절 6번)


@dataclass(frozen=True)
class ArmCfg:
    seconds_per_item: float = 4.0
    swap_seconds: float = 8.0
    fail_rate: float = 0.0
    cancel_seconds: float = 0.0  # 가짜 작업셀: 취소부터 결과까지 걸리는 시간(서버는 바쁨)


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
    robot: str = ""  # 로봇 고장의 대상
    type: str = ""
    duration_s: float = 0.0
    count: int = 1
    arm: str = ""  # arm_fail의 대상
    after_items: int = 0  # arm_fail: 한 목표에서 몇 개를 놓은 뒤 실패하는지


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
    orders: list  # 등록 순서
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
        # 변환한 격자 지도의 지점을 옛 [열, 행]으로 부른 것.
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
    return sorted(orders, key=lambda o: o.at_s)  # 안정 정렬: at_s가 같으면 파일 순서 유지


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
    if kind == "blocked":
        _positive(raw, "duration_s")  # 물체가 있는 시간
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
            localizing_s=_not_negative(raw, "localizing_s"),
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
