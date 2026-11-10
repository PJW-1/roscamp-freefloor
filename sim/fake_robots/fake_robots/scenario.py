"""가짜 로봇용 시나리오 파일: 읽기, 기본값, 검증. ROS를 쓰지 않는다."""
from __future__ import annotations

import copy
import math
import random
from dataclasses import dataclass, field
from pathlib import Path

import yaml

ORDER_RESULTS = ("SHIPPED", "PARTIAL", "HELD")
ROBOT_FAULTS = ("comm_loss", "nav_abort", "stuck", "operator_remove", "blocked", "drift",
                "operator_reinstate", "obstacle_behind", "low_battery")
ARM_FAULTS = ("arm_fail", "swap_partial")
# 로봇에 묶이지 않는 고장. object: 지도에 없는 물체가 (x, y)에 duration_s 동안 있다(짧으면 지나가는
# 사람, 길면 놓고 간 상자). 갈 길 앞에서 그것을 본 로봇은 서고 자기 복구 뒤 BLOCKED다. 관제는 모른다
WORLD_FAULTS = ("object",)
FAULT_TYPES = ROBOT_FAULTS + ARM_FAULTS + WORLD_FAULTS
# 이 고장이 난 로봇은 돌아오지 않으므로 주차를 기다리지 않는다.
GONE_FAULTS = ("stuck", "operator_remove")
OBJECT_RADIUS_M = 0.10  # object 고장의 기본 반지름: 사람 다리나 작은 상자
OBJECT_KEYS = ("x", "y", "radius_m")
OBJECT_MAP_MARGIN_M = 1.0  # 물체는 웨이포인트 범위에서 이만큼 안에 있어야 한다(오타 막기)
REINSTATE_BASKETS = ("EMPTY", "NONE")  # operator_reinstate가 관리자 대신 알리는 바구니
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
    "parked_while_backlog": 0,  # null이면 안 봄(충전하러 가는 로봇은 주문이 밀려도 WAIT로 간다)
    "timed_out": False,  # 시간 초과가 목적인 시나리오에서만 true
    "unexpected_errors": 0,  # 코드가 events_include나 event_counts에 없는 ERROR 이벤트
    "bound_after_reinstate_min": 0,  # 다시 투입한 로봇이 그 뒤에 묶은 주문 수
    "shipped_after_reinstate_min": 0,  # 그중 되돌려지지 않고 출고까지 간 주문 수
    "order_results": {},  # 주문 id: SHIPPED, PARTIAL, HELD
    "events_include": [],
    "event_counts": {},  # 코드: 정확히 몇 번 나와야 하는지
    "shipped_before_operator_min": 0,  # 격리 뒤 confirm_clear 전 출고(지시서 5.5)
    "detours_around_isolated_min": 0,  # 격리된 로봇을 피한 DETOUR(치우기 전)
    "released_while_off_lane": 0,  # drift로 레인 밖이라 보고하는 동안 그 로봇이 놓은 구역 수
    "path_not_connected": 0,  # 관제가 처리한 PATH_NOT_CONNECTED 실패 수(지시서 7.5). null이면 안 봄
    "robot_idle": 0,  # ROBOT_IDLE 이벤트 수(1.1 지시서 H2). 정상이면 0, null이면 안 봄
}
TOP_LEVEL_KEYS = {"seed", "speed_factor", "timeout_s", "stock", "inventory", "orders", "robots",
                  "arm", "dock", "faults", "expect", "map", "random"}
RANDOM_KEYS = {"order_gap_s", "low_battery"}
LOW_BATTERY_KEYS = {"from_s", "to_s", "charge_s"}


def scenario_map_file(scenario_path) -> str:
    """시나리오의 `map`(maps 폴더의 지도 이름)을 절대 경로로. 없으면 빈 값(기본 지도)."""
    path = Path(scenario_path)
    name = (yaml.safe_load(path.read_text(encoding="utf-8")) or {}).get("map")
    if not name:
        return ""
    found = path.resolve().parent.parent / "maps" / f"{name}.yaml"
    if not found.is_file():
        raise ScenarioError(f"map {name!r} is not in {found.parent}")
    return str(found)


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
    start: tuple | None = None  # operator_reinstate: 로봇을 다시 놓는 지점
    basket: str = "EMPTY"  # operator_reinstate: 관리자가 알리는 바구니
    target: str = ""  # nav_abort: 이 지점으로 가는 목표만 중단. 비면 모든 목표
    offset_m: float = 0.0  # drift: 보고 위치를 진행 방향 왼쪽으로 미는 거리(음수면 오른쪽)
    distance_m: float = 0.0  # obstacle_behind: 로봇 뒤 물체까지(path를 따라). 후진을 막는다
    x: float = 0.0  # object: 물체 중심(맵 좌표, m)
    y: float = 0.0
    radius_m: float = OBJECT_RADIUS_M  # object: 물체 반지름


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
    parked_while_backlog: int | None = 0
    timed_out: bool = False
    unexpected_errors: int = 0
    bound_after_reinstate_min: int = 0
    shipped_after_reinstate_min: int = 0
    order_results: dict = field(default_factory=dict)
    events_include: list = field(default_factory=list)
    event_counts: dict = field(default_factory=dict)
    shipped_before_operator_min: int = 0
    detours_around_isolated_min: int = 0
    released_while_off_lane: int = 0
    path_not_connected: int = 0
    robot_idle: int | None = 0


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
        """끝까지 돌아오지 않는 로봇. 마지막 치우기 뒤에 다시 투입하면 빠진다."""
        gone = set()
        for f in sorted(self.faults, key=lambda f: f.at_s):  # 안정 정렬: 같은 시각은 파일 순서
            if f.type in GONE_FAULTS:
                gone.add(f.robot)
            elif f.type == "operator_reinstate":
                gone.discard(f.robot)
        return gone


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
    if kind == "low_battery":
        _positive(raw, "duration_s")  # 충전에 걸리는 시간
    if kind == "obstacle_behind":
        _positive(raw, "duration_s")
        _positive(raw, "distance_m")
    if kind == "object":
        x, y, radius = _object_place(raw, lane_map)
    else:
        if any(key in raw for key in OBJECT_KEYS):
            raise ScenarioError(f"{', '.join(OBJECT_KEYS)} belong to an object fault: {raw}")
        x, y, radius = 0.0, 0.0, OBJECT_RADIUS_M
    start, basket = None, str(raw.get("basket", "EMPTY"))
    if kind == "operator_reinstate":
        if "start" not in raw:
            raise ScenarioError(f"operator_reinstate needs a start: {raw}")
        start = _resolve_start(raw["start"], lane_map)
        if basket not in REINSTATE_BASKETS:
            raise ScenarioError(f"operator_reinstate basket must be one of {REINSTATE_BASKETS}")
    if kind == "drift" and not (raw.get("duration_s", 0) > 0 and raw.get("offset_m", 0) != 0):
        raise ScenarioError(f"drift needs a positive duration_s and a nonzero offset_m: {raw}")
    target = str(raw.get("target", ""))
    if target and (kind != "nav_abort" or target not in lane_map.zone_of):
        raise ScenarioError(f"target needs nav_abort and a waypoint of the map in {raw}")
    return Fault(float(raw["at_s"]), raw.get("robot", ""), kind,
                 float(raw.get("duration_s", 0.0)), int(raw.get("count", 1)),
                 raw.get("arm", ""), int(raw.get("after_items", 0)),
                 start=start, basket=basket, target=target,
                 offset_m=float(raw.get("offset_m", 0.0)),
                 distance_m=float(raw.get("distance_m", 0.0)), x=x, y=y, radius_m=radius)


def _object_place(raw: dict, lane_map) -> tuple[float, float, float]:
    """object 고장의 (x, y, 반지름). 로봇에 묶이지 않고, 웨이포인트 범위 가까이에 있어야 한다."""
    if raw.get("robot"):
        raise ScenarioError(f"an object is not tied to a robot: {raw}")
    _positive(raw, "duration_s")  # 물체가 있는 시간
    radius = _positive(raw, "radius_m", OBJECT_RADIUS_M)
    place = []
    for key in ("x", "y"):
        value = raw.get(key)
        if (not isinstance(value, (int, float)) or isinstance(value, bool)
                or not math.isfinite(value)):
            raise ScenarioError(f"object needs a number {key}, got {value!r}: {raw}")
        place.append(float(value))
    x0, y0, x1, y1 = lane_map.bounds()
    m = OBJECT_MAP_MARGIN_M
    if not (x0 - m <= place[0] <= x1 + m and y0 - m <= place[1] <= y1 + m):
        raise ScenarioError(f"object at {tuple(place)} is far off the map: {raw}")
    return place[0], place[1], radius


def _check_reinstates(faults: list) -> None:
    """다시 투입은 그 로봇을 치운 뒤에만 할 수 있다."""
    removed: set[str] = set()
    for f in sorted(faults, key=lambda f: f.at_s):
        if f.type == "operator_remove":
            removed.add(f.robot)
        elif f.type == "operator_reinstate":
            if f.robot not in removed:
                raise ScenarioError(f"operator_reinstate of {f.robot} at {f.at_s} s comes "
                                    "before any operator_remove of it")
            removed.discard(f.robot)


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


def _span(lo, hi, what: str) -> tuple[float, float]:
    for value in (lo, hi):
        if not isinstance(value, (int, float)) or isinstance(value, bool) or value < 0:
            raise ScenarioError(f"{what} needs non-negative numbers, got {lo!r}, {hi!r}")
    if lo > hi:
        raise ScenarioError(f"{what}: {lo} is above {hi}")
    return float(lo), float(hi)


def expand_random(data: dict, seed: int | None = None) -> dict:
    """`random` 블록을 정해진 시각으로 푼 새 dict. 입력은 바꾸지 않는다.

    seed가 None이면 파일의 seed. 주문을 파일 순서로 먼저 뽑고 로봇은 이름 순서로 뽑는다.
    random이 없으면 내용이 같은 사본을 돌려준다.
    """
    out = copy.deepcopy(data)
    spec = out.pop("random", None)
    if spec is None:
        return out
    if not isinstance(spec, dict):
        raise ScenarioError(f"random must be a mapping, got {spec!r}")
    unknown = set(spec) - RANDOM_KEYS
    if unknown:
        raise ScenarioError(f"unknown random keys: {sorted(unknown)}")
    gap = spec.get("order_gap_s")
    if gap is not None and not (isinstance(gap, (list, tuple)) and len(gap) == 2):
        raise ScenarioError(f"random.order_gap_s must be [low, high], got {gap!r}")
    gap = _span(*gap, "random.order_gap_s") if gap is not None else None
    battery = spec.get("low_battery")
    if battery is not None:
        if not isinstance(battery, dict) or set(battery) != LOW_BATTERY_KEYS:
            raise ScenarioError(f"random.low_battery needs exactly {sorted(LOW_BATTERY_KEYS)}, "
                                f"got {battery!r}")
        span = _span(battery["from_s"], battery["to_s"], "random.low_battery from_s/to_s")
        charge_s = _positive(battery, "charge_s")
    seed = int(out.get("seed", 0) if seed is None else seed)
    rng = random.Random(seed)
    out["seed"] = seed
    if gap is not None:
        t = 0.0
        for order in out.get("orders") or []:
            t += rng.uniform(*gap)
            order["at_s"] = round(t, 1)
    if battery is not None:
        added = [{"type": "low_battery", "robot": robot, "at_s": round(rng.uniform(*span), 1),
                  "duration_s": charge_s}
                 for robot in sorted(out.get("robots") or {})]
        out["faults"] = list(out.get("faults") or []) + added
    return out


def parse_scenario(data: dict, lane_map, seed: int | None = None) -> Scenario:
    """seed: random 블록을 풀 seed. None이면 파일의 seed."""
    data = expand_random(data, seed)
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
    _check_reinstates(faults)
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


def load_scenario(path, lane_map=None, seed: int | None = None) -> Scenario:
    if lane_map is None:
        from fleet_core.traffic.lane_map import load_lane_map
        lane_map = load_lane_map(scenario_map_file(path) or None)
    return parse_scenario(yaml.safe_load(Path(path).read_text(encoding="utf-8")), lane_map,
                          seed)
