"""밖에서 온 잘못된 입력은 거부하거나 버리고, 한 번 알리고, 예외를 내지 않는다.

검사는 노드가 부르는 FleetManager에 있어 기록 재생도 같다.
두 코어도 모르는 로봇에 예외를 내지 않는다.
"""
import json
import math
import sys
from pathlib import Path

import pytest
from core_helpers import cmds, codes
from fleet_core.common.commands import DispatchResult, OrderRecord
from fleet_core.common.params import load_params
from fleet_core.manager import FleetManager
from fleet_core.orders.orchestrator import OrderOrchestrator
from fleet_core.traffic.controller import TrafficController
from fleet_core.traffic.lane_map import load_lane_map

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))

import replay  # noqa: E402

O1 = OrderRecord("O-001", {"A": 1})
O2 = OrderRecord("O-002", {"B": 2})
SPOTS = {"pinky1": "P-6", "pinky2": "P-5"}


def make(robots=SPOTS, board=True):
    now = [0.0]
    core = FleetManager(load_lane_map(), load_params(), list(robots), lambda: now[0])
    for robot, waypoint in robots.items():
        report(core, robot, waypoint)
    if board:
        core.on_order_board("RUN", [])
    core.tick()
    return core, now


def report(core, robot, waypoint, robot_id=None):
    x, y = core.site.waypoint_pose(waypoint)
    for _ in range(core.params.release_confirm_samples):
        if robot_id is None:
            core.on_robot_state(robot, x, y, 0.0, "IDLE", 90.0)
        else:
            core.on_robot_state(robot, x, y, 0.0, "IDLE", 90.0, robot_id)


def rejected(out):
    events = [e for e in cmds(out, "EventOut") if e.code == "INPUT_REJECTED"]
    assert all(e.level == "WARN" and e.detail for e in events), events
    return events


def pose_of(core, robot):
    v = core.traffic.status_view(robot)
    return v["x"], v["y"], v["yaw"]


# ---- confirm_clear ----------------------------------------------------------------------------


def test_clear_of_an_unknown_robot_is_refused_with_a_reason_and_input_rejected():
    core, _ = make()
    held = core.zones.held_by("pinky1")
    assert core.confirm_clear("pinky9", ["P-6"], "typo") is None
    assert core.last_clear_reason
    assert core.zones.held_by("pinky1") == held
    out = core.tick()
    assert len(rejected(out)) == 1
    assert "CLEAR_REFUSED" not in codes(out)


def test_clear_with_an_unknown_zone_is_refused_and_changes_nothing():
    core, _ = make()
    held = core.zones.held_by("pinky1")
    assert core.confirm_clear("pinky1", [*held, "NOPE"], "typo") is None
    assert "NOPE" in core.last_clear_reason
    assert core.zones.held_by("pinky1") == held
    assert core.robot_view("pinky1")["state"] != "OUT_OF_SERVICE"
    out = core.tick()
    events = rejected(out)
    assert len(events) == 1 and events[0].robot == "pinky1"
    assert "CLEAR_REFUSED" not in codes(out)
    assert "ZONES_CLEARED" not in codes(out)


def test_every_refused_clear_call_says_so_once():
    core, _ = make()
    core.confirm_clear("pinky1", ["NOPE"], "one")
    core.confirm_clear("pinky1", ["NOPE"], "two")
    assert len(rejected(core.tick())) == 2


def test_clear_with_no_zones_while_holding_some_stays_clear_refused_with_a_reason():
    core, _ = make()
    assert core.confirm_clear("pinky1", [], "forgot") is None
    assert "holds" in core.last_clear_reason
    out = core.tick()
    assert "CLEAR_REFUSED" in codes(out)
    assert not rejected(out)


def test_an_accepted_clear_has_no_reason():
    core, _ = make()
    core.confirm_clear("pinky1", ["NOPE"], "typo")
    assert core.confirm_clear("pinky1", core.zones.held_by("pinky1"), "lifted") == ["P-6"]
    assert core.last_clear_reason == ""


@pytest.mark.parametrize("robot, zones", [
    (None, ["P-6"]),
    (7, ["P-6"]),
    ("pinky1", None),
    ("pinky1", "P-6"),
    ("pinky1", [None]),
    ("pinky1", [""]),
])
def test_clear_with_arguments_of_the_wrong_shape_is_refused(robot, zones):
    core, _ = make()
    held = core.zones.held_by("pinky1")
    assert core.confirm_clear(robot, zones, "bad") is None
    assert core.last_clear_reason
    assert core.zones.held_by("pinky1") == held
    assert len(rejected(core.tick())) == 1


def test_traffic_alone_refuses_an_unknown_robot_or_zone_without_raising():
    site, params = load_lane_map(), load_params()
    traffic = TrafficController(site, params, ["pinky1"], lambda: 0.0)
    assert traffic.confirm_clear("pinky9", ["P-6"]) is None
    assert traffic.confirm_clear("pinky1", ["NOPE"]) is None
    out = traffic.take_output()
    assert [e.code for e in cmds(out, "EventOut")] == ["INPUT_REJECTED", "INPUT_REJECTED"]


# ---- reinstate --------------------------------------------------------------------------------


def _cleared_pinky2():
    core, now = make()
    core.confirm_clear("pinky2", core.zones.held_by("pinky2"), "lifted")
    core.tick()
    return core


def test_reinstate_of_an_unknown_robot_is_refused_with_a_reason_and_input_rejected():
    core = _cleared_pinky2()
    assert core.reinstate("pinky9", "EMPTY", "typo") is False
    assert "pinky9" in core.last_reinstate_reason
    out = core.tick()
    assert len(rejected(out)) == 1
    assert "ROBOT_REINSTATED" not in codes(out)


def test_reinstate_of_a_robot_that_is_not_out_of_service_is_refused():
    core = _cleared_pinky2()
    held = core.zones.held_by("pinky1")
    assert core.reinstate("pinky1", "EMPTY", "wrong robot") is False
    assert "OUT_OF_SERVICE" in core.last_reinstate_reason
    assert core.zones.held_by("pinky1") == held
    assert core.traffic.robot_view("pinky1")["on_map"]
    events = rejected(core.tick())
    assert len(events) == 1 and events[0].robot == "pinky1"


@pytest.mark.parametrize("basket", ["FULL", "empty", "", None, 3, ["EMPTY"]])
def test_reinstate_with_a_wrong_basket_is_refused_and_changes_nothing(basket):
    core = _cleared_pinky2()
    assert core.reinstate("pinky2", basket, "bad") is False
    assert "basket" in core.last_reinstate_reason
    assert core.traffic.robot_view("pinky2")["fault"] == "OUT_OF_SERVICE"
    out = core.tick()
    assert len(rejected(out)) == 1
    assert "ROBOT_REINSTATED" not in codes(out)


def test_reinstate_while_the_robot_still_holds_a_zone_is_refused():
    """위치로 풀리지 않는 구역이 남았다. confirm_clear로 먼저 풀어야 한다."""
    core, _ = make()
    core.confirm_clear("pinky2", [], "kept its zone")  # 쥔 구역이 있어 CLEAR_REFUSED
    core.confirm_clear("pinky2", ["P-3"], "a zone it does not hold")
    assert core.robot_view("pinky2")["state"] == "OUT_OF_SERVICE"
    assert core.zones.held_by("pinky2")
    assert core.reinstate("pinky2", "EMPTY", "too early") is False
    assert "holds" in core.last_reinstate_reason
    assert len(rejected(core.tick())) == 1
    core.confirm_clear("pinky2", core.zones.held_by("pinky2"), "now all")
    assert core.reinstate("pinky2", "EMPTY", "back") is True


def test_every_refused_reinstate_call_says_so_once():
    core = _cleared_pinky2()
    core.reinstate("pinky2", "FULL", "one")
    core.reinstate("pinky2", "FULL", "two")
    assert len(rejected(core.tick())) == 2


def test_an_accepted_reinstate_has_no_reason():
    core = _cleared_pinky2()
    core.reinstate("pinky2", "FULL", "typo")
    core.tick()
    assert core.reinstate("pinky2", "EMPTY", "back") is True
    assert core.last_reinstate_reason == ""
    assert not rejected(core.tick())


# ---- robot state ------------------------------------------------------------------------------


@pytest.mark.parametrize("bad", [
    dict(x=math.nan), dict(y=math.nan), dict(yaw=math.nan),
    dict(x=math.inf), dict(y=-math.inf), dict(yaw=math.inf),
    dict(x=-5.0), dict(y=40.0), dict(x="1.0"), dict(x=None),
])
def test_a_pose_that_is_not_a_number_or_off_the_map_is_dropped(bad):
    core, _ = make()
    before = pose_of(core, "pinky1")
    pose = dict(x=before[0], y=before[1], yaw=0.0)
    pose.update(bad)
    core.on_robot_state("pinky1", pose["x"], pose["y"], pose["yaw"], "IDLE", 90.0)
    assert pose_of(core, "pinky1") == before
    events = rejected(core.tick())
    assert len(events) == 1 and events[0].robot == "pinky1"


def test_a_pose_just_outside_the_waypoints_but_within_the_margin_is_taken():
    core, _ = make()
    lowest_x = min(x for x, _ in map(core.site.waypoint_pose, core.site.zone_of))
    margin = core.params.input_pose_margin_m
    core.on_robot_state("pinky1", lowest_x - margin / 2, 1.0, 0.0, "IDLE", 90.0)
    assert pose_of(core, "pinky1")[0] == lowest_x - margin / 2
    assert not rejected(core.tick())


def test_a_report_naming_another_robot_is_dropped():
    core, _ = make()
    x, y = core.site.waypoint_pose("P-4")
    before = pose_of(core, "pinky1")
    core.on_robot_state("pinky1", x, y, 0.0, "IDLE", 90.0, "pinky2")
    assert pose_of(core, "pinky1") == before
    events = rejected(core.tick())
    assert len(events) == 1 and "pinky2" in events[0].detail


def test_a_report_with_its_own_name_or_none_is_taken():
    core, _ = make()
    report(core, "pinky1", "P-6", "pinky1")
    report(core, "pinky1", "P-6", "")
    assert not rejected(core.tick())


def test_an_unknown_robot_state_is_dropped_and_said_once():
    core, _ = make()
    for _ in range(5):
        core.on_robot_state("pinky9", 1.0, 1.0, 0.0, "IDLE", 90.0)
    assert len(rejected(core.tick())) == 1
    assert "pinky9" not in core.traffic.robots


def test_an_unknown_state_string_and_any_battery_are_still_taken():
    """BLOCKED는 뒤 파트에서 더한다. 배터리는 여기서 판단하지 않는다."""
    core, _ = make()
    x, y = core.site.waypoint_pose("P-6")
    core.on_robot_state("pinky1", x, y, 0.0, "BLOCKED", math.nan)
    assert core.traffic.status_view("pinky1")["fsm_state"] == "BLOCKED"
    assert not rejected(core.tick())


def test_a_bad_state_source_is_said_once_until_it_sends_a_good_one():
    core, _ = make()
    x, y = core.site.waypoint_pose("P-6")
    for _ in range(3):
        core.on_robot_state("pinky1", math.nan, y, 0.0, "IDLE", 90.0)
    core.on_robot_state("pinky1", x, y, 0.0, "IDLE", 90.0, "pinky2")  # 다른 이유, 같은 연속
    assert len(rejected(core.tick())) == 1
    core.on_robot_state("pinky1", x, y, 0.0, "IDLE", 90.0)
    for _ in range(3):
        core.on_robot_state("pinky1", math.nan, y, 0.0, "IDLE", 90.0)
    assert len(rejected(core.tick())) == 1, "정상 보고가 다시 무장한다"


def test_sources_are_counted_apart():
    core, _ = make()
    core.on_robot_state("pinky1", math.nan, 1.0, 0.0, "IDLE")
    core.on_robot_state("pinky2", math.nan, 1.0, 0.0, "IDLE")
    core.on_order_board("", [])
    assert len(rejected(core.tick())) == 3


# ---- order board ------------------------------------------------------------------------------


@pytest.mark.parametrize("mode", ["", "GO", "run", None, 3])
def test_a_board_with_an_unknown_run_mode_is_dropped_whole(mode):
    core, _ = make()
    core.on_order_board(mode, [O1])
    assert core.traffic.running, "앞의 RUN이 유지된다"
    assert core.orchestrator._board == []
    assert len(rejected(core.tick())) == 1


BAD_ORDERS = {
    "empty order id": [OrderRecord("", {"A": 1})],
    "duplicate order id": [O1, OrderRecord("O-001", {"B": 1})],
    "no items": [OrderRecord("O-003", {})],
    "empty item name": [OrderRecord("O-003", {"": 1})],
    "zero quantity": [OrderRecord("O-003", {"A": 0})],
    "negative quantity": [OrderRecord("O-003", {"A": -2})],
    "quantity not a number": [OrderRecord("O-003", {"A": "2"})],
    "not an order": ["O-003"],
    "orders not a list": None,
}


@pytest.mark.parametrize("orders", BAD_ORDERS.values(), ids=BAD_ORDERS.keys())
def test_a_malformed_order_keeps_the_last_orders_but_the_mode_is_applied(orders):
    core, _ = make()
    core.on_order_board("RUN", [O1, O2])
    core.tick()
    core.on_order_board("PAUSE", orders)
    assert not core.traffic.running
    assert [o.order_id for o in core.orchestrator._board] == ["O-001", "O-002"]
    assert len(rejected(core.tick())) == 1


def test_stop_is_applied_even_with_a_malformed_order():
    core, _ = make()
    core.on_order_board("RUN", [O1])
    for _ in range(3):
        core.tick()
    assert core.traffic._missions["pinky1"].goal or core.traffic._missions["pinky2"].goal
    core.on_order_board("STOP", [OrderRecord("O-001", {"A": 0})])
    out = core.tick()
    assert not core.traffic.running
    assert cmds(out, "CancelCmd"), "STOP이 진행 중 goal을 취소했다"
    assert len(rejected(out)) == 1


def test_an_empty_board_with_a_good_mode_is_fine():
    core, _ = make()
    core.on_order_board("STOP", [])
    assert not rejected(core.tick())


def test_a_bad_board_is_said_once_until_a_good_one_comes():
    core, _ = make()
    for _ in range(3):
        core.on_order_board("", [])
    core.on_order_board("RUN", [OrderRecord("", {"A": 1})])
    assert len(rejected(core.tick())) == 1
    core.on_order_board("RUN", [])
    core.on_order_board("RUN", [OrderRecord("", {"A": 1})])
    assert len(rejected(core.tick())) == 1


# ---- goal results -----------------------------------------------------------------------------


def test_results_for_an_unknown_robot_are_refused_without_raising():
    core, _ = make()
    core.on_move_result("pinky9", 1, True, "")
    core.on_goal_rejected("pinky9", 1)
    core.on_goal_not_sent("pinky9", 1)
    core.on_dock_result("pinky9", 1, True)
    assert len(rejected(core.tick())) == 4


def test_stale_or_unknown_goal_ids_are_ignored_quietly():
    """취소 뒤 늦은 결과는 정상이다."""
    core, _ = make()
    before = {r: core.robot_view(r) for r in SPOTS}
    core.on_move_result("pinky1", 999, True, "")
    core.on_goal_rejected("pinky1", 999)
    core.on_goal_not_sent("pinky1", 999)
    core.on_dock_result("pinky1", 999, False)
    core.on_fill_result(999, True, {"A": 1}, {})
    core.on_swap_result(999, True, True, True)
    assert {r: core.robot_view(r) for r in SPOTS} == before
    assert not cmds(core.tick(), "EventOut")


def test_views_of_an_unknown_robot_do_not_raise():
    core, _ = make()
    assert core.robot_view("pinky9") == {}


# ---- the cores on their own -------------------------------------------------------------------


def test_traffic_ignores_an_unknown_robot_everywhere():
    site, params = load_lane_map(), load_params()
    traffic = TrafficController(site, params, ["pinky1"], lambda: 0.0)
    traffic.on_robot_state("ghost", 1.0, 1.0, 0.0, "IDLE")
    traffic.on_release("ghost")
    traffic.on_robot_fault("ghost", "why")
    traffic.on_move_result("ghost", 1, True, "")
    traffic.on_goal_rejected("ghost", 1)
    traffic.on_goal_not_sent("ghost", 1)
    traffic.on_dock_result("ghost", 1, True)
    traffic.step("ghost")
    assert traffic.available("ghost") is False
    assert traffic.robot_view("ghost") == {}
    assert traffic.status_view("ghost") == {}
    traffic.on_dispatch(5, "OUTBOUND", "ghost")
    assert traffic.take_output() == [DispatchResult(5, False, "ghost", "NO_SUCH_ROBOT")]
    assert traffic.robots == ["pinky1"]


def test_orchestrator_ignores_an_unknown_robot_everywhere():
    site, params = load_lane_map(), load_params()
    orders = OrderOrchestrator(site.stations, params, ["pinky1"], lambda: 0.0)
    orders.on_dispatch_result(5, True, "ghost")
    orders.on_robot_removed("ghost")
    orders.step("ghost")
    assert orders.order_of("ghost") == ""
    assert orders.robot_view("ghost") == {}
    assert orders.take_output() == []


# ---- record and replay ------------------------------------------------------------------------


def test_a_run_with_rejected_inputs_plays_back_the_same(tmp_path):
    now = [0.0]
    core = FleetManager(load_lane_map(), load_params(), list(SPOTS), lambda: now[0])
    lines = []
    core.record_to(lines.append)
    for robot, waypoint in SPOTS.items():
        report(core, robot, waypoint, robot)
    core.on_order_board("RUN", [O1])
    core.on_robot_state("pinky1", math.nan, 1.0, 0.0, "IDLE", 90.0, "pinky1")
    core.on_robot_state("pinky1", 1.0, 1.0, math.inf, "IDLE", 90.0, "pinky2")
    core.on_robot_state("pinky9", 1.0, 1.0, 0.0, "IDLE", 90.0)
    core.on_order_board("", [])
    core.on_order_board("RUN", [OrderRecord("O-002", {"A": 0})])
    core.confirm_clear("pinky1", ["NOPE"], "typo")
    core.on_move_result("pinky9", 3, True, "")
    for _ in range(20):
        now[0] += 0.1
        core.tick()
    assert any("INPUT_REJECTED" in line for line in lines)
    path = tmp_path / "rejects.jsonl"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    result = replay.replay(path)
    assert result.same and result.compared, result
    assert json.loads(lines[0])["params"]["input_pose_margin_m"] == 1.0


def _recorded():
    """기록을 켜고 주문 하나로 시작한 코어."""
    now = [0.0]
    core = FleetManager(load_lane_map(), load_params(), list(SPOTS), lambda: now[0])
    lines = []
    core.record_to(lines.append)
    for robot, waypoint in SPOTS.items():
        report(core, robot, waypoint)
    core.on_order_board("RUN", [O1])
    for _ in range(3):
        now[0] += 0.1
        core.tick()
    return core, now, lines


def _play(tmp_path, lines):
    path = tmp_path / "run.jsonl"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return replay.replay(path)


def test_stop_with_an_order_the_record_cannot_write_is_still_applied(tmp_path):
    core, now, lines = _recorded()
    core.on_order_board("STOP", [object()])
    out = core.tick()
    assert not core.traffic.running
    assert len(rejected(out)) == 1
    result = _play(tmp_path, lines)
    assert result.compared and result.same, result


WRONG_SHAPES = {
    "none": None,
    "a string": "O-009",
    "a dict missing items": [{"order_id": "O-009"}],
    "a dict that looks like an order": [{"order_id": "O-009", "items": {"A": 1}}],
    "a list in a list": [[1, 2]],
}


@pytest.mark.parametrize("orders", WRONG_SHAPES.values(), ids=WRONG_SHAPES.keys())
def test_a_board_of_the_wrong_shape_plays_back_the_same(tmp_path, orders):
    core, now, lines = _recorded()
    core.on_order_board("PAUSE", orders)
    assert len(rejected(core.tick())) == 1
    result = _play(tmp_path, lines)
    assert result.compared and result.same, result


def test_a_robot_id_that_is_not_a_string_plays_back_the_same(tmp_path):
    core, now, lines = _recorded()
    x, y = core.site.waypoint_pose("P-6")
    core.on_robot_state("pinky1", x, y, 0.0, "IDLE", 90.0, 0)
    assert len(rejected(core.tick())) == 1
    result = _play(tmp_path, lines)
    assert result.compared and result.same, result


def _cyclic_order():
    items = {}
    items["A"] = items
    return OrderRecord("O-009", items)


UNWRITABLE = {
    "a cycle": [_cyclic_order()],
    "a key that is not a string": [OrderRecord("O-009", {("A",): 1})],
}


@pytest.mark.parametrize("orders", UNWRITABLE.values(), ids=UNWRITABLE.keys())
def test_an_input_the_record_cannot_write_is_still_handled(tmp_path, orders):
    core, now, lines = _recorded()
    core.on_order_board("STOP", orders)
    out = core.tick()
    assert not core.traffic.running
    assert len(rejected(out)) == 1
    assert any("unrecordable" in json.loads(line) for line in lines)
    path = tmp_path / "run.jsonl"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    with pytest.raises(ValueError, match="could not be recorded"):
        replay.replay(path)


class _Unprintable:
    """repr이 실패하는 값. 검사기는 이유를 만들다가 죽으면 안 된다."""

    def __repr__(self):
        raise RuntimeError("no repr")


UNPRINTABLE = {
    "a count": [OrderRecord("O-009", {"A": _Unprintable()})],
    "an item name": [OrderRecord("O-009", {_Unprintable(): 1})],
    "an order id": [OrderRecord(_Unprintable(), {"A": 1})],
}


@pytest.mark.parametrize("record", (False, True), ids=("plain", "recorded"))
@pytest.mark.parametrize("orders", UNPRINTABLE.values(), ids=UNPRINTABLE.keys())
def test_stop_with_an_order_that_cannot_be_printed_is_still_applied(orders, record):
    if record:
        core, now, lines = _recorded()
    else:
        core, now = make()
        core.on_order_board("RUN", [O1])
    core.on_order_board("STOP", orders)
    assert not core.traffic.running
    assert len(rejected(core.tick())) == 1


class _Spiteful(str):
    """비교, 해시, repr, str이 모두 실패하는 문자열."""

    def __eq__(self, other):
        raise RuntimeError("no eq")

    def __ne__(self, other):
        raise RuntimeError("no ne")

    def __hash__(self):
        raise RuntimeError("no hash")

    def __repr__(self):
        raise RuntimeError("no repr")

    def __str__(self):
        raise RuntimeError("no str")


class _Incomparable(str):
    """해시는 되지만 비교가 실패하는 문자열. dict 키로 쓸 수 있다."""

    __hash__ = str.__hash__

    def __eq__(self, other):
        raise RuntimeError("no eq")

    def __ne__(self, other):
        raise RuntimeError("no ne")


ODD_CALLS = {
    "a pose": lambda c: c.on_robot_state("pinky1", _Unprintable(), 0.0, 0.0, "IDLE", 90.0),
    "a robot_id": lambda c: c.on_robot_state("pinky1", 0.0, 0.0, 0.0, "IDLE", 90.0,
                                             _Spiteful("pinky1")),
    "a robot on the state topic": lambda c: c.on_robot_state(_Spiteful("pinky1"), 0.0, 0.0,
                                                             0.0, "IDLE", 90.0),
    "a robot in a result": lambda c: c.on_move_result(_Spiteful("pinky1"), 1, True, ""),
    "a run mode": lambda c: c.on_order_board(_Spiteful("STOP"), [O1]),
    "a zone": lambda c: c.confirm_clear("pinky1", [_Unprintable()], "lifted"),
    "a robot to clear": lambda c: c.confirm_clear(_Spiteful("pinky1"), ["P-6"], "lifted"),
    "a note": lambda c: c.confirm_clear("pinky1", ["NO-SUCH-ZONE"], _Unprintable()),
    "a robot to reinstate": lambda c: c.reinstate(_Spiteful("pinky1"), "EMPTY", "back"),
    "a basket": lambda c: c.reinstate("pinky1", _Spiteful("EMPTY"), "back"),
    "a basket that cannot be printed": lambda c: c.reinstate("pinky1", _Unprintable(), "back"),
    "a reinstate note": lambda c: c.reinstate("pinky1", "FULL", _Unprintable()),
}


@pytest.mark.parametrize("call", ODD_CALLS.values(), ids=ODD_CALLS.keys())
def test_an_odd_value_anywhere_is_rejected_without_raising(call):
    core, now, lines = _recorded()
    call(core)
    assert len(rejected(core.tick())) == 1


ODD_ORDERS = {
    "an order id that cannot be hashed": [OrderRecord(_Spiteful("O-009"), {"A": 1})],
    "an item name that cannot be compared": [OrderRecord("O-009", {_Incomparable("A"): 1})],
}


@pytest.mark.parametrize("orders", ODD_ORDERS.values(), ids=ODD_ORDERS.keys())
def test_stop_with_an_order_that_cannot_be_checked_is_still_applied(orders):
    core, now, lines = _recorded()
    core.on_order_board("STOP", orders)
    assert not core.traffic.running
    assert len(rejected(core.tick())) == 1


def test_a_state_that_cannot_be_compared_is_kept_as_plain_text():
    """state는 거부하지 않는다. 비교가 실패하는 값이 tick까지 가면 안 된다."""
    now = [0.0]
    core = FleetManager(load_lane_map(), load_params(), list(SPOTS), lambda: now[0])
    for robot, waypoint in SPOTS.items():
        x, y = core.site.waypoint_pose(waypoint)
        for _ in range(core.params.release_confirm_samples):
            core.on_robot_state(robot, x, y, 0.0, _Incomparable("IDLE"), 90.0)
    assert not rejected(core.tick())
    state = core.traffic.status_view("pinky1")["fsm_state"]
    assert state == "IDLE" and type(state) is str
