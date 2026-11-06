"""살아 있는 그래프에서 ScenarioMonitor를 시험한다. 테스트가 플릿 매니저 역을 한다."""
import json
import time

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces.msg")

from conftest import CONTRACT, wait_until, write_scenario  # noqa: E402
from fleet_core.common.contract import load_contract, to_ros_qos  # noqa: E402
from fleet_interfaces.action import FillBasket, SwapBasket  # noqa: E402
from fleet_interfaces.msg import FleetEvent, OrderItem, ZoneState, ZoneStateArray  # noqa: E402
from fleet_interfaces.srv import ConfirmClear, Reinstate, ReportOrder  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402

C = load_contract(CONTRACT)
ONE_ROBOT = {"pinky1": {"start": "P-6", "speed_mps": 1.0, "pose_noise_m": 0.0}}


def start(harness, tmp_path, **scenario):
    import rclpy.node
    from fake_robots.fake_arm_node import FakeArmNode
    from fake_robots.fake_central_node import FakeCentralNode
    from fake_robots.fake_pinky_node import FakePinkyNode
    from fake_robots.fake_workcell_node import FakeWorkcellNode
    from fake_robots.scenario_monitor_node import ScenarioMonitorNode
    scenario.setdefault("robots", ONE_ROBOT)
    scn = write_scenario(tmp_path, **scenario)
    harness.add(FakePinkyNode(parameter_overrides=harness.params(
        robot_id="pinky1", scenario_file=scn)))
    harness.add(FakeWorkcellNode(parameter_overrides=harness.params(scenario_file=scn)))
    harness.add(FakeArmNode(parameter_overrides=harness.params(
        arm_id="arm3", scenario_file=scn)))
    harness.add(FakeCentralNode(parameter_overrides=harness.params(scenario_file=scn)))
    monitor = harness.add(ScenarioMonitorNode(parameter_overrides=harness.params(
        scenario_file=scn, result_dir=str(tmp_path / "result"))))
    probe = harness.add(rclpy.node.Node("fleet_stand_in"))
    return monitor, probe


def events_pub(probe):
    pub = probe.create_publisher(FleetEvent, C.name("events"), to_ros_qos(C.qos("events")))
    return lambda code, detail, source="pinky1": pub.publish(
        FleetEvent(level="INFO", source=source, code=code, detail=detail))


def zones_pub(probe):
    pub = probe.create_publisher(ZoneStateArray, C.name("zones"), to_ros_qos(C.qos("zones")))
    return lambda holders: pub.publish(ZoneStateArray(
        zones=[ZoneState(zone_id=z, holder=h) for z, h in holders.items()]))


def goal_result(client, goal):
    assert client.wait_for_server(10.0)
    fut = client.send_goal_async(goal)
    assert wait_until(fut.done, 5.0) and fut.result().accepted
    res = fut.result().get_result_async()
    assert wait_until(res.done, 15.0)
    return res.result().result


def run_one_order(probe, emit):
    """플릿 매니저가 주문 하나에 하는 일(주행 제외)."""
    items = [OrderItem(kind="A", count=1), OrderItem(kind="C", count=1)]
    fill = ActionClient(probe, FillBasket, C.name("fill_basket"))
    swap = ActionClient(probe, SwapBasket, C.name("swap_basket", arm="arm3"))
    report = probe.create_client(ReportOrder, C.name("report_order"))
    assert report.wait_for_service(5.0)
    emit("ORDER_BOUND", "O-1")
    filled = goal_result(fill, FillBasket.Goal(order_id="O-1", robot_id="pinky1", items=items))
    assert filled.success
    swapped = goal_result(swap, SwapBasket.Goal(order_id="O-1", robot_id="pinky1",
                                                take_full=True, put_empty=True))
    assert swapped.success
    fut = report.call_async(ReportOrder.Request(
        order_id="O-1", robot_id="pinky1", status=ReportOrder.Request.STATUS_SHIPPED,
        placed=filled.placed))
    assert wait_until(fut.done, 5.0) and fut.result().accepted


ONE_ORDER = [{"at_s": 0, "id": "O-1", "items": {"A": 1, "C": 1}}]


def test_a_finished_run_writes_a_passing_summary(harness, tmp_path):
    monitor, probe = start(harness, tmp_path, orders=ONE_ORDER, timeout_s=60, expect={
        "orders_shipped": 1, "orders_partial": 0, "orders_held": 0,
        "events_include": ["ORDER_BOUND", "ORDER_SHIPPED"]})
    emit, zones = events_pub(probe), zones_pub(probe)
    zones({"P-6": "pinky1"})
    time.sleep(1.0)  # 실제 기동처럼 모니터가 대역을 찾게 한다
    zones({"P-6": "pinky1"})
    run_one_order(probe, emit)
    assert wait_until(lambda: monitor.exit_code is not None, 15.0)
    summary = json.loads((tmp_path / "result" / "summary.json").read_text(encoding="utf-8"))
    assert summary["pass"] is True, summary
    assert monitor.exit_code == 0
    assert summary["orders_shipped"] == 1 and summary["timed_out"] is False
    for name in ("events.csv", "zones.csv", "robots.csv", "releases.csv"):
        assert (tmp_path / "result" / name).exists()
    events = (tmp_path / "result" / "events.csv").read_text(encoding="utf-8")
    assert "ARM_STATE" in events


def test_a_wrong_count_fails_the_run_with_exit_code_one(harness, tmp_path):
    monitor, probe = start(harness, tmp_path, orders=ONE_ORDER, timeout_s=60, expect={
        "orders_shipped": 2})
    emit, zones = events_pub(probe), zones_pub(probe)
    time.sleep(1.0)
    zones({"P-6": "pinky1"})
    run_one_order(probe, emit)
    assert wait_until(lambda: monitor.exit_code is not None, 15.0)
    assert monitor.exit_code == 1


def test_timeout_with_nothing_done_fails(harness, tmp_path):
    monitor, _ = start(harness, tmp_path, orders=ONE_ORDER, timeout_s=2)
    assert wait_until(lambda: monitor.exit_code is not None, 10.0)
    assert monitor.exit_code == 1
    summary = json.loads((tmp_path / "result" / "summary.json").read_text(encoding="utf-8"))
    assert summary["timed_out"] is True and summary["checks"]["all_orders_finished"]["ok"] is False


def test_operator_remove_calls_confirm_clear_with_the_held_zones(harness, tmp_path):
    _, probe = start(
        harness, tmp_path, orders=ONE_ORDER, timeout_s=60,
        expect={"orders_shipped_min": 1},
        faults=[{"at_s": 2.0, "robot": "pinky1", "type": "operator_remove"}])
    calls = []

    def serve(request, response):
        calls.append((request.robot_id, sorted(request.zone_ids), request.note))
        response.accepted = True
        response.released_zones = list(request.zone_ids)
        return response

    probe.create_service(ConfirmClear, C.name("confirm_clear"), serve)
    zones = zones_pub(probe)
    time.sleep(0.5)
    zones({"P-6": "pinky1", "Q-WORK-2": "pinky1", "Z-OUT": "pinky2", "Z-O": ""})
    assert wait_until(lambda: calls, 8.0), "confirm_clear was not called"
    time.sleep(0.5)
    assert len(calls) == 1
    assert calls[0][0] == "pinky1" and calls[0][1] == ["P-6", "Q-WORK-2"]


def test_confirm_clear_waits_until_the_robot_is_off_the_course(harness, tmp_path):
    monitor, probe = start(
        harness, tmp_path, orders=ONE_ORDER, timeout_s=60,
        expect={"orders_shipped_min": 1},
        faults=[{"at_s": 0.0, "robot": "pinky1", "type": "operator_remove"}])
    calls = []
    probe.create_service(ConfirmClear, C.name("confirm_clear"),
                         lambda req, res: (calls.append(req), res)[1])
    monitor.judge.truth["pinky1"] = monitor.judge.site.waypoint_pose("P-6")
    assert wait_until(lambda: calls, 8.0)  # 가짜가 제거를 알렸을 때만 호출된다
    assert monitor.judge.off_course("pinky1")


def test_operator_reinstate_calls_reinstate_once_the_robot_is_back(harness, tmp_path):
    monitor, probe = start(
        harness, tmp_path, orders=ONE_ORDER, timeout_s=60,
        expect={"orders_shipped_min": 1},
        faults=[{"at_s": 0.5, "robot": "pinky1", "type": "operator_remove"},
                {"at_s": 2.0, "robot": "pinky1", "type": "operator_reinstate", "start": "P-6",
                 "basket": "NONE"}])
    calls = []

    def clear(request, response):
        calls.append(("clear", request.robot_id))
        response.accepted = True
        return response

    def back(request, response):
        calls.append(("reinstate", request.robot_id, request.basket, request.note))
        response.accepted = True
        return response

    probe.create_service(ConfirmClear, C.name("confirm_clear"), clear)
    probe.create_service(Reinstate, C.name("reinstate"), back)
    assert wait_until(lambda: len(calls) >= 2, 10.0), calls
    time.sleep(0.5)
    assert calls == [("clear", "pinky1"), ("reinstate", "pinky1", "NONE", "operator_reinstate")]
    assert not monitor.judge.off_course("pinky1")


def _serve_both(probe, calls, clear_ok=True):
    def clear(request, response):
        calls.append(("clear", request.robot_id))
        response.accepted = clear_ok
        return response

    def back(request, response):
        calls.append(("reinstate", request.robot_id))
        response.accepted = True
        return response

    probe.create_service(ConfirmClear, C.name("confirm_clear"), clear)
    probe.create_service(Reinstate, C.name("reinstate"), back)


def test_a_later_second_removal_does_not_hold_back_the_first_reinstate(harness, tmp_path):
    """리뷰 3번: 앞으로 올 치우기는 지금의 재투입과 상관없다."""
    _, probe = start(
        harness, tmp_path, orders=ONE_ORDER, timeout_s=60,
        expect={"orders_shipped_min": 1},
        faults=[{"at_s": 0.5, "robot": "pinky1", "type": "operator_remove"},
                {"at_s": 2.0, "robot": "pinky1", "type": "operator_reinstate", "start": "P-6"},
                {"at_s": 50.0, "robot": "pinky1", "type": "operator_remove"}])
    calls = []
    _serve_both(probe, calls)
    assert wait_until(lambda: ("reinstate", "pinky1") in calls, 10.0), calls


def test_no_reinstate_after_a_refused_clear(harness, tmp_path):
    _, probe = start(
        harness, tmp_path, orders=ONE_ORDER, timeout_s=60,
        expect={"orders_shipped_min": 1},
        faults=[{"at_s": 0.5, "robot": "pinky1", "type": "operator_remove"},
                {"at_s": 2.0, "robot": "pinky1", "type": "operator_reinstate", "start": "P-6"}])
    calls = []
    _serve_both(probe, calls, clear_ok=False)
    assert wait_until(lambda: ("clear", "pinky1") in calls, 8.0)
    time.sleep(3.0)
    assert ("reinstate", "pinky1") not in calls


def test_a_reinstate_not_ready_does_not_hold_back_another_robot(harness, tmp_path):
    """2차 리뷰 5번: pinky2의 치우기가 끝나지 않아도 pinky1은 다시 넣는다."""
    robots = {**ONE_ROBOT, "pinky2": {"start": "P-5", "speed_mps": 1.0, "pose_noise_m": 0.0}}
    _, probe = start(
        harness, tmp_path, orders=ONE_ORDER, timeout_s=60, robots=robots,
        expect={"orders_shipped_min": 1},
        faults=[{"at_s": 0.3, "robot": "pinky1", "type": "operator_remove"},
                {"at_s": 0.5, "robot": "pinky2", "type": "operator_remove"},
                {"at_s": 1.0, "robot": "pinky2", "type": "operator_reinstate", "start": "P-5"},
                {"at_s": 2.0, "robot": "pinky1", "type": "operator_reinstate", "start": "P-6"}])
    calls = []
    _serve_both(probe, calls)
    assert wait_until(lambda: ("reinstate", "pinky1") in calls, 10.0), calls
    assert ("reinstate", "pinky2") not in calls
