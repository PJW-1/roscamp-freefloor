"""FakeCentral through its real topics and services."""
import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces.msg")

from conftest import CONTRACT, wait_until, write_scenario  # noqa: E402
from fleet_core.common.contract import load_contract, to_ros_qos  # noqa: E402
from fleet_interfaces.msg import FleetEvent, OrderBoard, OrderItem  # noqa: E402
from fleet_interfaces.srv import ReportOrder, SetRunMode, SubmitOrder  # noqa: E402

C = load_contract(CONTRACT)
ORDERS = [
    {"at_s": 0, "id": "O-1", "items": {"A": 1}},
    {"at_s": 0, "id": "O-2", "items": {"C": 2}},
]


class Central:
    def __init__(self, harness, tmp_path, params_file=None, **scenario):
        import rclpy.node
        from fake_robots.fake_central_node import FakeCentralNode
        scenario.setdefault("orders", ORDERS)
        extra = {"params_file": str(params_file)} if params_file else {}
        self.node = harness.add(FakeCentralNode(parameter_overrides=harness.params(
            scenario_file=write_scenario(tmp_path, **scenario), **extra)))
        probe = harness.add(rclpy.node.Node("central_probe"))
        self.boards, self.events = [], []
        probe.create_subscription(OrderBoard, C.name("order_board"), self.boards.append,
                                  to_ros_qos(C.qos("order_board")))
        probe.create_subscription(FleetEvent, C.name("events"), self.events.append,
                                  to_ros_qos(C.qos("events")))
        self.submit_cli = probe.create_client(SubmitOrder, C.name("submit_order"))
        self.report_cli = probe.create_client(ReportOrder, C.name("report_order"))
        self.mode_cli = probe.create_client(SetRunMode, C.name("set_run_mode"))
        for cli in (self.submit_cli, self.report_cli, self.mode_cli):
            assert cli.wait_for_service(5.0)

    def ids(self):
        return [o.order_id for o in self.boards[-1].orders] if self.boards else None

    def wait_ids(self, expected, timeout=8.0):
        assert wait_until(lambda: self.ids() == expected, timeout), self.ids()

    def call(self, cli, request):
        fut = cli.call_async(request)
        assert wait_until(fut.done, 5.0)
        return fut.result()

    def submit(self, **items):
        return self.call(self.submit_cli, SubmitOrder.Request(
            items=[OrderItem(kind=k, count=c) for k, c in items.items()]))

    def report(self, order_id, status, placed=None, missing=None, detail=""):
        return self.call(self.report_cli, ReportOrder.Request(
            order_id=order_id, robot_id="pinky1", status=status,
            placed=[OrderItem(kind=k, count=c) for k, c in (placed or {}).items()],
            missing=[OrderItem(kind=k, count=c) for k, c in (missing or {}).items()],
            detail=detail))

    def codes(self):
        return [e.code for e in self.events]

    def wait_code(self, code, timeout=5.0):
        assert wait_until(lambda: code in self.codes(), timeout), self.codes()
        return next(e for e in self.events if e.code == code)


SHIPPED = ReportOrder.Request.STATUS_SHIPPED
PARTIAL = ReportOrder.Request.STATUS_PARTIAL
HELD = ReportOrder.Request.STATUS_HELD


def test_board_is_oldest_first_with_items_and_run_mode(harness, tmp_path):
    c = Central(harness, tmp_path)
    c.wait_ids(["O-1", "O-2"])
    board = c.boards[-1]
    assert board.run_mode == OrderBoard.MODE_RUN
    assert {i.kind: i.count for i in board.orders[1].items} == {"C": 2}


def test_orders_are_released_at_their_time_in_order(harness, tmp_path):
    c = Central(harness, tmp_path, orders=[
        {"at_s": 2.0, "id": "late", "items": {"A": 1}},
        {"at_s": 0, "id": "early", "items": {"A": 1}},
    ])
    c.wait_ids(["early"])
    c.wait_ids(["early", "late"], timeout=6.0)
    assert [e.detail for e in c.events if e.code == "ORDER_ACCEPTED"][-1] == "late"


def test_set_run_mode_reaches_the_board(harness, tmp_path):
    c = Central(harness, tmp_path)
    assert c.call(c.mode_cli, SetRunMode.Request(mode=OrderBoard.MODE_PAUSE)).accepted
    assert wait_until(lambda: c.boards and c.boards[-1].run_mode == OrderBoard.MODE_PAUSE, 5.0)
    assert not c.call(c.mode_cli, SetRunMode.Request(mode="FAST")).accepted


def test_submit_accepts_and_appends_to_the_board(harness, tmp_path):
    c = Central(harness, tmp_path)
    c.wait_ids(["O-1", "O-2"])
    res = c.submit(A=2, C=1)
    assert res.accepted and res.order_id and res.reason == ""
    c.wait_ids(["O-1", "O-2", res.order_id])
    assert c.wait_code("ORDER_ACCEPTED")


@pytest.mark.parametrize("items", [{"Z": 1}, {"A": 4}, {}, {"A": 0}])
def test_submit_rejects_unknown_out_of_stock_and_empty(harness, tmp_path, items):
    c = Central(harness, tmp_path, orders=[])  # the record says A: 3, B: 1, C: 3
    res = c.submit(**items)
    assert not res.accepted and res.reason and res.order_id == ""
    c.wait_code("ORDER_REJECTED")
    assert c.ids() in ([], None)


def test_submit_goes_by_the_record_not_the_shelves(harness, tmp_path):
    c = Central(harness, tmp_path, orders=[], inventory={"A": 0, "E": 2})
    assert not c.submit(A=1).accepted  # on the shelf, record says none
    assert c.submit(E=1).accepted  # not on the shelf, record says some


def test_submit_rejects_over_capacity(harness, tmp_path):
    c = Central(harness, tmp_path, orders=[], stock={"arm1": {"A": 9}, "arm2": {"C": 9}})
    res = c.submit(A=3, C=2)  # 5 is over the capacity of 4 in fleet_params.yaml
    assert not res.accepted and "fit" in res.reason


def test_capacity_comes_from_the_params_file(harness, tmp_path):
    from fake_robots.common import DEFAULT_PARAMS
    params = tmp_path / "params.yaml"
    params.write_text(DEFAULT_PARAMS.read_text(encoding="utf-8").replace(
        "basket_capacity: 4", "basket_capacity: 2"), encoding="utf-8")
    c = Central(harness, tmp_path, params_file=params, orders=[],
                stock={"arm1": {"A": 9}, "arm2": {"C": 9}})
    assert c.submit(A=2).accepted
    assert not c.submit(A=3).accepted


def test_scenario_orders_skip_the_submit_checks(harness, tmp_path):
    c = Central(harness, tmp_path, inventory={"A": 0}, orders=[
        {"at_s": 0, "id": "big", "items": {"Z": 9}}])
    c.wait_ids(["big"])


def test_shipped_report_removes_the_order_and_deducts(harness, tmp_path):
    c = Central(harness, tmp_path)
    c.wait_ids(["O-1", "O-2"])
    assert c.report("O-1", SHIPPED, placed={"A": 1}).accepted
    c.wait_ids(["O-2"])
    assert c.node.inventory()["A"] == 2
    event = c.wait_code("ORDER_SHIPPED")
    assert event.detail.startswith("O-1")
    assert "OUT_OF_STOCK" not in c.codes() and "STOCK_MISMATCH" not in c.codes()


def test_report_of_an_unknown_order_or_status_is_refused(harness, tmp_path):
    c = Central(harness, tmp_path)
    c.wait_ids(["O-1", "O-2"])
    assert not c.report("nope", SHIPPED).accepted
    assert not c.report("O-1", "LOST").accepted
    c.wait_ids(["O-1", "O-2"])


def test_a_repeated_report_changes_nothing(harness, tmp_path):
    c = Central(harness, tmp_path)
    c.wait_ids(["O-1", "O-2"])
    assert c.report("O-1", SHIPPED, placed={"A": 1}).accepted
    assert c.report("O-1", SHIPPED, placed={"A": 1}).accepted
    c.wait_code("ORDER_SHIPPED")
    assert c.node.inventory()["A"] == 2
    assert c.codes().count("ORDER_SHIPPED") == 1


def test_missing_with_an_empty_record_is_out_of_stock(harness, tmp_path):
    c = Central(harness, tmp_path, inventory={"A": 1, "D": 0}, orders=[
        {"at_s": 0, "id": "O-1", "items": {"A": 1, "D": 1}}])
    c.wait_ids(["O-1"])
    assert c.report("O-1", PARTIAL, placed={"A": 1}, missing={"D": 1}).accepted
    assert c.wait_code("OUT_OF_STOCK").detail.startswith("O-1")
    assert c.wait_code("ORDER_PARTIAL").detail.startswith("O-1")
    assert "STOCK_MISMATCH" not in c.codes()
    assert c.node.inventory() == {"A": 0, "D": 0}


def test_missing_with_some_in_the_record_is_a_mismatch_and_gets_corrected(harness, tmp_path):
    c = Central(harness, tmp_path, inventory={"E": 3}, orders=[
        {"at_s": 0, "id": "O-1", "items": {"E": 1}}])
    c.wait_ids(["O-1"])
    assert c.report("O-1", HELD, missing={"E": 1}).accepted
    assert c.wait_code("STOCK_MISMATCH").detail.startswith("O-1")
    assert c.wait_code("ORDER_HELD").detail.startswith("O-1")
    assert "OUT_OF_STOCK" not in c.codes()
    assert c.node.inventory() == {"E": 0}


def test_after_a_correction_the_same_kind_is_out_of_stock(harness, tmp_path):
    c = Central(harness, tmp_path, inventory={"E": 3}, orders=[
        {"at_s": 0, "id": "O-1", "items": {"E": 1}},
        {"at_s": 0, "id": "O-2", "items": {"E": 1}}])
    c.wait_ids(["O-1", "O-2"])
    c.report("O-1", HELD, missing={"E": 1})
    c.report("O-2", HELD, missing={"E": 1})
    c.wait_code("OUT_OF_STOCK")
    assert c.codes().count("STOCK_MISMATCH") == 1


def test_placed_is_deducted_before_missing_is_judged(harness, tmp_path):
    """The record had exactly what was placed, so the shortfall is not a mismatch."""
    c = Central(harness, tmp_path, inventory={"A": 1}, orders=[
        {"at_s": 0, "id": "O-1", "items": {"A": 2}}])
    c.wait_ids(["O-1"])
    c.report("O-1", PARTIAL, placed={"A": 1}, missing={"A": 1})
    c.wait_code("OUT_OF_STOCK")
    assert "STOCK_MISMATCH" not in c.codes()


def test_each_missing_kind_gets_its_own_code(harness, tmp_path):
    c = Central(harness, tmp_path, inventory={"D": 0, "E": 1}, orders=[
        {"at_s": 0, "id": "O-1", "items": {"D": 1, "E": 1}}])
    c.wait_ids(["O-1"])
    c.report("O-1", HELD, missing={"D": 1, "E": 1})
    c.wait_code("ORDER_HELD")
    by_code = {e.code: e.detail for e in c.events}
    assert " D " in by_code["OUT_OF_STOCK"] and " E " in by_code["STOCK_MISMATCH"]
