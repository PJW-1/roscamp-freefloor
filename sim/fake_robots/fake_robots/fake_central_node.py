"""FakeCentral: stand-in for the central system. Holds the inventory record and the orders."""
from __future__ import annotations

import threading
from collections import Counter
from dataclasses import dataclass

import rclpy
from fleet_core.contract import to_ros_qos
from fleet_interfaces.msg import FleetEvent, Order, OrderBoard, OrderItem
from fleet_interfaces.srv import ReportOrder, SetRunMode, SubmitOrder
from rclpy.node import Node

from fake_robots.common import (
    ScenarioClock,
    load_contract_param,
    load_params_param,
    load_scenario_param,
    load_site_map_param,
)

MODES = (OrderBoard.MODE_RUN, OrderBoard.MODE_PAUSE, OrderBoard.MODE_STOP)
SOURCE = "central"
FINAL_EVENTS = {
    ReportOrder.Request.STATUS_SHIPPED: ("INFO", "ORDER_SHIPPED"),
    ReportOrder.Request.STATUS_PARTIAL: ("WARN", "ORDER_PARTIAL"),
    ReportOrder.Request.STATUS_HELD: ("WARN", "ORDER_HELD"),
}


@dataclass
class _Entry:
    order_id: str
    items: dict
    created_at: object


def _text(items) -> str:
    return ",".join(f"{i.kind}:{i.count}" for i in items) or "-"


class FakeCentralNode(Node):
    def __init__(self, parameter_overrides=None, **kwargs):
        super().__init__("fake_central", parameter_overrides=parameter_overrides, **kwargs)
        contract = load_contract_param(self)
        site = load_site_map_param(self)
        scenario = load_scenario_param(self, site)
        self._capacity = int(load_params_param(self)["basket_capacity"])
        self._sc = ScenarioClock(self)
        self._lock = threading.Lock()
        self._mode = OrderBoard.MODE_RUN
        self._record = dict(scenario.inventory)
        self._waiting_release = list(scenario.orders)  # registration order
        self._board: list[_Entry] = []  # oldest first
        self._final: set[str] = set()
        self._known_ids = set(scenario.order_ids)
        self._submitted = 0

        self._pub = self.create_publisher(
            OrderBoard, contract.name("order_board"), to_ros_qos(contract.qos("order_board")))
        self._events = self.create_publisher(
            FleetEvent, contract.name("events"), to_ros_qos(contract.qos("events")))
        self.create_service(SetRunMode, contract.name("set_run_mode"), self._set_mode)
        self.create_service(SubmitOrder, contract.name("submit_order"), self._submit)
        self.create_service(ReportOrder, contract.name("report_order"), self._report)
        self.create_timer(0.1, self._release_due)
        self.create_timer(1.0, self._publish)
        self._release_due()
        self._publish()

    def inventory(self) -> dict:
        with self._lock:
            return dict(self._record)

    def _event(self, level: str, code: str, detail: str) -> None:
        self._events.publish(FleetEvent(level=level, source=SOURCE, code=code, detail=detail,
                                        stamp=self.get_clock().now().to_msg()))

    def _add(self, order_id: str, items: dict) -> None:
        self._board.append(_Entry(order_id, items, self.get_clock().now().to_msg()))

    def _release_due(self) -> None:
        """Scenario orders skip the submit checks: an order the record cannot cover is a test."""
        t = self._sc.now()
        released = []
        with self._lock:
            while self._waiting_release and self._waiting_release[0].at_s <= t:
                order = self._waiting_release.pop(0)
                self._add(order.id, dict(order.items))
                released.append(order.id)
        for order_id in released:
            self._event("INFO", "ORDER_ACCEPTED", order_id)
        if released:
            self._publish()

    def _reject_reason(self, items: Counter) -> str:
        """Judged by the record as it stands now; orders on the board do not reserve anything."""
        if not items:
            return "the order has no items"
        if min(items.values()) <= 0:
            return "every item needs a count of at least 1"
        total = sum(items.values())
        if total > self._capacity:
            return f"{total} items do not fit a basket of {self._capacity}"
        for kind, count in sorted(items.items()):
            if kind not in self._record:
                return f"unknown item kind {kind}"
            if self._record[kind] < count:
                return f"{kind}: {count} requested, {self._record[kind]} in the record"
        return ""

    def _submit(self, request, response):
        items: Counter = Counter()
        for item in request.items:
            items[item.kind] += item.count
        with self._lock:
            reason = self._reject_reason(items)
            if not reason:
                self._submitted += 1
                order_id = f"O-U{self._submitted:03d}"
                while order_id in self._known_ids:
                    self._submitted += 1
                    order_id = f"O-U{self._submitted:03d}"
                self._known_ids.add(order_id)
                self._add(order_id, dict(items))
        response.accepted = not reason
        response.reason = reason
        if reason:
            self._event("WARN", "ORDER_REJECTED", f"{_text(request.items)}: {reason}")
        else:
            response.order_id = order_id
            self._event("INFO", "ORDER_ACCEPTED", order_id)
            self._publish()
        return response

    def _settle_stock(self, request) -> list[tuple[str, str, str]]:
        """Deduct what was placed and correct what turned out to be missing. Lock held."""
        out = []
        for item in request.placed:
            if item.kind in self._record:
                self._record[item.kind] = max(0, self._record[item.kind] - item.count)
        for item in request.missing:
            left = self._record.get(item.kind, 0)
            text = f"{request.order_id} {item.kind} missing {item.count}, record {left}"
            out.append(("WARN", "STOCK_MISMATCH" if left > 0 else "OUT_OF_STOCK", text))
            if item.kind in self._record:
                self._record[item.kind] = 0  # the arm found none, so there are none
        return out

    def _report(self, request, response):
        final = FINAL_EVENTS.get(request.status)
        with self._lock:
            entry = next((e for e in self._board if e.order_id == request.order_id), None)
            if final is None or (entry is None and request.order_id not in self._final):
                response.accepted = False
                return response
            response.accepted = True
            if entry is None:  # a repeated report changes nothing
                return response
            self._board.remove(entry)
            self._final.add(request.order_id)
            events = self._settle_stock(request)
        for level, code, text in events:
            self._event(level, code, text)
        level, code = final
        self._event(level, code, f"{request.order_id} robot={request.robot_id} "
                                 f"placed={_text(request.placed)} "
                                 f"missing={_text(request.missing)} {request.detail}".rstrip())
        self._publish()
        return response

    def _set_mode(self, request, response):
        response.accepted = request.mode in MODES
        if response.accepted:
            with self._lock:
                self._mode = request.mode
            self._publish()
        else:
            response.reason = f"unknown mode {request.mode!r}"
        return response

    def _publish(self) -> None:
        with self._lock:
            msg = OrderBoard(
                run_mode=self._mode,
                orders=[Order(order_id=e.order_id, created_at=e.created_at,
                              items=[OrderItem(kind=k, count=c) for k, c in e.items.items()])
                        for e in self._board],
                stamp=self.get_clock().now().to_msg())
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeCentralNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:  # SIGINT can land inside a message take; a real error has ROS still up
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
