"""OrderBoard 주문 목록 검사. 주문 쪽 이름이라 traffic이 닿는 common에 두지 않는다."""
from __future__ import annotations

from fleet_core.common.validate import shown


def _name(value) -> bool:
    return isinstance(value, str) and value != ""


def orders_problem(orders) -> str:
    if not isinstance(orders, (list, tuple)):
        return f"orders is a {type(orders).__name__}, not a list"
    seen = set()
    for order in orders:
        order_id = getattr(order, "order_id", None)
        items = getattr(order, "items", None)
        if not _name(order_id):
            return f"an order has the id {shown(order_id)}"
        if order_id in seen:
            return f"order {order_id} is listed twice"
        seen.add(order_id)
        if not isinstance(items, dict) or not items:
            return f"order {order_id} has no items"
        for kind, count in items.items():
            if not _name(kind):
                return f"order {order_id} has an item named {shown(kind)}"
            if not isinstance(count, int) or isinstance(count, bool) or count <= 0:
                return f"order {order_id} wants {shown(count)} of {kind}"
    return ""
