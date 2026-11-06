"""주문 목록과 reinstate 바구니 검사. 주문 쪽 이름이라 traffic이 닿는 common에 두지 않는다."""
from __future__ import annotations

from fleet_core.common.validate import robot_problem, shown

REINSTATE_BASKETS = ("EMPTY", "NONE")  # 재투입 때 트레이 위 상태. 담긴 바구니로는 받지 않는다


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


def reinstate_problem(robot, basket, robots) -> str:
    """reinstate 인자의 로봇과 바구니 값만 본다. 로봇 상태는 traffic이 본다."""
    problem = robot_problem(robot, robots)
    if problem:
        return problem
    if not isinstance(basket, str) or basket not in REINSTATE_BASKETS:
        return f"basket {shown(basket)} is not one of {list(REINSTATE_BASKETS)}"
    return ""
