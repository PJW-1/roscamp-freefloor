"""traffic 패키지는 orders 없이 쓸 수 있어야 한다. 레인과 로봇만 알고 바구니는 모른다.

모듈 이름은 fleet_core 안의 점 경로다: `traffic.controller`, `orders.orchestrator`,
`common.commands`, `manager`.
"""
import ast
from pathlib import Path

import fleet_core

PACKAGE = Path(fleet_core.__file__).parent


def _package(name):
    """패키지 자신(__init__.py)과 그 안의 모듈."""
    return [name] + sorted(f"{name}.{p.stem}" for p in (PACKAGE / name).glob("*.py")
                           if p.stem != "__init__")


TRAFFIC = _package("traffic")
ORDERS = _package("orders")
ORDER_SIDE_NAMES = {
    "OrderRecord", "FillCmd", "SwapCmd", "ReportOrderCmd", "SHIPPED", "PARTIAL", "HELD",
    "OrderOrchestrator", "FleetManager",
}
ORDER_SIDE_ATTRIBUTES = {"basket", "order", "orders", "placed_items", "last_order_id", "arm"}
# traffic이 commands.py에서 가져올 수 있는 이름. 허용 목록이라 추가는 결정 사항이다.
TRAFFIC_VOCABULARY = {
    "CancelCmd", "DispatchFeedback", "DispatchResult", "DockCmd", "EventOut", "MoveCmd",
    "NO_ROBOT", "ZoneView",
}


def _path(name):
    """fleet_core 모듈의 파일, 패키지면 __init__.py. 둘 다 없으면 None."""
    base = PACKAGE / name.replace(".", "/")
    for path in (base.with_suffix(".py"), base / "__init__.py"):
        if path.is_file():
            return path
    return None


def _is_module(name):
    return bool(name) and _path(name) is not None


def _order_side(module):
    return module in ("manager", "orders") or module.startswith("orders.")


def _traffic_side(module):
    return module == "traffic" or module.startswith("traffic.")


def _tree(module):
    return ast.parse(_path(module).read_text(encoding="utf-8"))


def _imports_of(tree, here=""):
    """파싱한 파일이 임포트하는 fleet_core 모듈(절대/상대)과 가져오는 이름.

    `here`는 상대 임포트를 풀 때 쓰는, 임포트하는 쪽 모듈.
    """
    modules, names = set(), set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            if node.level:
                parts = here.split(".") if here else []
                if _is_module(here) and _path(here).name == "__init__.py":
                    parts.append("__init__")  # 패키지 안에서 `.`은 패키지 자신
                parts = parts[:-node.level]
                base = ".".join(parts + ([node.module] if node.module else []))
            elif (node.module or "").startswith("fleet_core."):
                base = node.module.split(".", 1)[1]
            elif node.module == "fleet_core":
                base = ""
            else:
                continue
            for alias in node.names:
                inner = f"{base}.{alias.name}" if base else alias.name
                if _is_module(inner):
                    modules.add(inner)
                elif _is_module(base):
                    modules.add(base)
                    names.add(alias.name)
        elif isinstance(node, ast.Import):
            modules.update(alias.name.split(".", 1)[1] for alias in node.names
                           if alias.name.startswith("fleet_core."))
    return modules, names


def _package_imports(module):
    return _imports_of(_tree(module), module)


def _parents(name):
    """임포트할 때 먼저 실행되는 위 패키지. 맨 위 fleet_core는 ""."""
    parts = name.split(".")
    return [".".join(parts[:i]) for i in range(len(parts))]


def _closure(*modules):
    """`modules`에서 닿는 모든 fleet_core 모듈(자신과 위 패키지 포함)."""
    seen, stack = set(), list(modules)
    while stack:
        current = stack.pop()
        if current not in seen:
            seen.add(current)
            stack.extend(_package_imports(current)[0])
            stack.extend(_parents(current))
    return seen


def test_the_traffic_package_is_found():
    assert {"traffic.controller", "traffic.mission", "traffic.recovery",
            "traffic.zones"} <= set(TRAFFIC)


def test_traffic_does_not_reach_the_order_side_through_any_import():
    assert not {m for m in _closure(*TRAFFIC) if _order_side(m)}


def test_traffic_takes_no_order_or_basket_type_from_shared_modules():
    for module in _closure(*TRAFFIC):
        assert not _package_imports(module)[1] & ORDER_SIDE_NAMES, module


def test_traffic_takes_only_its_own_vocabulary_from_commands():
    taken = set()
    for module in TRAFFIC:
        for node in ast.walk(_tree(module)):
            if isinstance(node, ast.ImportFrom) and (node.module or "").endswith("commands"):
                taken.update(alias.name for alias in node.names)
    assert taken == TRAFFIC_VOCABULARY


def test_traffic_has_no_order_or_basket_field():
    for module in _closure(*TRAFFIC) - {"common.commands"}:  # 공유 어휘
        tree = _tree(module)
        used = {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
        used |= {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
        used |= {t.target.id for t in ast.walk(tree)
                 if isinstance(t, ast.AnnAssign) and isinstance(t.target, ast.Name)}
        assert not used & (ORDER_SIDE_ATTRIBUTES | ORDER_SIDE_NAMES), module


def test_the_checks_above_can_fail():
    """반례는 orchestrator다. 그 이름과 필드에 실제로 닿는다."""
    assert "basket" in {n.attr for n in ast.walk(_tree("orders.orchestrator"))
                        if isinstance(n, ast.Attribute)}
    assert _package_imports("orders.orchestrator")[1] & ORDER_SIDE_NAMES
    assert {"traffic.controller", "orders.orchestrator"} <= _closure("manager")


def test_every_spelling_of_an_import_is_seen():
    for source, here, module in (
        ("from fleet_core.orders.orchestrator import OrderOrchestrator", "",
         "orders.orchestrator"),
        ("from fleet_core.orders import orchestrator", "", "orders.orchestrator"),
        ("import fleet_core.orders.orchestrator", "", "orders.orchestrator"),
        ("from ..orders.orchestrator import OrderOrchestrator", "traffic.controller",
         "orders.orchestrator"),
        ("from ..orders import orchestrator", "traffic.controller", "orders.orchestrator"),
        ("from .manager import FleetManager", "probe", "manager"),  # 최상위 모듈
        ("from . import manager", "probe", "manager"),
        ("from fleet_core import orders", "", "orders"),  # 패키지 자체
        ("from .. import orders", "traffic.controller", "orders"),
        ("import fleet_core.orders", "", "orders"),
    ):
        assert module in _imports_of(ast.parse(source), here)[0], source
    assert "OrderOrchestrator" in _imports_of(ast.parse(
        "from ..orders.orchestrator import OrderOrchestrator"), "traffic.controller")[1]


def test_a_package_import_counts_as_the_order_side_and_can_be_followed():
    assert _order_side("orders") and not _order_side("traffic")
    assert "orders" in _closure("orders")  # `orders.py`가 아니라 `__init__.py`를 통해 읽는다
    assert "traffic.controller" in _imports_of(ast.parse("from . import controller"), "traffic")[0]


def test_orders_do_not_import_traffic():
    """둘은 FleetManager가 실어 나르는 명령으로만 대화한다."""
    assert not {m for m in _closure(*ORDERS) if _traffic_side(m)}

def test_a_module_brings_in_the_packages_above_it():
    """서브모듈을 임포트하면 위 패키지의 __init__.py도 실행된다. 맨 위 fleet_core는 ""."""
    assert {"traffic", ""} <= _closure("traffic.controller")
    assert "" in _closure("manager")


def test_both_sides_include_their_package_init():
    assert "traffic" in TRAFFIC and "orders" in ORDERS
    assert _traffic_side("traffic") and _traffic_side("traffic.zones")
    assert not _traffic_side("orders") and not _traffic_side("")
