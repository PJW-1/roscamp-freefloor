"""The traffic package must stay usable without orders: it knows lanes and robots, not baskets.

Modules are named by their dotted path inside fleet_core: `traffic.controller`,
`orders.orchestrator`, `common.commands`, `manager`.
"""
import ast
from pathlib import Path

import fleet_core

PACKAGE = Path(fleet_core.__file__).parent
TRAFFIC = sorted(f"traffic.{p.stem}" for p in (PACKAGE / "traffic").glob("*.py")
                 if p.stem != "__init__")
ORDER_SIDE_NAMES = {
    "OrderRecord", "FillCmd", "SwapCmd", "ReportOrderCmd", "SHIPPED", "PARTIAL", "HELD",
    "OrderOrchestrator", "FleetManager",
}
ORDER_SIDE_ATTRIBUTES = {"basket", "order", "orders", "placed_items", "last_order_id", "arm"}
# What traffic may take from commands.py. An allow list, so a new shared name is a decision.
TRAFFIC_VOCABULARY = {
    "CancelCmd", "DispatchFeedback", "DispatchResult", "DockCmd", "EventOut", "MoveCmd",
    "NO_ROBOT", "ZoneView",
}


def _path(name):
    """The file of a fleet_core module, or the __init__.py of a package; None if neither."""
    base = PACKAGE / name.replace(".", "/")
    for path in (base.with_suffix(".py"), base / "__init__.py"):
        if path.is_file():
            return path
    return None


def _is_module(name):
    return bool(name) and _path(name) is not None


def _order_side(module):
    return module in ("manager", "orders") or module.startswith("orders.")


def _tree(module):
    return ast.parse(_path(module).read_text(encoding="utf-8"))


def _imports_of(tree, here=""):
    """fleet_core modules a parsed file imports, absolute or relative, and the names it takes.

    `here` is the importing module, needed to resolve a relative import.
    """
    modules, names = set(), set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            if node.level:
                parts = here.split(".") if here else []
                if _is_module(here) and _path(here).name == "__init__.py":
                    parts.append("__init__")  # inside a package, `.` is the package itself
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


def _closure(*modules):
    """Every fleet_core module reachable from `modules`, themselves included."""
    seen, stack = set(), list(modules)
    while stack:
        current = stack.pop()
        if current not in seen:
            seen.add(current)
            stack.extend(_package_imports(current)[0])
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
    for module in _closure(*TRAFFIC) - {"common.commands"}:  # shared vocabulary
        tree = _tree(module)
        used = {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
        used |= {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
        used |= {t.target.id for t in ast.walk(tree)
                 if isinstance(t, ast.AnnAssign) and isinstance(t.target, ast.Name)}
        assert not used & (ORDER_SIDE_ATTRIBUTES | ORDER_SIDE_NAMES), module


def test_the_checks_above_can_fail():
    """The orchestrator is the counterexample: it does reach those names and fields."""
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
        ("from .manager import FleetManager", "probe", "manager"),  # a top-level module
        ("from . import manager", "probe", "manager"),
        ("from fleet_core import orders", "", "orders"),  # the package itself
        ("from .. import orders", "traffic.controller", "orders"),
        ("import fleet_core.orders", "", "orders"),
    ):
        assert module in _imports_of(ast.parse(source), here)[0], source
    assert "OrderOrchestrator" in _imports_of(ast.parse(
        "from ..orders.orchestrator import OrderOrchestrator"), "traffic.controller")[1]


def test_a_package_import_counts_as_the_order_side_and_can_be_followed():
    assert _order_side("orders") and not _order_side("traffic")
    assert "orders" in _closure("orders")  # read through its __init__.py, not orders.py
    assert "traffic.controller" in _imports_of(ast.parse("from . import controller"), "traffic")[0]


def test_orders_do_not_import_traffic():
    """The two talk only through commands that FleetManager carries across."""
    assert not {m for m in _closure("orders.orchestrator") if m.startswith("traffic.")}
