"""traffic_core must stay usable without orders: it knows lanes and robots, not baskets."""
import ast
from pathlib import Path

import fleet_core

PACKAGE = Path(fleet_core.__file__).parent
ORDER_SIDE_MODULES = {"orchestrator_core", "fms_core"}
ORDER_SIDE_NAMES = {
    "OrderRecord", "FillCmd", "SwapCmd", "ReportOrderCmd", "SHIPPED", "PARTIAL", "HELD",
    "OrchestratorCore", "FmsCore",
}
ORDER_SIDE_ATTRIBUTES = {"basket", "order", "orders", "placed_items", "last_order_id", "arm"}
# What traffic may take from commands.py. An allow list, so a new shared name is a decision.
TRAFFIC_VOCABULARY = {
    "CancelCmd", "DispatchFeedback", "DispatchResult", "DockCmd", "EventOut", "MoveCmd",
    "NO_ROBOT", "ZoneView",
}


def _tree(module):
    return ast.parse((PACKAGE / f"{module}.py").read_text(encoding="utf-8"))


def _imports_of(tree):
    """Fleet_core modules a parsed file imports, absolute or relative, and the names it takes."""
    modules, names = set(), set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            if node.level:  # from . import x, from .x import y
                inside = [node.module] if node.module else [a.name for a in node.names]
            elif (node.module or "").startswith("fleet_core."):
                inside = [node.module.split(".")[1]]
            elif node.module == "fleet_core":
                inside = [alias.name for alias in node.names]
            else:
                continue
            modules.update(name.split(".")[0] for name in inside)
            if node.module and node.module != "fleet_core":
                names.update(alias.name for alias in node.names)
        elif isinstance(node, ast.Import):
            modules.update(alias.name.split(".")[1] for alias in node.names
                           if alias.name.startswith("fleet_core."))
    return modules, names


def _package_imports(module):
    return _imports_of(_tree(module))


def _closure(module):
    """Every fleet_core module reachable from `module`, itself included."""
    seen, stack = set(), [module]
    while stack:
        current = stack.pop()
        if current not in seen:
            seen.add(current)
            stack.extend(_package_imports(current)[0])
    return seen


def test_traffic_core_does_not_reach_the_order_side_through_any_import():
    assert not _closure("traffic_core") & ORDER_SIDE_MODULES


def test_traffic_core_takes_no_order_or_basket_type_from_shared_modules():
    for module in _closure("traffic_core"):
        assert not _package_imports(module)[1] & ORDER_SIDE_NAMES, module


def test_traffic_core_takes_only_its_own_vocabulary_from_commands():
    taken = set()
    for node in ast.walk(_tree("traffic_core")):
        if isinstance(node, ast.ImportFrom) and (node.module or "").endswith("commands"):
            taken.update(alias.name for alias in node.names)
    assert taken == TRAFFIC_VOCABULARY


def test_traffic_core_has_no_order_or_basket_field():
    for module in _closure("traffic_core") - {"commands"}:  # commands.py is shared vocabulary
        tree = _tree(module)
        used = {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
        used |= {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
        used |= {t.target.id for t in ast.walk(tree)
                 if isinstance(t, ast.AnnAssign) and isinstance(t.target, ast.Name)}
        assert not used & (ORDER_SIDE_ATTRIBUTES | ORDER_SIDE_NAMES), module


def test_the_checks_above_can_fail():
    """The orchestrator is the counterexample: it does reach those names and fields."""
    assert "basket" in {n.attr for n in ast.walk(_tree("orchestrator_core"))
                        if isinstance(n, ast.Attribute)}
    assert _package_imports("orchestrator_core")[1] & ORDER_SIDE_NAMES
    assert "traffic_core" in _closure("fms_core") and "orchestrator_core" in _closure("fms_core")


def test_every_spelling_of_an_import_is_seen():
    for source, module in (
        ("from fleet_core.orchestrator_core import OrchestratorCore", "orchestrator_core"),
        ("from fleet_core import orchestrator_core", "orchestrator_core"),
        ("import fleet_core.orchestrator_core", "orchestrator_core"),
        ("from .orchestrator_core import OrchestratorCore", "orchestrator_core"),
        ("from . import orchestrator_core", "orchestrator_core"),
        ("from .fms_core import FmsCore", "fms_core"),
    ):
        assert module in _imports_of(ast.parse(source))[0], source
    assert "OrchestratorCore" in _imports_of(ast.parse(
        "from .orchestrator_core import OrchestratorCore"))[1]


def test_orchestrator_core_does_not_import_traffic_core():
    """The two talk only through commands that FmsCore carries across."""
    assert "traffic_core" not in _closure("orchestrator_core")
