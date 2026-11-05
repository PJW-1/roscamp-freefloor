#!/usr/bin/env python3
"""Check a running ROS graph against interfaces/fleet_interfaces/contract.yaml.

Fake mode (CI, simulation) skips entries that only exist on real robots; real mode checks
them too. Entries whose provider is intentionally not running are skipped by key or name:

  check_interfaces.py --mode fake --wait-s 5 --skip zones,assignments,confirm_clear

Exit code 0 when there is no error (warnings are printed), 1 otherwise.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
try:
    from fleet_core.contract import QosSpec, load_contract
    from fleet_core.contract_check import compare
except ImportError:  # run straight from a checkout, without colcon
    sys.path.insert(0, str(REPO / "pc" / "fleet_core"))
    from fleet_core.contract import QosSpec, load_contract
    from fleet_core.contract_check import compare


def default_contract() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("fleet_interfaces")) / "contract.yaml"
    except Exception:  # noqa: BLE001 - any lookup failure falls back to the checkout
        return REPO / "interfaces" / "fleet_interfaces" / "contract.yaml"


def _spec(endpoint, expected: QosSpec) -> QosSpec:
    """Depth 0 means the middleware does not share it (Fast DDS), so it cannot be checked."""
    qos = endpoint.qos_profile
    return QosSpec(qos.reliability.name.lower(), qos.durability.name.lower(),
                   int(qos.depth) or expected.depth)


def observe(node, entries) -> dict:
    """Read the parts of the graph the contract talks about into plain data."""
    from rclpy.action import get_action_names_and_types

    topics = {n: t for n, t in node.get_topic_names_and_types()}
    observed = {
        "topics": {},
        "actions": dict(get_action_names_and_types(node)),
        "services": dict(node.get_service_names_and_types()),
    }
    for entry in entries:
        if entry.kind == "topic" and entry.name in topics:
            observed["topics"][entry.name] = {
                "types": topics[entry.name],
                "publishers": [_spec(e, entry.qos)
                               for e in node.get_publishers_info_by_topic(entry.name)],
                "subscribers": [_spec(e, entry.qos)
                                for e in node.get_subscriptions_info_by_topic(entry.name)],
            }
    return observed


def run_check(node, contract, real: bool, skip: set[str]):
    entries = contract.expanded(include_real_only=real)
    skipped = {e.name for e in entries if e.key in skip or e.name in skip}
    problems = compare(contract, observe(node, entries), include_real_only=real)
    return [p for p in problems if p.name not in skipped], skipped, entries


def print_table(problems, checked: int, skipped: int) -> None:
    if problems:
        rows = [("SEVERITY", "KIND", "NAME", "DETAIL")]
        rows += [(p.severity, p.kind, p.name, p.detail) for p in problems]
        widths = [max(len(r[i]) for r in rows) for i in range(3)]
        for r in rows:
            print("  ".join(r[i].ljust(widths[i]) for i in range(3)) + "  " + r[3])
    print(f"checked {checked - skipped} entries ({skipped} skipped): "
          f"{sum(p.severity == 'error' for p in problems)} errors, "
          f"{sum(p.severity == 'warning' for p in problems)} warnings")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--contract", type=Path, default=None)
    parser.add_argument("--mode", choices=("fake", "real"), default="fake")
    parser.add_argument("--wait-s", type=float, default=5.0,
                        help="keep looking this long for entries to appear")
    parser.add_argument("--skip", default="", help="comma separated contract keys or names")
    args = parser.parse_args(argv)

    import rclpy
    contract = load_contract(args.contract or default_contract())
    skip = {s.strip() for s in args.skip.split(",") if s.strip()}
    rclpy.init()
    node = rclpy.create_node("check_interfaces")
    try:
        deadline = time.monotonic() + args.wait_s
        while True:
            problems, skipped, entries = run_check(node, contract, args.mode == "real", skip)
            if not any(p.severity == "error" for p in problems) or time.monotonic() >= deadline:
                break
            rclpy.spin_once(node, timeout_sec=0.25)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    print_table(problems, len(entries), len(skipped))
    return 1 if any(p.severity == "error" for p in problems) else 0


if __name__ == "__main__":
    sys.exit(main())
