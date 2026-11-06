"""계약과 실행 중인 ROS 그래프를 비교한다. ROS는 import하지 않는다.

그래프 없이 테스트하려고 `observed`는 순수 데이터다:
  {"topics": {name: {"types": [...], "publishers": [QosSpec], "subscribers": [QosSpec]}},
   "actions": {name: [types]}, "services": {name: [types]}}
"""
from __future__ import annotations

from dataclasses import dataclass

from fleet_core.common.contract import QosSpec


@dataclass(frozen=True)
class Problem:
    kind: str  # missing, wrong_type, qos 중 하나
    name: str
    detail: str
    severity: str = "error"  # error 또는 warning


def _norm(type_name: str) -> str:
    """`pkg/msg/Name`과 `pkg/Name`을 둘 다 받는다."""
    parts = type_name.split("/")
    return f"{parts[0]}/{parts[-1]}" if len(parts) == 3 else type_name


def _type_ok(expected: str, actual: list[str]) -> bool:
    return _norm(expected) in {_norm(t) for t in actual}


def _qos_problems(name: str, expected: QosSpec, role: str, specs: list[QosSpec]) -> list[Problem]:
    out = []
    for spec in specs:
        for attr in ("reliability", "durability"):
            if getattr(spec, attr) != getattr(expected, attr):
                out.append(Problem(
                    "qos", name,
                    f"{role} {attr} is {getattr(spec, attr)}, contract says "
                    f"{getattr(expected, attr)}"))
        if spec.depth != expected.depth:
            out.append(Problem(
                "qos", name, f"{role} depth is {spec.depth}, contract says {expected.depth}",
                "warning" if role == "subscriber" else "error"))
    return out


def compare(contract, observed: dict, include_real_only: bool = False) -> list[Problem]:
    problems: list[Problem] = []
    for entry in contract.expanded(include_real_only=include_real_only):
        if entry.kind == "topic":
            seen = observed.get("topics", {}).get(entry.name)
            types = seen["types"] if seen else []
        else:
            section = observed.get("actions" if entry.kind == "action" else "services", {})
            seen = section.get(entry.name)
            types = seen or []
        if seen is None:
            problems.append(Problem("missing", entry.name, f"{entry.kind} {entry.type}"))
            continue
        if not _type_ok(entry.type, types):
            problems.append(Problem(
                "wrong_type", entry.name, f"is {', '.join(types) or 'unknown'}, "
                                          f"contract says {entry.type}"))
            continue
        if entry.kind == "topic":
            problems += _qos_problems(entry.name, entry.qos, "publisher", seen["publishers"])
            problems += _qos_problems(entry.name, entry.qos, "subscriber", seen["subscribers"])
    return problems
