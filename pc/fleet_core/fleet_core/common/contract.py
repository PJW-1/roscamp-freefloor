"""인터페이스 계약(토픽, 액션, 서비스의 이름, 타입, QoS)을 읽는다.

노드는 `key`로 찾으므로 이름이나 QoS는 contract.yaml만 고치면 된다. ROS 없이 pytest에서 돈다.
"""
from __future__ import annotations

import itertools
import string
from dataclasses import dataclass
from pathlib import Path

import yaml


class ContractError(ValueError):
    """계약 파일이 잘못됐거나 항목을 잘못 요청했다."""


@dataclass(frozen=True)
class QosSpec:
    reliability: str
    durability: str
    depth: int


@dataclass(frozen=True)
class Entry:
    key: str
    kind: str  # topic, action, service 중 하나
    template: str
    type: str
    qos_name: str | None
    instances: dict
    real_only: bool


@dataclass(frozen=True)
class ExpandedEntry:
    name: str
    key: str
    kind: str
    type: str
    qos: QosSpec | None


def _fields(template: str) -> list[str]:
    return [f for _, f, _, _ in string.Formatter().parse(template) if f]


class Contract:
    def __init__(self, data: dict):
        self.placeholders = {k: list(v) for k, v in (data.get("placeholders") or {}).items()}
        self.profiles = {
            name: QosSpec(p["reliability"], p["durability"], int(p["depth"]))
            for name, p in (data.get("qos_profiles") or {}).items()
        }
        self.entries: dict[str, Entry] = {}
        for kind, section in (("topic", "topics"), ("action", "actions"), ("service", "services")):
            for raw in data.get(section) or []:
                self._add(kind, raw)

    def _add(self, kind: str, raw: dict) -> None:
        key = raw["key"]
        if key in self.entries:
            raise ContractError(f"duplicate key {key}")
        qos = raw.get("qos")
        if kind == "topic" and qos not in self.profiles:
            raise ContractError(f"{key}: undefined QoS profile {qos}")
        self.entries[key] = Entry(
            key=key,
            kind=kind,
            template=raw["name"],
            type=raw["type"],
            qos_name=qos,
            instances=raw.get("instances") or {},
            real_only=bool(raw.get("real_only", False)),
        )

    def _entry(self, key: str) -> Entry:
        try:
            return self.entries[key]
        except KeyError:
            raise ContractError(f"unknown contract key {key}") from None

    def name(self, key: str, **subs: str) -> str:
        entry = self._entry(key)
        try:
            return entry.template.format(**subs)
        except KeyError as exc:
            raise ContractError(f"{key}: missing placeholder {exc}") from None

    def type(self, key: str) -> str:
        return self._entry(key).type

    def qos(self, key: str) -> QosSpec:
        entry = self._entry(key)
        if entry.qos_name is None:
            raise ContractError(f"{key}: {entry.kind} entries have no QoS profile")
        return self.profiles[entry.qos_name]

    def expanded(self, include_real_only: bool = True) -> list[ExpandedEntry]:
        """실제 이름마다 하나씩 돌려준다. 실행 중인 시스템 점검용."""
        out = []
        for entry in self.entries.values():
            if entry.real_only and not include_real_only:
                continue
            fields = _fields(entry.template)
            values = [entry.instances.get(f, self.placeholders.get(f, [])) for f in fields]
            qos = self.profiles[entry.qos_name] if entry.qos_name else None
            for combo in itertools.product(*values):
                name = entry.template.format(**dict(zip(fields, combo, strict=True)))
                out.append(ExpandedEntry(name, entry.key, entry.kind, entry.type, qos))
        return out


def load_contract(path) -> Contract:
    return Contract(yaml.safe_load(Path(path).read_text(encoding="utf-8")))


def to_ros_qos(spec: QosSpec):
    """rclpy QoSProfile을 만든다. pytest에 rclpy가 없어도 되게 여기서 import한다."""
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=spec.depth,
        reliability=ReliabilityPolicy[spec.reliability.upper()],
        durability=DurabilityPolicy[spec.durability.upper()],
    )
