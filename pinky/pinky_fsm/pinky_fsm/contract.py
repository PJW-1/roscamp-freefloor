"""interfaces/fleet_interfaces/contract.yaml에서 이름과 QoS를 읽는다.

pinky_fsm은 pc/*를 import하지 않으므로 관제의 계약 모듈 대신 이 작은 판을 쓴다.
ROS 없이 돈다(QoS 객체를 만들 때만 rclpy를 부른다).
"""
from __future__ import annotations

from pathlib import Path
from typing import NamedTuple

import yaml


class Qos(NamedTuple):
    reliability: str
    durability: str
    depth: int


class Contract:
    def __init__(self, data: dict):
        self._profiles = {name: Qos(p["reliability"], p["durability"], int(p["depth"]))
                          for name, p in (data.get("qos_profiles") or {}).items()}
        self._entries = {}
        for section in ("topics", "actions", "services"):
            for raw in data.get(section) or []:
                self._entries[raw["key"]] = raw

    def name(self, key: str, **subs: str) -> str:
        return self._entries[key]["name"].format(**subs)

    def qos(self, key: str) -> Qos:
        return self._profiles[self._entries[key]["qos"]]


def load_contract(path: str | Path) -> Contract:
    with open(path, encoding="utf-8") as f:
        return Contract(yaml.safe_load(f))


def default_contract_path() -> Path:
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("fleet_interfaces")) / "contract.yaml"


def to_ros_qos(spec: Qos):
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=spec.depth,
                      reliability=ReliabilityPolicy[spec.reliability.upper()],
                      durability=DurabilityPolicy[spec.durability.upper()])
