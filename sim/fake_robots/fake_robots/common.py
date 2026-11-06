"""Shared setup for the fake nodes: contract, map, scenario, params, scenario clock."""
from __future__ import annotations

import time
from pathlib import Path

import yaml
from fleet_core.common.contract import load_contract
from fleet_core.traffic.lane_map import DEFAULT_MAP, load_lane_map

from fake_robots.scenario import load_scenario

# Fake-internal topics, intentionally not in the contract.
STOCK_TOPIC = "/sim/stock/{arm}"  # the true stock of one arm's item area, as JSON
TRUTH_TOPIC = "/sim/truth/{robot}"  # noiseless pose for the judge

DEFAULT_PARAMS = DEFAULT_MAP.parent / "fleet_params.yaml"


def node_name(prefix: str, key: str, overrides) -> str:
    """Name a node after its robot_id/arm_id override so several fit in one process."""
    for p in overrides or []:
        if p.name == key:
            return f"{prefix}_{p.value}"
    return prefix


def load_contract_param(node):
    node.declare_parameter("contract_file", "")
    path = node.get_parameter("contract_file").value
    if not path:
        from ament_index_python.packages import get_package_share_directory
        path = get_package_share_directory("fleet_interfaces") + "/contract.yaml"
    return load_contract(path)


def load_lane_map_param(node):
    node.declare_parameter("map_file", "")
    return load_lane_map(node.get_parameter("map_file").value or None)


def load_scenario_param(node, lane_map):
    node.declare_parameter("scenario_file", "")
    path = node.get_parameter("scenario_file").value
    if not path:
        raise ValueError("parameter scenario_file is required")
    return load_scenario(path, lane_map)


def load_fleet_params(path=None) -> dict:
    """Read the agreed values from the params file, as plain data."""
    source = Path(path) if path else DEFAULT_PARAMS
    return yaml.safe_load(source.read_text(encoding="utf-8"))


def load_params_param(node) -> dict:
    node.declare_parameter("params_file", "")
    return load_fleet_params(node.get_parameter("params_file").value or None)


def station_with_action(site, action: str):
    for station in site.stations.values():
        if station.action == action:
            return station
    raise ValueError(f"no station in the map has action {action!r}")


def split_fake_arms(text: str, site) -> tuple[bool, list[str], list[str]]:
    """Read launch argument fake_arms: (run the fake workcell, vision arms, swap arms).

    arm1 and arm2 sit behind one fill server, so they are faked together or not at all.
    """
    fill_arms = list(station_with_action(site, "fill").arms)
    swap_arms = list(station_with_action(site, "swap").arms)
    named = text.split()
    unknown = sorted(set(named) - set(fill_arms) - set(swap_arms))
    if unknown:
        raise ValueError(f"fake_arms names unknown arms: {unknown}")
    listed = [a for a in fill_arms if a in named]
    if listed and listed != fill_arms:
        raise ValueError(
            f"fake_arms must list all of {fill_arms} or none of them: one workcell server "
            f"answers for both, got {listed}")
    return bool(listed), listed, [a for a in swap_arms if a in named]


class ScenarioClock:
    """Scenario time in seconds. With sim time the ROS clock already starts at zero."""

    def __init__(self, node):
        self._node = node
        self._sim = bool(node.get_parameter("use_sim_time").value)
        self._base = self._ros_now()

    def _ros_now(self) -> float:
        return self._node.get_clock().now().nanoseconds * 1e-9

    def now(self) -> float:
        return self._ros_now() - (0.0 if self._sim else self._base)

    def sleep_until(self, deadline: float, alive) -> bool:
        """Block the calling thread until scenario time reaches deadline. False if shut down."""
        while self.now() < deadline:
            if not alive():
                return False
            time.sleep(0.01)
        return True
