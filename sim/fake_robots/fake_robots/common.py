"""가짜 노드 공통 설정: 계약, 지도, 시나리오, 파라미터, 시나리오 시계."""
from __future__ import annotations

import time
from pathlib import Path

import yaml
from fleet_core.common.contract import load_contract
from fleet_core.traffic.lane_map import DEFAULT_MAP, load_lane_map

from fake_robots.scenario import load_scenario

# 계약에 일부러 넣지 않은 가짜 내부 토픽.
STOCK_TOPIC = "/sim/stock/{arm}"  # 팔 하나의 실제 재고(JSON)
TRUTH_TOPIC = "/sim/truth/{robot}"  # 심판용 잡음 없는 자세

DEFAULT_PARAMS = DEFAULT_MAP.parent / "fleet_params.yaml"


def node_name(prefix: str, key: str, overrides) -> str:
    """robot_id/arm_id 값을 이름에 붙여 한 프로세스에 여러 개를 띄운다."""
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
    """scenario_seed: random 블록을 풀 seed. -1(기본)이면 파일의 seed. 같은 파일을 읽는 노드는
    모두 같은 값을 받아야 같은 시간표를 본다."""
    for name, default in (("scenario_file", ""), ("scenario_seed", -1)):
        if not node.has_parameter(name):
            node.declare_parameter(name, default)
    path = node.get_parameter("scenario_file").value
    if not path:
        raise ValueError("parameter scenario_file is required")
    seed = int(node.get_parameter("scenario_seed").value)
    return load_scenario(path, lane_map, None if seed < 0 else seed)


def load_fleet_params(path=None) -> dict:
    """합의된 값을 파라미터 파일에서 읽는다."""
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
    """launch 인자 fake_arms를 (가짜 작업셀 실행 여부, 비전 팔, 교체 팔)로 푼다.

    arm1과 arm2는 채움 서버 하나를 같이 쓰므로 함께 가짜로 만들거나 둘 다 안 만든다.
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
    """시나리오 시간(초). 시뮬 시간이면 ROS 시계가 이미 0에서 시작한다."""

    def __init__(self, node):
        self._node = node
        self._sim = bool(node.get_parameter("use_sim_time").value)
        self._base = self._ros_now()

    def _ros_now(self) -> float:
        return self._node.get_clock().now().nanoseconds * 1e-9

    def now(self) -> float:
        return self._ros_now() - (0.0 if self._sim else self._base)

    def sleep_until(self, deadline: float, alive) -> bool:
        """시나리오 시간이 deadline이 될 때까지 막는다. 종료되면 False."""
        while self.now() < deadline:
            if not alive():
                return False
            time.sleep(0.01)
        return True
