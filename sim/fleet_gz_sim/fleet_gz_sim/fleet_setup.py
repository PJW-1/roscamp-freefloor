"""gz.launch.py가 시나리오에서 정하는 것. ROS 없이 돈다(fleet_core, fake_robots의 순수 파이썬만).

- 핑키를 놓는 자리: 시나리오의 시작 지점. 방향은 그 지점으로 들어오는 레인 방향
  (대기 줄을 따라 들어와 섰다).
- 돌릴 수 없는 시나리오: 로봇 고장이 있는 것(가짜 핑키가 일으키는 고장이라 진짜 pinky_fsm에는
  일으킬 곳이 없다). 경기장은 기본 레인 지도로 만들었으므로 지도도 기본 지도를 쓴다.
"""
from __future__ import annotations

import math
from pathlib import Path

from fake_robots.scenario import ROBOT_FAULTS, load_scenario
from fleet_core.traffic.lane_map import load_lane_map


class ScenarioUnfit(RuntimeError):
    """Gazebo 경기장에서 돌릴 수 없는 시나리오."""


def scenario_path(value: str, share: str | Path) -> Path:
    """파일 경로이거나 fake_robots의 scenarios 폴더에 있는 이름."""
    path = Path(value)
    if path.is_file():
        return path
    name = value if value.endswith(".yaml") else value + ".yaml"
    path = Path(share) / "scenarios" / name
    if not path.is_file():
        raise ScenarioUnfit(f"scenario {value!r} is nowhere: not a file and not in {path.parent}")
    return path


def load(path):
    """(기본 레인 지도, 시나리오). 경기장에서 돌릴 수 없으면 ScenarioUnfit."""
    name = Path(path).name
    site = load_lane_map(None)
    scenario = load_scenario(path, site)
    faults = sorted({f.type for f in scenario.faults if f.type in ROBOT_FAULTS})
    if faults:
        raise ScenarioUnfit(f"{name} has robot faults {faults}; only fake pinkys can make them")
    return site, scenario


def start_poses(site, scenario) -> dict[str, tuple[float, float, float]]:
    """로봇마다 (x, y, yaw). 들어오는 레인이 없으면 나가는 레인 방향, 둘 다 없으면 0."""
    poses = {}
    for robot, cfg in scenario.robots.items():
        x, y = site.waypoint_pose(cfg.start)
        lanes = ([lane for lane in site.lanes.values() if lane.b == cfg.start]
                 or [lane for lane in site.lanes.values() if lane.a == cfg.start])
        yaw = math.atan2(lanes[0].uy, lanes[0].ux) if lanes else 0.0
        poses[robot] = (x, y, yaw)
    return poses
