"""traffic_mode로 교통 층을 고른다. 주문 코어와 FleetManager의 바깥 API는 그대로다."""
from __future__ import annotations

from fleet_core.traffic.controller import TrafficController

LANES = "lanes"
FREEFLOOR = "freefloor"
MODES = (LANES, FREEFLOOR)


def build_traffic(site, params, robots, clock, next_goal_id, floor=None):
    """lanes는 레인 제어기 그대로, freefloor는 자유바닥 제어기."""
    mode = params.traffic_mode
    if mode == LANES:
        return TrafficController(site, params, robots, clock, next_goal_id)
    if mode == FREEFLOOR:
        # 실험 모듈은 고를 때만 읽는다. lanes는 이 모듈 없이도 같게 돈다
        from fleet_core.traffic.floor.controller import FloorController
        return FloorController(site, params, robots, clock, next_goal_id, floor)
    raise ValueError(f"traffic_mode must be one of {', '.join(MODES)}, not {mode!r}")


def zone_space(site, params, floor=None):
    """Something that measures /fleet/zones names, for nodes outside the controller.

    lanes: the lane map itself. freefloor: zone names are cell names (c12_17), measured to the
    cell square of the floor map; any other name and every other attribute come from the lane map.
    """
    mode = params.traffic_mode
    if mode == LANES:
        return site
    if mode == FREEFLOOR:
        from fleet_core.traffic.floor.floor_map import load_floor_map
        return FloorZoneSpace(site, floor or load_floor_map())
    raise ValueError(f"traffic_mode must be one of {', '.join(MODES)}, not {mode!r}")


class FloorZoneSpace:
    """Lane map with cell names added for distance_to_zone. Read only."""

    def __init__(self, site, floor):
        self.site = site
        self.floor = floor

    def is_cell(self, zone: str) -> bool:
        return self.floor.cell_at(zone) is not None

    def distance_to_zone(self, x: float, y: float, zone: str) -> float:
        cell = self.floor.cell_at(zone)
        if cell is None:
            return self.site.distance_to_zone(x, y, zone)  # KeyError for an unknown name
        return self.floor.box_distance(cell, x, y)

    def cell_box(self, zone: str):
        """(x0, y0, x1, y1) of a cell, None for a name that is not a cell."""
        cell = self.floor.cell_at(zone)
        if cell is None:
            return None
        size = self.floor.cell_m
        x0 = self.floor.origin[0] + cell[0] * size
        y0 = self.floor.origin[1] + cell[1] * size
        return (x0, y0, x0 + size, y0 + size)

    def __getattr__(self, name):
        return getattr(self.site, name)
