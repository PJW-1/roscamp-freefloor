"""The older tests were written for the 12 x 7 cell layout, so they stay on it."""
from pathlib import Path

from fleet_core.traffic.lane_map import load_lane_map

GRID_MAP = Path(__file__).parent / "data" / "grid_cells.yaml"


def grid_site(**kwargs):
    return load_lane_map(GRID_MAP, **kwargs)
