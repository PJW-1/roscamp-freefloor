"""예전 테스트는 12 x 7 셀 배치로 쓰였으므로 그대로 쓴다."""
from pathlib import Path

from fleet_core.traffic.lane_map import load_lane_map

GRID_MAP = Path(__file__).parent / "data" / "grid_cells.yaml"


def grid_site(**kwargs):
    return load_lane_map(GRID_MAP, **kwargs)
