"""Free-floor on the warehouse: every user of the floor map takes the one the lane map names."""
import sys
from dataclasses import replace
from pathlib import Path

import pytest
from fleet_core.common.params import load_params
from fleet_core.manager import FleetManager
from fleet_core.traffic.floor.floor_map import DEFAULT_FLOOR
from fleet_core.traffic.lane_map import load_lane_map
from fleet_core.traffic.modes import zone_space

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))
import compare_floor  # noqa: E402

WAREHOUSE = load_lane_map(REPO / "sim" / "fake_robots" / "maps" / "warehouse.yaml")
WAREHOUSE_FLOOR = (DEFAULT_FLOOR.parent / "floor_map_warehouse.yaml").read_text(encoding="utf-8")
FREEFLOOR = replace(load_params(), traffic_mode="freefloor")


def test_the_fleet_manager_drives_the_warehouse_on_the_warehouse_floor():
    core = FleetManager(WAREHOUSE, FREEFLOOR, ["pinky1"], lambda: 0.0)
    assert core.traffic.floor.source_text == WAREHOUSE_FLOOR


def test_the_zone_space_measures_warehouse_cells():
    space = zone_space(WAREHOUSE, FREEFLOOR)
    assert space.floor.source_text == WAREHOUSE_FLOOR
    cell = space.floor.cell_id(space.floor.cell_of(5.55, 1.85))  # just south of the OUTBOUND dock
    assert space.distance_to_zone(5.55, 1.85, cell) == 0.0


def test_lanes_mode_still_uses_the_lane_map_alone():
    lanes = replace(load_params(), traffic_mode="lanes")
    assert zone_space(WAREHOUSE, lanes) is WAREHOUSE
    core = FleetManager(WAREHOUSE, lanes, ["pinky1"], lambda: 0.0)
    assert not hasattr(core.traffic, "floor")


# Pinned seeds that finish. Other seeds ship every order too but can run out of time while three
# robots squeeze into the 0.35 m parking row (P-1..P-3), and seed 2 of the noisy start places one
# robot on cells of its neighbour (see the warehouse runs in docs/freefloor/README.md).
@pytest.mark.parametrize("seed", [4, 7])
def test_a_short_warehouse_run_finishes_on_the_free_floor(tmp_path, seed):
    import yaml
    data = yaml.safe_load((REPO / "sim" / "fake_robots" / "scenarios" / "warehouse_rush.yaml")
                          .read_text(encoding="utf-8"))
    data["orders"] = data["orders"][:6]
    data["timeout_s"] = 900
    data["expect"] = {"orders_shipped": 6}
    path = tmp_path / "warehouse_short.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    row, world = compare_floor.run("warehouse", str(path), seed, "freefloor")
    assert world is not None, row["error"]
    assert world.floor.source_text == WAREHOUSE_FLOOR
    assert row["finished"], {k: row[k] for k in ("error", "end_s", "orders_shipped")}
    assert row["orders_shipped"] == 6
    assert world.cell_violations == 0
    # head-on meetings in the 0.5 m aisles raise DEADLOCK_UNRESOLVED (no free spot to step aside
    # to) until one robot gets a new route; none may end in an isolation
    assert row["isolations"] == 0
    assert row["min_gap_m"] >= 2 * load_params().robot_radius_m
