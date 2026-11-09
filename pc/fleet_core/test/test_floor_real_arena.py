"""Free-floor on the real arena (2.05 x 1.05 m, four objects): the floor map fits the Gazebo map
and the lane map, the whole floor is one piece, and parked or waiting robots leave the station
lines free."""
import math
import sys
from pathlib import Path

import pytest
import yaml
from fleet_core.common.params import DEFAULT_PARAMS as DEFAULT_PARAMS_FILE
from fleet_core.traffic.floor.cells import disk_cells, swept_cells
from fleet_core.traffic.floor.floor_map import DEFAULT_FLOOR, floor_map_for
from fleet_core.traffic.lane_map import load_lane_map

REPO = Path(__file__).resolve().parents[3]
LANES = load_lane_map(REPO / "sim" / "fake_robots" / "maps" / "real_arena.yaml")
FLOOR = floor_map_for(LANES)
BODY = FLOOR.robot_radius_m + FLOOR.cell_margin_m


def parts(cells) -> list[set]:
    left, found = set(cells), []
    while left:
        stack = [left.pop()]
        part = set(stack)
        while stack:
            i, j = stack.pop()
            for n in ((i + a, j + b) for a in (-1, 0, 1) for b in (-1, 0, 1)):
                if n in left:
                    left.discard(n)
                    part.add(n)
                    stack.append(n)
        found.append(part)
    return found


def test_the_lane_map_names_this_floor_and_its_image_is_the_gazebo_map():
    assert FLOOR.source_text == (DEFAULT_FLOOR.parent / "floor_map_real_arena.yaml").read_text(
        encoding="utf-8")
    gazebo = REPO / "sim" / "fleet_gz_sim" / "maps"
    assert (DEFAULT_FLOOR.parent / "real_arena_fine.pgm").read_bytes() == (
        gazebo / "real_arena_fine.pgm").read_bytes()
    data = yaml.safe_load((gazebo / "real_arena_fine.yaml").read_text(encoding="utf-8"))
    assert list(FLOOR.origin) == data["origin"][:2]
    assert FLOOR.px_m == data["resolution"]


def test_the_margins_are_small_for_a_small_floor():
    assert (FLOOR.robot_radius_m, FLOOR.cell_margin_m, FLOOR.wall_margin_m) == (0.08, 0.03, 0.01)
    assert FLOOR.cell_m == 0.03  # 5 cm cells shut the 0.25 m gap between block_long and box_small


def test_the_whole_floor_is_one_piece_from_end_to_end():
    pieces = parts(FLOOR.clear)
    assert len(pieces) == 1
    xs = [FLOOR.centre(c)[0] for c in FLOOR.clear]
    assert min(xs) < 0.2 and max(xs) > 1.85


def cells_of(line) -> set:
    return set(swept_cells(FLOOR, list(line.points), BODY))


def test_outbound_is_a_one_way_loop_in_by_the_north_gap_out_by_the_south():
    """The east end has two ways in: the 0.25 m gap north of box_small and the 0.25 m gap
    between block_long and box_small (south of table_2, then up the middle). The way in takes
    the north one and the way out the other, so the two station lines share only the dock end.
    This checks the lines, not two robots driving them (the headless runs below do some)."""
    lines = FLOOR.fixed_lines
    north = {c for c in FLOOR.clear
             if 1.30 < FLOOR.centre(c)[0] < 1.55 and FLOOR.centre(c)[1] > 0.85}
    middle = {c for c in FLOOR.clear
              if 1.20 < FLOOR.centre(c)[0] < 1.30 and 0.53 < FLOOR.centre(c)[1] < 0.60}
    assert north and middle
    assert north <= cells_of(lines["outbound_in"]) and not north & cells_of(lines["outbound_out"])
    assert middle <= cells_of(lines["outbound_out"])
    assert not middle & cells_of(lines["outbound_in"])
    assert len(parts(FLOOR.clear - north - middle)) == 2
    assert len(parts(FLOOR.clear - north)) == 1 and len(parts(FLOOR.clear - middle)) == 1
    south = [p for p in lines["outbound_out"].points if p[1] < 0.30]
    assert south and max(x for x, _ in south) > 1.55  # under table_2


def test_a_parked_robot_leaves_the_station_lines_and_the_other_spot_free():
    line_cells = set().union(*(swept_cells(FLOOR, list(line.points), BODY)
                               for line in FLOOR.fixed_lines.values()))
    bodies = {spot: set(disk_cells(FLOOR, x, y, BODY)) for spot, (x, y) in FLOOR.parking}
    for spot, cells in bodies.items():
        assert not cells & line_cells, spot
    (_, one), (_, other) = bodies.items()
    assert not one & other


def test_docks_and_parking_are_the_lane_map_waypoints():
    for station, pose in FLOOR.docks.items():
        assert math.dist(pose, LANES.waypoint_pose(LANES.stations[station].dock)) < 1e-9
    assert [spot for spot, _ in FLOOR.parking] == list(LANES.parking)


def test_station_lines_start_and_end_where_the_lane_map_says():
    work, out = LANES.stations["WORK"], LANES.stations["OUTBOUND"]
    lines = FLOOR.fixed_lines
    assert lines["work_in"].end == LANES.waypoint_pose(work.dock)
    assert lines["work_out"].start == LANES.waypoint_pose(work.dock)
    assert lines["outbound_in"].end == LANES.waypoint_pose(out.dock)
    assert lines["outbound_out"].start == LANES.waypoint_pose(out.dock)
    assert lines["outbound_in"].start == LANES.waypoint_pose("QO")
    assert lines["outbound_out"].end == LANES.waypoint_pose("PS")


PARAMS = DEFAULT_PARAMS_FILE.parent / "fleet_params_real_arena.yaml"
LOCKS = {"floor_lock_ahead_m": 0.25, "floor_extend_step_m": 0.05, "release_distance_m": 0.17}


def test_the_real_arena_params_change_only_the_lock_sizes():
    """Small locks for a small floor; robots run slower (nav_max_speed 0.12 m/s) to match."""
    base = yaml.safe_load(DEFAULT_PARAMS_FILE.read_text(encoding="utf-8"))
    mine = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))
    assert set(mine) == set(base)
    assert {k: mine[k] for k in base if mine[k] != base[k]} == LOCKS


def test_the_body_is_eleven_centimetres_and_still_covers_the_robot():
    assert BODY == 0.11
    assert BODY > math.hypot(0.0575, 0.0575)  # 11.5 cm square Pinky turning in place


sys.path.insert(0, str(REPO / "tools"))
import compare_floor  # noqa: E402


@pytest.mark.parametrize("seed", [1, 2])
def test_two_pinkys_ship_the_first_six_orders_on_the_real_arena(tmp_path, monkeypatch, seed):
    data = yaml.safe_load((REPO / "sim" / "fleet_gz_sim" / "scenarios" / "gz_real_arena.yaml")
                          .read_text(encoding="utf-8"))
    for spec in data["robots"].values():
        spec.update(speed_mps=0.12, pose_noise_m=0.03)
    data["orders"] = data["orders"][:6]
    data["timeout_s"] = 600
    data["expect"] = {"orders_shipped": 6}
    path = tmp_path / "real_arena_short.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    monkeypatch.setattr(compare_floor, "OVERRIDES", dict(LOCKS))
    row, world = compare_floor.run("real_arena", str(path), seed, "freefloor")
    assert world is not None, row["error"]
    assert row["finished"], {k: row[k] for k in ("error", "end_s", "orders_shipped")}
    assert row["orders_shipped"] == 6
    assert world.cell_violations == 0
