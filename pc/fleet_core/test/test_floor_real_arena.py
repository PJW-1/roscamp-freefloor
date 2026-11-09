"""Free-floor on the real arena (2.05 x 1.05 m, four objects): the floor map fits the Gazebo map
and the lane map, the whole floor is one piece, and parked or waiting robots leave the station
lines free."""
import itertools
import math
import sys
from pathlib import Path

import pytest
import yaml
from fleet_core.common.params import DEFAULT_PARAMS as DEFAULT_PARAMS_FILE
from fleet_core.common.commands import MoveCmd
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
    for (a, one), (b, other) in itertools.combinations(bodies.items(), 2):
        assert not one & other, (a, b)


def test_the_rehearsal_runs_three_pinkys_at_fifteen_centimetres_a_second():
    """실전은 늘 3대다: 대기 칸 셋, 시나리오 로봇 셋, 0.15 m/s."""
    assert [spot for spot, _ in FLOOR.parking] == ["P-3", "P-1", "P-2"]
    data = yaml.safe_load((REPO / "sim" / "fleet_gz_sim" / "scenarios" / "gz_real_arena.yaml")
                          .read_text(encoding="utf-8"))
    assert sorted(data["robots"]) == ["pinky1", "pinky2", "pinky3"]
    assert {spec["start"] for spec in data["robots"].values()} == {"P-1", "P-2", "P-3"}
    assert "nav_max_speed:=0.15" in (REPO / "sim" / "fleet_gz_sim" / "scenarios"
                                      / "gz_real_arena.yaml").read_text(encoding="utf-8")


def test_every_station_line_here_is_one_way_so_it_rolls_like_the_floor():
    """스테이션마다 들어가는 선과 나오는 선은 도크 둘레 칸만 나눈다: 나가는 로봇이 들어오는 선을
    거슬러 가지 않는다. 그래서 통째로 잠그지 않는다(자유 경로가 선을 거스르는 것은 막지 않는다:
    선 칸 비용이 4배라 피할 뿐이고, 만나면 칸 잠금과 교착 처리가 맡는다)."""
    assert FLOOR.rolling_lines == frozenset(FLOOR.fixed_lines)
    for station, dock in FLOOR.docks.items():
        way_in = cells_of(FLOOR.fixed_lines[f"{station.lower()}_in"])
        way_out = cells_of(FLOOR.fixed_lines[f"{station.lower()}_out"])
        assert way_in & way_out <= set(disk_cells(FLOOR, *dock, 2 * BODY)), station


NAV2 = REPO / "pinky" / "pinky_nav_config" / "params" / "nav2_params.yaml"


def test_the_half_width_is_half_the_nav2_footprint():
    """차동구동 핑키는 옆으로 가지 않는다: 곧게 달리는 동안 옆은 반폭 + 여유만 잠근다."""
    nav = yaml.safe_load(NAV2.read_text(encoding="utf-8"))
    footprint = yaml.safe_load(nav["local_costmap"]["local_costmap"]["ros__parameters"]
                               ["footprint"])
    assert FLOOR.robot_half_width_m == max(abs(y) for _, y in footprint) == 0.06
    # 제자리에서 돌 때 쓰는 원(반지름 + 여유)은 발자국 모서리를 덮는다
    assert BODY > math.hypot(*footprint[0])


def arena_traffic():
    from dataclasses import replace

    from fleet_core.common.params import load_params
    from fleet_core.manager import FleetManager
    core = FleetManager(LANES, replace(load_params(PARAMS), traffic_mode="freefloor"), ["a"],
                        lambda: 0.0)
    return core.traffic


def beside(traffic, x, y, near, far):
    """(x, y) 옆(북쪽) 칸 중 곧은 선 y까지 거리가 near 넘고 far 이하인 칸."""
    floor = traffic.floor
    return {floor.cell_id(c) for c in disk_cells(FLOOR, x, y, 0.2)
            if abs(FLOOR.centre(c)[0] - x) < 0.02
            and near < floor.box_distance(c, FLOOR.centre(c)[0], y) <= far}


def test_a_straight_stretch_locks_only_the_half_width_but_a_corner_the_whole_circle():
    from fleet_core.traffic.floor.planner import Route
    traffic = arena_traffic()
    side, body = traffic._side, traffic._body
    assert (round(side, 6), round(body, 6)) == (0.09, 0.11)
    route = Route([(1.20, 0.925), (1.875, 0.925), (1.875, 0.25)])  # outbound_in, corner at 0.675
    cells = traffic._lock_cells(route, 0.1, 0.6)
    reach = FLOOR.cell_m / 8
    straight = beside(traffic, 1.40, 0.925, side + reach, body)
    corner = beside(traffic, 1.80, 0.925, side + reach, body)  # 0.075 before the corner
    assert straight and corner
    assert not straight & cells  # 곧은 데서는 옆 칸을 안 잠근다
    assert corner <= cells  # 꺾이기 전 RPP가 제자리에서 돌 수 있는 곳은 원 전체


def test_the_robot_holds_only_its_half_width_beside_it_on_a_straight_line(monkeypatch):
    data = yaml.safe_load((REPO / "sim" / "fleet_gz_sim" / "scenarios" / "gz_real_arena.yaml")
                          .read_text(encoding="utf-8"))
    # 잡음 없이: 보고가 옆으로 튀면 그 자리 둘레를 잡는 것은 맞다(관제는 보고 자리만 안다)
    data["robots"] = {"pinky1": {"start": "P-2", "speed_mps": SPEED, "pose_noise_m": 0.0}}
    data["orders"] = data["orders"][:1]
    monkeypatch.setattr(compare_floor, "OVERRIDES", dict(LOCKS))
    world = compare_floor.build(data, "real_arena", 3, "freefloor")
    traffic = world.core.traffic
    seen = 0
    for _ in range(3000):
        world.step()
        r = world.virtual["pinky1"]
        if 1.30 < r.x < 1.50 and abs(r.y - 0.925) < 0.02:  # outbound_in 북쪽 곧은 구간
            seen += 1
            wide = beside(traffic, r.x, 0.925, traffic._side + FLOOR.cell_m / 8, 0.2)
            assert not wide & traffic.zones.held_set("pinky1"), (r.x, r.y)
        if world.done():
            break
    assert seen > 5 and world.cell_violations == 0


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
SPEED = 0.15  # gz.launch.py nav_max_speed:=0.15
LOCKS = {"floor_lock_ahead_m": 0.15, "floor_extend_step_m": 0.05, "release_distance_m": 0.17}


def test_the_real_arena_params_change_only_the_lock_sizes():
    """Small locks for a small floor; robots run slower (nav_max_speed 0.15 m/s) to match."""
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
def test_three_pinkys_ship_the_first_six_orders_on_the_real_arena(tmp_path, monkeypatch, seed):
    data = yaml.safe_load((REPO / "sim" / "fleet_gz_sim" / "scenarios" / "gz_real_arena.yaml")
                          .read_text(encoding="utf-8"))
    for spec in data["robots"].values():
        spec.update(speed_mps=SPEED, pose_noise_m=0.03)
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
    # 선 안에서도 잠금 앞 거리만큼만 보낸다(선 끝까지 한 번에 보내지 않는다)
    ahead = world.core.params.floor_lock_ahead_m + 0.1
    on_line = set().union(*(cells_of(line) for line in FLOOR.fixed_lines.values()))
    inside = 0
    for move in world.commands_of(MoveCmd):
        assert sum(math.dist(a[:2], b[:2]) for a, b in itertools.pairwise(move.path)) <= ahead
        inside += all(FLOOR.cell_of(*p[:2]) in on_line for p in move.path)
    assert inside > 20  # 선 위에서 보낸 짧은 MoveTo가 실제로 있었다
