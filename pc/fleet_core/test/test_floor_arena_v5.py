"""Free-floor on the fifth layout of the real arena, a proposal (v5, 2026-11-11): the walls and
the four objects of v4, put so that no passage is a narrow one. Two small islands stand on the
middle line; north and south of them the floor is seven or eight clear cells wide (with eight
two robots pass side by side), and between them five. The only fixed lines are the last 0.23 m into each dock: they give the
direction a robot comes in by, and everywhere else a robot plans its own way."""
import collections
import copy
import itertools
import math
import sys
from dataclasses import replace
from pathlib import Path

import pytest
import yaml
from fleet_core.common.params import load_params
from fleet_core.traffic.floor.cells import disk_cells, swept_cells
from fleet_core.traffic.floor.floor_map import DEFAULT_FLOOR, floor_map_for
from fleet_core.traffic.lane_map import load_lane_map

REPO = Path(__file__).resolve().parents[3]
LANES = load_lane_map(REPO / "sim" / "fake_robots" / "maps" / "arena_v5.yaml")
FLOOR = floor_map_for(LANES)
V4 = floor_map_for(load_lane_map(REPO / "sim" / "fake_robots" / "maps" / "arena_v4.yaml"))
BODY = FLOOR.robot_radius_m + FLOOR.cell_margin_m
ISLAND_1 = (0.52, 0.42, 0.69, 0.58)
SHELF_4 = (0.0, 0.85, 0.51, 0.99)
ISLAND_3 = (1.03, 0.43, 1.30, 0.56)
LOCKS = {"floor_lock_ahead_m": 0.15, "floor_extend_step_m": 0.05, "release_distance_m": 0.17}
THREE = (("pinky1", "P-3"), ("pinky2", "P-2"), ("pinky3", "P-1"))
KINDS = [{"A": 1, "C": 1}, {"B": 1}, {"D": 1, "A": 1}, {"C": 1}, {"B": 1, "D": 1}, {"A": 1}]

sys.path.insert(0, str(REPO / "tools"))
import compare_floor  # noqa: E402
from floor_world import FloorWorld  # noqa: E402  (compare_floor가 테스트 폴더를 경로에 넣는다)


def parts(cells) -> list[set]:
    """맞닿은 칸끼리 묶은 덩어리들."""
    left, found = set(cells), []
    while left:
        todo, part = [left.pop()], set()
        while todo:
            i, j = todo.pop()
            part.add((i, j))
            for near in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
                if near in left:
                    left.remove(near)
                    todo.append(near)
        found.append(part)
    return found


def clear_rows(x: float, y0: float, y1: float) -> int:
    """x에서 y0과 y1 사이에 있는 빈 칸(clear)의 수."""
    i = FLOOR.cell_of(x, 0.5)[0]
    return sum(1 for c in FLOOR.clear if c[0] == i and y0 < FLOOR.centre(c)[1] < y1)


def clear_columns(y: float, x0: float, x1: float) -> int:
    j = FLOOR.cell_of(0.9, y)[1]
    return sum(1 for c in FLOOR.clear if c[1] == j and x0 < FLOOR.centre(c)[0] < x1)


# ---- the map -----------------------------------------------------------------------------


def test_the_lane_map_names_this_floor_and_the_map_is_the_generated_one_centimetre_map():
    assert FLOOR.source_text == (DEFAULT_FLOOR.parent / "floor_map_arena_v5.yaml").read_text(
        encoding="utf-8")
    assert FLOOR.px_m == 0.01 and list(FLOOR.origin) == [-0.15, -0.15]
    image = (DEFAULT_FLOOR.parent / "arena_v5_fine.pgm").read_bytes()
    assert image.split(None, 4)[:4] == [b"P5", b"228", b"129", b"255"]  # 1.98 x 0.99 m + 테두리
    assert image == (REPO / "sim" / "fleet_gz_sim" / "maps" / "arena_v5_fine.pgm").read_bytes()


def test_the_robot_cells_and_margins_are_those_of_v4():
    names = ("robot_radius_m", "cell_margin_m", "cell_m", "robot_half_width_m", "side_margin_m",
             "wall_margin_m")
    assert [getattr(FLOOR, n) for n in names] == [getattr(V4, n) for n in names]


def test_the_clear_floor_is_one_piece_and_larger_than_on_v4():
    """v4는 동쪽과 서쪽이 좁은 선으로만 이어졌다. 여기는 빈 칸이 한 덩어리다."""
    assert len(parts(FLOOR.clear)) == 1
    assert len(FLOOR.clear) >= 1150 > len(V4.clear)


def test_two_robots_fit_side_by_side_past_the_islands():
    """빈 칸 여덟 줄: 줄 3과 줄 10의 가운데는 0.21 m 떨어져 있고, 곧게 달리는 로봇은 선 옆으로
    0.08 m(반폭 + 옆 여유)를 잠그니 두 로봇의 칸이 겹치지 않는다. 물체 3은 양쪽이 여덟 줄,
    물체 1(두께 0.16 m)은 남쪽이 여덟 줄이고 북쪽은 일곱 줄이다."""
    x0, y0, x1, y1 = ISLAND_3
    for x in (x0 + 0.03, (x0 + x1) / 2, x1 - 0.03):
        assert clear_rows(x, 0.0, y0) == 8, (x, "south")
        assert clear_rows(x, y1, 0.99) == 8, (x, "north")
    x0, y0, x1, y1 = ISLAND_1
    for x in (x0 + 0.03, (x0 + x1) / 2, x1 - 0.03):
        assert clear_rows(x, 0.0, y0) == 8, (x, "south")
    assert clear_rows((x0 + x1) / 2, y1, 0.99) == clear_rows(x1 - 0.03, y1, 0.99) == 7
    side = FLOOR.robot_half_width_m + FLOOR.side_margin_m
    assert 7 * FLOOR.cell_m - 2 * side >= FLOOR.cell_m  # 두 띠 사이에 칸 하나가 남는다


def test_the_throat_between_the_long_shelf_and_the_first_island_has_two_clear_rows():
    """물체 4 선반의 모서리와 물체 1 사이는 0.27 m다. 세 곳에서 빈 칸이 두 줄 이상이다
    (한 대가 지나는 폭). 다른 곳이 모두 더 넓다는 것은 여기서 보지 않는다."""
    assert ISLAND_1[0] - SHELF_4[2] == pytest.approx(0.01)
    assert SHELF_4[1] - ISLAND_1[3] == pytest.approx(0.27)
    throat = [clear_rows(x, ISLAND_1[3], SHELF_4[1]) for x in (0.495, 0.525, 0.555)]
    assert min(throat) >= 2, throat


def test_the_way_between_the_islands_is_five_clear_cells_wide():
    for y in (0.45, 0.50, 0.54):
        assert clear_columns(y, ISLAND_1[2], ISLAND_3[0]) == 5, y


def test_the_stations_stand_where_they_stood_on_v4():
    assert FLOOR.docks == V4.docks
    assert list(FLOOR.parking) == list(V4.parking)
    spots = [pose for _, pose in FLOOR.parking]
    assert all(FLOOR.cell_of(*p) in FLOOR.clear for p in [*FLOOR.docks.values(), *spots])


def test_the_only_lines_are_the_last_steps_into_the_docks():
    """도킹 앞에 들르는 지점이 없다: 선은 도크 앞 0.23 m뿐이고 들어가는 방향만 정한다."""
    lines = FLOOR.fixed_lines
    assert sorted(lines) == ["outbound_in", "work_in"]
    assert not FLOOR.rolling_lines and not FLOOR.narrow_lines
    for name, station in (("work_in", "WORK"), ("outbound_in", "OUTBOUND")):
        line = lines[name]
        assert line.end == tuple(FLOOR.docks[station])
        assert 0.22 <= line.length() <= 0.25
        assert FLOOR.cell_of(*line.start) in FLOOR.clear
    (x0, y0), (x1, y1) = lines["work_in"].points
    assert y0 == y1 and x1 > x0  # 남쪽 벽을 따라 동쪽으로
    (x0, y0), (x1, y1) = lines["outbound_in"].points
    assert math.atan2(y1 - y0, x1 - x0) == pytest.approx(-3 * math.pi / 4)  # 남서쪽으로


def test_a_dock_line_is_longer_than_the_arrival_tolerance():
    """선이 도착 허용 거리보다 짧으면 도크 옆을 지나던 로봇이 도착한 것으로 처리된다
    (Gazebo 2026-11-11: 0.14 m 선에서 남쪽에서 올라온 로봇이 반대 방향으로 도킹했다)."""
    for line in FLOOR.fixed_lines.values():
        assert line.length() >= load_params().arrival_tolerance_m + 0.07


def test_a_robot_on_its_parking_spot_leaves_the_dock_lines_and_the_other_spots_free():
    """자리 좌표에 정확히 선 로봇을 본다. 도착 허용 거리(0.15 m)만큼 벗어나 선 로봇은 보지 않는다."""
    line_cells = set()
    for line in FLOOR.fixed_lines.values():
        line_cells |= set(swept_cells(FLOOR, list(line.points), BODY))
    bodies = {spot: set(disk_cells(FLOOR, x, y, BODY)) for spot, (x, y) in FLOOR.parking}
    for spot, cells in bodies.items():
        assert not cells & line_cells, spot
    for (a, one), (b, other) in itertools.combinations(bodies.items(), 2):
        assert not one & other, (a, b)


def test_docks_and_parking_are_the_lane_map_waypoints():
    for station, pose in FLOOR.docks.items():
        assert math.dist(pose, LANES.waypoint_pose(LANES.stations[station].dock)) < 1e-9
    assert [spot for spot, _ in FLOOR.parking] == list(LANES.parking)
    for spot, pose in FLOOR.parking:
        assert math.dist(pose, LANES.waypoint_pose(spot)) < 1e-9


# ---- whole runs --------------------------------------------------------------------------


def scenario(count=8, period_s=20.0, noise_m=0.03, timeout_s=600):
    return {
        "map": "arena_v5", "seed": 1, "speed_factor": 5.0, "timeout_s": timeout_s,
        "stock": {"arm1": {"A": 40, "B": 40}, "arm2": {"C": 40, "D": 40}},
        "orders": [{"at_s": period_s * k, "id": f"R-{k + 1:03d}", "items": KINDS[k % 6]}
                   for k in range(count)],
        "robots": {name: {"start": spot, "speed_mps": 0.15, "pose_noise_m": noise_m}
                   for name, spot in THREE},
        "arm": {"seconds_per_item": 4.0, "swap_seconds": 8.0, "fail_rate": 0.0},
        "dock": {"duration_s": 3.0, "error_cm": 1.0}, "faults": [],
        "expect": {"orders_shipped": count},
    }


def run(tmp_path, monkeypatch, seed, count=8, period_s=20.0, timeout_s=600, noise_m=0.03):
    path = tmp_path / "arena_v5_run.yaml"
    path.write_text(yaml.safe_dump(scenario(count, period_s, noise_m, timeout_s)),
                    encoding="utf-8")
    monkeypatch.setattr(compare_floor, "OVERRIDES", dict(LOCKS))
    row, world = compare_floor.run("arena_v5", str(path), seed, "freefloor", frame_every=2)
    assert world is not None, row["error"]
    assert row["finished"], {k: row[k] for k in ("error", "end_s", "orders_shipped")}
    assert row["orders_shipped"] == count
    assert row["unresolved"] == 0 and row["isolations"] == 0
    assert world.cell_violations == 0
    assert row["min_gap_m"] >= 0.22  # 두 몸과 칸 여유
    return row, world


@pytest.mark.parametrize("seed", [1, 2, 3])
def test_three_pinkys_ship_eight_orders_and_park_again(tmp_path, monkeypatch, seed):
    run(tmp_path, monkeypatch, seed)


def test_three_pinkys_ship_eight_orders_when_every_report_is_exact(tmp_path, monkeypatch):
    run(tmp_path, monkeypatch, 1, noise_m=0.0)


def test_three_pinkys_keep_up_when_orders_come_faster_than_they_ship(tmp_path, monkeypatch):
    run(tmp_path, monkeypatch, 1, count=16, period_s=15, timeout_s=900)


def test_robots_are_seen_in_both_passages_and_between_the_islands(tmp_path, monkeypatch):
    """한 줄로 도는 길이 아니다: 한 번의 실행에서 로봇 위치가 북쪽 통로, 남쪽 통로, 건너는 통로
    세 곳에서 모두 나온다(끝까지 지났는지, 어느 방향인지는 보지 않는다)."""
    _, world = run(tmp_path, monkeypatch, 1)
    seen = collections.Counter()
    for frame in world.frames:
        for x, y in frame["robots"].values():
            if ISLAND_1[0] < x < ISLAND_1[2] or ISLAND_3[0] < x < ISLAND_3[2]:
                seen["north" if y > 0.5 else "south"] += 1
            elif ISLAND_1[2] < x < ISLAND_3[0] and ISLAND_3[1] < y < ISLAND_3[3]:
                seen["between"] += 1
    assert all(seen[where] > 0 for where in ("north", "south", "between")), seen


def off_line(points, p) -> float:
    """점 p에서 선분 points까지 거리."""
    (ax, ay), (bx, by) = points
    dx, dy = bx - ax, by - ay
    u = min(1.0, max(0.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / (dx * dx + dy * dy)))
    return math.dist(p, (ax + u * dx, ay + u * dy))


@pytest.mark.parametrize("seed", [1, 2])
def test_every_robot_comes_into_a_dock_down_its_line(tmp_path, monkeypatch, seed):
    """들어가는 방향: 도크에 닿기 전 0.15 m 안에 들어선 자리가 그 도크의 선 위다(0.05 m 안).
    관제는 이를 강제하지 않는다(도크 0.15 m 안에서 다시 계획하면 선 없이 도착한다): 이 실행들에서
    그런 일이 없었다는 것만 본다."""
    _, world = run(tmp_path, monkeypatch, seed)
    lines = {"WORK": FLOOR.fixed_lines["work_in"].points,
             "OUTBOUND": FLOOR.fixed_lines["outbound_in"].points}
    arrivals = 0
    tracks = collections.defaultdict(list)
    for frame in world.frames:
        for name, xy in frame["robots"].items():
            tracks[name].append(tuple(xy))
    for name, track in tracks.items():
        for station, dock in FLOOR.docks.items():
            entered = None  # 0.15 m 안에 처음 들어선 자리
            for before, here in zip(track, track[1:], strict=False):
                far, near = math.dist(before, dock), math.dist(here, dock)
                if far > 0.15 >= near:
                    entered = here
                if entered is not None and near <= 0.03:
                    arrivals += 1
                    assert off_line(lines[station], entered) <= 0.05, (name, station, entered)
                    entered = None
    assert arrivals >= 16  # 주문 8건: 작업 도크 8번, 출고 도크 8번


# ---- robots waiting for the outbound dock and the robot leaving it -------------------------
# Gazebo 2026-11-11: pinky2 had left OUTBOUND for WORK and stood at about (0.33, 0.37), its body
# still on cells of the dock line. pinky1 and pinky3 waited for their turn on the line at the
# west mouths of the south and of the north passage, the only two ways out of the west part:
# pinky2 could not get past them, the line never became free, and no wait cycle was seen (a
# robot waiting for a line turn waits for no cell). The same three robots here on the headless
# fake, with the dock line of today's map: pinky2 works at the dock while the other two come
# from the east and take their wait spots, then it leaves for WORK.

DOCKED = ("DOCKING", "AT_STATION")
WAITERS = ("pinky1", "pinky3")


def unnamed_floor():
    """이 바닥에서 지도에 적은 대기 자리만 뺀 것: 관제가 대기 자리를 스스로 찾는다."""
    floor = copy.copy(FLOOR)
    floor.wait_spots = {}
    return floor


def dock_wait_world(closes=None, hold_s=25.0, order=WAITERS, floor=None):
    """pinky2 at the outbound dock, pinky1 and pinky3 sent there from the east (in this order).
    After hold_s (both stand at their wait spots) pinky2 is sent to WORK. closes replaces the
    wait spot rule (_closes_way); None keeps it. floor: None is the floor without the wait
    spots of the map (the rule and the wait cycles are what these tests are about)."""
    robots = {n: {"start": s, "speed_mps": 0.15, "pose_noise_m": 0.0} for n, s in THREE}
    world = FloorWorld(seed=1, orders=[], robots=robots, site=LANES,
                       floor=unnamed_floor() if floor is None else floor,
                       params=replace(load_params(), **LOCKS))
    for name, (x, y) in {"pinky1": (1.60, 0.21), "pinky2": FLOOR.docks["OUTBOUND"],
                         "pinky3": (1.57, 0.87)}.items():
        world.virtual[name].x, world.virtual[name].y = x, y
    traffic = world.core.traffic
    if closes is not None:
        traffic._closes_way = closes
    world._report_states()
    docked = traffic._missions["pinky2"]
    docked.phase, docked.dest, docked.hold, docked.path, docked.idx = (
        "AT_STATION", "OUTBOUND", True, ["O"], 0)
    for goal_id, name in enumerate(order, 901):
        traffic.on_dispatch(goal_id, "OUTBOUND", name)
        world.step()  # 하나씩: 먼저 보낸 로봇이 먼저 줄에 선다
    world.run(hold_s)
    legs = {n: traffic._legs[n] for n in WAITERS}
    assert all(leg.waiting and leg.done for leg in legs.values()), legs
    traffic.on_dispatch(903, "WORK", "pinky2")
    return world, traffic


def out_and_in(world, traffic, seconds=60.0) -> bool:
    """pinky2 leaves the west part and one of the two waiters docks at OUTBOUND in time."""
    def done():
        return world.virtual["pinky2"].x > 0.9 and any(
            traffic._missions[n].phase in DOCKED for n in WAITERS)
    return world.run_until(done, timeout_s=seconds)


def ways_out(spots) -> bool:
    """A robot body (radius BODY) can go from the outbound dock to every parking spot without
    touching the standing cells of robots at spots (radius max(BODY, release_distance_m))."""
    radius = max(BODY, LOCKS["release_distance_m"])
    taken = set().union(*(disk_cells(FLOOR, x, y, radius) for x, y in spots))
    free = {c for c in FLOOR.clear if not disk_cells(FLOOR, *FLOOR.centre(c), BODY) & taken}
    start = FLOOR.cell_of(*FLOOR.docks["OUTBOUND"])
    part = next((p for p in parts8(free) if start in p), set())
    return all(FLOOR.cell_of(*pose) in part for _, pose in FLOOR.parking)


def parts8(cells) -> list[set]:
    """맞닿은 칸끼리(대각선 포함) 묶은 덩어리들."""
    left, found = set(cells), []
    while left:
        todo, part = [left.pop()], set()
        while todo:
            i, j = todo.pop()
            part.add((i, j))
            for near in ((i + a, j + b) for a in (-1, 0, 1) for b in (-1, 0, 1)):
                if near in left:
                    left.remove(near)
                    todo.append(near)
        found.append(part)
    return found


def wait_spots(traffic) -> list:
    return [traffic._legs[n].route.points[-1] for n in WAITERS]


def test_the_robot_leaving_the_outbound_dock_gets_past_the_robots_waiting_for_it():
    world, traffic = dock_wait_world()
    assert out_and_in(world, traffic)
    assert world.cell_violations == 0
    assert "DEADLOCK_UNRESOLVED" not in [e.code for e in world.events]


def test_robots_waiting_for_the_outbound_dock_leave_a_way_out_of_the_west_part():
    world, traffic = dock_wait_world()
    assert ways_out(wait_spots(traffic)), wait_spots(traffic)
    assert out_and_in(world, traffic)
    codes = [e.code for e in world.events]
    assert "DEADLOCK_DETECTED" not in codes and "FLOOR_SIDESTEP" not in codes, codes


def test_a_robot_waits_where_it_waited_before_when_every_spot_closes_the_way():
    """No spot keeps a way out: the old choice (the nearest clear spot it can drive to), not
    nowhere to wait."""
    _, before = dock_wait_world(closes=lambda *args: False)
    _, now = dock_wait_world(closes=lambda *args: True)
    assert wait_spots(now) == wait_spots(before)


@pytest.mark.parametrize("order", [WAITERS, WAITERS[::-1]])
def test_a_robot_waiting_for_a_line_turn_waits_on_the_robot_that_holds_the_line(order):
    """With the wait spot rule off both waiters stand in the two mouths as in Gazebo: the robot
    leaving waits for cells of a waiter and that waiter for the line the leaving robot still
    holds. The cycle is seen, a waiter steps aside (it crosses the way of the robot leaving to
    get there), and both go on; the waiter that asked first docks first."""
    world, traffic = dock_wait_world(closes=lambda *args: False, order=order)
    assert not ways_out(wait_spots(traffic)), wait_spots(traffic)  # the two mouths
    asked = {n: traffic._legs[n].asked_at for n in WAITERS}
    first = min(WAITERS, key=lambda n: (asked[n], n))
    assert out_and_in(world, traffic)
    codes = [e.code for e in world.events]
    assert "DEADLOCK_DETECTED" in codes and "DEADLOCK_UNRESOLVED" not in codes, codes
    aside = {e.robot for e in world.events if e.code == "FLOOR_SIDESTEP"}
    assert aside and aside <= set(WAITERS), aside
    assert world.cell_violations == 0
    # a waiter that stepped aside keeps its place in the line queue: the first one docks first
    assert traffic._missions[first].phase in DOCKED
    later = next(n for n in WAITERS if n != first)
    assert later in aside  # 비켜 선 것은 나중에 줄에 선 로봇이다
    assert world.run_until(lambda: getattr(traffic._legs.get(later), "in_line", None)
                           == "outbound_in", timeout_s=20.0)  # 비켜 섰다 줄로 돌아왔다
    assert traffic._legs[later].asked_at == asked[later]


# ---- wait spots written on the map ---------------------------------------------------------
# Gazebo 2026-11-11 (after the rule above): pinky1 left WORK for OUTBOUND, pinky2 waited for its
# turn at WORK in the east mouth of the south passage and pinky3 stood on P-3; pinky1 found no
# way between them. A robot that waits near the line start can stand in a passage on this
# floor, so the map says where to wait: between the islands and on the parking spots.

CROSS = (0.86, 0.495)  # 두 섬 사이


def spot_of(traffic, name):
    return traffic._legs[name].route.points[-1]


def test_the_map_names_the_wait_spots_between_the_islands_and_on_the_parking_spots():
    spots = [pose for _, pose in FLOOR.parking]
    assert set(FLOOR.wait_spots) == {"work_in", "outbound_in"}
    assert set(FLOOR.wait_spots["work_in"]) == set(spots)
    assert FLOOR.wait_spots["outbound_in"][0] == CROSS
    assert set(FLOOR.wait_spots["outbound_in"][1:]) == set(spots)


def test_a_robot_on_a_wait_spot_holds_no_cell_of_a_dock_line_and_its_body_no_other_spot():
    """선 칸은 서 있는 로봇이 쥘 수 있는 칸(몸과 release_distance_m 안)으로, 자리끼리는 몸으로 본다
    (옆 주차 자리와는 0.29 m라 release_distance_m 원은 겹친다)."""
    radius = max(BODY, LOCKS["release_distance_m"])
    line_cells = set()
    for line in FLOOR.fixed_lines.values():
        line_cells |= set(swept_cells(FLOOR, list(line.points), BODY))
    places = sorted({spot for spots in FLOOR.wait_spots.values() for spot in spots})
    for spot in places:
        assert not set(disk_cells(FLOOR, *spot, radius)) & line_cells, spot
    bodies = {spot: set(disk_cells(FLOOR, *spot, BODY)) for spot in places}
    for (a, one), (b, other) in itertools.combinations(bodies.items(), 2):
        assert not one & other, (a, b)


def test_a_robot_between_the_islands_leaves_three_clear_rows_beside_it_north_and_south():
    radius = max(BODY, LOCKS["release_distance_m"])
    taken = set(disk_cells(FLOOR, *CROSS, radius))
    free = {c for c in FLOOR.clear if not disk_cells(FLOOR, *FLOOR.centre(c), BODY) & taken}
    for ys in ((0.105, 0.135, 0.165), (0.825, 0.855, 0.885)):  # 남쪽, 북쪽 통로
        for y in ys:  # 빈 칸 세 줄씩: 한 대가 지난다
            row = {FLOOR.cell_of(x / 100, y) for x in range(72, 100, 3)}
            assert row <= free, y


def test_robots_waiting_for_the_outbound_dock_stand_on_the_wait_spots_of_the_map():
    world, traffic = dock_wait_world(floor=FLOOR)
    named = FLOOR.wait_spots["outbound_in"]
    spots = [spot_of(traffic, n) for n in WAITERS]
    assert spots[0] == CROSS, spots
    assert min(math.dist(spots[1], place) for place in named[1:]) <= 0.15, spots  # 서 있던 주차 자리
    assert out_and_in(world, traffic)
    codes = [e.code for e in world.events]
    assert "DEADLOCK_DETECTED" not in codes and "FLOOR_SIDESTEP" not in codes, codes
    assert world.cell_violations == 0


def work_wait_world():
    """pinky1 at the work dock, pinky3 on P-3, pinky2 on P-2 and sent to WORK (it waits). Then
    pinky1 is sent on to OUTBOUND."""
    robots = {n: {"start": s, "speed_mps": 0.15, "pose_noise_m": 0.0} for n, s in THREE}
    world = FloorWorld(seed=1, orders=[], robots=robots, site=LANES, floor=FLOOR,
                       params=replace(load_params(), **LOCKS))
    spots = dict(FLOOR.parking)
    for name, (x, y) in {"pinky1": FLOOR.docks["WORK"], "pinky2": spots["P-2"],
                         "pinky3": spots["P-3"]}.items():
        world.virtual[name].x, world.virtual[name].y = x, y
    traffic = world.core.traffic
    world._report_states()
    docked = traffic._missions["pinky1"]
    docked.phase, docked.dest, docked.hold, docked.path, docked.idx = (
        "AT_STATION", "WORK", True, ["W"], 0)
    traffic.on_dispatch(901, "WORK", "pinky2")
    world.run(10.0)
    return world, traffic


def test_a_robot_called_to_a_busy_work_dock_waits_on_its_parking_spot():
    world, traffic = work_wait_world()
    leg = traffic._legs["pinky2"]
    assert leg.waiting and leg.done
    assert spot_of(traffic, "pinky2") == pytest.approx(dict(FLOOR.parking)["P-2"], abs=0.01)
    assert math.dist((world.virtual["pinky2"].x, world.virtual["pinky2"].y),
                     dict(FLOOR.parking)["P-2"]) < 0.02  # 그 자리에 그대로 있다


def test_the_robot_leaving_the_work_dock_gets_past_the_waiting_and_the_parked_robot():
    world, traffic = work_wait_world()
    traffic.on_dispatch(902, "OUTBOUND", "pinky1")

    def done():
        return (world.virtual["pinky1"].x < 1.0
                and traffic._missions["pinky2"].phase in DOCKED)
    assert world.run_until(done, timeout_s=60.0)
    codes = [e.code for e in world.events]
    assert "DEADLOCK_UNRESOLVED" not in codes and world.cell_violations == 0
