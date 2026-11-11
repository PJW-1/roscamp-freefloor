"""Free-floor on the fifth layout of the real arena, a proposal (v5, 2026-11-11): the walls and
the four objects of v4, put so that no passage is a narrow one. Two small islands stand on the
middle line; north and south of them the floor is seven or eight clear cells wide (with eight
two robots pass side by side), and between them five. The only fixed lines are the last 0.23 m into each dock: they give the
direction a robot comes in by, and everywhere else a robot plans its own way."""
import collections
import itertools
import math
import sys
from pathlib import Path

import pytest
import yaml
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
    assert math.atan2(y1 - y0, x1 - x0) == pytest.approx(-math.pi / 4)  # 남동쪽으로
    (x0, y0), (x1, y1) = lines["outbound_in"].points
    assert math.atan2(y1 - y0, x1 - x0) == pytest.approx(-3 * math.pi / 4)  # 남서쪽으로


def test_a_dock_line_is_longer_than_the_arrival_tolerance():
    """선이 도착 허용 거리보다 짧으면 도크 옆을 지나던 로봇이 도착한 것으로 처리된다
    (Gazebo 2026-11-11: 0.14 m 선에서 남쪽에서 올라온 로봇이 반대 방향으로 도킹했다)."""
    from fleet_core.common.params import load_params
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
