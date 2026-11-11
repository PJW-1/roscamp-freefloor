"""arena_v4_twoway: 도킹 구간만 한 방향 선이고 나머지 통로는 양방향(지도 passages).

통로마다 가운데 선이 있다. line: true 통로(2·3 사이, 물체 3 북쪽)는 로봇 중심이 설 칸(clear)이 없거나
거의 없어 가운데 선을 탄다(양쪽으로). 나머지 통로(가운데, 물체 4 남쪽과 북쪽)는 A*가 칸 위로 지난다.
서쪽(출고)은 두 문(gap23, north3)으로만 나머지와 이어진다.
"""
import hashlib
import math
from pathlib import Path

import pytest
import yaml
from fleet_core.traffic.floor.cells import line_cells
from fleet_core.traffic.floor.floor_map import DEFAULT_FLOOR, FloorMap, FloorMapError, floor_map_for
from fleet_core.traffic.floor.planner import FloorPlanner
from fleet_core.traffic.lane_map import load_lane_map

REPO = Path(__file__).resolve().parents[3]
MAPS = REPO / "sim" / "fake_robots" / "maps"
LANES = load_lane_map(MAPS / "arena_v4_twoway.yaml")
FLOOR = floor_map_for(LANES)
V4 = floor_map_for(load_lane_map(MAPS / "arena_v4.yaml"))
DATA = yaml.safe_load((DEFAULT_FLOOR.parent / "floor_map_arena_v4_twoway.yaml").read_text(
    encoding="utf-8"))
WEST = (0.30, 0.65)      # 서쪽 넓은 곳(출고 위)
MIDDLE = (0.97, 0.30)    # 가운데 통로 아래쪽
EAST = (1.60, 0.40)      # 동쪽 넓은 곳


def floor_with(**change) -> FloorMap:
    data = {**DATA, **change}
    return FloorMap(data, DEFAULT_FLOOR.parent)


def on_line(points, p) -> float:
    best = math.inf
    for a, b in zip(points, points[1:], strict=False):
        dx, dy = b[0] - a[0], b[1] - a[1]
        u = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)))
        best = min(best, math.dist(p, (a[0] + u * dx, a[1] + u * dy)))
    return best


def through(route):
    """경로가 탄 line 통로 [(이름, 방향)]."""
    out = []
    for s0, s1, name in route.fixed:
        if name in FLOOR.passages:
            pts = FLOOR.passages[name].points
            a, b = route.at(s0), route.at(s1)
            out.append((name, 1 if math.dist(a, pts[0]) < math.dist(b, pts[0]) else -1))
    return out


def door_cells(name) -> frozenset:
    """그 문의 가운데 선이 지나는 칸(계획에 벽으로 주면 그 문이 막힌다)."""
    return frozenset(line_cells(FLOOR, list(FLOOR.passages[name].points)))


# ---- 지도 ----------------------------------------------------------------------------------


def test_the_twoway_floor_is_arena_v4_with_passages_and_only_the_docking_stretches_as_lines():
    assert FLOOR.source_text.startswith("#")
    assert FLOOR.occupied_pixels() == V4.occupied_pixels() and FLOOR.clear == V4.clear
    assert FLOOR.docks == V4.docks and FLOOR.parking == V4.parking
    assert sorted(FLOOR.fixed_lines) == ["outbound_in", "outbound_out", "work_in"]
    assert FLOOR.fixed_lines["work_in"] == V4.fixed_lines["work_in"]
    assert FLOOR.rolling_lines == {"outbound_in", "outbound_out"}
    assert not FLOOR.narrow_lines and not FLOOR.shortcuts
    # 들어가는 선은 도크 위에서 도크까지, 나오는 선은 도크에서 남서쪽 서쪽 넓은 곳까지
    assert FLOOR.fixed_lines["outbound_in"].end == FLOOR.docks["OUTBOUND"]
    assert FLOOR.fixed_lines["outbound_out"].start == FLOOR.docks["OUTBOUND"]
    for line in FLOOR.fixed_lines.values():
        assert line.length() < 0.5
    assert V4.passages == {}


def test_the_passages_are_the_five_corridors_and_the_two_doors_of_the_west_part():
    assert sorted(FLOOR.passages) == ["gap23", "mid_n", "mid_s", "north3", "north4", "south4"]
    assert {n for n, p in FLOOR.passages.items() if p.line} == {"gap23", "north3"}
    # line 통로는 가운데 선 밑에 clear가 아닌 칸이 있다(그래서 선을 탄다). 양 끝은 clear
    for name in ("gap23", "north3"):
        p = FLOOR.passages[name]
        assert FLOOR.passage_floor_of[name]
        assert FLOOR.cell_of(*p.points[0]) in FLOOR.clear
        assert FLOOR.cell_of(*p.points[-1]) in FLOOR.clear
    assert not FLOOR.passage_floor_of.get("mid_s")


def test_each_centre_line_keeps_the_same_distance_to_both_sides_within_two_centimetres():
    """가운데 선은 양쪽에서 같은 거리(물체와 벽으로 잰다): 선 위 점에서 옆으로 옮겨 봐도 벽까지
    거리가 2 cm 넘게 늘지 않는다."""
    for name, p in FLOOR.passages.items():
        for a, b in zip(p.points, p.points[1:], strict=False):
            n = (-(b[1] - a[1]) / math.dist(a, b), (b[0] - a[0]) / math.dist(a, b))
            mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
            here = FLOOR._gap_to_wall(*mid)
            for d in (-0.02, -0.01, 0.01, 0.02):
                there = FLOOR._gap_to_wall(mid[0] + n[0] * d, mid[1] + n[1] * d)
                assert there <= here + 0.02 + 1e-9, (name, mid, d)


@pytest.mark.parametrize("change, message", [
    ({"passages": [1]}, "passages"),
    ({"passages": {"x": {"points": [[0.97, 0.2]]}}}, "two points"),
    ({"passages": {"x": {"points": [[0.97, 0.2], [0.97, 0.3]], "line": "yes"}}}, "line"),
    ({"passages": {"x": {"points": [[0.97, 0.2], [0.97, 0.3]], "wide": 1}}}, "unknown"),
    ({"passages": {"work_in": {"points": [[0.97, 0.2], [0.97, 0.3]]}}}, "fixed line"),
    # 선 없는 통로가 clear 밖으로(물체 3 북쪽)
    ({"passages": {"x": {"points": [[0.60, 0.905], [0.90, 0.905]]}}}, "clear"),
    # line 통로가 벽에 너무 가깝다
    ({"passages": {"x": {"points": [[0.60, 0.885], [0.75, 0.93], [0.93, 0.885]],
                         "line": True}}}, "nearer"),
    # line 통로의 끝이 clear가 아니다
    ({"passages": {"x": {"points": [[0.75, 0.905], [0.85, 0.905], [0.93, 0.885]],
                         "line": True}}}, "end"),
])
def test_a_bad_passage_is_refused(change, message):
    with pytest.raises(FloorMapError, match=message):
        floor_with(**change)


def test_the_lane_map_names_the_twoway_floor_and_has_the_places_of_arena_v4():
    v4 = load_lane_map(MAPS / "arena_v4.yaml")
    assert LANES.floor_map == "floor_map_arena_v4_twoway.yaml"
    assert {w: LANES.waypoint_pose(w) for w in LANES._pose} == {
        w: v4.waypoint_pose(w) for w in v4._pose}
    assert LANES.stations.keys() == v4.stations.keys() and LANES.parking == v4.parking
    # 양방향 통로: 가운데 통로와 두 문을 거꾸로도 갈 수 있다
    for a, b in (("M1", "S2"), ("S2", "S1"), ("M2", "M1"), ("E1", "M2"), ("N1", "M2"),
                 ("M1", "X3"), ("X3", "X2")):
        assert (a, b) in LANES.lanes


# ---- 계획: 두 문, 양쪽 방향 ------------------------------------------------------------------


@pytest.mark.parametrize("start, goal", [(WEST, MIDDLE), (MIDDLE, WEST), (WEST, EAST),
                                         (EAST, WEST)])
def test_the_planner_goes_between_the_west_part_and_the_rest_through_a_door(start, goal):
    route = FloorPlanner(FLOOR).leg(start, goal)
    assert route is not None
    doors = through(route)
    assert len(doors) == 1 and doors[0][0] in ("gap23", "north3")
    # 문 구간은 선 위를 간다
    for s0, s1, name in route.fixed:
        pts = FLOOR.passages[name].points
        for k in range(21):
            assert on_line(pts, route.at(s0 + (s1 - s0) * k / 20)) <= 1e-6
    # 나머지는 clear 칸 위
    lines = [(s0, s1) for s0, s1, _ in route.fixed]
    for k in range(int(route.length / 0.01) + 1):
        s = k * 0.01
        if not any(a - 1e-9 <= s <= b + 1e-9 for a, b in lines):
            assert FLOOR.cell_of(*route.at(s)) in FLOOR.clear


@pytest.mark.parametrize("door, other", [("gap23", "north3"), ("north3", "gap23")])
@pytest.mark.parametrize("start, goal", [(WEST, MIDDLE), (MIDDLE, WEST)])
def test_with_one_door_walled_the_planner_takes_the_other_in_both_directions(door, other, start,
                                                                            goal):
    route = FloorPlanner(FLOOR).leg(start, goal, avoid=door_cells(door))
    assert route is not None
    assert [name for name, _ in through(route)] == [other]
    way = through(route)[0][1]
    pts = FLOOR.passages[other].points
    # 서쪽에서 나가면 선의 서쪽 끝에서 들어간다(두 문 모두 첫 점이 서쪽)
    assert way == (1 if start == WEST else -1), pts


def test_with_both_doors_walled_there_is_no_way_into_the_west_part():
    walls = door_cells("gap23") | door_cells("north3")
    assert FloorPlanner(FLOOR).leg(MIDDLE, WEST, avoid=walls) is None


@pytest.mark.parametrize("door", ["gap23", "north3"])
def test_a_start_inside_a_door_goes_out_along_its_line_either_way(door):
    pts = FLOOR.passages[door].points
    k = len(pts) // 2
    inside = ((pts[k - 1][0] + pts[k][0]) / 2, (pts[k - 1][1] + pts[k][1]) / 2)
    planner = FloorPlanner(FLOOR)
    for goal, way in ((WEST, -1), (EAST, 1)):
        route = planner.leg(inside, goal)
        assert route is not None and route.fixed[0][2] == door and route.fixed[0][0] == 0.0
        assert through(route)[0] == (door, way)
    # 방향을 정해 주면(쥔 방향 표시) 그쪽으로만 나간다
    route = planner.leg(inside, WEST, ways={door: 1})
    assert route is not None and through(route)[0] == (door, 1)


def test_arena_v4_plans_as_before_without_passages():
    """통로가 없는 지도에서는 길이 그대로다: 바닥 경로 여섯의 점과 고정 구간을 바꾸기 전(b2c4632)에
    구한 해시로 고정한다."""
    planner = FloorPlanner(V4)
    h = hashlib.sha256()
    for a, b, out, into in [((1.57, 0.87), (1.50, 0.45), None, None),
                            ((1.78, 0.17), (1.86, 0.58), None, None),
                            ((1.86, 0.58), (0.17, 0.52), None, "outbound_in"),
                            ((0.17, 0.52), (1.57, 0.87), "outbound_out", None),
                            ((1.78, 0.17), (0.17, 0.52), None, "outbound_in"),
                            ((1.30, 0.875), (1.78, 0.17), None, "work_in")]:
        route = planner.leg(a, b, out_line=out, in_line=into)
        h.update(repr((route.points, route.fixed)).encode())
    assert h.hexdigest() == "a5092504e64767fca4400a2dd0c458d64cc2ec518534a2c63080197b4222cd0c"
