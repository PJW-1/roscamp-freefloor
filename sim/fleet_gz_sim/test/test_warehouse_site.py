"""창고 사이트: 레인 지도와 상자 배치가 서로 맞는지, 장애물 일정이 그 지도에서 읽히는지."""
import itertools
import math
from pathlib import Path

import pytest
import yaml

lane_map_module = pytest.importorskip("fleet_core.traffic.lane_map")
from fleet_gz_sim import arena, obstacles  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
MAPS = PKG.parent / "fake_robots" / "maps"
SITE = yaml.safe_load((PKG / "sites" / "warehouse.yaml").read_text(encoding="utf-8"))
LANES = lane_map_module.load_lane_map(MAPS / f"{SITE['lane_map']}.yaml")
BOXES = [arena.Box(b["name"], b["x0"], b["y0"], b["x1"], b["y1"]) for b in SITE["boxes"]]
CLEAR_M = 0.25  # 레인에서 벽·상자까지
PARALLEL_M = 0.35  # 나란한 레인 사이
EPS = 1e-9


def test_site_names_an_existing_lane_map():
    assert (MAPS / f"{SITE['lane_map']}.yaml").is_file()


def test_waypoints_and_lanes_keep_clear_of_the_walls():
    x0, y0, x1, y1 = SITE["bounds_m"]
    for (ax, ay), (bx, by), name in arena.lane_segments(LANES):
        for x, y in ((ax, ay), (bx, by)):
            gap = min(x - x0, x1 - x, y - y0, y1 - y)
            assert gap >= CLEAR_M - EPS, f"{name} is {gap:.3f} m from a wall"


def test_every_lane_keeps_clear_of_every_box():
    for a, b, name in arena.lane_segments(LANES):
        for box in BOXES:
            gap = arena.clearance(a, b, box)
            assert gap >= CLEAR_M - EPS, f"{name} is {gap:.3f} m from {box.name}"


def test_boxes_are_valid_inside_and_apart():
    x0, y0, x1, y1 = SITE["bounds_m"]
    names = [b["name"] for b in SITE["boxes"]]
    assert len(names) == len(set(names))
    for raw, box in zip(SITE["boxes"], BOXES, strict=True):
        assert raw["kind"] in ("rack", "table", "pillar", "arm"), box.name
        assert raw["height_m"] > 0, box.name
        assert box.x0 < box.x1 and box.y0 < box.y1, box.name
        assert x0 - EPS <= box.x0 and box.x1 <= x1 + EPS, box.name
        assert y0 - EPS <= box.y0 and box.y1 <= y1 + EPS, box.name
    kinds = {raw["name"]: raw["kind"] for raw in SITE["boxes"]}
    for one, other in itertools.combinations(BOXES, 2):
        overlap = (min(one.x1, other.x1) - max(one.x0, other.x0) > EPS
                   and min(one.y1, other.y1) - max(one.y0, other.y0) > EPS)
        on_table = {kinds[one.name], kinds[other.name]} == {"arm", "table"}
        assert not overlap or on_table, f"{one.name} overlaps {other.name}"


def _inside(inner, outer) -> bool:
    return (outer.x0 - EPS <= inner.x0 and inner.x1 <= outer.x1 + EPS
            and outer.y0 - EPS <= inner.y0 and inner.y1 <= outer.y1 + EPS)


def test_each_arm_stands_on_its_table_by_its_dock():
    """arm1, arm2 at WORK and arm3 at OUTBOUND, each on top of its table (base taller than it)."""
    raw = {b["name"]: b for b in SITE["boxes"]}
    box = {b.name: b for b in BOXES}
    arms = sorted(n for n, b in raw.items() if b["kind"] == "arm")
    assert arms == ["arm1", "arm2", "arm3"]
    for arm in arms:
        table = f"table_{arm}"
        assert _inside(box[arm], box[table]), f"{arm} is not on {table}"
        assert raw[arm]["height_m"] > raw[table]["height_m"], arm
        station = "OUTBOUND" if arm == "arm3" else "WORK"
        assert arm in LANES.stations[station].arms
        x, y = LANES.waypoint_pose(LANES.stations[station].dock)
        a = box[arm]
        assert math.hypot(max(a.x0 - x, 0, x - a.x1), max(a.y0 - y, 0, y - a.y1)) < 0.5, arm
    for other in ("rack", "table"):  # 바닥이 창고처럼 보이게: 출고 쪽 포장대, 작업 쪽 대기 랙
        assert any(n.startswith(f"{'packing' if other == 'table' else 'staging'}_")
                   and b["kind"] == other for n, b in raw.items()), other


def test_tables_stand_by_both_docks():
    tables = [b for raw, b in zip(SITE["boxes"], BOXES, strict=True) if raw["kind"] == "table"]
    for station in ("WORK", "OUTBOUND"):
        x, y = LANES.waypoint_pose(LANES.stations[station].dock)
        near = min(math.hypot(max(t.x0 - x, 0, x - t.x1), max(t.y0 - y, 0, y - t.y1))
                   for t in tables)
        assert near < 0.4, f"no table next to the {station} dock"


def test_parallel_lanes_are_far_enough_apart():
    segments = arena.lane_segments(LANES)
    for (a1, a2, one), (b1, b2, other) in itertools.combinations(segments, 2):
        if {a1, a2} & {b1, b2}:
            continue  # 지점에서 만나는 레인
        ux, uy = a2[0] - a1[0], a2[1] - a1[1]
        vx, vy = b2[0] - b1[0], b2[1] - b1[1]
        sine = abs(ux * vy - uy * vx) / (math.hypot(ux, uy) * math.hypot(vx, vy))
        if sine > math.sin(math.radians(15)):
            continue
        gap = lane_map_module._segment_distance(a1, a2, b1, b2)
        assert gap >= PARALLEL_M - EPS, f"{one} and {other} are {gap:.3f} m apart"


def test_waypoints_are_spaced_out():
    for lane in LANES.lanes.values():
        assert lane.span >= 0.35 - EPS, f"{lane.a}->{lane.b} is {lane.span:.3f} m"


def test_stations_and_parking_match_the_contract():
    work, out = LANES.stations["WORK"], LANES.stations["OUTBOUND"]
    assert (work.server, work.arms, work.action) == ("workcell", ("arm1", "arm2"), "fill")
    assert (out.server, out.arms, out.action) == ("arm3", ("arm3",), "swap")
    assert set(LANES.stations) == {"WORK", "OUTBOUND"}
    assert 3 <= len(LANES.parking) <= 4
    # 작업대와 출고대는 양 끝에 둔다
    assert abs(LANES.waypoint_pose(work.dock)[0] - LANES.waypoint_pose(out.dock)[0]) > 4.0


def test_obstacle_schedule_parses_on_the_warehouse_map():
    schedule = obstacles.load_schedule(PKG / "obstacles" / "warehouse.yaml", LANES)
    assert 5 <= len(schedule.obstacles) <= 7
    assert schedule.repeat_every_s is not None
    for one in schedule.obstacles:
        assert 20 <= one.duration_s <= 40, one.label


def test_every_blocked_lane_leaves_a_way_around():
    """A robot at the tail of a blocked lane must have another exit, so the fleet can DETOUR
    instead of waiting for the box to go."""
    schedule = obstacles.load_schedule(PKG / "obstacles" / "warehouse.yaml", LANES)
    for one in schedule.obstacles:
        a, b = one.lane
        tails = [(a, b)] if (b, a) not in LANES.lanes else [(a, b), (b, a)]
        for tail, head in tails:
            exits = [lane for lane in LANES.lanes if lane[0] == tail and lane[1] != head]
            assert exits, f"{one.label}: {tail} has no other exit than {head}"


def _reachable(lanes, start, shut) -> set:
    """Waypoints a detour from start can reach: no shut lane, no passing through a parking
    spot (detour.py does not route through the waiting queue). A parking spot itself counts
    as reached, so the queue entrance is a destination like a dock."""
    parking = set(lanes.parking)
    seen, todo = {start}, [start]
    while todo:
        here = todo.pop()
        for a, b in lanes.lanes:
            if a == here and (a, b) not in shut and b not in seen:
                seen.add(b)
                if b not in parking:
                    todo.append(b)
    return seen


def cut_off(lane, lanes=LANES) -> list[tuple[str, str]]:
    """(tail, place) pairs a box on lane (and its reverse) cuts off, for every dock and parking
    spot: reachable from the tail before, not after."""
    a, b = lane
    shut = {(a, b), (b, a)}
    tails = [a] if (b, a) not in lanes.lanes else [a, b]
    places = [station.dock for station in lanes.stations.values()] + list(lanes.parking)
    found = []
    for tail in tails:
        before, after = _reachable(lanes, tail, set()), _reachable(lanes, tail, shut)
        found += [(tail, place) for place in places if place in before and place not in after]
    return found


def test_every_blocked_lane_keeps_the_docks_and_the_queue_reachable():
    """Another exit is not enough: through it a robot at the tail must still reach WORK,
    OUTBOUND and the waiting queue (Codex reviews of the portfolio map). This holds for a
    robot that has not taken the lane yet; one that already holds the next zone keeps its path
    and waits for the box (the fleet picks a detour only when it asks for the next zone)."""
    schedule = obstacles.load_schedule(PKG / "obstacles" / "warehouse.yaml", LANES)
    for one in schedule.obstacles:
        assert cut_off(one.lane) == [], one.label


def test_no_box_stands_on_a_queue_lane():
    """cut_off reaches the queue at its entrance only. The spots behind it are reached from
    there along the queue lanes, so no box may stand on a lane that touches a parking spot
    (Codex review of f4104e5)."""
    schedule = obstacles.load_schedule(PKG / "obstacles" / "warehouse.yaml", LANES)
    for one in schedule.obstacles:
        assert not set(one.lane) & set(LANES.parking), one.label


def test_every_box_blocks_only_its_own_lane():
    """The checks above treat a box as shutting its lane and nothing else. So the box keeps
    CLEAR_M from every other lane, a queue exit next to a junction included (Codex review of
    26502c6: C3->C4 at fraction 0.9 would stand in the P-3->C4 exit)."""
    schedule = obstacles.load_schedule(PKG / "obstacles" / "warehouse.yaml", LANES)
    # Any face of the box, whatever its yaw, is at most half its diagonal from the centre.
    reach = math.hypot(schedule.size_m[0], schedule.size_m[1]) / 2
    for one in schedule.obstacles:
        # By end points, not by name: waypoint names such as P-1 hold the "-" of the lane name.
        own = {tuple(LANES.waypoint_pose(w)[:2]) for w in one.lane}
        for a, b, name in arena.lane_segments(LANES):
            if {tuple(a), tuple(b)} == own:
                continue
            gap = arena._point_segment((one.x, one.y), a, b) - reach
            assert gap >= CLEAR_M - EPS, f"{one.label} is {gap:.3f} m from lane {name}"


def test_the_check_catches_an_exit_that_leads_nowhere():
    """A small map of its own: T has another exit than H, so the exit test passes, yet a box on
    T->H cuts T off from the dock and the queue."""
    from types import SimpleNamespace
    lanes = SimpleNamespace(
        lanes={("T", "H"): 1, ("T", "S"): 1, ("S", "T"): 1, ("H", "D"): 1, ("H", "Q"): 1,
               ("Q", "D"): 1, ("H", "P-1"): 1},
        parking=("P-1",), stations={"WORK": SimpleNamespace(dock="D")})
    assert any(lane[0] == "T" and lane[1] != "H" for lane in lanes.lanes)
    assert cut_off(("T", "H"), lanes) == [("T", "D"), ("T", "P-1")]
    assert cut_off(("H", "D"), lanes) == [], "H still reaches D through Q"
