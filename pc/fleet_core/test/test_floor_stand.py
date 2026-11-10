"""자유바닥: 서 있는 로봇의 자리는 보고들의 평균으로 본다.

보고 자리는 잡음(시험 세계 σ 3 cm)으로 흔들린다. 서 있는 로봇이 출발할 때 흔들린 자리 둘레 칸을
새로 잡으려 하면 (1) 첫 출발에서 실제 몸이 잡지 않은 칸을 밟고(8자 경기장 20씨드 중 6개가 0.1 s에
멈춤), (2) 도크를 나가는 로봇이 뒤에 붙어 선 로봇의 칸에 걸려 서로 기다린다(같은 20씨드 중 1~4개).
"""
import math
import sys
from dataclasses import replace
from pathlib import Path

import pytest
import yaml
from fleet_core.common.commands import MoveCmd
from fleet_core.common.params import load_params
from fleet_core.manager import FleetManager
from fleet_core.traffic.floor.controller import (
    FIRST_MOVE_REPORTS,
    MOVING_REPORTS,
    STAND_JUMP_M,
)
from fleet_core.traffic.lane_map import load_lane_map

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))
import compare_floor  # noqa: E402

HERE = (1.0, 1.15)
NOISE = [(0.03, -0.02), (-0.04, 0.01), (0.02, 0.03), (-0.01, -0.04), (0.035, 0.02),
         (-0.03, 0.03), (0.0, -0.03), (0.01, 0.04), (-0.02, -0.01), (0.005, -0.01)]


def floor_core(robots=("a",)):
    now = [0.0]
    core = FleetManager(load_lane_map(), replace(load_params(), traffic_mode="freefloor"),
                        list(robots), lambda: now[0])
    core.on_order_board("RUN", [])
    core.traffic._plan_budget = 99
    return core, now


def noisy(core, now, robot, n, at=HERE, state="IDLE"):
    """잡음 섞인 보고 n번(0.1초마다)."""
    for k in range(n):
        dx, dy = NOISE[k % len(NOISE)]
        now[0] += 0.1
        core.on_robot_state(robot, at[0] + dx, at[1] + dy, 0.0, state)


def moves(core):
    return [c for c in core.tick() if isinstance(c, MoveCmd)]


def test_a_standing_robot_stands_at_the_mean_of_its_reports():
    core, now = floor_core()
    noisy(core, now, "a", 10)
    traffic, m = core.traffic, core.traffic._missions["a"]
    assert math.dist(traffic._here(m), HERE) < 0.005  # 잡음 열 개의 평균
    assert math.dist((m.x, m.y), HERE) > 0.005  # 마지막 보고는 그보다 멀다


def test_a_noisy_report_does_not_change_the_cells_a_standing_robot_needs():
    core, now = floor_core()
    noisy(core, now, "a", 20)
    traffic, m = core.traffic, core.traffic._missions["a"]
    before = traffic._body_now(m)
    now[0] += 0.1
    core.on_robot_state("a", HERE[0] + 0.06, HERE[1] + 0.05, 0.0, "IDLE")  # 8 cm 튄 보고
    assert traffic._body_now(m) == before
    assert set(traffic._body_cells(m.x, m.y)) != before  # 보고 자리로 잡았으면 달랐다


def test_a_standing_robot_holds_the_cells_round_where_it_stands():
    """서 있는 동안 평균 자리 둘레의 빈 칸을 채워 둔다: 뒤에 온 로봇이 그 칸을 먼저 잡지 못한다."""
    core, now = floor_core()
    noisy(core, now, "a", 20)
    core.tick()
    traffic, m = core.traffic, core.traffic._missions["a"]
    assert traffic._body_now(m) <= traffic.zones.held_set("a")


def test_a_standing_robot_asks_for_no_cell_another_robot_holds():
    core, now = floor_core(("a", "b"))
    traffic = core.traffic
    floor = traffic.floor
    core.on_robot_state("b", HERE[0] + 0.25, HERE[1], 0.0, "IDLE")
    core.on_robot_state("a", *HERE, 0.0, "IDLE")
    between = floor.cell_id(floor.cell_of(HERE[0] + 0.12, HERE[1]))
    traffic.zones.release("a", between)
    assert traffic.zones.acquire_all("b", {between})  # 둘 사이 칸을 b가 쥐었다
    noisy(core, now, "a", 20)
    core.tick()
    assert traffic.zones.holder(between) == "b"
    assert not traffic.zones.waited_cells("a")  # 줄을 서지 않는다(대기 순환을 만들지 않는다)


def test_the_first_move_waits_until_a_noisy_place_has_settled():
    core, now = floor_core()
    traffic = core.traffic
    noisy(core, now, "a", 3)
    traffic._start_leg(traffic._missions["a"], "TO_PARK", None, "B")
    assert not moves(core)  # 보고 셋으로는 자리를 믿지 못한다
    noisy(core, now, "a", FIRST_MOVE_REPORTS - 4)
    assert not moves(core)
    noisy(core, now, "a", 1)
    first = moves(core)
    assert first and math.dist(first[0].path[0][:2], HERE) < 0.01  # 평균 자리에서 떠난다


def test_reports_that_do_not_move_at_all_need_no_settling():
    core, now = floor_core()
    traffic = core.traffic
    for _ in range(2):
        core.on_robot_state("a", *HERE, 0.0, "IDLE")
    traffic._start_leg(traffic._missions["a"], "TO_PARK", None, "B")
    assert moves(core)  # 잡음이 없으면 기다릴 것이 없다


def test_a_report_far_from_the_mean_starts_the_mean_again():
    core, now = floor_core()
    noisy(core, now, "a", 10)
    traffic, m = core.traffic, core.traffic._missions["a"]
    there = (HERE[0] + STAND_JUMP_M + 0.05, HERE[1])
    core.on_robot_state("a", *there, 0.0, "IDLE")  # 누가 로봇을 옮겼다
    assert traffic._here(m) == there


def test_a_moving_robot_is_at_the_mean_of_its_last_few_reports():
    """달리는 로봇의 자리는 최근 보고 MOVING_REPORTS개의 평균이다: 잡음이 줄고, 조금 뒤처진다
    (뒤는 아직 쥔 칸이다)."""
    core, now = floor_core()
    noisy(core, now, "a", 10)
    traffic, m = core.traffic, core.traffic._missions["a"]
    core.on_robot_state("a", 1.2, 1.15, 0.0, "MOVING")
    assert traffic._here(m) == (1.2, 1.15)  # 달리기 시작: 서 있던 평균은 버린다
    xs = [1.2]
    for k in range(1, 8):
        xs.append(1.2 + 0.015 * k)
        core.on_robot_state("a", xs[-1], 1.15 + (0.03 if k % 2 else -0.03), 0.0, "MOVING")
    x, y = traffic._here(m)
    assert x == pytest.approx(sum(xs[-MOVING_REPORTS:]) / MOVING_REPORTS)
    assert abs(y - 1.15) < 0.011  # 좌우로 3 cm씩 튀는 보고가 1 cm 안으로 모인다
    core.on_robot_state("a", 1.6, 1.15, 0.0, "MOVING")  # 0.3 m 튐: 옮겨졌다
    assert traffic._here(m) == (1.6, 1.15)
    core.on_robot_state("a", 1.3, 1.15, 0.0, "IDLE")  # 섰다: 평균은 여기서부터
    assert traffic._here(m) == (1.3, 1.15)


def test_a_moving_robot_needs_no_cell_behind_it_because_one_report_fell_back():
    """보고 하나가 6 cm 뒤로 튀어도 달리는 로봇이 쥘 몸 칸은 뒤로 늘지 않는다."""
    core, now = floor_core()
    traffic, m = core.traffic, core.traffic._missions["a"]
    floor = traffic.floor
    for k in range(6):
        core.on_robot_state("a", 1.2 + 0.015 * k, 1.15, 0.0, "MOVING")
    before = traffic._body_now(m)
    core.on_robot_state("a", 1.2 + 0.015 * 6 - 0.06, 1.15, 0.0, "MOVING")
    west = min(floor.centre(floor.cell_at(c))[0] for c in before)
    assert min(floor.centre(floor.cell_at(c))[0] for c in traffic._body_now(m)) >= west


FIGURE8 = REPO / "sim" / "fleet_gz_sim" / "scenarios" / "gz_figure8_arena.yaml"
LOCKS = {"floor_lock_ahead_m": 0.15, "floor_extend_step_m": 0.05, "release_distance_m": 0.17}


def figure8_run(tmp_path, monkeypatch, seed, seconds):
    data = yaml.safe_load(FIGURE8.read_text(encoding="utf-8"))
    for spec in data["robots"].values():
        spec.update(speed_mps=0.15, pose_noise_m=0.03)
    data["timeout_s"] = seconds
    path = tmp_path / "figure8.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    monkeypatch.setattr(compare_floor, "OVERRIDES", dict(LOCKS))
    return compare_floor.run("figure8_arena", str(path), seed, "freefloor", timeout_s=seconds)


# 561d111에서 이 씨드들은 0.1 s에 멈췄다: 첫 MoveTo가 실제 몸으로 잡지 않은 칸을 밟는다.
@pytest.mark.parametrize("seed", [1, 8, 12, 13, 14, 18])
def test_the_first_move_from_a_noisy_place_stays_on_held_cells(tmp_path, monkeypatch, seed):
    row, world = figure8_run(tmp_path, monkeypatch, seed, 20.0)
    assert world is not None and row["violations"] == 0, row["error"]
    assert world.cell_violations == 0
    assert len({c.robot for _, c in world.commands if isinstance(c, MoveCmd)}) == 3


# 561d111에서 씨드 7은 WORK 도크를 나가는 로봇과 뒤에 붙어 선 로봇이 서로 기다려 900 s에 1건이었다.
def test_a_robot_leaving_a_dock_is_not_held_by_the_one_queued_behind_it(tmp_path, monkeypatch):
    row, world = figure8_run(tmp_path, monkeypatch, 7, 400.0)
    assert row["finished"], {k: row[k] for k in ("error", "end_s", "orders_shipped")}
    assert row["orders_shipped"] == 8 and row["unresolved"] == 0 and row["isolations"] == 0


# ---- 마주 오는 로봇이 있는 길은 고르지 않는다 -----------------------------------------------

FIGURE8_LANES = load_lane_map(REPO / "sim" / "fake_robots" / "maps" / "figure8_arena.yaml")
ARENA_PARAMS = REPO / "pc" / "fleet_core" / "fleet_core" / "data" / "fleet_params_real_arena.yaml"


def figure8_core(robots):
    now = [0.0]
    core = FleetManager(FIGURE8_LANES, replace(load_params(ARENA_PARAMS), traffic_mode="freefloor"),
                        list(robots), lambda: now[0])
    core.on_order_board("RUN", [])
    core.traffic._plan_budget = 99
    return core, now


def leaving_work_with_a_robot_coming_in(parked_at_p1: bool):
    """a는 WORK 도크에서 OUTBOUND로 떠나고, b는 들어가는 선으로 오고 있다. c는 P-1 칸에 서 있거나 없다."""
    core, now = figure8_core(("a", "b", "c"))
    traffic = core.traffic
    w, p1 = FIGURE8_LANES.waypoint_pose("W"), FIGURE8_LANES.waypoint_pose("P-1")
    for _ in range(3):
        core.on_robot_state("a", *w, 0.0, "IDLE")
        core.on_robot_state("b", 0.45, 0.88, 0.0, "IDLE")
        if parked_at_p1:
            core.on_robot_state("c", *p1, 0.0, "IDLE")
    traffic._start_leg(traffic._missions["b"], "TO_WORK", "WORK", "W")
    core.on_robot_state("b", 0.45, 0.88, 0.0, "MOVING")
    assert traffic._passing("b")
    traffic._start_leg(traffic._missions["a"], "TO_OUTBOUND", "OUTBOUND", "O")
    return traffic


def goes_back_up_through_the_dock(route) -> bool:
    """떠난 뒤 서쪽 통로를 다시 북쪽으로 올라간다(들어오는 선을 거스른다)."""
    w = FIGURE8_LANES.waypoint_pose("W")
    south = False
    for x, y in route.points:
        south = south or y < w[1] - 0.2
        if south and x < 0.3 and y > w[1] + 0.1:
            return True
    return False


def test_a_robot_parked_beside_the_way_does_not_send_a_leaving_robot_back_against_the_line_in():
    """8자 경기장 씨드 7(561d111): P-1에 선 로봇이 남쪽 통로를 벽으로 막은 것으로 보여, WORK를 나가는
    로봇이 도크를 거슬러 북쪽으로 되돌아가는 길을 받았고 들어오던 로봇과 맞서 끝내 못 풀었다."""
    free = leaving_work_with_a_robot_coming_in(parked_at_p1=False)._legs["a"].route
    assert not goes_back_up_through_the_dock(free)
    traffic = leaving_work_with_a_robot_coming_in(parked_at_p1=True)
    route = traffic._legs["a"].route
    assert route is not None and not goes_back_up_through_the_dock(route)
    assert not traffic._oncoming("a", route)


def test_a_stuck_robot_does_not_take_a_way_round_that_meets_an_oncoming_robot(monkeypatch):
    traffic = leaving_work_with_a_robot_coming_in(parked_at_p1=False)
    m, leg = traffic._missions["a"], traffic._legs["a"]
    old = leg.route
    monkeypatch.setattr(traffic, "_oncoming", lambda robot, route: {"b"})  # 돌아갈 길마다 b가 마주 온다
    monkeypatch.setattr(traffic, "_blockers", lambda robot: {"c"})
    leg.route = type(old)(old.points[2:])  # 고정 선을 벗어난 자리에서 막혔다
    leg.s = leg.s_lock = leg.s_sent = 0.0
    leg.stuck_since = leg.planned_at = -10.0
    before = traffic.metrics["replans"]
    traffic._replan_if_stuck(m, leg, False)
    assert traffic.metrics["replans"] == before and leg.no_way_since is not None


def test_a_cell_is_let_go_only_when_it_is_far_from_the_report_and_from_the_mean():
    """평균은 틀릴 수 있다(도킹이 끝나며 몸이 옮겨진 직후 등). 놓는 것은 보고 자리와 평균 자리 둘 다에서
    멀 때만이다: 예전(보고 자리 기준)보다 일찍 놓지 않는다. 실제 경기장 씨드 4에서 평균 기준으로만
    놓았더니 도크에 선 로봇의 몸 아래 칸을 놓았다."""
    from fleet_core.traffic.floor.controller import REST_RELEASE_SAMPLES
    core, now = floor_core()
    traffic, m = core.traffic, core.traffic._missions["a"]
    floor = traffic.floor
    spot = (1.0, 1.12)  # 남쪽 칸 줄(y 0.9~1.0)이 여기서 0.12 m: 몸 반지름 0.13 m 안
    for _ in range(10):
        core.on_robot_state("a", spot[0], spot[1] + 0.08, 0.0, "IDLE")  # 처음 보고들은 8 cm 북쪽
    south = {c for c in traffic._body_cells(*spot) if floor.centre(floor.cell_at(c))[1] < 1.0}
    assert south and traffic.zones.acquire_all("a", traffic.zones.held_set("a") | south)  # 오며 쥔 칸
    for _ in range(REST_RELEASE_SAMPLES + 2):  # 그 뒤 보고는 제자리: 평균은 아직 북쪽으로 치우쳤다
        now[0] += 0.1
        core.on_robot_state("a", *spot, 0.0, "IDLE")
        traffic._release_cells(m)
    mean = traffic._here(m)
    assert all(floor.box_distance(floor.cell_at(c), *mean) > traffic._body for c in south)
    assert south <= traffic.zones.held_set("a")  # 평균에서는 멀어도 보고 자리 둘레라 그대로 쥔다


def test_a_new_route_starts_where_the_robot_stands_not_at_its_last_noisy_report():
    """길의 첫 점 둘레로 출발 원을 잠근다. 첫 점이 튄 보고 자리면 그 원이 뒤에 선 로봇의 칸에 걸린다
    (실제 경기장 씨드 1: OUTBOUND를 나가는 로봇이 0.2 m 뒤 로봇과 서로 기다림)."""
    core, now = floor_core()
    noisy(core, now, "a", 20)
    traffic, m = core.traffic, core.traffic._missions["a"]
    now[0] += 0.1
    core.on_robot_state("a", HERE[0] - 0.06, HERE[1] + 0.05, 0.0, "IDLE")  # 8 cm 튄 보고
    traffic._start_leg(m, "TO_PARK", None, "B")
    start = traffic._legs["a"].route.points[0]
    assert math.dist(start, HERE) < 0.01


# ---- 출발 원의 가장자리 칸 --------------------------------------------------------------------
# 8자 경기장(3 cm 칸, 몸 원 0.11 m, 반폭 띠 0.09 m). a는 WORK 도크(0.15, 0.525)에서 남쪽으로 떠난다.

AT_WORK = (0.15, 0.522)  # WORK 도크(0.15, 0.525)에 3 mm 못 미쳐 섰다
NORTH_EDGE = (0.15, 0.64)  # 칸 줄 y 0.63~0.66: 선 자리에서 0.108 m, 원(0.11 m)의 테두리, 가는 길 반대쪽
NORTH_INSIDE = (0.15, 0.61)  # 칸 줄 y 0.60~0.63: 0.078 m, 원 안
SOUTH_EDGE = (0.15, 0.40)  # 칸 줄 y 0.39~0.42: 0.102 m, 가는 길 위


def leaving_work_beside(cell_xy):
    """a는 WORK 도크에 서서 OUTBOUND로 떠나려 하고, cell_xy가 든 칸은 뒤에 선 b가 쥐고 있다."""
    core, now = figure8_core(("a", "b"))
    traffic = core.traffic
    floor = traffic.floor
    for _ in range(3):
        core.on_robot_state("a", *AT_WORK, 0.0, "IDLE")
        core.on_robot_state("b", 0.15, 0.80, 0.0, "IDLE")
    cell = floor.cell_id(floor.cell_of(*cell_xy))
    traffic.zones.release("a", cell)
    assert traffic.zones.acquire_all("b", traffic.zones.held_set("b") | {cell})
    traffic._start_leg(traffic._missions["a"], "TO_OUTBOUND", "OUTBOUND", "O")
    assert cell in traffic._drive_cells_all(traffic._missions["a"], traffic._legs["a"])
    return traffic, cell


def test_a_standing_robot_starts_without_an_edge_cell_of_its_circle_that_another_robot_holds():
    """실제 경기장 씨드 1: OUTBOUND 도크의 로봇과 뒤 로봇의 원이 맞닿아, 평균이 1 mm 달라지자 가장자리
    칸 한 줄이 겹쳐 서로 기다렸다. 몸(반지름 0.08 m)은 그 칸에 닿지 않는다."""
    traffic, cell = leaving_work_beside(NORTH_EDGE)
    m, leg = traffic._missions["a"], traffic._legs["a"]
    assert traffic._extend(m, leg)
    assert traffic.zones.holder(cell) == "b" and not traffic.zones.waited_cells("a")


@pytest.mark.parametrize("cell_xy", [NORTH_INSIDE, SOUTH_EDGE])
def test_a_standing_robot_still_waits_for_a_cell_its_body_or_its_way_needs(cell_xy):
    traffic, cell = leaving_work_beside(cell_xy)
    m, leg = traffic._missions["a"], traffic._legs["a"]
    assert not traffic._extend(m, leg)
    assert cell in traffic.zones.waited_cells("a")


def test_a_moving_robot_needs_its_whole_circle():
    traffic, cell = leaving_work_beside(NORTH_EDGE)
    m = traffic._missions["a"]
    traffic._stand.pop("a")  # 서 있던 보고가 없다(달리는 중)
    m.robot_state = "MOVING"
    assert not traffic._extend(m, traffic._legs["a"])
    assert cell in traffic.zones.waited_cells("a")


# ---- 서서 기다리는 로봇이 쥐는 칸, 달리는 로봇에게 보내는 길 ---------------------------------


def test_a_robot_standing_with_nothing_locked_ahead_lets_go_of_the_cells_it_came_over():
    """기본 지도 blocked_lane 씨드 1: 대기 자리에 선 로봇이 0.16 m 뒤(놓는 거리 0.2 m 안)의 지나온 칸을
    끝내 쥐고 있어, 접근선으로 가는 로봇의 길을 막았다. 예전에는 보고 잡음이 우연히 풀어 줬다."""
    from fleet_core.traffic.floor.controller import REST_RELEASE_SAMPLES
    core, now = floor_core()
    traffic, m = core.traffic, core.traffic._missions["a"]
    floor = traffic.floor
    for _ in range(3):
        core.on_robot_state("a", *HERE, 0.0, "IDLE")
    traffic._start_leg(m, "TO_PARK", None, "B")
    leg = traffic._legs["a"]  # 다리는 있지만 앞을 잠그지 못하고 서 있다
    behind = floor.cell_id(floor.cell_of(HERE[0] - 0.2, HERE[1]))  # 몸 원(0.13 m) 밖, 0.2 m 안
    assert 0.13 < floor.box_distance(floor.cell_at(behind), *HERE) <= 0.2
    assert traffic.zones.acquire_all("a", traffic.zones.held_set("a") | {behind})
    for _ in range(REST_RELEASE_SAMPLES + 1):
        now[0] += 0.1
        core.on_robot_state("a", *HERE, 0.0, "IDLE")
        traffic._release_cells(m)
    assert leg.s_lock <= leg.s and behind not in traffic.zones.held_set("a")
    assert traffic._body_now(m) <= traffic.zones.held_set("a")


def test_a_path_sent_to_a_moving_robot_starts_beside_its_progress_not_behind_it():
    """달리는 로봇의 평균 자리는 뒤처진다. 보내는 길은 진행점(leg.s) 옆에서 시작해, 길이가 앞 잠금을
    넘지 않는다(옆으로 벗어난 만큼은 그대로 둔다)."""
    core, now = floor_core()
    traffic, m = core.traffic, core.traffic._missions["a"]
    for _ in range(3):
        core.on_robot_state("a", *HERE, 0.0, "IDLE")
    traffic._start_leg(m, "TO_PARK", None, "B")
    leg = traffic._legs["a"]
    route = leg.route
    for k in range(1, 9):  # 경로를 따라 1.5 cm씩, 옆으로 2 cm 벗어나 달린다
        x, y = route.at(0.015 * k)
        core.on_robot_state("a", x, y + 0.02, 0.0, "MOVING")
    leg.s = 0.015 * 8
    here = traffic._here(m)
    assert route.project(*here)[0] < leg.s - 0.02  # 평균은 진행점보다 뒤다
    end = leg.s + 0.3
    points = traffic._drive_path(m, leg, end)
    length = sum(math.dist(a, b) for a, b in zip(points, points[1:], strict=False))
    assert length <= end - leg.s + 0.01
    first = points[0]
    assert route.project(*first)[0] == pytest.approx(leg.s, abs=0.005)
    assert route.project(*first)[1] == pytest.approx(route.project(*here)[1], abs=0.005)  # 옆 거리는 그대로


# ---- Codex 리뷰(859564e) ------------------------------------------------------------------


def test_an_edge_cell_is_left_to_another_robot_only_when_the_place_is_well_known():
    """달린 적이 있어도 서 있는 보고가 적고 흔들리면 평균을 믿지 못한다: 원 전체를 요구한다."""
    core, now = figure8_core(("a", "b"))
    traffic = core.traffic
    floor = traffic.floor
    core.on_robot_state("b", 0.15, 0.80, 0.0, "IDLE")
    for dx, dy in ((0.004, -0.003), (-0.003, 0.002), (-0.001, 0.001)):  # 평균은 AT_WORK, 보고 셋
        core.on_robot_state("a", AT_WORK[0] + dx, AT_WORK[1] + dy, 0.0, "IDLE")
    traffic._started.add("a")
    cell = floor.cell_id(floor.cell_of(*NORTH_EDGE))
    traffic.zones.release("a", cell)
    assert traffic.zones.acquire_all("b", traffic.zones.held_set("b") | {cell})
    traffic._start_leg(traffic._missions["a"], "TO_OUTBOUND", "OUTBOUND", "O")
    assert not traffic._extend(traffic._missions["a"], traffic._legs["a"])
    assert cell in traffic.zones.waited_cells("a")


def test_a_report_on_cells_another_robot_holds_stops_the_robot_whatever_the_mean_says(monkeypatch):
    """평균은 늦거나 틀릴 수 있다. 보고 자리의 몸(반지름 robot_radius_m)이 남의 칸 위에 세 번 이어지면
    예전처럼 세운다."""
    core, now = floor_core(("a", "b"))
    traffic = core.traffic
    floor = traffic.floor
    core.on_robot_state("b", 1.05, 1.75, 0.0, "IDLE")
    for _ in range(3):
        core.on_robot_state("a", *HERE, 0.0, "IDLE")
    m = traffic._missions["a"]
    traffic._start_leg(m, "TO_PARK", None, "B")
    assert moves(core) and m.goal is not None
    there = (HERE[0], HERE[1] + 0.35)  # 평균 자리에서 먼 곳, b가 쥔 칸
    taken = {floor.cell_id(c) for c in [floor.cell_of(*there)]}
    assert traffic.zones.acquire_all("b", traffic.zones.held_set("b") | taken)
    monkeypatch.setattr(traffic, "_here", lambda mission: HERE)  # 평균은 제자리에 머문다
    for _ in range(traffic.params.release_confirm_samples):
        core.on_robot_state("a", *there, 0.0, "MOVING")
    assert m.goal is None  # 가던 MoveTo를 거뒀다


def test_the_mean_of_a_standing_robot_follows_a_slow_move():
    """서 있는 평균은 최근 STAND_REPORTS개만 본다: 조금씩 밀린 로봇을 따라간다."""
    from fleet_core.traffic.floor.controller import STAND_REPORTS
    core, now = floor_core()
    traffic, m = core.traffic, core.traffic._missions["a"]
    for _ in range(STAND_REPORTS + 10):
        core.on_robot_state("a", *HERE, 0.0, "IDLE")
    for _ in range(STAND_REPORTS):
        core.on_robot_state("a", HERE[0] + 0.08, HERE[1], 0.0, "IDLE")  # 8 cm 밀렸다(다시 모을 만큼은 아님)
    assert math.dist(traffic._here(m), (HERE[0] + 0.08, HERE[1])) < 0.005


def test_a_robot_placed_again_starts_with_no_memory_of_where_it_stood():
    core, now = floor_core()
    traffic, m = core.traffic, core.traffic._missions["a"]
    noisy(core, now, "a", FIRST_MOVE_REPORTS)
    traffic._started.add("a")
    m.placed = False  # 치웠다가 다시 올렸다
    core.on_robot_state("a", HERE[0] + 0.05, HERE[1], 0.0, "IDLE")
    assert m.placed and "a" not in traffic._started
    assert traffic._here(m) == (HERE[0] + 0.05, HERE[1]) and not traffic._settled(m)


def test_the_first_move_goes_after_a_while_even_if_the_place_never_settles():
    """보고가 계속 튀어 자리가 가라앉지 않아도 FIRST_MOVE_WAIT_S 뒤에는 떠난다(예전처럼 마지막 자리로)."""
    from fleet_core.traffic.floor.controller import FIRST_MOVE_WAIT_S
    core, now = floor_core()
    traffic = core.traffic
    spots = [HERE, (HERE[0] + STAND_JUMP_M + 0.02, HERE[1])]
    core.on_robot_state("a", *HERE, 0.0, "IDLE")
    traffic._start_leg(traffic._missions["a"], "TO_PARK", None, "B")
    sent = []
    for k in range(int(FIRST_MOVE_WAIT_S / 0.1) + 3):
        now[0] += 0.1
        core.on_robot_state("a", *spots[(k + 1) % 2], 0.0, "IDLE")  # 번갈아 튄다: 매번 다시 모은다
        sent += moves(core)
        if k < int(FIRST_MOVE_WAIT_S / 0.1) - 2:
            assert not sent
    assert sent


def test_the_edge_left_to_another_robot_grows_as_the_place_gets_better_known():
    """몸이 닿을 수 있는 반지름 = 돌 때 모서리(0.085 m) + 평균 오차(3 x 표준편차 / sqrt(보고 수)).
    그보다 먼 칸만 남에게 둔다. 보고가 쌓일수록 그 반지름이 준다(반폭 띠 0.09 m보다 작아지지는 않음)."""
    core, now = figure8_core(("a",))
    traffic, m = core.traffic, core.traffic._missions["a"]
    assert traffic._body_reach(m) >= traffic._body  # 보고가 없다: 원 전체
    k = 0
    reach = []
    for n in (FIRST_MOVE_REPORTS, 50):
        while len(traffic._stand.get("a", ())) < n:
            dx, dy = NOISE[k % len(NOISE)]
            k += 1
            core.on_robot_state("a", AT_WORK[0] + dx, AT_WORK[1] + dy, 0.0, "IDLE")
        reach.append(traffic._body_reach(m))
    corner = math.hypot(traffic.floor.robot_half_width_m, traffic.floor.robot_half_width_m)
    assert corner + 0.01 < reach[1] < reach[0] < traffic._body
    core2, _ = figure8_core(("a",))
    for _ in range(3):
        core2.on_robot_state("a", *AT_WORK, 0.0, "IDLE")  # 흔들림 없는 보고
    assert core2.traffic._body_reach(core2.traffic._missions["a"]) == core2.traffic._near


# ---- 리뷰(6e3a120)에서 고친 것 -------------------------------------------------------------

def unsettled_warnings(out):
    return [c for c in out if type(c).__name__ == "EventOut" and c.code == "FLOOR_PLACE_UNSETTLED"]


def later(core, now, dt, k):
    """시계를 dt만큼 보낸다(관제 시계는 보고가 와야 간다). 보고는 1 cm씩 달라 자리가 가라앉지 않는다."""
    now[0] += dt
    core.on_robot_state("a", HERE[0] + 0.01 * (k % 2), HERE[1], 0.0, "IDLE")


def test_the_first_move_wait_does_not_start_again_when_the_move_could_not_be_sent():
    """다 기다린 뒤 앞 칸을 못 잡아 보내지 못해도, 다음에 다시 5초를 기다리지 않고 경고도 한 번만 낸다."""
    from fleet_core.traffic.floor.controller import FIRST_MOVE_WAIT_S
    core, now = floor_core()
    traffic, m = core.traffic, core.traffic._missions["a"]
    later(core, now, 0.1, 0)
    assert not traffic._settled(m)
    later(core, now, FIRST_MOVE_WAIT_S + 0.1, 1)
    assert traffic._settled(m)  # 다 기다렸다(이번에는 보내지 못했다고 하자)
    later(core, now, 0.1, 2)
    assert not traffic._knows_place(m)
    assert traffic._settled(m)  # 처음부터 다시 기다리지 않는다
    later(core, now, FIRST_MOVE_WAIT_S + 0.1, 3)
    assert traffic._settled(m)
    assert len(unsettled_warnings(core.tick())) == 1


def test_a_robot_placed_again_waits_for_its_first_move_from_the_start():
    from fleet_core.traffic.floor.controller import FIRST_MOVE_WAIT_S
    core, now = floor_core()
    traffic, m = core.traffic, core.traffic._missions["a"]
    later(core, now, 0.1, 0)
    assert not traffic._settled(m)
    later(core, now, FIRST_MOVE_WAIT_S + 0.1, 1)
    assert traffic._settled(m)
    m.placed = False  # 치웠다가 다시 올렸다
    later(core, now, 0.1, 2)
    assert m.placed and not traffic._settled(m)


def test_the_body_reach_is_never_less_than_the_robot_radius_of_the_map():
    """폭보다 긴 로봇: 돌 때 모서리(반폭 x sqrt 2)가 지도의 몸 반지름보다 작으면 몸 반지름으로 본다."""
    core, _ = figure8_core(("a",))
    traffic, m = core.traffic, core.traffic._missions["a"]
    traffic.floor.robot_half_width_m = 0.03  # 폭 6 cm, 몸 반지름은 그대로 0.08
    traffic._near = 0.05
    for _ in range(3):
        core.on_robot_state("a", *AT_WORK, 0.0, "IDLE")  # 흔들림 없는 보고
    assert traffic._body_reach(m) >= traffic.floor.robot_radius_m
