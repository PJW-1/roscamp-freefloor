"""자유바닥: 지나가는 길은 스테이션 도크에 선 로봇의 몸 둘레 칸에서 몸 하나만큼 떨어진다.

실측 경기장(arena_1110)에서 틈에 도킹한 로봇 옆(남쪽 출구)을 지나던 로봇의 길이 도킹한 로봇의 몸
둘레 칸에 걸쳐, 출구 옆에 선 채 돌지도 지나가지도 못했다. 틈에서 나오는 로봇은 그 로봇의 칸을
기다리니 둘이 서로 기다리고, 격리되면 작업대가 막혔다(무작위 주문·충전 20씨드 중 4개).
"""
from dataclasses import replace
from pathlib import Path

from fleet_core.common.params import load_params
from fleet_core.manager import FleetManager
from fleet_core.traffic.floor.cells import disk_cells
from fleet_core.traffic.lane_map import load_lane_map

REPO = Path(__file__).resolve().parents[3]
# 동쪽 물체의 동쪽에서 북서쪽 충전 자리로: 남쪽 통로로 가면 두 물체의 아래 모서리에 붙어
# 틈의 남쪽 출구 앞을 지난다(북쪽 통로는 시험에서 막아 둔다)
EAST, WEST = (1.45, 0.45), (0.19, 0.80)
NORTH = {"x": (1.0, 1.3), "y": (0.6, 1.0)}  # 북쪽 통로의 한 토막


def core_for(name="arena_1110"):
    site = load_lane_map(REPO / "sim" / "fake_robots" / "maps" / f"{name}.yaml")
    params = replace(load_params(), traffic_mode="freefloor")
    core = FleetManager(site, params, ["a"], lambda: 0.0)
    core.on_robot_state("a", *EAST, 0.0, "IDLE")
    return core


def corner(traffic, cell) -> tuple[float, float]:
    """칸 (i, j)의 왼쪽 아래 모서리 좌표."""
    floor = traffic.floor
    return floor.origin[0] + cell[0] * floor.cell_m, floor.origin[1] + cell[1] * floor.cell_m


def south_route(traffic):
    """북쪽 통로를 막고 찾은 길(남쪽 통로로 틈 출구 앞을 지난다)."""
    m = traffic._missions["a"]
    block = frozenset(c for c in traffic.floor.clear
                      if NORTH["x"][0] < corner(traffic, c)[0] < NORTH["x"][1]
                      and corner(traffic, c)[1] > NORTH["y"][0])
    return traffic._route(m, EAST, WEST, extra=block)


def body_on_route(traffic, route) -> set:
    step, cells = 0.01, set()
    for k in range(int(route.length / step) + 1):
        cells |= set(disk_cells(traffic.floor, *route.at(min(route.length, k * step)),
                                traffic._body))
    return cells


def dock_cells(traffic, station="WORK") -> set:
    return set(disk_cells(traffic.floor, *traffic.floor.docks[station], traffic._body))


def test_a_docked_robot_has_cells_a_careless_way_past_the_mouth_would_touch():
    """이 시험이 보는 칸이 실제로 있다: 도크 몸 둘레 칸이 남쪽 통로 쪽으로 나와 있다."""
    traffic = core_for().traffic
    low = min(corner(traffic, c)[1] for c in dock_cells(traffic))
    assert low < 0.40  # 동쪽 물체의 아래 끝(y 0.40)보다 아래


def test_a_way_past_the_mouth_keeps_the_body_off_the_cells_of_a_docked_robot():
    traffic = core_for().traffic
    route = south_route(traffic)
    assert route is not None and min(y for _, y in route.points) < 0.35  # 남쪽 통로로 갔다
    assert body_on_route(traffic, route).isdisjoint(dock_cells(traffic))


def test_without_another_way_the_route_still_goes_past_the_dock():
    """도크 칸을 피하면 길이 없을 때는 예전처럼 찾는다(길 없음으로 끝내지 않는다)."""
    traffic = core_for().traffic
    m = traffic._missions["a"]
    traffic._dock_keep = {c for c in traffic.floor.clear if 0.7 < corner(traffic, c)[0] < 1.2}
    assert traffic._route(m, EAST, WEST) is not None


def test_a_way_into_and_out_of_the_dock_itself_is_still_planned():
    traffic = core_for().traffic
    m = traffic._missions["a"]
    dock = traffic.floor.docks["WORK"]
    assert traffic._route(m, (0.45, 0.80), dock, in_line="work_in") is not None
    assert traffic._route(m, dock, traffic.floor.docks["OUTBOUND"], "work_out",
                          "outbound_in") is not None


# ---- arena_v6: 갈 길이 있는 로봇이 통로에 서 있을 때 (Gazebo 2026-11-12) ----
#
# 북쪽 통로에서 OUTBOUND 차례를 기다리며 선 로봇을 벽으로 피하니, 뒤따르는 로봇의 길이 긴 진열대 사이
# 통로(폭 0.23 m)를 동쪽으로 지나 도크 자리 옆 3 cm를 스쳐 접근선 시작점으로 갔다. Gazebo에서 그 로봇이
# 통로 입구 모서리에 걸려 위치 추정이 0.2 m 밀렸고, 나가는 로봇과 반대로 돌아 두 대가 붙었다.

def v6_core():
    site = load_lane_map(REPO / "sim" / "fake_robots" / "maps" / "arena_v6.yaml")
    params = replace(load_params(), traffic_mode="freefloor")
    clock = [0.0]
    core = FleetManager(site, params, ["a", "b"], lambda: clock[0])
    core.on_robot_state("a", 0.98, 0.84, 0.0, "IDLE")  # 북쪽 통로 서쪽 입구
    core.on_robot_state("b", 1.46, 0.72, 0.0, "IDLE")  # 북쪽 통로 안
    for _ in range(5):
        clock[0] += 0.1
        core.tick()
    return core


def way_to_outbound(traffic):
    m = traffic._missions["a"]
    return traffic._route(m, (0.98, 0.84), traffic.floor.docks["OUTBOUND"], in_line="outbound_in")


def free_part(traffic, route):
    """접근선에 닿기 전까지의 길."""
    line_at = min(s0 for s0, _, _ in route.fixed)
    return [route.at(k * 0.01) for k in range(int(line_at / 0.01) + 1)]


def test_a_robot_parked_in_the_passage_is_still_driven_around():
    """갈 길이 없는(주차했거나 격리된) 로봇 뒤에는 줄 서지 않는다: 예전처럼 다른 길을 찾는다."""
    traffic = v6_core().traffic
    route = way_to_outbound(traffic)
    assert route is not None
    assert min(y for _, y in free_part(traffic, route)) < 0.5  # 북쪽 통로를 버리고 돌아간다


def test_behind_a_robot_that_waits_its_turn_the_way_queues_instead_of_passing_the_dock():
    core = v6_core()
    traffic = core.traffic
    b = traffic._missions["b"]
    traffic._start_leg(b, "TO_PARK", None, "QO")  # b는 갈 길이 있다
    leg = traffic._legs["b"]
    assert leg.route is not None
    leg.stuck_since = 0.0  # 앞이 막혀 서서 기다리는 중이다(지나가는 로봇이 아니다)
    assert not traffic._passing("b")
    route = way_to_outbound(traffic)
    assert route is not None
    assert min(y for _, y in free_part(traffic, route)) > 0.6  # 북쪽 통로로 간다: b 뒤에서 기다리게 된다


def test_a_robot_waits_for_the_line_on_the_side_the_line_starts_not_behind_the_dock():
    """대기 자리는 접근선 시작점에서 직선으로 가까운 자리가 아니라, 도크 자리를 지나지 않고 접근선에
    닿는 자리다. 직선으로는 긴 진열대 사이 통로 안(1.52, 0.42)이나 도크 남쪽 구석(1.86, 0.18)이
    가깝지만, 거기서는 도크를 통과해야 접근선에 닿는다. Gazebo와 머리 없는 시뮬에서 로봇이 좁은 통로를
    거슬러 그 자리로 갔다(2026-11-12)."""
    from fleet_core.traffic.floor.controller import _Leg
    site = load_lane_map(REPO / "sim" / "fake_robots" / "maps" / "arena_v6.yaml")
    params = replace(load_params(), traffic_mode="freefloor")
    clock = [0.0]
    core = FleetManager(site, params, ["a", "b"], lambda: clock[0])
    core.on_robot_state("a", 0.50, 0.86, 0.0, "IDLE")
    core.on_robot_state("b", 1.61, 0.83, 0.0, "IDLE")  # 북쪽 통로 동쪽 끝: 접근선 앞자리를 쥐고 있다
    for _ in range(5):
        clock[0] += 0.1
        core.tick()
    traffic = core.traffic
    m = traffic._missions["a"]
    leg = _Leg(None, traffic.floor.docks["OUTBOUND"], in_line="outbound_in")
    leg.waiting = True
    traffic._legs["a"] = leg
    traffic._plan_wait(m, leg, None)
    if leg.route is not None:  # 지금 갈 자리가 없으면 None(다음 주기에 다시 찾는다)
        x, y = leg.route.points[-1]
        assert y > 0.6 or x < 1.0, leg.route.points[-1]  # 도크 남쪽이나 긴 진열대 사이로 가지 않는다


def test_the_way_from_a_wait_spot_to_the_line_is_measured_around_the_dock():
    traffic = v6_core().traffic
    far = traffic._approach_m("outbound_in")
    floor = traffic.floor
    from fleet_core.traffic.floor.controller import WAIT_SPOT_M
    limit = WAIT_SPOT_M[1]  # 대기 자리로 치는 가장 먼 거리
    assert far[floor.cell_of(1.60, 0.80)] < 0.5  # 북쪽 통로: 접근선 시작점 바로 서쪽
    assert far[floor.cell_of(1.86, 0.18)] > limit  # 도크 남쪽 구석: 직선 0.55 m, 돌아가면 2 m 넘는다
    assert far[floor.cell_of(1.50, 0.41)] > limit  # 긴 진열대 사이 통로: 직선 0.48 m, 돌아가면 1.5 m
