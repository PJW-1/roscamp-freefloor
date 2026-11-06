"""세 로봇에 대해 셀 배치의 일방통행 레인과 구역 잠금을 모두 탐색한다.

zone_model_check.py에서 옮겼다. 잠금 그래프만 보고 모든 배치를 탐색하며,
실제 코어는 tools/model_check.py가 돌린다. 이동은 실제 프로토콜처럼 두 단계다:
acquire(구역 A의 로봇이 B를 예약하고 A를 유지)와 complete(A를 떠나 A를 놓음).

보이는 것과 보이지 않는 것:
  - 실제 배정 정책이 닿는 배치만이 아니라 모든 배치를 탐색한다.
  - "교착 없음"은 어떤 로봇은 항상 움직일 수 있다는 뜻이고, "기아 없음"은
    어떤 순서에서는 로봇이 다시 움직일 수 있다는 뜻이다. 공정성은 zones.py의
    FIFO 큐가 준다.
  - 진행 중인 이동은 끝난다고 가정한다. 이동 중 멈춤은 고정 로봇 표만 다루며,
    경로 칸을 로봇이 쥔 구역과 대조한다.
  - 기하, Nav2 경로, 시간은 모델링하지 않는다.
  - 정차 중 마음을 바꾸는 것도 이동으로 센다. 정차한 로봇은 막히지 않는다.
  - FIFO 없음, 지난 구역은 즉시 놓음, 경로는 정차 때만 바뀐다. 그래서 잠금
    그래프의 빠른 회귀 테스트일 뿐이고, 근거는 tools/model_check.py다.
"""
import itertools
from collections import defaultdict

import pytest
from fleet_core.traffic.lane_map import load_lane_map
from grid_map import grid_site

ROBOTS = 3
WORK, OUTBOUND = (5, 3), (9, 3)

# 대조군: 복귀가 출고 레인을 거슬러 가므로 두 로봇이 정면으로 마주칠 수 있다.
# 검사기가 여기서 교착을 보고하지 못하면 설계의 "없음"은 의미가 없다.
CONTROL = (33480, 66, 8304)  # 실측
CLEARANCE, RELEASE = 0.16, 0.2  # 2r, d


def design_graph(site):
    """그래프의 점은 (칸, 레인 이름)이다. 여기서 칸에는 레인이 하나라 이름은 비어 있다."""
    succ = defaultdict(set)
    for a, b in site.lanes:
        succ[(a, "")].add((b, ""))
    return succ


def head_on_graph(site):
    """출고 레인을 양방향으로 달리게 한다. 같은 칸 위의 별개 트랙이다."""
    out = site.plan(WORK, OUTBOUND)
    succ = defaultdict(set)

    def track(cells, name):
        for a, b in zip(cells, cells[1:], strict=False):
            succ[(a, "" if a in site.docks else name)].add((b, "" if b in site.docks else name))

    track(out, "out")
    track(out[::-1], "back")
    track(site.parking, "wait")
    return succ


def route_graph(site):
    """점은 (지점, 정차)다. 정차 사이에는 계획기가 준 경로만 따르므로 design_graph와
    달리 반대편에서 오는 로봇을 피할 수 없다. 정차 때는 다른 곳으로 보낼 수 있다.
    """
    stops = {name: station.dock for name, station in site.stations.items()}
    stops["PARK"] = site.parking[-1]
    free_to_turn = set(stops.values()) | set(site.parking)
    succ = defaultdict(set)
    for node in site.zone_of:
        for name, goal in stops.items():
            if node != goal:
                succ[(node, name)].add((site.plan(node, goal)[1], name))
            if node in free_to_turn:
                succ[(node, name)].update((node, other) for other in stops if other != name)
    return succ


def check(site, succ, robots=ROBOTS, conflicts=()):
    """모든 배치에 대해 (상태 수, 교착, 굶는 로봇 상태)를 돌려준다."""
    zone_of = site.zone_of
    nodes = set(succ) | {v for vs in succ.values() for v in vs}
    single = [(n,) for n in sorted(nodes, key=str)]
    single += [(u, v) for u in sorted(succ, key=str) for v in sorted(succ[u], key=str)]
    near = defaultdict(set)  # 구역과 충돌 쌍인 구역들
    for pair in conflicts:
        one, other = tuple(pair)
        near[one].add(other)
        near[other].add(one)
    held = {s: frozenset(zone_of[n[0]] for n in s) for s in single}
    blocks = {s: held[s].union(*(near[z] for z in held[s])) for s in single}

    states = [
        combo for combo in itertools.product(single, repeat=robots)
        if all(blocks[a].isdisjoint(held[b]) and blocks[b].isdisjoint(held[a])
               for a, b in itertools.combinations(combo, 2))
    ]

    forward = {}
    deadlocks = 0
    for state in states:
        per_robot = []
        for i, me in enumerate(state):
            if len(me) == 2:  # 이동 중: 옛 칸을 떠나는 데 허가가 필요 없다
                per_robot.append([state[:i] + ((me[1],),) + state[i + 1:]])
                continue
            busy = set().union(*(blocks[s] for j, s in enumerate(state) if j != i))
            per_robot.append([
                state[:i] + ((me[0], v),) + state[i + 1:]
                for v in succ.get(me[0], ()) if zone_of[v[0]] not in busy
            ])
        forward[state] = per_robot
        if not any(per_robot):
            deadlocks += 1

    # 로봇 i는 어떤 이동 순서로도 i가 다시 움직일 수 없으면 그 상태에서 굶는다.
    reverse = defaultdict(list)
    for state, per_robot in forward.items():
        for targets in per_robot:
            for target in targets:
                reverse[target].append(state)
    starved = 0
    for i in range(robots):
        seen = {s for s in states if forward[s][i]}
        stack = list(seen)
        while stack:
            for prev in reverse[stack.pop()]:
                if prev not in seen:
                    seen.add(prev)
                    stack.append(prev)
        starved += len(states) - len(seen)
    return len(states), deadlocks, starved


def frozen_robot_table(site):
    """각 정차 칸에서 WORK-OUTBOUND 순환에 막힌 경로 칸이 없는가?"""
    loop = set(site.plan(WORK, OUTBOUND)) | set(site.plan(OUTBOUND, WORK))
    rows = defaultdict(list)
    for cell in sorted(site.zone_of):
        rows[not loop & site.zones[site.zone_of[cell]]].append(cell)
    return rows


@pytest.fixture(scope="module")
def site():
    return grid_site()


def test_design_has_no_deadlock_and_no_starvation(site):
    """지도를 레인으로 바꾸기 전과 같은 수. 변환이 그래프를 보존했다."""
    assert check(site, design_graph(site)) == (149238, 0, 0)


def test_head_on_control_finds_deadlocks(site):
    states, deadlocks, starved = check(site, head_on_graph(site))
    assert deadlocks > 0, "a checker that finds nothing here proves nothing about the design"
    assert (states, deadlocks, starved) == CONTROL


def test_frozen_robot_table(site):
    rows = frozen_robot_table(site)
    assert set(rows[True]) == set(site.parking), "only a waiting cell leaves the loop free"
    assert len(rows[True]) == 6
    assert len(rows[False]) == len(site.zone_of) - 6


# ---- the default map --------------------------------------------------------------------
#
# 잠금 그래프와 계획기가 주는 경로만 다룬다.
# tools/model_check.py가 이 지도에서 모든 호출 순서로 실제 코어를 돌리지만,
# 로봇 셋으로는 여기서 돌리기엔 너무 느리다.


def _pairs(site):
    return site.conflict_pairs(CLEARANCE, RELEASE)


def test_default_map_any_lane_at_any_time_has_no_deadlock_and_no_starvation():
    site = load_lane_map()
    assert _pairs(site) == {frozenset({"AVENUE", "S"})}
    _, deadlocks, starved = check(site, design_graph(site), conflicts=_pairs(site))
    assert (deadlocks, starved) == (0, 0)


def test_default_map_with_routes_kept_has_no_deadlock_and_no_starvation():
    site = load_lane_map()
    _, deadlocks, starved = check(site, route_graph(site), robots=2, conflicts=_pairs(site))
    assert (deadlocks, starved) == (0, 0)


def test_default_map_with_routes_kept_starves_robots_when_the_avenue_is_not_one_lock():
    site = load_lane_map(bundle_two_way=False)
    _, _, starved = check(site, route_graph(site), robots=2, conflicts=_pairs(site))
    assert starved > 0, "a checker that finds nothing here proves nothing about the bundle"
