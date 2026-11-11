"""마주 오는 로봇: 서로의 자리로 가는 두 대, 돌아가며 자리를 바꾸는 세 대.

평소 주문 흐름은 한 바퀴를 돌아 교착이 잘 나지 않는다(arena_v6). 사용자가 교착이 날 만한 상황을
만들어 고치라고 했다(2026-11-11). tools/stress_floor.py로 서로의 자리로 가는 두 대를 지점의 여러
쌍으로 돌리니 절반쯤이 끝나지 않았다: 교착을 감지하고도 "비켜 설 빈 자리가 없다"며 풀지
못하고 로봇을 격리했다. 넓은 바닥에서도 그랬다. 비켜 서는 로봇이 상대가 가려는 길을 통째로 피해서
경로를 짰는데, 마주 오는 상대의 길은 바로 제 자리를 지나므로 출발 칸 둘레가 모두 막혔다."""
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))
import stress_floor  # noqa: E402

BENCH = stress_floor.Bench("arena_v6", {}, 0.03)
OPEN_FLOOR = [("SC", "SM"), ("QW", "WS"), ("SM", "WS"), ("SC", "WS")]  # 넓은 남서쪽 바닥


def swap(a, b, bench=BENCH, seed=1, limit_s=90.0):
    return bench.run({"pinky1": a, "pinky2": b}, {"pinky1": b, "pinky2": a}, seed, limit_s)


@pytest.mark.parametrize("pair", OPEN_FLOOR)
def test_two_robots_swapping_places_on_open_floor_both_arrive(pair):
    row = swap(*pair)
    assert row["result"] == "ok", row
    assert row["DEADLOCK_UNRESOLVED"] == 0 and row["ROBOT_ISOLATED"] == 0, row


def test_the_robot_that_steps_aside_stands_clear_of_the_way_the_other_takes():
    """비켜 선 자리는 상대의 길 밖이다: 상대가 지나가고, 비켜 선 로봇이 뒤따라 도착한다."""
    row = swap("SC", "SM")
    assert row["FLOOR_SIDESTEP"] >= 1, row
    assert row["result"] == "ok" and row["longest_stop_s"] < 30.0, row


# 62c82e6 뒤에도 끝나지 않던 쌍(2026-11-11 밤 스트레스). 둘이 잠금 끝까지 달려 중심 0.18~0.2 m로 붙은
# 뒤에야 순환이 보였고, 그때는 제자리에서 돌 원도 잠글 수 없어 비켜 서지 못했다
CLOSE_UP = [("NA", "QO"), ("MC", "SE"), ("NC", "QO"), ("SE", "WS")]
CLOSE_UP_KEEP_RIGHT = [("NA", "QO"), ("NC", "QO"), ("QO", "SC")]
BENCH_KR = stress_floor.Bench("arena_v6", {"floor_keep_right": True}, 0.03)


@pytest.mark.parametrize("pair", CLOSE_UP)
def test_robots_that_would_end_nose_to_nose_stop_early_and_both_arrive(pair):
    row = swap(*pair)
    assert row["result"] == "ok", row
    assert row["DEADLOCK_UNRESOLVED"] == 0 and row["ROBOT_ISOLATED"] == 0, row


@pytest.mark.parametrize("pair", CLOSE_UP_KEEP_RIGHT)
def test_the_same_with_keep_right(pair):
    row = swap(*pair, bench=BENCH_KR)
    assert row["result"] == "ok", row
    assert row["DEADLOCK_UNRESOLVED"] == 0 and row["ROBOT_ISOLATED"] == 0, row


def test_a_robot_halted_before_a_facing_one_is_let_go_once_the_two_pass_side_by_side():
    """일찍 선 로봇이 나란히 지나가기에 들면 선 표시를 지운다: 남아 있으면 잠금이 늘 때까지 목표를
    보내지 않아, 제 줄이 이미 잡혀 있어도 출발하지 못한다(Codex 리뷰 2026-11-11, 8bb8a9e)."""
    from floor_world import FloorWorld
    robots = {n: {"start": s, "speed_mps": 0.15, "pose_noise_m": 0.0}
              for n, s in (("pinky1", "SC"), ("pinky2", "P-2"), ("pinky3", "P-3"))}
    world = FloorWorld(seed=1, orders=[], robots=robots, site=BENCH_KR.lanes,
                       floor=BENCH_KR.floor, params=BENCH_KR.params)
    traffic = world.core.traffic
    traffic._spot_taken = lambda m: False
    world._report_states()
    m = traffic._missions["pinky1"]
    traffic._start_leg(m, "TO_PARK", None, "SM")
    leg = traffic._legs["pinky1"]
    leg.halt_on, leg.halt_lock = leg.route, leg.s_lock
    assert traffic._halted(leg)
    traffic._passes["pinky1"] = object()  # 나란히 지나가기에 들었다
    traffic._halt_head_on(m, leg, grew=False)
    assert not traffic._halted(leg)


def test_three_robots_taking_each_others_places_park_on_different_spots_afterwards():
    """측정 도구: 목표에 닿은 로봇은 주차 자리 규칙대로 빈 자리로 간다. 규칙을 계속 꺼 두면 두 로봇이
    같은 주차 자리로 가서 하나가 영영 기다리고, 그 로봇이 남의 목표를 막아 '못 닿음'으로 셌다."""
    row = BENCH.run({"pinky1": "MC", "pinky2": "SM", "pinky3": "WS"},
                    {"pinky1": "SM", "pinky2": "WS", "pinky3": "MC"}, 1, 90.0)
    assert row["result"] == "ok", row


def boxed_in_world():
    """pinky1이 넓은 남서쪽 바닥(SM)에 서 있고, 동쪽과 서쪽 0.12~0.15 m에 남(pinky2)이 쥔 칸이 있다:
    몸(반지름 0.11 m)은 닿지 않지만 pinky1의 자리와 둘레 칸이 모두 '남의 칸에 몸이 닿을 수 있는 칸'
    (넓힌 칸) 안이다. 남북으로는 비어 있다."""
    from floor_world import FloorWorld
    robots = {n: {"start": s, "speed_mps": 0.15, "pose_noise_m": 0.0}
              for n, s in (("pinky1", "SM"), ("pinky2", "P-2"), ("pinky3", "P-3"))}
    world = FloorWorld(seed=1, orders=[], robots=robots, site=BENCH.lanes, floor=BENCH.floor,
                       params=BENCH.params)
    traffic = world.core.traffic
    traffic._spot_taken = lambda m: False
    world.virtual["pinky1"].x -= 0.008  # 칸 가운데가 아니라 서쪽 끝에 선다(양옆 칸까지의 거리가 다르다)
    world._report_states()
    m = traffic._missions["pinky1"]
    traffic._start_leg(m, "TO_PARK", None, "SC")
    x, y = traffic._here(m)
    floor = traffic.floor
    i0, j0 = floor.cell_of(x, y)
    east = next(di for di in range(1, 12) if floor.box_distance((i0 + di, j0), x, y) > 0.115)
    west = next(di for di in range(1, 12) if floor.box_distance((i0 - di, j0), x, y) > 0.115)
    side = {(i0 + di, j0 + dj) for di in (east, east + 1, -west, -west - 1)
            for dj in range(-3, 4)}  # 양옆으로 몸 바로 밖 두 줄
    for cell in side:  # 서 있는 로봇은 둘레 0.17 m를 쥔다: 달려와 선 로봇처럼 몸 둘레만 남긴다
        traffic.zones.release("pinky1", floor.cell_id(cell))
    assert traffic.zones.acquire_all("pinky2", {floor.cell_id(c) for c in side})
    return traffic, m


def test_a_robot_standing_too_close_to_cells_of_another_has_a_straight_step_out():
    from fleet_core.traffic.floor.cells import swept_cells
    traffic, m = boxed_in_world()
    floor, here = traffic.floor, traffic._here(m)
    others = traffic._others_cells("pinky1")
    avoid = traffic._inflate(others)
    i, j = floor.cell_of(*here)
    assert all((i + di, j + dj) in avoid for di in (-1, 0, 1) for dj in (-1, 0, 1))  # 둘레가 다 막혔다
    steps = traffic._escape_steps(here, others, avoid)
    assert steps
    for p in steps:
        assert floor.cell_of(*p) in floor.clear and floor.cell_of(*p) not in avoid
        assert not swept_cells(floor, [here, p], traffic._body) & others  # 몸이 남의 칸에 닿지 않는다
        assert abs(p[1] - here[1]) > abs(p[0] - here[0])  # 남이 없는 남북 쪽이다


def test_it_steps_aside_from_there_instead_of_finding_no_free_spot():
    """예전: 출발 칸 둘레가 모두 넓힌 칸이라 어느 빈 자리로도 길이 없었다('no free spot'). 세 대가
    0.2 m 간격으로 모인 판(ring MB>SC>WS>MB)에서 아무도 비켜 서지 못하고 격리됐다."""
    traffic, m = boxed_in_world()
    here = traffic._here(m)
    assert traffic._sidestep(m, ["pinky1", "pinky2"])
    route = traffic._legs["pinky1"].route
    assert route.points[0] == pytest.approx(here)
    others = traffic._others_cells("pinky1")
    assert traffic.floor.cell_of(*route.points[1]) not in traffic._inflate(others)


def test_two_robots_that_wait_for_each_other_at_a_right_angle_stop_at_once():
    """서로를 기다리게 된 두 로봇은 마주 보지 않아도(직각으로 만나도) 잠금 끝까지 달리지 않고 선다.
    끝까지 달리면 0.18 m로 붙어 둘 다 제자리에서 돌 원을 잡지 못했고, 비켜 설 자리가 없다고
    알렸다(ring QW>SC>WS>QW: WS로 오는 로봇과 서쪽 통로를 내려오는 로봇)."""
    row = BENCH.run({"pinky1": "QW", "pinky2": "SC", "pinky3": "WS"},
                    {"pinky1": "SC", "pinky2": "WS", "pinky3": "QW"}, 1, 90.0)
    assert row["result"] == "ok", row
    assert row["DEADLOCK_UNRESOLVED"] == 0 and row["ROBOT_ISOLATED"] == 0, row
