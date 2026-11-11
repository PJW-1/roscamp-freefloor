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
