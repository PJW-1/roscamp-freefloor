"""배터리 모형: low_battery 고장이면 낮아지고 충전 자리에 서 있으면 다시 찬다. ROS가 필요 없다."""
import pytest
from fake_robots.battery import CHARGE_NEAR_M, FULL_PCT, LOW_PCT, Battery
from fake_robots.scenario import Fault

SPOT = (1.0, 2.0)
AWAY = (3.0, 2.0)


def low(at_s, duration_s, robot="pinky1"):
    return Fault(at_s=at_s, robot=robot, type="low_battery", duration_s=duration_s)


def test_without_an_event_the_battery_stays_full():
    b = Battery([], [SPOT])
    assert [b.percent(t, *SPOT, True) for t in (0.0, 50.0, 500.0)] == [FULL_PCT] * 3


def test_other_fault_types_are_not_battery_events():
    other = Fault(at_s=1.0, robot="pinky1", type="comm_loss", duration_s=5.0)
    assert Battery([other], [SPOT]).percent(10.0, *AWAY, False) == FULL_PCT


def test_the_level_drops_at_the_event_and_stays_low_away_from_a_spot():
    b = Battery([low(40.0, 30.0)], [SPOT])
    assert b.percent(39.9, *AWAY, False) == FULL_PCT
    assert b.percent(40.0, *AWAY, False) == LOW_PCT
    assert b.percent(200.0, *AWAY, True) == LOW_PCT  # 충전 자리가 아니다


def test_standing_at_a_spot_charges_linearly_over_the_duration():
    b = Battery([low(40.0, 30.0)], [SPOT])
    assert b.percent(40.0, *SPOT, True) == LOW_PCT
    assert b.percent(55.0, *SPOT, True) == pytest.approx((LOW_PCT + FULL_PCT) / 2)
    assert b.percent(70.0, *SPOT, True) == FULL_PCT
    assert b.percent(300.0, *AWAY, False) == FULL_PCT  # 끝난 사건은 다시 낮아지지 않는다


def test_charging_counts_from_the_event_time_not_the_last_call():
    b = Battery([low(40.0, 30.0)], [SPOT])
    assert b.percent(30.0, *SPOT, True) == FULL_PCT
    assert b.percent(55.0, *SPOT, True) == pytest.approx((LOW_PCT + FULL_PCT) / 2)


def test_moving_or_leaving_the_spot_pauses_the_charge():
    b = Battery([low(0.0, 10.0)], [SPOT])
    b.percent(0.0, *SPOT, True)
    half = b.percent(5.0, *SPOT, True)
    assert half == pytest.approx((LOW_PCT + FULL_PCT) / 2)
    assert b.percent(20.0, *SPOT, False) == pytest.approx(half)  # 자리 위지만 움직이는 중
    assert b.percent(30.0, *AWAY, True) == pytest.approx(half)  # 자리 밖
    assert b.percent(30.0, *SPOT, True) == pytest.approx(half)
    assert b.percent(35.0, *SPOT, True) == FULL_PCT


def test_near_means_within_charge_near_m_of_any_spot():
    other = (5.0, 5.0)
    b = Battery([low(0.0, 10.0)], [SPOT, other])
    b.percent(0.0, 5.0 + CHARGE_NEAR_M * 0.9, 5.0, True)
    assert b.percent(5.0, 5.0 + CHARGE_NEAR_M * 0.9, 5.0, True) > LOW_PCT
    c = Battery([low(0.0, 10.0)], [SPOT, other])
    c.percent(0.0, 5.0 + CHARGE_NEAR_M * 1.1, 5.0, True)
    assert c.percent(5.0, 5.0 + CHARGE_NEAR_M * 1.1, 5.0, True) == LOW_PCT


def test_later_events_work_the_same_way():
    b = Battery([low(100.0, 10.0), low(10.0, 10.0)], [SPOT])  # 순서는 at_s로
    assert b.percent(10.0, *SPOT, True) == LOW_PCT
    assert b.percent(20.0, *SPOT, True) == FULL_PCT
    assert b.percent(99.0, *SPOT, True) == FULL_PCT
    assert b.percent(100.0, *SPOT, True) == LOW_PCT
    assert b.percent(105.0, *SPOT, True) == pytest.approx((LOW_PCT + FULL_PCT) / 2)
    assert b.percent(110.0, *SPOT, True) == FULL_PCT


def test_an_event_due_before_the_last_one_ended_starts_when_it_ends():
    b = Battery([low(10.0, 10.0), low(12.0, 10.0)], [SPOT])
    b.percent(10.0, *AWAY, False)
    assert b.percent(15.0, *AWAY, False) == LOW_PCT  # 둘째 시각이 지났어도 첫째가 아직이다
    b.percent(15.0, *SPOT, True)
    assert b.percent(25.0, *SPOT, True) == LOW_PCT  # 첫째가 끝나자 둘째가 바로 시작
    assert b.percent(35.0, *SPOT, True) == FULL_PCT


def test_raw_fault_dicts_work_too():
    """머리 없는 세계(sim_world)는 시나리오의 고장을 dict로 쓴다."""
    b = Battery([{"type": "low_battery", "robot": "pinky1", "at_s": 0.0, "duration_s": 10.0}],
                [SPOT])
    b.percent(0.0, *SPOT, True)
    assert b.percent(10.0, *SPOT, True) == FULL_PCT


def test_no_spots_means_no_charging():
    b = Battery([low(0.0, 10.0)], [])
    b.percent(0.0, *SPOT, True)
    assert b.percent(100.0, *SPOT, True) == LOW_PCT
