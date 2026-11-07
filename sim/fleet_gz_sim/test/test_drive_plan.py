"""관제 흉내 주행 계획(경로 늘리기)과 지표 계산. ROS 없이 돈다."""
import math

import pytest
from fleet_gz_sim.drive_plan import (
    Trip,
    full_stop_after,
    lap_stops,
    settle_time,
    speeds,
    still_episodes,
    stopped_after,
    time_to_stop,
    turn_speeds,
)

POINTS = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0), (4.0, 0.0)]


def test_first_path_reaches_two_waypoints_ahead():
    trip = Trip(POINTS, ahead=2)
    assert trip.first() == [0, 1, 2]


def test_path_grows_at_the_middle_of_each_segment_and_starts_at_the_last_passed_waypoint():
    trip = Trip(POINTS, ahead=2, extend_at=0.5)
    trip.first()
    assert trip.update(0.4, 0.0) is None
    assert trip.update(0.5, 0.01) == [0, 1, 2, 3]
    assert trip.update(0.9, 0.0) is None
    assert trip.update(1.2, 0.0) is None  # 1번 지점을 지났지만 아직 구간 중간 전
    assert trip.update(1.6, 0.0) == [1, 2, 3, 4]
    assert trip.update(2.7, 0.0) is None  # 끝까지 줬다
    assert trip.granted_all()


def test_a_waypoint_counts_as_passed_when_the_robot_cuts_the_corner():
    trip = Trip([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (1.0, 2.0), (1.0, 3.0)], ahead=2)
    trip.first()
    assert trip.update(0.5, 0.0) == [0, 1, 2, 3]
    assert trip.update(0.95, 0.04) is None  # 모서리 0.1 m 안: 1번 지점을 지난 것으로 친다
    assert trip.update(1.0, 0.5) == [1, 2, 3, 4]


def test_laps_start_and_end_at_the_stop_the_robot_is_on():
    cycle = ("P-3", "W", "O")
    assert lap_stops(cycle, "P-3", 1) == ["P-3", "W", "O", "P-3"]
    assert lap_stops(cycle, "W", 2) == ["W", "O", "P-3", "W", "O", "P-3", "W"]


def test_short_trip_is_granted_whole_at_once():
    trip = Trip(POINTS[:2], ahead=2)
    assert trip.first() == [0, 1] and trip.granted_all()


def test_stop_after_an_extension_is_a_drop_from_driving_to_zero():
    driving = [(t / 20, 0.25) for t in range(0, 40)]
    assert stopped_after(driving, 1.0) is False
    dropped = [(t / 20, 0.25 if t < 22 else 0.0) for t in range(0, 40)]
    assert stopped_after(dropped, 1.0) is True
    standing = [(t / 20, 0.0) for t in range(0, 40)]
    assert stopped_after(standing, 1.0) is None  # 서 있던 중이면 판정하지 않는다


def test_full_stop_needs_both_speeds_at_zero():
    """모서리 제자리 회전(선속도 0, 각속도 있음)은 연장 때문의 정지가 아니다."""
    driving = [(t / 20, 0.25, 0.0) for t in range(0, 20)]
    turning = driving + [(1 + t / 20, 0.0, 1.0) for t in range(0, 20)]
    assert full_stop_after(turning, 1.0) is False
    halted = driving + [(1 + t / 20, 0.0, 0.0) for t in range(0, 20)]
    assert full_stop_after(halted, 1.0) is True
    assert full_stop_after([(t / 20, 0.0, 1.0) for t in range(40)], 1.0) is None


def test_a_zero_crossing_while_switching_between_turning_and_driving_is_not_a_stop():
    """RPP가 모서리에서 제자리 회전과 주행을 오가면 각속도가 0을 한 표본 지난다(Gazebo 측정)."""
    driving = [(t / 20, 0.25, 0.25) for t in range(0, 20)]
    chatter = [(0.25, 0.39), (0.0, -0.007), (0.0, 0.073), (0.25, -0.97), (0.25, -0.42),
               (0.0, 0.039), (0.0, 0.073), (0.25, -0.83)]
    after = [(1.4 + t / 20, 0.25, 0.25) for t in range(0, 6)]
    samples = driving + [(1 + i / 20, v, w) for i, (v, w) in enumerate(chatter)] + after
    assert full_stop_after(samples, 1.0) is False
    halted = driving + [(1 + t / 20, 0.0, 0.0) for t in range(0, 4)]  # 0.15 s
    assert full_stop_after(halted + [(1.2 + t / 20, 0.25, 0.0) for t in range(0, 8)], 1.0) is True


def test_a_window_cut_short_by_the_result_is_not_judged():
    """결과가 연장 뒤 0.5 s 안에 와서 기록이 끝나면 정지 여부를 모른다(Codex 리뷰)."""
    driving = [(t / 20, 0.25, 0.0) for t in range(0, 25)]  # 1.2 s에서 기록이 끝난다
    assert full_stop_after(driving, 1.0) is None
    assert stopped_after([(t, v) for t, v, _ in driving], 1.0) is None


def test_turn_speed_from_headings_wraps_around_pi():
    heading = [(t / 10, 3.0 + 0.05 * t) for t in range(10)]  # +0.5 rad/s, π를 넘어간다
    wrapped = [(t, math.remainder(y, math.tau)) for t, y in heading]
    rate = dict(turn_speeds(wrapped, window=0.2))
    assert rate[0.5] == pytest.approx(0.5)
    assert rate[0.9] == pytest.approx(0.5)


def test_still_episodes_count_runs_that_last_long_enough():
    v = [0.2] * 10 + [0.0] * 4 + [0.2] * 10 + [0.0] * 1 + [0.2] * 5 + [0.0] * 6
    samples = [(i / 20, x) for i, x in enumerate(v)]
    # 0.15 s 정지, 0.05 s 깜빡임(세지 않음), 끝의 0.25 s 정지
    assert still_episodes(samples, 0.0, 10.0, min_s=0.1) == 2
    assert still_episodes(samples, 0.0, 1.0, min_s=0.1) == 1


def test_speed_from_positions_over_a_window():
    truth = [(t / 10, (0.02 * t if t <= 10 else 0.2, 0.0)) for t in range(30)]
    speed = dict(speeds(truth, window=0.2))
    assert speed[0.5] == pytest.approx(0.2)
    assert speed[2.0] == pytest.approx(0.0)


def test_time_to_stop_from_a_moment():
    samples = [(t / 10, 0.2 if t < 13 else 0.0) for t in range(30)]
    assert time_to_stop(samples, 0.0) == pytest.approx(1.3)
    assert time_to_stop([(0.0, 0.2), (1.0, 0.2)], 0.0) is None


def test_time_to_stop_needs_the_stop_to_last():
    """감속 중 한 표본만 낮아졌다가 다시 움직이면 정지가 아니다(Codex 리뷰)."""
    v = [0.2] * 11 + [0.0] + [0.2] * 8 + [0.0] * 10  # 1.1 s에 한 표본, 2.0 s부터 계속
    samples = [(i / 10, x) for i, x in enumerate(v)]
    assert time_to_stop(samples, 0.0) == pytest.approx(2.0)
    assert time_to_stop(samples[:22], 0.0) is None  # 2.0 s 정지가 0.3 s 이어지는 것을 못 봤다


def test_a_command_holds_after_the_last_zero_until_the_record_ends():
    """controller는 목표가 끝나면 0을 한 번 내고 발행을 멈춘다. 마지막 값이 이어진다."""
    cmd = [(0.0, 0.2), (0.05, 0.2), (0.1, 0.0)]
    assert time_to_stop(cmd, 0.0, end=1.0) == pytest.approx(0.1)
    assert time_to_stop(cmd, 0.0) is None  # 기록이 언제 끝났는지 모르면 이어졌는지 모른다


def test_a_gap_in_a_streamed_record_is_not_seen_as_a_stop():
    """위치 기록은 계속 오는 표본이다. 비어 있던 동안 서 있었는지는 모른다(Codex 재리뷰)."""
    assert time_to_stop([(0.0, 0.0), (1.0, 0.0)], 0.0, max_gap=0.1) is None
    truth = [(0.0, 0.0)] + [(1.0 + i / 20, 0.0) for i in range(8)]
    assert time_to_stop(truth, 0.0, max_gap=0.1) == pytest.approx(1.0)


def test_settle_time_counts_from_the_last_entry_into_the_radius():
    target = (1.0, 0.0)
    truth = [(0.0, (0.0, 0.0)), (1.0, (0.9, 0.0)), (2.0, (0.7, 0.0)),
             (3.0, (0.9, 0.0)), (4.0, (0.95, 0.0)), (5.0, (0.97, 0.0))]
    assert settle_time(truth, target, 5.0) == pytest.approx(2.0)
    assert settle_time([(0.0, (0.0, 0.0))], target, 1.0) is None
