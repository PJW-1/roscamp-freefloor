"""직선 이동, 자세 잡음, 고장 일정."""
import random

import pytest
from fake_robots.motion import (
    ArmFaults,
    DelayLine,
    FaultSchedule,
    Motion,
    abort_at_shutdown,
    finish_action,
    noisy_pose,
)
from fake_robots.scenario import Fault


def test_moves_toward_target_at_speed():
    m = Motion(0.0, 0.0)
    m.set_target(1.0, 0.0)
    m.step(1.0, 0.2)
    assert (m.x, m.y) == pytest.approx((0.2, 0.0))
    assert m.remaining() == pytest.approx(0.8)
    assert m.moving


def test_arrival_snaps_and_stops():
    m = Motion(0.0, 0.0)
    m.set_target(0.0, 0.1)
    m.step(10.0, 0.2)
    assert (m.x, m.y) == (0.0, 0.1) and m.remaining() == 0.0 and not m.moving


def test_no_target_means_no_motion():
    m = Motion(1.0, 1.0)
    m.step(5.0, 1.0)
    assert (m.x, m.y) == (1.0, 1.0)


def test_yaw_follows_heading():
    m = Motion(0.0, 0.0)
    m.set_target(0.0, 1.0)
    m.step(0.1, 0.2)
    assert m.yaw == pytest.approx(1.5707963, abs=1e-6)


def test_a_path_is_followed_segment_by_segment():
    """path 모드: 모서리를 가로지르지 않고 선분을 차례로 간다."""
    m = Motion(0.0, 0.0)
    m.set_target(1.0, 1.0, via=[(1.0, 0.0)])
    assert m.remaining() == pytest.approx(2.0)
    m.step(1.0, 0.5)
    assert (m.x, m.y) == pytest.approx((0.5, 0.0))
    m.step(1.0, 1.0)  # 모서리를 돌아 0.5 m 더 간다
    assert (m.x, m.y) == pytest.approx((1.0, 0.5))
    assert m.yaw == pytest.approx(1.5707963, abs=1e-6)
    assert m.remaining() == pytest.approx(0.5)
    m.step(5.0, 1.0)
    assert (m.x, m.y) == (1.0, 1.0) and not m.moving


def test_a_point_just_behind_is_skipped_not_driven_back_to():
    m = Motion(1.0, 0.1)  # 모서리 (1, 0)을 지나 위로 0.1 m
    m.set_target(1.0, 1.0, via=[(1.0, 0.0)])
    m.step(0.1, 1.0)
    assert (m.x, m.y) == pytest.approx((1.0, 0.2))


def test_a_corner_ahead_on_the_lane_in_is_kept():
    m = Motion(0.9, 0.0)  # 모서리 (1, 0) 앞 0.1 m
    m.set_target(1.0, 1.0, via=[(1.0, 0.0)])
    m.step(0.1, 1.0)
    assert (m.x, m.y) == pytest.approx((1.0, 0.0))


def test_clearing_the_target_drops_the_path():
    m = Motion(0.0, 0.0)
    m.set_target(1.0, 1.0, via=[(1.0, 0.0)])
    m.clear_target()
    m.set_target(0.0, 1.0)
    m.step(10.0, 1.0)
    assert (m.x, m.y) == (0.0, 1.0)


def test_noise_is_seeded_and_zero_sigma_is_exact():
    a = noisy_pose(1.0, 2.0, 0.03, random.Random(5))
    b = noisy_pose(1.0, 2.0, 0.03, random.Random(5))
    assert a == b and a != (1.0, 2.0)
    assert noisy_pose(1.0, 2.0, 0.0, random.Random(5)) == (1.0, 2.0)


def test_what_goes_in_a_delay_line_comes_out_late_and_in_order():
    line = DelayLine(0.5)
    line.push(1.0, "a")
    line.push(1.1, "b")
    assert line.due(1.4) == []
    assert line.due(1.5) == ["a"]
    assert line.due(1.7) == ["b"]
    assert line.due(9.0) == []


def test_a_delay_line_with_no_delay_lets_it_straight_through():
    line = DelayLine(0.0)
    line.push(1.0, "a")
    assert line.due(1.0) == ["a"]


def test_what_is_still_in_a_cleared_delay_line_is_lost():
    line = DelayLine(0.5)
    line.push(1.0, "a")
    line.clear()
    assert line.due(2.0) == []


def faults():
    return [
        Fault(at_s=40, robot="pinky2", type="comm_loss", duration_s=20, count=1),
        Fault(at_s=75, robot="pinky2", type="nav_abort", duration_s=0, count=2),
        Fault(at_s=90, robot="pinky2", type="stuck", duration_s=0, count=1),
        Fault(at_s=10, robot="pinky1", type="stuck", duration_s=0, count=1),
    ]


def test_comm_loss_window():
    s = FaultSchedule(faults(), "pinky2")
    assert [s.comms_down(t) for t in (39.9, 40.0, 59.9, 60.0)] == [False, True, True, False]


def test_nav_abort_consumes_count_after_start_time():
    s = FaultSchedule(faults(), "pinky2")
    assert s.take_nav_abort(74.0) is False
    assert [s.take_nav_abort(80.0) for _ in range(3)] == [True, True, False]


def test_stuck_never_recovers_and_is_per_robot():
    s = FaultSchedule(faults(), "pinky2")
    assert not s.stuck(89.0) and s.stuck(90.0) and s.stuck(1e6)
    assert FaultSchedule(faults(), "pinky3").stuck(1e6) is False


def test_an_obstacle_is_there_for_its_window_and_only_for_its_robot():
    s = FaultSchedule([Fault(at_s=8, robot="pinky1", type="blocked", duration_s=15)], "pinky1")
    assert [s.obstacle(t) for t in (7.9, 8.0, 22.9, 23.0)] == [False, True, True, False]
    assert not s.comms_down(10.0) and not s.stuck(10.0)
    other = FaultSchedule([Fault(at_s=8, robot="pinky1", type="blocked", duration_s=15)], "pinky2")
    assert not other.obstacle(10.0)


def test_operator_remove_is_per_robot_and_lasts():
    s = FaultSchedule([Fault(at_s=60, robot="pinky1", type="operator_remove")], "pinky1")
    assert not s.removed(59.9) and s.removed(60.0) and s.removed(1e6)
    other = FaultSchedule([Fault(at_s=60, robot="pinky1", type="operator_remove")], "pinky2")
    assert not other.removed(1e6)


def test_arm_fail_counts_down_per_arm_after_its_start_time():
    f = ArmFaults([Fault(at_s=5, type="arm_fail", arm="arm1", count=2)])
    assert f.take_arm_fail("arm1", 4.9, 1) is None
    assert f.take_arm_fail("arm2", 6.0, 1) is None
    assert [f.take_arm_fail("arm1", 6.0, 1) for _ in range(3)] == [0, 0, None]


def test_arm_fail_is_not_used_up_by_a_goal_too_short_to_reach_it():
    f = ArmFaults([Fault(at_s=0, type="arm_fail", arm="arm1", after_items=1)])
    assert f.take_arm_fail("arm1", 1.0, 1) is None
    assert f.take_arm_fail("arm1", 1.0, 2) == 1
    assert f.take_arm_fail("arm1", 1.0, 2) is None


def test_swap_partial_counts_down_after_its_start_time():
    f = ArmFaults([Fault(at_s=5, type="swap_partial", count=1)])
    assert f.take_swap_partial(4.0) is False
    assert [f.take_swap_partial(6.0) for _ in range(2)] == [True, False]


class _Handle:
    def __init__(self, cancel_requested):
        self.is_cancel_requested = cancel_requested
        self.calls = []

    def canceled(self):
        self.calls.append("canceled")

    def succeed(self):
        self.calls.append("succeed")

    def abort(self):
        self.calls.append("abort")


def test_goal_that_already_succeeded_is_not_reported_as_canceled():
    handle = _Handle(cancel_requested=True)
    finish_action(handle, success=True)
    assert handle.calls == ["succeed"]


def test_canceled_and_failed_goals_keep_their_action_status():
    handle = _Handle(cancel_requested=True)
    finish_action(handle, success=False)
    assert handle.calls == ["canceled"]
    handle = _Handle(cancel_requested=False)
    finish_action(handle, success=False)
    assert handle.calls == ["abort"]


def test_abort_at_shutdown_survives_a_context_that_is_already_gone():
    class Gone:
        aborted = False

        def abort(self):
            self.aborted = True
            raise RuntimeError("feedback publisher is invalid")

    handle = Gone()
    abort_at_shutdown(handle)
    assert handle.aborted
