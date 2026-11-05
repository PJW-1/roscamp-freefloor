"""Goal bookkeeping of the wrapper. Plain Python, so it also runs without ROS."""
from fleet_manager.goal_tracker import GoalTracker, ReportOutbox

RETRY_S = 1.0
WARN_AFTER = 3


class _Handle:
    def __init__(self):
        self.cancels = 0

    def cancel_goal_async(self):
        self.cancels += 1


def running(key="k"):
    goals = GoalTracker()
    handle = _Handle()
    goals.sent(key)
    goals.accepted(key, handle)
    return goals, handle


def retry(goals, now):
    return goals.retry_cancels(now, RETRY_S, WARN_AFTER)


def test_cancel_of_an_accepted_goal_goes_to_its_handle():
    goals, handle = running()
    goals.cancel("k", 0.0)
    assert handle.cancels == 1


def test_cancel_before_the_server_accepted_is_applied_on_acceptance():
    goals = GoalTracker()
    handle = _Handle()
    goals.sent("k")
    goals.cancel("k", 0.0)
    assert handle.cancels == 0
    goals.accepted("k", handle)
    assert handle.cancels == 1


def test_cancel_of_an_unknown_or_finished_goal_is_ignored():
    goals = GoalTracker()
    handle = _Handle()
    goals.cancel("never-sent", 0.0)
    goals.sent("k")
    goals.accepted("k", handle)
    goals.finished("k")
    goals.cancel("k", 0.0)
    assert handle.cancels == 0
    goals.sent("never-sent")
    goals.accepted("never-sent", handle)
    assert handle.cancels == 0, "the old cancel must not hit a later goal"
    assert retry(goals, 10.0) == []
    assert handle.cancels == 0


def test_rejected_goal_forgets_its_pending_cancel():
    goals = GoalTracker()
    goals.sent("k")
    goals.cancel("k", 0.0)
    goals.finished("k")
    handle = _Handle()
    goals.sent("k")
    goals.accepted("k", handle)
    assert handle.cancels == 0
    assert retry(goals, 10.0) == []


def test_a_goal_nobody_asked_to_cancel_gets_no_cancel():
    goals, handle = running()
    assert retry(goals, 10.0) == []
    assert handle.cancels == 0


def test_an_unfinished_cancel_goes_out_again_every_retry_period():
    goals, handle = running()
    goals.cancel("k", 10.0)
    retry(goals, 10.9)
    assert handle.cancels == 1, "not due yet"
    retry(goals, 11.0)
    assert handle.cancels == 2
    retry(goals, 11.5)
    assert handle.cancels == 2, "the period runs from the last try"
    retry(goals, 12.0)
    assert handle.cancels == 3


def test_the_warning_comes_once_after_the_last_of_the_tries_went_unanswered():
    goals, handle = running()
    goals.cancel("k", 0.0)
    assert retry(goals, 1.0) == []
    assert retry(goals, 2.0) == [], "the third try has only just gone out"
    assert handle.cancels == 3
    assert retry(goals, 3.0) == ["k"]
    assert handle.cancels == 4, "it keeps trying"
    assert retry(goals, 4.0) == [], "one warning"
    assert retry(goals, 5.0) == []
    assert handle.cancels == 6


def test_the_result_ends_the_retries_and_the_warning():
    goals, handle = running()
    goals.cancel("k", 0.0)
    retry(goals, 1.0)
    goals.finished("k")
    assert retry(goals, 5.0) == []
    assert handle.cancels == 2


def test_asking_again_does_not_start_the_count_over():
    goals, handle = running()
    goals.cancel("k", 0.0)
    goals.cancel("k", 0.9)
    assert handle.cancels == 1
    assert retry(goals, 1.0) == []
    assert handle.cancels == 2, "the period runs from the first ask"
    assert retry(goals, 2.0) == []
    assert retry(goals, 3.0) == ["k"]


def test_each_goal_has_its_own_count():
    goals, first = running("a")
    second = _Handle()
    goals.sent("b")
    goals.accepted("b", second)
    goals.cancel("a", 0.0)
    assert retry(goals, 1.0) == []
    goals.cancel("b", 1.5)
    assert retry(goals, 2.0) == []
    assert retry(goals, 3.0) == ["a"]
    assert (first.cancels, second.cancels) == (4, 2)
    assert retry(goals, 4.0) == []
    assert retry(goals, 5.0) == ["b"]


def test_a_cancel_waiting_for_an_acceptance_that_never_comes_is_warned_about_too():
    """With no handle nothing can be sent, and the goal may still be running on the robot."""
    goals = GoalTracker()
    goals.sent("k")
    goals.cancel("k", 0.0)
    assert retry(goals, 1.0) == []
    assert retry(goals, 2.0) == []
    assert retry(goals, 3.0) == ["k"]
    assert retry(goals, 4.0) == []
    handle = _Handle()
    goals.accepted("k", handle)
    assert handle.cancels == 1
    retry(goals, 5.0)
    assert handle.cancels == 2, "accepted late, and it keeps being asked"


def test_outbox_keeps_reports_until_the_service_is_ready():
    outbox = ReportOutbox()
    outbox.add("a")
    outbox.add("b")
    assert outbox.drain(ready=False) == []
    assert outbox.drain(ready=True) == ["a", "b"]
    assert outbox.drain(ready=True) == []
