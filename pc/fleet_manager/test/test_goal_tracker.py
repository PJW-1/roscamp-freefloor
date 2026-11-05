"""Goal bookkeeping of the wrapper. Plain Python, so it also runs without ROS."""
from fleet_manager.goal_tracker import GoalTracker, ReportOutbox


class _Handle:
    def __init__(self):
        self.cancels = 0

    def cancel_goal_async(self):
        self.cancels += 1


def test_cancel_of_an_accepted_goal_goes_to_its_handle():
    goals = GoalTracker()
    handle = _Handle()
    goals.sent("k")
    goals.accepted("k", handle)
    goals.cancel("k")
    assert handle.cancels == 1


def test_cancel_before_the_server_accepted_is_applied_on_acceptance():
    goals = GoalTracker()
    handle = _Handle()
    goals.sent("k")
    goals.cancel("k")
    assert handle.cancels == 0
    goals.accepted("k", handle)
    assert handle.cancels == 1


def test_cancel_of_an_unknown_or_finished_goal_is_ignored():
    goals = GoalTracker()
    handle = _Handle()
    goals.cancel("never-sent")
    goals.sent("k")
    goals.accepted("k", handle)
    goals.finished("k")
    goals.cancel("k")
    assert handle.cancels == 0
    goals.sent("never-sent")
    goals.accepted("never-sent", handle)
    assert handle.cancels == 0, "the old cancel must not hit a later goal"


def test_rejected_goal_forgets_its_pending_cancel():
    goals = GoalTracker()
    goals.sent("k")
    goals.cancel("k")
    goals.finished("k")
    handle = _Handle()
    goals.sent("k")
    goals.accepted("k", handle)
    assert handle.cancels == 0


def test_outbox_keeps_reports_until_the_service_is_ready():
    outbox = ReportOutbox()
    outbox.add("a")
    outbox.add("b")
    assert outbox.drain(ready=False) == []
    assert outbox.drain(ready=True) == ["a", "b"]
    assert outbox.drain(ready=True) == []
