"""FakeWorkcell and FakeVision through their real action and topics."""
import threading
import time
from collections import Counter

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces.msg")

from conftest import CONTRACT, wait_until, write_scenario  # noqa: E402
from fleet_core.contract import load_contract, to_ros_qos  # noqa: E402
from fleet_interfaces.action import FillBasket  # noqa: E402
from fleet_interfaces.msg import ArmState, ItemArray, OrderItem  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402

C = load_contract(CONTRACT)
ARMS = ("arm1", "arm2")


class Cell:
    def __init__(self, harness, tmp_path, **scenario):
        import rclpy.node
        from fake_robots.fake_vision_node import FakeVisionNode
        from fake_robots.fake_workcell_node import FakeWorkcellNode
        scn = write_scenario(tmp_path, **scenario)
        self.node = harness.add(FakeWorkcellNode(
            parameter_overrides=harness.params(scenario_file=scn)))
        for arm in ARMS:
            harness.add(FakeVisionNode(parameter_overrides=harness.params(
                arm_id=arm, scenario_file=scn)))
        probe = harness.add(rclpy.node.Node("cell_probe"))
        self.seen = {arm: [] for arm in ARMS}
        self.log = []  # one list across both arms, in order of arrival
        self.lock = threading.Lock()
        self.items = {arm: [] for arm in ARMS}
        for arm in ARMS:
            probe.create_subscription(ArmState, C.name("arm_state", arm=arm), self._on_state,
                                      to_ros_qos(C.qos("arm_state")))
            probe.create_subscription(ItemArray, C.name("vision_items", arm=arm),
                                      self.items[arm].append,
                                      to_ros_qos(C.qos("vision_items")))
        self.client = ActionClient(probe, FillBasket, C.name("fill_basket"))
        assert self.client.wait_for_server(10.0)

    def _on_state(self, msg):
        with self.lock:
            self.seen[msg.arm_id].append(msg)
            self.log.append(msg)

    def send(self, **items):
        fb = []
        goal = FillBasket.Goal(order_id="O-1", robot_id="pinky1", station_id="WORK",
                               items=[OrderItem(kind=k, count=c) for k, c in items.items()])
        fut = self.client.send_goal_async(goal, feedback_callback=fb.append)
        assert wait_until(fut.done, 5.0)
        return fut.result(), fb

    def fill(self, **items):
        handle, fb = self.send(**items)
        assert handle.accepted
        return result_of(handle), fb

    def visible(self, arm):
        return Counter(self.items[arm][-1].kinds) if self.items[arm] else None


def result_of(handle):
    res = handle.get_result_async()
    assert wait_until(res.done, 30.0)
    return res.result().result


def counts(items):
    return {i.kind: i.count for i in items}


def test_items_from_both_arms_are_placed(harness, tmp_path):
    cell = Cell(harness, tmp_path)
    result, fb = cell.fill(A=2, C=1)
    assert result.success is True and counts(result.placed) == {"A": 2, "C": 1}
    assert result.missing == [] and result.reason == ""
    assert fb and fb[-1].feedback.placed_total == 3
    assert {f.feedback.arm_id for f in fb} == {"arm1", "arm2"}
    assert wait_until(lambda: any(s.state == ArmState.STATE_PLACING for s in cell.seen["arm1"]))
    assert wait_until(lambda: any(s.state == ArmState.STATE_PLACING for s in cell.seen["arm2"]))


def test_a_kind_not_in_stock_ends_up_missing(harness, tmp_path):
    cell = Cell(harness, tmp_path)
    result, _ = cell.fill(A=1, Z=2)
    assert result.success is True  # finished trying
    assert counts(result.placed) == {"A": 1} and counts(result.missing) == {"Z": 2}


def test_more_than_the_stock_places_what_there_is(harness, tmp_path):
    cell = Cell(harness, tmp_path)
    result, _ = cell.fill(B=3)  # arm1 has one B
    assert result.success is True
    assert counts(result.placed) == {"B": 1} and counts(result.missing) == {"B": 2}


def test_nothing_in_stock_places_nothing(harness, tmp_path):
    cell = Cell(harness, tmp_path)
    result, _ = cell.fill(Z=1)
    assert result.success is True and result.placed == [] and counts(result.missing) == {"Z": 1}


def test_a_kind_on_both_shelves_uses_both_arms(harness, tmp_path):
    cell = Cell(harness, tmp_path, stock={"arm1": {"A": 1}, "arm2": {"A": 1}})
    result, fb = cell.fill(A=2)
    assert result.success is True and counts(result.placed) == {"A": 2}
    assert {f.feedback.arm_id for f in fb} == {"arm1", "arm2"}
    assert wait_until(lambda: cell.visible("arm1") == {} and cell.visible("arm2") == {}, 5.0)


def test_stock_is_used_up_across_goals(harness, tmp_path):
    cell = Cell(harness, tmp_path)
    assert counts(cell.fill(A=3)[0].placed) == {"A": 3}
    second, _ = cell.fill(A=1)
    assert second.placed == [] and counts(second.missing) == {"A": 1}


def test_arms_never_place_at_once_but_do_pick_at_once(harness, tmp_path):
    cell = Cell(harness, tmp_path, stock={"arm1": {"A": 6}, "arm2": {"C": 6}},
                arm={"seconds_per_item": 0.3, "swap_seconds": 0.4, "fail_rate": 0.0})
    result, _ = cell.fill(A=4, C=4)
    assert counts(result.placed) == {"A": 4, "C": 4}
    assert wait_until(lambda: len(cell.log) > 8, 2.0)
    stamp = {}
    spans = {arm: [] for arm in ARMS}
    picking_overlap = False
    state_now = dict.fromkeys(ARMS, "")
    for msg in sorted(cell.log, key=lambda m: (m.stamp.sec, m.stamp.nanosec)):
        t = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        if msg.state == ArmState.STATE_PLACING and msg.arm_id not in stamp:
            stamp[msg.arm_id] = t
        elif msg.state != ArmState.STATE_PLACING and msg.arm_id in stamp:
            spans[msg.arm_id].append((stamp.pop(msg.arm_id), t))
        state_now[msg.arm_id] = msg.state
        picking_overlap |= all(s == ArmState.STATE_PICKING for s in state_now.values())
    assert len(spans["arm1"]) == 4 and len(spans["arm2"]) == 4
    for a_start, a_end in spans["arm1"]:
        for b_start, b_end in spans["arm2"]:
            assert a_end <= b_start or b_end <= a_start, "both arms were PLACING at once"
    assert picking_overlap, "the arms never picked in parallel"


def test_the_lock_is_what_keeps_them_apart(harness, tmp_path):
    """With the lock taken by somebody else nobody may reach PLACING."""
    cell = Cell(harness, tmp_path)
    assert cell.node._basket_space.acquire(timeout=1.0)
    handle, _ = cell.send(A=1, C=1)
    assert handle.accepted
    done = handle.get_result_async()
    assert not wait_until(done.done, 1.0)
    assert not any(s.state == ArmState.STATE_PLACING
                   for arm in ARMS for s in cell.seen[arm])
    cell.node._basket_space.release()
    assert wait_until(done.done, 10.0)
    assert counts(done.result().result.placed) == {"A": 1, "C": 1}


def test_arm_fail_returns_failure_with_nothing_placed(harness, tmp_path):
    cell = Cell(harness, tmp_path, faults=[
        {"at_s": 0, "arm": "arm1", "type": "arm_fail", "count": 1}],
        expect={"orders_shipped_min": 1})
    result, _ = cell.fill(A=2)
    assert result.success is False and result.placed == [] and result.reason
    assert wait_until(lambda: any(s.state == ArmState.STATE_ERROR for s in cell.seen["arm1"]), 2.0)


def test_arm_fail_returns_what_was_placed_so_far(harness, tmp_path):
    cell = Cell(harness, tmp_path, faults=[
        {"at_s": 0, "arm": "arm1", "type": "arm_fail", "count": 1, "after_items": 1}],
        expect={"orders_shipped_min": 1})
    result, _ = cell.fill(A=2)
    assert result.success is False and counts(result.placed) == {"A": 1}
    assert result.missing == []
    assert wait_until(lambda: cell.visible("arm1") == {"A": 2, "B": 1}, 5.0)


def test_arm_fail_runs_out_after_its_count(harness, tmp_path):
    cell = Cell(harness, tmp_path, faults=[
        {"at_s": 0, "arm": "arm1", "type": "arm_fail", "count": 3}],
        expect={"orders_shipped_min": 1})
    assert [cell.fill(A=1)[0].success for _ in range(4)] == [False, False, False, True]


def test_arm_fail_on_the_other_arm_is_not_used_up_by_this_order(harness, tmp_path):
    cell = Cell(harness, tmp_path, faults=[
        {"at_s": 0, "arm": "arm2", "type": "arm_fail", "count": 1}],
        expect={"orders_shipped_min": 1})
    assert cell.fill(A=1)[0].success is True
    assert cell.fill(C=1)[0].success is False


def test_fail_rate_returns_failure(harness, tmp_path):
    cell = Cell(harness, tmp_path,
                arm={"seconds_per_item": 0.2, "swap_seconds": 0.4, "fail_rate": 1.0})
    result, _ = cell.fill(A=2)
    assert result.success is False and result.reason
    assert sum(i.count for i in result.placed) < 2


def test_cancel_returns_canceled_with_what_was_placed(harness, tmp_path):
    cell = Cell(harness, tmp_path, stock={"arm1": {"A": 6}},
                arm={"seconds_per_item": 0.4, "swap_seconds": 0.4, "fail_rate": 0.0})
    handle, fb = cell.send(A=5)
    assert handle.accepted
    assert wait_until(lambda: fb, 10.0)  # at least one is in the basket
    cancel = handle.cancel_goal_async()
    assert wait_until(cancel.done, 5.0)
    result = result_of(handle)
    assert result.success is False and result.reason == "CANCELED"
    assert 1 <= counts(result.placed)["A"] < 5 and result.missing == []
    # Nothing is lost: whatever did not reach the basket is still on the shelf.
    assert wait_until(lambda: cell.visible("arm1") == {"A": 6 - counts(result.placed)["A"]}, 5.0)


def test_slow_cancel_keeps_the_server_busy_until_its_result(harness, tmp_path):
    cell = Cell(harness, tmp_path, stock={"arm1": {"A": 6}},
                arm={"seconds_per_item": 0.2, "swap_seconds": 0.4, "cancel_seconds": 1.5})
    handle, fb = cell.send(A=5)
    assert handle.accepted
    assert wait_until(lambda: fb, 10.0)
    canceled_at = time.monotonic()
    assert wait_until(handle.cancel_goal_async().done, 5.0)
    time.sleep(0.6)  # the arm threads have stopped by now, and the cancel still runs
    second, _ = cell.send(A=1)
    assert not second.accepted, "busy while the arm stops"
    result = result_of(handle)
    assert time.monotonic() - canceled_at >= 1.4
    assert result.success is False and result.reason == "CANCELED"
    third, _ = cell.send(A=1)
    assert third.accepted, "free again once the result is out"


def test_a_second_goal_is_rejected_while_busy(harness, tmp_path):
    cell = Cell(harness, tmp_path, arm={"seconds_per_item": 0.6, "swap_seconds": 0.4})
    first, _ = cell.send(A=1)
    assert first.accepted
    second, _ = cell.send(A=1)
    assert not second.accepted
    assert result_of(first).success is True


def test_vision_shows_the_true_stock_and_follows_it(harness, tmp_path):
    cell = Cell(harness, tmp_path)
    assert wait_until(lambda: cell.visible("arm1") == {"A": 3, "B": 1}, 5.0)
    assert wait_until(lambda: cell.visible("arm2") == {"C": 3}, 5.0)
    msg = cell.items["arm2"][-1]
    assert msg.arm_id == "arm2" and len(msg.poses) == len(msg.kinds)
    cell.fill(A=1, C=2)
    assert wait_until(lambda: cell.visible("arm1") == {"A": 2, "B": 1}, 5.0)
    assert wait_until(lambda: cell.visible("arm2") == {"C": 1}, 5.0)


@pytest.mark.parametrize("arm", ["arm3"])
def test_vision_refuses_an_arm_without_a_camera_on_stock(harness, tmp_path, arm):
    from fake_robots.fake_vision_node import FakeVisionNode
    with pytest.raises(ValueError):
        FakeVisionNode(parameter_overrides=harness.params(
            arm_id=arm, scenario_file=write_scenario(tmp_path)))
