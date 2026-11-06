"""FakeArm (arm3) through its real action, and the sim clock."""
import time

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("gg_interfaces.msg")

from conftest import CONTRACT, wait_until, write_scenario  # noqa: E402
from gg_fleet_core.common.contract import load_contract, to_ros_qos  # noqa: E402
from gg_interfaces.action import SwapBasket  # noqa: E402
from gg_interfaces.msg import ArmState  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402

C = load_contract(CONTRACT)


def start_arm(harness, tmp_path, **scenario):
    import rclpy.node
    from gg_sim.fake_arm_node import FakeArmNode
    scn = write_scenario(tmp_path, **scenario)
    harness.add(FakeArmNode(parameter_overrides=harness.params(arm_id="arm3", scenario_file=scn)))
    probe = harness.add(rclpy.node.Node("arm_probe"))
    states = []
    probe.create_subscription(ArmState, C.name("arm_state", arm="arm3"), states.append,
                              to_ros_qos(C.qos("arm_state")))
    client = ActionClient(probe, SwapBasket, C.name("swap_basket", arm="arm3"))
    assert client.wait_for_server(10.0)
    return client, states


def send(client, take_full=True, put_empty=True):
    fb = []
    fut = client.send_goal_async(
        SwapBasket.Goal(order_id="O-1", robot_id="pinky1", station_id="OUTBOUND",
                        take_full=take_full, put_empty=put_empty),
        feedback_callback=fb.append)
    assert wait_until(fut.done, 5.0)
    handle = fut.result()
    assert handle.accepted
    return handle, fb


def result_of(handle):
    res = handle.get_result_async()
    assert wait_until(res.done, 20.0)
    return res.result().result


def swap(client, **kwargs):
    handle, fb = send(client, **kwargs)
    return result_of(handle), fb


def test_full_swap_does_both_steps(harness, tmp_path):
    client, states = start_arm(harness, tmp_path)
    result, fb = swap(client)
    assert result.success is True and result.took_full is True and result.put_empty_done is True
    assert result.reason == ""
    assert [f.feedback.phase for f in fb] == ["TAKE_FULL", "PUT_EMPTY"]
    assert wait_until(lambda: any(s.state == ArmState.STATE_SWAPPING for s in states), 2.0)
    assert states[-1].arm_id == "arm3"


def test_only_put_empty_leaves_take_full_alone(harness, tmp_path):
    client, _ = start_arm(harness, tmp_path)
    result, fb = swap(client, take_full=False, put_empty=True)
    assert result.success is True and result.took_full is False and result.put_empty_done is True
    assert [f.feedback.phase for f in fb] == ["PUT_EMPTY"]


def test_only_take_full_leaves_put_empty_alone(harness, tmp_path):
    client, _ = start_arm(harness, tmp_path)
    result, _ = swap(client, take_full=True, put_empty=False)
    assert result.success is True and result.took_full is True and result.put_empty_done is False


def test_swap_partial_takes_the_full_basket_but_fails_the_empty_one(harness, tmp_path):
    client, states = start_arm(harness, tmp_path, faults=[
        {"at_s": 0, "type": "swap_partial", "count": 1}],
        expect={"orders_shipped_min": 1})
    result, _ = swap(client)
    assert result.success is False and result.took_full is True
    assert result.put_empty_done is False and result.reason
    assert wait_until(lambda: any(s.state == ArmState.STATE_ERROR for s in states), 2.0)
    # The retry only has the open step left, and the fault is used up.
    retry, _ = swap(client, take_full=False, put_empty=True)
    assert retry.success is True and retry.took_full is False and retry.put_empty_done is True


def test_swap_partial_with_count_left_also_fails_the_retry(harness, tmp_path):
    client, _ = start_arm(harness, tmp_path, faults=[
        {"at_s": 0, "type": "swap_partial", "count": 2}],
        expect={"orders_shipped_min": 1})
    swap(client)
    retry, _ = swap(client, take_full=False, put_empty=True)
    assert retry.success is False and retry.took_full is False and retry.put_empty_done is False


def test_swap_partial_is_kept_for_a_goal_that_places_an_empty_basket(harness, tmp_path):
    client, _ = start_arm(harness, tmp_path, faults=[
        {"at_s": 0, "type": "swap_partial", "count": 1}],
        expect={"orders_shipped_min": 1})
    take_only, _ = swap(client, take_full=True, put_empty=False)
    assert take_only.success is True
    put, _ = swap(client, take_full=False, put_empty=True)
    assert put.success is False


def test_fail_rate_fails_the_goal(harness, tmp_path):
    client, _ = start_arm(harness, tmp_path, arm={
        "seconds_per_item": 0.2, "swap_seconds": 0.2, "fail_rate": 1.0})
    result, _ = swap(client)
    assert result.success is False and result.reason
    assert not (result.took_full and result.put_empty_done)


def test_cancel_reports_what_was_done(harness, tmp_path):
    client, _ = start_arm(harness, tmp_path, arm={
        "seconds_per_item": 0.2, "swap_seconds": 4.0, "fail_rate": 0.0})
    handle, _ = send(client)
    cancel = handle.cancel_goal_async()
    assert wait_until(cancel.done, 5.0)
    result = result_of(handle)
    assert result.success is False and result.reason == "CANCELED"
    assert result.took_full is False and result.put_empty_done is False


def test_a_second_goal_is_rejected_while_busy(harness, tmp_path):
    client, _ = start_arm(harness, tmp_path, arm={
        "seconds_per_item": 0.2, "swap_seconds": 1.0, "fail_rate": 0.0})
    first, _ = send(client)
    fut = client.send_goal_async(SwapBasket.Goal(take_full=True, put_empty=True))
    assert wait_until(fut.done, 5.0)
    assert not fut.result().accepted
    assert result_of(first).success is True


@pytest.mark.parametrize("arm", ["arm1", "arm2"])
def test_a_workspace_arm_is_refused(harness, tmp_path, arm):
    from gg_sim.fake_arm_node import FakeArmNode
    scn = write_scenario(tmp_path)
    with pytest.raises(ValueError):
        FakeArmNode(parameter_overrides=harness.params(arm_id=arm, scenario_file=scn))


def test_sim_clock_runs_at_speed_factor(harness):
    import rclpy.node
    from gg_sim.sim_clock_node import SimClockNode
    from rosgraph_msgs.msg import Clock
    harness.add(SimClockNode(parameter_overrides=harness.params(speed_factor=10.0)))
    probe = harness.add(rclpy.node.Node("clock_probe"))
    stamps = []
    probe.create_subscription(Clock, "/clock", lambda m: stamps.append(
        (time.monotonic(), m.clock.sec + m.clock.nanosec * 1e-9)), 10)
    assert wait_until(lambda: stamps, 5.0)
    time.sleep(1.0)
    (w0, s0), (w1, s1) = stamps[0], stamps[-1]
    assert (s1 - s0) / (w1 - w0) == pytest.approx(10.0, rel=0.25)


def test_sim_clock_holds_zero_during_the_start_delay(harness):
    import rclpy.node
    from gg_sim.sim_clock_node import SimClockNode
    from rosgraph_msgs.msg import Clock
    harness.add(SimClockNode(parameter_overrides=harness.params(
        speed_factor=10.0, start_delay_s=1.5)))
    probe = harness.add(rclpy.node.Node("clock_probe"))
    stamps = []
    probe.create_subscription(Clock, "/clock", lambda m: stamps.append(
        (time.monotonic(), m.clock.sec + m.clock.nanosec * 1e-9)), 10)
    assert wait_until(lambda: stamps, 5.0)
    first_wall = stamps[0][0]
    assert stamps[0][1] == 0.0
    assert wait_until(lambda: stamps[-1][1] > 0.0, 5.0)
    assert next(w for w, s in stamps if s > 0.0) - first_wall > 0.5, "it held zero for a while"
