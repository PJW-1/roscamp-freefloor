"""Compare a contract with what a running system exposes. Plain data in, problems out."""
from pathlib import Path

import pytest
from gg_fleet_core.common.contract import QosSpec, load_contract
from gg_fleet_core.common.contract_check import compare

CONTRACT = Path(__file__).parents[3] / "interfaces" / "gg_interfaces" / "contract.yaml"
STATE = QosSpec("best_effort", "volatile", 5)


@pytest.fixture(scope="module")
def contract():
    return load_contract(CONTRACT)


def healthy(contract):
    """An observed graph that matches the contract exactly (real-only entries left out)."""
    observed = {"topics": {}, "actions": {}, "services": {}}
    for e in contract.expanded(include_real_only=False):
        if e.kind == "topic":
            observed["topics"][e.name] = {
                "types": [e.type], "publishers": [e.qos], "subscribers": [e.qos]}
        elif e.kind == "action":
            observed["actions"][e.name] = [e.type]
        else:
            observed["services"][e.name] = [e.type]
    return observed


def kinds(problems):
    return sorted((p.kind, p.name) for p in problems)


def test_matching_system_has_no_problems(contract):
    assert compare(contract, healthy(contract)) == []


def test_missing_entries_are_reported_per_kind(contract):
    obs = healthy(contract)
    del obs["topics"]["/pinky1/state"]
    del obs["actions"]["/pinky2/dock"]
    del obs["services"]["/central/set_run_mode"]
    assert kinds(compare(contract, obs)) == [
        ("missing", "/central/set_run_mode"),
        ("missing", "/pinky1/state"),
        ("missing", "/pinky2/dock"),
    ]


def test_wrong_type_is_reported(contract):
    obs = healthy(contract)
    obs["topics"]["/central/orders"]["types"] = ["std_msgs/msg/String"]
    assert kinds(compare(contract, obs)) == [("wrong_type", "/central/orders")]


def test_best_effort_publisher_with_reliable_subscriber_is_an_error(contract):
    obs = healthy(contract)
    obs["topics"]["/fleet/zones"]["publishers"] = [STATE]
    problems = compare(contract, obs)
    assert ("qos", "/fleet/zones") in kinds(problems)
    assert all(p.severity == "error" for p in problems)


def test_best_effort_publisher_is_reported_even_when_contract_subscriber_absent(contract):
    """The contract says reliable, so a best_effort publisher can never reach a contract-conform
    subscriber, whatever subscribers are currently attached."""
    obs = healthy(contract)
    obs["topics"]["/fleet/zones"]["publishers"] = [STATE]
    obs["topics"]["/fleet/zones"]["subscribers"] = []
    assert ("qos", "/fleet/zones") in kinds(compare(contract, obs))


def test_durability_mismatch_on_publisher(contract):
    obs = healthy(contract)
    obs["topics"]["/central/events"]["publishers"] = [QosSpec("reliable", "transient_local", 50)]
    assert ("qos", "/central/events") in kinds(compare(contract, obs))


def test_incompatible_subscriber_is_reported(contract):
    """A reliable subscriber of a best_effort topic gets nothing."""
    obs = healthy(contract)
    obs["topics"]["/pinky1/state"]["subscribers"] = [QosSpec("reliable", "volatile", 5)]
    assert ("qos", "/pinky1/state") in kinds(compare(contract, obs))


def test_depth_mismatch_is_warning_on_subscribers_error_on_publishers(contract):
    obs = healthy(contract)
    obs["topics"]["/pinky1/state"]["subscribers"] = [QosSpec("best_effort", "volatile", 1)]
    assert [(p.kind, p.severity) for p in compare(contract, obs)] == [("qos", "warning")]
    obs = healthy(contract)
    obs["topics"]["/pinky1/state"]["publishers"] = [QosSpec("best_effort", "volatile", 1)]
    assert [p.severity for p in compare(contract, obs)] == ["error"]


def test_topic_without_publishers_is_not_a_qos_problem(contract):
    obs = healthy(contract)
    obs["topics"]["/pinky1/state"]["publishers"] = []
    assert compare(contract, obs) == []


def test_real_only_entries_are_skipped_unless_asked(contract):
    obs = healthy(contract)
    assert compare(contract, obs) == []
    names = {p.name for p in compare(contract, obs, include_real_only=True)}
    assert names == {f"/pinky{i}/navigate_to_pose" for i in (1, 2, 3)}
