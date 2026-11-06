"""이름, 타입, QoS의 기준은 계약 파일 하나다."""
from pathlib import Path

import pytest
from fleet_core.common.contract import ContractError, load_contract

CONTRACT = Path(__file__).parents[3] / "interfaces" / "fleet_interfaces" / "contract.yaml"


@pytest.fixture(scope="module")
def contract():
    return load_contract(CONTRACT)


def test_topic_name_is_built_from_key_and_placeholder(contract):
    assert contract.name("robot_state", pinky="pinky2") == "/pinky2/state"
    assert contract.name("order_board") == "/central/orders"
    assert contract.name("swap_basket", arm="arm3") == "/arm3/swap_basket"


def test_qos_lookup_returns_the_profile_of_the_entry(contract):
    qos = contract.qos("robot_state")
    assert (qos.reliability, qos.durability, qos.depth) == ("best_effort", "volatile", 5)
    assert contract.qos("zones").durability == "transient_local"


def test_unknown_key_is_an_error_not_a_guess(contract):
    with pytest.raises(ContractError):
        contract.name("no_such_key")


def test_missing_placeholder_is_an_error(contract):
    with pytest.raises(ContractError):
        contract.name("robot_state")


def test_expand_lists_every_instance_with_type_and_kind(contract):
    entries = {e.name: e for e in contract.expanded()}
    assert entries["/pinky3/move_to"].kind == "action"
    assert entries["/pinky3/move_to"].type == "fleet_interfaces/action/MoveTo"
    assert entries["/arm3/state"].qos.reliability == "best_effort"
    assert entries["/workcell/fill_basket"].type == "fleet_interfaces/action/FillBasket"
    assert "/arm1/vision/items" in entries and "/arm2/vision/items" in entries
    assert "/arm3/vision/items" not in entries, "instances must limit the expansion"
    assert "/arm3/swap_basket" in entries
    assert "/arm1/swap_basket" not in entries


def test_expand_skips_real_only_entries_when_asked(contract):
    with_real = {e.name for e in contract.expanded(include_real_only=True)}
    without = {e.name for e in contract.expanded(include_real_only=False)}
    assert "/pinky1/navigate_to_pose" in with_real
    assert "/pinky1/navigate_to_pose" not in without


def test_entry_with_undefined_qos_profile_is_rejected(tmp_path):
    bad = tmp_path / "contract.yaml"
    bad.write_text(
        'placeholders: {}\nqos_profiles: {}\n'
        'topics:\n  - {key: a, name: /a, type: x/msg/A, qos: NOPE}\n',
        encoding="utf-8",
    )
    with pytest.raises(ContractError):
        load_contract(bad)
