"""Free-floor final-goal flag (version 2 section 4 horizon minimum, contract 0.8-draft proposal).

Free-floor sends only the path up to the end of its rolling cell lock. Each of those partial
MoveTos must say final_goal False; only the MoveTo whose end is the route's last point says True.
"""
from dataclasses import replace

import pytest
from fleet_core.common.commands import MoveCmd
from fleet_core.common.params import load_params
from test_floor_single import run_one


@pytest.fixture(scope="module")
def run():
    return run_one()


def test_partial_moves_are_not_final_and_the_last_one_is(run):
    world, _, finished = run
    assert finished
    moves = world.commands_of(MoveCmd)
    partial = [m for m in moves if "~" in m.target_id]
    final = [m for m in moves if "~" not in m.target_id]
    assert partial and final
    assert all(m.final_goal is False for m in partial)
    assert all(m.final_goal is True for m in final)


def test_each_leg_ends_with_a_final_move(run):
    world, _, _ = run
    moves = world.commands_of(MoveCmd)
    # the first move of a leg is partial (the lock reaches 0.5 m), the leg's last one is final
    assert moves[0].final_goal is False
    assert moves[-1].final_goal is True


def test_a_record_without_the_flag_replays_with_every_floor_move_final():
    world, _, _ = run_one(seconds=60.0, params=replace(load_params(), mark_final_goal=False))
    moves = world.commands_of(MoveCmd)
    assert any("~" in m.target_id for m in moves)
    assert all(m.final_goal for m in moves)
