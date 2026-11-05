"""tools/model_check.py on a map small enough for a test. The big numbers are in the worklog."""
import sys
from pathlib import Path

import pytest

pytest.importorskip("numpy")

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))

import model_check  # noqa: E402
from fleet_core.site_map import load_site_map  # noqa: E402

# The trip out and the trip back share one two-way lane, A to G.
SMALL_MAP = """
half_width_m: 0.05
lane_entry_m: 0.25
origin_m: [0.0, 0.0]
bundle_two_way: true
nodes:
  - {id: W, x: 1.0, y: 1.0}
  - {id: A, x: 2.0, y: 1.0}
  - {id: G, x: 3.0, y: 1.0}
  - {id: O, x: 4.0, y: 1.0}
  - {id: E, x: 3.5, y: 2.0}
  - {id: F, x: 1.5, y: 0.0}
  - {id: P-1, x: 0.5, y: 0.0}
  - {id: P-2, x: 0.0, y: 1.0}
two_way:
  SHORT: [A, G]
lanes:
  - {from: W, to: A}
  - {from: G, to: O}
  - {from: O, to: E}
  - {from: E, to: G}
  - {from: A, to: F}
  - {from: F, to: P-1}
  - {from: P-1, to: P-2}
  - {from: P-2, to: W}
stations:
  WORK: {dock: W, queue: [[P-2, W]], commit: W, server: workcell, arms: [arm1, arm2], action: fill}
  OUTBOUND: {dock: O, queue: [[G, O]], commit: O, server: arm3, arms: [arm3], action: swap}
parking: [P-1, P-2]
"""


@pytest.fixture(scope="module")
def small_map(tmp_path_factory) -> Path:
    path = tmp_path_factory.mktemp("maps") / "small.yaml"
    path.write_text(SMALL_MAP, encoding="utf-8")
    return path


@pytest.fixture(scope="module")
def bundled(small_map):
    return model_check.check(load_site_map(small_map, bundle_two_way=True), robots=2)


@pytest.fixture(scope="module")
def unbundled(small_map):
    return model_check.check(load_site_map(small_map, bundle_two_way=False), robots=2)


def test_robots_meet_head_on_on_a_two_way_lane_with_no_shared_lock(unbundled):
    assert unbundled.complete
    assert unbundled.deadlocks > 0
    assert unbundled.starved > 0


def test_one_lock_for_both_directions_ends_that(bundled):
    assert bundled.complete
    assert (bundled.deadlocks, bundled.starved) == (0, 0)
    assert bundled.example == []


def test_the_example_it_prints_really_ends_in_a_wait_cycle(small_map, unbundled):
    model = model_check.Model(load_site_map(small_map, bundle_two_way=False), 2)
    model.restore(model.initial)
    assert not model.has_cycle()
    for step in unbundled.example:
        kind, _, robot = step.rpartition(" ")
        event = (kind, robot) if robot in model.names else (step,)
        assert event in model.events(model.capture())
        model.apply(event)
    assert model.has_cycle()


def test_calls_that_change_nothing_can_be_left_out(small_map, bundled):
    """events() leaves out a second dispatch and a release with nothing to release."""

    class EveryCall(model_check.Model):
        def events(self, state):
            found = super().events(state)
            for name in self.names:
                for event in (("send", name), ("release", name)):
                    if event not in found:
                        found.append(event)
            return found

    site = load_site_map(small_map, bundle_two_way=True)
    every = model_check.check(site, robots=2, model_class=EveryCall)
    assert (every.states, every.deadlocks, every.starved) == (
        bundled.states, bundled.deadlocks, bundled.starved)


def test_several_processes_give_the_same_answer(small_map, unbundled):
    site = load_site_map(small_map, bundle_two_way=False)
    parallel = model_check.check(site, robots=2, workers=2)
    assert (parallel.states, parallel.deadlocks, parallel.starved, parallel.example) == (
        unbundled.states, unbundled.deadlocks, unbundled.starved, unbundled.example)


def test_with_three_reports_a_zone_is_freed_on_the_third(small_map):
    model = model_check.Model(load_site_map(small_map), 1, start=["W"], confirm=3)
    model.restore(model.initial)
    model.apply(("send", "r1"))
    model.apply(("pass", "r1"))
    assert "W" in model.core.zones.held_by("r1")
    model.apply(("report", "r1"))
    assert "W" in model.core.zones.held_by("r1")
    model.apply(("report", "r1"))
    assert "W" not in model.core.zones.held_by("r1")


def test_with_three_reports_the_same_way_still_ends_in_a_wait_cycle(small_map, unbundled):
    """Searching all of it takes minutes, so that comparison is in the worklog."""
    model = model_check.Model(load_site_map(small_map, bundle_two_way=False), 2, confirm=3)
    model.restore(model.initial)
    for step in unbundled.example:
        kind, _, robot = step.rpartition(" ")
        model.apply((kind, robot) if robot in model.names else (step,))
        if kind in ("pass", "arrive"):
            model.apply(("report", robot))
            model.apply(("report", robot))
    assert model.has_cycle()


def test_a_state_limit_is_reported_as_an_incomplete_search(small_map):
    site = load_site_map(small_map, bundle_two_way=False)
    cut = model_check.check(site, robots=2, max_states=50)
    assert not cut.complete
    assert cut.starved == -1


@pytest.mark.parametrize("path", [None, REPO / "sim/maps/oneway_loop.yaml",
                                  REPO / "sim/maps/oneway_shortcut.yaml"])
def test_one_robot_alone_finishes_every_shipped_map(path):
    result = model_check.check(load_site_map(path), robots=1)
    assert result.complete
    assert (result.deadlocks, result.starved) == (0, 0)
