"""tools/model_check.py를 테스트용 작은 지도에서 돌린다. 큰 수치는 worklog에 있다."""
import sys
from dataclasses import replace
from pathlib import Path

import pytest

pytest.importorskip("numpy")

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))

import model_check  # noqa: E402
from fleet_core.common.params import load_params  # noqa: E402
from fleet_core.traffic.lane_map import load_lane_map  # noqa: E402

# 도구 시험은 상태가 적은 point 모드로 돈다. path 모드는 아래 시험 하나와 worklog의 5건.
POINT = replace(load_params(), lane_goal_mode="point")

# 가는 길과 오는 길이 양방향 레인 하나(A~G)를 공유한다.
SMALL_MAP = """
half_width_m: 0.05
lane_entry_m: 0.25
origin_m: [0.0, 0.0]
bundle_two_way: true
waypoints:
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
    return model_check.check(load_lane_map(small_map, bundle_two_way=True), robots=2, params=POINT)


@pytest.fixture(scope="module")
def unbundled(small_map):
    return model_check.check(load_lane_map(small_map, bundle_two_way=False), robots=2,
                             params=POINT)


def test_robots_meet_head_on_on_a_two_way_lane_with_no_shared_lock(unbundled):
    assert unbundled.complete
    assert unbundled.deadlocks > 0
    assert unbundled.starved > 0


def test_one_lock_for_both_directions_ends_that(bundled):
    assert bundled.complete
    assert (bundled.deadlocks, bundled.starved) == (0, 0)
    assert bundled.example == []


def test_the_example_it_prints_really_ends_in_a_wait_cycle(small_map, unbundled):
    model = model_check.Model(load_lane_map(small_map, bundle_two_way=False), 2, params=POINT)
    model.restore(model.initial)
    assert not model.has_cycle()
    for step in unbundled.example:
        kind, _, robot = step.rpartition(" ")
        event = (kind, robot) if robot in model.names else (step,)
        assert event in model.events(model.capture())
        model.apply(event)
    assert model.has_cycle()


def test_calls_that_change_nothing_can_be_left_out(small_map, bundled):
    """events()는 두 번째 dispatch와 놓을 것 없는 release를 뺀다."""

    class EveryCall(model_check.Model):
        def events(self, state):
            found = super().events(state)
            for name in self.names:
                for event in (("send", name), ("release", name)):
                    if event not in found:
                        found.append(event)
            return found

    site = load_lane_map(small_map, bundle_two_way=True)
    every = model_check.check(site, robots=2, params=POINT, model_class=EveryCall)
    assert (every.states, every.deadlocks, every.starved) == (
        bundled.states, bundled.deadlocks, bundled.starved)


def test_no_step_of_the_search_fails_a_goal_so_a_state_needs_no_retry_time(small_map, bundled):
    """재시도 시간은 탐색 상태가 되고, 시간이 멈추면 끝나지 않는다."""

    class Watched(model_check.Model):
        def apply(self, event):
            super().apply(event)
            for mission in self.core._missions.values():
                state = (mission.attempts, mission.retry_at, mission.arrival_by)
                assert state == (0, 0.0, None), event

    site = load_lane_map(small_map, bundle_two_way=True)
    watched = model_check.check(site, robots=2, params=POINT, model_class=Watched)
    assert watched.states == bundled.states


def test_a_robot_queues_for_one_zone_at_a_time(small_map, unbundled):
    """/fleet/robots의 wait_zone이 구역 하나만 말할 수 있는 근거다."""

    class OneQueue(model_check.Model):
        def capture(self):
            for name in self.names:
                queued = [zone for zone, who in self.core.zones._arrived if who == name]
                assert len(queued) <= 1, f"{name} queues for {queued}"
            return super().capture()

    site = load_lane_map(small_map, bundle_two_way=False)
    queued = model_check.check(site, robots=2, params=POINT, model_class=OneQueue)
    assert queued.states == unbundled.states


def test_several_processes_give_the_same_answer(small_map, unbundled):
    site = load_lane_map(small_map, bundle_two_way=False)
    parallel = model_check.check(site, robots=2, params=POINT, workers=2)
    assert (parallel.states, parallel.deadlocks, parallel.starved, parallel.example) == (
        unbundled.states, unbundled.deadlocks, unbundled.starved, unbundled.example)


def test_with_three_reports_a_zone_is_freed_on_the_third(small_map):
    model = model_check.Model(load_lane_map(small_map), 1, start=["W"], confirm=3)
    model.restore(model.initial)
    model.apply(("send", "r1"))
    model.apply(("pass", "r1"))
    assert "W" in model.core.zones.held_by("r1")
    model.apply(("report", "r1"))
    assert "W" in model.core.zones.held_by("r1")
    model.apply(("report", "r1"))
    assert "W" not in model.core.zones.held_by("r1")


def test_with_three_reports_the_same_way_still_ends_in_a_wait_cycle(small_map, unbundled):
    """전체 탐색에 몇 분 걸려 그 비교는 worklog에 있다."""
    model = model_check.Model(load_lane_map(small_map, bundle_two_way=False), 2, params=POINT,
                              confirm=3)
    model.restore(model.initial)
    for step in unbundled.example:
        kind, _, robot = step.rpartition(" ")
        model.apply((kind, robot) if robot in model.names else (step,))
        if kind in ("pass", "arrive"):
            model.apply(("report", robot))
            model.apply(("report", robot))
    assert model.has_cycle()


def test_a_start_two_robots_cannot_share_is_refused(small_map):
    with pytest.raises(ValueError, match="r2"):
        model_check.Model(load_lane_map(small_map), 2, start=["W", "W"])


def test_a_state_limit_is_reported_as_an_incomplete_search(small_map):
    site = load_lane_map(small_map, bundle_two_way=False)
    cut = model_check.check(site, robots=2, max_states=50)
    assert not cut.complete
    assert cut.starved == -1


@pytest.mark.parametrize("path", [None, REPO / "sim/fake_robots/maps/oneway_loop.yaml",
                                  REPO / "sim/fake_robots/maps/oneway_shortcut.yaml"])
def test_one_robot_alone_finishes_every_shipped_map(path):
    result = model_check.check(load_lane_map(path), robots=1)
    assert result.complete
    assert (result.deadlocks, result.starved) == (0, 0)


def test_path_goals_with_one_lock_for_both_directions_have_no_deadlock_and_no_starvation(
        small_map):
    """path 모드(지시서 7번): 유턴 지점에서 새 목표를 못 받아 굶던 결함을 잡은 시험이다."""
    path = replace(load_params(), lane_goal_mode="path")
    result = model_check.check(load_lane_map(small_map, bundle_two_way=True), robots=2,
                               params=path)
    assert result.complete
    assert (result.deadlocks, result.starved) == (0, 0)
