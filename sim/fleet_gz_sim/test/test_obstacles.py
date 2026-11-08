"""시뮬 장애물 일정: 읽기와 검사, 레인 위 점, 반복, 놓기·치우기 시각, gz 요청. ROS 없이 돈다."""
import math
from pathlib import Path

import pytest

lane_map_module = pytest.importorskip("fleet_core.traffic.lane_map")
from fleet_gz_sim import obstacles  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
LANES = lane_map_module.load_lane_map()


def parse(**top):
    data = {"obstacles": [{"at_s": 10, "lane": ["W", "A"], "duration_s": 20}]}
    data.update(top)
    return obstacles.parse_schedule(data, LANES)


def entry(**fields):
    one = {"at_s": 10, "lane": ["W", "A"], "duration_s": 20}
    one.update(fields)
    return one


# ---- 읽기와 검사 ------------------------------------------------------------------------------

def test_parse_fills_defaults_and_sorts_by_time():
    schedule = obstacles.parse_schedule({"obstacles": [
        entry(at_s=50, lane=["B", "R"]), entry(at_s=10, fraction=0.25)]}, LANES)
    assert schedule.repeat_every_s is None
    assert schedule.size_m == obstacles.DEFAULT_SIZE_M
    first, second = schedule.obstacles
    assert (first.at_s, first.lane, first.fraction) == (10.0, ("W", "A"), 0.25)
    assert (second.lane, second.fraction, second.duration_s) == (("B", "R"), 0.5, 20.0)


def test_parse_reads_repeat_and_size():
    schedule = parse(repeat_every_s=300, size_m=[0.1, 0.2, 0.3])
    assert schedule.repeat_every_s == 300.0
    assert schedule.size_m == (0.1, 0.2, 0.3)


def test_lane_may_be_named_against_its_direction():
    schedule = obstacles.parse_schedule({"obstacles": [entry(lane=["A", "W"])]}, LANES)
    assert schedule.obstacles[0].lane == ("A", "W")


@pytest.mark.parametrize("data, words", [
    ({"obstacles": [entry()], "speed": 1}, "speed"),
    ({"obstacles": [entry(colour="red")]}, "colour"),
    ({}, "obstacles"),
    ({"obstacles": []}, "obstacles"),
    ({"obstacles": [{"lane": ["W", "A"], "duration_s": 20}]}, "at_s"),
    ({"obstacles": [{"at_s": 1, "duration_s": 20}]}, "lane"),
    ({"obstacles": [{"at_s": 1, "lane": ["W", "A"]}]}, "duration_s"),
    ({"obstacles": [entry(lane=["W", "B"])]}, "W->B"),
    ({"obstacles": [entry(lane=["W", "nowhere"])]}, "nowhere"),
    ({"obstacles": [entry(lane=["W"])]}, "lane"),
    ({"obstacles": [entry(fraction=0)]}, "fraction"),
    ({"obstacles": [entry(fraction=1.0)]}, "fraction"),
    ({"obstacles": [entry(fraction=1.5)]}, "fraction"),
    ({"obstacles": [entry(duration_s=0)]}, "duration_s"),
    ({"obstacles": [entry(duration_s=-5)]}, "duration_s"),
    ({"obstacles": [entry(at_s=-1)]}, "at_s"),
    ({"obstacles": [entry(at_s="soon")]}, "at_s"),
    ({"obstacles": [entry(at_s=True)]}, "at_s"),
    ({"obstacles": [entry()], "repeat_every_s": 0}, "repeat_every_s"),
    ({"obstacles": [entry(at_s=290, duration_s=20)], "repeat_every_s": 300}, "repeat_every_s"),
    ({"obstacles": [entry()], "size_m": [0.1, 0.1]}, "size_m"),
    ({"obstacles": [entry()], "size_m": [0.1, 0.1, 0]}, "size_m"),
    ({"obstacles": [entry(at_s=10, duration_s=20), entry(at_s=29, duration_s=5)]}, "overlap"),
    ({"obstacles": [entry(at_s=10, lane=["A", "B"]), entry(at_s=20, lane=["B", "A"])]},
     "overlap"),
    # YAML .nan and .inf are floats; every comparison with NaN is false, so they slipped through
    ({"obstacles": [entry(at_s=math.nan)]}, "at_s"),
    ({"obstacles": [entry(at_s=math.inf)]}, "at_s"),
    ({"obstacles": [entry(duration_s=math.inf)]}, "duration_s"),
    ({"obstacles": [entry(duration_s=math.nan)]}, "duration_s"),
    ({"obstacles": [entry(fraction=math.nan)]}, "fraction"),
    ({"obstacles": [entry()], "repeat_every_s": math.nan}, "repeat_every_s"),
    ({"obstacles": [entry()], "size_m": [0.1, 0.1, math.inf]}, "size_m"),
])
def test_bad_schedules_are_refused_with_the_reason(data, words):
    with pytest.raises(obstacles.ScheduleError, match=words):
        obstacles.parse_schedule(data, LANES)


def test_back_to_back_and_other_lanes_do_not_overlap():
    schedule = obstacles.parse_schedule({"obstacles": [
        entry(at_s=10, duration_s=20), entry(at_s=30, duration_s=5),
        entry(at_s=12, lane=["B", "R"])]}, LANES)
    assert len(schedule.obstacles) == 3


def test_non_mapping_file_is_refused(tmp_path):
    path = tmp_path / "bad.yaml"
    path.write_text("- 1\n- 2\n", encoding="utf-8")
    with pytest.raises(obstacles.ScheduleError):
        obstacles.load_schedule(path, LANES)


# ---- 레인 위 점 ------------------------------------------------------------------------------

def test_lane_point_is_along_the_segment_with_the_lane_yaw():
    # W (1.05, 1.55) -> A (1.45, 1.15): 남동쪽 45도
    x, y, yaw = obstacles.lane_point(LANES, ("W", "A"), 0.25)
    assert (x, y) == pytest.approx((1.15, 1.45))
    assert yaw == pytest.approx(-math.pi / 4)


def test_lane_point_follows_the_named_direction():
    x, y, yaw = obstacles.lane_point(LANES, ("B", "A"), 0.5)
    assert (x, y) == pytest.approx((2.0, 1.15))
    assert yaw == pytest.approx(math.pi)


def test_parsed_obstacle_carries_its_point():
    one = parse().obstacles[0]
    assert (one.x, one.y, one.yaw) == pytest.approx((1.25, 1.35, -math.pi / 4))
    assert one.label == "W->A"


# ---- 로봇과의 거리 ---------------------------------------------------------------------------

def test_clear_of_robots():
    point = (1.0, 1.0)
    assert obstacles.clear_of_robots(point, [(2.0, 2.0), (1.0, 1.4)], 0.35)
    assert not obstacles.clear_of_robots(point, [(2.0, 2.0), (1.0, 1.3)], 0.35)
    assert obstacles.clear_of_robots(point, [], 0.35)


def test_robot_without_a_position_counts_as_not_clear():
    assert not obstacles.clear_of_robots((1.0, 1.0), [(3.0, 3.0), None], 0.35)


# ---- 반복과 시각 -----------------------------------------------------------------------------

def two_entry_schedule(repeat=None):
    data = {"obstacles": [entry(at_s=10, duration_s=20),
                          entry(at_s=50, lane=["B", "R"], duration_s=30)]}
    if repeat:
        data["repeat_every_s"] = repeat
    return obstacles.parse_schedule(data, LANES)


FAR = [(0.0, 0.0)]


def test_occurrences_repeat_shifted_by_the_period():
    got = obstacles.occurrences(two_entry_schedule(repeat=100))
    first = [next(got) for _ in range(5)]
    assert [(o.number, o.due_s) for o in first] == [
        (1, 10.0), (2, 50.0), (3, 110.0), (4, 150.0), (5, 210.0)]
    assert first[2].obstacle is first[0].obstacle
    assert first[2].name == "obstacle_3"


def test_occurrences_without_repeat_end():
    assert [o.due_s for o in obstacles.occurrences(two_entry_schedule())] == [10.0, 50.0]


PLACED, SKIPPED, UNKNOWN = "placed", "skipped", "unknown"  # obstacles.PLACED, ...


def names(occs):
    return [o.name for o in occs]


def test_spawn_outcome_names():
    assert (obstacles.PLACED, obstacles.SKIPPED, obstacles.UNKNOWN) == (PLACED, SKIPPED, UNKNOWN)


def test_planner_places_when_due_and_removes_after_the_duration():
    planner = obstacles.Planner(two_entry_schedule(), start_s=0.0, max_delay_s=20.0)
    assert planner.step(9.9, FAR, 0.35) == obstacles.Step()
    step = planner.step(10.0, FAR, 0.35)
    assert names(step.spawn) == ["obstacle_1"]
    planner.spawned(step.spawn[0], 10.0, PLACED)
    assert planner.step(29.9, FAR, 0.35) == obstacles.Step()
    step = planner.step(30.0, FAR, 0.35)
    assert names(step.remove) == ["obstacle_1"]
    planner.removed(step.remove[0], 30.0, ok=True)
    assert planner.step(31.0, FAR, 0.35) == obstacles.Step()
    assert planner.in_gazebo() == []


def test_planner_waits_for_a_robot_to_move_away_and_counts_duration_from_placing():
    planner = obstacles.Planner(two_entry_schedule(), start_s=0.0, max_delay_s=20.0)
    on_box = [(1.25, 1.35)]
    step = planner.step(10.0, on_box, 0.35)
    assert step.spawn == [] and names(step.waiting) == ["obstacle_1"]
    assert planner.step(15.0, on_box, 0.35).spawn == []
    [occ] = planner.step(16.0, FAR, 0.35).spawn
    planner.spawned(occ, 16.0, PLACED)
    assert planner.step(35.9, FAR, 0.35).remove == []
    assert names(planner.step(36.0, FAR, 0.35).remove) == ["obstacle_1"]


def test_hold_time_counts_from_the_actual_spawn_not_from_the_hand_off():
    """The gz calls run one after another; a spawn can finish seconds after it was handed out."""
    planner = obstacles.Planner(two_entry_schedule(), start_s=0.0, max_delay_s=20.0)
    [occ] = planner.step(10.0, FAR, 0.35).spawn
    assert planner.step(29.0, FAR, 0.35).remove == []  # call still running: nothing to remove
    planner.spawned(occ, 14.0, PLACED)
    assert planner.step(33.9, FAR, 0.35).remove == []
    assert names(planner.step(34.0, FAR, 0.35).remove) == ["obstacle_1"]


def test_planner_gives_up_after_the_maximum_delay():
    planner = obstacles.Planner(two_entry_schedule(), start_s=0.0, max_delay_s=20.0)
    on_box = [(1.25, 1.35)]
    planner.step(10.0, on_box, 0.35)
    assert planner.step(30.0, on_box, 0.35).give_up == []
    step = planner.step(30.1, on_box, 0.35)
    assert names(step.give_up) == ["obstacle_1"]
    assert planner.step(31.0, FAR, 0.35) == obstacles.Step()


def test_an_entry_past_the_maximum_delay_is_given_up_even_if_the_spot_is_free_now():
    planner = obstacles.Planner(two_entry_schedule(), start_s=0.0, max_delay_s=20.0)
    planner.step(10.0, [(1.25, 1.35)], 0.35)  # waits
    step = planner.step(31.0, FAR, 0.35)  # next look comes late, and the spot is free
    assert step.spawn == [] and names(step.give_up) == ["obstacle_1"]


def test_a_spawn_skipped_at_the_last_check_waits_again_under_the_same_delay_limit():
    """The worker checks the robots again right before the gz call (try_spawn)."""
    planner = obstacles.Planner(two_entry_schedule(), start_s=0.0, max_delay_s=20.0)
    [occ] = planner.step(10.0, FAR, 0.35).spawn
    planner.spawned(occ, 11.0, SKIPPED)
    assert planner.in_gazebo() == []
    on_box = [(1.25, 1.35)]
    assert names(planner.step(12.0, on_box, 0.35).waiting) == ["obstacle_1"]
    assert names(planner.step(13.0, FAR, 0.35).spawn) == ["obstacle_1"]
    planner2 = obstacles.Planner(two_entry_schedule(), start_s=0.0, max_delay_s=20.0)
    [occ] = planner2.step(10.0, FAR, 0.35).spawn
    planner2.spawned(occ, 30.5, SKIPPED)
    assert names(planner2.step(30.5, FAR, 0.35).give_up) == ["obstacle_1"]


def test_a_spawn_without_a_clear_answer_is_removed_in_case_it_landed():
    """A lost reply does not mean Gazebo did not create the box."""
    planner = obstacles.Planner(two_entry_schedule(), start_s=0.0, max_delay_s=20.0)
    [occ] = planner.step(10.0, FAR, 0.35).spawn
    planner.spawned(occ, 13.0, UNKNOWN)
    assert names(planner.in_gazebo()) == ["obstacle_1"]
    assert planner.placed() == []
    step = planner.step(13.2, FAR, 0.35)
    assert names(step.remove) == ["obstacle_1"]
    planner.removed(step.remove[0], 13.5, ok=True)
    assert planner.in_gazebo() == []


def test_a_failed_remove_is_tried_again_and_reported_once_it_runs_out_of_tries():
    planner = obstacles.Planner(two_entry_schedule(), start_s=0.0, max_delay_s=20.0)
    [occ] = planner.step(10.0, FAR, 0.35).spawn
    planner.spawned(occ, 10.0, PLACED)
    t = 30.0
    for attempt in range(obstacles.MAX_REMOVE_TRIES):
        step = planner.step(t, FAR, 0.35)
        assert names(step.remove) == ["obstacle_1"], attempt
        planner.removed(step.remove[0], t, ok=False)
        assert names(planner.in_gazebo()) == ["obstacle_1"]
        if attempt < obstacles.MAX_REMOVE_TRIES - 1:
            assert planner.step(t + obstacles.REMOVE_RETRY_S - 0.1, FAR, 0.35).remove == []
        t += obstacles.REMOVE_RETRY_S
    step = planner.step(t, FAR, 0.35)
    assert step.remove == [] and names(step.lost) == ["obstacle_1"]
    assert planner.step(t + 5.0, FAR, 0.35) == obstacles.Step()  # before obstacle_2 is due
    assert names(planner.in_gazebo()) == ["obstacle_1"]  # shutdown cleanup still tries it


def test_planner_catches_up_several_due_at_once_and_repeats():
    planner = obstacles.Planner(two_entry_schedule(repeat=100), start_s=0.0, max_delay_s=20.0)
    step = planner.step(25.0, FAR, 0.35)
    assert names(step.spawn) == ["obstacle_1"]  # due at 10 s, still within the delay limit
    step = planner.step(55.0, FAR, 0.35)
    assert names(step.spawn) == ["obstacle_2"]
    assert names(planner.step(110.0, FAR, 0.35).spawn) == ["obstacle_3"]


def test_planner_skips_what_was_due_before_it_started():
    planner = obstacles.Planner(two_entry_schedule(repeat=100), start_s=120.0, max_delay_s=20.0)
    assert planner.step(120.0, FAR, 0.35).spawn == []
    assert names(planner.step(150.0, FAR, 0.35).spawn) == ["obstacle_4"]


def test_planner_lists_what_is_placed_and_what_may_be_in_gazebo():
    planner = obstacles.Planner(two_entry_schedule(), start_s=0.0, max_delay_s=20.0)
    [occ] = planner.step(10.0, FAR, 0.35).spawn
    assert planner.placed() == [] and names(planner.in_gazebo()) == ["obstacle_1"]
    planner.spawned(occ, 10.5, PLACED)
    assert names(planner.placed()) == ["obstacle_1"]
    assert names(planner.in_gazebo()) == ["obstacle_1"]


# ---- the last check before a spawn, and stale positions ---------------------------------------

def test_try_spawn_checks_the_robots_again_right_before_the_call():
    occ = obstacles.Occurrence(1, parse().obstacles[0], 10.0)
    calls = []
    far, near = [(0.0, 0.0)], [(1.25, 1.35)]
    assert obstacles.try_spawn(occ, lambda: near, 0.35, lambda: calls.append(1) or True) \
        == SKIPPED
    assert calls == []
    assert obstacles.try_spawn(occ, lambda: far, 0.35, lambda: True) == PLACED
    assert obstacles.try_spawn(occ, lambda: far, 0.35, lambda: False) == UNKNOWN


def test_a_stale_position_counts_as_unknown():
    truth = {"pinky1": ((0.0, 0.0), 100.0), "pinky2": ((3.0, 3.0), 98.0)}
    got = obstacles.fresh_positions(truth, ["pinky1", "pinky2", "pinky3"], now=100.5,
                                    max_age_s=1.0)
    assert got == [(0.0, 0.0), None, None]
    assert not obstacles.clear_of_robots((1.0, 1.0), got, 0.35)


def test_a_non_finite_position_counts_as_unknown():
    truth = {"pinky1": ((math.inf, 0.0), 100.0), "pinky2": ((0.0, math.nan), 100.0)}
    assert obstacles.fresh_positions(truth, ["pinky1", "pinky2"], now=100.0,
                                     max_age_s=1.0) == [None, None]


def test_try_spawn_gives_a_late_entry_back_without_calling():
    """A spawn can wait in the worker queue past max_delay_s; the planner then gives it up."""
    occ = obstacles.Occurrence(1, parse().obstacles[0], 10.0)
    calls = []
    got = obstacles.try_spawn(occ, lambda: [(0.0, 0.0)], 0.35, lambda: calls.append(1) or True,
                              too_late=lambda: True)
    assert got == SKIPPED and calls == []
    planner = obstacles.Planner(two_entry_schedule(), start_s=0.0, max_delay_s=20.0)
    [occ] = planner.step(10.0, FAR, 0.35).spawn
    planner.spawned(occ, 31.0, got)
    assert names(planner.step(31.0, FAR, 0.35).give_up) == ["obstacle_1"]


def test_planner_late_check_matches_try_spawn():
    planner = obstacles.Planner(two_entry_schedule(), start_s=0.0, max_delay_s=20.0)
    occ = obstacles.Occurrence(1, parse().obstacles[0], 10.0)
    assert not planner.too_late(occ, 30.0)
    assert planner.too_late(occ, 30.1)


@pytest.mark.parametrize("bad", [math.nan, math.inf, -1.0])
def test_planner_refuses_a_bad_delay_limit(bad):
    with pytest.raises(ValueError, match="max_delay_s"):
        obstacles.Planner(two_entry_schedule(), start_s=0.0, max_delay_s=bad)


@pytest.mark.parametrize("bad", [math.nan, math.inf, -0.1])
def test_bad_clearance_is_refused(bad):
    with pytest.raises(ValueError, match="clearance_m"):
        obstacles.check_clearance(bad)


# ---- gz service 요청 ------------------------------------------------------------------------

def test_spawn_argv_names_the_world_service_and_a_static_box():
    one = parse(size_m=[0.12, 0.12, 0.25]).obstacles[0]
    argv = obstacles.spawn_argv("lane_arena", "obstacle_3", one, (0.12, 0.12, 0.25))
    assert argv[:3] == ["gz", "service", "-s"]
    assert argv[argv.index("-s") + 1] == "/world/lane_arena/create"
    assert argv[argv.index("--reqtype") + 1] == "gz.msgs.EntityFactory"
    assert argv[argv.index("--reptype") + 1] == "gz.msgs.Boolean"
    assert int(argv[argv.index("--timeout") + 1]) == 3000
    req = argv[argv.index("--req") + 1]
    assert req.startswith('sdf: "') and '"' not in req[6:req.index('" pose')]
    assert "<model name='obstacle_3'>" in req and "<static>true</static>" in req
    assert "<collision" in req and "<visual" in req
    assert req.count("<size>0.12 0.12 0.25</size>") == 2
    assert "<pose>0 0 0.125 0 0 0</pose>" in req  # 상자 밑면이 바닥에 닿는다
    assert 'name: "obstacle_3"' in req
    assert "position: {x: 1.25 y: 1.35 z: 0}" in req
    qz, qw = math.sin(-math.pi / 8), math.cos(-math.pi / 8)
    assert f"orientation: {{x: 0 y: 0 z: {qz:.6g} w: {qw:.6g}}}" in req


def test_remove_argv_names_the_model():
    argv = obstacles.remove_argv("lane_arena", "obstacle_3", timeout_ms=1000)
    assert argv == ["gz", "service", "-s", "/world/lane_arena/remove",
                    "--reqtype", "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean",
                    "--timeout", "1000", "--req", 'name: "obstacle_3" type: MODEL']


def test_reply_ok_reads_the_boolean_reply():
    assert obstacles.reply_ok("data: true\n\n")
    assert not obstacles.reply_ok("data: false\n")
    assert not obstacles.reply_ok("Service call timed out\n")


# ---- 기본 일정 -------------------------------------------------------------------------------

def test_default_roam_schedule_fits_the_default_map_and_the_blocked_timeout():
    schedule = obstacles.load_schedule(PKG / "obstacles" / "roam.yaml", LANES)
    assert schedule.repeat_every_s == 300.0
    assert len(schedule.obstacles) >= 3
    assert all(o.duration_s < 60.0 for o in schedule.obstacles)


# ---- RViz 표시 --------------------------------------------------------------------------------

def test_marker_specs_clear_everything_then_draw_a_box_and_label_per_placed_obstacle():
    one = parse(size_m=[0.12, 0.12, 0.25]).obstacles[0]
    placed = [obstacles.Occurrence(3, one, 10.0), obstacles.Occurrence(7, one, 20.0)]
    specs = obstacles.marker_specs(placed, (0.12, 0.12, 0.25))
    assert specs[0] == {"action": "deleteall"}
    cubes = [s for s in specs if s.get("type") == "cube"]
    texts = [s for s in specs if s.get("type") == "text"]
    assert [c["id"] for c in cubes] == [3, 7] and [t["id"] for t in texts] == [3, 7]
    cube, text = cubes[0], texts[0]
    assert cube["action"] == "add" and cube["ns"] != text["ns"]
    assert (cube["x"], cube["y"], cube["z"]) == pytest.approx((1.25, 1.35, 0.125))
    assert cube["yaw"] == pytest.approx(-math.pi / 4)
    assert cube["scale"] == (0.12, 0.12, 0.25)
    r, g, b, a = cube["color"]
    assert r > 0.8 and 0.3 < g < 0.6 and b < 0.2 and a == 1.0
    assert text["text"] == "obstacle_3" and text["action"] == "add"
    assert text["z"] > 0.25 and (text["x"], text["y"]) == pytest.approx((1.25, 1.35))


def test_marker_specs_with_nothing_placed_only_clear():
    assert obstacles.marker_specs([], obstacles.DEFAULT_SIZE_M) == [{"action": "deleteall"}]
