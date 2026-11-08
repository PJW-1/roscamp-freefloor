"""recovery_probe: ROS-free logic (episodes, truth matching, CSV). Wiring: test_launch_wiring."""
import csv
import math

import pytest
from fleet_gz_sim.recovery_probe import COLUMNS, CsvSink, RecoveryProbe, wrap_deg

R = "pinky1"


def probe(settle_s=2.0):
    return RecoveryProbe([R, "pinky2"], settle_s=settle_s)


def feed_truth(p, t, x=1.0, y=2.0, yaw=0.0, robot=R):
    p.on_truth(robot, t, x, y, yaw)


def state(p, now, st, x=1.0, y=2.0, yaw=0.0, stamp=None, robot=R):
    return p.on_state(robot, now, st, x, y, yaw, now if stamp is None else stamp)


def test_wrap_deg():
    assert wrap_deg(190) == pytest.approx(-170)
    assert wrap_deg(-190) == pytest.approx(170)
    assert wrap_deg(180) == pytest.approx(180)
    assert wrap_deg(360) == pytest.approx(0)


def test_no_rows_outside_recovering():
    p = probe()
    feed_truth(p, 1.0)
    assert state(p, 1.0, "MOVING") == []
    assert state(p, 1.1, "IDLE") == []


def test_start_row_has_errors_and_age():
    p = probe()
    feed_truth(p, 1.0, x=1.0, y=2.0, yaw=math.radians(170))
    [row] = state(p, 1.05, "RECOVERING", x=1.3, y=2.4, yaw=math.radians(-170), stamp=0.98)
    assert row["event"] == "start" and row["episode"] == 1 and row["robot"] == R
    assert row["sim_time"] == pytest.approx(1.05)
    assert row["err_m"] == pytest.approx(0.5)
    assert row["yaw_err_deg"] == pytest.approx(20.0)  # -170 - 170 = -340 -> 20
    assert row["report_age_s"] == pytest.approx(0.02)
    assert row["truth_x"] == 1.0 and row["amcl_x"] == 1.3


def test_recovering_to_recovering_is_one_episode_and_end_has_max_err():
    p = probe()
    for t in (1.0, 2.0, 3.0):
        feed_truth(p, t)
    assert [r["event"] for r in state(p, 1.0, "RECOVERING", x=1.1)] == ["start"]
    assert state(p, 2.0, "RECOVERING", x=1.6) == []
    [end] = state(p, 3.0, "MOVING", x=1.2)
    assert end["event"] == "end" and end["episode"] == 1
    assert end["next_state"] == "MOVING" and end["state"] == "MOVING"
    assert end["duration_s"] == pytest.approx(2.0)
    assert end["max_err_m"] == pytest.approx(0.6)


def test_leaving_to_blocked_and_episode_numbers_are_per_robot():
    p = probe()
    feed_truth(p, 1.0)
    feed_truth(p, 5.0)
    state(p, 1.0, "RECOVERING")
    [end] = state(p, 1.5, "BLOCKED")
    assert end["next_state"] == "BLOCKED"
    [start] = state(p, 5.0, "RECOVERING")
    assert start["episode"] == 2
    [other] = state(p, 5.0, "RECOVERING", robot="pinky2")
    assert other["episode"] == 1


def test_settled_row_comes_settle_s_after_leaving():
    p = probe(settle_s=2.0)
    for t in (1.0, 1.5, 3.4, 3.6):
        feed_truth(p, t)
    state(p, 1.0, "RECOVERING")
    [end] = state(p, 1.5, "MOVING")
    assert end["event"] == "end"
    assert state(p, 3.4, "MOVING") == []  # 1.9 s after leaving
    [settled] = state(p, 3.6, "MOVING")
    assert settled["event"] == "settled" and settled["episode"] == 1
    assert settled["state"] == "MOVING"
    assert state(p, 4.0, "MOVING") == []  # only once


def test_reentering_before_settle_drops_the_pending_settled_row():
    p = probe()
    state(p, 1.0, "RECOVERING")
    state(p, 1.5, "MOVING")
    [start] = state(p, 2.0, "RECOVERING")
    assert start["episode"] == 2
    state(p, 2.5, "MOVING")
    rows = state(p, 5.0, "MOVING")
    assert [(r["event"], r["episode"]) for r in rows] == [("settled", 2)]


def test_truth_matches_nearest_to_the_report_stamp():
    p = probe()
    feed_truth(p, 1.00, x=10.0)
    feed_truth(p, 1.04, x=11.0)
    feed_truth(p, 1.10, x=12.0)
    [row] = state(p, 1.2, "RECOVERING", stamp=1.05)
    assert row["truth_x"] == 11.0
    assert row["report_age_s"] == pytest.approx(-0.01)


def test_no_truth_within_0_1_s_leaves_truth_fields_empty_and_notes_it():
    p = probe()
    feed_truth(p, 1.0)
    [row] = state(p, 1.3, "RECOVERING", stamp=1.3)
    assert row["truth_x"] == "" and row["err_m"] == "" and row["yaw_err_deg"] == ""
    assert row["report_age_s"] == ""
    assert "no_truth" in row["note"]
    [none] = state(p, 1.3, "RECOVERING", robot="pinky2", stamp=1.3)  # no truth at all
    assert "no_truth" in none["note"]


def test_max_err_ignores_reports_without_truth():
    p = probe()
    feed_truth(p, 1.0)
    state(p, 1.0, "RECOVERING", x=1.2)
    state(p, 5.0, "RECOVERING", x=9.0)  # no truth near 5.0
    [end] = state(p, 5.0, "MOVING", x=1.3)  # no truth either
    assert end["max_err_m"] == pytest.approx(0.2)


def test_old_truth_is_dropped():
    p = probe()
    for i in range(1000):
        feed_truth(p, i * 0.05)
    assert len(p.truth[R]) < 1000


def test_csv_header_once_and_flushed_per_row(tmp_path):
    path = tmp_path / "sub" / "recovery_probe.csv"
    p = probe()
    feed_truth(p, 1.0)
    [row] = state(p, 1.0, "RECOVERING")
    sink = CsvSink(str(path))
    sink.write(row)
    assert len(list(csv.DictReader(open(path)))) == 1  # visible before close
    sink.close()
    sink = CsvSink(str(path))  # reopening appends without a second header
    sink.write(row)
    sink.close()
    lines = path.read_text().splitlines()
    assert lines[0].split(",") == COLUMNS
    assert sum(1 for line in lines if line.startswith("sim_time")) == 1
    assert len(lines) == 3


def test_empty_result_dir_means_log_only():
    sink = CsvSink("")
    sink.write({"event": "start"})
    sink.close()
