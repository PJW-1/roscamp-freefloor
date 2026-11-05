"""tools/pressure_sweep.py without ROS: scenario variants, result files, the table."""
import json
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))

import pressure_sweep as sweep  # noqa: E402

SCENARIO = {"seed": 7, "robots": {
    "pinky1": {"start": "P-6", "speed_mps": 0.2, "pose_noise_m": 0.03},
    "pinky2": {"start": "P-5", "speed_mps": 0.18, "pose_noise_m": 0.03}}}


def results(tmp_path, *, early=0, near=0, codes=(), passed=True, violations=()):
    (tmp_path / "summary.json").write_text(json.dumps({
        "pass": passed, "orders_total": 6, "orders_shipped": 5, "duration_s": 179.3,
        "violations": list(violations),
        "checks": {"orders_shipped": {"ok": passed}, "fifo_violations": {"ok": True}}}))
    (tmp_path / "safety.json").write_text(json.dumps({
        "early": early, "near": near, "releases": 71, "min_gap_m": 0.4, "min_release_m": 0.21,
        "radius_m": 0.08, "log": []}))
    lines = ["time_s,level,source,code,detail"]
    lines += [f"1.0,ERROR,pinky1,{code}," for code in codes]
    (tmp_path / "events.csv").write_text("\n".join(lines) + "\n")
    return tmp_path


def test_a_variant_sets_every_robot_and_leaves_the_original_alone():
    data = sweep.variant(SCENARIO, tau=0.5, eps=0.05, stop=0.02)
    assert {r["report_delay_s"] for r in data["robots"].values()} == {0.5}
    assert {r["pose_noise_m"] for r in data["robots"].values()} == {0.05}
    assert {r["stop_error_m"] for r in data["robots"].values()} == {0.02}
    assert data["robots"]["pinky2"]["speed_mps"] == 0.18
    assert "report_delay_s" not in SCENARIO["robots"]["pinky1"]
    assert SCENARIO["robots"]["pinky1"]["pose_noise_m"] == 0.03


def test_the_other_release_distance_is_all_a_params_file_changes(tmp_path):
    path = sweep.write_params(tmp_path / "p.yaml", 0.12)
    written = yaml.safe_load(path.read_text())
    default = yaml.safe_load(sweep.DEFAULT_PARAMS.read_text())
    assert written.pop("release_distance_m") == 0.12
    default.pop("release_distance_m")
    assert written == default


def test_a_row_reads_the_three_result_files(tmp_path):
    case = sweep.Case("basic_orders", 0.2, 0.03, 0.2)
    row = sweep.read_row(case, results(tmp_path, early=2, near=1, codes=(
        "DEADLOCK_DETECTED", "DEADLOCK_RETREAT", "LANE_ERROR"), passed=False))
    assert (row.early, row.near, row.deadlocks, row.releases) == (2, 1, 1, 71)
    assert row.lane_errors == 1
    assert (row.finished, row.passed, row.shipped, row.total) == (True, False, 5, 6)
    assert row.failed_checks == ("orders_shipped",)
    assert row.min_release_m == 0.21


def test_a_run_that_wrote_nothing_is_not_counted_as_clean(tmp_path):
    row = sweep.read_row(sweep.Case("basic_orders", 0.0, 0.0, 0.2), tmp_path)
    assert not row.finished
    assert "did not finish" in sweep.table([row])


def test_the_table_gives_every_combination_and_what_the_formula_asks_for(tmp_path):
    rows = [sweep.read_row(sweep.Case("basic_orders", tau, 0.05, 0.2), results(tmp_path))
            for tau in (0.0, 0.5)]
    text = sweep.table(rows)
    assert text.count("basic_orders") == 2
    # r + eps + v * (tau + tick) with r 0.08, v 0.2, tick 0.1
    assert "0.150" in text and "0.250" in text


@pytest.mark.parametrize("tau, eps, wanted", [
    (0.0, 0.0, 0.1), (0.2, 0.03, 0.17), (0.5, 0.05, 0.25)])
def test_the_formula_of_fleet_params(tau, eps, wanted):
    assert sweep.required_distance(tau, eps) == pytest.approx(wanted)


def test_the_robot_network_cannot_be_reached_by_accident():
    env = sweep.ros_env(80, {"ROS_DOMAIN_ID": "6", "ROS_AUTOMATIC_DISCOVERY_RANGE": "SUBNET"})
    assert env["ROS_DOMAIN_ID"] == "80"
    assert env["ROS_AUTOMATIC_DISCOVERY_RANGE"] == "LOCALHOST"
    with pytest.raises(ValueError):
        sweep.ros_env(6, {})
    assert "ROS_STATIC_PEERS" not in sweep.ros_env(80, {"ROS_STATIC_PEERS": "192.168.4.2"})


class FakeLaunch:
    """What subprocess.Popen gives back, for a launch that writes `files` and then ends or not."""

    def __init__(self, folder, ends=True, files=False):
        self.pid, self._folder, self._ends, self._files = 1, folder, ends, files

    def __call__(self, *_args, **_kwargs):
        if self._files:
            results(self._folder)
        return self

    def wait(self, timeout=None):
        if not self._ends and timeout == 1.0:
            raise subprocess.TimeoutExpired("ros2", timeout)
        return 0


def test_a_run_that_dies_does_not_show_the_last_run_in_that_folder(tmp_path, monkeypatch):
    case = sweep.Case("basic_orders", 0.0, 0.0, 0.2)
    folder = tmp_path / case.name
    folder.mkdir()
    results(folder)
    monkeypatch.setattr(sweep.subprocess, "Popen", FakeLaunch(folder))
    assert not sweep.run_case(case, 80, tmp_path, 1.0).finished


def test_a_run_cut_at_the_wall_is_not_a_result(tmp_path, monkeypatch):
    """A launch told to stop can still write its files, with the run half done."""
    case = sweep.Case("basic_orders", 0.0, 0.0, 0.2)
    folder = tmp_path / case.name
    folder.mkdir()
    monkeypatch.setattr(sweep.subprocess, "Popen", FakeLaunch(folder, ends=False, files=True))
    monkeypatch.setattr(sweep.os, "killpg", lambda *_: None)
    assert not sweep.run_case(case, 80, tmp_path, 1.0).finished


def test_a_run_that_ended_is_read(tmp_path, monkeypatch):
    case = sweep.Case("basic_orders", 0.0, 0.0, 0.2)
    folder = tmp_path / case.name
    folder.mkdir()
    monkeypatch.setattr(sweep.subprocess, "Popen", FakeLaunch(folder, files=True))
    assert sweep.run_case(case, 80, tmp_path, 1.0).finished


def test_every_combination_is_asked_for_once():
    cases = sweep.cases(["a", "b"], [0.0, 0.5], [0.0, 0.03, 0.05], [0.2])
    assert len(cases) == 12 == len(set(cases))
    assert len(sweep.cases(["a"], [0.0, 0.0], [0.0], [0.2])) == 1, "two runs in one folder"


def test_a_release_distance_the_core_would_refuse_is_not_run():
    """It would only spend the whole timeout of the scenario finding that out."""
    with pytest.raises(ValueError, match="2r"):
        sweep.cases(["a"], [0.0], [0.0], [0.2, 0.16])
