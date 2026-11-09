"""Sim-only Nav2 parameters: AMCL takes sim values, the real robot's file stays as it is."""
from pathlib import Path

import pytest
import yaml

launch = pytest.importorskip("launch")
from fleet_gz_sim import launch_parts  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
NAV = REPO / "pinky" / "pinky_nav_config" / "params" / "nav2_params.yaml"


def _amcl(data: dict) -> dict:
    return data["amcl"]["ros__parameters"]


def test_sim_params_change_amcl_only_and_leave_the_real_file(tmp_path):
    before = NAV.read_bytes()
    out = launch_parts.sim_nav_params(NAV, tmp_path / "nav2_sim.yaml")
    assert NAV.read_bytes() == before
    base, sim = yaml.safe_load(before), yaml.safe_load(out.read_text(encoding="utf-8"))
    assert {k: v for k, v in sim.items() if k != "amcl"} == {
        k: v for k, v in base.items() if k != "amcl"}
    for key, value in launch_parts.SIM_AMCL.items():
        assert _amcl(sim)[key] == value, key
    untouched = set(_amcl(base)) - set(launch_parts.SIM_AMCL)
    assert {k: _amcl(sim)[k] for k in untouched} == {k: _amcl(base)[k] for k in untouched}
    # nav2.launch.py fills these in per robot
    assert "<robot_namespace>" in out.read_text(encoding="utf-8")


def test_sim_amcl_trusts_the_scan_more_than_the_real_config():
    """In the warehouse A4->A3 corridor the real values let the AMCL pose sit 0.13-0.29 m to the
    side of the true pose (2026-11-08). The sim values weigh the map match more, use more beams
    and update the filter more often."""
    base, sim = _amcl(yaml.safe_load(NAV.read_text(encoding="utf-8"))), launch_parts.SIM_AMCL
    assert sim["z_rand"] < base["z_rand"] and sim["z_hit"] > base["z_hit"]
    assert sim["z_hit"] + sim["z_rand"] == pytest.approx(1.0)
    assert sim["max_beams"] > base["max_beams"]
    assert sim["update_min_d"] < base["update_min_d"]
    assert sim["update_min_a"] < base["update_min_a"]
    assert sim["laser_max_range"] == pytest.approx(12.0)  # the sim lidar's <max>


def _need(*packages):
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    for package in packages:
        try:
            get_package_share_directory(package)
        except PackageNotFoundError:
            pytest.skip(f"{package} is not installed")


def test_each_pinky_starts_nav2_with_the_sim_params(tmp_path, monkeypatch):
    _need("fleet_gz_sim", "pinky_nav_config")
    from launch.actions import IncludeLaunchDescription
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    monkeypatch.setattr(launch_parts.tempfile, "gettempdir", lambda: str(tmp_path))
    context = launch.LaunchContext()

    def text(value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))

    for ns in ("pinky1", "pinky2"):
        includes = [a for a in launch_parts.pinky(ns, 0.5, 0.5, 0.0, fsm=False)
                    if isinstance(a, IncludeLaunchDescription)]
        args = [{text(k): text(v) for k, v in a.launch_arguments} for a in includes]
        nav = next(a for a in args if "map_fine" in a)
        path = Path(nav["params_file"])
        assert path.parent == tmp_path
        # one file per launch process: two sim runs with the same robot names do not share it
        # (Codex review of 5a0587b)
        assert ns in path.name and str(launch_parts.os.getpid()) in path.name
        amcl = _amcl(yaml.safe_load(path.read_text(encoding="utf-8")))
        assert {k: amcl[k] for k in launch_parts.SIM_AMCL} == launch_parts.SIM_AMCL


def test_one_pinky_sends_lock_boundary_paths_to_the_through_rpp_by_default(tmp_path, monkeypatch):
    _need("fleet_gz_sim", "pinky_nav_config", "pinky_fsm")
    from launch.actions import IncludeLaunchDescription
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    monkeypatch.setattr(launch_parts.tempfile, "gettempdir", lambda: str(tmp_path))
    context = launch.LaunchContext()

    def text(value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))

    def fsm_args(**kw):
        includes = [a for a in launch_parts.pinky("pinky1", 0.5, 0.5, 0.0, **kw)
                    if isinstance(a, IncludeLaunchDescription)]
        args = [{text(k): text(v) for k, v in a.launch_arguments} for a in includes]
        return next(a for a in args if "heartbeat_timeout_s" in a)

    assert fsm_args()["through_controller_id"] == "FollowPathThrough"
    assert fsm_args(through_controller_id="")["through_controller_id"] == ""


def _rpp(data: dict, name: str) -> dict:
    return data["controller_server"]["ros__parameters"][name]


def test_sim_params_can_cap_both_rpp_speeds_and_change_nothing_else(tmp_path):
    """A small arena runs slower (real_arena: 0.12 m/s): both RPPs get the cap, nothing else."""
    capped = yaml.safe_load(launch_parts.sim_nav_params(
        NAV, tmp_path / "capped.yaml", max_speed=0.12).read_text(encoding="utf-8"))
    plain = yaml.safe_load(launch_parts.sim_nav_params(
        NAV, tmp_path / "plain.yaml").read_text(encoding="utf-8"))
    for name in (launch_parts.THROUGH_CONTROLLER, "FollowPath"):
        assert _rpp(capped, name)["desired_linear_vel"] == 0.12
        assert _rpp(plain, name)["desired_linear_vel"] == 0.25
        _rpp(plain, name)["desired_linear_vel"] = 0.12
    assert capped == plain


@pytest.mark.parametrize("bad", [0.0, -0.1, float("nan"), float("inf"), 0.3, True, "0.1"])
def test_a_speed_cap_must_be_a_number_between_zero_and_the_robot_speed(tmp_path, bad):
    with pytest.raises(ValueError, match="max_speed"):
        launch_parts.sim_nav_params(NAV, tmp_path / "a.yaml", max_speed=bad)
