"""pinky_fsm.launch.py arguments. Needs the launch package, not a running ROS graph."""
import importlib.util
from pathlib import Path

import pytest

launch = pytest.importorskip("launch")

LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "pinky_fsm.launch.py"


def defaults() -> dict:
    spec = importlib.util.spec_from_file_location("pinky_fsm_launch", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = launch.LaunchContext()
    out = {}
    for action in module.generate_launch_description().entities:
        if isinstance(action, launch.actions.DeclareLaunchArgument):
            out[action.name] = launch.utilities.perform_substitutions(
                context, action.default_value)
    return out


def test_lock_boundary_paths_use_the_through_controller_by_default():
    """pinky_nav_config loads FollowPathThrough next to FollowPath; "" switches it off."""
    assert defaults()["through_controller_id"] == "FollowPathThrough"
