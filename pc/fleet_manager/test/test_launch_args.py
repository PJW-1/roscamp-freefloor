"""파일 지정은 가짜 노드에도 전달돼야 한다. 아니면 다른 맵으로 돈다."""
import importlib.util
from pathlib import Path

import pytest

pytest.importorskip("launch_ros")

from ament_index_python.packages import (  # noqa: E402
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchContext  # noqa: E402
from launch.actions import IncludeLaunchDescription  # noqa: E402
from launch.utilities import (  # noqa: E402
    normalize_to_list_of_substitutions,
    perform_substitutions,
)

PC_LAUNCH = Path(__file__).parents[3] / "bringup" / "iponic_bringup" / "launch" / "pc.launch.py"
FILES = {"map_file": "/m.yaml", "params_file": "/p.yaml", "contract_file": "/c.yaml"}


def test_file_overrides_are_forwarded_to_the_fake_launch():
    try:
        get_package_share_directory("fake_robots")
    except PackageNotFoundError:
        pytest.skip("fake_robots is not built in this workspace")
    spec = importlib.util.spec_from_file_location("pc_launch", PC_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update({
        "use_fake": "true", "use_sim_clock": "true", "scenario": "basic_orders",
        "robots": "pinky1", "fake_robots": "pinky1", "fake_arms": "arm1", "fake_central": "true",
        "result_dir": "/tmp/x", "record": "", **FILES,
    })
    include = next(a for a in module._setup(context) if isinstance(a, IncludeLaunchDescription))

    def text(value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))

    forwarded = {text(k): text(v) for k, v in include.launch_arguments}
    assert {k: forwarded.get(k) for k in FILES} == FILES
