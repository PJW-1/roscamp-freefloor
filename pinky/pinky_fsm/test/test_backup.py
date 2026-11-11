"""The back-up primitive (backup.py): how far the robot may back up and what a result means.

The self-recovery uses it now; the version 2 Retreat action will use the same code
(FMS 개선 계획 버전 2, 1-A). ROS is needed only for the last test.
"""
import pytest
from pinky_fsm.backup import (
    CANCELED,
    FAILED,
    NAV2_COLLISION_AHEAD,
    NAV2_NO_VALID_CONTROL,
    NAV2_TIMEOUT,
    OBSTACLE_BEHIND,
    TIMEOUT,
    backup_distance,
    keep_lateral,
    reason_of,
)


def test_distance_is_capped_at_the_maximum():
    assert backup_distance(1.0, max_m=0.15, keep_m=0.23) == pytest.approx(0.15)


def test_distance_keeps_the_centre_clear_of_the_last_vertex():
    """Distance is counted from the last path vertex the robot passed (the path start counts as
    one). Once the robot is release_distance_m (0.2) past a vertex the fleet manager may release
    the lane behind, and a follower may wait at that vertex. The centre stays release + eps."""
    assert backup_distance(0.32, max_m=0.15, keep_m=0.23) == pytest.approx(0.09)


def test_no_distance_close_to_the_last_vertex():
    assert backup_distance(0.20, max_m=0.15, keep_m=0.23) == 0.0
    assert backup_distance(0.0, max_m=0.15, keep_m=0.23) == 0.0


def test_lateral_limit_leaves_a_back_up_toward_the_path_alone():
    # 0.05 m left of the path, facing 30 deg left of it: backing up moves toward the path
    assert keep_lateral(0.15, lateral_m=0.05, heading_err=0.5236, limit_m=0.10) == \
        pytest.approx(0.15)


def test_lateral_limit_shortens_a_back_up_away_from_the_path():
    # 0.05 m left, facing 30 deg right: backing up drifts left by half the distance
    assert keep_lateral(0.15, lateral_m=0.05, heading_err=-0.5236, limit_m=0.10) == \
        pytest.approx(0.10, abs=1e-3)


def test_lateral_limit_never_makes_an_offset_beyond_the_limit_worse():
    assert keep_lateral(0.15, lateral_m=0.12, heading_err=-0.5236, limit_m=0.10) == 0.0
    # beyond the limit but moving back toward the path is fine
    assert keep_lateral(0.15, lateral_m=0.12, heading_err=0.5236, limit_m=0.10) == \
        pytest.approx(0.15)


def test_lateral_limit_with_a_straight_heading_changes_nothing():
    assert keep_lateral(0.15, lateral_m=0.09, heading_err=0.0, limit_m=0.10) == \
        pytest.approx(0.15)


def test_reasons_follow_the_version_2_retreat_result():
    assert reason_of("succeeded", 0) == ""
    assert reason_of("canceled", 0) == CANCELED
    assert reason_of("aborted", NAV2_COLLISION_AHEAD) == OBSTACLE_BEHIND
    assert reason_of("aborted", NAV2_TIMEOUT) == TIMEOUT
    # not in the version 2 list: no server, TF error, bad input
    assert reason_of("rejected", 0) == FAILED
    assert reason_of("aborted", 712) == FAILED


def test_a_reverse_follow_path_stopped_by_its_collision_check_is_an_obstacle_behind():
    """Retreat.path(제안): RPP collision check on the backing motion ends FollowPath with
    NO_VALID_CONTROL. Other FollowPath failures (no progress, TF) are FAILED."""
    assert reason_of("aborted", NAV2_NO_VALID_CONTROL) == OBSTACLE_BEHIND
    assert reason_of("aborted", 105) == FAILED


def test_error_codes_match_nav2():
    action = pytest.importorskip("nav2_msgs.action")
    assert NAV2_TIMEOUT == action.BackUp.Result.TIMEOUT
    assert NAV2_COLLISION_AHEAD == action.BackUp.Result.COLLISION_AHEAD
    assert NAV2_NO_VALID_CONTROL == action.FollowPath.Result.NO_VALID_CONTROL
