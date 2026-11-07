"""격리 검사 시험. Gazebo 시뮬은 실물과 같은 /pinkyN/... 이름을 쓰므로
로봇 네트워크에 붙으면 안 된다.
"""
import pytest
from fleet_gz_sim.isolation import problems, require_isolated

SAFE = {"ROS_DOMAIN_ID": "43", "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST"}


def with_(**changes):
    env = dict(SAFE)
    for key, value in changes.items():
        if value is None:
            env.pop(key, None)
        else:
            env[key] = value
    return env


def test_isolated_setting_passes():
    assert problems(SAFE) == []
    assert problems(with_(RMW_IMPLEMENTATION="rmw_fastrtps_cpp", ROS_STATIC_PEERS="")) == []


@pytest.mark.parametrize("domain", ["43", "101"])
def test_domain_range_ends_are_accepted(domain):
    assert problems(with_(ROS_DOMAIN_ID=domain)) == []


# 102~232는 리눅스 기본 임시 포트(32768~60999)와 겹치거나 프로세스 수가 모자란다.
# "²"는 str.isdigit()이 참이지만 int()가 받지 않는다. 4300자리를 넘는 숫자도 int()가 거부한다.
@pytest.mark.parametrize("domain", [None, "", "abc", "6", "42", "102", "232", "233",
                                    "²", " 43", "43 ",
                                    pytest.param("9" * 5000, id="5000_digits")])
def test_domain_must_be_set_and_away_from_the_robot_network(domain):
    found = problems(with_(ROS_DOMAIN_ID=domain))
    assert len(found) == 1 and "ROS_DOMAIN_ID" in found[0]


@pytest.mark.parametrize("scope", [None, "SUBNET", "SYSTEM_DEFAULT", "localhost"])
def test_discovery_must_stay_on_this_computer(scope):
    found = problems(with_(ROS_AUTOMATIC_DISCOVERY_RANGE=scope))
    assert len(found) == 1 and "ROS_AUTOMATIC_DISCOVERY_RANGE" in found[0]


def test_static_peers_reach_other_computers():
    found = problems(with_(ROS_STATIC_PEERS="192.168.0.10"))
    assert len(found) == 1 and "ROS_STATIC_PEERS" in found[0]


def test_only_the_repo_dds_is_accepted():
    found = problems(with_(RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"))
    assert len(found) == 1 and "RMW_IMPLEMENTATION" in found[0]


def test_the_rmw_actually_loaded_wins_over_the_variable():
    # 변수가 없을 때 실제 기본값이 Fast DDS가 아닌 설치도 있다.
    found = problems(SAFE, rmw="rmw_cyclonedds_cpp")
    assert len(found) == 1 and "rmw_cyclonedds_cpp" in found[0]
    assert problems(SAFE, rmw="rmw_fastrtps_cpp") == []
    assert problems(with_(RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"), rmw="rmw_fastrtps_cpp") == []


@pytest.mark.parametrize("name", ["ROS_DISCOVERY_SERVER", "FASTRTPS_DEFAULT_PROFILES_FILE",
                                  "FASTDDS_DEFAULT_PROFILES_FILE", "FASTDDS_ENVIRONMENT_FILE",
                                  "FASTDDS_BUILTIN_TRANSPORTS"])
def test_fast_dds_settings_that_can_bypass_localhost_are_refused(name):
    found = problems(with_(**{name: "x"}))
    assert len(found) == 1 and name in found[0]
    assert problems(with_(**{name: ""})) == []


@pytest.mark.parametrize("name", ["DEFAULT_FASTRTPS_PROFILES.xml", "DEFAULT_FASTDDS_PROFILES.xml"])
def test_profile_file_in_the_working_directory_is_refused(tmp_path, name):
    assert problems(SAFE, workdir=tmp_path) == []
    (tmp_path / name).write_text("<dds/>")
    found = problems(SAFE, workdir=tmp_path)
    assert len(found) == 1 and name in found[0]


def test_every_problem_is_reported_at_once():
    assert len(problems({"ROS_STATIC_PEERS": "10.0.0.1"})) == 3


def test_require_isolated_refuses_with_the_reasons():
    with pytest.raises(RuntimeError, match="ROS_DOMAIN_ID"):
        require_isolated(with_(ROS_DOMAIN_ID="6"))
    require_isolated(SAFE)
