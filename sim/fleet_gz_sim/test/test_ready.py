"""wait_ready의 판정: 모든 로봇의 최근 보고가 IDLE인가. ROS 없이 돈다."""
from fleet_gz_sim.ready import everyone_idle

ROBOTS = ["pinky1", "pinky2", "pinky3"]


def last(**ages):
    """로봇마다 (상태, 받은 시각). 지금은 10.0이다."""
    return {robot: (state, 10.0 - age) for robot, (state, age) in ages.items()}


def test_ready_when_every_robot_reported_idle_just_now():
    seen = last(pinky1=("IDLE", 0.1), pinky2=("IDLE", 0.0), pinky3=("IDLE", 0.3))
    assert everyone_idle(seen, ROBOTS, now=10.0, fresh_s=0.5)


def test_an_old_idle_report_does_not_count():
    """IDLE을 한 번 보내고 끊긴 로봇을 준비됐다고 치면 관제가 없는 로봇에 명령한다(Codex)."""
    seen = last(pinky1=("IDLE", 0.1), pinky2=("IDLE", 2.0), pinky3=("IDLE", 0.1))
    assert not everyone_idle(seen, ROBOTS, now=10.0, fresh_s=0.5)


def test_a_robot_not_idle_or_never_heard_is_not_ready():
    seen = last(pinky1=("IDLE", 0.1), pinky2=("MOVING", 0.1), pinky3=("IDLE", 0.1))
    assert not everyone_idle(seen, ROBOTS, now=10.0, fresh_s=0.5)
    seen = last(pinky1=("IDLE", 0.1), pinky2=("IDLE", 0.1))
    assert not everyone_idle(seen, ROBOTS, now=10.0, fresh_s=0.5)
