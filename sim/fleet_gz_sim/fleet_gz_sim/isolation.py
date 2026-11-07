"""Gazebo 시뮬을 띄워도 되는 네트워크 설정인지 본다.

시뮬은 실물과 같은 이름(/pinkyN/cmd_vel 등)을 쓰므로 로봇 네트워크(기본 도메인 6)에
붙으면 실물 핑키가 움직일 수 있다. 저장소 규칙(docs/개발_규칙.md "실행")대로
도메인 43 이상, 이 컴퓨터 안에서만 찾기, 고정 피어 없음, Fast DDS일 때만 통과시킨다.
Fast DDS가 따로 읽는 설정(discovery server, XML 프로파일)은 LOCALHOST를 넘을 수
있으므로 있으면 거부한다.
"""
from collections.abc import Mapping
from pathlib import Path

MIN_DOMAIN = 43
# ROS 문서의 안전 범위 0~101. 102~214는 리눅스 기본 임시 포트(32768~60999)와 겹치고,
# 215~232는 한 컴퓨터에 띄울 수 있는 프로세스 수가 적다.
MAX_DOMAIN = 101
DDS = "rmw_fastrtps_cpp"
# Fast DDS 2.14(Jazzy)가 읽는 이름과 3.x의 새 이름
FAST_DDS_VARIABLES = ("ROS_DISCOVERY_SERVER", "FASTRTPS_DEFAULT_PROFILES_FILE",
                      "FASTDDS_DEFAULT_PROFILES_FILE", "FASTDDS_ENVIRONMENT_FILE",
                      "FASTDDS_BUILTIN_TRANSPORTS")
# 작업 폴더에 있으면 Fast DDS가 저절로 읽는 파일
FAST_DDS_FILES = ("DEFAULT_FASTRTPS_PROFILES.xml", "DEFAULT_FASTDDS_PROFILES.xml")


def problems(env: Mapping[str, str], *, rmw: str | None = None,
             workdir: Path | None = None) -> list[str]:
    """격리가 안 된 이유 목록. 비어 있으면 띄워도 된다.

    rmw: 실제로 불러온 RMW. 없으면 RMW_IMPLEMENTATION, 그것도 없으면 Jazzy 기본값.
    workdir: 노드가 뜰 작업 폴더. 주면 Fast DDS가 저절로 읽는 XML이 있는지 본다.
    """
    found = []
    domain = env.get("ROS_DOMAIN_ID", "")
    # ASCII 숫자 세 자리까지만 int()에 넘긴다("²"나 아주 긴 숫자는 int()가 예외를 낸다)
    digits = domain.isascii() and domain.isdigit() and len(domain) <= 3
    if not digits or not MIN_DOMAIN <= int(domain) <= MAX_DOMAIN:
        found.append(f"ROS_DOMAIN_ID={domain!r}: {MIN_DOMAIN}~{MAX_DOMAIN} 사이로 정해야 한다")
    scope = env.get("ROS_AUTOMATIC_DISCOVERY_RANGE", "")
    if scope != "LOCALHOST":
        found.append(f"ROS_AUTOMATIC_DISCOVERY_RANGE={scope!r}: LOCALHOST여야 한다")
    if env.get("ROS_STATIC_PEERS", ""):
        found.append("ROS_STATIC_PEERS가 있다: 다른 컴퓨터와 이어지므로 비워야 한다")
    rmw = rmw or env.get("RMW_IMPLEMENTATION", DDS)
    if rmw != DDS:
        found.append(f"RMW={rmw!r}: 저장소 기준인 {DDS}만 시험했다(RMW_IMPLEMENTATION)")
    for name in FAST_DDS_VARIABLES:
        if env.get(name, ""):
            found.append(f"{name}가 있다: LOCALHOST 밖으로 찾을 수 있으므로 비워야 한다")
    if workdir is not None:
        for name in FAST_DDS_FILES:
            if (workdir / name).exists():
                found.append(f"{workdir / name}가 있다: Fast DDS가 저절로 읽으므로 치워야 한다")
    return found


def require_isolated(env: Mapping[str, str], *, rmw: str | None = None,
                     workdir: Path | None = None) -> None:
    """격리가 안 됐으면 이유를 모두 담아 RuntimeError를 낸다."""
    found = problems(env, rmw=rmw, workdir=workdir)
    if found:
        raise RuntimeError("Gazebo 시뮬을 띄우지 않는다:\n  " + "\n  ".join(found))
