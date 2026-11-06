#!/usr/bin/env python3
"""FMS 기록을 ROS 없이 두 코어에 다시 재생하고, 나온 결과를 비교한다.

    ros2 run fleet_manager fleet_manager_node --record run.jsonl
        (또는 pc.launch.py에 record:=run.jsonl)
    python3 tools/replay.py run.jsonl

첫 줄은 코어를 띄운 설정이다(로봇, 파라미터, 맵 본문, 양방향 설정).
이후 줄은 모두 시각 `t`와 아래 중 하나를 가진다.

    {"t": 12.5, "in": "on_robot_state", "args": ["pinky1", 1.2, 2.0, 0.0, "MOVING", 90.0]}
    {"t": 12.6, "in": "tick", "args": []}
    {"t": 12.6, "inside": "DispatchCmd", "goal_id": 3, "station_id": "WORK", "robot_id": ""}
    {"t": 12.6, "out": "MoveCmd", "robot": "pinky1", "goal_id": 4, "target_id": "N", ...}

`in`은 FleetManager의 메서드, `args`는 그 인자다. 한 번의 실행은 `in` 줄이 이어지고 각각 뒤에
그 결과가 붙는 형태다. `inside`는 두 코어끼리 주고받은 것, `out`은 ROS로 나간 것이다.
시계는 `t`다. 코어는 호출 한 번에 시각 하나만 읽으며 실제 실행도 같다.
노드가 코어를 거치지 않고 스스로 정하는 것은 기록에 없다. CANCEL_UNCONFIRMED,
ORDER_REPORT_REFUSED 이벤트와 그 바탕이 되는 취소/주문 보고의 응답이다.
두 이벤트는 /central/events로 나가므로 bag에는 있다.

첫 줄의 `"recorded": true`는 코어가 직접 쓴 기록이라는 뜻이며, 결과가 파일에 있으므로 일치해야 한다.
입력만 있는 기록(변환한 bag 등)은 `inside`, `out` 줄이 없어 그냥 돌리고 나온 것을 보고한다.
이런 파일에는 `tick` 줄(실제 노드는 기본 초당 10번)과 첫 줄이 따로 필요하다.

기록은 프로세스가 죽은 경우처럼 줄 중간에서 끝나도 된다. 있는 부분만 비교한다.
끝 뒤의 줄은 마지막 입력이 낳은 것뿐이며, 비교하지 못한 줄 수를 알려 준다.

종료 코드: 모두 같으면 0, 처음 다른 곳에서 1, 재생할 수 없으면 2.
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "pc" / "fleet_core"))

from fleet_core.common.commands import OrderRecord  # noqa: E402
from fleet_core.common.params import FleetParams  # noqa: E402
from fleet_core.manager import FleetManager  # noqa: E402
from fleet_core.traffic.lane_map import LaneMap  # noqa: E402


@dataclass(frozen=True)
class Result:
    inputs: int
    inside: int
    commands: int
    compared: bool  # 입력만 있는 기록이면 False
    same: bool
    line: int = 0  # 처음 다른 줄(1부터)
    recorded: str = ""
    produced: str = ""
    uncompared: int = 0  # 기록이 끝난 뒤에 나온 줄 수
    cut_line: bool = False  # 마지막 줄이 반쯤만 써져 빼 둠


def _count(lines: list[str], key: str) -> int:
    return sum(key in json.loads(line) for line in lines[1:])


def _arguments(entry: dict) -> list:
    if entry["in"] == "on_order_board":
        mode, orders = entry["args"]
        return [mode, [OrderRecord(**order) for order in orders]]
    return entry["args"]


def _read(path) -> tuple[list[str], list[dict], bool]:
    """줄 목록, 내용, 끝의 반쯤 쓴 줄을 뺐는지 여부."""
    text = Path(path).read_text(encoding="utf-8")
    lines = text.splitlines()
    entries = []
    for number, line in enumerate(lines, start=1):
        try:
            entries.append(json.loads(line))
        except json.JSONDecodeError as error:
            if number == len(lines) and not text.endswith("\n"):
                return lines[:-1], entries, True
            raise ValueError(f"line {number} is not JSON: {error}") from error
    return lines, entries, False


def start_core(head: dict, clock) -> FleetManager:
    missing = {"robots", "params", "map"} - set(head)
    if missing:
        raise ValueError(f"the first line has no {sorted(missing)}")
    try:
        params = FleetParams(**head["params"])
    except TypeError as error:
        raise ValueError(f"these parameters do not fit this code: {error}") from error
    site = LaneMap(yaml.safe_load(head["map"]), head["map"], head.get("bundle_two_way"))
    return FleetManager(site, params, head["robots"], clock)


def replay(path) -> Result:
    lines, entries, cut_line = _read(path)
    if not entries:
        raise ValueError("no first line, so this is not a record")
    head, *rest = entries
    if not any("in" in entry for entry in rest):
        raise ValueError("no inputs, so there is nothing to play back")
    now = [0.0]
    core = start_core(head, lambda: now[0])
    produced: list[str] = []
    core.record_to(produced.append)
    for entry in rest:
        if "in" in entry:
            now[0] = entry["t"]
            getattr(core, entry["in"])(*_arguments(entry))
    counts = tuple(_count(produced, key) for key in ("in", "inside", "out"))
    if not head.get("recorded") and not any("inside" in e or "out" in e for e in rest):
        return Result(*counts, False, True, cut_line=cut_line)
    # 첫 줄은 코어를 만든 입력이지 코어의 판단이 아니라 제외한다.
    for number, (old, new) in enumerate(zip(lines[1:], produced[1:], strict=False), start=2):
        if old != new:
            return Result(*counts, True, False, number, old, new, cut_line=cut_line)
    if len(lines) > len(produced):
        return Result(*counts, True, False, len(produced) + 1, lines[len(produced)],
                      "(nothing came out)", cut_line=cut_line)
    return Result(*counts, True, True, uncompared=len(produced) - len(lines), cut_line=cut_line)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("record", help="the JSONL file the fleet manager node wrote with --record")
    try:
        result = replay(parser.parse_args().record)
    except (ValueError, OSError) as error:
        print(f"cannot play this back: {error}")
        return 2
    print(f"{result.inputs} inputs, {result.inside} inside calls, {result.commands} commands")
    if result.cut_line:
        print("the last line was cut off and left out")
    if not result.compared:
        print("inputs only: nothing to compare with")
    elif result.same and result.uncompared:
        print(f"same as recorded, as far as it goes: {result.uncompared} lines past its end "
              "were not compared")
    elif result.same and result.cut_line:
        print("same as recorded, as far as it goes")
    elif result.same:
        print("same as recorded")
    else:
        print(f"line {result.line} differs\n recorded: {result.recorded}\n produced: "
              f"{result.produced}")
    return 0 if result.same else 1


if __name__ == "__main__":
    sys.exit(main())
