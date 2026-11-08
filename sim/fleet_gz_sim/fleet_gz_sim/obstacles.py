"""시뮬 장애물 일정: 정한 시각에 레인 위에 상자를 놓고 정한 시간 뒤에 치운다.

ROS도 Gazebo도 부르지 않는다. 노드(obstacle_node)가 시각과 로봇 위치를 넘기고, 여기서
돌려준 gz service 명령을 실행한다. 일정 파일 형식은 obstacles/roam.yaml 머리말에 있다.
시각은 시뮬 시각(초)이다.
"""
from __future__ import annotations

import math
from collections.abc import Iterator
from dataclasses import dataclass, field
from pathlib import Path

import yaml

DEFAULT_SIZE_M = (0.12, 0.12, 0.25)  # 라이다 높이(약 0.125 m)보다 높게
GZ_TIMEOUT_MS = 3000
TOP_KEYS = {"repeat_every_s", "size_m", "obstacles"}
ENTRY_KEYS = {"at_s", "lane", "fraction", "duration_s"}
REQUIRED_KEYS = {"at_s", "lane", "duration_s"}


class ScheduleError(ValueError):
    """일정 파일이 맞지 않다."""


@dataclass(frozen=True)
class Obstacle:
    at_s: float
    lane: tuple  # (from, to) 지점
    fraction: float
    duration_s: float
    x: float
    y: float
    yaw: float

    @property
    def label(self) -> str:
        return f"{self.lane[0]}->{self.lane[1]}"


@dataclass(frozen=True)
class Schedule:
    obstacles: tuple[Obstacle, ...]
    repeat_every_s: float | None
    size_m: tuple[float, float, float]


@dataclass(frozen=True)
class Occurrence:
    number: int  # 반복을 통틀어 1부터
    obstacle: Obstacle
    due_s: float

    @property
    def name(self) -> str:
        return f"obstacle_{self.number}"


@dataclass
class Step:
    spawn: list = field(default_factory=list)  # 지금 놓을 것
    remove: list = field(default_factory=list)  # 지금 치울 것
    give_up: list = field(default_factory=list)  # 로봇이 비켜 주지 않아 버린 것
    waiting: list = field(default_factory=list)  # 로봇이 가까워 기다리는 것


def _number(raw, what: str) -> float:
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        raise ScheduleError(f"{what} must be a number, got {raw!r}")
    return float(raw)


def _waypoint(raw):
    return tuple(int(v) for v in raw) if isinstance(raw, (list, tuple)) else str(raw)


def lane_point(lane_map, lane: tuple, fraction: float) -> tuple[float, float, float]:
    """레인 a→b 직선에서 fraction만큼 간 (x, y)와 레인 방향 yaw."""
    (ax, ay), (bx, by) = lane_map.waypoint_pose(lane[0]), lane_map.waypoint_pose(lane[1])
    return (ax + fraction * (bx - ax), ay + fraction * (by - ay),
            math.atan2(by - ay, bx - ax))


def _entry(raw, i: int, lane_map) -> Obstacle:
    where = f"obstacles[{i}]"
    if not isinstance(raw, dict):
        raise ScheduleError(f"{where} must be a mapping")
    unknown = set(raw) - ENTRY_KEYS
    if unknown:
        raise ScheduleError(f"{where}: unknown keys {sorted(unknown)}")
    missing = REQUIRED_KEYS - set(raw)
    if missing:
        raise ScheduleError(f"{where}: missing keys {sorted(missing)}")
    at_s = _number(raw["at_s"], f"{where}.at_s")
    if at_s < 0:
        raise ScheduleError(f"{where}.at_s must not be negative")
    duration_s = _number(raw["duration_s"], f"{where}.duration_s")
    if duration_s <= 0:
        raise ScheduleError(f"{where}.duration_s must be positive")
    fraction = _number(raw.get("fraction", 0.5), f"{where}.fraction")
    if not 0.0 < fraction < 1.0:
        raise ScheduleError(f"{where}.fraction must be between 0 and 1, got {fraction}")
    lane = raw["lane"]
    if not isinstance(lane, (list, tuple)) or len(lane) != 2:
        raise ScheduleError(f"{where}.lane must be [from, to]")
    lane = (_waypoint(lane[0]), _waypoint(lane[1]))
    # 상자는 바닥 위 선분에 놓이므로 어느 방향 레인이든 있으면 된다
    if lane not in lane_map.lanes and lane[::-1] not in lane_map.lanes:
        raise ScheduleError(f"{where}: lane {lane[0]}->{lane[1]} is not on the map")
    return Obstacle(at_s, lane, fraction, duration_s, *lane_point(lane_map, lane, fraction))


def parse_schedule(data, lane_map) -> Schedule:
    if not isinstance(data, dict):
        raise ScheduleError("the schedule must be a mapping")
    unknown = set(data) - TOP_KEYS
    if unknown:
        raise ScheduleError(f"unknown keys {sorted(unknown)}")
    raw = data.get("obstacles")
    if not isinstance(raw, list) or not raw:
        raise ScheduleError("obstacles must be a non-empty list")
    repeat = data.get("repeat_every_s")
    if repeat is not None:
        repeat = _number(repeat, "repeat_every_s")
        if repeat <= 0:
            raise ScheduleError("repeat_every_s must be positive")
    size = data.get("size_m", DEFAULT_SIZE_M)
    if not isinstance(size, (list, tuple)) or len(size) != 3:
        raise ScheduleError("size_m must be [x, y, z]")
    size = tuple(_number(v, "size_m") for v in size)
    if min(size) <= 0:
        raise ScheduleError("size_m must be positive")
    found = sorted((_entry(one, i, lane_map) for i, one in enumerate(raw)),
                   key=lambda o: o.at_s)
    for one in found:
        # 다음 주기의 같은 항목과 겹치지 않게 한 주기 안에서 끝나야 한다
        if repeat is not None and one.at_s + one.duration_s > repeat:
            raise ScheduleError(f"{one.label} at {one.at_s:g} s ends after repeat_every_s")
    for i, one in enumerate(found):
        for other in found[i + 1:]:
            same = frozenset(one.lane) == frozenset(other.lane)
            if same and other.at_s < one.at_s + one.duration_s:
                raise ScheduleError(f"{one.label} at {one.at_s:g} s and {other.label} at "
                                    f"{other.at_s:g} s overlap on the same lane")
    return Schedule(tuple(found), repeat, size)


def load_schedule(path, lane_map) -> Schedule:
    return parse_schedule(yaml.safe_load(Path(path).read_text(encoding="utf-8")), lane_map)


def occurrences(schedule: Schedule) -> Iterator[Occurrence]:
    """시각 순서대로 끝없이(반복이 없으면 한 번) 나오는 장애물."""
    n = len(schedule.obstacles)
    cycle = 0
    while True:
        shift = cycle * schedule.repeat_every_s if schedule.repeat_every_s else 0.0
        for i, one in enumerate(schedule.obstacles):
            yield Occurrence(cycle * n + i + 1, one, shift + one.at_s)
        if schedule.repeat_every_s is None:
            return
        cycle += 1


def clear_of_robots(point, robot_xy_list, clearance_m: float) -> bool:
    """모든 로봇이 clearance_m 이상 떨어져 있다. 위치를 모르는 로봇이 있으면 False."""
    return all(xy is not None and math.dist(point[:2], xy) >= clearance_m
               for xy in robot_xy_list)


class Planner:
    """시각마다 놓을 것, 치울 것, 버릴 것을 정한다. start_s 전에 정해졌던 것은 건너뛴다."""

    def __init__(self, schedule: Schedule, start_s: float, max_delay_s: float):
        self._upcoming = occurrences(schedule)
        self._next = next(self._upcoming, None)
        self._start_s = start_s
        self._max_delay_s = max_delay_s
        self._waiting: list[Occurrence] = []
        self._placed: dict[int, tuple[Occurrence, float]] = {}  # 번호 → (장애물, 치울 시각)

    def step(self, now_s: float, robot_xy_list, clearance_m: float) -> Step:
        out = Step()
        while self._next is not None and self._next.due_s <= now_s:
            if self._next.due_s >= self._start_s:
                self._waiting.append(self._next)
            self._next = next(self._upcoming, None)
        still = []
        for occ in self._waiting:
            point = (occ.obstacle.x, occ.obstacle.y)
            if clear_of_robots(point, robot_xy_list, clearance_m):
                out.spawn.append(occ)
                self._placed[occ.number] = (occ, now_s + occ.obstacle.duration_s)
            elif now_s - occ.due_s > self._max_delay_s:
                out.give_up.append(occ)
            else:
                still.append(occ)
        self._waiting = still
        out.waiting = list(still)
        for number, (occ, remove_at) in list(self._placed.items()):
            if remove_at <= now_s:
                out.remove.append(occ)
                del self._placed[number]
        return out

    def placed(self) -> list[Occurrence]:
        return [occ for occ, _ in self._placed.values()]


def _g(value: float) -> str:
    return f"{value:.6g}"


def box_sdf(name: str, size: tuple[float, float, float]) -> str:
    """정적 상자 모델. 밑면이 모델 원점(바닥)에 닿는다. 큰따옴표는 쓰지 않는다."""
    dims = " ".join(_g(v) for v in size)
    geometry = f"<geometry><box><size>{dims}</size></box></geometry>"
    colour = "0.9 0.4 0.1 1"
    return (f"<?xml version='1.0'?><sdf version='1.9'><model name='{name}'>"
            f"<static>true</static><link name='link'>"
            f"<pose>0 0 {_g(size[2] / 2)} 0 0 0</pose>"
            f"<collision name='collision'>{geometry}</collision>"
            f"<visual name='visual'>{geometry}<material><ambient>{colour}</ambient>"
            f"<diffuse>{colour}</diffuse></material></visual>"
            f"</link></model></sdf>")


def _quote(text: str) -> str:
    return '"' + text.replace("\\", "\\\\").replace('"', '\\"') + '"'


def spawn_argv(world: str, name: str, obstacle: Obstacle, size: tuple[float, float, float],
               timeout_ms: int = GZ_TIMEOUT_MS) -> list[str]:
    """/world/<world>/create(EntityFactory) 호출. 답은 gz.msgs.Boolean."""
    half = obstacle.yaw / 2
    req = (f"sdf: {_quote(box_sdf(name, size))} "
           f"pose: {{position: {{x: {_g(obstacle.x)} y: {_g(obstacle.y)} z: 0}} "
           f"orientation: {{x: 0 y: 0 z: {_g(math.sin(half))} w: {_g(math.cos(half))}}}}} "
           f"name: {_quote(name)}")
    return ["gz", "service", "-s", f"/world/{world}/create",
            "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
            "--timeout", str(timeout_ms), "--req", req]


def remove_argv(world: str, name: str, timeout_ms: int = GZ_TIMEOUT_MS) -> list[str]:
    """/world/<world>/remove(Entity) 호출. 답은 gz.msgs.Boolean."""
    return ["gz", "service", "-s", f"/world/{world}/remove",
            "--reqtype", "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean",
            "--timeout", str(timeout_ms), "--req", f"name: {_quote(name)} type: MODEL"]


def reply_ok(stdout: str) -> bool:
    return "data: true" in stdout
